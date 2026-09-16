// C++ port of scripts/multi_ultrasonic_scan.py
// Node name: multi_ultrasonic_scan (not anonymous)
//
// Multi DYP-A21 ultrasonic sensors -> /voscan (sensor_msgs/LaserScan, frame base_link)
// plus per-sensor raw distances on /ultrasonic/{front,left,right} (Float32, meters).
//
// Protocol: FF Data_H Data_L SUM, SUM = (FF + Data_H + Data_L) & 0xFF
//   0xFFFE -> same-frequency interference (invalid)
//   0xFFFD -> no object detected        (invalid)
//   valid  -> distance in mm = Data_H * 256 + Data_L
//
// Serial I/O uses boost::asio::serial_port (the ros-serial package is not
// installed on this system).  One reader thread per sensor; the ROS timer
// assembles LaserScan frames at the configured rate.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <boost/asio.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace asio = boost::asio;

// ---------------------------------------------------------------------------
class A21Sensor
{
public:
  A21Sensor(const std::string& name, const std::string& port, uint32_t baud)
    : name_(name), port_(port), io_(), serial_(io_), timer_(io_),
      mm_(0.0), has_mm_(false), running_(false), ok_(false)
  {
    try
    {
      serial_.open(port_);
      serial_.set_option(asio::serial_port_base::baud_rate(baud));
      serial_.set_option(asio::serial_port_base::character_size(8));
      serial_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
      serial_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
      serial_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
      ok_ = true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to open %s: %s", name_.c_str(), port_.c_str(), e.what());
    }
  }

  ~A21Sensor()
  {
    stop();
  }

  // ---- thread loop --------------------------------------------------------
  void run()
  {
    if (!ok_)
      return;
    running_ = true;
    while (running_ && ros::ok())
    {
      try
      {
        uint8_t first = 0;
        if (!read_bytes(&first, 1))
          continue;
        if (first != 0xFF)
          continue;
        uint8_t buf[3];
        if (!read_bytes(buf, 3))
          continue;
        uint8_t data_h = buf[0], data_l = buf[1], recv_sum = buf[2];
        uint8_t calc_sum = static_cast<uint8_t>((0xFF + data_h + data_l) & 0xFF);
        if (calc_sum != recv_sum)
          continue;
        uint16_t value = static_cast<uint16_t>((data_h << 8) | data_l);
        if (value == 0xFFFE || value == 0xFFFD)
          continue;  // invalid readings
        {
          std::lock_guard<std::mutex> lk(mutex_);
          mm_ = static_cast<double>(value);
          has_mm_ = true;
        }
      }
      catch (...)
      {
        // transient serial glitch, retry next iteration
      }
    }
  }

  void stop()
  {
    running_ = false;
    try
    {
      if (serial_.is_open())
      {
        boost::system::error_code ec;
        serial_.cancel(ec);
        serial_.close(ec);
      }
    }
    catch (...)
    {
    }
  }

  // Return latest distance in meters; false if no valid reading yet.
  bool get_m(double& out)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_mm_)
      return false;
    out = mm_ / 1000.0;
    return true;
  }

private:
  // Blocking read of up to n bytes with a 50 ms timeout (matches the Python
  // serial timeout=0.05).  Returns number of bytes actually received (0 = timeout).
  size_t read_some_timeout(uint8_t* dst, size_t n)
  {
    boost::system::error_code ec;
    size_t got = 0;
    std::atomic<bool> done{ false };

    timer_.expires_from_now(std::chrono::milliseconds(50));
    timer_.async_wait([&](const boost::system::error_code& e) {
      if (!e)
        serial_.cancel();
    });

    serial_.async_read_some(asio::buffer(dst, n),
                            [&](const boost::system::error_code& e, size_t s) {
                              if (!e)
                                got = s;
                              done = true;
                            });

    io_.restart();
    while (io_.run_one() > 0 && !done.load())
    {
    }
    try
    {
      timer_.cancel();
    }
    catch (...)
    {
    }
    return got;
  }

  bool read_bytes(uint8_t* dst, size_t n)
  {
    size_t got = 0;
    while (got < n)
    {
      size_t r = read_some_timeout(dst + got, n - got);
      if (r == 0)
        return false;
      got += r;
    }
    return true;
  }

  std::string name_, port_;
  asio::io_context io_;
  asio::serial_port serial_;
  asio::steady_timer timer_;
  std::mutex mutex_;
  double mm_;
  bool has_mm_;
  std::atomic<bool> running_;
  bool ok_;
};

// ---------------------------------------------------------------------------
class MultiUltrasonicScan
{
public:
  MultiUltrasonicScan() : nh_(), pnh_("~")
  {
    std::string port_front = pnh_.param<std::string>("port_front", "/dev/ttyS6");
    std::string port_left = pnh_.param<std::string>("port_left", "/dev/ttyS3");
    std::string port_right = pnh_.param<std::string>("port_right", "/dev/ttyS7");
    double angle_front = pnh_.param("angle_front", 0.0);
    double angle_left = pnh_.param("angle_left", -80.0);
    double angle_right = pnh_.param("angle_right", 80.0);
    double beam_width = pnh_.param("beam_width", 25.0);
    double scan_fov = pnh_.param("scan_fov", 180.0);
    double angular_res = pnh_.param("angular_res", 1.0);
    int baud = pnh_.param("baud", 115200);
    rate_ = pnh_.param("rate", 20.0);
    range_min_ = pnh_.param("range_min", 0.02);
    range_max_ = pnh_.param("range_max", 6.0);
    frame_id_ = pnh_.param<std::string>("frame_id", "base_link");

    const std::vector<std::string> port_names = { "front", "left", "right" };
    const std::vector<std::string> ports = { port_front, port_left, port_right };
    const std::vector<double> angles_deg = { angle_front, angle_left, angle_right };

    for (size_t i = 0; i < port_names.size(); ++i)
    {
      sensors_.emplace_back(new A21Sensor(port_names[i], ports[i], static_cast<uint32_t>(baud)));
      angles_rad_.push_back(angles_deg[i] * M_PI / 180.0);
      threads_.emplace_back([this, i]() { sensors_[i]->run(); });
    }

    beam_half_rad_ = (beam_width / 2.0) * M_PI / 180.0;

    angle_min_ = (-scan_fov / 2.0) * M_PI / 180.0;
    angle_max_ = (+scan_fov / 2.0) * M_PI / 180.0;
    angle_increment_ = angular_res * M_PI / 180.0;
    num_beams_ = static_cast<int>(std::round(
                      (angle_max_ - angle_min_) / angle_increment_)) + 1;

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("/voscan", 10);
    raw_pub_front_ = nh_.advertise<std_msgs::Float32>("/ultrasonic/front", 5);
    raw_pub_left_ = nh_.advertise<std_msgs::Float32>("/ultrasonic/left", 5);
    raw_pub_right_ = nh_.advertise<std_msgs::Float32>("/ultrasonic/right", 5);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(rate_, 0.1)),
                             &MultiUltrasonicScan::publish_scan, this);
  }

  ~MultiUltrasonicScan()
  {
    for (auto& s : sensors_)
      s->stop();
    for (auto& t : threads_)
      if (t.joinable())
        t.join();
  }

private:
  static double angle_diff_abs(double a, double b)
  {
    double d = std::fabs(a - b);
    if (d > M_PI)
      d = 2.0 * M_PI - d;
    return d;
  }

  bool in_beam(double beam_angle, double sensor_angle)
  {
    return angle_diff_abs(beam_angle, sensor_angle) <= beam_half_rad_;
  }

  void publish_scan(const ros::TimerEvent&)
  {
    sensor_msgs::LaserScan msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.angle_min = angle_min_;
    msg.angle_max = angle_max_;
    msg.angle_increment = angle_increment_;
    msg.range_min = range_min_;
    msg.range_max = range_max_;

    msg.ranges.resize(num_beams_);
    msg.intensities.resize(num_beams_);

    for (int i = 0; i < num_beams_; ++i)
    {
      double beam_angle = angle_min_ + i * angle_increment_;
      double best_m = std::numeric_limits<double>::infinity();
      bool has_best = false;

      for (size_t s = 0; s < sensors_.size(); ++s)
      {
        if (!in_beam(beam_angle, angles_rad_[s]))
          continue;
        double d_m = 0.0;
        if (sensors_[s]->get_m(d_m))
        {
          if (!has_best || d_m < best_m)
          {
            best_m = d_m;
            has_best = true;
          }
        }
      }

      if (!has_best)
        msg.ranges[i] = std::numeric_limits<float>::infinity();
      else if (best_m < range_min_)
        msg.ranges[i] = std::numeric_limits<float>::infinity();
      else if (best_m > range_max_)
        msg.ranges[i] = range_max_;
      else
        msg.ranges[i] = static_cast<float>(best_m);

      msg.intensities[i] = 0.0f;
    }

    pub_.publish(msg);

    // per-sensor raw distances for the virtual bumper (meters)
    for (size_t s = 0; s < sensors_.size(); ++s)
    {
      double d_m = 0.0;
      if (!sensors_[s]->get_m(d_m))
        continue;
      std_msgs::Float32 raw;
      raw.data = static_cast<float>(d_m);
      if (s == 0)
        raw_pub_front_.publish(raw);
      else if (s == 1)
        raw_pub_left_.publish(raw);
      else
        raw_pub_right_.publish(raw);
    }
  }

  ros::NodeHandle nh_, pnh_;
  std::vector<boost::shared_ptr<A21Sensor> > sensors_;
  std::vector<std::thread> threads_;
  std::vector<double> angles_rad_;
  double beam_half_rad_;
  double angle_min_, angle_max_, angle_increment_;
  int num_beams_;
  double range_min_, range_max_, rate_;
  std::string frame_id_;
  ros::Publisher pub_, raw_pub_front_, raw_pub_left_, raw_pub_right_;
  ros::Timer timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_ultrasonic_scan");
  MultiUltrasonicScan scanner;
  ros::spin();
  return 0;
}
