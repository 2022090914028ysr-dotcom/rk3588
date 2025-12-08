/*
 * @Version      : V1.0
 * @Date         : 2024-10-12 17:35:37
 * @Description  : serial api unix 
 */
#include "interface/serial/interface_serial.hpp"
#include <unistd.h>
#include <chrono>
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <thread>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>
#include <err.h>
#include <linux/serial.h>
#include <cstring>
#include <sys/ioctl.h> //ioctl

namespace nvistar{
class InterfaceSerialImpl{
public:
  //var 
  int  _fd = -1;                    //File descriptor for the port
  bool open_flag = false;          //serial open flag
  //serial var 
  std::string port_name = "/dev/ttyUSB";
  int baud_rate = 230400;
  InterfaceSerial::serial_parity_t   parity = InterfaceSerial::ParityNone;
  InterfaceSerial::serial_databits_t databits = InterfaceSerial::DataBits8;
  InterfaceSerial::serial_stopbits_t stopbits = InterfaceSerial::StopOne;
  InterfaceSerial::serial_flowcontrol_t flowcontrol = InterfaceSerial::FlowNone;

  // linux/include/uapi/asm-generic/termbits.h
  struct termios2 {
      tcflag_t c_iflag;       /* input mode flags */
      tcflag_t c_oflag;       /* output mode flags */
      tcflag_t c_cflag;       /* control mode flags */
      tcflag_t c_lflag;       /* local mode flags */
      cc_t c_line;            /* line discipline */
      cc_t c_cc[19];          /* control characters */
      speed_t c_ispeed;       /* input speed */
      speed_t c_ospeed;       /* output speed */
  };
  #ifndef TCGETS2
  #define TCGETS2     _IOR('T', 0x2A, struct termios2)
  #endif

  #ifndef TCSETS2
  #define TCSETS2     _IOW('T', 0x2B, struct termios2)
  #endif

  #ifndef BOTHER
  #define BOTHER      0010000
  #endif

  //function 

  /**
  * @Function: serial_set_databits
  * @Description: set databits 
  * @Return: bool
  * @param {termios} *tio
  * @param {serial_databits_t} databits
  */
  bool serial_set_databits(struct termios *tio,InterfaceSerial::serial_databits_t databits){
    tio->c_cflag &= (tcflag_t)~CSIZE;
    //set databits 
    switch (databits) {
      case InterfaceSerial::DataBits5:{
        tio->c_cflag |= CS5;
        break;
      }
      case InterfaceSerial::DataBits6:{
        tio->c_cflag |= CS6;
        break;
      }
      case InterfaceSerial::DataBits7:{
        tio->c_cflag |= CS7;
        break;
      }
      case InterfaceSerial::DataBits8:{
        tio->c_cflag |= CS8;
        break;
      }
      default:{
        tio->c_cflag |= CS8;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_parity
  * @Description: set parity 
  * @Return: bool 
  * @param {termios} *tio
  * @param {serial_parity_t} parity
  */
  bool serial_set_parity(struct termios *tio,InterfaceSerial::serial_parity_t parity){
    tio->c_iflag &= ~(PARMRK | INPCK);
    tio->c_iflag |= IGNPAR;

    switch (parity) {
    #ifdef CMSPAR
      // Here Installation parity only for GNU/Linux where the macro CMSPAR.
      case InterfaceSerial::ParitySpace:{
        tio->c_cflag |=  (PARENB | CMSPAR);
        tio->c_cflag &= ~(PARODD);
        break;
      }
      case InterfaceSerial::ParityMark:{
        tio->c_cflag |= (PARENB | CMSPAR | PARODD);
        break;
      }
    #endif
      case InterfaceSerial::ParityNone:{
        tio->c_cflag &= ~PARENB;
        break;
      }
      case InterfaceSerial::ParityEven:{
        tio->c_cflag &= ~(PARODD);
        tio->c_cflag |= PARENB;
        break;
      }
      case InterfaceSerial::ParityOdd:{
        tio->c_cflag |= (PARENB | PARODD);
        break;
      }
      default:{
        tio->c_cflag &= ~PARENB;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_stopbits
  * @Description: set stopbit 
  * @Return: bool 
  * @param {termios} *tio
  * @param {serial_stopbits_t} stop_bits
  */
  bool serial_set_stopbits(struct termios *tio, InterfaceSerial::serial_stopbits_t stopbits){
    switch (stopbits) {
      case InterfaceSerial::StopOne:{
        tio->c_cflag &= ~CSTOPB;
        break;
      }
      case InterfaceSerial::StopTwo:{
        tio->c_cflag |= CSTOPB;
        break;
      }
      default:{
        tio->c_cflag &= ~CSTOPB;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_flowcontrol
  * @Description: set flowcontrol 
  * @Return: bool 
  * @param {termios} *tio
  * @param {serial_flowcontrol_t} flowcontrol
  */
  bool serial_set_flowcontrol(struct termios *tio, InterfaceSerial::serial_flowcontrol_t flowcontrol){
    switch (flowcontrol) {
      case InterfaceSerial::FlowNone:{
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
      }
      case InterfaceSerial::FlowHardware:{
        tio->c_cflag |= CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
      }
      case InterfaceSerial::FlowSoftware:{
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag |= (IXON | IXOFF | IXANY);
        break;
      }
      default:{
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
      }
    } 
    return true;
  }
  /**
  * @Function: serial_baudrate_to_unix
  * @Description: baudrate to unix 
  * @Return: int
  */
  int serial_baudrate_to_unix(int baudrate){
   #define B(x) \
    case x:  \
        return B##x

    switch (baudrate)
    {
    #ifdef B50
      B(50);
    #endif
    #ifdef B75
      B(75);
    #endif
    #ifdef B110
      B(110);
    #endif
    #ifdef B134
      B(134);
    #endif
    #ifdef B150
      B(150);
    #endif
    #ifdef B200
      B(200);
    #endif
    #ifdef B300
      B(300);
    #endif
    #ifdef B600
      B(600);
    #endif
    #ifdef B1200
      B(1200);
    #endif
    #ifdef B1800
      B(1800);
    #endif
    #ifdef B2400
      B(2400);
    #endif
    #ifdef B4800
      B(4800);
    #endif
    #ifdef B9600
      B(9600);
    #endif
    #ifdef B19200
      B(19200);
    #endif
    #ifdef B38400
      B(38400);
    #endif
    #ifdef B57600
      B(57600);
    #endif
    #ifdef B115200
      B(115200);
    #endif
    #ifdef B230400
      B(230400);
    #endif
    #ifdef B460800
      B(460800);
    #endif
    #ifdef B500000
      B(500000);
    #endif
    #ifdef B576000
      B(576000);
    #endif
    #ifdef B921600
      B(921600);
    #endif
    #ifdef B1000000
      B(1000000);
    #endif
    #ifdef B1152000
      B(1152000);
    #endif
    #ifdef B1500000
      B(1500000);
    #endif
    #ifdef B2000000
      B(2000000);
    #endif
    #ifdef B2500000
      B(2500000);
    #endif
    #ifdef B3000000
      B(3000000);
    #endif
    #ifdef B3500000
      B(3500000);
    #endif
    #ifdef B4000000
      B(4000000);
    #endif
      default:
        return 0;
    }

#undef B
  }
  /**
  * @Function: serial_set_baudrate
  * @Description: set serial baudrate 
  * @Return: bool 
  * @param {int} baudrate
  */
  bool serial_set_baudrate(int fd,struct termios *tio, int baudrate){
    int baud_unix;
    struct termios2 tio2;
    //standard baudrate
    baud_unix = serial_baudrate_to_unix(baudrate);
    if(0 != baud_unix){
      //set standard baudrate 
      if(cfsetispeed(tio, baud_unix) < 0){
        return false;
      }
      if(cfsetospeed(tio, baud_unix) < 0){
        return false;
      }
    }
    //custom baudrate 
    else{
      //get tio2 
      if(ioctl(fd, TCGETS2, &tio2) == -1){
        return false;
      }
      tio2.c_cflag &= ~CBAUD;
      tio2.c_cflag |= BOTHER;
      tio2.c_ispeed = baudrate;
      tio2.c_ospeed = baudrate;
      //set & get tio 2
      if(ioctl(fd, TCSETS2, &tio2)){
        return false;
      }
      if(ioctl(fd, TCGETS2, &tio2)){
        return false;
      }
    }
    return true;
  }
};

/**
 * @Function: InterfaceSerial
 * @Description: 
 * @Return: None
 */
InterfaceSerial::InterfaceSerial() : _impl(new InterfaceSerialImpl){

}

/**
 * @Function: ~InterfaceSerial
 * @Description: 
 * @Return: None
 */
InterfaceSerial::~InterfaceSerial(){
  serial_close();
  delete _impl;
}

/**
 * @Function: serial_open
 * @Description: open serial
 * @Return: bool --- success or fail
 * @param {string} port_name 
 * @param {int} baud_rate
 * @param {int} party
 * @param {int} data_bits
 * @param {int} stop_bits
 * @param {int} flow_control
 */
bool InterfaceSerial::serial_open(std::string port_name, int baud_rate,
                  serial_parity_t parity,
                  serial_databits_t databits,
                  serial_stopbits_t stopbits,
                  serial_flowcontrol_t flow_control){
  //var set value 
  _impl->port_name = port_name;
  _impl->baud_rate = baud_rate;
  _impl->parity = parity;
  _impl->databits = databits;
  _impl->stopbits = stopbits;
  _impl->flowcontrol = flow_control;
  //open the serial 
  _impl->_fd = open(port_name.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);  //no block 
  //open failed
  if(_impl->_fd == -1){
    return false;
  }
  struct termios tio;
  //termios get info
  if (tcgetattr(_impl->_fd, &tio) == -1) {
    _impl->_fd = -1;
    return false;
  }
  //set up raw mode / no echo / binary
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  //set parameter 
  _impl->serial_set_parity(&tio, parity);            //set parity 
  _impl->serial_set_databits(&tio, databits);        //set databits 
  _impl->serial_set_stopbits(&tio, stopbits);        //set stopbits 
  _impl->serial_set_flowcontrol(&tio, flow_control); //set flowcontrol
  //set baudrate
  if(false == _impl->serial_set_baudrate(_impl->_fd, &tio, baud_rate)){         //set baudrate
    _impl->_fd = -1;
    return false;
  }  
  //clear buffer
  tcflush(_impl->_fd, TCIFLUSH);
   //set flag 
  if (tcsetattr(_impl->_fd, TCSANOW, &tio) < 0) {
    _impl->_fd = -1;
    return false;
  }             
  return true;
}

/**
 * @Function: serial_close
 * @Description: close serialport
 * @Return: void
 */
void InterfaceSerial::serial_close(){
  if(!serial_isopen()){
    return;
  }
  //close the serial 
  close(_impl->_fd);
  _impl->_fd = -1;
}

/**
 * @Function: serial_reopen
 * @Description: serial reopen 
 * @Return: void 
 */
void InterfaceSerial::serial_reopen(){
  //first close the serial 
  serial_close();
  //delay some times 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //reopen the serial 
  serial_open(_impl->port_name, _impl->baud_rate,
              _impl->parity, _impl->databits,_impl->stopbits, _impl->flowcontrol);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

/**
 * @Function: serial_isopen
 * @Description: judge serial is open?
 * @Return: bool 
 */
bool InterfaceSerial::serial_isopen(){
  if(_impl->_fd != -1){
    return true;
  }
  return false;
}

/**
 * @Function: serial_read
 * @Description: serial read data 
 * @Return: int --- read true length
 * @param {uint8_t} *data
 * @param {int} max_length
 */
int InterfaceSerial::serial_read(uint8_t *data,int max_length){
  if(!serial_isopen()){
    return 0;
  }
  return read(_impl->_fd, data, max_length);
}

/**
 * @Function: serial_write
 * @Description: serial write data 
 * @Return: int --- write true length 
 * @param {uint8_t*} data
 * @param {int} length
 */
int InterfaceSerial::serial_write(const uint8_t* data,int length){
  if(!serial_isopen()){
    return 0;
  }
  return write(_impl->_fd, data, length);
}

/**
 * @Function: serial_flush
 * @Description: flush serial input and output
 * @Return: void
 */
void InterfaceSerial::serial_flush(){
  if(!serial_isopen()){
    return;
  }
  tcflush(_impl->_fd, TCIOFLUSH);
}
}
