
"""
DYP-A21 超声波传感器 UART → ROS（毫米 mm）

协议：
    FF Data_H Data_L SUM
    SUM = (FF + Data_H + Data_L) & 0xFF

特殊值：
    0xFFFE 同频干扰
    0xFFFD 未检测到物体

ROS：
    Topic: /ultrasonic/distance_mm
    Type : std_msgs/Int32
"""

import argparse
import time
import serial
import rospy
from std_msgs.msg import Int32

HEADER = 0xFF


class A21UARTReader:
    def __init__(self,
                 port="/dev/ttyS6",
                 baudrate=115200,
                 timeout=1.0,
                 trigger=False,
                 period=0.2):

        # ---------- UART ----------
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )

        self.trigger = trigger
        self.period = period

        # ---------- ROS ----------
        rospy.init_node("a21_ultrasonic_mm", anonymous=False)

        self.pub = rospy.Publisher(
            "/ultrasonic/distance_mm",
            Int32,
            queue_size=10
        )

        rospy.loginfo(f"A21 ultrasonic started on {port}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ------------------ 协议解析 ------------------ #

    def _read_frame(self):
        """
        帧格式：FF Data_H Data_L SUM
        返回：int (毫米)，或 None
        """
        while not rospy.is_shutdown():
            first = self.ser.read(1)
            if not first:
                return None

            if first[0] != HEADER:
                continue

            rest = self.ser.read(3)
            if len(rest) != 3:
                return None

            data_h, data_l, recv_sum = rest
            calc_sum = (HEADER + data_h + data_l) & 0xFF

            if calc_sum != recv_sum:
                rospy.logwarn(
                    f"Checksum error: FF {data_h:02X} {data_l:02X} {recv_sum:02X}"
                )
                continue

            value = (data_h << 8) | data_l
            return value

    def _is_valid_distance(self, value):
        if value == 0xFFFE:
            rospy.logwarn("Ultrasonic interference detected (0xFFFE)")
            return False
        if value == 0xFFFD:
            rospy.logwarn("No object detected (0xFFFD)")
            return False
        return True

    # ------------------ 主循环 ------------------ #

    def loop(self):
        rospy.loginfo("Ultrasonic ROS publisher (mm) running")
        rate = rospy.Rate(1.0 / self.period)

        while not rospy.is_shutdown():

        
            if self.trigger:
                self.ser.write(b'\x55') 

            raw_value = self._read_frame()

            if raw_value is not None and self._is_valid_distance(raw_value):
                msg = Int32()
                msg.data = raw_value      # 毫米
                self.pub.publish(msg)

                #rospy.loginfo(f"Distance: {raw_value} mm")

            rate.sleep()

        self.close()


def main():
    parser = argparse.ArgumentParser(
        description="DYP-A21 UART → ROS (mm)"
    )

    parser.add_argument("--port",
                        default="/dev/ttyS6",
                        help="串口号，例如 /dev/ttyS6")
    parser.add_argument("--baud",
                        type=int,
                        default=115200,
                        help="波特率，默认 115200")
    parser.add_argument("--trigger",
                        default=False,
                        action="store_true",
                        help="启用 UART 触发模式")
    parser.add_argument("--period",
                        type=float,
                        default=0.15,
                        help="触发周期（秒，>=0.15）")

    args = parser.parse_args()

    reader = A21UARTReader(
        port=args.port,
        baudrate=args.baud,
        trigger=args.trigger,
        period=args.period
    )
    reader.loop()


if __name__ == "__main__":
    main()
