#!/usr/bin/env python3
"""
NavOA external keyboard control
Publishes to:
    /external_cmd_vel
    /external_control_enable
"""

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class NavOAKeyboardControl:
    def __init__(self):
        rospy.init_node('navoa_keyboard_control', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/external_cmd_vel', Twist, queue_size=10)
        self.enable_pub = rospy.Publisher('/external_control_enable', Bool, queue_size=10)
        
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_controls()
    
    def print_controls(self):
        print("\n" + "="*50)
        print("NavOA External Keyboard Control")
        print("="*50)
        print("Movement Control:")
        print("  w   : Forward")
        print("  s   : Stop")
        print("  a   : Turn Left")
        print("  d   : Turn Right")
        print("  x   : Backward")
        print("\nExternal Control Switch:")
        print("  t   : Enable External Control")
        print("  f   : Disable External Control")
        print("\nSpeed Adjustment:")
        print("  q   : Increase Linear Speed")
        print("  e   : Decrease Linear Speed")
        print("  z   : Increase Angular Speed")
        print("  c   : Decrease Angular Speed")
        print("\nQuit: ESC or Ctrl+C")
        print("="*50)
        print(f"Current: Linear={self.linear_speed:.2f}m/s, Angular={self.angular_speed:.2f}rad/s")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    print("\nQuit program")
                    break
                elif key == '\x03':  # Ctrl+C
                    print("\nCtrl+C to quit")
                    break
                
                elif key == 'q':
                    self.linear_speed += 0.1
                    print(f"\nLinear Speed: {self.linear_speed:.2f} m/s")
                elif key == 'e':
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
                    print(f"\nLinear Speed: {self.linear_speed:.2f} m/s")
                elif key == 'z':
                    self.angular_speed += 0.1
                    print(f"\nAngular Speed: {self.angular_speed:.2f} rad/s")
                elif key == 'c':
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    print(f"\nAngular Speed: {self.angular_speed:.2f} rad/s")
                
                elif key == 't':
                    enable_msg = Bool()
                    enable_msg.data = True
                    self.enable_pub.publish(enable_msg)
                    print("\n[External Control Enabled]")
                elif key == 'f':
                    enable_msg = Bool()
                    enable_msg.data = False
                    self.enable_pub.publish(enable_msg)
                    print("\n[External Control Disabled]")
                
                twist = Twist()
                if key == 'w':
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                    print(f"\nForward: v={self.linear_speed:.2f}")
                elif key == 'x':
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                    print(f"\nBackward: v={-self.linear_speed:.2f}")
                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                    print(f"\nTurn Left: w={self.angular_speed:.2f}")
                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                    print(f"\nTurn Right: w={-self.angular_speed:.2f}")
                elif key == 's' or key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print("\nStop")
                
                if key in ['w', 'x', 'a', 'd', 's', ' ']:
                    self.cmd_pub.publish(twist)
                
        except Exception as e:
            rospy.logerr(f"Keyboard control error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        
        disable_msg = Bool()
        disable_msg.data = False
        self.enable_pub.publish(disable_msg)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("\nNavOA Keyboard Control Stopped")

if __name__ == "__main__":
    try:
        controller = NavOAKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass