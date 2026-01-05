#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class VideoRecorder:
    def __init__(self, topic, output_filename, fps=30.0, resolution=(640, 480)):
        self.topic = topic
        self.output_filename = output_filename
        self.fps = fps
        self.resolution = resolution
        self.bridge = CvBridge()
        
        # 创建VideoWriter对象
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用MP4编码器
        self.out = cv2.VideoWriter(self.output_filename, fourcc, self.fps, self.resolution)
        
        # 创建ROS节点和订阅者
        rospy.init_node('video_recorder', anonymous=True)
        self.subscriber = rospy.Subscriber(self.topic, Image, self.image_callback)
        
        # 用于控制录制的标志
        self.recording = True
    
    def image_callback(self, data):
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        # 写入视频文件
        self.out.write(cv_image)
        
        # 显示图像（可选）
        cv2.imshow('Recording...', cv_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.stop_recording()
    
    def stop_recording(self):
        self.recording = False
        self.out.release()
        cv2.destroyAllWindows()
        rospy.signal_shutdown('Recording stopped by user.')

if __name__ == '__main__':
    try:
        # 配置录制参数
        topic = '/ascamera_hp60c/rgb0/image'
        output_filename = 'ascam.mp4'
        fps = 30.0
        resolution = (640, 480)
        
        # 创建VideoRecorder实例并开始录制
        recorder = VideoRecorder(topic, output_filename, fps, resolution)
        
        # 保持主循环
        while not rospy.is_shutdown() and recorder.recording:
            rospy.sleep(0.1)
    
    except rospy.ROSInterruptException:
        pass

