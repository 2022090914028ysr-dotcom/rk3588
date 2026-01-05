#!/usr/bin/env python3
# -*-coding: utf-8 -*-
"""
    @Project: python-learning-notes
    @File   : openpose_for_image_test_ros.py
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2019-07-29 21:50:17
"""
import time
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

######################### Detection ##########################
# load the COCO class names
with open('/home/yahboom/ascam_ws/src/ascam_visual/detection/object_detection_coco.txt', 'r') as f: class_names = f.read().split('\n')
# get a different color array for each of the classes
COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))
# load the DNN model
model = cv.dnn.readNet(model='/home/yahboom/ascam_ws/src/ascam_visual/detection/frozen_inference_graph.pb', config='/home/yahboom/ascam_ws/src/ascam_visual/detection/ssd_mobilenet_v2_coco.txt', framework='TensorFlow')

######################### openpose ##########################
BODY_PARTS = {"Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
              "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
              "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
              "LEye": 15, "REar": 16, "LEar": 17, "Background": 18}
POSE_PAIRS = [["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
              ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
              ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
              ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
              ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]]
net = cv.dnn.readNetFromTensorflow("/home/yahboom/ascam_ws/src/ascam_visual/detection/graph_opt.pb")


def Target_Detection(image):
    image_height, image_width, _ = image.shape
    # create blob from image
    blob = cv.dnn.blobFromImage(image=image, size=(300, 300), mean=(104, 117, 123), swapRB=True)
    model.setInput(blob)
    output = model.forward()
    # loop over each of the detections
    for detection in output[0, 0, :, :]:
        # extract the confidence of the detection
        confidence = detection[2]
        # draw bounding boxes only if the detection confidence is above...
        if confidence > .4:
            # get the class id
            class_id = detection[1]
            class_name = class_names[int(class_id) - 1]
            color = COLORS[int(class_id)]
            # get the bounding box coordinates
            box_x = detection[3] * image_width
            box_y = detection[4] * image_height
            box_width = detection[5] * image_width
            box_height = detection[6] * image_height
            # draw a rectangle around each detected object
            cv.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), color, thickness=2)
            cv.putText(image, class_name, (int(box_x), int(box_y - 5)), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    return image


def openpose(frame):
    frameHeight, frameWidth = frame.shape[:2]
    net.setInput(cv.dnn.blobFromImage(frame, 1.0, (368, 368), (127.5, 127.5, 127.5), swapRB=True, crop=False))
    out = net.forward()
    out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements
    assert (len(BODY_PARTS) == out.shape[1])
    points = []
    for i in range(len(BODY_PARTS)):
        # Slice heatmap of corresponding body's part.
        heatMap = out[0, i, :, :]
        _, conf, _, point = cv.minMaxLoc(heatMap)
        x = (frameWidth * point[0]) / out.shape[3]
        y = (frameHeight * point[1]) / out.shape[2]
        points.append((int(x), int(y)) if conf > 0.2 else None)
    for pair in POSE_PAIRS:
        partFrom = pair[0]
        partTo = pair[1]
        assert (partFrom in BODY_PARTS)
        assert (partTo in BODY_PARTS)
        idFrom = BODY_PARTS[partFrom]
        idTo = BODY_PARTS[partTo]
        if points[idFrom] and points[idTo]:
            cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
            cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
            cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
    return frame


class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.callback)
        self.frame = None

    def callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def get_frame(self):
        return self.frame


def main():
    rospy.init_node('openpose_ros', anonymous=True)
    image_subscriber = ImageSubscriber()

    state = True
    rate = rospy.Rate(30)  # 30 Hz

    while not rospy.is_shutdown():
        frame = image_subscriber.get_frame()
        if frame is not None:
            start = time.time()

            if state:
                frame = Target_Detection(frame)
                cv.putText(frame, "Detection", (240, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            else:
                frame = openpose(frame)
                cv.putText(frame, "Openpose", (240, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (100, 200, 200), 1)

            cv.imshow('frame', frame)

            action = cv.waitKey(10) & 0xFF
            if action == ord('q') or action == ord('Q'):
                break
            if action == ord('f') or action == ord('F'):
                state = not state

            rate.sleep()

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

