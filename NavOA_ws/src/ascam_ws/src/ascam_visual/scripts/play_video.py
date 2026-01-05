#!/usr/bin/env python
# coding:utf-8

import numpy as np
import cv2

# 打开视频文件
cap = cv2.VideoCapture('/home/yahboom/ascam.mp4')

while cap.isOpened():
    # 读取一帧
    ret, frame = cap.read()

    # 如果读取成功，显示帧
    if ret:
        cv2.imshow('frame', frame)
    else:
        # 如果读取失败，可能是视频结束了
        print("视频播放结束")
        
        # 创建一个空白帧来显示提示信息
        if frame is None:
            frame = np.zeros((int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), 3), dtype=np.uint8)
        
        # 显示提示信息
        cv2.putText(frame, "Video Ended", (50, int(0.75 * frame.shape[0])), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 64, 255), 2, cv2.LINE_AA)
        cv2.imshow('frame', frame)
        cv2.waitKey(3000)  # 显示提示信息3秒
        break

    # 检查按键事件，按下 'q' 键退出
    if cv2.waitKey(40) & 0xFF == ord('q'):
        break

# 释放视频捕获对象
cap.release()
# 关闭所有OpenCV窗口
cv2.destroyAllWindows()

