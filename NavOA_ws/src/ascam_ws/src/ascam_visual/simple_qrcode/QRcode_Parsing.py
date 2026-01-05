#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageDraw, ImageFont


def decodeDisplay(image, font_path):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # 需要先把输出的中文字符转换成Unicode编码形式
    # The output Chinese characters need to be converted to Unicode encoding first
    barcodes = pyzbar.decode(gray)
    for barcode in barcodes:
        # 提取二维码的边界框的位置
        # Extract the position of the boundary box of the TWO-DIMENSIONAL code
        # 画出图像中条形码的边界框
        # Draw the bounding box for the bar code in the image
        (x, y, w, h) = barcode.rect
        cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
        encoding = 'UTF-8'
        # 画出来，就需要先将它转换成字符串
        # to draw it, you need to convert it to a string
        barcodeData = barcode.data.decode(encoding)
        barcodeType = barcode.type
        # 绘出图像上数据和类型
        # Draw the data and type on the image
        pilimg = PILImage.fromarray(image)
        # 创建画笔
        # create brush
        draw = ImageDraw.Draw(pilimg)  # 图片上打印  Print on picture
        # 参数1：字体文件路径，参数2：字体大小
        # parameter 1: font file path, parameter 2: font size
        fontStyle = ImageFont.truetype(font_path, size=12, encoding=encoding)
        # # 参数1：打印坐标，参数2：文本，参数3：字体颜色，参数4：字体
        # Parameter 1: print coordinates, parameter 2: text, parameter 3: font color, parameter 4: font
        draw.text((x, y - 25), str(barcode.data, encoding), fill=(255, 0, 0), font=fontStyle)
        # # PIL图片转cv2 图片
        # PIL picture to CV2 picture
        image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
        # 向终端打印条形码数据和条形码类型
        # Print barcode data and barcode type to terminal
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    return image


class BarcodeDecoder:
    def __init__(self):
        # 初始化节点
        rospy.init_node('barcode_decoder', anonymous=True)
        self.bridge = CvBridge()  # OpenCV与ROS消息的桥接
        self.image_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.image_callback)
        self.font_path = "/home/yahboom/ascam_ws/src/ascam_visual/font/Block_Simplified.TTF"

    def image_callback(self, msg):
        try:
            # 从ROS消息中转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 处理图像并显示
            frame = decodeDisplay(cv_image, self.font_path)

            # 显示图像
            cv.imshow("Barcode Decoder", frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

        except CvBridgeError as e:
            print(e)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        barcode_decoder = BarcodeDecoder()
        barcode_decoder.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv.destroyAllWindows()

