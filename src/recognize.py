#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import PointStamped

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                              dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    return image_opencv


class YOLOImageDetector:
    def __init__(self):
        #设置成员变量
        self.bridge = CvBridge()
        self.point_pub = rospy.Publisher("/camera_optical_point", PointStamped, queue_size=10)
        self.pixel_point_pub = rospy.Publisher("/pixel_point", PointStamped, queue_size=10)
        self.light_id_pub = rospy.Publisher("/pass", Int8, queue_size=10)
        # 加载 bestW 模型
        self.model = None
        try:
            rospy.loginfo("Model loading .....")
            self.model = YOLO("/home/wheeltec/my_ws/src/image_recognize/src/bestW.pt")
            rospy.loginfo("Model loaded successfully.")
            # 只有模型加载成功后才开启订阅
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")

    def image_callback(self, data):
        try:
            # 将 ROS Image 转换为 OpenCV 图像
            cv_image = imgmsg_to_cv2(data)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # 使用 YOLO 做推理, 禁用输出
        results = self.model(cv_image,verbose=False)

        # 相机的内参矩阵
        k = np.array([[400,0,320],[0,400,240],[0,0,1]])
        if results[0]:
            # 计算像素平面坐标的位置
            xyxy = results[0].boxes.xyxy[0].tolist()
            x_center = (xyxy[0] + xyxy[2]) / 2
            y_center = (xyxy[1] + xyxy[3]) / 2
            # 发布像素坐标
            pixel = PointStamped()
            pixel.point.x = x_center
            pixel.point.y = y_center
            pixel.point.z = 0
            pixel.header.stamp = rospy.Time.now()
            pixel.header.frame_id = "pixel_frame"
            self.pixel_point_pub.publish(pixel)
            pixel_point = np.array([x_center,y_center,1])
            # 计算归一化相机坐标系的坐标点
            k_inv = np.linalg.inv(k)
            temp_point = k_inv.dot(pixel_point)
            # 发布归一化坐标点
            optical_point = PointStamped()
            # 设置时间戳
            optical_point.header.stamp = rospy.Time.now()
            # 设置坐标系 ID
            optical_point.header.frame_id = "camera_optical_frame"
            # 设置点的位置
            optical_point.point.x = temp_point[0]
            optical_point.point.y = temp_point[1]
            optical_point.point.z = temp_point[2]
            self.point_pub.publish(optical_point)
            # 根据红绿灯颜色来判断是否让小车通过
            pass_through = Int8()
            if int(results[0].boxes.cls.item()):
                pass_through = 0
            else:
                pass_through = 1     
            self.light_id_pub.publish(pass_through)
            

        # 绘制结果
        annotated_frame = results[0].plot()

        # 显示图像
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    # anonymous表示是否允许同样名字的节点存在，会加后缀
    rospy.init_node('yolo_image_detector', anonymous=False)
    detector = YOLOImageDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    finally:
        cv2.destroyAllWindows()