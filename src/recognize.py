#!/usr/bin/env python3

import rospy
import tf2_ros

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # 用于 do_transform_point

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO


class YOLOImageDetector:
    def __init__(self):
        #设置成员变量
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.point_pub = rospy.Publisher("/camera_optical_point", PointStamped, queue_size=10)
         # 创建tf_buffer，所有的坐标变化找buffer要
        self.tf_buffer = tf2_ros.Buffer()
        # api内部已经实现了订阅
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # 加载 bestW 模型
        self.model = YOLO("/home/kook/my_ws/src/image_recognize/src/bestW.pt")  

    def image_callback(self, data):
        try:
            # 将 ROS Image 转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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