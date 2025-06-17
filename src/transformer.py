#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import tf2_geometry_msgs  # 用于 do_transform_point


class PointTransFormer:
    def __init__(self):
        #设置成员变量
        self.optical_point_sub = rospy.Subscriber("/camera_optical_point", PointStamped, self.point_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.transformed_pub = rospy.Publisher("/transformed_point_lidar", PointStamped, queue_size=10)
         # 创建tf_buffer，所有的坐标变化找buffer要
        self.tf_buffer = tf2_ros.Buffer()
        # api内部已经实现了订阅
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def point_callback(self, data):
        # 从相机光学坐标系变换到base_link下，只有y,z变化
        try:
            transform = self.tf_buffer.lookup_transform("laser","camera_depth_optical_frame",rospy.Time(0))
            point_lidar = tf2_geometry_msgs.do_transform_point(data, transform)
            self.transformed_pub.publish(point_lidar)
            angle = np.arctan2(point_lidar.point.y,point_lidar.point.x)
            # compensate
            # a = int((angle + 0.760577)/self.lidar_data.angle_increment)
            # if a==335:
            #     print(1.6699999570846558)
            # else:
            #     print(self.lidar_data.ranges[a])
            a = int((angle - self.lidar_data.angle_min)/self.lidar_data.angle_increment)
            print(self.lidar_data.ranges[a])
        except tf2_ros.LookupException as e:
            rospy.logerr(f"lookup transform error: {e}")
            
    def lidar_callback(self,data):
        self.lidar_data = data

if __name__ == '__main__':
    # anonymous表示是否允许同样名字的节点存在，会加后缀
    rospy.init_node('point_transformer', anonymous=False) 
    transformer = PointTransFormer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")