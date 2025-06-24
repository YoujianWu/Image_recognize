#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan

def quaternion_matrix(q):
    x, y, z, w = q
    xx = x * x
    xy = x * y
    xz = x * z
    xw = x * w
    yy = y * y
    yz = y * z
    yw = y * w
    zz = z * z
    zw = z * w

    rot_mat = np.array([
        [1 - 2*yy - 2*zz,     2*xy - 2*zw,     2*xz + 2*yw, 0],
        [    2*xy + 2*zw, 1 - 2*xx - 2*zz,     2*yz - 2*xw, 0],
        [    2*xz - 2*yw,     2*yz + 2*xw, 1 - 2*xx - 2*yy, 0],
        [              0,               0,               0, 1]
    ])
    return rot_mat

def do_transform_point(point_stamp, transform):
    # 提取平移
    trans = transform.transform.translation
    quat = transform.transform.rotation

    # 四元数转旋转矩阵 (4x4)
    rot_mat = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])

    # 设置平移部分
    rot_mat[0, 3] = trans.x
    rot_mat[1, 3] = trans.y
    rot_mat[2, 3] = trans.z

    # 构造点 (x, y, z, 1) 用于齐次变换
    point_vec = np.array([point_stamp.point.x, point_stamp.point.y, point_stamp.point.z, 1.0])

    # 应用变换
    transformed_point = np.dot(rot_mat, point_vec)

    # 返回新的点
    result = PointStamped()
    result.point.x = transformed_point[0]
    result.point.y = transformed_point[1]
    result.point.z = transformed_point[2]
    return result


class PointTransFormer:
    def __init__(self):
        #设置成员变量
        self.optical_point_sub = rospy.Subscriber("/camera_optical_point", PointStamped, self.point_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.transformed_pub = rospy.Publisher("/transformed_point_lidar", PointStamped, queue_size=10)
        self.point_base_pub = rospy.Publisher("/base_point", PointStamped, queue_size=10)
        self.distance_pub = rospy.Publisher("/traffic_light_distance", Float32, queue_size=10)
         # 创建tf_buffer，所有的坐标变化找buffer要
        self.tf_buffer = tf2_ros.Buffer()
        # api内部已经实现了订阅
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def point_callback(self, data):
        # 从相机光学坐标系变换到base_link下，只有y,z变化
        transform = self.tf_buffer.lookup_transform("laser","camera_depth_optical_frame",rospy.Time(0))
        point_lidar = do_transform_point(data, transform)
        self.transformed_pub.publish(point_lidar)
        angle = np.arctan2(point_lidar.point.y,point_lidar.point.x)
        a = int((angle + 0.760577)/self.lidar_data.angle_increment)
        if a==335:
            real_lidar_dis = 1.6699999570846558
        else:
            real_lidar_dis = self.lidar_data.ranges[a]
        virtual_lidar_dis = np.sqrt(np.square(point_lidar.point.y)+np.square(point_lidar.point.x))
        # 缩放系数
        scale = real_lidar_dis/virtual_lidar_dis
        # 雷达坐标下红绿灯坐标
        real_lidar_point = PointStamped()
        real_lidar_point.header.stamp = rospy.Time.now()
        real_lidar_point.point.x = point_lidar.point.x * scale
        real_lidar_point.point.y = point_lidar.point.y * scale
        real_lidar_point.point.z = point_lidar.point.z * scale
        transform = self.tf_buffer.lookup_transform("base_link","laser",rospy.Time(0))
        # base_link坐标下红绿灯坐标
        point_base = do_transform_point(real_lidar_point, transform)
        point_base.header.frame_id = "base_link"
        self.point_base_pub.publish(point_base)
        # 发布红绿灯距离
        dis = Float32()
        dis = point_base.point.x
        self.distance_pub.publish(dis)
            
    def lidar_callback(self,data):
        self.lidar_data = data

if __name__ == '__main__':
    # anonymous表示是否允许同样名字的节点存在，会加后缀
    rospy.init_node('point_transformer', anonymous=False)
    print("Tranforming.....")
    transformer = PointTransFormer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")