#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg
import math

def publish_extra_frame():
    rospy.init_node('extra_frame_tf2_publisher')

    # 创建一个 TransformBroadcaster 实例
    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)  # 每秒发送10次变换

    while not rospy.is_shutdown():
        # 构造 TransformStamped 消息
        transform = geometry_msgs.msg.TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_link"     # 父坐标系
        transform.child_frame_id = "camera_depth_optical_frame"    # 子坐标系

        # 设置平移部分
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = -0.02
        transform.transform.translation.z = 0.0

        # 设置旋转部分（四元数）
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5

        # 广播该变换
        br.sendTransform(transform)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_extra_frame()
    except rospy.ROSInterruptException:
        pass