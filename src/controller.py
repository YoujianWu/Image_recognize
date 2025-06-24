#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalID
from collections import deque

class TrafficNavController:
    def __init__(self):
        rospy.init_node('traffic_nav_controller', anonymous=True)

        # 订阅红绿灯状态和距离
        rospy.Subscriber('/traffic_light', String, self.traffic_cb)
        rospy.Subscriber('/traffic_light_distance', Float32, self.distance_cb)

        # 发布目标点、取消导航和停车控制
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # RViz 目标监听
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)

        # 记录状态
        self.last_goal = None
        self.last_status = None
        self.current_distance = None
        self.should_stop = False
        self.stop_distance = 2.0  # 修改为 2.5 米

        # 红绿灯状态滑窗（用于停车后判断是否恢复）
        self.status_window_after_stop = deque(maxlen=10)

        self.rate = rospy.Rate(10)  # 10Hz
        rospy.loginfo("控制器已启动。")

    def goal_cb(self, msg):
        self.last_goal = msg
        rospy.loginfo_throttle(1, "接收到新的导航目标点")

    def distance_cb(self, msg):
        try:
            self.current_distance = float(msg.data)
        except ValueError:
            rospy.logwarn("距离数据格式错误: %s", msg.data)
            self.current_distance = None

    def traffic_cb(self, msg):
        status = msg.data

        if status != self.last_status:
            rospy.loginfo("红绿灯状态变化: %s", status)
            self.last_status = status

        # 条件1：红灯 + 距离小于阈值 => 停车
        if status == "red" and self.current_distance is not None and self.current_distance < self.stop_distance:
            if not self.should_stop:
                self.cancel_navigation()
                self.should_stop = True
                self.status_window_after_stop.clear()
                rospy.logwarn("检测红灯且距离 < %.2f 米，小车已停车", self.stop_distance)
            else:
                self.publish_stop_velocity()
            return

        # 条件2：处于停止状态，评估是否恢复导航
        if self.should_stop:
            self.status_window_after_stop.append(status)
            if len(self.status_window_after_stop) > 10:
                self.status_window_after_stop.popleft()

            green_or_none = sum(1 for s in self.status_window_after_stop if s in ["green", "none"])
            red_count = sum(1 for s in self.status_window_after_stop if s == "red")

            rospy.logdebug("状态窗口: %s | green+none: %d | red: %d",
                           list(self.status_window_after_stop), green_or_none, red_count)

            if len(self.status_window_after_stop) == 10 and green_or_none > red_count:
                rospy.loginfo("满足条件：green+none > red，恢复导航")
                self.resume_navigation()
                self.should_stop = False
                self.status_window_after_stop.clear()
            else:
                self.publish_stop_velocity()
            return

        # 正常导航中，不干预
        pass

    def cancel_navigation(self):
        # 取消当前导航目标
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.logwarn("已发布取消导航目标。")
        self.publish_stop_velocity()

    def publish_stop_velocity(self):
        # 持续发布 /cmd_vel 0，确保小车完全停止
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)
        rospy.logdebug("发布 /cmd_vel 0，强制停车。")

    def resume_navigation(self):
        if self.last_goal is not None:
            for i in range(3):  # 重试 3 次确保 move_base 接收
                self.goal_pub.publish(self.last_goal)
                rospy.loginfo("第 %d 次重新发布目标点", i + 1)
                rospy.sleep(0.5)
        else:
            rospy.logwarn("无有效导航目标，无法恢复导航。")

    def spin(self):
        while not rospy.is_shutdown():
            # 停车状态下持续强制发布零速度，防止滑动
            if self.should_stop:
                self.publish_stop_velocity()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TrafficNavController()
        controller.spin()
    except rospy.ROSInterruptException:
        pass