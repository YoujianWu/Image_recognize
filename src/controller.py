#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult

class TrafficNavController:
    def __init__(self):
        rospy.init_node('traffic_nav_controller', anonymous=True)

        # 订阅红绿灯状态和距离
        rospy.Subscriber('/traffic_light', String, self.traffic_cb)
        rospy.Subscriber('/traffic_light_distance', Float32, self.distance_cb)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.reached_cb)

        # 发布目标点、取消导航和速度控制
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 红绿灯状态和距离
        self.traffic_state = None
        self.traffic_distance = float('inf')

        # 目标点列表和当前索引
        self.waypoints = self.create_waypoints()
        self.current_index = 0

        # 是否已发送停止指令
        self.stopped = False
        
        self.goal_succeeded = False

        # 启动主循环
        self.run()

    def create_waypoints(self):
        """
        创建三个目标点，格式为 (x, y, yaw)
        """
        points = [
            (1.0, 0.0, 2.1),    # 红绿灯附近
            (2.0, 1.5, 1.57),   # 终点
            (0.0, 3.0, 3.14)    # 起点
        ]

        waypoints = []
        for i, point in enumerate(points):
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = point[0]
            goal.pose.position.y = point[1]
            goal.pose.orientation.z = point[2]  # 简化表示yaw角
            goal.pose.orientation.w = 1.0      # 默认四元数w=1
            waypoints.append(goal)

        return waypoints

    def traffic_cb(self, msg):
        self.traffic_state = msg.data.lower()
        rospy.loginfo(f"收到红绿灯状态: {self.traffic_state}")

    def distance_cb(self, msg):
        self.traffic_distance = msg.data
        rospy.loginfo(f"红绿灯距离: {self.traffic_distance} 米")

    def send_stop_cmd(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)

    def cancel_current_goal(self):
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)

    def send_next_goal(self):
        if len(self.waypoints) == 0:
            rospy.logwarn("没有可用的目标点")
            return

        goal = self.waypoints[self.current_index]
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)
        rospy.loginfo(f"正在前往第 {self.current_index + 1} 个目标点")
    
    def reached_cb(self,msg):
        if msg.status.status == 3:  
            rospy.loginfo("成功到达目标点")
            self.goal_succeeded = True

    def is_goal_reached(self):
        return self.goal_succeeded                

    def run(self):
        rate = rospy.Rate(10)  # 10Hz 主循环频率
        i=0
        while not rospy.is_shutdown():
            # 先发送第一个目标点
            if i <= 3:
                self.send_next_goal()
                i = i+1
            else:      
                # 红灯处理逻辑
                if self.traffic_state == 'red' and self.current_index == 0 and self.goal_succeeded:
                    if not self.stopped:
                        rospy.loginfo("检测到红灯，正在停车...")
                        goal = PoseStamped()
                        goal.header.frame_id = "map"
                        goal.pose.position.x = 1
                        goal.pose.position.y = 1
                        goal.pose.orientation.z = 1  # 简化表示yaw角
                        goal.pose.orientation.w = 1.0      # 默认四元数w=1
                        goal.header.stamp = rospy.Time.now()
                        self.goal_pub.publish(goal)
                        self.stopped = True
                        self.goal_succeeded = False

                elif self.traffic_state == 'green' and self.stopped and self.goal_succeeded:
                    rospy.loginfo("绿灯亮起，恢复导航")
                    self.stopped = False

                if self.goal_succeeded and not self.stopped:
                    self.current_index = (self.current_index + 1) % len(self.waypoints)
                    self.goal_succeeded = False
                    rospy.loginfo("已到达目标，准备前往下一个目标点")
                    self.send_next_goal()

            rate.sleep()

if __name__ == '__main__':
    try:
        nav = TrafficNavController()
        nav.run()
    except rospy.ROSInterruptException:
        pass            