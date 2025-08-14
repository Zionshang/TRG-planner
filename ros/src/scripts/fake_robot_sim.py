#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一个简单的全向(始终支持 vx+vy)机器人仿真节点：
- 订阅: /cmd_vel (geometry_msgs/Twist)
- 发布: fake_robot_pose (nav_msgs/Odometry)
- 订阅: goal_pose (geometry_msgs/PoseStamped) -> 直接转发为 fake_goal (PoseStamped)
- 初始位姿: 参数 ~init_x, ~init_y, ~init_yaw
- 简单积分: x += vx*dt*cos(yaw) - vy*dt*sin(yaw); y += vx*dt*sin(yaw) + vy*dt*cos(yaw); yaw += w*dt
"""
import math
import signal
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class FakeRobotSim(Node):
    def __init__(self):
        super().__init__('fake_robot_sim')

        # Params (已去除 holonomic 与噪声相关参数)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('freq', 50.0)  # Hz
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        gp = self.get_parameter
        self.x = float(gp('init_x').get_parameter_value().double_value)
        self.y = float(gp('init_y').get_parameter_value().double_value)
        self.yaw = float(gp('init_yaw').get_parameter_value().double_value)
        self.frame_id = gp('frame_id').get_parameter_value().string_value
        self.child_frame_id = gp('child_frame_id').get_parameter_value().string_value

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, 'fake_robot_pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'fake_goal', 10)

        self.last_cmd: Twist = Twist()

        self.last_time: Optional[float] = None
        freq = float(gp('freq').get_parameter_value().double_value)
        dt = 1.0 / max(freq, 1.0)
        self.timer = self.create_timer(dt, self.update)

        self.get_logger().info('FakeRobotSim started freq=%.2f Hz (omnidirectional, no noise)' % (freq,))

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg

    def goal_cb(self, msg: PoseStamped):
        # 直接转发到 fake_goal
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.pose = msg.pose
        self.goal_pub.publish(out)
        self.get_logger().info('Relay goal to fake_goal (%.2f, %.2f)' % (msg.pose.position.x, msg.pose.position.y))

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        vx = self.last_cmd.linear.x
        vy = self.last_cmd.linear.y  # 始终支持侧向速度
        w = self.last_cmd.angular.z

        # 积分
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += (vx * cos_y - vy * sin_y) * dt
        self.y += (vx * sin_y + vy * cos_y) * dt
        self.yaw += w * dt
        # 归一化角度
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        elif self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # 发布里程计
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # yaw -> quaternion
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.get_logger().info('FakeRobotSim shutting down')
        return super().destroy_node()


def main(args=None):
    def _signal_handler(sig, frame):
        print('Ctrl+C received, shutting down fake_robot_sim')
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, _signal_handler)

    rclpy.init(args=args)
    node = FakeRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
