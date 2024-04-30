import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.qos import QoSProfile
import random

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(PoseStamped, '/NDI/Pointer/measured_cp', qos_profile)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # 枢轴点位置
        pivot = np.array([1, 1, 1])

        # Pointer到枢轴点的偏移
        offset = np.array([2, -1, 0])

        # 生成随机旋转角度
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, 2 * np.pi)
        psi = np.random.uniform(0, 2 * np.pi)

        # 绕Z轴旋转
        Rz = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,             0,             1]
        ])

        # 绕Y轴旋转
        Ry = np.array([
            [np.cos(phi), 0, np.sin(phi)],
            [0,           1, 0],
            [-np.sin(phi),0, np.cos(phi)]
        ])

        # 绕X轴旋转
        Rx = np.array([
            [1, 0,            0],
            [0, np.cos(psi), -np.sin(psi)],
            [0, np.sin(psi),  np.cos(psi)]
        ])

        # 综合旋转矩阵
        R = Rz @ Ry @ Rx

        # 计算Marker的平移，保证pointer tip在枢轴点
        t_marker = pivot - R @ offset

        # 发布消息
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = t_marker[0]
        msg.pose.position.y = t_marker[1]
        msg.pose.position.z = t_marker[2]
        quaternion = self.euler_to_quaternion(theta, phi, psi)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        print(f"Publishing: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        self.publisher.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
