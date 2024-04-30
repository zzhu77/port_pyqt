import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
import json

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/NDI/Pointer/measured_cp', 10)
        timer_period = 5  # seconds
        self.ref_points_p = self.read_ref_markers_ct('/home/zhu/cis2/Study4/Shape.mrk.json')
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def read_ref_markers_ct(self, file_path):
        with open(file_path, 'r') as file:
            data = json.load(file)
        
        control_points = data['markups'][0]['controlPoints']
        
        positions = []
        
        for point in control_points:
            positions.append(point['position'])
        
        return np.array(positions)

    def timer_callback(self):
        if self.i < len(self.ref_points_p):
            point = self.ref_points_p[self.i]
            for _ in range(1):  # Publish each point 10 times
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "world"  # Example frame_id
                msg.pose.position.x = point[0]
                msg.pose.position.y = point[1]
                msg.pose.position.z = point[2]
                msg.pose.orientation.x = random.random()
                msg.pose.orientation.y = random.random()
                msg.pose.orientation.z = random.random()
                msg.pose.orientation.w = random.random()
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
            self.i += 1
        else:
            self.i = 0  # Reset index to loop over the points again if needed

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
