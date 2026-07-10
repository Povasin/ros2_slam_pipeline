#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        
        # Объявляем параметры
        self.declare_parameter('topic', '/ov_msckf/odomimu')
        self.declare_parameter('file_path', '/home/kirill_fdx/ros2_ws/data/trajectory.txt')
        
        # Получаем значения параметров
        topic_name = self.get_parameter('topic').value
        output_file = self.get_parameter('file_path').value
        
        # Подготовка файла
        out_dir = os.path.dirname(output_file)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir)

        self.output_file = output_file
        
        with open(self.output_file, 'w') as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")

        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odom_callback,
            10)
        
        self.get_logger().info(f'Слушаю топик: {topic_name}')
        self.get_logger().info(f'Пишу траекторию в: {self.output_file}')

    def odom_callback(self, msg):
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        timestamp = sec + nanosec * 1e-9

        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        with open(self.output_file, 'a') as f:
            f.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()