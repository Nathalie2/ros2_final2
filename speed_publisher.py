import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class SpeedPublisher(Node):
    def __init__(self):
        super().__init__('speed_publisher')

        # Subscribe to odometry topic
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Publisher for current speed
        self.speed_pub = self.create_publisher(Float32, '/current_speed', 10)

    def odom_callback(self, msg: Odometry):
        # Calculate the current speed from odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)

        # Publish current speed
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()
    rclpy.spin(speed_publisher)
    speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
