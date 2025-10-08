import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.01  # Integral gain
        self.kd = 0.05  # Derivative gain

        self.target_speed = 3.0  # Target linear velocity (m/s)
        self.integral = 0.0
        self.previous_error = 0.0

        # Subscribers
        self.current_speed_sub = self.create_subscription(
            Float32, '/current_speed', self.current_speed_callback, 10
        )

        # Publisher for linear velocity (cmd_vel)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_speed = 0.0

    def current_speed_callback(self, msg: Float32):
        self.current_speed = msg.data
        self.control_velocity()

    def control_velocity(self):
        error = self.target_speed - self.current_speed
        self.integral += error
        derivative = error - self.previous_error

        # PID control
        linear_velocity = (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )

        # Create and publish Twist message with linear velocity
        twist = Twist()
        twist.linear.x = max(0.0, linear_velocity)  # Ensure velocity is non-negative
        self.cmd_vel_publisher.publish(twist)

        self.previous_error = error
        self.get_logger().info(f'Published linear velocity: {twist.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
