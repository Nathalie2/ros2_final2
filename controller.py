import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
import math
from tf_transformations import euler_from_quaternion
class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.logger = self.get_logger()

        # Vehicle Parameters
        self.wheelbase = 1.8  # Distance between the front and rear axles of the vehicle.
        
        # Tunable parameters
        self.k = 1.0  # Tuning parameter for adjusting lookahead distance dynamically.
        self.linear_velocity = 3.0  # Constant forward velocity for the vehicle.
        
        # Subscribe to /pose_msg topic for the target pose
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.pose_callback,
            10  # Message queue size.
        )
        
        # Subscribe to /pose_info topic for the current car pose
        self.car_pose_subscription = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.car_pose_callback,
            10  
        )
        
        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # Publisher for sending velocity commands.
        
        self.logger.info("Pure Pursuit Controller node started")  # Log a message indicating the node has started.
        
        # Initialize variables to store poses and orientation
        self.target_pose = None  # Placeholder for the target pose.
        self.car_pose = None  # Placeholder for the car's current pose.
        self.yaw = None  # Placeholder for the car's yaw (orientation).

    def pose_callback(self, msg: PoseStamped):
        # Callback for receiving the target pose from /pose_msg topic.
        self.target_pose = msg.pose  # Update the target pose.
        self.logger.info(f'Received target pose: {self.target_pose}')  # Log the received target pose.

    def car_pose_callback(self, msg: PoseArray):
        # Callback for receiving the car's current pose from /pose_info topic.
        if len(msg.poses) > 0:  # Ensure there are poses in the message.
            self.car_pose = msg.poses[0]  # Update the car's current pose.
            self.yaw = self.get_yaw_from_pose(self.car_pose)  # Calculate the car's yaw (orientation).
            self.logger.info(f'Received car pose: {self.car_pose}, yaw: {self.yaw}')  # Log the car's pose and yaw.
            self.control_vehicle()  # Call the control function to generate velocity commands.

    def get_yaw_from_pose(self, pose):
        """Extract yaw from the quaternion of the car's pose."""
        orientation_q = pose.orientation  # Get the orientation as a quaternion.
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]  # Convert quaternion to a list.
        _, _, yaw = euler_from_quaternion(orientation_list)  # Extract yaw using Euler conversion.
        return yaw

    def distance(self, point1, point2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)  # Compute the distance formula.

    def steering_angle_to_angular_velocity(self, steering_angle):
        """Convert the steering angle to angular velocity."""
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase  # Use vehicle geometry for conversion.
        return angular_velocity

    def control_vehicle(self):
        """Control the vehicle based on the current car pose and target pose."""
        if self.target_pose is None or self.car_pose is None or self.yaw is None:
            return  # Do nothing if the required data is missing.
        
        # Extract the current and target positions.
        current_x = self.car_pose.position.x
        current_y = self.car_pose.position.y
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y
        
        # Calculate dx, dy, and lookahead distance.
        dx = target_x - current_x # Calculates the differences in x positions between the car and the target.
        dy = target_y - current_y # Calculates the differences in  y positions between the car and the target.
        lookahead_distance = self.distance((current_x, current_y), (target_x, target_y))  # Distance to target.

        # Adjust lookahead distance dynamically based on velocity.
        lookahead_distance = max(self.k * self.linear_velocity, lookahead_distance)

        # Calculate the steering angle.
        alpha = math.atan2(dy, dx) - self.yaw  # Relative angle between car's heading and target.
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), lookahead_distance)  # Steering angle formula.

        # Create and publish Twist message for vehicle control.
        twist = Twist()
        twist.linear.x = self.linear_velocity  # Set forward velocity.
        twist.angular.z = self.steering_angle_to_angular_velocity(steering_angle)  # Set angular velocity.
        self.cmd_vel_publisher.publish(twist)  # Publish the Twist message.

        # Log the current waypoint and velocity commands.
        self.logger.info(f'Current Waypoint: x={target_x}, y={target_y}')
        self.logger.info(f'Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()