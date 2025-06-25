import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualStopperNode(Node):
    def __init__(self):
        super().__init__('visual_stopper_node')

        # Publisher for robot velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to the LIDAR sensor
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        # Subscriber to the robot's camera
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # Standard Gazebo camera topic
            self.image_callback,
            10)

        self.bridge = CvBridge()
        
        # Timer to run the control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.forward_distance = 999.0
        self.red_detected = False

        self.get_logger().info('Visual Stopper Node has started.')

    def laser_callback(self, msg):
        """Processes LIDAR data to find the distance to the obstacle directly in front."""
        # The forward distance is the first element in the ranges array
        if len(msg.ranges) > 0:
            self.forward_distance = msg.ranges[0]

    def image_callback(self, msg):
        """Processes camera image to detect if a red object is visible."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range for red color in HSV
        # This might need tuning depending on the specific red in the world
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        red_mask = mask1 + mask2

        # Check if any red is detected in the mask
        if cv2.countNonZero(red_mask) > 100: # Threshold to avoid noise
            self.red_detected = True
        else:
            self.red_detected = False

    def control_loop(self):
        """Main control loop that makes decisions and publishes commands."""
        twist_msg = Twist()
        distance_threshold = 0.7  # meters

        if self.red_detected:
            # Red object detected: STOP completely
            self.get_logger().info('Red object detected! Stopping.')
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        elif self.forward_distance <= distance_threshold:
            # Generic obstacle detected: Stop moving forward and TURN
            self.get_logger().info('Obstacle detected! Turning...')
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5  # rad/s
        else:
            # Path is clear: Move forward
            self.get_logger().info('Path is clear, moving forward.')
            twist_msg.linear.x = 0.15  # m/s
            twist_msg.angular.z = 0.0

        # Publish the final movement command
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    visual_stopper_node = VisualStopperNode()
    rclpy.spin(visual_stopper_node)
    visual_stopper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
