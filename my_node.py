"""
    my_node.py

    A ROS node that repeats the map and odometry topic to the correct ecte477 
    namespace topics for map and path.

    Subscribed: map/, odom/
    Publishes: ecte477/map/, ecte477/e_path/, ecte477/r_path/
    Services: explore/explore_service
    Created: 2021/04/08
    Author: Brendan Halloran
    Updated 12/04/2022 by Jeff Moscrop
    Updated 2/04/2024 for ROS2 by Jeff Moscrop
"""

import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class my_node(Node):
    def __init__(self):
        # Initialise subs, pubs, service calls, path object
        super().__init__('demo1_node')
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.command_pub = self.create_publisher(String, '/start_explore', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.explore_path_pub = self.create_publisher(Path, '/ecte477/e_path', 1)
        self.return_path_pub = self.create_publisher(Path, '/ecte477/r_path', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/ecte477/maze', qos_policy)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.callback_map, qos_policy)
        self.stack_sub = self.create_subscription(MarkerArray, '/stack_points', self.callback_stack_points, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom, 1)

        self.e_path = Path()
        self.r_path = Path()
        self.e_path.header.frame_id = 'odom'
        self.r_path.header.frame_id = 'odom'

        self.home_time = False
        
        start_message = String()
        start_message.data = 'start'
        time.sleep(8)
        self.command_pub.publish(start_message)
		
    def callback_map(self, data):
        # Do something with map
        self.map_pub.publish(data)
        
    def callback_odom(self, data):
        # Do something with odometry
        path_point = PoseStamped()
        path_point.pose = data.pose.pose
        if self.home_time:
            self.r_path.poses.append(path_point)
            self.return_path_pub.publish(self.r_path)
        else:
            self.e_path.poses.append(path_point)
            self.explore_path_pub.publish(self.e_path)
        
    def callback_stack_points(self, stack_points):
        # Do something with stack_points array
        if len(stack_points.markers) == 0:
            print("Exploration is completed, returning home")
            self.home_time = True
            time.sleep(5)
            home_pos = PoseStamped()
            home_pos.header.frame_id = 'map'
            home_pos.pose.position.x = 0.0
            home_pos.pose.position.y = 0.0
            home_pos.pose.position.z = 0.0
            home_pos.pose.orientation.w = 1.0
            self.goal_pub.publish(home_pos)         
	
# Main function
def main(args=None):
    rclpy.init(args=args)

    mn = my_node()

    rclpy.spin(mn)

if __name__ == '__main__':
    main()