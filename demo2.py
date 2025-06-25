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
import cv2
import imutils
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from jeff_messages.msg import Beacon
from cv_bridge import CvBridge
from . import transformations as trans

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class my_node(Node):
    def __init__(self):
        # Initialise subs, pubs, service calls, path object
        super().__init__('demo1_node')
        map_qos_policy = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        cam_qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


        self.declare_parameters(
            namespace='',
            parameters=[
                ('beacons.id', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('beacons.top', rclpy.Parameter.Type.STRING_ARRAY),
                ('beacons.bottom', rclpy.Parameter.Type.STRING_ARRAY)
            ]
        )
        self.beacons = (self.get_parameter('beacons.id').value, self.get_parameter('beacons.top').value, self.get_parameter('beacons.bottom').value)
                
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.explore_path_pub = self.create_publisher(Path, '/ecte477/path', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/ecte477/map', map_qos_policy)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.callback_map, map_qos_policy)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.subscriber_colour = self.create_subscription(Image, '/intel_realsense_r200_depth/image_raw', self.callback_colour, cam_qos_policy)
        self.subscriber_depth = self.create_subscription(Image, '/intel_realsense_r200_depth/depth/image_raw', self.callback_depth, cam_qos_policy)
        self.subscriber_info = self.create_subscription(CameraInfo, '/intel_realsense_r200_depth/camera_info', self.callback_cam_info, cam_qos_policy)
        self.beacon_markers = self.create_publisher(MarkerArray, '/ecte477/beacon_markers', 5)
        self.beacon_pub = self.create_publisher(Beacon, '/ecte477/beacons', 5)

        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.K = None
        self.transform_cam_to_world = None
        self.beacon_array = MarkerArray()
        self.beacons_found = []
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.colour_ranges = (('red', (0,200,100), (9,255,205), (1.0,0.0,0.0,1.0)), ('yellow', (25,175,100), (32,255,205), (1.0,1.0,0.0,1.0)), ('blue', (115,155,95), (120,255,132), (0.0,0.0,1.0,1.0)), ('green', (55,200,90), (60,255,135), (0.0,1.0,0.0,1.0)))

        		
    def callback_map(self, data):
        # Do something with map
        self.map_pub.publish(data)
        
    def callback_odom(self, data):
        # Do something with odometry
        path_point = PoseStamped()
        path_point.pose = data.pose.pose
        self.path.poses.append(path_point)
        self.explore_path_pub.publish(self.path)
        self.transform_cam_to_world = trans.msg_to_se3(data.pose.pose)

    def callback_colour(self, colour_image):
        self.get_logger().info('[Image Processing] callback_colour')
        self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        colours_found = []
        for colour in self.colour_ranges:
            mask = cv2.inRange(hsv, colour[1], colour[2])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            if len(contours) == 0:
                continue
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            if w > 15.0 and h > 15.0:
                colours_found.append([colour[0], int(x+w/2), int(y+h/2)])
        if len(colours_found) == 2:
            if abs(colours_found[0][1] - colours_found[1][1]) < 5:
                # Beacon Found
                if colours_found[0][2] < colours_found[1][2]:
                    top = colours_found[0][0]
                    bottom = colours_found[1][0]
                else:
                    top = colours_found[1][0]
                    bottom = colours_found[0][0]
                self.get_logger().info('[Image Processing] Found Beacon {}/{}'.format(top, bottom))
                for id in self.beacons[0]:
                    if self.beacons[1][id] == top and self.beacons[2][id] == bottom and not (id in self.beacons_found):
                        self.beacons_found.append(id)
                        cent_x = (colours_found[0][1] + colours_found[1][1])/2
                        cent_y = (colours_found[0][2] + colours_found[1][2])/2
                        self.mark_beacon(id, int(cent_x), int(cent_y), top, bottom)
                        break
        
    def callback_depth(self, depth_image):
        self.get_logger().info('[Image Processing] callback_depth')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
    
    def callback_cam_info(self, info):
        self.K = np.array(info.k).reshape([3, 3]) 

    def mark_beacon(self, id, x, y, top_colour, bottom_colour):
        if np.any(self.K) and np.any(self.transform_cam_to_world) and np.any(self.depth_frame):
            depth = self.depth_frame[y, x]
            p_h = np.array([[x], [y], [1]])
            p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
            p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]])
            p3d_w_h = np.matmul(self.transform_cam_to_world, p3d_h)
            p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]], [p3d_w_h[1][0]/p3d_w_h[3][0]], [p3d_w_h[2][0]/p3d_w_h[3][0]]])
            marker = Marker()
            marker.id = id
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            my_point = Point()
            my_quat = Quaternion()
            my_pose = Pose()
            my_point.x, my_point.y, my_point.z = p3d_w[0][0], p3d_w[1][0], p3d_w[2][0] 
            my_quat.x, my_quat.y, my_quat.z, my_quat.w = 0.0, 1.0, 0.0, 1.0
            my_pose.position, my_pose.orientation = my_point, my_quat
            marker.pose = my_pose
            my_scale = Vector3()
            my_scale.x, my_scale.y, my_scale.z = 0.1, 0.1, 0.1
            marker.scale = my_scale
            my_colour = ColorRGBA()
            for colour in self.colour_ranges:
                if colour[0] == top_colour:
                    my_colour.r, my_colour.g, my_colour.b, my_colour.a = colour[3][0], colour[3][1], colour[3][2], colour[3][3]
                    break
            marker.color = my_colour
            self.beacon_array.markers.append(marker)
            self.beacon_markers.publish(self.beacon_array)
            beacon = Beacon()
            beacon.header = marker.header
            beacon.top = top_colour
            beacon.bottom = bottom_colour
            beacon.position = my_point
            self.beacon_pub.publish(beacon)
	
# Main function
def main(args=None):
    rclpy.init(args=args)

    mn = my_node()

    rclpy.spin(mn)

if __name__ == '__main__':
    main()