#!/usr/bin/env python3
"""
Demonstration 3: Week 12 Lab Basic Implementation
Following the exact algorithms from ECTE477 Week 12 Lab Notes
"""

import rclpy
import message_filters
import cv2
import numpy as np
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class Demo3Node(Node):
    def __init__(self):
        super().__init__('demo3_basic')
        self.get_logger().info('Starting Demonstration 3: Basic Week 12 Lab Implementation')

        self.bridge = CvBridge()

        # Camera topics for simulation
        colour_image_topic = "/intel_realsense_r200_depth/image_raw"
        depth_image_topic = "/intel_realsense_r200_depth/depth/image_raw"

        colour_sub = message_filters.Subscriber(self, Image, colour_image_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_image_topic)

        self.camera_sync = message_filters.TimeSynchronizer([colour_sub, depth_sub], 10)
        self.camera_sync.registerCallback(self.camera_callback)

        self.move_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Add odometry subscriber for distance tracking
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # HSV color ranges from lab notes
        self.lower_yellow = (25, 200, 100)
        self.upper_yellow = (35, 255, 255)
        self.lower_red = (0, 200, 100)
        self.upper_red = (5, 255, 255)
        self.lower_blue = (115, 200, 100)
        self.upper_blue = (125, 255, 255)

        # Algorithm 2: Stop sign variables
        self.stopped = False
        self.time_stopped = None
        self.stop_duration = 3.0  # 3 seconds
        self.min_depth = 0.10  # Stop when beacon is closer than 10cm (was 25cm)

        # Algorithm 3: Obstacle detection with speed management
        self.depth_threshold = 0.6  # 60cm threshold
        self.obstacle_detected = False
        self.initial_speed = 0.1   # Low speed for initial obstacle detection
        self.normal_speed = 0.2    # Normal speed after obstacles are cleared
        self.current_speed = self.initial_speed  # Start with low speed
        
        # Offset-based turning system
        self.current_position = None
        self.turn_detected = False
        self.turn_direction = None  # "LEFT" or "RIGHT"
        self.turn_detection_position = None
        self.turn_offset_distance = 0.35  # meters - continue straight for 0.5m before turning

        # Display image
        self.display_image = None

        # Main processing loop
        self.get_logger().info('Node initialized, entering display loop.')
        while rclpy.ok():
            if self.display_image is not None:
                cv2.imshow("Demo 3 - Basic Week 12 Lab", self.display_image)
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    break
            rclpy.spin_once(self, timeout_sec=0.01)
        
        cv2.destroyAllWindows()
        self.get_logger().info('Shutting down Demo 3.')

    def find_line(self, colour_image, lower_hsv, upper_hsv):
        """
        Find line using the exact method from lab notes
        """
        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        h, w = mask.shape
        
        # Lab notes: search in bottom portion with wider region than 10 pixels
        search_top = 3*h//4  # Search in bottom quarter
        search_bot = search_top + h//4  # Use bottom quarter of image
        mask[0:search_top][:] = 0
        if search_bot < h:
            mask[search_bot:h][:] = 0

        M = cv2.moments(mask)
        if M['m00'] > 200:  # Minimum area threshold
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return True, cx, cy
        
        return False, 0, 0

    def find_beacon(self, colour_image, depth_image):
        """Find blue beacon for stop sign detection - using area instead of depth"""
        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        M = cv2.moments(mask)
        if M['m00'] > 500:  # Minimum area for beacon
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            beacon_area = M['m00']
            
            self.get_logger().info(f"Blue beacon detected at pixel ({cx}, {cy}) with area {beacon_area:.0f}")
            
            # Use area-based detection instead of depth - larger area means closer
            # From previous logs: areas ranged from ~70k (far) to ~800k+ (close)
            # Set threshold for very large area indicating very close proximity
            area_threshold = 600000  # Only trigger when beacon is very large (very close)
            
            if beacon_area > area_threshold:
                self.get_logger().info(f"STOP SIGN DETECTED - area {beacon_area:.0f} > {area_threshold} threshold!")
                return True, cx, cy
            else:
                self.get_logger().info(f"Blue beacon too far: area {beacon_area:.0f} < {area_threshold} threshold")
            
        return False, 0, 0

    def odom_callback(self, odom_msg):
        """Track robot position for distance-based turning"""
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        self.current_position = (x, y)
    
    def calculate_distance(self, pos1, pos2):
        """Calculate distance between two positions"""
        if pos1 is None or pos2 is None:
            return 0.0
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
    
    def get_distance_since_turn_detection(self):
        """Get distance traveled since turn was detected"""
        if self.turn_detection_position is None or self.current_position is None:
            return 0.0
        return self.calculate_distance(self.turn_detection_position, self.current_position)

    def camera_callback(self, colour_msg, depth_msg):
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        h, w, _ = colour_image.shape
        display_image = colour_image.copy()
        
        # Algorithm 2: Stop sign detection (highest priority)
        beacon_found, beacon_x, beacon_y = self.find_beacon(colour_image, depth_image)
        if beacon_found:
            cv2.circle(display_image, (beacon_x, beacon_y), 10, (0, 0, 255), -1)  # Red circle as in lab notes
            
            # Beacon is already close (depth checked in find_beacon), so stop
            if not self.stopped:
                self.stopped = True
                self.time_stopped = time.time()
                # Get actual depth for logging
                beacon_depth_raw = depth_image[beacon_y, beacon_x]
                beacon_depth = beacon_depth_raw / 1000.0 if beacon_depth_raw > 0 else 0.0
                self.get_logger().info(f"Stop sign detected at {beacon_depth:.2f}m - STOPPING")

        # Handle stop sign duration (Algorithm 2)
        current_time = time.time()
        if self.stopped and self.time_stopped is not None:
            if current_time - self.time_stopped < self.stop_duration:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.move_pub.publish(twist)
                remaining = self.stop_duration - (current_time - self.time_stopped)
                cv2.putText(display_image, f"STOPPED: {remaining:.1f}s", 
                           (w//2 - 100, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                self.display_image = display_image
                return

        # Algorithm 3: Obstacle detection
        dh, dw = depth_image.shape
        centre_depth_raw = depth_image[dh//2, dw//2]
        
        if centre_depth_raw > 0 and centre_depth_raw < 100:  # Lab notes: 100 means max range/invalid
            centre_depth = centre_depth_raw / 1000.0  # Convert mm to m
            
            if centre_depth < self.depth_threshold:
                # Obstacle detected
                if not self.obstacle_detected:
                    self.obstacle_detected = True
                    self.get_logger().info(f"Obstacle detected at {centre_depth:.2f}m - STOPPING")
                
                twist = Twist()
                twist.linear.x = 0.0  # Stop completely when obstacle detected
                twist.angular.z = 0.0
                self.move_pub.publish(twist)
                cv2.putText(display_image, f"OBSTACLE: {centre_depth:.2f}m", 
                           (w//2 - 100, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.display_image = display_image
                return
            else:
                # No obstacle detected - update speed if we were previously in obstacle mode
                if self.obstacle_detected:
                    self.obstacle_detected = False
                    self.current_speed = self.normal_speed  # Switch to normal speed
                    self.get_logger().info("Obstacle cleared - resuming with normal speed")
                elif self.current_speed == self.initial_speed:
                    # First time clear path detected, switch to normal speed
                    self.current_speed = self.normal_speed
                    self.get_logger().info("Initial path clear - switching to normal speed")
        else:
            # Invalid depth reading - treat as clear path
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.current_speed = self.normal_speed
                self.get_logger().info("Obstacle cleared (invalid depth) - resuming with normal speed")

        # Algorithm 1: Lane following with 0.8m offset
        colours_found = {'RED': False, 'YELLOW': False}
        colours_location = {'RED': 0, 'YELLOW': 0}
        
        # Find both lines
        yellow_found, yellow_x, yellow_y = self.find_line(colour_image, self.lower_yellow, self.upper_yellow)
        red_found, red_x, red_y = self.find_line(colour_image, self.lower_red, self.upper_red)
        
        colours_found['YELLOW'] = yellow_found
        colours_found['RED'] = red_found
        if yellow_found:
            colours_location['YELLOW'] = yellow_x
            cv2.circle(display_image, (yellow_x, yellow_y), 10, (0, 0, 255), -1)  # Red circle as in lab notes
        if red_found:
            colours_location['RED'] = red_x  
            cv2.circle(display_image, (red_x, red_y), 10, (0, 0, 255), -1)  # Red circle as in lab notes

        # Detect turns and manage offset
        distance_since_detection = self.get_distance_since_turn_detection()
        
        # Check for turn detection
        if self.current_position is not None:
            if not self.turn_detected:
                # Check if we should detect a turn
                if colours_found['RED'] and colours_found['YELLOW']:
                    # Both lines visible - reset any previous detection
                    pass
                elif not colours_found['RED'] and colours_found['YELLOW']:
                    # Lost red line - detect right turn
                    self.turn_detected = True
                    self.turn_direction = "RIGHT"
                    self.turn_detection_position = self.current_position
                    self.get_logger().info(f"TURN DETECTED: RIGHT at {self.current_position}")
                elif colours_found['RED'] and not colours_found['YELLOW']:
                    # Lost yellow line - detect left turn
                    self.turn_detected = True
                    self.turn_direction = "LEFT"
                    self.turn_detection_position = self.current_position
                    self.get_logger().info(f"TURN DETECTED: LEFT at {self.current_position}")
            else:
                # Turn detected - check if we should reset
                if colours_found['RED'] and colours_found['YELLOW']:
                    # Found both lines again - reset turn detection
                    self.turn_detected = False
                    self.turn_direction = None
                    self.turn_detection_position = None
                    self.get_logger().info("Both lines found - turn detection reset")

        twist = Twist()

        # Execute Algorithm 1 with offset consideration
        if not colours_found['RED'] and not colours_found['YELLOW']:
            # Stop() - end of road
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cv2.putText(display_image, "STOPPED - END OF ROAD", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
        elif self.turn_detected and distance_since_detection < self.turn_offset_distance:
            # Turn detected but still within offset distance - continue straight
            remaining_distance = self.turn_offset_distance - distance_since_detection
            twist.linear.x = self.current_speed
            twist.angular.z = 0.0  # Continue straight
            cv2.putText(display_image, f"TURN DETECTED ({self.turn_direction}) - CONTINUE STRAIGHT", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.putText(display_image, f"Distance to turn: {remaining_distance:.2f}m", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            
        elif self.turn_detected and distance_since_detection >= self.turn_offset_distance:
            # Execute the detected turn
            if self.turn_direction == "RIGHT":
                twist.linear.x = self.current_speed
                twist.angular.z = -0.5  # Turn right
                cv2.putText(display_image, "EXECUTING TURN RIGHT", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            else:  # LEFT
                twist.linear.x = self.current_speed
                twist.angular.z = 0.5   # Turn left
                cv2.putText(display_image, "EXECUTING TURN LEFT", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
        else:
            # Normal Algorithm 1 behavior
            if not colours_found['RED']:
                # TurnRight() - immediate (no offset detected)
                twist.linear.x = self.current_speed
                twist.angular.z = -0.5  # Turn right
                cv2.putText(display_image, "TURN RIGHT", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
            elif not colours_found['YELLOW']:
                # TurnLeft() - immediate (no offset detected)
                twist.linear.x = self.current_speed
                twist.angular.z = 0.5   # Turn left
                cv2.putText(display_image, "TURN LEFT", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
            else:
                # TrackLane(Avg(coloursLocation(RED), coloursLocation(YELLOW)))
                avg_x = (colours_location['RED'] + colours_location['YELLOW']) / 2
                err = avg_x - w/2
                
                twist.linear.x = self.current_speed
                twist.angular.z = -float(err) / 1000  # Lab notes proportional control
                
                cv2.putText(display_image, f"LANE FOLLOWING - Error: {err:.0f}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Visual indicators
                cv2.circle(display_image, (int(avg_x), h-50), 8, (0, 255, 0), -1)  # Lane center
                cv2.line(display_image, (w//2, 0), (w//2, h), (255, 255, 255), 1)  # Image center

        self.move_pub.publish(twist)
        
        # Enhanced status display
        cv2.putText(display_image, f"Speed: {twist.linear.x:.2f} Turn: {twist.angular.z:.2f}", 
                   (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if self.turn_detected:
            cv2.putText(display_image, f"Offset distance: {distance_since_detection:.2f}m", 
                       (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        self.display_image = display_image


def main(args=None):
    rclpy.init(args=args)
    try:
        demo3_node = Demo3Node()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down.")
    finally:
        if 'demo3_node' in locals() and demo3_node:
            if rclpy.ok():
                demo3_node.destroy_node()
            if not rclpy.is_shutdown():
                rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main() 