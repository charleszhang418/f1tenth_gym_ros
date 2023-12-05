#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class WallFollow(Node):
    """Implement Wall Following on the car"""

    def __init__(self):
        super().__init__('wall_follow_node')

        # Subscriber
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, "/ego_racecar/odom", self.state_callback, 10)

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive_topic", 25)

        # State variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.midpoints = []

        self.output_timer = self.create_timer(1, self.print_car_state_and_midpoints)

    def scan_callback(self, msg):
        # Extract relevant attributes from the LaserScan message
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Assuming equal distance between left and right boundaries
        left_boundary_index = 0
        right_boundary_index = len(ranges) - 1

        # Calculate the midpoint distances
        # self.midpoints = []
        # for i in range(left_boundary_index, right_boundary_index + 1):
        #     midpoint_distance = (ranges[i] + ranges[right_boundary_index - i]) / 2
        #     self.midpoints.append(midpoint_distance)
        for i, distance in enumerate(ranges):
            # Calculate the angle for this measurement
            angle = angle_min + i * angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            self.midpoints.append((x, y))

    def follow_wall(self):
        speed = 0.5
        steering_angle = 0.0

        # Publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def state_callback(self, msg):
        """
        pose:
            x
            y
            z
            rotation about X axis
            rotation about Y axis
            rotation about Z axis
        """
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

        # Output car state and midpoints
        self.print_car_state_and_midpoints()

    def print_car_state_and_midpoints(self):
        print(f"Car State - Position X: {self.position_x}, Y: {self.position_y}")
        #print(f"Midpoints: {self.midpoints}")

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
