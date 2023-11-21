#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class WallFollow(Node):
    """Implement Wall Following on the car"""

    def __init__(self, lidarscan_topic):
        super().__init__('wall_follow_node')


        # Subscriber
        # self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        # self.subscription_pose = self.create_subscription(AckermannDriveStamped, "/ego_drive_topic", self.drive_callback, 10)
        self.subscription_pose = self.create_subscription(Odometry, "/ego_racecar/odom", self.state_callback, 10)
        

    # def scan_callback(self, msg):
    #     """Callback function for LaserScan messages. Print the range data."""
    #     ranges = msg.ranges
    #     print(f"LaserScan Ranges: {ranges}")
    #     for i, range_value in enumerate(ranges):
    #         if range_value < 1.0:  # Replace 1.0 with your desired threshold
    #             print(f"Obstacle detected at angle {msg.angle_min + i * msg.angle_increment}")
    

    # def drive_callback(self, drive_msg):
    #     ego_requested_speed = drive_msg.drive.speed
    #     ego_steer = drive_msg.drive.steering_angle
    #     self.logger.info(f"Ego Drive Callback - Requested Speed: {ego_requested_speed}, Steering Angle: {ego_steer}")

    def state_callback(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        position_z = msg.pose.pose.position.z
        print(f"Car State Postion X: {position_x}, Position Y: {position_y}, Position Z: {position_z}")


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow("/ego_racecar/odom")
    rclpy.spin(wall_follow_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()