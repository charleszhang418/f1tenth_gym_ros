#!/usr/bin/env python3
import math
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
        

    """
    float32 angle_min        # start angle of the scan [rad]
    float32 angle_max        # end angle of the scan [rad]
    float32 angle_increment  # angular distance between measurements [rad]

    float32 time_increment   # time between measurements [seconds] - if your scanner
                            # is moving, this will be used in interpolating position
                            # of 3d points
    float32 scan_time        # time between scans [seconds]

    float32 range_min        # minimum range value [m]
    float32 range_max        # maximum range value [m]

    float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    float32[] intensities    # intensity data [device-specific units].  If your
                            # device does not provide intensities, please leave
                            # the array empty.
    """
    def scan_callback(self, msg):
    #     """Callback function for LaserScan messages. Print the range data."""
    #     ranges = msg.ranges
    #     print(f"LaserScan Ranges: {ranges}")
    #     for i, range_value in enumerate(ranges):
    #         if range_value < 1.0:  # Replace 1.0 with your desired threshold
    #             print(f"Obstacle detected at angle {msg.angle_min + i * msg.angle_increment}")
    # Extract relevant attributes from the LaserScan message
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Define the angles corresponding to the sides of your car
        angle_left_side = -math.pi / 4  # Adjust based on your specific setup
        angle_right_side = math.pi / 4   # Adjust based on your specific setup

        # Calculate the corresponding indices in the ranges array
        index_left_side = int((angle_left_side - angle_min) / angle_increment)
        index_right_side = int((angle_right_side - angle_min) / angle_increment)

        # Get the range values at the specified angles
        distance_left_side = ranges[index_left_side]
        distance_right_side = ranges[index_right_side]

        return distance_left_side, distance_right_side
    
        # center_points = []

        # # Iterate over the range of indices corresponding to the walls
        # for i in range(index_left_wall, index_right_wall + 1):
        #     # Calculate the angle for the current index
        #     current_angle = angle_min + i * angle_increment

        #     # Calculate the center point in Cartesian coordinates
        #     x = ranges[i] * math.cos(current_angle)
        #     y = ranges[i] * math.sin(current_angle)

        #     center_points.append((x, y))

        # return center_points


        """
        """
        # Initialize arrays to store left and right wall points
        left_wall_points = []
        right_wall_points = []

        # Iterate over the range of indices corresponding to the walls
        for i in range(index_left_wall, index_right_wall + 1):
            # Calculate the angle for the current index
            current_angle = angle_min + i * angle_increment

            # Calculate the wall point in Cartesian coordinates
            x = ranges[i] * math.cos(current_angle)
            y = ranges[i] * math.sin(current_angle)

            # Determine whether the point is on the left or right side
            if current_angle < 0:
                left_wall_points.append((x, y))
            else:
                right_wall_points.append((x, y))

        # Calculate the center points
        center_points = [(0, 0)] * max(len(left_wall_points), len(right_wall_points))

        for i in range(len(center_points)):
            x_left, y_left = left_wall_points[i] if i < len(left_wall_points) else (0, 0)
            x_right, y_right = right_wall_points[i] if i < len(right_wall_points) else (0, 0)

            center_points[i] = ((x_left + x_right) / 2, (y_left + y_right) / 2)

        return left_wall_points, right_wall_points, center_points
    

    # def drive_callback(self, drive_msg):
    #     ego_requested_speed = drive_msg.drive.speed
    #     ego_steer = drive_msg.drive.steering_angle
    #     self.logger.info(f"Ego Drive Callback - Requested Speed: {ego_requested_speed}, Steering Angle: {ego_steer}")

    def state_callback(self, msg):
        """
        string child_frame_id
            pose:
                x, 
                y, 
                z, 
                rotation about X axis, 
                rotation about Y axis, 
                rotation about Z axis
            twist:
                
        """
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