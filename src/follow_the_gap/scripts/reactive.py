#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker

#RQT reconfigure
from dynamic_reconfigure.server import Server
from follow_the_gap.cfg import gapparamsConfig


LIDAR_SCANS = 1080

INFLATE_RADIUS = lambda d : int(1080 * np.arctan(WIDTH/(2*d)) / 4.71239)


def reconfigure_callback(config, level):
    
    global MOVING_WINDOW   
    global MIN_OBJ_DIST
    global WIDTH 
    rospy.loginfo("""Reconfiugre Request: {moving_window}, {min_obj_dist}, {width}, """.format(**config))
    #print("config.kp:", config.kp, "config.kd:", config.kd)
    
    MOVING_WINDOW = int(config.moving_window)
    MIN_OBJ_DIST = config.min_obj_dist
    WIDTH = config.width

    #print("\nkp:", kp, "kd:", kd)
    return config





def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


class reactive_follow_gap:

    def __init__(self):
        # Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        gaps_topic = '/gaps'
        angle_topic = '/angle'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.gaps_pub = rospy.Publisher(gaps_topic, LaserScan, queue_size=1)
        self.angle_pub = rospy.Publisher(angle_topic, Marker, queue_size=1)


    def vizualize_internals(self, data, angle, vel, gaps):
        data2 = data
        data2.ranges = gaps[0].tolist()
        data2.intensities = gaps[1] 
        self.gaps_pub.publish(data2)

        marker = Marker()
        marker.header.frame_id = "laser"
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        quaternion = get_quaternion_from_euler(0, 0, angle)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = vel
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
        self.angle_pub.publish(marker)
        
    
    def get_intensities(self, closest, a, b, d):
        intensities = [10 for i in range(LIDAR_SCANS)]  # not selected gap
        
        for i in range(a, b+1):  # selected gap
            intensities[i] = 5
            
        if closest - INFLATE_RADIUS(d) > 0:
            s = closest - INFLATE_RADIUS(d)
        else:
            s = 0
            
        if closest + INFLATE_RADIUS(d) < LIDAR_SCANS:
            e = closest + INFLATE_RADIUS(d)
        else:
            e = LIDAR_SCANS - 1
            
        for i in range(s, e+1):  # obstacle area
            intensities[i] = 0
        intensities[s] = 10
            
        return intensities
        

    def find_angle(self, aim):
        right_angle = aim/LIDAR_SCANS * 4.71238898038469
        right_to_middle_angle = 2.356194490192345
        
        return right_angle - right_to_middle_angle


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # Apply running mean
        proc_ranges = np.convolve(ranges, np.ones(MOVING_WINDOW)/MOVING_WINDOW, mode='valid')
        
        # Add missing values from the running mean
        t = proc_ranges[0]
        for i in range( (MOVING_WINDOW-1)//2 ):
            proc_ranges = np.insert(proc_ranges, 0, t)
            
        t = proc_ranges[-1]
        for i in range( (MOVING_WINDOW-1)//2 ):
            proc_ranges= np.append(proc_ranges, t)            
            
        # Reject high values
        for i in range(len(proc_ranges)):
            if proc_ranges[i] > MIN_OBJ_DIST:
                proc_ranges[i] = MIN_OBJ_DIST

        return proc_ranges


    def find_max_gap(self, closest_point, d):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        if closest_point > LIDAR_SCANS/2:
            return [0, closest_point - INFLATE_RADIUS(d)]

        return [closest_point + INFLATE_RADIUS(d), LIDAR_SCANS-1]


    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """

        if end_i < LIDAR_SCANS/2:
            mxi = end_i
            for i in range(end_i, start_i-1, -1):
                if ranges[i] > ranges[mxi]:
                    mxi = i

        elif start_i > LIDAR_SCANS/2:
            mxi = end_i
            for i in range(start_i, end_i+1):
                if ranges[i] > ranges[mxi]:
                    mxi = i

        else:
            mxi1 = LIDAR_SCANS//2
            for i in range(LIDAR_SCANS//2, end_i+1):
                if ranges[i] > ranges[mxi1]:
                    mxi1 = i

            mxi2 = LIDAR_SCANS//2
            for i in range(LIDAR_SCANS//2, start_i-1, -1):
                if ranges[i] > ranges[mxi2]:
                    mxi2 = i

            if ranges[mxi1] == ranges[mxi2]:
                if mxi1 - LIDAR_SCANS/2 > LIDAR_SCANS/2 - mxi2:
                    mxi = mxi2
                else:
                    mxi = mxi1
            elif ranges[mxi1] > ranges[mxi2]:
                mxi = mxi1
            else:
                mxi = mxi2

        return mxi
        
        #return (end_i + start_i)//2
        
        #return np.argmax(ranges[start_i:end_i+1]) + start_i


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        # Find closest point to LiDAR
        closest = np.argmin(proc_ranges)

        # Find max length gap
        a, b = self.find_max_gap(closest, proc_ranges[closest])
        colors = self.get_intensities(closest, a, b, proc_ranges[closest])

        # Find the best point in the gap
        aim = self.find_best_point(a, b, proc_ranges)

        # Find angle
        angle = self.find_angle(aim)

        # Speed Controller
        if abs(angle) < 0.174533:
            vel = 2
        elif abs(angle) < 0.349066:
            vel = 2
        else:
            vel = 0.5

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = vel # --------------------------------------------------------
        self.drive_pub.publish(drive_msg)
        
        # Publish Vizualizations
        self.vizualize_internals(data, angle, vel, [proc_ranges, colors])


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    srv = Server(gapparamsConfig, reconfigure_callback)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
