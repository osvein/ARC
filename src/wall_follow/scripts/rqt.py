#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#RQT Reconfigure

from dynamic_reconfigure.server import Server
from controller_config.cfg import controlConfig



#PID CONTROL PARAMS
kp = -1  #TODO
kd = -0.001  #TODO
ki = -0.005  #TODO
servo_offset = 0.0
prev_error = 0.0
prev_time = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55  # -------------------------------------------------------------------------- 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
MAX_RANGE = 40.0

def reconfigure_callback(config, level):
    
    rospy.loginfo("""Reconfiugre Request: {}, {}, {}, """.format(kp, kd, ki))
    return config


class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        global prev_time
        prev_time = rospy.get_time()

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc. ------------------------------------------------------ what does this mean??
        # ----------------------------------------------------------------------------------------- maybe throw an axception
        beam = int(len(data.ranges) * (angle + 45) / ANGLE_RANGE)
        if not np.isinf(data.ranges[beam]):
            return data.ranges[beam]
        return MAX_RANGE

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global prev_time
        global kp
        global ki
        global kd

        angle = 0.0
        vel = 0.0
        time_interval = rospy.get_time() - prev_time

        # P - Controller
        angle += kp * error

        # PD - Controller
        angle += kd * (error - prev_error) / time_interval  # ---------------------------------- might need to dived by time

        # PID - Controller
        angle += ki * integral
        integral += error * time_interval  # --------------------------------------------------- might need to dived by time
        print(angle)
        print("----------")

        # prev values
        prev_error = error
        prev_time = rospy.get_time()
    
        # Speed Controller
        if abs(angle) < 0.174533:
            vel = 1.5
        elif abs(angle) < 0.349066:
            vel = 1.0
        else:
            vel = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = vel
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        theta = 55
        theta_rad = math.radians(theta)

        a = self.getRange(data, 180 - theta)	
        b = self.getRange(data, 180)

        alpha = math.atan2(a * math.cos(theta_rad) - b, a * math.sin(theta_rad))

        wall_distance = b * math.cos(alpha) + CAR_LENGTH * math.sin(alpha)
        print(wall_distance)
	
        return leftDist -  wall_distance

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    srv = Server(controlConfig, reconfigure_callback)
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
    