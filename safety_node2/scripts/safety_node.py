#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from math import cos


SCAN_BEAMS = 1080
MIN_TTC = 1
BIG_TTC = 1000
SCAN_FIELD_OF_VIEW = 4.71


class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        self.speed = 0
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        #Publishers
        self.drive_pub = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        self.boolean_pub = rospy.Publisher('/brake_bool', Bool, queue_size=10)

        #Subscribers
        odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        scan_sub =  rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        

        print('hey there-----1------------------------------------------------------------------------------------------')

	
    @staticmethod
    def _beam_angle(beam):
        return -SCAN_FIELD_OF_VIEW/2 + beam * SCAN_FIELD_OF_VIEW / SCAN_BEAMS


    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x
        

    def scan_callback(self, scan_msg):
        must_brake = False

        # calculate speed for each beam angle
        r_dot = []
        for i in range(SCAN_BEAMS):
            temp = self.speed * cos(Safety._beam_angle(i))
            if temp > 0:
                r_dot.append(temp)
            else:
                r_dot.append(0)
                
        print(r_dot[540])

        # calculate TTC for each beam
        TTCs = []
        print(scan_msg.ranges[540])
        print("-------")
        for i in range(SCAN_BEAMS):
            if r_dot[i] > 0:
                TTCs.append(scan_msg.ranges[i] / r_dot[i])
                if TTCs[i] < MIN_TTC:
                    must_brake = True
                    #print(i)
                    #print(scan_msg.ranges[i])
                    #print( r_dot[i])
                    #print("----------")
            else:
                TTCs.append(BIG_TTC)

        #Drive Publisher
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = 0

        braking_msg = Bool()
        braking_msg.data = must_brake

        self.boolean_pub.publish(braking_msg)
        if must_brake == True:
            self.drive_pub.publish(ack_msg)


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
