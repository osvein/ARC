#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def beam_angle(beam):
  return -3.14159274101 + beam * 6.28318548202 / scan_beams

 



# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        #Publishers
        self.drive_pub = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        self.boolean_pub = rospy.Publisher('brake_bool', Bool, queue_size=10)

        #Subscribers
        odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        scan_sub =  rospy.Subscriber('scan', LaserScan, self.scan_callback)
        
        self.speed = 0
        self.brake_boolean = True
        self.scan_beams = rospy.get_param('scan_beams')
        self.max_decel = rospy.get_param('max_decel')
        self.min_ttc = rospy.get_param('min_ttc')


        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
       

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        for x in range(self.scan_beams):
            r_dot
        

        # TODO: publish brake message and publish controller bool

        #Drive Publisher
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = 'your_frame_here'
        ack_msg.drive.steering_angle = 0
        ack_msg.drive.speed = self.speed
        self.drive_pub.publish(ack_msg)

        #Brake Boolean Publisher
        self.boolean_pub.publish(self.brake_boolean)



def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
