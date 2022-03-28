#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <cmath>

#define SCAN_BEAMS 1080
#define MIN_TTC 0.3
#define BIG_TTC 1000.0
#define SCAN_FIELD_OF_VIEW 4.71



class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;

    // Auxiliary variables for monitoring
    double max_decel, min_ttc;


    // Listen for scan messages
    ros::Subscriber scan_sub;
    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Publish braking data
    ros::Publisher brake_pub;
    // Publish bool data
    ros::Publisher bool_pub;
  
    // Auxiliary publishers for monitoring
    ros::Publisher minDist_pub;


    double beam_angle(int beam) {
		return -SCAN_FIELD_OF_VIEW/2 + beam * SCAN_FIELD_OF_VIEW / SCAN_BEAMS;
    }


public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe("/odom", 1, &Safety::odom_callback, this);
        // Start a subscriber to listen to scan messages
        scan_sub = n.subscribe("/scan", 1, &Safety::scan_callback, this);

        // Make a publisher for brakeing messages
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 10);
        // Make a publisher for bool messages
        bool_pub = n.advertise<std_msgs::Bool>("/brake_bool", 10);

        // Make auxiliary publisher for monitoring distance
        minDist_pub = n.advertise<std_msgs::Float32>("/min_dist", 10);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
		double r_dot[SCAN_BEAMS];
		double TTCs[SCAN_BEAMS];
		bool must_brake = false;

        // calculate speed for each beam angle  
		for (int i = 0; i < SCAN_BEAMS; i++)
			r_dot[i] = speed * cos(beam_angle(i))>0?speed * cos(beam_angle(i)) : 0;

		// calculate TTC for each beam
		for (int i = 0; i < SCAN_BEAMS; i++)
			if (r_dot[i] > 0) {
				TTCs[i] = scan_msg->ranges[i] / r_dot[i];
				if (TTCs[i] < MIN_TTC) 
					must_brake = true;
			}
			else 
				TTCs[i] = BIG_TTC;

	
		// Find closest distance
		double mn_dist = 1000.0;
		for (int i = 0; i < SCAN_BEAMS; i++)
			if (scan_msg->ranges[i] < mn_dist)
				mn_dist = scan_msg->ranges[i];

    	// initialize message to be published
    	ackermann_msgs::AckermannDriveStamped ack_msg;
		ack_msg.drive.speed = 0.0;

		std_msgs::Bool braking_msg;
		braking_msg.data = must_brake;
	
		bool_pub.publish(braking_msg);
		if (must_brake) {
        	brake_pub.publish(ack_msg);
		}

		std_msgs::Float32 mn;
		mn.data = mn_dist;
		minDist_pub.publish(mn);
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}
