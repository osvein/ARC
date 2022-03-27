#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries
// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "std_msgs/Bool.h"
#include <cmath>

#define SCAN_BEAMS 1080
#define MAX_DECEL 8.26
#define MIN_TTC 1.0


class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    int scan_beams;
    double max_decel, min_ttc;


    // TODO: create ROS subscribers and publishers
    // Listen for scan messages
    ros::Subscriber scan_sub;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Publish braking data
    ros::Publisher brake_pub;

    // Publish bool data
    ros::Publisher bool_pub;

    double beam_angle(int beam) {
	return -3.14159274101 + beam * 6.28318548202 / scan_beams;
    }

	

public:
    Safety() {
        n = ros::NodeHandle("~");
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
	
	//get params
	n.getParam("scan_beams", scan_beams);
	n.getParam("max_decel", max_decel);
	n.getParam("min_ttc", min_ttc);

	std::cout << min_ttc << " <----------------------------dsafdsf-sad-f-adsf-ads-f-a" << std::endl;

        // TODO: create ROS subscribers and publishers
        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe("/odom", 1, &Safety::odom_callback, this);

        // Start a subscriber to listen to scan messages
        scan_sub = n.subscribe("/scan", 1, &Safety::scan_callback, this);

        // Make a publisher for brakeing messages
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 10);

        // Make a publisher for bool messages
        bool_pub = n.advertise<std_msgs::Bool>("/brake_bool", 10);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
	double r_dot[scan_beams];
	double TTCs[scan_beams];
	bool must_brake = false;
	
	for (int i = 0; i < scan_beams; i++)
		r_dot[i] = speed * cos(beam_angle(i))>0?speed * cos(beam_angle(i)) : 0;

	for (int i = 0; i < scan_beams; i++)
		if (r_dot[i] > 0) {
			TTCs[i] = scan_msg->ranges[i] / r_dot[i];
			if (TTCs[i] < min_ttc) 
				must_brake = true;
		}
		else 
			TTCs[i] = 1000.0;


	/*std::cout << std::endl << " ------------------------------------------------------------------- " << std::endl;
	for (int i = 0; i < scan_beams; i++)
		std::cout << TTCs[i] << "  ";
	std::cout << std::endl << " ------------------------------------------------------------------- " << std::endl;
	std::cout << speed << std::endl;
	std::cout << scan_msg->ranges[540] << std::endl;
	std::cout << TTCs[540] << std::endl;*/



        // TODO: publish drive/brake message
        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
	drive_st_msg.drive.speed = 0.0;

	std_msgs::Bool braking;
	braking.data = true;
	
	if (must_brake) {
        // publish AckermannDriveStamped message to drive topic
        brake_pub.publish(drive_st_msg);
	bool_pub.publish(braking);}
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}
