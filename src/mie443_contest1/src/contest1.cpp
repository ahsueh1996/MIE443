#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

using namespace std;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
	cout<<*msg<<endl;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
	//cout<<*msg<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	// Get bumper and kinect laser "interrupts"
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	// Use the tf channel for base_footprint pose estimates
	tf::TransformListener listener;

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	double angular = 0.0;
	double linear = 0.0;
	geometry_msgs::Twist vel;

	while(ros::ok()){
		tf::StampedTransform transform;
		listener.lookupTransform("/base_footprint","/map",ros::Time(0), transform);

		

		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
