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

/*
 * The robot's drive ax := x
 * 				  up ax := z
 */

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
	//cout<<*msg<<endl;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
	//cout<<*msg<<endl;
}

class Event {
public:
	double duration;
	double linear;
	double angular;

	void Event(double dur, double lin, double, ang)
	{
		this->duration = dur;
		this->linear = lin;
		this->angular = ang;
	}
};


class Scheduler {
private:
	vector<Event> queue;
	Event curr_event;
	ros::Timer timer;
public:
	void pushEvent(double dur, double lin, double ang)
	{
		Event e(dur,lin,ang);
		this->queue.append(e);
	}

	void setNextEvent()
	{
		if (!this->queue.empty())
		{
			this->curr_event = this->queue.pop();
			this->timer = nh.createTimer(ros::Duration(curr_event.duration), this->schedulerCallback);
		}
	}

	void schedulerCallback(const ros::TimerEvent&)
	{
		this->setNextEvent();
	}

	Event& getCurrEvent() {return *this->curr_event};
};


/*
 * From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * Converts a quaternion to (roll pitch yaw) doubles
 */
void toEulerAngle(double q1, double q2, double q3, double q4,
				  double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q4 * q1 + q2 * q3);
	double cosr_cosp = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q4 * q2 - q3 * q1);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q4 * q3 + q1 * q2);
	double cosy_cosp = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
	yaw = atan2(siny_cosp, cosy_cosp);
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
	double q1,q2,q3,q4;
	double roll, pitch, yaw;
	double x, y, z;

	while(ros::ok()){
		
		// The basePose wrt the map frame
		geometry_msgs::StampedTransform map_basePose;
		try {
		  listener.lookupTransform("map", "base_footprint", ros::Time(0), map_basePose);
		} catch(tf::TransformException &exception) {
		  ROS_ERROR("%s", exception.what());
		}

		x = map_basePose.tranform.translation.x;
		y = map_basePose.tranform.translation.y;
		z = map_basePose.tranform.translation.z;
		q1 = map_basePose.tranform.rotation.x;
		q2 = map_basePose.tranform.rotation.y;
		q3 = map_basePose.tranform.rotation.z;
		q4 = map_basePose.tranform.rotation.w;

		toEulerAngle(q1,q2,q3,q4,*roll,*pitch,*yaw);

		cout<<x<<"\t"<<y<<"\t"<<z<<"||"<<roll<<"\t"<<pitch<<"\t"<<yaw<<endl;

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
