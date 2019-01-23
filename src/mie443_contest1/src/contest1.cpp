#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <iostream>
#include <chrono>

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

		Event(double dur, double lin, double ang)
		{
			this->duration = dur;
			this->linear = lin;
			this->angular = ang;
		}
};


class Scheduler {
	private:
		vector<Event> queue;
		Event default_event = Event(0,0,0);
		Event curr_event = Event(0,0,0);
		ros::Timer timer;
		ros::NodeHandle* nh;
	public:
		Scheduler(ros::NodeHandle* nh_)
		{
			this->nh = nh_;
		}

		void pushEvent(double dur, double lin, double ang)
		{
			Event e = Event(dur,lin,ang);
			this->queue.insert(this->queue.begin(),e);
		}

		void setNextEvent()
		{
			if (!this->queue.empty())
			{
				this->curr_event = this->queue.back();
				this->queue.pop_back();
				this->timer = this->nh->createTimer(ros::Duration(this->curr_event.duration), &Scheduler::schedulerCallback, this);
			}
			else
			{
				this->curr_event = this->default_event;
			}
		}

		void schedulerCallback(const ros::TimerEvent&)
		{
			cout<<this->curr_event.duration<<"\t"<<this->curr_event.linear<<"\t"<<this->curr_event.angular<<endl;		
			this->setNextEvent();
		}

		Event getCurrEvent()
		{
			return this->curr_event;
		}
};



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
	double yaw;
	double x, y, z;
	Scheduler scheduler(&nh);
	scheduler.pushEvent(3,1,2);
	scheduler.pushEvent(10,2,3);
	scheduler.pushEvent(1,3,4);
	scheduler.setNextEvent();

	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now(); /* start timer */
    uint64_t secondsElapsed = 0; // the timer just started, so we know it is less than 480, no need to check.

	while(ros::ok() && secondsElapsed <= 480)
    	{
		
		// The basePose wrt the map frame
		tf::StampedTransform map_basePose;
		try {
			ros::Time now = ros::Time(0);
			listener.waitForTransform("map", "base_footprint", now, ros::Duration(3.0));
		  	listener.lookupTransform("map", "base_footprint", now, map_basePose);
		} catch(tf::TransformException &exception) {
		  	ROS_INFO("%s", exception.what());
		}

		x = map_basePose.getOrigin().x();
		y = map_basePose.getOrigin().y();
		yaw = tf::getYaw(map_basePose.getRotation());

		// cout<<x<<"\t"<<y<<"||"<<yaw<<endl;

		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
		// Event event = scheduler.getCurrEvent();

  		vel.angular.z = angular;//event.angular;
  		vel.linear.x = linear;//event.linear;

  		vel_pub.publish(vel);

        // The last thing to do is to update the timer.
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
