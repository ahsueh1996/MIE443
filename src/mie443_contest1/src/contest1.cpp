/*
 * MIE443 contest 1
 * Authors: Richard, Albert, Scott, Simon, Mike
 * Jan. 2019
 */

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

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//cout<<*msg<<endl;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//cout<<*msg<<endl;
}


class Event {
	/*
	 * This class describes executable actions by the robot.
	 * So far, actions have to do with how fast and for how long (zero-order-hold)
	 */
	public:
		double duration;		// [sec]
		double linear;			// [m/s]    around the robot x axis
		double angular;			// [rad/s]  about the robot z axis

		Event(double dur, double lin, double ang)
		{
			this->duration = dur;
			this->linear = lin;
			this->angular = ang;
		}
};


class Scheduler {
	/*
	 * This class describes a time line of Events implemented by a queue.
	 * Users push Events to the queue and when appropriate, the scheduler will
	 * automatically executing the next Event. Given that each Event has a duration,
	 * the automatic scheduling is done via a timer that sets off a callback to
	 * set the robot's velocity according to the next event.
	 *
	 * Ex.
	 * 		Q = <(spd 1 for 3s), (rot pi/6 for 3s), (spd -1 for 1s)>
	 *  	Allowing the scheduler to complete this queue would result in the robot
	 *  	driving forward 3 meters, rotating clockwise 90deg then backing up 1 meter
	 *
	 * If the queue is empty the default event is to do nothing.
	 */
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

		/*
		 * Push event creates a new event and enqueues it.
		 */
		void pushEvent(double dur, double lin, double ang)
		{
			Event e = Event(dur,lin,ang);
			this->queue.insert(this->queue.begin(),e);
		}

		/*
		 * Set next event can be called to manually move on to the next event in the queue.
		 * It also sets the timer to call the next next event.
		 */
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

		/*
		 * This call back print out a debug message and moves uses set next event to execute the next event.
		 */
		void schedulerCallback(const ros::TimerEvent&)
		{
			cout<<this->curr_event.duration<<"\t"<<this->curr_event.linear<<"\t"<<this->curr_event.angular<<endl;		
			this->setNextEvent();
		}

		/*
		 * Use this function to get the event that should be executed.
		 */
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

	// Here is an example of how to initialize the scheduler
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

		// This is a debug message to demonstrate the accessed pose information of the robot
		// cout<<x<<"\t"<<y<<"||"<<yaw<<endl;

		//.....**Resolve one publish and subscription cycle DO NOT TOUCH**.......
		ros::spinOnce();

		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();

		//.....**Action routine DO NOT TOUCH (probably)**.......
		Event event = scheduler.getCurrEvent();
  		vel.angular.z = event.angular;
  		vel.linear.x = event.linear;
  		vel_pub.publish(vel);
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
