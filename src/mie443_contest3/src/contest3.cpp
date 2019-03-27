#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <string>
#include <fstream>

inline bool exist_file (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);


	int NONE = -1;
	int NEUTRAL = 0;
	int FEAR = 1;
	int RAGE = 2;
	int EXCITE = 3;
	int RESENT= 4;


	int state = NEUTRAL;
	int onEnter = NONE;
	int onExit = NONE;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		 ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		switch(onExit) {

			/*
			 * Insert code that get's executed one time upon entering a state
			 */
			case NEUTRAL:
				break;
			default:
				break;
		} // onEnter
		onExit = -1;
		switch (state) {
			/*
			 * Insert code that get's run every cycle spent in the state
			 */
			case NEUTRAL:
				// ros::spinOnce(); Assume to be done perhaps, or not. either method is okay
				gone = true if blah
				bumped = true if blah or via callback
				if (gone)
					onEnter = FEAR;
					onExit = NEUTRAL;
				if (bumped)
					bump_count += 1;
				else
					bump_count = 0;
				if (bump_count > n)
					onEnter = RAGE;
					onExit = NEUTRAL;
				if (picked_up)
					onEnter = EXCITE;
					onExit = NEUTRAL;
				break;
			case FEAR:
				if (back)
					onEnter = NEUTRAL;
					onExit = FEAR;
				break;
			case RAGE:
				if (free)
					onEnter = NEUTRAL;
					onExit = RAGE;
				break;
			case EXCITE:
				if (bump_L)
					onEnter = RESENT;
					onExit = EXCITE;
				break;
			case RESENT:
				// https://askubuntu.com/questions/37767/how-to-access-a-usb-flash-drive-from-the-terminal
				bool sorry = exist_file("/dev/sdb1/mie443imsorry.txt")
				if (sorry)
					onEnter = NEUTRAL;
					onExit = RESENT;
				break;
			default:
				break;
		}	// Normal cycle of state
		switch (onEnter) {
			/*
			 * Insert code that gets executed once upon exiting a state
			 */
			case NEUTRAL:
				state = NEUTRAL;
				break;
			case FEAR:
				state = FEAR;
				break;
			case RAGE:
				state = RAGE;
				break;
			case EXCITE:
				state = EXCITE;
				break;
			case RESENT:
				state = RESENT;
				break;
			default:
				break;
		}	// onEnter
	}	// while loop

	return 0;
}
