#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <string>
#include <fstream>

// added by mike
#include <std_msgs/Bool.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>

using namespace std;

inline bool exist_file (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

const double PI = 3.14;

//bumper global variable
bool bumper_left = false; 
bool bumper_center = false;
bool bumper_right = false;

bool gone = false;
bool picked_up = false;
double gone_sum = 0.0;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
    if(msg.bumper == 0){
    	bumper_left = !bumper_left;
    	cout << "left" << endl;
    }
    if(msg.bumper == 1){
    	bumper_center = !bumper_center;
    	cout << "center" << endl;
    }
    if(msg.bumper == 2){
    	bumper_right = !bumper_right;
    	cout << "right" << endl;
    }


}

void personCB(const std_msgs::Bool msg){

	if(msg.data == false){
		//cout << "no person found" << endl;
		gone = true;
	}
	if(msg.data == true){
		//cout << "See a person !!!!!!!" <<endl;
		gone = false;
	}
}

void pickedupCB(const kobuki_msgs::WheelDropEvent msg) {
	if (msg.state == 1) {
		//cout << "picked up!!" << endl;
		picked_up = true;
	}
	if (msg.state == 0) {
		//cout << "dropped" << endl;
		picked_up = false;
	}
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
	ros::Subscriber drop_sens = nh.subscribe("mobile_base/events/wheel_drop", 10, &pickedupCB);

	ros::Subscriber person_detection = nh.subscribe("turtlebot_follower/detect_person",10, &personCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);


	const int NONE = -1;
	const int NEUTRAL = 0;
	const int FEAR = 1;
	const int RAGE = 2;
	const int EXCITE = 3;
	const int DISGUST= 4;

	int state = NEUTRAL;
	int onEnter = NONE;
	int onExit = NONE;


	int bump_count = 0;
	const int BUMP_THRESH = 2;

	
	const double FOLLOW_FORWARD_LIMIT = 0.25;
	const double FOLLOW_BACKWARD_LIMIT = 0.0;


	int fear_spin_direction; //1 turn left , -1 turn right
	double fear_begin_spin_time;
	const double FEAR_SPIN_SPEED = 0.6;
	const double FEAR_SPIN_ANGLE = 0.6;



	double rage_begin_time;
	const double RAGE_SPIN_SPEED = 2.0;
	const double RAGE_SPIN_ANGLE = 2*PI;




	double angular = 0.0;
	double linear = 0.0;
	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;




	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();




	cout << "starting state: NEUTRAL" << endl;

	//show neutral image




	while(ros::ok()){
		 ros::spinOnce();

		//immediately trigger bumper condition if hit something
		if ((bumper_center || bumper_right || bumper_left) && !picked_up){ 
			follow_cmd.linear.x = -0.3;
			follow_cmd.angular.z = 0;
			vel_pub.publish(follow_cmd);
			ros::Duration(1).sleep();
			follow_cmd.linear.x = 0;
			follow_cmd.angular.z = 0;
			vel_pub.publish(follow_cmd);
		}
		 ros::Duration(0.1).sleep();

		// //.....**E-STOP DO NOT TOUCH**.......
		// //eStop.block();
		// //...................................


		 // test play sound

		 // sc.playWave(path_to_sounds + "sound.wav");
		 // ros::Duration(5).sleep();

		 // need to sleep enough for the sound file to finish playing


		switch(onExit) {
			/*
			 * Insert code that get's executed one time upon entering a state
			 */
			default:
				break;
		} // onExit
		onExit = -1;


		switch (state) {
			/*
			 * Insert code that get's run every cycle spent in the state
			 */
			case NEUTRAL:
			{
				// ros::spinOnce(); Assumed to be done perhaps, or not. either method is okay
				if (gone){
					gone_sum += 1;
					if (gone_sum > 5)
					{
						onEnter = FEAR;
						onExit = NEUTRAL;	
					}
				} else {gone_sum *= 0.9;}
				//cout<< gone_sum << endl;

				if ((bumper_center || bumper_right || bumper_left) && !picked_up){ 
					bump_count = bump_count + 1;
					cout << "bump count " << bump_count << endl;
						if (bump_count > BUMP_THRESH){
							onEnter = RAGE;
							onExit = NEUTRAL;
						}
					
				}

				if (picked_up){
					onEnter = EXCITE;
					onExit = NEUTRAL;
				}

				double vel_x = follow_cmd.linear.x;

				follow_cmd.linear.x = min(FOLLOW_FORWARD_LIMIT, max(FOLLOW_BACKWARD_LIMIT,vel_x));

				vel_pub.publish(follow_cmd);

				break;
			}

			case FEAR:
			{

				if (!gone){
					onEnter = NEUTRAL;
					onExit = FEAR;
				}

				else{ //turn back and forth looking for the person
					if (ros::Time::now().toSec() >= (fear_begin_spin_time + FEAR_SPIN_ANGLE/FEAR_SPIN_SPEED)){
						fear_spin_direction = -1 * fear_spin_direction;
						fear_begin_spin_time = ros::Time::now().toSec();

						// not sure necessary
						vel.linear.x = 0.0;
						vel.angular.z = fear_spin_direction * FEAR_SPIN_SPEED;
						vel_pub.publish(vel);
					}
					else{
						vel.linear.x = 0.0;
						vel.angular.z = fear_spin_direction * FEAR_SPIN_SPEED;
						vel_pub.publish(vel);
					}
				}


				break;
			}


			case RAGE:
			{
				if (ros::Time::now().toSec() >= (rage_begin_time + RAGE_SPIN_ANGLE/RAGE_SPIN_SPEED) )
				{

					//show transition picture of calming down?

					onEnter = NEUTRAL;
					onExit = RAGE;
				}
				else{ //not sure if necessary
					vel.linear.x = 0.0;
					vel.angular.z = RAGE_SPIN_SPEED;
					vel_pub.publish(vel);
				}
				break;
			}



			case EXCITE:
			{
				if (!picked_up){
					onEnter = NEUTRAL;
					onExit = EXCITE;
				}
				else{
					if (bumper_left){
						onEnter = DISGUST;
						onExit = EXCITE;
					}
				}
				break;
			}



			case DISGUST:
			{

				//forward velocity thresholding

				double vel_x = follow_cmd.linear.x;
				follow_cmd.linear.x = min(FOLLOW_FORWARD_LIMIT, vel_x);
				vel_pub.publish(follow_cmd);


				// https://askubuntu.com/questions/37767/how-to-access-a-usb-flash-drive-from-the-terminal
				bool sorry = exist_file("/media/turtlebot/F857-6592/imsorry.txt");
		 		cout << "Sorry?: " << sorry << endl;
				if (sorry){

					//show accepting apology picture
					//play sound like "oh bro i forgive you"

					onEnter = NEUTRAL;
					onExit = DISGUST;
				}
				break;
			}


			default:
				break;



		}	// Normal cycle of state



		switch (onEnter) {
			/*
			 * Insert code that gets executed once upon exiting a state
			 */
			case NEUTRAL:
			{
				state = NEUTRAL;
				onEnter = NONE;
				gone_sum = 0;
				cout << "Enter NEUTRAL ======" << endl;
				// show neutral image
				break;
			}
			case FEAR:
			{
				state = FEAR;
				onEnter = NONE;
				bump_count = 0;	
				gone_sum = 0.0;

				fear_spin_direction = 1;
				fear_begin_spin_time = ros::Time::now().toSec();

				vel.linear.x = 0.0;
				vel.angular.z = fear_spin_direction * FEAR_SPIN_SPEED;
				vel_pub.publish(vel);


				cout << "Enter FEAR ======" << endl;
				// show fear picture
				// play fear sound
				break;
			}
			case RAGE:
			{
				state = RAGE;
				onEnter = NONE;
				bump_count = 0;
				cout << "Enter RAGE ======" << endl;

				//make robot spin in place when enraged
				rage_begin_time = ros::Time::now().toSec();

				vel.linear.x = 0.0;
				vel.angular.z = RAGE_SPIN_SPEED;

				vel_pub.publish(vel);

				
				// show rage picture
				// play rage sound
				break;
			}
			case EXCITE:
			{
				state = EXCITE;
				onEnter = NONE;
				bump_count = 0;	
				cout << "Enter EXCITE ======" << endl;
				// show excite picture
				// show excite sound
				break;
			}
			case DISGUST:
			{
				state = DISGUST;
				onEnter = NONE;
				cout << "Enter DISGUST ======" << endl;
				// show resent picture
				// play sound like "oh don't touch me like that"
				break;
			}
			default:
				break;
		}	// onEnter


	}	// while loop

	return 0;
}
