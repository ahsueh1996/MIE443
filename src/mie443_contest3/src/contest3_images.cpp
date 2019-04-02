#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>

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
	string path_to_images = ros::package::getPath("mie443_contest3") + "/images/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	//ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;


	// List of emotions
	string emotions[] = {"disgust", "fear", "excited", "rage"}; 

	std::vector<Mat> images;
	Mat image;

	// Grab the images from the images folder
	for (int i=0;i<6;i++)
	{
		image = imread(path_to_images + emotions[i] + ".jpg",	IMREAD_COLOR);
		if(!image.data )
		{
			printf( " No image data \n " );
			continue;
		}
		
		images.push_back(image);
	}
	
	namedWindow("image", CV_WINDOW_NORMAL);
	// Full screen
	//setWindowProperty("image", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	imshow("image", images[0]); 
	//ros::Duration(3).sleep();
	//imshow("image", images[1]); 
	

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		//Disgust sound and image for 3 seconds
		  
		//sc.playWave(path_to_sounds + emotions[0] + ".wav");
		//waitKey(3000);

		//Fear sound and image for 3 seconds 
		//imshow("fear", images[1]);
		//sc.playWave(path_to_sounds + emotions[1] + ".wav");
		//waitKey(3000);


	}

	return 0;
}
