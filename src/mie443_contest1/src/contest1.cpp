#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


#include <stdio.h>
#include <cmath>

#include <ctime>

using namespace std;


const double pi = 3.14159;




double angular;
double linear;


const double max_lin_speed = 0.25;
const double max_ang_speed = pi/5;



class positionVector
{
public:
	double x;
	double y;
	double yaw;

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
		x = msg->pose.pose.position.x;
		y = msg->pose.pose.position.y;
		yaw = tf::getYaw(msg->pose.pose.orientation);
	}
};



class Timer
{
    private:
    clock_t startTime;

    public:
    Timer ()
    {
        startTime = std::clock();
    }

    double getElapsedTime()
    {
        return (double) (std::clock() - startTime) / CLOCKS_PER_SEC;
    }
};





// used by planner to issues commands into the main while loop
class command{
	public:
		double angularSpeed;
		double linearSpeed;
		double duration;
		double start_time;
		bool turn_avoid; //true if the last command was a turn to get away from a wall
		bool override_current;


	command(double ang, double lin, double dur, bool avoid, bool override){
		this->duration = dur;
		this->angularSpeed = ang;
		this->linearSpeed = lin;
		this->start_time = ros::Time::now().toSec();
		this->turn_avoid = avoid;
		this->override_current = override;
	}
	
};


// randomly pick either 1 or -1
int randomSign(){
	return 2*(rand()%2)-1;
}




void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
}








const int laserSize = 639;

const int angleSectors = 5; // try to divide cone into 3 approximately equal sized angle sectors
double sectorRange[angleSectors];
int sectorIndicies[angleSectors];
int del_sector = laserSize/angleSectors;
// Increasing indicies means going from right to left
// if looking at it from a top down view

double sectorWeights[angleSectors];




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your 


	// return the average laser range in each sector 
	for(int i=0; i<angleSectors; i++){
		double sum = 0;
		for(int j=sectorIndicies[i]; j<sectorIndicies[i]+del_sector; j++){

			if(!isnan(msg->ranges[j])){
				sum = sum + msg->ranges[j];
			}
		}
		sectorRange[i] = sum/del_sector;
	}
	
	
}











// simple plan based on sector ranges
double turnState[angleSectors];
double linState[angleSectors];

// below wallThreshold is critical, move away
// beyond exploreThreshold means that that side is pretty open, take on an exploratory behaviour

command avoidancePlanner(double wallThreshold, double exploreThreshold, command prevCommand){


	command plannedCommaned = command(0,0,0,false,false);



	// stats about the sectors
	double max_range = 0; //can see how far one way is one next obstacle
	double min_range = sectorRange[1]; //can use to see if atleast one section is at a wall
	double avgRange = 0;  //average range value

	double turnNumber = 0;



	for(int i=0; i<angleSectors; i++){
		if(sectorRange[i]<wallThreshold){

			//set to negative number if near a wall
			turnState[i] = -10; 
          
		}
		else{
			turnState[i] = sectorRange[i];
		}

		if(sectorRange[i] < min_range){ min_range = sectorRange[i]; }
		if(sectorRange[i] > max_range){ max_range = sectorRange[i]; }

		avgRange = avgRange + sectorRange[i];
	}

	avgRange = avgRange / angleSectors;
 

	for(int i=0; i<angleSectors; i++){ turnNumber = turnNumber + turnState[i]*sectorWeights[i]; }
	turnNumber = -1*turnNumber / angleSectors; //switch sign of turnNumber around
	if(turnNumber > 5){ turnNumber = 5; }
	if(turnNumber < -5){ turnNumber = -5; }




	// if last command was a turn to turn away from a wall
	if(prevCommand.turn_avoid == true){

		//go straight for a bit after turning away from the wall
		return command(0,0.2,1,false,false);
		goto endPlanning;

		//printf("going straight after turning from wall\n");
	}



	// this is true if the full 60 degree cone is within a wall - F
	// since this command has override, only do it if the previous command
	// was not a turn avoid command
	if(max_range < wallThreshold &&  prevCommand.turn_avoid == false){

		//if left slightly more range than right, turn left90
		if(sectorRange[4]/sectorRange[0] > 1.2 ){
			plannedCommaned = command(max_ang_speed, 0, (3*pi/4)/max_ang_speed, true, true);
			goto endPlanning;
		}
		//if right slightly more range than left, turn right 90
		else if(sectorRange[4]/sectorRange[0] < 1/1.2){
			plannedCommaned = command(-max_ang_speed, 0, (3*pi/4)/max_ang_speed, true, true);
			goto endPlanning;
		}
		//if mostly straight on the wall, then turn in a random direction
		else{
			plannedCommaned = command(randomSign()*max_ang_speed, 0, (pi)/max_ang_speed, true, true);
			goto endPlanning;
		}

	} 



	//if right corner is near wall, but left corner not near wall
	if(sectorRange[0] < wallThreshold && sectorRange[4] > wallThreshold  &&  prevCommand.turn_avoid == false){
		
		if(sectorRange[1] < wallThreshold){
			// if right also near wall, then left turn by more, say ~45 degrees
			plannedCommaned = command(max_ang_speed, 0, (pi/4)/max_ang_speed, true, true);
			goto endPlanning;
		}
		else{
			// if only right corner near wall, then just turn left by less, ~ 22 degrees
			plannedCommaned = command(max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, true);
			goto endPlanning;
		}
	}

	//if left corner is near wall, but right corner not near wall
	if(sectorRange[4] < wallThreshold && sectorRange[0] > wallThreshold  &&  prevCommand.turn_avoid == false){
		
		if(sectorRange[3] < wallThreshold){
			// if left also near wall, then right turn by more, say ~45 degrees
			plannedCommaned = command(-max_ang_speed, 0, (pi/4)/max_ang_speed, true, true);
			goto endPlanning;			
		}
		else{
			// if only right corner near wall, then just turn right by less, ~ 22 degrees
			plannedCommaned = command(-max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, true);
			goto endPlanning;
		}
	}


	//--- If passed checks for avoiding obstacles, then do more exploratory behaviour ---//

	//relative measure of which side has more range
	//more positive -> right side comparatively more range than left
	//more negative -> left side comparatively more range than right
	// ~0 -> both sides about the same





	//explore via turnNumber
	plannedCommaned = command((turnNumber/5)*1*max_ang_speed, max_lin_speed, 0, false, false);


	endPlanning:


	//plannedCommaned =  command(randomSign()*max_ang_speed, 0, (pi/4)/max_ang_speed, true, false);

	return plannedCommaned;
}











int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);



	positionVector currPosition;

	ros::Subscriber odom = nh.subscribe("/odom", 1, &positionVector::odomCallback, &currPosition);



	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	Timer timer = Timer();


	// for getting indices of scan sectors
	for(int s=0; s < angleSectors; s++){
		sectorIndicies[s] = del_sector * s;
	}

	// setting up sectorWeights
	for(int s=0; s < angleSectors; s++){
		sectorWeights[s] = (angleSectors - s) - ceil((double)angleSectors/2);
	}




//------- learning how to use time in ROS ------------//

	/*ros::Time prevTime = ros::Time::now();
	ros::Time currTime;
	double elapsedLoopTime;*/

//----------------------------------------------------//



	command currCommand = command(0,0,0,false,false);
	command turnCircle = command(0,0,0,false,false);
	int countTimer=5;
	bool turn360=false;

	while(ros::ok() /*&& timer.getElapsedTime() < 480*/){

		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		//fill with your code



	//-------- learning how to use time -------------------------------//

/*		currTime = ros::Time::now();
		elapsedLoopTime = currTime.toSec() - prevTime.toSec();
		prevTime = currTime;

		ROS_INFO("elapsedLoopTime:%f",elapsedLoopTime);


		ros::Duration(0.69).sleep();*/
	//-----------------------------------------------------------------//

		if (timer.getElapsedTime()>countTimer){
			printf("timeElapsed: %f \n", timer.getElapsedTime());
			turn360=true;
			countTimer+= 2*pi/max_ang_speed + 10;
		}


		// threshold parameters for planner
		double wallThresh = 0.4 , exploreThresh = 3;



		// get a new command each spin
		command newCommand = avoidancePlanner(wallThresh, exploreThresh, currCommand);
		if (turn360==true){
				turnCircle = command(max_ang_speed, 0, 2*pi/max_ang_speed, true, true);
				turn360=false;
				
				//vel.angular.z = turnCircle.angularSpeed;
  				//vel.linear.x = turnCircle.linearSpeed;
  				//vel_pub.publish(vel);
  				printf("turning");
  				newCommand = turnCircle;
			}

		// if the current expires, set currCommand as the newCommand
		if(ros::Time::now().toSec() - currCommand.start_time > currCommand.duration || newCommand.override_current == true){

			currCommand = newCommand;
			printf("Got a new command - start time: %f - turn avoid? %i \n",currCommand.start_time,currCommand.turn_avoid);
			printf("angular: %f   linear: %f \n",currCommand.angularSpeed, currCommand.linearSpeed);


		}

		


  		vel.angular.z = currCommand.angularSpeed;
  		vel.linear.x = currCommand.linearSpeed;

  		vel_pub.publish(vel);


  		//ROS_INFO("X:%f Y:%f Yaw:%f", currPosition.x, currPosition.y,currPosition.yaw);
  		//ROS_INFO("LL:%f   L: %f   C:%f   R:%f  RR:%f   TN:%f",sectorRange[4],sectorRange[3], sectorRange[2],sectorRange[1],sectorRange[0],tempTurnNumber);
	}

	return 0;
}




























//stuff here



/*float obstacleVisionCone(vector<float> scanArray, int laserSize, double obstacleThreshold){


	float sum = 0;
	for(int i = 0; i < laserSize; i++){

		if(isnan(scanArray[i])){
			printf("nan at index %i \n",i);
		}
		else{
			sum = sum + scanArray[i];
			printf("%f \n",sum);
			printf("%f \n",scanArray[i]);
		}
	}

	return sum/laserranges;


	int * obstacleArray;

}*/
