#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// to get info from map
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>



#include <stdio.h>
#include <cmath>

#include <iostream>
#include <chrono>

#include <ctime>

#include <string>

using namespace std;


const double pi = 3.14159;




double angular;
double linear;


const double max_lin_speed = 0.25;
const double max_ang_speed = pi/5;


vector<signed char> map_data;
double resolution;
int width, height;
double originX, originY;





double bot_x;
double bot_y;
double bot_yaw;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	bot_x = msg->pose.pose.position.x;
	bot_y = msg->pose.pose.position.y;
	bot_yaw = tf::getYaw(msg->pose.pose.orientation);
}





// used by planner to issues commands into the main while loop
class command{
	public:
		double angularSpeed;
		double linearSpeed;
		double duration;
		double start_time;
		bool turn_avoid; //true if the last command was a turn to get away from a wall
		int type; //basically a flag for if the request command is critical for avoidance
	


	command(double ang, double lin, double dur, bool avoid, int command_type){
		this->duration = dur;
		this->angularSpeed = ang;
		this->linearSpeed = lin;
		this->start_time = ros::Time::now().toSec();
		this->turn_avoid = avoid;
		this->type = command_type;
	}
	
};



class bumperStates{
	// 0 is left      1 is center     2 is right
	public:
		bool left;
		bool center;
		bool right;

	void bumperCallback(const kobuki_msgs::BumperEvent msg){
		if(msg.bumper == 0){
			this->left = !this->left;		
		}
		else if(msg.bumper == 1){
			this->center = !this->center;
		}
		else if(msg.bumper == 2){
			this->right = !this->right;
		}
	}

	bool isBumped(){
		return left || center || right;
	}
};





// get squared euclidean distance between point a and b
// do not square root this distance to save computation
double squaredNorm(double x_a, double y_a, double x_b, double y_b){
	return pow((x_a - x_b),2) + pow((y_a - y_b),2);
}


class LocationHistory{

	public:
		int size = 0;
		vector<pair<double,double>> prev_locations;


	void add_location(double x, double y){
		prev_locations.push_back(make_pair(x,y));
		size = size + 1;
	}

	// returns true if too close to any other stored location
	bool too_close(double currX, double currY, double minAllowable){
		for (int i = 0; i < size; i++){
			if(squaredNorm(prev_locations[i].first, prev_locations[i].second, currX, currY) < minAllowable){
				return true;
			}
		}

		return false;
	}
};


LocationHistory _360_locations; 





// randomly pick either 1 or -1
int randomSign(){
	return 2*(rand()%2)-1;
}


int sign(double a){
	return a / abs(a);
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



 
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	map_data = msg->data;
	//cout << (map_data[480]==-1) << endl;
	resolution = msg->info.resolution;
	width = msg->info.width;
	height = msg->info.height;
	originX = msg->info.origin.position.x;
	originY = msg->info.origin.position.y;

}


const double ARMDISTANCE = 1.5;
const double ARMRESOLUTION = 0.025; //needs to equal map resolution;
const int ARMLENGTH = ARMDISTANCE/ARMRESOLUTION;
const double ARMANGLE = 0;

pair<double,double> leftarm_bot[ARMLENGTH];
pair<double,double> rightarm_bot[ARMLENGTH];

pair<double,double> leftarm_world[ARMLENGTH];
pair<double,double> rightarm_world[ARMLENGTH];

signed char leftarm_map[ARMLENGTH];
signed char rightarm_map[ARMLENGTH];


//this will operate with global variable
//coordinates in robot frame to odom frame (approximately map frame)
void armWorld(){

	for(int i =0; i < ARMLENGTH; i++){

		leftarm_world[i].first = cos(bot_yaw)*leftarm_bot[i].first - sin(bot_yaw)*leftarm_bot[i].second + bot_x;

		leftarm_world[i].second = sin(bot_yaw)*leftarm_bot[i].first + cos(bot_yaw)*leftarm_bot[i].second + bot_y;



		rightarm_world[i].first = cos(bot_yaw)*rightarm_bot[i].first - sin(bot_yaw)*rightarm_bot[i].second + bot_x;

		rightarm_world[i].second = sin(bot_yaw)*rightarm_bot[i].first + cos(bot_yaw)*rightarm_bot[i].second + bot_y;

	}
}


//this will operate with global variables
//coordinates in odom frame (approximately map frame) are transformed to map grid points
//and the value at that grid point is stored in the respective arm array
void armMapGrid(){

	for(int i = 0; i < ARMLENGTH; i++){

		int leftX = (leftarm_world[i].first - originX) / resolution;
		int leftY = (leftarm_world[i].second - originY) / resolution;

		//cout<<"width leftX  height leftY" << width<<" "<<leftX <<" "<<height<<" "<<leftY<<endl;


		int rightX = (rightarm_world[i].first - originX) / resolution;
		int rightY = (rightarm_world[i].second - originY) / resolution;

			
		// check within map array bounds
		if((leftX < width && leftX > 0) && (leftY < height && leftY > 0)){
			//cout<<"width leftX  height leftY" << width<<" "<<leftX <<" "<<height<<" "<<leftY<<endl;
/*			if(map_data[leftX + width*leftY] != -1){
				cout<<"not -1"<<endl;
			}
			else{
				cout<<"is -1"<<endl;
			}*/
			leftarm_map[i] = map_data[leftX + width*leftY];
		}



		// check within map array bounds
		if((rightX < width && rightX > 0) && (rightY < height && rightY > 0)){

			rightarm_map[i] = map_data[rightX + width*rightY];

		}

	}
}


// these will operate with global variables
// 1 if found unknown area
// 0 if is known area but not occupied
// -1 wall
int checkLeftArm(){

	for(int i=0; i<ARMLENGTH; i++){
		if(leftarm_map[i] > 25){
			return -1;
		}
		if(leftarm_map[i] == -1){
			return 1;
		}
	}

	return 0;
}


int checkRightArm(){

	for(int i=0; i<ARMLENGTH; i++){
		if(rightarm_map[i] > 25){
			return -1;
		}
		if(rightarm_map[i] == -1){
			return 1;
		}
	}

	return 0;
}






// simple plan based on sector ranges
double turnState[angleSectors];

// below wallThreshold is critical, move away
// beyond exploreThreshold means that that side is pretty open, take on an exploratory behaviour

command avoidancePlanner(double wallThreshold, command prevCommand, bumperStates bump, int leftArm, int rightArm){


	command plannedCommand = command(0,0,0,false,0);




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
 



/*------------------------------------------- Check conditions -------------------------------------------- */


	//check bumper first since its the most critical
	if(bump.isBumped()){

		//turn_avoid must be false to prevent
		//just running back into the bumper
		//back up at max speed for 0.35 seconds
		plannedCommand = command(0, -max_lin_speed, 0.25/max_lin_speed, false, 3);
		//printf("got bumped, backing up \n");
		goto endPlanning;
	}


	//if the last command was to backup due to bumper
	//turn a bit in a random direction
	if(prevCommand.type == 3){

		if(sectorRange[0] > sectorRange[4] && sectorRange[0] > wallThreshold){
			plannedCommand = command(-max_ang_speed, 0, (pi/3)/max_ang_speed, false, 2);
			goto endPlanning;
		}

		if(sectorRange[0] < sectorRange[4] && sectorRange[4] > wallThreshold){
			plannedCommand = command(max_ang_speed, 0, (pi/3)/max_ang_speed, false, 2);
			goto endPlanning;
		}
		else{
			plannedCommand = command(randomSign() * max_ang_speed, 0, (pi/3)/max_ang_speed, false, 2);
			goto endPlanning;
		}
	}


	// if last command was a turn to turn away from a wall
	if(prevCommand.turn_avoid == true){

		//go straight for a bit after turning away from the wall
		// cannot be overwritten by laser avoidance command
		// to avoid condition of getting stuck turning in a corner
		return command(0,0.1,0.2,false,2);
		goto endPlanning;

		//printf("going straight after turning from wall\n");
	}



	// this is true if the full 60 degree cone is within a wall - F
	// since this command has override, only do it if the previous command
	// was not a turn avoid command
	if(max_range < wallThreshold &&  prevCommand.turn_avoid == false){


		// if both arms say wall, then we're probably in a corner and should do a 180
		if(leftArm == -1 && rightArm == -1){
			plannedCommand = command(max_ang_speed, 0, pi/max_ang_speed, true, 2);
			goto endPlanning;
		}


		// if left arm says unknown, but not right
		if(leftArm == 1 && rightArm != 1){
			plannedCommand = command(max_ang_speed, 0, (100*pi/180)/max_ang_speed, true, 2);
			goto endPlanning;
		}

		// if right says unknown, but not left
		if(leftArm != 1 && rightArm == 1){
			plannedCommand = command(-max_ang_speed, 0, (100*pi/180)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		else{
			//if left slightly more range than right, turn left
			if(sectorRange[4]/sectorRange[0] > 1.2 ){
				plannedCommand = command(max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
				goto endPlanning;
			}
			//if right slightly more range than left, turn right
			else if(sectorRange[4]/sectorRange[0] < 1/1.2){
				plannedCommand = command(-max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
				goto endPlanning;
			}
			//if mostly straight on the wall, then turn in a random direction
			else{
				plannedCommand = command(randomSign()*max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
				goto endPlanning;
			}
		}

	} 



	//if right corner is near wall, but left corner not near wall
	if(sectorRange[0] < wallThreshold && sectorRange[4] > wallThreshold  &&  prevCommand.turn_avoid == false){

		
		if(sectorRange[1] < wallThreshold){
			// if right also near wall, then left turn by more, say ~45 degrees
			plannedCommand = command(max_ang_speed, 0, (pi/4)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		else{
			// if only right corner near wall, then just turn left by less, ~ 22 degrees
			plannedCommand = command(max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, 2);
			goto endPlanning;
		}
	}

	//if left corner is near wall, but right corner not near wall
	if(sectorRange[4] < wallThreshold && sectorRange[0] > wallThreshold  &&  prevCommand.turn_avoid == false){

		
		if(sectorRange[3] < wallThreshold){
			// if left also near wall, then right turn by more, say ~45 degrees
			plannedCommand = command(-max_ang_speed, 0, (pi/4)/max_ang_speed, true, 2);
			goto endPlanning;			
		}
		else{
			// if only right corner near wall, then just turn right by less, ~ 22 degrees
			plannedCommand = command(-max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, 2);
			goto endPlanning;
		}
	}


	//------------------------------------------------------------------------------------------------------------//

	//relative measure of which side has more range
	//more positive -> right side comparatively more range than left
	//more negative -> left side comparatively more range than right
	// ~0 -> both sides about the same



	for(int i=0; i<angleSectors; i++){ turnNumber = turnNumber + turnState[i]*sectorWeights[i]; }
	turnNumber = -1*turnNumber / angleSectors; //switch sign of turnNumber around
	if(turnNumber > 5){ turnNumber = 5; }
	if(turnNumber < -5){ turnNumber = -5; }

	//explore via turnNumber
	plannedCommand = command((turnNumber/5)*1*max_ang_speed, max_lin_speed, 0, false, 0);


	endPlanning:


	//plannedCommand =  command(randomSign()*max_ang_speed, 0, (pi/4)/max_ang_speed, true, false);

	return plannedCommand;
}



// current command MUST be passed by REFERENCE

void highLevelPlanner(command newAvoidanceCommand, command& currentCommand, double timeNow, double timeBetweenSpins, double& timeLastSpin, bool& leftExplore, bool& rightExplore, double& exploreTimer, double exploreInterval, bool _360_on){

		
	// speed of 360
	double spinSpeed = 1*max_ang_speed;
	double rangeExploreThreshold = 1.5; //if greater than this angle, then go forward and explore
	double exploreTurnAmount = pi/2;


	// if the avoidance says we're in danger, do what we always do
	// but if we're already doing a 360 then just go ahead, unless we get a bumper avoidance
	if(newAvoidanceCommand.type != 0 && currentCommand.type != 1){
		if(timeNow - currentCommand.start_time > currentCommand.duration || newAvoidanceCommand.type > currentCommand.type){

			currentCommand = newAvoidanceCommand;
			exploreTimer = ros::Time::now().toSec();  //avoidance commands reset the exploration 
			printf("Doing new avoidance command - start time: %f - turn avoid? %i \n",currentCommand.start_time,currentCommand.turn_avoid);
			printf("angular: %f   linear: %f \n",currentCommand.angularSpeed, currentCommand.linearSpeed);
			printf("command type: %i \n \n", currentCommand.type);

			goto endHighLevelPlanning;

		}
	}

	// should override spin if we hit a bumper
	else if(newAvoidanceCommand.type == 3 && currentCommand.type == 1){
		currentCommand = newAvoidanceCommand;
		exploreTimer = ros::Time::now().toSec();
		printf("Hit bumper during spin, performing bumper avoidance manuever \n \n");

		goto endHighLevelPlanning;
	}
    


	// since we're past the avoidance conditions check the explore conditions



	// what to do if the previous command expires and it was a type 4 exploration command
	if (timeNow - currentCommand.start_time > currentCommand.duration && currentCommand.type == 4){
 
		// check sector range

		// if a good amount of range then go forward
		if (sectorRange[2] > rangeExploreThreshold){
			currentCommand = command(0,0.2,2,false,0);
			cout<<"Good range here, will go further into here"<<endl<<endl;

			//clear the explore conditions just in case
			leftExplore = false;
			rightExplore = false;
	
			//printf("explore reset \n");

			goto endHighLevelPlanning;
		}

		else{

			// all these command are type 2, can't be disrupted by laser avoidance
			// but since not 4, will not trigger another check next time

			// if left turn to get here, right turn to get back 
			if(sign(currentCommand.angularSpeed) > 0){
				currentCommand = currentCommand = command(-spinSpeed, 0, exploreTurnAmount/spinSpeed, false, 2);
				cout<<"Not too much range, will turn back right"<<endl<<endl;

				//clear the explore conditions just in case
				leftExplore = false;
				rightExplore = false;
				exploreTimer = ros::Time::now().toSec() + exploreInterval;  //add exploreInterval to prevent from getting stuck
				//printf("explore reset \n");
			
				goto endHighLevelPlanning;

			}
			// if right turn to get here, left turn to get back
			if(sign(currentCommand.angularSpeed) < 0){
				currentCommand = currentCommand = command(spinSpeed, 0, exploreTurnAmount/spinSpeed, false, 2);
				cout<<"Not too much range, will turn back left"<<endl<<endl;

				//clear the explore conditions just in case
				leftExplore = false;
				rightExplore = false;
				//printf("explore reset\n");
				exploreTimer = ros::Time::now().toSec() + exploreInterval;  //add exploreInterval to prevent from getting stuck
				goto endHighLevelPlanning;

			}
		}
	}









	// is it about time for us to spin again?
    if(_360_on == true && timeNow - currentCommand.start_time > currentCommand.duration && timeNow - timeLastSpin > timeBetweenSpins){

    	if(_360_locations.too_close(bot_x, bot_y, 1.2)){ //will not do a 360 if within some distance of previous 360 spot
    		cout<<"already did a 360 close to here, moving on"<<endl<<endl;
    		timeLastSpin = ros::Time::now().toSec() + 2*pi/spinSpeed;
    	}
    	else{
	    	currentCommand = command(spinSpeed, 0 , 2*pi/spinSpeed, false, 1);
	    	cout<<"You spin me right round, baby right round, like a record baby, right round right round"<<endl;

	    	timeLastSpin = ros::Time::now().toSec() + 2*pi/spinSpeed;
	    	_360_locations.add_location(bot_x, bot_y);

	    	goto endHighLevelPlanning;
    	}
    }





    // if we don't have to do a spin what to do?
    // either explore or use turn number

    if(timeNow - currentCommand.start_time > currentCommand.duration){

		//if both left and right explore conditions, pick one at random
		if (leftExplore && rightExplore){
			currentCommand = command(randomSign()*spinSpeed, 0, exploreTurnAmount/spinSpeed, false, 4);
			leftExplore = false;
			rightExplore = false;
			exploreTimer = ros::Time::now().toSec();
			cout<<"Left and right maybe open, turning in random direction"<<endl;
			goto endHighLevelPlanning;
		}

		if(leftExplore && !rightExplore){
			currentCommand = command(spinSpeed, 0, exploreTurnAmount/spinSpeed, false, 4);
			leftExplore = false;
			exploreTimer = ros::Time::now().toSec();
			cout<<"Left appears to be unexplored, turning left to take a look"<<endl;
			goto endHighLevelPlanning;

		}

		if(!leftExplore && rightExplore){
			currentCommand = command(-spinSpeed, 0, exploreTurnAmount/spinSpeed, false, 4);
			rightExplore = false;
			exploreTimer = ros::Time::now().toSec();
			cout<<"Right appears to be unexplored, turning right to take a look"<<endl;
			goto endHighLevelPlanning;

		}

		else{
			currentCommand = newAvoidanceCommand;
		}

    	
/*    	printf("Doing new exploration command - start time: %f - turn avoid? %i \n",currentCommand.start_time,currentCommand.turn_avoid);
		printf("angular: %f   linear: %f \n",currentCommand.angularSpeed, currentCommand.linearSpeed);
		printf("command type: %i \n\n ", currentCommand.type);  */ 
	}



    endHighLevelPlanning:
    ; //null statement to close the function after the label

}










int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;


	// laser subscriber
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);


	// bumper subscriber
	bumperStates bumper;
	bumper.left = false; bumper.center = false; bumper.right = false;
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperStates::bumperCallback, &bumper);


	// odom position subscriber

	ros::Subscriber odom = nh.subscribe("/odom", 10, &odomCallback);



	// map occupany grid subscriber
	ros::Subscriber occupany_map = nh.subscribe("map", 1, &mapCallback);





	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;



	// for getting indices of scan sectors
	for(int s=0; s < angleSectors; s++){
		sectorIndicies[s] = del_sector * s;
	}

	// setting up sectorWeights
	for(int s=0; s < angleSectors; s++){
		sectorWeights[s] = (angleSectors - s) - ceil((double)angleSectors/2);
	}



	//set up arms
	for(int i=0; i<ARMLENGTH; i++){
		leftarm_bot[i].first = sin(ARMANGLE)*(i+1)*ARMRESOLUTION;
		leftarm_bot[i].second = cos(ARMANGLE)*(i+1)*ARMRESOLUTION;


		rightarm_bot[i].first = sin(ARMANGLE)*(i+1)*ARMRESOLUTION;
		rightarm_bot[i].second = -cos(ARMANGLE)*(i+1)*ARMRESOLUTION;

		//cout<<"x = " << rightarm_bot[i].first << "   " << "y = " << rightarm_bot[i].second<<endl;		
	}

	int left_flag, right_flag;
	bool left_lookout_flag = false, right_lookout_flag = false;
	bool left_explore_flag = false, right_explore_flag = false;
	
	double left_arm_timer = 0;
	double right_arm_timer = 0;

	double overall_exploration_timer = 0;
	double time_between_explorations = 1;



	command currCommand = command(0,0,0,false,0);



	//time stamps for 360
	double betweenSpin = 10;
	double lastSpin = 0;

	double enable360_time = 5*60; // 360s will be enabled after this many seconds
	bool enable360 = false;




	// time for contest
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();
	uint64_t secondsElapsed = 0;

	while(ros::ok() && secondsElapsed <= 480){

		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		//fill with your code


		if(enable360 == false && secondsElapsed > enable360_time){
			enable360 = true;
			cout<<"360s enabled after "<< enable360_time << " seconds"<<endl<<endl;
		}


		armWorld();
/*		cout<< "left X "<<leftarm_world[20].first<<"   "<<"left Y"<<leftarm_world[20].second<<endl;
		cout<< "right X "<<rightarm_world[20].first<<"   "<<"right Y"<<rightarm_world[20].second<<endl;*/

		armMapGrid();



		int temp_left = checkLeftArm();
		int temp_right = checkRightArm();


		// on rising edge, start timing and see if in some time that side is still unknown, to indentify significant unknown region
		if (left_flag != 1 && temp_left == 1){
			left_arm_timer = ros::Time::now().toSec();
			left_lookout_flag = true;

		}

		if (right_flag != 1 && temp_right == 1){
			right_arm_timer = ros::Time::now().toSec();
			right_lookout_flag = true;
		}

		left_flag = checkLeftArm();
		right_flag = checkRightArm();


		if (ros::Time::now().toSec() - left_arm_timer > 1.18 && left_flag == 1 && left_lookout_flag == 1 && ros::Time::now().toSec() - overall_exploration_timer > time_between_explorations){
			left_lookout_flag = false;

			left_explore_flag = true;
			//printf("explore left\n");

		}


		if (ros::Time::now().toSec() - right_arm_timer > 1.18 && right_flag == 1 && right_lookout_flag == 1 && ros::Time::now().toSec() - overall_exploration_timer > time_between_explorations){
			right_lookout_flag = false;

			right_explore_flag = true;
			//printf("explore right\n");


		}


/*		cout<<"left right"<<left_flag<<"   "<<right_flag<<endl;
		cout<<"center range"<<sectorRange[2]<<endl;
*/


		// threshold parameters for planner
		double wallThresh = 0.4;


		command avoidanceCommand = avoidancePlanner(wallThresh, currCommand, bumper, left_flag, right_flag);

		highLevelPlanner(avoidanceCommand, currCommand, ros::Time::now().toSec(), betweenSpin, lastSpin, left_explore_flag, right_explore_flag, overall_exploration_timer, time_between_explorations, enable360);

		


  		vel.angular.z = currCommand.angularSpeed;
  		vel.linear.x = currCommand.linearSpeed;

  		vel_pub.publish(vel);


  		//cout<<(180/pi)*bot_yaw<<endl;

  		//printf("width %i  height  %i  resolution %f  originX %f  originY %f \n",width, height, resolution, originX, originY);


  		//ROS_INFO("X:%f Y:%f Yaw:%f", currPosition.x, currPosition.y,currPosition.yaw);		
  		//ROS_INFO("LL:%f   L: %f   C:%f   R:%f  RR:%f   TN:%f",sectorRange[4],sectorRange[3], sectorRange[2],sectorRange[1],sectorRange[0],tempTurnNumber);
	
  		//ROS_INFO("left: %i   center: %i   right: %i   isbumped: %i", bumper.left,bumper.center,bumper.right,bumper.isBumped());

	
  		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	} 


	cout<<"Bye Felicia"<<endl<<endl;



	//automatically save the map
	cout<<"Saving map"<<endl;

	time_t now = time(0);
	tm *ltm = localtime(&now);

	string year = to_string(1900 + ltm->tm_year);
    string month = to_string(1+ ltm->tm_mon);
	string day = to_string(ltm->tm_mday);
	string hour = to_string(ltm->tm_hour);
	string minute = to_string(ltm->tm_min);


	string directory_name = "~/ros_maps/" + year + "_" + month + "_" + day + "_" + hour + "_" + minute;
	string make_folder = "mkdir " + directory_name;
	system(make_folder.c_str());
	string save_map = "rosrun map_server map_saver -f " + directory_name + "/map";
	system(save_map.c_str());






	return 0;
}









//to do

// 360 turn on time more robust



// bumper callback
// just get bumper state

/*can get the bumper state in an object, also has a function to check if any bumper 
are currently bumped, should pass bumper into avoidance planner*/




//check for turning left then turning right if in corner




/*----------- change log --------------- */

/*changed turning for big turn to 120 degrees, to help turn out of corners
*/













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
