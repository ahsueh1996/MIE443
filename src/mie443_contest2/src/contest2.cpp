#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <math.h>
#include <algorithm>
#include <limits>

#include <iostream>
#include <fstream>

class pathPlanner {
	public:
		static const int NUM_LOC = 6;
		double adjacency [NUM_LOC][NUM_LOC];
		std::vector<int> plan;

		double squaredEuclDist(double x1, double y1, double x2, double y2){
			return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
		}

		void setAdjacency(std::vector<std::vector<double>> loc){ // coordinates has boxes and initial

			for (int i=0; i<NUM_LOC; i++){

				for (int j=0; j<NUM_LOC; j++){

					if (i != j){

						//std::cout << squaredEuclDist(loc[0][i], loc[1][i], loc[0][j], loc[1][j]) << std::endl;

						adjacency[i][j] = squaredEuclDist(loc[0][i], loc[1][i], loc[0][j], loc[1][j]);
					}
					else{
						adjacency[i][j] = 0;
					}
				}
			}
		}

		void printAdjacency(){

			for (int i=0; i<NUM_LOC; i++){

				for (int j=0; j<NUM_LOC; j++){

					std::cout << " " << adjacency[i][j] << " ";
				}
				std::cout << std::endl;
			}
		}

	 	void permute(){
			//std::vector<int> places = {0,1,2,3,4,5};
			double minSum = std::numeric_limits<double>::infinity();
			int places[] = {0,1,2,3,4,5};
			int temp[] = {0,0,0,0,0,0};
			do {
				double distSum = 0;
				for (int i=0; i<NUM_LOC-1; i++){
					distSum = distSum + adjacency[places[i]][places[i+1]];
				}
				distSum = distSum + adjacency[places[5]][0];
				if(distSum < minSum){
					minSum = distSum;
					std::copy ( places, places+6, temp);
				}
    			//std::cout << places[0] << ' ' << places[1] << ' ' << places[2] << ' ' << places[3] << ' ' << places[4] << ' ' << places[5] << '\n';
    		} while ( std::next_permutation(places+1,places+6) );
    		// std::cout << temp[0] << ' ' << temp[1] << ' ' << temp[2] << ' ' << temp[3] << ' ' << temp[4] << ' ' << temp[5] << '\n';
    		for (int i = 1; i <NUM_LOC ; ++i)
    		{
				plan.push_back(temp[i]);
    		}
    		plan.push_back(temp[0]);
    		std::cout << plan[0] << ' ' << plan[1] << ' ' << plan[2] << ' ' << plan[3] << ' ' << plan[4] << ' ' << plan[5] << '\n';
		}
};

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Define our goals, ie. the 5 objects with pictures on them
    //double dist_b_g = 0.60; //cm
	double dist_b_g = 0.55;
    const double pi = 3.14159;

    double x;
    double y;
    double phi;

	// Array to check if we arrived at the goal
    int checked[6] = {0,0,0,0,0,0};

    Navigation  nav;

	// Initialize the goals vector
    std::vector<std::vector<double>> goals;
    for(int i = 0; i < 3; ++i) {
        goals.push_back(std::vector<double>(boxes.coords.size()+1,0.0));
    }

	// Initialize the planner
    pathPlanner planner;

	// Need the robot pose callback
	ros::spinOnce();
	ros::Duration(0.10).sleep();
	ros::spinOnce();
	ros::Duration(0.10).sleep();
	ros::spinOnce();

	std::cout << "robot initial pose: " << robotPose.x <<  robotPose.y << robotPose.phi << std::endl;

	// First goal is the initial position (We have to get back to the initial position at the end)
	goals[0][0] = robotPose.x;
	goals[1][0] = robotPose.y;
	goals[2][0] = robotPose.phi;

	// Fill up the goal positions
	for(int i = 1; i < boxes.coords.size() + 1; ++i) {

		// Index starts from 1, start checking from index 0
		x = boxes.coords[i-1][0];
		y = boxes.coords[i-1][1];
		phi = boxes.coords[i-1][2];

		// Start filling up the goals from index 1
		// dist_b_g distance away from the box
		goals[0][i] = x + dist_b_g * cos(phi);
		goals[1][i] = y + dist_b_g * sin(phi);

		// Face the box
		goals[2][i] =  pi + phi;
	}
	// Insert the goals
	planner.setAdjacency(goals);
	planner.printAdjacency();

	// Update the planner
	planner.permute();

    int ind;
	int i = 0;
	int first_img;
	int second_img;
	int third_img;
	int final_decision;
	int tags[5] = {0,0,0,0,0}; 

	std::cout << "Open output file" << std::endl;
    std::ofstream output;
    output.open("/home/turtlebot/MIE443/output.txt");

    // Execute strategy.
    while(ros::ok()){
		/***YOUR CODE HERE***/

		// Index of the box to check
		ind = planner.plan[i];
		std::cout << "Box: " << ind << std::endl;

		ros::Duration(0.10).sleep();
		ros::spinOnce();

		// Image recognition
		// img = imagePipeline.getTemplateID(boxes);
		// std::cout << "Image: " << img << std::endl;
		// continue;
		//std::cout << "moving towards: " << goals[0][ind] <<  goals[1][ind] << goals[2][ind] << std::endl;

		// Move towards the first box
		if(nav.moveToGoal(goals[0][ind], goals[1][ind], goals[2][ind])){
			// Increment the index of the box for the next iteration
			i += 1;
        	ros::Duration(0.10).sleep();
			ros::spinOnce();

			// Image recognition
			std::cout << "Taking first picture" << std::endl << std::endl;
			first_img = imagePipeline.getTemplateID(boxes);
			std::cout << std::endl;
			ros::Duration(0.10).sleep();
			ros::spinOnce();
			ros::Duration(0.60).sleep();
			ros::spinOnce();

			std::cout << "Taking second picture" << std::endl << std::endl;
			second_img = imagePipeline.getTemplateID(boxes);
			std::cout << std::endl;

			if (first_img != second_img){
				ros::Duration(0.10).sleep();
				ros::spinOnce();
				ros::Duration(0.60).sleep();
				ros::spinOnce();


				std::cout << "Taking third picture" << std::endl << std::endl;
				third_img = imagePipeline.getTemplateID(boxes);
				std::cout << std::endl;


				if (third_img == first_img){ 
					final_decision = first_img;
				}
				if (third_img == second_img){ 
					final_decision = second_img;
				}
				else{
					final_decision = 3; //if can't agree then maybe blank
				}

			}
			else{
				final_decision = first_img;
			}
	
			std::cout << "===>  tag: ";

			tags[ind-1] = final_decision;
		
			output << "Box: " << ind << std::endl;
    		output << "At location (" << boxes.coords[ind-1][0] << ", " << boxes.coords[ind-1][1] << ", " << boxes.coords[ind-1][2] <<  ")" << std::endl;
    		

			switch(final_decision){
                case 0 : std::cout << "Raisin Bran" << std::endl;
                		 output << "Raisin Bran was found" << std::endl;
                break;

                case 1 : std::cout << "Cinnamon Toast Crunch" << std::endl;
                		 output << "Cinnamon Toast Crunch was found" << std::endl;
                break;

                case 2 : std::cout << "Rice Krispies" << std::endl;
                		 output << "Rice Krispies was found" << std::endl;
                break;

                case 3 : std::cout << "Blank" << std::endl;
                		 output << "nothing was found" << std::endl;
                break;

            }

            std::cout << std::endl;



			// Mark the box as visited
			checked[ind] = 1;	
		}
		else{
			// If failed to visit this box, move onto the next box
			i += 1;
		}
		if (i > 4){
			// Return back to the origianl pose and terminate
			if(nav.moveToGoal(goals[0][0], goals[1][0], goals[2][0])) return 0;
		}

		// // Probably don't have enough time to do this.
		// if (i > 5){
		// 	for (int j = 0; j < 5; ++j){
		// 		ind = planner.plan[j];
		// 		if (checked[ind] == 0){
		// 			checked[ind] = nav.moveToGoal(goals[0][ind], goals[1][ind], goals[2][ind]);
		// 			ros::Duration(0.01).sleep();
		// 			ros::spinOnce();
	
		// 			img = imagePipeline.getTemplateID(boxes);
		// 			std::cout << "Image: " << img << std::endl;
		// 		}
		// 	}
		// 	nav.moveToGoal(goals[0][0], goals[1][0], goals[2][0]);
		// 	return 0;
		// }		
        
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

	// Assume the coordinates are in 2D vector: goal
 //    Assume the tags are in 1D vector: tags
    
  //   for (int i=0;i<5;i++){
		// ind = planner.plan[i];
		// output << "Box: " << ind << std::endl;
  //   	output << "At location ( " << goals[0][ind+1] << ", " << goals[1][i+1] << " ),";
  //   	output << " the tag " << tags[ind] << " was found.\n";
  //   }
    output.close();
    std::cout << "Wrote output file" << std::endl;

    return 0;
}
