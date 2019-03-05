#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

class pathPlanner {
	public:
		int NUM_LOC = 6;
		double adjacency [NUM_LOC][NUM_LOC];
		std::vector<int> plan [6];

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
    		std::cout << temp[0] << ' ' << temp[1] << ' ' << temp[2] << ' ' << temp[3] << ' ' << temp[4] << ' ' << temp[5] << '\n';
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
    double dist_b_g = 0.45; //cm
    const double pi = 3.14159;

    double x;
    double y;
    double phi;
    int checked[5] = {0,0,0,0,0};

    Navigation  nav;

    std::cout << "initializing goals" << std::endl;

    std::vector<std::vector<double>> goals;
    for(int i = 0; i < 3; ++i) {
        goals.push_back(std::vector<double>(boxes.coords.size()+1,0.0));
    }

    std::cout << "filling box coords" << std::endl;
    goals[0][0] = robotPose.x;
    goals[1][0] = robotPose.y;
    goals[2][0] = robotPose.phi;
    for(int i = 1; i < boxes.coords.size(); ++i) {
        x = boxes.coords[i][0];
        y = boxes.coords[i][1];
        phi = boxes.coords[i][2];

        goals[0][i] = x + dist_b_g * cos(phi);
        goals[1][i] = y + dist_b_g * sin(phi);
        goals[2][i] =  pi + phi;
    }


    pathPlanner planner;
    planner.setAdjacency(goals);
    planner.printAdjacency();
    planner.permute();

    std::cout << "ready to exec strat" << std::endl;


    // Execute strategy.
    while(ros::ok()){
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords

//        for (int i=0;i < boxes.coords.size(); ++i){
//			if(checked[i] == 0){
//				if(nav.moveToGoal(goals[i][0], goals[i][1], goals[i][2])){
//					checked[i] = 1;
//				}
//			}
//		}

        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
