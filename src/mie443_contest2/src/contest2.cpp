#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

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
    float dist_b_g = 45.0; //cm

    float x;
    float y;
    float phi;

    Navigation  nav;

    std::vector<std::vector<float>> goals;
    for(int i = 0; i < boxes.coords.size(); ++i) {
        x = boxes.coords[i][0];
        y = boxes.coords[i][1];
        phi = boxes.coords[i][2];

        goals[i][0] = x + dist_b_g * cos(phi);
        goals[i][1] = y - dist_b_g * sin(phi);
        goals[i][2] =  90 + phi;
    }

    // Execute strategy.
    while(ros::ok()){
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        nav.moveToGoal(goals[0][0], goals[0][1], goals[0][2]);
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
