#include <imagePipeline.h>
#include <string>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();

        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {

        // Code for saving the images
        // std::string time = std::to_string((int)ros::Time::now().toSec());
        // std::string name = "/home/turtlebot/images/" + time +".jpg";
        // std::cout << name << std::endl;
        //imwrite( name,  img );

        cv::waitKey(1);

        // Blank images have some multiple matches greater than ~10
        double min_matches_blank = 10;
        
        // initialize it to the blank index
        template_id = 3;

        // Marks whether the box is blank or not
        // bool marked = false;

        // Records the best match, also if all matches less than this value, then probably blank
        double best_matches = 0;


        // For each box templates
        for (int i = 0; i < boxes.templates.size(); ++i)
        {
            // Match each box to the template
            double matches = matchToTemplate(boxes.templates[i]);

            switch(i){
                case 0 : std::cout << "Raisin Bran ";
                break;

                case 1 : std::cout << "Cinnamon Toast Crunch ";
                break;

                case 2 : std::cout << "Rice Krispies ";
                break;

            }
            std::cout  << " matched:  " << matches << std::endl;
            // std::cout << "img.rows:" << img.rows << std::endl;
            // std::cout << "img.cols:" << img.cols << std::endl;
            
            // // Heuristics for classification 1
            // if (matches > min_matches_blank){
            //     // Blank box: two or more template with multiple matches
            //     if (best_matches > min_matches_blank) marked = true;
            //     if (matches > best_matches){
            //         best_matches = matches;
            //         template_id = i;
            //     }
                
            // }
            // if (marked) template_id = 3;


            // Heuristics for classification 2
            if (matches > best_matches){
                best_matches = matches;
                template_id = i;
            }
                
        }
    }

        // For displaying the image
        // std::cout << "attempt disp img" << std::endl;
        // cv::imshow("view", img);
        // cv::waitKey(1000);


    std::cout  << "best id:  " << template_id << std::endl;
    return template_id;
}

double ImagePipeline::matchToTemplate(Mat img_object){
	/***
	 * SURF feature detector using implementation found at:
	 * https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_homography/feature_homography.html#feature-homography
	 *
	 * This function takes a template image path, loads it as a grayscale image and finds
	 * a number of feature matches using SURF features.
	 * For this template, it then computes a probability P(object|scene).
	 ***/

    // Mat img_object = imread( template_img_path, IMREAD_GRAYSCALE );    Passed in
    // Mat img_scene = img;		For us image_scene is simply called img

    //--Step 1& 2: Detect the keypoints and calculate descriptors using SURF Detectorintmin
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint>keypoints_object,keypoints_scene;
    Mat descriptors_object, descriptors_scene;


    //find features in a cropped image
    cv::Rect myROI(0,130,630,340);

    Mat cropped_img = img(myROI);

    detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);

    detector->detectAndCompute(cropped_img, Mat(), keypoints_scene, descriptors_scene);    //cropped_img instead of just img
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
     try {
        matcher.match( descriptors_object, descriptors_scene, matches );
    } catch (Exception& e) {
        ;
    }

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
    }

    //printf("-- Max dist : %f \n", max_dist );
    //printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
        { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    // drawMatches( img_object, keypoints_object, img, keypoints_scene,
    //              good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
    //              std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    /***
     * We have completed the matches. In the next section of code we will attempt to fit a
     * bounding box over the identified object in the scene image.
     */

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    Mat H = findHomography( obj, scene, RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);

    // Define scene_corners using Homography
    try {
        perspectiveTransform( obj_corners, scene_corners, H);
    } catch (Exception& e) {
        ;
    }

    // Define a contour using the scene_corners
    std::vector<Point2f> contour;
    for (int i = 0; i < 4; i++){
	    contour.push_back(scene_corners[i] + Point2f( img_object.cols, 0));
    }
    double area = contourArea(contour);
    double area_weight = 1.0;
    if (area >  600*400 || area < 10 * 10)
    {
        area_weight = 0.0;
    }
    else
    {
        area_weight = 1.0;
    }
    std::cout << "area: " << area << ", weight: " << area_weight << std::endl;

    double indicator;
    std::vector< DMatch > best_matches;

    // Check if the good match is inside the contour.
    Point2f matched_point;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        matched_point = keypoints_scene[ good_matches[i].trainIdx ].pt + Point2f( img_object.cols, 0);
        indicator = pointPolygonTest(contour, matched_point, false);
        if(indicator >= 0) best_matches.push_back( good_matches[i]);
    }
    
    drawMatches( img_object, keypoints_object, cropped_img, keypoints_scene,
                 best_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    
    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );
    cv::waitKey(10);
    /***
     * In this section of the code we use a chosen heuristic to decided how good the match
     * is the to given template.
     * One such heuristic is the absolute number of good_matches found.
     */

    return (double)best_matches.size()*area_weight;
}
