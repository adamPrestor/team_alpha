#include "ros/ros.h"
#include <vector>
#include <nav_msgs/GetMap.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>

#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <localizer/Localize.h>

#include <detection_msgs/Detection.h>
#include <sensor_msgs/CameraInfo.h>

#include <boost/circular_buffer.hpp>

using namespace std; 
using namespace cv;
using namespace sensor_msgs;
using namespace boost;
using namespace visualization_msgs;
using namespace std_msgs;

/*
	VERSION CONTROL:
	Adam, 31.3.2017 / 10.40
	version 1.23
*/

/*
Control flags
*/
bool create_goals = false;
bool face_detection = true;

Mat cv_map;
Mat mask;
Mat visited;
bool build = false;
bool new_goal = true;
bool reached = false;

float map_resolution = 0;
int global_x;
int global_y;
int marker_counter;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Publisher makrers_pub;

ros::ServiceClient localize;

ros::Subscriber map_sub;
ros::Subscriber goal_check;
ros::Subscriber echo_goal;
geometry_msgs::PoseStamped *goal;

/*
Face detector and localizer
*/
ros::Subscriber detections;
ros::Subscriber camera_info;
circular_buffer<CameraInfo> buffer_cam(50);
MarkerArray markers;


/*
	TODO:
		-	non-maxima surpresion box!! /funkcija =  82 vrstica, klic = 150 vrstica
		-	preverjanje laznih detekcij - ali je res obraz? / predlogi??
*/

bool nonMaxima() {

	return false;
}

void markersFlush() 
{
	printf("Tako velik sem postal: %d\n", marker_counter);
	if(marker_counter == 0) 
	{
		markers = MarkerArray();
		return;
	}
	makrers_pub.publish(markers);
	
}

void detectionCallback(const detection_msgs::Detection& msg)
{
    //printf("%d : %d .::: %d %d \n", (msg).x, (msg).y, msg.width, msg.height);

    //CameraInfo& info;
    double best_time = 100;
    double time = 100;
    int best_cam = -1;

    for (int i = 0; i < buffer_cam.size(); i++)
    {
		time = abs(buffer_cam[i].header.stamp.toSec() - msg.header.stamp.toSec());
		if (time < best_time) 
		{
			best_time = time;
			best_cam = i;
		}
    }
    
    //printf("%d : %f -> return\n", best_cam, best_time);
    if (best_cam < 0 || best_time > 1)
    {
    	return;
    }
    	
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(buffer_cam[best_cam]);
    
    double u = msg.x + msg.width / 2;
    double v = msg.y + msg.height / 2;
    
	geometry_msgs::Point point;
	point.x = ((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx();
	point.y = ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy();
    point.z = 1.0;
    
    
    localizer::Localize local;
    local.request.header = msg.header;
    local.request.point = point;
    local.request.scope = 3;


    bool ok = localize.call(local.request, local.response);
    printf(ok ? "fine\n" : "nope\n" );
    if(!ok)
    	return;

    geometry_msgs::Pose pose;
    pose = local.response.pose;

    //if(nonMaxima)
    //	return;

    Marker marker;
    marker.header.stamp = msg.header.stamp;
    marker.header.frame_id = msg.header.frame_id;
    marker.pose = pose;
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.frame_locked = false;
    marker.lifetime = ros::Duration(1);
    marker.id = marker_counter;
    marker.scale = geometry_msgs::Vector3();
    marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
    marker.color = ColorRGBA();
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    markers.markers.push_back(marker);
    marker_counter+= 1;

    markersFlush();

}

void cameraInfo(const CameraInfo& msg)
{
    buffer_cam.push_back(msg);
}

class Goal
{
    public:
	  double move_x;
	  double move_y;
	  double rot_z;
	  double rot_w;
	  double rot_need;
	  Goal(double new_move_x,double new_move_y,double new_rot_z,double new_rot_w,double new_rot_need){
	  	 move_x = new_move_x;
	  	 move_y = new_move_y;
	  	 rot_z = new_rot_z;
	  	 rot_w = new_rot_w;
	  	 rot_need = new_rot_need;
	  }
};

std::vector<Goal> init_goals()
{

	std::vector<Goal> new_goals;
	new_goals.push_back(Goal(0.322843551636,-1.94747805595,0.992624093946,-0.12123286732,0));
	new_goals.push_back(Goal(0.0697892904282,-1.48674416542,0.999990795432,0.00429057699037,0));
	new_goals.push_back(Goal(0.439363002777,-0.746374368668,0.308404907133,0.951255177782,0));
	new_goals.push_back(Goal(0.182053804398,-0.210078477859,0.271035554277,0.962569336888,0));
	new_goals.push_back(Goal(-0.649054288864,0.0892417430878,-0.759095743624,0.650978995062,0));
	new_goals.push_back(Goal(-1.209122,-0.946491,-0.793062,0.609142,0));
	new_goals.push_back(Goal(-2.075730,-0.743495,-0.799946,0.600072,0));
	new_goals.push_back(Goal(-1.467552,0.162782,0.612537,0.790442,0));
	new_goals.push_back(Goal(-2.363542,0.152101,-0.776328,0.630329,0));
	new_goals.push_back(Goal(-2.070276,0.693174,-0.130295,0.991475,0));
	new_goals.push_back(Goal(-1.403334,1.050112,-0.148203,0.988957,0));
	new_goals.push_back(Goal(-2.308952,1.419876,0.570843,0.821059,0));
	new_goals.push_back(Goal(-3.090300,1.626125,0.610743,0.791829,0));
	new_goals.push_back(Goal(-3.925270,1.526461,0.637624,0.770348,0));
	new_goals.push_back(Goal(-4.696361,1.529686,0.916202,0.400717,0));
	new_goals.push_back(Goal(-4.903606,0.642826,0.997934,0.064241,0));
	new_goals.push_back(Goal(-4.593679,0.154904,0.964421,0.264372,0));
	new_goals.push_back(Goal(-4.328271,0.109782,-0.847527,0.530752,0));
	new_goals.push_back(Goal(-3.308464,0.740244,0.989210,0.146505,0));
	new_goals.push_back(Goal(-3.248306,-0.431818,-0.807465,0.589915,0));
	new_goals.push_back(Goal(2.715916,0.109927,0.664610,0.747187,0));
	return new_goals;
}
void checkCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg_res) {
	new_goal = true;
	reached = true;
	string s = msg_res->status.text;
	//printf(new_goal ? "true\n" : "false\n");
	printf("%s\n", s.c_str());
}

void echoGoal(const geometry_msgs::PoseStamped& msg_goal) {
	printf("Position: x=%f, y=%f, z=%f\n", msg_goal.pose.position.x, msg_goal.pose.position.y, msg_goal.pose.position.z);
	printf("Orientation: z=%f, w=%f\n", msg_goal.pose.orientation.z, msg_goal.pose.orientation.w);
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;
    global_x = size_x;
    global_y = size_y;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
}

void mouseCallback(int event, int x, int y, int, void* data) {

    if( event != EVENT_LBUTTONDOWN || cv_map.empty())
        return;

    int v = (int)cv_map.at<unsigned char>(y, x);

	if (v != 255) {
		ROS_WARN("Unable to move to (x: %d, y: %d) global: %d, not reachable", x, y, global_y);
		return;
	}
	
	y = global_y - y;

    ROS_INFO("Moving to (x: %d, y: %d), global: %d", x, y, global_y);

	tf::Point pt((float)x * map_resolution, (float)y * map_resolution, 0.0);
	tf::Point transformed = map_transform * pt;

	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "map";
	goal.pose.orientation.w = 1;
	goal.pose.position.x = transformed.x();
	goal.pose.position.y = transformed.y();
	goal.header.stamp = ros::Time::now();

	goal_pub.publish(goal);
}

void spitgoal()
{
	if (!cv_map.empty() && new_goal)
	{
		threshold(cv_map, mask, 254, 255, THRESH_BINARY);		
		mask = mask.mul(visited);

		imshow("AdamPrestar", mask);
		waitKey(30);
		
		Mat nz;
		findNonZero(mask, nz);
		
		cout << nz.total() << endl;
		//Select goal
		int point = rand() % nz.total();
		int x = nz.at<Point>(point).x;
		int y = global_y - nz.at<Point>(point).y;		
		for (int i = -5; i < 5; i++){
			for (int j = -5; j < 5; j++){
                if (x + i >= 0 && x + i < global_x && y + j >= 0 && j + j < global_y)
                    visited.at<uchar>(nz.at<Point>(point).y+j, x+i) = 0.0;

			}
		}
	
		tf::Point pt((float)x * map_resolution, (float)y * map_resolution, 0.0);
		tf::Point transformed = map_transform * pt;

		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = "map";
		goal.pose.orientation.w = 1;
		goal.pose.position.x = transformed.x();
		goal.pose.position.y = transformed.y();
		goal.header.stamp = ros::Time::now();

		goal_pub.publish(goal);	
		
		cout << x << y << endl;
		new_goal = false;
		
		waitKey(30);
	}
}

int main(int argc, char** argv) 
{
	namedWindow("AdamPrestar", CV_WINDOW_AUTOSIZE );
    ros::init(argc, argv, "map_goals");
    
    ros::NodeHandle n;

    std::vector<Goal> goals;
    if (create_goals)
    {
    	std::vector<Goal> goals = init_goals();
	}

	if (face_detection)
	{
    	map_sub = n.subscribe("map", 10, &mapCallback);
    	detections = n.subscribe("dlib_detector/detections", 100, &detectionCallback);
    	camera_info = n.subscribe("/camera/rgb/camera_info", 100, &cameraInfo);

    	makrers_pub = n.advertise<MarkerArray>("/markers", 10);

    	localize = n.serviceClient<localizer::Localize>("localizer/localize");
		bool ok = localize.exists();
		printf(ok ? "fine\n" : "nope\n" );

    }

    if (create_goals)
    {

    	goal_check = n.subscribe("/move_base/result", 10, &checkCallback);
   		echo_goal = n.subscribe("/move_base_simple/goal", 10, &echoGoal);
   		goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
   	}

    tf::TransformListener listener;
    namedWindow("Map");
    
    //setMouseCallback("Map", mouseCallback, NULL);
    int i = 0;
    
    while(ros::ok) 
    {
    	
        if (!cv_map.empty())
        {
        	if (!build && create_goals)
        	{
        		visited = Mat::ones(cv_map.rows, cv_map.cols, CV_8U);
        		build = true;
        	}	
        	imshow("Map", cv_map);

		}
        waitKey(30);
		
		if (!cv_map.empty() && reached && create_goals)
		{
			reached = false;
            tf::StampedTransform poseRobot;
            geometry_msgs::PoseStamped robot_pose;
            double yaw_r(50);
            try{
	            listener.lookupTransform("/map","/base_link",ros::Time(0), poseRobot);
	            robot_pose.pose.orientation.x = poseRobot.getRotation().getX();
	            robot_pose.pose.orientation.y = poseRobot.getRotation().getY();
	            robot_pose.pose.orientation.z = poseRobot.getRotation().getZ();
	            robot_pose.pose.orientation.w = poseRobot.getRotation().getW();
	            yaw_r = tf::getYaw(robot_pose.pose.orientation);  
	            double delta_yaw = M_PI - yaw_r;
	            if (delta_yaw > M_PI)
                   delta_yaw -= 2*M_PI;
                if (delta_yaw <= -M_PI)
                   delta_yaw += 2*M_PI;
                tf::TransformBroadcaster br; 
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(poseRobot.getOrigin().getX(), poseRobot.getOrigin().getY(), 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, delta_yaw);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "goal"));
                std::cout << "TURNING" << std::endl;
	        }
	        catch(tf::TransformException &ex){ //if the try doesn't succeed, you fall here!
		        // continue
		        ROS_INFO("error. no robot pose!");
		    }
            //and here I just make sure my angle is between minus pi and pi!
		}

		//Call goal maker
		//spitgoal();
		if (!cv_map.empty() && new_goal && create_goals)
		{
			geometry_msgs::PoseStamped goal;
			goal.header.frame_id = "map";
			goal.pose.position.x = goals[i].move_x;
			goal.pose.position.y = goals[i].move_y;
			
			std::cout << "indeks: " << i << "x: " << goal.pose.position.x << "y: " << goal.pose.position.y << std::endl;
			goal.pose.orientation.z = goals[i].rot_z;
			goal.pose.orientation.w = goals[i].rot_w;
			goal.header.stamp = ros::Time::now();
			goal_pub.publish(goal);
			new_goal = false;
			reached = false;
		    waitKey(30);
	        i++;

		}

        ros::spinOnce();
    }
    return 0;

}
