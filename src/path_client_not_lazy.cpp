//path_client:
// illustrates how to send a request to the path_service service
//not lazy version

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<example_ros_service::PathSrv>("path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    example_ros_service::PathSrv path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
//================Init=======
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // start (0, 0) 
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    

//=====not lazy ver==
//=====move 1 (0,2.8)=====
    pose.position.x = 0.0; 
    pose.position.y = 2.8;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 2 (7,2.8)=====
    pose.position.x = 7.0; 
    pose.position.y = 2.8;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 3 (7,5.8)=====
    pose.position.x = 7.0; 
    pose.position.y = 5.8;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 4 (5.2,5.8)=====
    pose.position.x = 5.2; 
    pose.position.y = 5.8;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 5 (5.2,9)=====
    pose.position.x = 5.2; 
    pose.position.y = 9;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 6 (2.4,9)=====
    pose.position.x = 2.4; 
    pose.position.y = 9;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 7 (2.4,12)=====
    pose.position.x = 2.4; 
    pose.position.y = 12;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 8 (0.4,12)=====
    pose.position.x = 0.4; 
    pose.position.y = 12;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);




    client.call(path_srv);

    return 0;
}
