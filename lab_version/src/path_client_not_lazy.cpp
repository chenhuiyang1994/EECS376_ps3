//path_client:
// illustrates how to send a request to the path_service service
//not lazy version
//lab version is the same since no change is made for the service

#include <ros/ros.h>
#include <lab3/PathSrv.h> // this message type is defined in the current package
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
    ros::ServiceClient client = n.serviceClient<lab3::PathSrv>("path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    lab3::PathSrv path_srv;
    
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
    pose.position.y = 0.98;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 2 (7,2.8)=====
    pose.position.x = 0.98; 
    pose.position.y = 0.98;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 3 (7,5.8)=====
    pose.position.x = 0.98; 
    pose.position.y = 0;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
//=====move 4 (5.2,5.8)=====
    pose.position.x = 0; 
    pose.position.y = 0;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);





    client.call(path_srv);

    return 0;
}
