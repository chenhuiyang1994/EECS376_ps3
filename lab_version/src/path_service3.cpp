//path_service:
// example showing how to receive a nav_msgs/Path request
// run with complementary path_client
// responds immediately to ack new path...but execution takes longer

// this is a crude service; just assumes robot initial pose is 0,
// and all subgoals are expressed with respect to this initial frame.
// i.e., equivalent to expressing subgoals in odom frame

//works better with not lazy version

#include <ros/ros.h>
#include <lab3/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
using namespace std;
//some tunable constants, global
const double g_move_speed = 0.2; // real bot 1m/s will be too fast
const double g_spin_speed = 0.5; // a save value to try
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 


// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//corrected
void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
//=========edit==================
 double dx=goal_pose.position.x-current_pose.position.x;
 double dy=goal_pose.position.y-current_pose.position.y;
 dist = sqrt(dx*dx+dy*dy); //corrected

 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
    heading = atan2(dy,dx); //corrected
 }

}


bool callback(lab3::PathSrvRequest& request, lab3::PathSrvResponse& response)
{
    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    geometry_msgs::Quaternion quat_current;
    int npts = request.nav_path.poses.size();
    ROS_INFO("received path request with %d poses",npts);    
    
    for (int i=0;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = request.nav_path.poses[i].pose; //get next pose from vector of poses
        
        //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
        get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,
           pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);         
        
        
        // a quaternion is overkill for navigation in a plane; really only need a heading angle
        // this yaw is measured CCW from x-axis
        // GET RID OF NEXT LINE AFTER FIXING get_yaw_and_dist()
        //yaw_desired = convertPlanarQuat2Phi(pose_desired.orientation); //from i'th desired pose
        ROS_INFO("pose %d: current yaw = %f",i,yaw_current);    
        ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
        yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        do_spin(spin_angle); // carry out this incremental action
        //add quat update
        quat_current=convertPlanarPhi2Quaternion(yaw_desired);
        
        //FIXED
        do_move(travel_distance);  // fixed
// we will just assume that this action was successful--really should have sensor feedback here
        g_current_pose.position = pose_desired.position; // position update
        g_current_pose.orientation=quat_current;
        }

  return true;
}

void do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);    //change topic to the one real rebot used , the only change made to make the srv work for the bot!
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_service");
  ros::NodeHandle n;
  
  // to clean up "main", do initializations in a separate function
  // a poor-man's class constructor
  do_inits(n); //pass in a node handle so this function can set up publisher with it
  
  // establish a service to receive path commands
  ros::ServiceServer service = n.advertiseService("path_service", callback);
  ROS_INFO("Ready to accept paths.");
  ros::spin(); //callbacks do all the work now

  return 0;
}
