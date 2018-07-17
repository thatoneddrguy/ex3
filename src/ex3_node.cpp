#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include "turtlesim/Pose.h"
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>
#include <ros/master.h>
#include <boost/algorithm/string.hpp>
using namespace std;

const int MAX_TTURTLES = 7; //this number can be changed 
const int MAX_XTURTLES = 10; //this number can be changed 
const double DANGER_TOLERANCE = 0.5;
const double LOWER_LIMIT = 0.0;
const double UPPER_LIMIT = 11.0;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
const double PI = 3.14159265359;

void rotate(double angular_speed, double relative_angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation(double desired_angle_radians);
void moveGoal(double relative_x, double relative_y, double distance_tolerance);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);

struct TurtlePose {
  string turtlename;
  string topicname;
  turtlesim::Pose pose;
};

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "ex3_node");
	ros::NodeHandle nh;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	
	velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = nh.subscribe("/turtle1/pose", 1000, poseCallback);
	
	ros::Rate loop(1);
	turtlesim::Pose goal_pose;

	setDesiredOrientation(0);
	loop.sleep();

	moveGoal(0, 0, 0.01);
	loop.sleep();	

	setDesiredOrientation(0); // make sure loop() is set to 1 before this call(?)
	loop.sleep();

	ros::Rate loop_rate(100);

	moveGoal(4, 0, 0.01); //actually "relative" at this point...
	loop.sleep();

	setDesiredOrientation(degrees2radians(90));
	loop.sleep();

	moveGoal(0, 2, 0.01);
	loop.sleep();

	setDesiredOrientation(degrees2radians(225));
	loop.sleep();

	moveGoal(-2, -2 / sqrt(3), 0.01); // 30/60/90 triangle
	loop.sleep();

	setDesiredOrientation(degrees2radians(135));
	loop.sleep();
	
	moveGoal(-2, 2 / sqrt(3), 0.01);
	loop.sleep();
	
	setDesiredOrientation(degrees2radians(270));
	loop.sleep();

	moveGoal(0, -2, 0.01);
	loop.sleep();
}


///////////////////////////////
namespace HW {
  static TurtlePose turtle1;
  static TurtlePose tturtles[MAX_TTURTLES];
  static TurtlePose xturtles[MAX_XTURTLES];
  static ros::ServiceClient sClient;
  static ros::ServiceClient kClient;
  static string getTurtlename(const string topicname);
  static bool topicExist(const string topicname);
  static bool turtleExist(const string turtlename);
  static turtlesim::Pose getNonOverlappingPoint(char tType);
  static void createTurtles(char tType, int cnt);
  static double getDistance(double x1, double y1, double x2, double y2);
  static bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
  static void removeTurtle1();
};

namespace HW {

string getTurtlename(const string topicname) {
  vector<string> elems;
  char lc_delim[2];
  lc_delim[0] = '/';
  lc_delim[1] = '\0';

  boost::algorithm::split(elems, topicname, boost::algorithm::is_any_of(lc_delim));
  return elems[1];
}

bool topicExist(const string topicname) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names 
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = alltopics[i].name;
     if (tname.compare(topicname) == 0) {
        return true;
     };
  };
  return false;
}

bool turtleExist(const string turtlename) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names 
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = getTurtlename(alltopics[i].name);
     if (tname.compare(turtlename) == 0) {
        return true;
     };
  };
  return false;
}

turtlesim::Pose getNonOverlappingPoint(char tType) {
  turtlesim::Pose xp;
  bool tooclose = false;
  int i;
  int ocnt=0;

  xp.x = double((rand() % 10) + 2.0);
  xp.y = double((rand() % 10) + 2.0);

  while (true) {
    if (HW::isTooClose(HW::turtle1.pose.x, HW::turtle1.pose.y, xp.x, xp.y, DANGER_TOLERANCE)) 
        tooclose = true;
    else if (tType == 'T')
            break; //out of while loop
    else { //X turtle needs to check all T turtles
       for (i=0; i<MAX_TTURTLES; i++) {
           if (HW::isTooClose(HW::tturtles[i].pose.x, HW::tturtles[i].pose.y, xp.x, xp.y, DANGER_TOLERANCE)) {
              tooclose = true;
              break; //out of for loop and regenerate a point
           };
       };
    };

    if (!tooclose) //checking for X turtle case
       break; //out of while loop

    if (ocnt>1000) { //only to check abnormality
       ROS_INFO_STREAM("chk: " << xp.x << "," << xp.y << "\n");
       break; //possibly wrong so exit 
    };
    //generate another random pose
    xp.x = double((rand() % 10) + 2.0);
    xp.y = double((rand() % 10) + 2.0);
    tooclose = false;
    ocnt++;
    ROS_INFO_STREAM(".");
  };
  return xp;
}

void createTurtles(char tType, int cnt) {
  int i;
  stringstream tname, cmdstr;
  bool success = false;
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;
  turtlesim::Pose nop;

  for (i=0; i<cnt; i++) {
     tname.clear();
     tname.str("");
     tname << tType << i + 1;
     req.name = tname.str();
     nop = HW::getNonOverlappingPoint(tType);
     req.x = nop.x;
     req.y = nop.y;
     req.theta = M_PI/2; //face up for target turtles

     tname.clear();
     tname.str("");
     tname << "/" << req.name << "/pose";

     //fill out turtles tables for pose tracking
     if (tType == 'X') {
        req.theta = 3.0*req.theta; //change to face down for villain turtles
        HW::xturtles[i].turtlename = req.name;
        HW::xturtles[i].topicname = tname.str();
        HW::xturtles[i].pose.x = req.x;
        HW::xturtles[i].pose.y = req.y;
        HW::xturtles[i].pose.theta = req.theta;
     }
     else {
        HW::tturtles[i].turtlename = req.name;
        HW::tturtles[i].topicname = tname.str();
        HW::tturtles[i].pose.x = req.x;
        HW::tturtles[i].pose.y = req.y;
        HW::tturtles[i].pose.theta = req.theta;
     };

     //if this turtle does not exist, create one else teleport it.
     if (!turtleExist(req.name.c_str())) {
        success = HW::sClient.call(req, resp);
        if(success) {
           if (tType == 'X')
              ROS_INFO("%s landed with face down.", req.name.c_str()); //X turtle
           else 
              ROS_INFO("%s landed with face up.", req.name.c_str()); //T turtle
        }
        else { 
          ROS_ERROR_STREAM("Error: Failed to create " << tType << " turtle.");
          ros::shutdown();
        }
     }
     else {
        cmdstr.clear();
        cmdstr.str("");
        cmdstr << "rosservice call /";
        cmdstr << req.name.c_str() << "/teleport_absolute " << req.x << " " << req.y << " " << req.theta;
        system(cmdstr.str().c_str()); 
        ROS_INFO_STREAM(req.name.c_str() << " already landed, so it's teleported!\n");
     };
  };
} 

double getDistance(const double x1, const double y1, const double x2, const double y2) {
  return sqrt(pow((x1-x2),2) + pow(y1-y2, 2));
}

bool isTooClose(double x1, double y1, double x2, double y2, double threshhold) {
  if (HW::getDistance(x1, y1, x2, y2) <= threshhold)
     return true;
  else
     return false;
}

void removeTurtle1() {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  reqk.name = HW::turtle1.turtlename;
  if (!HW::kClient.call(reqk, respk))
     ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
  else
     ROS_INFO_STREAM("!!! Mission failed !!!");

  ROS_INFO_STREAM("...shutting down...\n");
  ros::shutdown();
}

}; //end of namespace
///////////////////////////////////////

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;
	double Kw = 1.0;
	
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if(clockwise)
	{
		vel_msg.angular.z = Kw * -abs(angular_speed);
	}
	else
	{
		vel_msg.angular.z = Kw * abs(angular_speed);
	}

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	double t1;
	ros::Rate loop_rate(100);

	do {
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_angle < relative_angle);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees * PI / 180.0;
}

void setDesiredOrientation(double desired_angle_radians)
{
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians < 0) ? true : false);
	rotate(degrees2radians(180), abs(relative_angle_radians), clockwise);
}

void moveGoal(double relative_x, double relative_y, double distance_tolerance)
{
	turtlesim::Pose goal_pose;
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();
	
	ROS_INFO_STREAM("x: " << turtlesim_pose.x);
	ROS_INFO_STREAM("y: " << turtlesim_pose.y);
	ROS_INFO_STREAM("theta: " << turtlesim_pose.theta);

	goal_pose.x = turtlesim_pose.x + relative_x;
	goal_pose.y = turtlesim_pose.y + relative_y;

	do {
		ros::Rate loop_rate(100);		
		double Kv = 1.0;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

		vel_msg.linear.x = (Kv * e);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		double Kw = 4.0;

		vel_msg.angular.z = Kw * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
		velocity_publisher.publish(vel_msg);
		ROS_INFO_STREAM("publish called");
		ros::spinOnce();
		ROS_INFO_STREAM("spinOnce called");
		//ROS_INFO_STREAM("x: " << turtlesim_pose.x);
		//ROS_INFO_STREAM("y: " << turtlesim_pose.y);
		//ROS_INFO_STREAM("theta: " << turtlesim_pose.theta);
		//ROS_INFO_STREAM("goal x: " << goal_pose.x);
		//ROS_INFO_STREAM("goal y: " << goal_pose.y);
		//ROS_INFO_STREAM("goal theta: " << goal_pose.theta);
		loop_rate.sleep();
		ROS_INFO_STREAM("sleep called");
	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);

	//ROS_INFO_STREAM("x: " << turtlesim_pose.x);
	//ROS_INFO_STREAM("y: " << turtlesim_pose.y);
	//ROS_INFO_STREAM("theta: " << turtlesim_pose.theta);
	cout << "end move goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
	/*ROS_INFO_STREAM("x: " << turtlesim_pose.x);
	ROS_INFO_STREAM("y: " << turtlesim_pose.y);
	ROS_INFO_STREAM("theta: " << turtlesim_pose.theta);*/
	ROS_INFO_STREAM("turtlesim_pose updated");
}

double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
