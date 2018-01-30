// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <turtlesim/Pose.h>   // For turtlesim::Pose
#include <stdlib.h> // For rand() and RAND_MAX
#include <math.h>
#include <iostream>
using namespace std;

// Object Construction
geometry_msgs::Twist msg;
turtlesim::Pose pose;


// Variables Init
namespace turtle_pose{
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
}

namespace arena{
    float x1 = 0.0;
    float x2 = 11.1;
    float y1 = 0.0;
    float y2 = 11.1;
}

float goal[2] = {0.0, 0.0};
float center[2] = {5.5,5.5};
float Euclidean_distance = 1000.0;
float target_orient = 0.0;

// Check if a (x, y) pair is within the arena boundaries
bool in_boundary(float x, float y){
    if(x>=arena::x1 && x<=arena::x2 && y>=arena::y1 && y<=arena::y2) return true;
    return false;
}

// Def a callback function for "turtlesim::Pose" subscriber
void poseHandler(const turtlesim::Pose &pose){
    //ROS_INFO_STREAM("Tutle Pose: x: "<<pose.x<<" y: "<<pose.y<<" theta: "<<pose.theta);
    turtle_pose::x = pose.x;
    turtle_pose::y = pose.y;
    turtle_pose::theta = pose.theta;
}

void get_user_input(void){
  float user_x, user_y;
  char separator;
  cout<<"Please input turtlebot goal location x and y: ";
  cin>> user_x >> user_y;
  while (!in_boundary(user_x,user_y)){
        cout<<"User inputs (x,y) is out of range. \n"
        << "Please input turtlebot goal location x and y: ";
        cin>> user_x >> separator>> user_y;
  }
    goal[0] = user_x;
    goal[1] = user_y;
  cout<< "The goal location is: ("<<user_x<<", "<<user_y<<" )\n.";
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, poseHandler);

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(10);
  double time = ros::Time::now().toSec();

  get_user_input();

  while(ros::ok()) {

    // Compute the distance b/w the goal and the current location
    Euclidean_distance = sqrt( pow(goal[0]-turtle_pose::x,2) + pow(goal[1]-turtle_pose::y,2));
    // Compute the goal orientation
    target_orient = atan2(goal[1]-turtle_pose::y, goal[0]-turtle_pose::x);

    if( ros::Time::now().toSec()-time>2.5){
        ROS_INFO_STREAM_ONCE("Starting control...");
        if( Euclidean_distance < 0.2){
            ROS_INFO_STREAM("Mission Completed!");
            get_user_input();
        }
        else{
            msg.angular.z = std::max(float(-1.0),std::min(float(1.0), target_orient-turtle_pose::theta));

            if (fabs(msg.angular.z) > 0.2){
                msg.linear.x = 0.01;
            }else{
                msg.linear.x = std::max(float(0.0), std::min(float(1.0), Euclidean_distance));
                }
        }

    }else{
	//random init
        msg.linear.x = double(rand())/double(RAND_MAX);
        msg.angular.z = double(rand())/double(RAND_MAX);
        }
    // Publish the message.
    pub.publish(msg);

    // Send a message to rosout with the details.

	ROS_INFO_STREAM("Pose info:"
      << " x = " <<turtle_pose::x
      << " y = " <<turtle_pose::y
      << " angle = " << turtle_pose::theta);

    ros::spinOnce();
    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
