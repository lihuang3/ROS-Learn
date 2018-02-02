// This program steers a turtlesim turtle1 toward the xy locatins entered in the command line.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <turtlesim/Pose.h>
#include <iomanip> // for std: setprecision and std::fixed
#include <stdlib.h> // For rand() and RAND_MAX
#include <math.h>       /* sqrt */
#include <std_srvs/Empty.h> // for clearing the screen
turtlesim::Pose turtlePose;  // global variable for the turtle's pose.  
// Note: global variables is generally bad programming practice.
int poseInitialized = 0;

// A callback function. Executed each time a new pose message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) {
    // TODO 2:  copy the msg pose to your global variable
    turtlePose.x = msg.x;
    turtlePose.y = msg.y;
    turtlePose.theta = msg.theta;
    poseInitialized = 1;
}

int main(int argc, char ** argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "quiz_steer_turtle_to_xy");
    ros::NodeHandle nh;
    // goal location
    float goalx, goaly;
    // clear the screen
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clearClient.call(srv);
     
    // Create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    // Create a subscriber object
    //TODO 1: setup the callback
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);
    // get the goal x and y locations
    if (argc == 3){
        goalx = atof(argv[1]);
        goaly = atof(argv[2]);
        // TODO 2: save the y location from the command input
        ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "goal position=(" << goalx << "," <<goaly << ")");
    }else{
        ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "You need to supply an x and y output");
        return -1;
    }
    // Loop at 10 Hz until the node is shut down.
    ros::Rate rate(10);
    // control variables
    float angErr,angGoal,distErr;
    //message to be sent to turtle.  The other four fields, which are ignored by turtlesim, default to zero.
    geometry_msgs::Twist msg;

    while(ros::ok()) {
        //TODO 3: you need to hand control over to ROS or your callbacks will never be checked (let ROS spin)
        ros::spinOnce();
        //compute control parameters
        distErr = sqrt( pow(goalx-turtlePose.x,2)+ pow(goaly-turtlePose.y,2) ); //TODO 4: calculate the distance
        angGoal = atan2( (goaly-turtlePose.y),  (goalx-turtlePose.x) );
        angErr  = atan2(sin(angGoal-turtlePose.theta), cos(angGoal-turtlePose.theta));  // TODO 5: use atan2 to calculate the angular error
        // control law
        if( fabsf(angErr) > 1.0){
            msg.linear.x = 0.2; // if angular error is large, turn (mostly) in place
         }else{
            msg.linear.x = fmin(distErr,1.0);
         }
        msg.angular.z = 20*angErr;  //TODO 6: what happens if you change this control gain from 1/2?
        // Publish the message.
        pub.publish(msg);
        // Send a message to rosout with the details.
        ROS_INFO_STREAM("vel command:"
            << " linear=" << msg.linear.x << " angular=" << msg.angular.z 
            << " pose=("<< turtlePose.x<<","<<turtlePose.y<<","<<turtlePose.theta<<"), angErr="<<angErr);
        if(distErr < 0.2 && poseInitialized ){ // return if at the goal.
            return 0;
        }    
            
        // Wait until it's time for another interaction
        rate.sleep();// 
    }
 } 
