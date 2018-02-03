// This program steers a turtlesim turtle1 toward the xy locatins entered in the command line.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <turtlesim/Pose.h>
#include <iomanip> // for std: setprecision and std::fixed
#include <stdlib.h> // For rand() and RAND_MAX
#include <math.h>       /* sqrt */
#include <std_srvs/Empty.h> // for clearing the screen
// The srv class for the service.
#include <turtlesim/Spawn.h>

turtlesim::Pose turtlePose1, turtlePose2;  // global variable for the turtle's pose.
// Note: global variables is generally bad programming practice.
bool poseInitialized1 = false, poseInitialized2 = false;

float dx, dy, du, dv;
// A callback function. Executed each time a new pose message arrives.
void poseHandler1(const turtlesim::Pose& msg) {
    // TODO 2:  copy the msg pose to your global variable
    turtlePose1.x = msg.x;
    turtlePose1.y = msg.y;
    turtlePose1.theta = msg.theta;
    poseInitialized1 = true;
}

// A callback function. Executed each time a new pose message arrives.
void poseHandler2(const turtlesim::Pose& msg) {
    // TODO 2:  copy the msg pose to your global variable
    turtlePose2.x = msg.x;
    turtlePose2.y = msg.y;
    turtlePose2.theta = msg.theta;
    poseInitialized2 = true;
}


int main(int argc, char ** argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;
    // goal location
    float goalx, goaly;
    srand(time(0));
    // clear the screen
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clearClient.call(srv);

    // Create a client object for the spawn service.  This
    // needs to know the data type of the service and its
    // name.
    ros::ServiceClient spawnClient
    = nh.serviceClient<turtlesim::Spawn>("spawn");

    // Create the request and response objects.
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    // Fill in the request data members.
    req.x = 10*double(rand())/double(RAND_MAX);
    req.y = 10*double(rand())/double(RAND_MAX);
    req.theta = double(rand())/double(RAND_MAX);
    req.name = "turtle2";

    // Actually call the service.  This won't return until
    // the service is complete.
    bool success = spawnClient.call(req, resp);

    // Check for success and use the response.
    if(success) {
    ROS_INFO_STREAM("Spawned a turtle named "
      << resp.name);
    } else {
    ROS_ERROR_STREAM("Failed to spawn.");
    }

    // Create a publisher object
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    // Create a publisher object
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1000);

    // Create a subscriber object
    ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 1000, &poseHandler1);
        // Create a subscriber object
    ros::Subscriber sub2 = nh.subscribe("turtle2/pose", 1000, &poseHandler2);
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
    geometry_msgs::Twist msg1, msg2;

    while(ros::ok()) {
        //TODO 3: you need to hand control over to ROS or your callbacks will never be checked (let ROS spin)
        ros::spinOnce();
        //compute control parameters


        distErr = sqrt( pow(turtlePose2.x-turtlePose1.x,2)+ pow(turtlePose2.y-turtlePose1.y,2) ); //TODO 4: calculate the distance
        angGoal = atan2( (goaly-turtlePose1.y),  (goalx-turtlePose1.x) );
        angErr  = atan2(sin(angGoal-turtlePose1.theta), cos(angGoal-turtlePose1.theta));  // TODO 5: use atan2 to calculate the angular error
        if (distErr<0.1 && poseInitialized2 && poseInitialized1){
            ROS_INFO_STREAM(std::fixed << "Mission Completed!");
            ros::shutdown();
        }

        dx = turtlePose1.x-turtlePose2.x;
        dy = turtlePose1.y-turtlePose2.y;
        du = -cos(turtlePose1.theta) + cos(turtlePose2.theta);
        dv = -sin(turtlePose1.theta) + sin(turtlePose2.theta);
        angErr = atan2(  (-dv*dx+du*dy) , (du*dx+dv*dy));

        // control law
        if( fabsf(angErr) > 0.1){
            msg1.linear.x = 0.1; // if angular error is large, turn (mostly) in place
            msg2.linear.x = 0.1;
         }else{
            msg1.linear.x = fmin(distErr, 1.0);
            msg2.linear.x = fmin(distErr, 1.0);
         }
        msg1.angular.z = 2*angErr;  //TODO 6: what happens if you change this control gain from 1/2?
        msg2.angular.z = 2*angErr;

        // Publish the message.

        pub1.publish(msg1);
        pub2.publish(msg2);

        // Send a message to rosout with the details.
        ROS_INFO_STREAM("vel command:"
            << " linear=" << msg1.linear.x << " angular=" << msg1.angular.z
            << " pose=("<< turtlePose1.x<<","<<turtlePose1.y<<","<<turtlePose1.theta<<"), angErr="<<angErr
            << " linear=" << msg2.linear.x << " angular=" << msg2.angular.z
            << " pose=("<< turtlePose2.x<<","<<turtlePose2.y<<","<<turtlePose2.theta<<"), angErr="<<angErr);
        if(distErr < 0.2 && poseInitialized1 && poseInitialized2 ){ // return if at the goal.
            return 0;
        }

        // Wait until it's time for another interaction
        rate.sleep();//
    }
 }