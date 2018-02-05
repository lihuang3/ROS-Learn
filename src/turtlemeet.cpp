// This program is collaborated by Li Huang and Haoran Zhao
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <turtlesim/Pose.h>
#include <iomanip> // for std: setprecision and std::fixed
#include <stdlib.h> // For rand() and RAND_MAX
#include <math.h>       /* sqrt */
#include <std_srvs/Empty.h> // for clearing the screen
// The srv class for the service.
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>  // for setpen

turtlesim::Pose turtlePose1, turtlePose2;  // global variable for the turtle's pose.
// Note: global variables is generally bad programming practice.
bool poseInitialized1 = false, poseInitialized2 = false;

// Specify arena boundaries
namespace arena{
    float x1 = 0.1;
    float x2 = 11.0;
    float y1 = 0.1;
    float y2 = 11.0;
}

// Define dx = X1-X2, dy = Y1-Y2, du = Vx2-Vx1, dv = Vy2-Vy1
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
    ros::service::waitForService("clear");
    // Create a client object for the spawn service.  This
    // needs to know the data type of the service and its
    // name.
    ros::ServiceClient spawnClient
    = nh.serviceClient<turtlesim::Spawn>("spawn");

    // Create the request and response objects.
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    // Fill in the request data members.
    req.x = 2+6*double(rand())/double(RAND_MAX);
    req.y = 2+6*double(rand())/double(RAND_MAX);
    req.theta = 2*M_PI*double(rand())/double(RAND_MAX);
    req.name = "turtle2";

    // Create a client object for the teleport service.
    ros::ServiceClient teleportAbsClient1 = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    ros::ServiceClient teleportAbsClient2 = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle2/teleport_absolute");

    turtlesim::TeleportAbsolute::Request req1, req2;
    turtlesim::TeleportAbsolute::Response resp1, resp2;


    // Actually call the service.  This won't return until
    // the service is complete.
    bool success = spawnClient.call(req, resp);

    // Check for success and use the response.
    if(success) {
        ROS_INFO_STREAM("Spawned a turtle named "
          << resp.name);
    } else {
        ROS_ERROR_STREAM("Failed to spawn.");
        req2.x = 2+6*double(rand())/double(RAND_MAX);
        req2.y = 2+6*double(rand())/double(RAND_MAX);
        req2.theta = 2*M_PI*double(rand())/double(RAND_MAX);
        teleportAbsClient2.call(req2,resp2);
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
        req1.x = atof(argv[1]);
        req1.y = atof(argv[2]);
        req1.theta = 2*M_PI*double(rand())/double(RAND_MAX);;
        teleportAbsClient1.call(req1,resp1);
        ROS_INFO_STREAM("arg0="<<atof(argv[0])<<"arg1"<<atof(argv[1])<<"arg2"<<atof(argv[2]));
    }else{
        req1.x = 2+6*double(rand())/double(RAND_MAX);
        req1.y = 2+6*double(rand())/double(RAND_MAX);
        req1.theta = 2*M_PI*double(rand())/double(RAND_MAX);
        teleportAbsClient1.call(req1,resp1);
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

        // Apply teleport if out-of-boundary
        if (poseInitialized1 && poseInitialized2)
        {
            if( (turtlePose1.x<arena::x1) || (turtlePose1.x>arena::x2) ||
            (turtlePose1.y<arena::y1) || (turtlePose1.y>arena::y2))
            {
                req1.x = turtlePose1.x;
                req1.y = turtlePose1.y;
                req1.theta = turtlePose1.theta;
                if      (turtlePose1.x<arena::x1){req1.x = arena::x2-0.05;}
                else if (turtlePose1.x>arena::x2){req1.x = arena::x1+0.05;}
                if      (turtlePose1.y<arena::y1){req1.y = arena::y2-0.05;}
                else if (turtlePose1.y>arena::y2){req1.y = arena::y1+0.05;}

            ROS_INFO_STREAM("\nturtle1 out of boundary! Teleporting to a new location\n");
            bool success1 = teleportAbsClient1.call(req1,resp1);

            }

            if( (turtlePose2.x<arena::x1) || (turtlePose2.x>arena::x2) ||
            (turtlePose2.y<arena::y1) || (turtlePose2.y>arena::y2))
            {
                req2.x = turtlePose2.x;
                req2.y = turtlePose2.y;
                req2.theta = turtlePose2.theta;
                if      (turtlePose2.x<arena::x1){req2.x = arena::x2-0.05;}
                else if (turtlePose2.x>arena::x2){req2.x = arena::x1+0.05;}
                if      (turtlePose2.y<arena::y1){req2.y = arena::y2-0.05;}
                else if (turtlePose2.y>arena::y2){req2.y = arena::y1+0.05;}

            ROS_INFO_STREAM("\nturtle2 out of boundary! Teleporting to a new location\n");
            bool success2 = teleportAbsClient2.call(req2,resp2);

            }

        }


        distErr = sqrt( pow(turtlePose2.x-turtlePose1.x,2)+ pow(turtlePose2.y-turtlePose1.y,2) ); //TODO 4: calculate the distance
        angGoal = atan2( (goaly-turtlePose1.y),  (goalx-turtlePose1.x) );
        angErr  = atan2(sin(angGoal-turtlePose1.theta), cos(angGoal-turtlePose1.theta));  // TODO 5: use atan2 to calculate the angular error
//        if (distErr<0.1 && poseInitialized2 && poseInitialized1){
//            ROS_INFO_STREAM(std::fixed << "Mission Completed!");
//            return 0;
//        }
        // Compute the angle angErr and set
        // turtlePose1.theta+=angErr, turtlePose2.theta+=angErr.
        // Hence two turtles turn the same angle to meet.
        dx = turtlePose1.x-turtlePose2.x;
        dy = turtlePose1.y-turtlePose2.y;
        du = -cos(turtlePose1.theta) + cos(turtlePose2.theta);
        dv = -sin(turtlePose1.theta) + sin(turtlePose2.theta);
        angErr = atan2(  (-dv*dx+du*dy) , (du*dx+dv*dy));

        // control law
        if( fabsf(angErr) > 1.0){
            msg1.linear.x = 0.2; // if angular error is large, turn (mostly) in place
            msg2.linear.x = 0.2;
         }else{
            msg1.linear.x = fmin(distErr, 3.0);
            msg2.linear.x = fmin(distErr, 3.0);
         }
        msg1.angular.z = 2*angErr;  //TODO 6: what happens if you change this control gain from 1/2?
        msg2.angular.z = 2*angErr;

        // Publish the message.

        pub1.publish(msg1);
        pub2.publish(msg2);

        // Send a message to rosout with the details.
        ROS_INFO_STREAM(std::setprecision(3) << std::fixed << "vel command:"
            << " linear1=" << msg1.linear.x << " angular1=" << msg1.angular.z
            << " turtle1pos=("<< turtlePose1.x<<","<<turtlePose1.y<<","<<turtlePose1.theta<<"),"
            << " linear2=" << msg2.linear.x << " angular2=" << msg2.angular.z
            << " turtle2pos=("<< turtlePose2.x<<","<<turtlePose2.y<<","<<turtlePose2.theta<<")");
        if(distErr < 0.1 && poseInitialized1 && poseInitialized2 ){ // return if at the goal.
            ROS_INFO_STREAM("Mission Completed!");
            return 0;
        }



        // Wait until it's time for another interaction
        rate.sleep();//
    }
 }
