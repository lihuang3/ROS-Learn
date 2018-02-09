### ROS Tutorial Step-by-Step
### Program List
- [`pubvel.cpp`](src/pubvel.cpp) Turtlesim with translation and rotation speed change.
- [`turtle00.cpp`](src/turtle00.cpp) Turtlesim with closed-loop control and goal location is set as (0,0).
- [`turtlexy.cpp`](src/turtlexy.cpp) Turtlesim with closed-loop control with user-input goal location.
- [`turtlexyquiz.cpp`](src/turtlexyquiz.cpp) Turtlesim with closed-cloop control (better version)
- [`turtlemeet.cpp`](src/turtlemeet.cpp)
- [`turtlemeet.launch`](src/turtlemeet.launch)
- [roslaunch Doc](http://wiki.ros.org/roslaunch/XML)

#### 0. Useful Terminal Command
- `roscore` initialize a ROS master
- `rostopic` list published topics
- `rosnode info`
- `rqt_graph`
- `rosrun turtlesim turtlesim_node`
- `rosrun turtlesim turtle_teleop_key`

#### 1. Creating a workspace and a package
ROS initialization:

```
source /opt/ros/knitic/setup.bash 
```
**1.1** __Creating and building and catkin workspace:__
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
This gives you a `CMakeLists.txt` in your '`src`' folder, as well as a '`build`' and '`devel`' folder. 

**1.2** __Creating a catkin package__

Run from the '`src`' directory:
```angular2html
cd ~/catkin_ws/src
catkin_create_pkg <package_name>
``` 
This gives you a `CMakelists.txt` and a `package.xml` file.
- `package.xml`: the manifest
- `CMakeLists.txt`: it contains a list of build instructions including what executables 
should be created, what source files to use to build each of them, and where to find 
the include files and libraries needed for those executables.
- POS package names allows only lowercase letters, digits and underscores.

**1.3** __Hello, ROS!__

**1.3.1** __Compiling__
Before compiling the program, you need to 
**[declaring dependencies](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)** 
and **decalring an executable**. Start `roscore` first.

We need to modify `CMakeLists.txt` and `package.xml` before `catkin_make`. For examplem, if you are using
a message type from the `geometry_msgs` package, we must declare a dependency on that package. We modify the 
`find_package` line in `CMakeLists.txt`to mention geometry_msgs in addi-
tion to roscpp: 
```angularjs
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
```

**1.3.2** __Building and Excuting__

**Building the workspace** `catkin_make` compiling all of the executables in all of its packages. 
Because it's designed to build all of the packages in your workspace, this command must be run from 
from your workspace directory.

**Sourcing `setup.bash`** `source devel/setup.bash` Unless the directory structure changes, you only need to
run this once in each terminal.

**Executing the program**
```rosrun <program_name>``` 

#### 2. Publisher and Subscriber  
**2.1** __A Publisher Program__

**Creating a publisher object**

The work of actually publishing the messages is done by an
object of class ros::Publisher.
```
ros::Publisher pub = node_handle.advertise<message_type>(topic_name, queue_size);
```
If you want to publish messages on multiple topics from the same node, you’ll need to
create a separate ros::Publisher object for each topic.

**Creating and filling in the message object**

Use `rosmsg show` to see that certain message type's fields and sub-fields. For example, 
geometry_msgs/Twist message type has two top-level fields ( linear and angular ), each of which contains three sub-fields
( x , y , and z ). 

To create a a message class, and fill in the contents of the message: 

```
geometry_msgs::Twist msg;
msg.linear.x = 1.0;
msg.angullar.z = 0.0;

```
**Publishing the message** using the publish method of the `ros::Publisher` object:
```angularjs
pub.publish(msg);
```
This method adds the given msg the publisher’s outgoing message queue, from which it
will be sent as soon as possible to any subscribers of the corresponding topic.


**2.2** __The Publishing Loop__

To construct a publishing while loop, we check for node shutdown:
```angularjs
ros::ok()
```
It will return true , until the node has some reason to shut down. There are a few
ways to get ros::ok() to return false:
- you can use `rosnode kill` on the node.
- You can send an interrupt signal ( Ctrl-C ) to the program.
- You can call, somewhere in the program itself,
`ros::shutdown()`. This function can be a useful way to signal that your node’s work is complete from
deep within your code.
- You can start another node with the same name. This usually happens if you start a
new instance of the same program.

To control the publishing rate:
```angularjs
ros:Rate rate(2);
``` 
This object controls how rapidly the loop runs in Hz. Near the end of each loop iteration, we call
the sleep method of this object:
```
rate.sleep();
```
Each call to the this method causes a delay in the program. The duration of the delay is calculated 
to prevent the loop from iterating faster than the specified rate.

**2.3** __A Subscriber Program__

**Writing a callback function**
A subscriber node doesn’t know 
when messages will arrive. We must place any code that responds to incoming 
messages inside a callback function, which ROS calls once for each arriving message.
```
void function_name(const package_name::type_name &msg) {
        ...
    }
```
Notice that subscriber callback functions have a void return type.

**Creating a subscriber object**
```angularjs
ros::Subscriber sub = node_handle.subscribe(topic_name, queue_size, pointer_to_callback_function);
```
When we construct a ros::Subscriber , our node establishes connections
with any publishers of the named topic.

**Giving ROS control**  The final complication is that ROS will only execute our callback
function when we give it explicit permission to do so. Does your program have any repetitive work to do, other than responding to callbacks? 
- `ros::spin()` if the answer is “No"; 
- `ros::spinOnce()` if the answer is “Yes,” then a reasonable option is to
write a loop that does that other work and calls `ros::spinOnce()` periodically to process
callbacks.


#### 4. Parameters
In addition to the messages that we’ve studied so far, ROS provides another mechanism
called parameters to get information to nodes. 
The idea is that a centralized parameter server keeps track of a collection of values—things like integers, floating point numbers,
strings, or other data—each identified by a short string name.
Because parameters must be actively queried by the nodes that are interested in their values, they are most
suitable for configuration information that will not change (much) over time.

**4.1** __Accessing Parameters from the Command Line__
- `rosparam list` Listing parameters
- `rosparam get parameter_name` Querying the parameter sever for the value of a parameter.  
  `rosparam get namespace` retrieve the values of every parameter in a namespace.    
  `rosparam get /` by asking about the global namespace, we can see the values of every parameter all at once
- `rosparam set parameter_name parameter_value` assign a value to a parameter. This command can modify the values of existing parameters or create new ones.
- 
  
The important thing to notice here is that updated parameter values are not automatically “pushed” to nodes. Instead, nodes that care about changes to some or all of their
parameters must explicitly ask the parameter server for those values.

**4.2** __ Accessing Parameters from C++__
- `void ros::param::set(parameter_name, input_value);`
- `bool ros::param::get(parameter_name, output_value);`
   
In both cases, the parameter name is a string, which can be a global, relative, or private
name. The input value for set can be a std::string , a bool , an int , or a double ; the output
value for get should be a variable (which is passed by reference) of one of those types.
The get function returns true if the value was read successfully and false if there was a
problem, usually indicating that the requested parameter has not been assigned a value.

Here's an example of C++ parameter set:
<p align="center">
<img src="https://github.com/lihuang3/ROS-Learn/blob/master/Images/ROS_Set_Param.png" width="625">
</p>

#### 5. Service
**5.1** __Create a Client Object__
- Create a Client Object
```
ros::ServiceClient teleportAbsClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
```
- Create the request and response objects
```angularjs
turtlesim::TeleportAbsolute::Request reqt;
turtlesim::TeleportAbsolute::Response respt;
```
- Assign value to request variables
```angularjs
reqt.x = 10*double(rand())/double(RAND_MAX);
reqt.y = 10*double(rand())/double(RAND_MAX);
reqt.theta = 0;
```
- Actually call the service. This won't return until the service is complete.
```angularjs
bool success = teleportAbsClient.call(reqt,respt);
```


