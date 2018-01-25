### ROS Tutorial Step-by-Step
### Program List
- [`pubvel.cpp`](pubvel.cpp) Turtlesim with translation and rotation speed change.

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

Before compiling the program, you need to 
**[declaring dependencies](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)** 
and **decalring an executable**. Start `roscore` first.

**Building the workspace** `catkin_make` compiling all of the executables in all of its packages. 
Because it's designed to build all of the packages in your workspace, this command must be run from 
from your workspace directory.

**Sourcing `setup.bash`** `source devel/setup.bash` Unless the directory structure changes, you only need to
run this once in each terminal.

**Executing the program**
```rosrun <program_name>``` 
  
**1.4** __A Publisher Program__
