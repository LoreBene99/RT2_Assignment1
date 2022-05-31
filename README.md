# First Assignment of Research Track 2
## Professor : Carmine Recchiuto, University of Genoa

Code Documentation
------------
The project documentation, generated with Doxygen can be found in the following link:

[__lorebene99.github.io/RT2_Assignment1/index.html__](https://lorebene99.github.io/RT2_Assignment1/)

Jupyter notebook
------------
The jupyter notebook can be found here: [__RT2_Assignment1_Jupyter.ipynb__](https://github.com/LoreBene99/RT2_Assignment1/blob/main/RT2_Assignment1_Jupyter.ipynb)

Data Analysis
------------
The results of the data analysis, computed with Matlab, can be found in [__RT2_Statistical_Analysis.pdf__](https://github.com/LoreBene99/RT2_Assignment1/blob/main/RT2_Statistical_Analysis.pdf)

-----------------------
Preface
-------
The main gol of the project, given by the Professor Carmine Recchiuto, is to drive the robot inside a particular environment letting the user decide what type of mode to use to guide the robot inside this map. 
In fact it has to be developed a software architecture for the control of the robot in the environment. The software will rely on the `move_base` and `gmapping packages` for localizing the robot and plan the motion.
The architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user’s input):
1) Autonomously reach a x,y coordinate inserted by the user.
2) Let the user drive the robot with the keyboard.
3) Let the user drive the robot assisting them to avoid collisions.

In order to do this i have created 4 specific nodes:
* control node (in which the user chooses the desired modality). 
* reach node (regarding point 1); the move_base pakage requires goal to be sent to the topic `move_base/goal`, by sending a message of type `move_base_msgs/MoveBaseActionGoal` 
* keyb node (regarding point 2). 
* assistkeyb node (regarding point 3). 
The last two nodes rely on `teleop_twist_keyboard`

Installing and Running
----------------------

The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. First of all in order to run and install the project it is required to have the `slam_gmapping` package, `the ros navigation stack` (sudo apt-get install ros-<your_ros_distro>-navigation)
and `xterm` (sudo apt install xterm).
Then you have to create your own ROS workspace and in the src folder you have to:
* Download the RT2_Assignment1 folder and put it inside the src folder, since the RT2_Assignment1 folder represents the ROS package in which there are the nodes.
* Do catkin_make in the ROOT FOLDER OF YOUR WORKSPACE (Catkin is the official build system of ROS and the successor to the original ROS build system, rosbuild).

At the end, to run the project, i have created a ROS launch file, named final.launch, contained in the launch_file folder : 

```xml
<launch>
    <include file="$(find RT2_Assignment1)/launch/simulation_gmapping.launch"/>
    <include file="$(find RT2_Assignment1)/launch/move_base.launch"/>
    <node pkg="RT2_Assignment1" type="control" name="control" output="screen" required="true" launch-prefix="xterm -e"/>
</launch>
```
To launch the entire project at once, avoding running the single nodes, type: 
#### roslaunch RT2_Assignment1 final.launch

Introduction
------------
The environment in which the robot moves is :

<p align="center">
<img src="https://github.com/LoreBene99/RT2_Assignment1/blob/main/images/env.png" width="550" height="400">
</p>
This represents the point of view from Gazebo, a 3D robot simulator in which we can see the robot moving in a real 3D space. 
It integrates with ROS using ROS messages, services and dynamic reconfigure.

Whereas :

<p align="center">
<img src="https://github.com/LoreBene99/RT2_Assignment1/blob/main/images/map.png" width="550" height="400">
</p>
This represents the point of view from Rviz. 
Rviz is a 3D visualization tool for ROS applications. It offers a view of the robot model, acquires sensor information from the robot sensors, and reproduces the acquired data. It can display data from video cameras, lasers, 3D and 2D devices, including images and point clouds.
To obtain this result the robot must have explored all the surroundings since with the gmapping algorithm we do not have a totale knowledge of the environment, whereas with a pre-existing map we do.

Nodes
-----
### control node

The control node spawn with the launch file. The user has to decide with which modality wants to drive the robot inside the environment. I have done a switch in order to "execute" the particular node that provides the specific behaviour requested by the user to guide the robot. 

0 : Exit 

1 : Reach the position requested by the user  

2 : Use keyboard to drive the robot in the environment

3 : Use keyboard to drive the robot in the environment with automatic collision avoidance 

4 : Reset the simulation, calling the ROS service to reset the simulation and restart from the beginning. 

```cpp
switch(n){ 

			case '0': //exit from main 
				system("clear"); 
				return 0; 
			break; 

			case '1': //launch node1 
				system("rosrun RT2_Assignment1 reach"); 
			break; 

			case '2': //launch node2 
				system("rosrun RT2_Assignment1 keyb"); 
			break; 

			case '3': //launch node3 
				system("rosrun RT2_Assignment1 assistkey"); 
			break; 

			case '4': //reset the simulation 
				ros::service::call("/gazebo/reset_simulation", reset); 
			break; 

			default: 
			break; 
		} 
```



### reach node
The reach node is very important since it regards the first request made by the professor. 
Thanks to this node the user can choose the x and y coordinates to reach a specific point P(x,y). The "goal" has to be sent to the move_base package, a package to move the robot around and to compute and follow the path, so the node will generate the message `move_base_msgs::MoveBaseActionGoal goal` that will be published on the ROS topic `/move_base/goal`.
The robot will follow the path made in order to reach the desired Point. I have made a function called `void inputs()` that detect the x and y insterted by the user. As i said the robot will start its driving during that the user can easily cancel the goal the robot is reaching (with the button "q") or exit directly from the node (with the button "t"):

```cpp
while (1){
    
		//get an input exactly from the user
		char input = getchar();

		//cancel the goal if the user want
		if (input == 'q' && ongoing_goal){
		
		    cancelling_goal(1);
		}
		
		//exit from the node if i press t
		else if (input == 't' && ongoing_goal){
		
		    cancelling_goal(0);
		    system("clear");
		    
		    ros::shutdown();
		    
		    exit(0);
		}
```
If there aren't any onogoing goal then user can choose if he wants to go on or to stop the robot.

```cpp
        else if ((input == 'y' || input == 'n') && !ongoing_goal){
		
		    if (input == 'y'){
		    
		        inputs();
		    }
		    
		    else
		        exit(1);
		}
```
#### cancelling_goal is another function that i made in order to cancel the goal to reach if the user wants, whereas ongoing_goal is a variable needed to know whenever there are inputs from the user during the driving mode of the robot.

Whenever the cancelling_goal function is invoked, the message `actionlib_msgs::GoalID msg_canc` is created and published on the ROS topic `/move_base/cancel`.
The function `void handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)` control the STATUS (checking the messages published on the ROS topic `/move_base/status`) just to know if our robot has reached the Point(x,y) or if the robot can't reach the desired Point(x,y).
#### NB: each goal has its id generated by rand() to simplify the sent of the cancel message.
#### NB: When the robot is moving the status code is equal to 1: the robot is driving, following the path. When the robot stops from moving --> If the status code is 3: the robot has succesfully reached the Point(x,y). If it is 4: the robot can't reach the Point(x,y).

### keyb node
Thanks to this node the user can drive easily the robot inside the environment. The user inputs are checked by the node and the the resulting speed is published on the ROS topic 
/cmd_vel. 
This is the interface: 

<center>

|| Turn left | Don't turn | Turn right|
|:--------:|:--------:|:----------:|:----------:|
|__Go forward__|u|i|o
|__Dont' go__|j|k|l
|__Go backward__|m|,|.

</center>

There is also another set of commands very useful that i implemented: 

* q/z : increase/decrease all speeds by 10%
* w/x : increase/decrease only linear speed by 10%
* e/c : increase/decrease only angular speed by 10%
* r   : reset all speeds
* t : QUIT
The most important function in this node (and the only one) is `void take_input()`, a function to take the input and compute the correct driving behaviour.

### assistkey node
The assistkey node is basically identical to the previous one, but the only difference is very important: the user can drive the robot, thanks to the keyboard, inside the environment, but it must avoid all the possible collisions. In fact the robot should not go forward if there is an obstacle in the front andshould not turn left/right if there are obstacles on the left/right.
The node not only checks the inputs from the user, but checks also the laser scanner of the robot and it subscribes to the ROS topic `/scan` in order to detect the walls. 
The topic provides an array, which returns the distances between the robot and the obstacles; every distance is given by the array ranges[i] (i = 0,...,720) and it is computed considering each of 721 section in which the vision of the robot is divided, since the vision of the robot is included in a spectrum range of 180 degrees in front of it and expressed in radiant. I have separated 3 big subsections (right, left and in front of the robot), inside the 0-720 spectrum, for the vision of the robot and i have computed the minimum distance between the robot and the obstacle for each subsection, implementing the similar logic seen in the previous assignment. A piece of the function `detecting_walls()`:

```cpp

    double right_wall[160];
    double left_wall[160];
    double front_wall[160];
    
    for (int i = 0; i < 160; i++){
    
        right_wall[i] = msg->ranges[i];
    }

    for (int i = 559; i <= 719; i++){
    
        left_wall[i - 559] = msg->ranges[i];
    }

    for (int i = 290; i < 450; i++){
    
        front_wall[i - 290] = msg->ranges[i];
    }
	
```
#### NB : The robot can't get closer to the wall, since the minimum theshold to prevent collisions is 0.9 meter (wall_th).

#### NB : I have personally tested ALL the parameters used in the code. Probably there are better ones.

Flowchart
---------

This is an image that show how nodes are connected to each other (thanks to the command rosrun rqt_graph rqt_graph):

<p align="center">
<img src="https://github.com/LoreBene99/RT2_Assignment1/blob/main/images/ros.png" width="9000" height="150">
</p>

#### reach flowchart

<p align="center">
<img src="https://github.com/LoreBene99/RT2_Assignment1/blob/noetic/images/reach.jpg" width="500" height="600">
</p>

#### keyb flowchart

<p align="center">
<img src="https://github.com/LoreBene99/RT2_Assignment1/blob/main/images/keyb.jpg" width="600" height="250">
</p>

#### assistkey flowchart

<p align="center">
<img src="https://github.com/LoreBene99/RT2_Assignment1/blob/main/images/assistkey.jpg" width="500" height="600">
</p>

Conclusions
-----------
I'm satisfied with the final result, even if better improvements can be done.
First of all some parameters can be changed since they may be not optimal. Having said that, there are 2 particular improvenments that i want to highlight:

* In the assisted driving mode, the robot avoid obstacles in the front/left/right related to its vision field, but since it can go also backwards it will inevitably crush on the back side, not avoiding the wall. Future improvements can be done, in order to avoid hitting the wall on the back side, probably using the geometry and the space of the environment.
* In the reach point node, we may compute the current position of the robot in the environment and match it to the goal position in order to have an instant feedback if the robot has reached the Point(x,y). The ROS topic `base_scan/status` takes a long time to control if the robot has reached the goal.
