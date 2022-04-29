/**
* \file reach.cpp
* \brief Node for reaching x and y coordinates by the robot
* \author Lorenzo Benedetti
* \version 1.0
* \date 12/04/2022
*
* \details
*
* Subscribes to: <BR>
* /move_base/status
*
* Publishes to: <BR>
* /cmd_vel
*
* Description :
* Publishes to: <BR>
* /move_base/goal
* /move_base/cancel
*
* Description :
*
* Thanks to this node the user can choose the x and y coordinates to let the robot reach a specific point P(x,y) in the environment.
*
**/

//-----------------------------------------------------------NEEDED LIBRARIES-------------------------------------------------------

#include "ros/ros.h"
#include "actionlib_msgs/GoalID.h"
#include <time.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"

ros::Subscriber subscriber; ///< Global subscriber for the status. 

ros::Publisher pub_goal; ///< Global publisher the goal.
ros::Publisher pub_canc; ///< Global publisher to cancel the goal.

//-----------------------------------------------------------VARIABLES--------------------------------------------------------------

//variables regarding the point to reach
float x; ///< Global variable for x coordinate. 
float y; ///< Global varibale for y coordinate.

//goal to send to the move_base package, a package to move the robot around and to compute and follow the path
move_base_msgs::MoveBaseActionGoal goal; ///< Global variable for the goal to send to move_base package

//it is used to know whenever there are inputs from the user during the driving mode of the robot
int ongoing_goal; ///< Global variable to detect input from the user.

int id; ///< Global variable for id.

//functions used
void cancelling_goal(int go_on);
void inputs();
void handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

//-----------------------------------------------------------FUNCTIONS--------------------------------------------------------------

/**
*
* \brief cancelling_goal function 
* \param go_on The variable is needed for the driving mode of the robot. 
* 
* cancelling_goal is another function that has been made in order to cancel the goal to reach if the user wants, whereas ongoing_goal 
* is a variable needed to know whenever there are inputs from the user during the driving mode of the robot.
*
**/

void cancelling_goal(int go_on){
	//I have made this function in order to cancel the goal that the robot has to reach if the user wants 

    //create the cancel message
    actionlib_msgs::GoalID msg_canc;
    
    msg_canc.id = std::to_string(id);
    
    pub_canc.publish(msg_canc);

    subscriber.shutdown();

    //no goal is in progress so much so:
    ongoing_goal = 0;

    if (!go_on){
    
        return;
    }

    system("clear");

    std::cout << "AUTOMATIC DRIVING! \n";
    
    std::cout << "GOAL CANCELLED! \n";

    std::cout << "SET A GOAL: Y/N \n";

    //Through this while we can obtain the input inserted by the user 
    while (1){
    
		//get an input exactly from the user
		char input = getchar();

		//cancel the goal if the user want and if there is an ongoing goal
		if (input == 'q' && ongoing_goal){
		
		    cancelling_goal(1);
		}
		
		//exit from the node if the user press t
		else if (input == 't' && ongoing_goal){
		
		    cancelling_goal(0);
		    system("clear");
		    
		    ros::shutdown();
		    
		    exit(0);
		}
		
		//if there aren't any onogoing goal then user can choose if he wants to go on or to stop
		else if ((input == 'y' || input == 'n') && !ongoing_goal){
		
		    if (input == 'y'){
		    
		        inputs();
		    }
		    
		    else
		        exit(1);
		}
	}
}

/**
* \brief inputs function 
*
* The function simply detect the x and y coordinates inserted by the user. 
*
**/

//I have made a function called void inputs() that detect the x and y insterted by the user
void inputs(){

    system("clear");
    
    std::cout << "AUTOMATIC DRIVING! \n";

    std::cout << "Insert the x variable of the point to reach: ";
    std::cin >> x;
    
    while (!std::cin.good()){
 
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        
        std::cout <<"AUTOMATIC DRIVING! \n";
        
        std::cout << "Insert the x variable of the point to reach: ";
        std::cin >> x;
    }
    
    system("clear");
    
    std::cout << "AUTOMATIC DRIVING! \n";

    std::cout << "Insert the y variable of the point to reach: ";
    std::cin >> y;
    
    while (!std::cin.good()){
    
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        
        std::cout << "AUTOMATIC DRIVING! \n";
        
        std::cout << "Insert the y variable of the point to reach: ";
        std::cin >> y;
    }

    //create the goal message
    id = rand();
  
    goal.goal.target_pose.pose.orientation.w = 1;
    
    goal.goal.target_pose.pose.position.x = x;
    goal.goal.target_pose.pose.position.y = y;
    
    goal.goal.target_pose.header.frame_id = "map";
    
    goal.goal_id.id = std::to_string(id);

    //publish
    pub_goal.publish(goal);

    //There is an ongoing goal!!!
    ongoing_goal = 1;

    system("clear");

    std::cout << "AUTOMATIC DRIVING! \n";

    std::cout << "\nGOAL SET:\n";
	std::cout << "x is : " << x <<"\n"; 
	std::cout << "y is : " << y <<"\n"; 
	 
    std::cout << "q : CANCEL GOAL; t : QUIT \n";

	//Whenever the cancelling_goal function is invoked, the message actionlib_msgs::GoalID msg_canc is created and published on the ROS topic /move_base/cancel. 
    ros::NodeHandle node_handle;
    subscriber = node_handle.subscribe("/move_base/status", 500, handler);

    //taking inputs
    ros::AsyncSpinner spinner(4);
	spinner.start();
    
    while (1){
    
    	//get an input
		char input = getchar();

		//if there aren't any onogoing goal then user can choose if he wants to go on or to stop
		if (input == 'q' && ongoing_goal){
		
		    cancelling_goal(1);
		}
		
		else if (input == 't' && ongoing_goal){
		
		    cancelling_goal(0);
		    system("clear");
		    
		    ros::shutdown();
		    
		    exit(0);
		}
		
		//if there aren't any onogoing goal then user can choose if he wants to go on or to stop
		else if ((input == 'y' || input == 'n') && !ongoing_goal){
		
		    if (input == 'y'){
		    
		        inputs();
		    }
		    
		    else
		        exit(1);
		}
	}
	
	spinner.stop();
}

//The function void handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) control the STATUS (checking the messages published on the ROS topic /move_base/
//status) just to know if our robot has reached the Point(x,y) or if the robot can't reach the desired Point(x,y).
//When the robot is moving the status code is equal to 1: the robot is driving, following the path. When the robot stops from moving --> If the status code is 3: the 
//robot has succesfully reached the Point(x,y). If it is 4: the robot can't reach the Point(x,y).

/**
* \brief handler function
* \param msg The param is related to the robot status.
* 
* The function void handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) control the STATUS (checking the messages published on 
* the ROS topic /move_base/status) just to know if our robot has reached the Point(x,y) or if the robot can't reach the desired Point(x,y).
*
**/

void handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){

    //status 
    int status = 0;
    
    if (msg->status_list[0].goal_id.id == std::to_string(id)){
    
        status = msg->status_list[0].status;
    }

    if (status != 3 && status != 4){
    
        return;
    }
    
    subscriber.shutdown();

    ongoing_goal = 0;

    system("clear");

    std::cout <<"AUTOMATIC DRIVING! \n";

    if (status == 3){ 

        std::cout << "I've reached the point! \n";
    }
        
    else{
    
        std::cout << "Sorry :(, i can't reach the goal! \n";
    }

    std::cout << "\nSET A GOAL: Y/N \n";
}

//-----------------------------------------------------------MAIN-------------------------------------------------------------------

/**
*
* \brief Main function
* \param  argc An integer argument count of the command line arguments.
* \param  argv An argument vector of the command line arguments.
* \return An integer 0 upon exit success.
* 
* The Main function contain the core of the project.
*
**/

int main(int argc, char **argv){

    srand(time(NULL));

    ros::init(argc, argv, "reach");
    ros::NodeHandle node_handle;

    //this will send messages to goal and cancel
    pub_goal = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    pub_canc = node_handle.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    //get the goal point coordinates
    inputs();

    ros::spin();

    return 0;
}

