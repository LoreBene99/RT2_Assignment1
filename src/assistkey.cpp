/**
* \file assistkey.cpp
* \brief Node to drive the robot with the keyboard in assisted mode
* \author Lorenzo Benedetti
* \version 1.0
* \date 12/04/2022
*
* \details
*
* Subscribes to: <BR>
* /scan
*
* Publishes to: <BR>
* /cmd_vel
*
* Description :
* Publishes to: <BR>
* /cmd_vel
*
* Description :
*
* The assistkey node is basically identical to the previous one, but the only difference is very important: the user can drive the robot, thanks to the keyboard, 
* inside the environment, but it must avoid all the possible collisions. In fact the robot should not go forward if there is an obstacle in the front and 
* should not turn left/right if there are obstacles on the left/right.
*
**/

//-------------------------------------------------------------NEEDED LIBRARIES----------------------------------------------------------

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <termios.h>

//-------------------------------------------------------------VARIABLES-----------------------------------------------------------------

ros::Publisher publisher; ///< Global publisher for the velocity.

//variable needed in order to maintain a certain distance from the wall
double wall_th = 0.9; ///< Global variable to maintain a certain distance from the wall.

//for velocities
int linear_vel = 0; ///< Global variable for linear velocity.
int angular_vel = 0; ///< Global variable for angular velocity.

//for speed
double speed = 0.4; ///< Global variable for speed.
double turn_speed = 1.1; ///< Global variable for turn speed.

//for Twist
geometry_msgs::Twist vel; ///< Global variable for the toal velocity.

//-------------------------------------------------------------FUNCTION------------------------------------------------------------------

/**
* \brief min_value function 
* \param array An array that contains the distances between the robot and the obstacles.
* \return A parameter that represents the minimum value of that array.
* 
* The function returns the minumim value of a given array and it is needed to return the minimum distance from the robot to the wall 
* considered in that particular subsection (front, left, right).  
*
**/

double min_value(double array[]){
	//Returns the minumim value of a given array and it is needed to return the minimu distance from the robot to the wall considered in that particular subsection 	
	//(front, left, right)  

    double min_value = 100;
    
    for (int i = 0; i < 160; i++){
    
        if (array[i] < min_value){
        
            min_value = array[i];
        }
    }
    
    return min_value;
}

//The node not only checks the inputs from the user, but checks also the laser scanner of the robot and it subscribes to the ROS topic /scan in order to detect the 
//walls. The topic provides an array, which returns the distances between the robot and the obstacles; every distance is given by the array ranges[i] (i = 0,...,720) 
//and it is computed considering each of 721 section in which the vision of the robot is divided, since the vision of the robot is included in a spectrum range of 180 
//degrees in front of it and expressed in radiant. I have separated 3 big subsections (right, left and in front of the robot).

/**
*
* \brief detecting_wall function
* \param msg The param is related to the varibales that the robot gives through its laser.   
* 
* The function checks the walls on the right, left and in front of it thanks to the scanner of the robot, 
* checking if the robot is close to a wall.
*
**/

void detecting_wall(const sensor_msgs::LaserScan::ConstPtr &msg){
	//the function check the walls thanks to the scanner of the robot 

	//check the walls on the right, left and front 
	
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
    
    //the robot can only go straight if there are walls near it (right and left)
    if (((min_value(right_wall) < wall_th) && angular_vel < 0) || (((min_value(left_wall) < wall_th) && angular_vel > 0))){
    
        angular_vel = 0;
    }

    //the robot can only turn since there is a wall in front of it
    if (min_value(front_wall) < wall_th && linear_vel > 0){
    
        linear_vel = 0;
    }
    
    if ((min_value(front_wall) < wall_th && linear_vel > 0) || ((min_value(left_wall) < wall_th) && angular_vel > 0) || ((min_value(right_wall) < wall_th) && angular_vel < 0)){
    
        std::cout << "There is a wall near me!\n";
    }
    
    vel.angular.z = angular_vel * turn_speed;
    vel.linear.x = linear_vel * speed;

    //publish the new speed to the relative topic
    publisher.publish(vel);
}

/**
*
* \brief quick_input function
* \return The integer n that takes into account the button selected in the keyboard.
* 
* The quick_input function is a function needed to avoid to press enter every time the user 
* press a button on the keyboard.
*
**/

int quick_input(void){
	//function to not press enter every time i press a button
 
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);
    
    int n;
    n = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return n;
}

/**
*
* \brief take_input function
* 
* The take_input function is a function needed to avoid to take the input
* and compute the correct driving beahviour.
*
**/

void take_input(){
	//function to take the input and compute the correct driving behaviour

    char input;
    
	std::cout << R"(Use the keyboard to drive the robot in the environment
   u    i    o
   j    k    l
   m    ,    .
Any other pressed button will stop the robot.
Press:
q/z : increase/decrease all speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
r   : reset all speeds
t : QUIT
)";

    input = quick_input();
	
	//switch function in order to associate the input given by the user to the correct command to execute 
	switch (input){
	
		case 'u':
		    linear_vel = 1;
		    angular_vel = 1;
		break;
		
		case 'i':
		    linear_vel = 1;
		    angular_vel = 0;
		break;
		
		case 'o':
		    linear_vel = 1;
		    angular_vel = -1;
		break;
		
		case 'j':
		    linear_vel = 0;
		    angular_vel = 1;
		break;
		
		case 'k':
		    linear_vel = 0;
		    angular_vel = 0;
		break;
		
		case 'l':
		    linear_vel = 0;
		    angular_vel = -1;
		break;
		
		case 'm':
		    linear_vel = -1;
		    angular_vel = 1;
		break;
		
		case ',':
		    linear_vel = -1;
		    angular_vel = 0;
		break;
		
		case '.':
		    linear_vel = -1;
		    angular_vel = -1;
		break;
		
		case 'q':
		    speed *= 1.1;
		    turn_speed *= 1.1;
		break;
		
		case 'z':
		    speed *= 0.9;
		    turn_speed *= 0.9;
		break;
		
		case 'w':
		    speed *= 1.1;
		break;
		
		case 'x':
		    speed *= 0.9;
		break;
		
		case 'e':
		    turn_speed *= 1.1;
		break;
		
		case 'c':
		    turn_speed *= 0.9;
		break;
		
		case 'r':
		    speed = 0.4;
		    turn_speed = 1.1;
		break;
		
		case 't':
			//exit
		    vel.angular.z = 0;
		    vel.linear.x = 0;
		    
		    publisher.publish(vel);
		    
		    system("clear");
		    
		    ros::shutdown();
		    
		    exit(0);
		break;
		
		default:
		    linear_vel = 0;
		    angular_vel = 0;
		break;
    }
    
    system("clear");
}
 
//-------------------------------------------------------------MAIN----------------------------------------------------------------------

/**
*
* \brief Main function
* \param  argc An integer argument count of the command line arguments.
* \param  argv An argument vector of the command line arguments.
* \return an integer 0 upon exit success.
* 
* The Main function contain the core of the project.
*
**/

int main(int argc, char **argv){

    system("clear");
    
    ros::init(argc, argv, "assistkey");
    ros::NodeHandle node_handle;
	
	//subscribing to the ROS topic /scan
    ros::Subscriber subscriber = node_handle.subscribe("/scan", 500, detecting_wall);
	
	//publishing into the ROS topic /cmd_vel
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    while (1){
    
    	take_input();
    }
    
    spinner.stop();

    return 0;
}
