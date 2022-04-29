/**
* \file keyb.cpp
* \brief Node to drive the robot with the keyboard
* \author Lorenzo Benedetti
* \version 1.0
* \date 12/04/2022
*
* \details
*
* Publishes to: <BR>
* /cmd_vel
*
* Description :
*
* Thanks to this node the user can drive easily the robot inside the environment. 
* The user inputs are checked by the node and the the resulting speed is published on the ROS topic /cmd_vel.
*
**/

//-------------------------------------------------------------NEEDED LIBRARIES----------------------------------------------------------

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <termios.h>

//-------------------------------------------------------------VARIABLES-----------------------------------------------------------------

ros::Publisher publisher; ///< Global publisher for the velocity.

//for velocities
int linear_vel = 0; ///< Global variable for linear velocity.
int angular_vel = 0; ///< Global variable for angular velocity.

//for speed
double speed = 0.4;  ///< Global variable for speed.
double turn_speed = 1.1; ///< Global variable for turn speed.

//for Twist
geometry_msgs::Twist vel; ///< Global variable for the total velocity.

//-------------------------------------------------------------FUNCTIONS-----------------------------------------------------------------

/**
*
* \brief quick_input function
* \return n The integer takes into account the button selected in the keyboard.
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
    
    std::cout << "Use the keyboard to drive the robot in the environment!\n";
	std::cout << R"(Use the buttons below to drive the robot as you prefer!
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
    
    vel.angular.z = angular_vel * turn_speed;
    vel.linear.x = linear_vel * speed;

    //publish in the correct topic
    publisher.publish(vel);

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
    
    ros::init(argc, argv, "keyb");
    ros::NodeHandle node_handle;
	
	//publish into the ROS topic /cmd_vel
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (1){
    
        take_input();
    }
   
	return 0;
}
