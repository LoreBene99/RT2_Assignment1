/**
* \file control.cpp
* \brief Node for choosing the driving modality
* \author Lorenzo Benedetti
* \version 1.0
* \date 12/04/2022
*
* \details
*
* Services : <BR>
* /gazebo/reset_simulation It is needed to reset the simulation
*
* Description :
*
* The control node spawn with the launch file. The user has to decide with which modality wants to drive the robot inside the environment. 
* A switch has been made in order to "execute" the particular node that provides the specific behaviour requested by the user to guide the robot.
*
**/

//------------------------------------------------------------NEEDED LIBRARIES------------------------------------------------------------- 

#include "ros/ros.h" 
#include "std_srvs/Empty.h"

/** 
* \brief Define color for the interface.
*   
**/

#define RED "\033[1;31m"

/** 
* \brief Define color for the interface.
*   
**/

#define NC "\033[0m"

std_srvs::Empty reset; ///< Global variable to reset the simulation.

std::string menu = R"( 

Please, choose the modality you desire to use in order to control the robot: 
---------------------------------------------------------------------------- 
0 : Exit 
1 : Reach the position 
2 : Use keyboard to drive the robot in the environment 
3 : Use keyboard to drive the robot in the environment with automatic collision avoidance 
4 : Reset the simulation 
)"; ///< Global variable to print out the MENU to choose the modality.

//------------------------------------------------------------MAIN------------------------------------------------------------------------- 

/**
*
* \brief Main function
* \param  argc An integer argument count of the command line arguments.
* \param  argv An argument vector of the command line arguments.
* \return An integer 0 upon exit success.
* 
* The Main function contains the core of the project. In the main function
* there is a switch function used to let the user choose the driving modality 
* through the utilization of the n button on the keyboard.
*
**/

int main(int argc, char **argv){ 

	ros::init(argc, argv, "control"); 

	//while loop 
	while (1){
	
		system("clear"); 
		
		std::cout << RED << "Welcome to the Robot simulation developed by Lorenzo Benedetti!\n" << NC; 
		std::cout << menu; 
	
		int n; 

		//get the character 
		n = getchar();  
		
		// I have done a switch function in order to check the button pressed to execute the particular 
		// node related to the precise modality through which the usere drive the robot in the environment
		
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
	}
	
	return 0; 
}
