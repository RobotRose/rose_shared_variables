/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/12
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include <random>

#include <ros/ros.h>

#include "shared_variables/shared_variable.hpp"

using namespace shared_variables;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_write_client");
	ros::NodeHandle n;
	printf("Started shared variable read client.\n");

	SharedVariable<int> shared_integer("shared_integer");
	shared_integer.connect(false);
	std::default_random_engine generator;
	
	int i = 0;
	do{
		if(i++%50 == 0)
		{

			std::uniform_int_distribution<int> distribution(-6516,6464168);
			int dice_roll = distribution(generator);  // generates number in the range 1..6 

			ROS_INFO_NAMED(ROS_NAME, "Setting to %d!", dice_roll);
			shared_integer = dice_roll;
		}
		
		
		ROS_INFO_NAMED(ROS_NAME, "Last set int is: %d", (int)shared_integer);
		
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());

	return 0;
}




