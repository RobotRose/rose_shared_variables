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


	SharedVariable<std::vector<int>> shared_integer_list("shared_integer_list");
	shared_integer_list.connect(false);

	std::vector<int> intlijst;
	intlijst.push_back(1);
	intlijst.push_back(2);
	intlijst.push_back(3);
	
	int i = 0;
	do{
		if(i%5 == 0)
		{
			std::uniform_int_distribution<int> distribution(0,100);
			int dice_roll = distribution(generator);  // generates number in the range 1..6 

			ROS_INFO_NAMED(ROS_NAME, "Setting to %d!", dice_roll);
			shared_integer = dice_roll;

			shared_integer_list.push_back(shared_integer_list.back() + 1);
			
		}

		if(i%10 == 0)
		{
			ROS_INFO_NAMED(ROS_NAME, "Erasing front ---------------------------------------------------------------------------------------------------------------");
			if (shared_integer_list.size() > 3)
			{
				shared_integer_list.erase(0);
			}
		}

		if(i%20 == 0)
		{
			ROS_INFO_NAMED(ROS_NAME, "Erasing front ---------------------------------------------------------------------------------------------------------------");
			if (shared_integer_list.size() > 3)
			{
				shared_integer_list = intlijst;
			}
		}

		for(const auto& integer : shared_integer_list.get())
			ROS_INFO_NAMED(ROS_NAME, "  Intlijst int is: %d", integer);
		
		ROS_INFO_NAMED(ROS_NAME, "Last set int is: %d", (int)shared_integer);
		
		i++;
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());

	return 0;
}




