/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/10
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/

#include <ros/ros.h>

#include "shared_variables/shared_variables.hpp"

using namespace shared_variables;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_read_client");
	ros::NodeHandle n;
	printf("Started shared variable read client.\n");

	auto timeout = ros::Duration(2.0);

	SharedVariables<int> shared_integers;
	shared_integers.connect("shared_integer");

	// SharedVariables<std::vector<int>> shared_integer_list;
	// SharedVariables<std::vector<std::vector<int>>> shared_integer_list;
	SharedVariables<std::vector<std::vector<std::vector<int>>>> shared_integer_list;
	shared_integer_list.connect("shared_integer_list");

	std::vector<int> list;
	std::vector<std::vector<int>> list_of_lists;
	std::vector<std::vector<std::vector<int>>> list_of_lists_of_lists;

	do{
		// list = shared_integer_list["shared_integer_list"]->get();
		// list_of_lists = shared_integer_list["shared_integer_list"]->get();
		list_of_lists_of_lists = shared_integer_list["shared_integer_list"]->get();

		int i = 0;
		for(const auto& list_of_lists : list_of_lists_of_lists)
		{
			ROS_INFO_NAMED(ROS_NAME, "Level 1 inlist #%d", i++);
			int j = 0;
			for(const auto& list : list_of_lists)
			{
				ROS_INFO_NAMED(ROS_NAME, " Level 2 intlist #%d", j++);
				for(const auto& integer : list)
					ROS_INFO_NAMED(ROS_NAME, "  Intlijst int is: %d", integer);
			}
		}
			

		ROS_INFO_NAMED(ROS_NAME, "Looping %d", shared_integers["shared_integer"]->get());
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());


	return 0;
}




