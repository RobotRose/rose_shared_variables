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

#include "shared_variables/shared_variable.hpp"

#include <geometry_msgs/Point.h>

using namespace shared_variables;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_read_client");
	ros::NodeHandle n;
	printf("Started shared variable read client.\n");

	auto timeout = ros::Duration(1.0);

	// shared_integers.connect("shared_integer");
	SharedVariable<int> test_sv("shared_integer");
	test_sv.connect(ros::Duration(2.0));
	SharedVariable<std::vector<int>> shared_integer_list("shared_integer_list");
	// SharedVariables<std::vector<std::vector<int>>> shared_integer_list;
	// SharedVariables<std::vector<std::vector<std::vector<int>>>> shared_integer_list;
	shared_integer_list.connect(ros::Duration(0.0));

	std::vector<int> list;
	std::vector<std::vector<int>> list_of_lists;
	std::vector<std::vector<std::vector<int>>> list_of_lists_of_lists;

	do{
		int a = 0;
		// a = test_sv;
		// list = shared_integer_list;
		a = shared_integer_list[0] + shared_integer_list[0] + shared_integer_list[0] + shared_integer_list[0] + shared_integer_list[0];
		// list_of_lists = shared_integer_list["shared_integer_list"]->get();
		// list_of_lists_of_lists = shared_integer_list["shared_integer_list"]->get();

		// int i = 0;
		// for(const auto& list_of_lists : list_of_lists_of_lists)
		// {
		// 	ROS_INFO_NAMED(ROS_NAME, "Level 1 inlist #%d", i++);
		// 	int j = 0;
		// 	for(const auto& list : list_of_lists)
		// 	{
		// 		ROS_INFO_NAMED(ROS_NAME, " Level 2 intlist #%d", j++);
				for(const auto& integer : list)
					ROS_INFO_NAMED(ROS_NAME, "  Intlijst int is: %d", integer);
		// 	}
		// }
			

		// ROS_INFO_NAMED(ROS_NAME, "Looping %d", (int)test_sv);//shared_integers["shared_integer"]);
		ROS_INFO_NAMED(ROS_NAME, "Looping %d", a);
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());


	int i = 42;
	std_msgs::Int32 msg;

	std_msgs::Int32 int32_msg;
	int integer;

	int32_msg 				= ros::conversion::convert<int>().get(i);
	std_msgs::String p		= ros::conversion::special_convert<int, std_msgs::String>().get(i);
	integer  				= ros::conversion::convert<int>().get(msg);



	return 0;
}




