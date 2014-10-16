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

	SharedVariable<int> test_sv("shared_integer");
	test_sv.connect(ros::Duration(5.0));
	
	SharedVariable<std::vector<int>> shared_integer_list("shared_integer_list");
	shared_integer_list.connect(ros::Duration(2.0));

	std::vector<int> list;

	do{
		int a = 0;
		a = test_sv;
		list = shared_integer_list;

		for(const auto& integer : list)
			ROS_INFO_NAMED(ROS_NAME, "  Intlijst int is: %d", integer);

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




