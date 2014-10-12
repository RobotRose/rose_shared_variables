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

#include <ros/ros.h>

#include "shared_variables/shared_variables.hpp"

using namespace shared_variables;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_write_client");

	printf("Started shared variable read client.\n");



	// SharedVariable<int32_t> int_sv("ns", "var_name");
	// SharedVariable<bool> bool_sv("ns", "var_name");
	// SharedVariable<std::vector<int>> vector_sv("ns", "var_name");

	//ros::async_spinner();
	// std::vector<int> integer_list;
	// integer_list.push_back(1001);

	// int_sv.set(0);
	do{
		// int_sv.set(100);
		
		
		// ROS_INFO_NAMED(ROS_NAME, "Int is: %d", int_sv.getCached());
		ros::Duration(2.0).sleep();
		ros::spinOnce();
	} while(ros::ok());

	
	// bool_sv.set(true);
	// vector_sv.set(integer_list);

	// ROS_INFO_NAMED(ROS_NAME, "Bool is: %d", bool_sv.get());
	// ROS_INFO_NAMED(ROS_NAME, "Vector is: %d", vector_sv.get().front());


	return 0;
}




