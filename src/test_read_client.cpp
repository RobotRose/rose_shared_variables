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


	SharedVariables<int> shared_integers;
	shared_integers.connectToSharedVariable("shared_integer");

	auto timeout = ros::Duration(2.0);
	do{
		ROS_INFO_NAMED(ROS_NAME, "Int is: %d", shared_integers["shasdfared_integer"]->get(timeout));
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());


	return 0;
}




