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
	ros::NodeHandle n;
	printf("Started shared variable read client.\n");

	SharedVariables<int> shared_integers;
	shared_integers.connectToSharedVariable("shared_integer");

	do{
		shared_integers["shared_integer"]->set(100);
		
		
		ROS_INFO_NAMED(ROS_NAME, "Int is: %d", shared_integers["shared_integer"]->get());
		ros::Duration(2.0).sleep();
		ros::spinOnce();
	} while(ros::ok());

	return 0;
}




