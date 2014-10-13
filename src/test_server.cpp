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
#include "ros/service_server.h"

#include "shared_variables/shared_variables.hpp"

using namespace shared_variables;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_server");
	ros::NodeHandle n;
	printf("Started shared variable test server.\n");
	
	SharedVariables<int> shared_integers;
	shared_integers.hostSharedVariable("shared_integer");

	do{
		printf("Setting to 42\n");
		shared_integers["shared_integer"]->set(42);

		ros::Duration(2.0).sleep();
		ros::spinOnce();
	} while(ros::ok());
	
	ros::spin();

	return 0;
}
