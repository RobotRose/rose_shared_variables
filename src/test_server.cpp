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

#include "shared_variables/shared_variable.hpp"

using namespace shared_variables;



void test_function(int& reference_to_integer)
{

	reference_to_integer += 5;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_server");
	ros::NodeHandle n;
	printf("Started shared variable test server.\n");
	
	SharedVariable<int> shared_integer("shared_integer");
	shared_integer.host(false, false);

	SharedVariable<std::vector<int>> shared_integer_list("shared_integer_list");
	shared_integer_list.host(false, false);

	std::vector<int> intlijst;
	intlijst.push_back(1);
	intlijst.push_back(2);
	intlijst.push_back(3);

	shared_integer 		 	= 0;
    shared_integer_list 	= intlijst;

	do{
		shared_integer = shared_integer + 1;
        shared_integer_list[0] = shared_integer_list[1] + 1;
        shared_integer_list[1] = shared_integer_list[0] + 1;
        
        //test_function(shared_integer_list[2]);

        for(const auto& integer : shared_integer_list.get())
			ROS_INFO_NAMED(ROS_NAME, "  Intlijst int is: %d", integer);

		ROS_INFO_NAMED(ROS_NAME, "Looping");
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());
	
	ros::spin();

	return 0;
}
