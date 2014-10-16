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

	reference_to_integer += 100;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "shared_variable_test_server");
	ros::NodeHandle n;
	printf("Started shared variable test server.\n");
	
//	SharedVariable<int> shared_integer("shared_integer");
//	shared_integer.host(false, false);

	SharedVariable<std::vector<int>> shared_integer_list("shared_integer_list");
	// SharedVariables<std::vector<std::vector<int>>> shared_integer_list;
	// SharedVariables<std::vector<std::vector<std::vector<int>>>> shared_integer_list;
	shared_integer_list.host(false, false);

	std::vector<int> intlijsta;
	intlijsta.push_back(1);
	intlijsta.push_back(1);
	intlijsta.push_back(1);

	std::vector<int> intlijstb;
	intlijstb.push_back(100);
	intlijstb.push_back(200);
	intlijstb.push_back(300);

	std::vector<int> intlijstc;
	intlijstc.push_back(1000);
	intlijstc.push_back(2000);
	intlijstc.push_back(3000);

	std::vector<int> intlijstd;
	intlijstd.push_back(1000);
	intlijstd.push_back(2000);
	intlijstd.push_back(3000);


	std::vector<std::vector<int>> list_of_listsa;
	list_of_listsa.push_back(intlijsta);
	list_of_listsa.push_back(intlijstb);

	std::vector<std::vector<int>> list_of_listsb;
	list_of_listsb.push_back(intlijstc);
	list_of_listsb.push_back(intlijstd);

	std::vector<std::vector<std::vector<int>>> list_of_lists_of_lists;
	list_of_lists_of_lists.push_back(list_of_listsa);
	list_of_lists_of_lists.push_back(list_of_listsb);
//	shared_integer = 0;

	// shared_integer_list.MyFunction();
    shared_integer_list = intlijsta;
	do{
//		shared_integer = shared_integer + 1;
        shared_integer_list[0] = shared_integer_list[0] + 1;
        // test_function(shared_integer_list[0]);
        // shared_integer_list[2] = 1;
		// shared_integer_list.push_back(4);
		// shared_integer_list["shared_integer_list"]->set(list_of_lists);
		// shared_integer_list["shared_integer_list"]->set(list_of_lists_of_lists);
		// shared_integer_list[2] = 3;
		// intlijst.push_back(shared_integer->get());


        ROS_INFO_NAMED(ROS_NAME, "shared_integer_list[0]  %d", (int)shared_integer_list[0] );
        ROS_INFO_NAMED(ROS_NAME, "shared_integer_list[1]  %d", (int)shared_integer_list[1] );
        ROS_INFO_NAMED(ROS_NAME, "shared_integer_list[2]  %d", (int)shared_integer_list[2] );

		ROS_INFO_NAMED(ROS_NAME, "Looping");
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());
	
	ros::spin();

	return 0;
}
