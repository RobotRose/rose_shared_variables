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
	shared_integers.host("shared_integer", true, true);

	SharedVariables<std::vector<int>> shared_integer_list;
	// SharedVariables<std::vector<std::vector<int>>> shared_integer_list;
	// SharedVariables<std::vector<std::vector<std::vector<int>>>> shared_integer_list;
	shared_integer_list.host("shared_integer_list");

	std::vector<int> intlijsta;
	intlijsta.push_back(42);
	intlijsta.push_back(84);
	intlijsta.push_back(168);

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

	shared_integers["shared_integer"]->set(1);

	do{
		shared_integers["shared_integer"]->set(shared_integers["shared_integer"]->get() + 1);
		shared_integer_list["shared_integer_list"]->set(intlijsta);
		// shared_integer_list["shared_integer_list"]->set(list_of_lists);
		// shared_integer_list["shared_integer_list"]->set(list_of_lists_of_lists);

		// intlijst.push_back(shared_integers["shared_integer"]->get());

		// ROS_INFO_NAMED(ROS_NAME, "Int is: %d", shared_integers["shared_integer"]->get());

		ROS_INFO_NAMED(ROS_NAME, "Looping");

		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());
	
	ros::spin();

	return 0;
}
