/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/10
* 		- File created.
*
* Description:
*	Boolean server
* 
***********************************************************************************/

#include <ros/ros.h>
#include "rose_common/common.hpp"
#include "rose_conversions/conversions.hpp"

#include "shared_variables/shared_variable.hpp"

using namespace rose_shared_variables;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "bool_shared_variable_server", ros::init_options::AnonymousName);
	ros::NodeHandle n;
	ros::NodeHandle n_p("~");
	ROS_INFO_NAMED(ROS_NAME, "Started boolean shared variable server.\n");

	std::string shared_variable_name = "";
    if(!n_p.getParam("name", shared_variable_name))
    {
    	ROS_ERROR_NAMED(ROS_NAME, "Parameter name must be specified.");
    	return 1;
    }


	
	SharedVariable<bool> shared_bool(shared_variable_name);
	shared_bool.host(true, true);

	do{
		if(rose_conversions::kbhit())
		{
		    uint c = getchar();
		    ROS_INFO_NAMED(ROS_NAME, "Key pressed: %c", (char)c);
		    switch(c)
		    {
		    	case 't':	
		    		shared_bool = true;
		    		ROS_INFO_NAMED(ROS_NAME, "Set shared variable '%s' to TRUE", shared_variable_name.c_str());
					break;
				case 'f':	
					ROS_INFO_NAMED(ROS_NAME, "Set shared variable '%s' to FALSE", shared_variable_name.c_str());
		    		shared_bool = false;
					break;
			}
		}

		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} while(ros::ok());
	
	return 0;
}
