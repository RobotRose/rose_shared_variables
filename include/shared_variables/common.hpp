/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/13
* 		- File created.
*
* Description:
*	Contains functions used throughout shared_variables library
* 
***********************************************************************************/
#ifndef SHARED_VARIABLES_COMMON_HPP
#define SHARED_VARIABLES_COMMON_HPP

#include <ros/ros.h>
#include <string>

namespace shared_variables
{
	std::string getSharedVariableName(ros::NodeHandle& n, const std::string& variable_name);
	std::string getServiceGetName(ros::NodeHandle& n, const std::string& variable_name);
	std::string getServiceSetName(ros::NodeHandle& n, const std::string& variable_name);
	std::string getUpdateTopicName(ros::NodeHandle& n, const std::string& variable_name);
}; // namespace shared_variables

#endif // SHARED_VARIABLES_COMMON_HPP 
