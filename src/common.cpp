/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/14
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/

#include "rose_shared_variables/common.hpp"

namespace rose_shared_variables
{

	std::string getSharedVariableName(ros::NodeHandle& n, const std::string& variable_name)
	{
		return n.resolveName("/rose_shared_variables/" + variable_name);
	}

	std::string getServiceGetName(ros::NodeHandle& n, const std::string& variable_name)
	{
		return n.resolveName("/rose_shared_variables/" + variable_name + "/get");
	}

	std::string getServiceSetName(ros::NodeHandle& n, const std::string& variable_name)
	{
		return n.resolveName("/rose_shared_variables/" + variable_name + "/set");
	}

	std::string getUpdateTopicName(ros::NodeHandle& n, const std::string& variable_name)
	{
		return n.resolveName("/rose_shared_variables/" + variable_name + "/updates");
	}

}; // namespace rose_shared_variables
