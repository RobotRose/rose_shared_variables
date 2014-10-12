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

namespace shared_variables
{

std::string getServiceGetName(const std::string& ns, const std::string& variable_name)
{
	return (ns + "/shared_variables/" + variable_name + "_get");
}

std::string getServiceSetName(const std::string& ns, const std::string& variable_name)
{
	return (ns + "/shared_variables/" + variable_name + "_set");
}

std::string getUpdateTopicName(const std::string& ns, const std::string& variable_name)
{
	return (ns + "/shared_variables/" + variable_name + "_updates");
}

}; // namespace shared_variables

#endif // SHARED_VARIABLES_COMMON_HPP 
