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

#include <string>

namespace shared_variables
{
	std::string getConcatenatedName(const std::string& ns, const std::string& variable_name);
	std::string getServiceGetName(const std::string& ns, const std::string& variable_name);
	std::string getServiceSetName(const std::string& ns, const std::string& variable_name);
	std::string getUpdateTopicName(const std::string& ns, const std::string& variable_name);

}; // namespace shared_variables

#endif // SHARED_VARIABLES_COMMON_HPP 
