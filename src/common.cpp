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

#include "shared_variables/common.hpp"

namespace shared_variables
{

	std::string getConcatenatedName(const std::string& ns, const std::string& variable_name)
	{
		return (ns + "/shared_variables/" + variable_name);
	}

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
