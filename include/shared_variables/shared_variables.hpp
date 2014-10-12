/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/12
* 		- File created.
*
* Description:
*	Container for shared variables.
* 
***********************************************************************************/

#ifndef SHARED_VARIABLES_HPP
#define SHARED_VARIABLES_HPP

#include <memory>
#include <vector>

#include <boost/any.hpp>

#include <ros/ros.h>
#include <ros/service.h>

#include "shared_variables/common.hpp"
#include "shared_variables/shared_variable.hpp"

#include "shared_variables/shared_int32_t.hpp"
#include "shared_variables/shared_bool.hpp"
#include "shared_variables/shared_list.hpp"


namespace shared_variables
{

template <typename T>
class SharedVariables
{
public:
	SharedVariables()
	{
		ns_ = ros::this_node::getNamespace();
	}

	~SharedVariables()
	{
		printf("Destructing shared variables.");
	}

	bool hostSharedVariable(const std::string& variable_name)
	{
		// Create a server shared variable
		if(!isUnique(ns_, variable_name))
		{
			ROS_ERROR_NAMED(ROS_NAME, "A shared variable '%s' already exists in namespace '%s'.", variable_name.c_str(), ns_.c_str());
			ROS_BREAK();
		}

		shared_variables_[getNamespaceVarPair(ns_, variable_name)] = std::shared_ptr<SharedVariable<T>>(new SharedVariable<T>(ns_, variable_name, true));
		printf("Hosted new SharedVariable.");
	}

	bool connectToSharedVariable(const std::string& variable_name)
	{
		// Create a client shared variable
		if(!isUnique(ns_, variable_name))
		{
			ROS_ERROR_NAMED(ROS_NAME, "A shared variable '%s' already exists in namespace '%s'.", variable_name.c_str(), ns_.c_str());
			ROS_BREAK();
		}

				shared_variables_[getNamespaceVarPair(ns_, variable_name)] = std::shared_ptr<SharedVariable<T>>(new SharedVariable<T>(ns_, variable_name, false));
	}

	bool isUnique(const std::string& ns, const std::string& variable_name)
	{
		// Check if we ourself already have a shared variable by that name
		if(shared_variables_.find(getNamespaceVarPair(ns, variable_name)) != shared_variables_.end())
			return false;

		// Check if another server is already advertising this variable name in this namespace
		ros::service::exists(getServiceGetName(ns, variable_name), false);

		return true;
	}

	std::pair<std::string, std::string> getNamespaceVarPair(const std::string& ns, const std::string& variable_name)
	{
		return std::pair<std::string, std::string>(ns, variable_name);
	}

    std::shared_ptr<SharedVariable<T>> operator[](const std::string& variable_name)
    {
    	return shared_variables_.at(getNamespaceVarPair(ns_, variable_name));	
    }

private:
	std::string 		ns_;
	std::map<std::pair<std::string, std::string>, std::shared_ptr<SharedVariable<T>>> shared_variables_;
};

}; // namespace shared_variables 

#endif // SHARED_VARIABLES_HPP 
