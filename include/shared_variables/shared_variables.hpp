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

#include "shared_variables/shared_bool.hpp"
#include "shared_variables/shared_int32_t.hpp"
#include "shared_variables/shared_string.hpp"
#include "shared_variables/shared_vector.hpp"

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
	{}

	bool host(const std::string& variable_name, const bool& read_only = true, const bool& use_updates = true)
	{
		// Create a server shared variable
		if(!isUnique(ns_, variable_name))
			return false;

		// Check if another server is already advertising this variable name in this namespace
		if(ros::service::exists(getServiceGetName(ns_, variable_name), false))
		{
			ROS_ERROR_NAMED(ROS_NAME, "A shared variable '%s' already exists in namespace '%s', aborting.", variable_name.c_str(), ns_.c_str());
			return false;
		}

		shared_variables_[getNameVarPair(ns_, variable_name)] = std::shared_ptr<SharedVariable<T>>(new SharedVariable<T>(ns_, variable_name, true, read_only, use_updates));
		
		return true;
	}

	bool connect(const std::string& variable_name, const bool& read_only = true, const bool& use_updates = true)
	{
		// Create a client shared variable
		if(!isUnique(ns_, variable_name))
			return false;

		shared_variables_[getNameVarPair(ns_, variable_name)] = std::shared_ptr<SharedVariable<T>>(new SharedVariable<T>(ns_, variable_name, false, read_only, use_updates));

		return true;
	}

	std::pair<std::string, std::string> getNameVarPair(const std::string& ns, const std::string& variable_name)
	{
		return std::pair<std::string, std::string>(ns, variable_name);
	}

    std::shared_ptr<SharedVariable<T>> operator[](const std::string& variable_name)
    {
    	std::shared_ptr<SharedVariable<T>> shared_var;
    	try
		{
		    shared_var = shared_variables_.at(getNameVarPair(ns_, variable_name));
		}
		catch (std::out_of_range& oor)
		{
		    ROS_ERROR_NAMED(ROS_NAME, "Error while trying to get variable '%s' from shared variables", getConcatenatedName(ns_, variable_name).c_str());
		    throw oor;
		}

    	return shared_var;
    }

private:
	bool isUnique(const std::string& ns, const std::string& variable_name)
	{
		// Check if we ourself already have a shared variable by that name
		if(shared_variables_.find(getNameVarPair(ns, variable_name)) != shared_variables_.end())
		{
			ROS_ERROR_NAMED(ROS_NAME, "A shared variable '%s' already exists for node '%s', overwriting.", variable_name.c_str(), ns.c_str());
			return false;
		}

		return true;
	}

private:
	std::string ns_;
	std::map<std::pair<std::string, std::string>, std::shared_ptr<SharedVariable<T>>> shared_variables_;
};

}; // namespace shared_variables 

#endif // SHARED_VARIABLES_HPP 
