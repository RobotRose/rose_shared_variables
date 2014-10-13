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
#ifndef SHAREABLE_BOOL_HPP
#define SHAREABLE_BOOL_HPP

#include <std_msgs/Bool.h>

#include "shared_variables/shared_variable.hpp"

namespace shared_variables
{

template<>
class Shareable<bool> 
{
public:
	// Typedefs of the shareable type, used in the shared_variable for the callbacks etc.
	SHAREABLE_SPECIALIZATION(bool, std_msgs::Bool)
	
	Shareable()
	{}

	Shareable(const localType& init_var)
	{
		set(init_var);
	}

	Shareable(const shareableType& init_var)
		: shared_variable_(init_var)
	{}

	localType get()
	{
		return shared_variable_.data; 
	} 

	void set(const localType& new_value)
	{
		shared_variable_.data = new_value; 
	}

	shareableType& getRef()
	{
		return shared_variable_;
	}

private:
	shareableType shared_variable_;
};

}; // namespace shared_variables 

#endif // SHAREABLE_BOOL_HPP 
