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
#ifndef SHARED_INT_HPP
#define SHARED_INT_HPP

#include <ros/message_traits.h>

#include <std_msgs/Int32.h>

#include "shared_variables/shared_variable.hpp"

namespace shared_variables
{

template <>
class Shareable<int32_t> : public ShareableBaseClass
{
public:
	// Typedefs of the shareable type (message) and localType the type used in user code
	// These typedefs are used in the shared_variable for the callbacks etc. and in 
	// this file for laziness :)
	SHAREABLE_SPECIALIZATION(int32_t, std_msgs::Int32)
	
	Shareable()
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

#endif // SHARED_INT_HPP 
