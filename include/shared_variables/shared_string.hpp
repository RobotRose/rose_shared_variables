/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/13
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef SHAREABLE_STRING_HPP
#define SHAREABLE_STRING_HPP

#include <std_msgs/String.h>

#include "shared_variables/shared_variable.hpp"

namespace shared_variables
{

template<>
class Shareable<std::string> 
{
public:
	// Typedefs of the shareable type, used in the shared_variable for the callbacks etc.
	SHAREABLE_SPECIALIZATION(std::string, std_msgs::String)
	
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

#endif // SHAREABLE_STRING_HPP 
