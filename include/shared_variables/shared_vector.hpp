/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/10
* 		- File created.
*
* Description:
*	Shareable vector of a shareable type
* 
***********************************************************************************/
#ifndef SHARED_VECTOR_HPP
#define SHARED_VECTOR_HPP

#include <ros/message_traits.h>

#include <std_msgs/Int32.h>

#include "shared_variables/shared_variable.hpp"

#include "luctor_base_class.hpp"


namespace shared_variables
{

template <typename T>
class Shareable<std::vector<T>>
{
public:
	// Typedefs of the shareable type (message) and localType the type used in user code
	// These typedefs are used in the shared_variable for the callbacks etc. and in 
	// this file for laziness :)
	SHAREABLE_SPECIALIZATION(typename std::vector<T>, typename std::vector< typename Shareable<T>::shareableType>)
									// localType 								shareableType
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
		localType value(shared_variable_.size());

		auto value_it = value.begin();
		for(auto shared_value_it = shared_variable_.begin(); shared_value_it < shared_variable_.end(); shared_value_it++, value_it++)
		{
			Shareable<T> shareable(*shared_value_it); 
			*value_it = shareable.get();
		}

		return value; 
	} 

	void set(const localType& new_value)
	{
		shared_variable_.resize(new_value.size());

		auto shared_value_it = shared_variable_.begin();
		for(auto new_value_it = new_value.begin(); new_value_it < new_value.end(); new_value_it++, shared_value_it++)
		{
			Shareable<T> shareable(*new_value_it); 
			*shared_value_it = shareable.getRef();
		}
	}

	shareableType& getRef()
	{
		return shared_variable_;
	}

private:
	shareableType shared_variable_;
};

}; // namespace shared_variables 

#endif // SHARED_VECTOR_HPP 
