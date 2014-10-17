/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/10
* 		- File created.
*
* Description:
*	SharedVariable object, one server, multiple clients
* 	Clients can get and set the shared variable
* 
***********************************************************************************/
#ifndef shareable_HPP
#define shareable_HPP

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <type_traits>
#include <typeinfo>

#include <rose20_common/ros_name.hpp>

#include "roscomm/conversion_bool.hpp"
#include "roscomm/conversion_bool_list.hpp"
#include "roscomm/conversion_int32_t.hpp"
#include "roscomm/conversion_int32_t_list.hpp"

#include "shared_variables/common.hpp"
#include "shared_variables/update_engine.hpp"

template <class T>
struct is_vector { static const bool value = false; };

template <class T, class Alloc>
struct is_vector<std::vector<T, Alloc> > { static const bool value = true; };

// // template<class K, class T, class Comp, class Alloc> 
// // struct is_container<std::map<K, T, Comp, Alloc>> : public std::true_type {};

namespace shared_variables
{

template <typename T, typename E = T>
class SharedVariable
{
public:
	SharedVariable(const std::string& variable_name)
		: shared_value_(new T())
	{
		update_engine_ = std::shared_ptr<UpdateEngine<T>>(new UpdateEngine<T>(variable_name, *shared_value_));
	}

	SharedVariable(T* shared_value, std::shared_ptr<UpdateEngine<E>> update_engine)
		: update_engine_(update_engine)
		, shared_value_(shared_value)
	{}

	~SharedVariable()
	{};

	bool host(const bool& read_only = true, const bool& use_updates = true)
	{
		return update_engine_->host(read_only, use_updates);
	}

	bool connect(const ros::Duration& max_age = ros::Duration(-1))
	{
		return update_engine_->connect(true, max_age);
	}

	bool connect(const bool& use_updates = true, const ros::Duration& max_age = ros::Duration(-1))
	{
		return update_engine_->connect(use_updates, max_age);
	}

	bool set(const T& value)
	{
		*shared_value_ = value;
		return update_engine_->set();
	}

	// Call with a certain ros::Duration(x) in order to get a cached version of the variable if available.
	// The duration indicates the max age of the cached variable
	const T& get()
	{
		update_engine_->get();
		return *this;
	}

	// Copy assignment
	SharedVariable<T, E> operator=(const SharedVariable<T, E>& rhs)
	{
		// ROS_INFO("Copy operator SharedVariable");

		*shared_value_ = *rhs.shared_value_;
		
		update_engine_->set();

		return *this;
	}

	// Assignment from native type
	SharedVariable<T, E>& operator=(T rhs)
	{
		// ROS_INFO("Assignment operator SharedVariable");
	 	*shared_value_ = rhs;
	 	update_engine_->set();
		return *this;
	}

	operator T&()
	{
		// ROS_INFO("Conversion operator SharedVariable");
		update_engine_->get();
		return *shared_value_;
	}

	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, SharedVariable<typename U::value_type, U> >::type
	operator[](unsigned int idx)
  	{
		update_engine_->get();

		SharedVariable<typename U::value_type, U> sv(&(*shared_value_).at(idx), update_engine_);
		return sv;
  	}

  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, void>::type
	push_back(const typename U::value_type& new_elem)
  	{
  		update_engine_->get();
  		shared_value_->push_back(new_elem);
		update_engine_->set();
  	}

  				
  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, void>::type
	erase(unsigned int idx)
  	{
  		update_engine_->get();
  		auto it = shared_value_->begin(); 
  		std::next(it, idx);
  		shared_value_->erase(it);
		update_engine_->set();
  	}

  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, typename U::iterator>::type
	begin()
  	{
  		update_engine_->get();
  		return shared_value_->begin();
  	}

  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, typename U::iterator>::type 
	end()
  	{
  		update_engine_->get();
  		return shared_value_->end();
  	}

  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, unsigned int>::type 
	size()
  	{
  		update_engine_->get();
  		return shared_value_->size();
  	}

  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, SharedVariable<typename U::value_type, U>>::type 
	front()
  	{
  		update_engine_->get();
  		return (*this)[0];
  	}

  	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, SharedVariable<typename U::value_type, U>>::type 
	back()
  	{
  		update_engine_->get();
  		return (*this)[shared_value_->size() - 1];
  	}


private:
	T* 										shared_value_;
	std::shared_ptr<UpdateEngine<E>> 		update_engine_;
};

}; // namespace shared_variables 

#endif // shareable_HPP 
