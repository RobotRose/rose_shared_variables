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

#include <rose20_common/ros_name.hpp>

#include "roscomm/conversion_bool.hpp"
#include "roscomm/conversion_bool_list.hpp"
#include "roscomm/conversion_int32_t.hpp"
#include "roscomm/conversion_int32_t_list.hpp"

#include "shared_variables/common.hpp"
#include "shared_variables/shareable.hpp"
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
	typedef typename ros::conversion::convert<T>::nativeType 	nativeType;
	typedef typename ros::conversion::convert<T>::messageType 	messageType;

	SharedVariable()
		: variable_name_("NO_NAME_DEFAULT_CONSTRUCTOR")
		, is_server_(false)
		, is_client_(false)
		, use_updates_(false)
		, read_only_(true)
		, update_engine_(NULL)
	{}

	SharedVariable(const std::string& variable_name)
		: variable_name_(variable_name)
		, is_server_(false)
		, is_client_(false)
		, use_updates_(true)
		, read_only_(false)
	{
		update_engine_ 			= std::shared_ptr<UpdateEngine<T>>(new UpdateEngine<T>(variable_name, shared_value_));
	}

	SharedVariable(T& s, std::shared_ptr<UpdateEngine<E>> update_engine)
		: update_engine_(update_engine)
	{
		*shared_value_ = s;
	}

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
		// if(is_client_)
		// {
		// 	ROS_WARN_NAMED(ROS_NAME, "Already connected to a shared variable '%s', doing nothing.", shared_name_.c_str());
		// 	return false;
		// }

		// if(is_server_)
		// {
		// 	ROS_ERROR_NAMED(ROS_NAME, "Cannot connected to the shared variable '%s' if already hosting it.", shared_name_.c_str());
		// 	return false;
		// }

		// max_age_ = max_age;

		// // Become a client side shared variable
		// service_client_get_ = createServiceClient(n_, service_name_get_);
		// ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client created get client at '%s'.", service_client_get_.getService().c_str());
		// service_client_set_ = createServiceClient(n_, service_name_set_);
		// ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client created set client at '%s'.", service_client_set_.getService().c_str());
		
		// // Settings
		// use_updates_ 	= use_updates;
		// is_client_		= true;

		// // Create subscriber if we use the updates topic
		// if(use_updates_)
		// {
		// 	updates_subscriber_ = n_.subscribe<messageType>(topic_name_updates_, 100, &SharedVariable::CB_update, this);
		// 	ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client subscribed to topic '%s' for updates.", topic_name_updates_.c_str());
		// }

		// ROS_INFO_NAMED(ROS_NAME, "Shared variable '%s' client created.", shared_name_.c_str());

		// return true;
	}

	// bool set()
	// {
	// 	return update_engine_->set(this->shareable_);
	// }

	bool set(const T& value)
	{
		*shared_value_ = value;
		return update_engine_->set();
		// if( not is_server_ and not is_client_ )
		// {
		// 	ROS_WARN_NAMED(ROS_NAME, "Cannot set shared variable '%s' while not connected-to, or hosting it.", shared_name_.c_str());
		// 	return false;
		// }

		// shareable_ = value;
		// if(is_server_)
		// {
		// 	publishUpdate();
		// }
		// else
		// {
		// 	if(!setRemote())
		// 	{
		// 		ROS_WARN_NAMED(ROS_NAME, "Could not set shared variable '%s', not setting value.", shared_name_.c_str());
		// 		return false;
		// 	}
		// }

		// return true;
	}

	// Call with a certain ros::Duration(x) in order to get a cached version of the variable if available.
	// The duration indicates the max age of the cached variable
	SharedVariable<T, E>& get()
	{
		update_engine_->get();
		return *this;
		// if( not is_server_ and not is_client_ )
		// {
		// 	ROS_WARN_NAMED(ROS_NAME, "Cannot get shared variable '%s' while not connected-to, or hosting it.", shared_name_.c_str());
		// 	return false;
		// }

		// // The client has to update if the last update was too long ago
		// bool update_required = (ros::Time::now() - last_update_received_) > max_age_;

		// // The server always returns its local version
		// if( not is_server_ and update_required )
		// {
		// 	ROS_DEBUG_NAMED(ROS_NAME, "Timeout, aksing for update to shared variable server.");
		// 	if(!getRemote())
		// 		ROS_WARN_NAMED(ROS_NAME, "Could not get shared variable '%s', using old value.", shared_name_.c_str());
		// }

		// return true;
	}

	// Copy assignment
	SharedVariable<T, E> operator=(SharedVariable<T> rhs)
	{
		ROS_INFO("Copy operator SharedVariable");
		// No copy exists, only move
		move(*this, rhs);
		
		update_engine_->set();

		return *this;
	}

	// Assignment from native type
	SharedVariable<T, E>& operator=(const nativeType& rhs)
	{
		ROS_INFO("Assignment operator SharedVariable");
	 	*shared_value_ = rhs;
	 	update_engine_->set();
		return *this;
	}

	// Conversion operator
	operator const T&()
	{
		ROS_INFO("Conversion operator SharedVariable");
		update_engine_->get();
		return *shared_value_;
	}


	template<typename U = T>
	typename std::enable_if<!is_vector<U>::value>::type
	MyFunction()
	{
		std::cout << "T is non-vector." << std::endl;
	}

	template<typename U = T>
	typename std::enable_if<is_vector<U>::value>::type
	MyFunction()
	{
		std::cout << "T is vector." << std::endl;
	}


	// template<typename W = T>
	// typename std::enable_if<!is_vector<W>::value, T>::type; 

	// template<typename W = T>
	// typename std::enable_if<is_vector<W>::value, T>::type::value_type;



	template<typename U = T>
	typename std::enable_if<is_vector<U>::value, SharedVariable<typename U::value_type, U> >::type
	operator[](unsigned int idx)
  	{
		ROS_INFO("vector[] operator SharedVariable");
		update_engine_->get();

		SharedVariable<typename U::value_type, U> sv((*shared_value_).at(idx), update_engine_);
		return sv;
  	}

	// template <typename U = T> //is_vector<T>::value
	// typename std::enable_if<!is_vector<T>::value, SharedVariable<T>&>::type
	//  operator[](unsigned int idx)
 //  	{
	// 	ROS_INFO("[] operator SharedVariable");
	// 	get();
	// 	return shareable_;
 //  	}

	// template <typename U = T> //is_vector<T>::value
	// typename std::enable_if<is_vector<T>::value, SharedVariable<T>&>::type
	//  operator[](unsigned int idx)
 //  	{
	// 	ROS_INFO("[] operator SharedVariable");
	// 	get();
	// 	return shareable_[idx];
 //  	}


	// std::enable_if<is_vector<U>::value, typename U::value_type>
	// SharedVariable<U>& operator[](unsigned int idx)
 //  	{
	// 	static_assert(false, "Here2");
	// 	ROS_INFO("[] operator SharedVariable");
	// 	get();
	// 	return shareable_[idx];
 //  	}

private:
	// // This function will be invoked by a client in order to get the state of the variable from the server
	// bool getRemote()
	// {
	// 	ROS_DEBUG_NAMED(ROS_NAME, "Getting shared variable '%s' using the 'get' service.", shared_name_.c_str());
		
	// 	// Check if remote is available
	// 	if(!remoteAvailable(service_client_get_))
	// 		return false;

	// 	auto ref  		= ros::conversion::convert<T>().get(shareable_);
	// 	auto response 	= ros::conversion::convert<T>().get(shareable_);
	// 	if ( service_client_get_.call(ref, response, ros::message_traits::MD5Sum<messageType>::value()) )
	// 	{
	// 		shareable_ = ros::conversion::convert<T>().get(response);
	// 		// Store the time of the last update
	// 		last_update_received_ = ros::Time::now();
	// 		return true;
	// 	}
	// 	else
	// 		return false;
	// }

	// // This function will be invoked by a client in order to change the variable at the server
	// bool setRemote()
	// {
	// 	ROS_DEBUG_NAMED(ROS_NAME, "Setting shared variable '%s' using the 'set' service.", shared_name_.c_str());

	// 	// Check if remote is available
	// 	if(!remoteAvailable(service_client_get_))
	// 		return false;

	// 	auto ref  						= ros::conversion::convert<T>().get(shareable_);
	// 	auto response 					= ros::conversion::convert<T>().get(shareable_);
	// 	if ( service_client_set_.call(ref, response, ros::message_traits::MD5Sum<messageType>::value()) )
	// 	{
	// 		shareable_ = ros::conversion::convert<T>().get(response);
	// 		return true;
	// 	}
	// 	else
	// 		return false;
	// }

	// // The server uses this to publish updates of the shared variable
	// bool publishUpdate()
	// {
	// 	if(not is_server_ )
	// 	{
	// 		ROS_WARN_NAMED(ROS_NAME, "Cannot publish shared variable '%s' while not hosting it.", shared_name_.c_str());
	// 		return false;
	// 	}

	// 	ROS_DEBUG_NAMED(ROS_NAME, "Publishing shared variable update via '%s'.", topic_name_updates_.c_str());
	// 	if(use_updates_)
	// 		updates_publisher_.publish(ros::conversion::convert<T>().get(shareable_));

	// 	return true;
	// }

	// bool remoteAvailable(ros::ServiceClient& service_client, const ros::Duration& timeout = ros::Duration(1.0))
	// {
	// 	if(!service_client_get_.waitForExistence(timeout))
	// 	{
	// 		ROS_WARN_NAMED(ROS_NAME, "Could not communicate with service client '%s', is it up and running?", service_client.getService().c_str());
	// 		return false;
	// 	}

	// 	return true;
	// }	

	// // Is invoked at the server when an client calls its 'set variable' service
	// bool CB_set(messageType & req, messageType & res)
	// {
	// 	// If this variable is read only so do not alter it on a request by the client
	// 	if(read_only_)
	// 	{
	// 		ROS_WARN_NAMED(ROS_NAME, "Someone tried to set read-only shared variable '%s'", shared_name_.c_str());
	// 		return false;
	// 	}

	// 	ROS_INFO_NAMED(ROS_NAME, "Received shared variable 'set' service request.");
	// 	shareable_ 	= ros::conversion::convert<T>().get(req);
	// 	res 				= ros::conversion::convert<T>().get(shareable_); 
		
	// 	return true;
	// }

	// // Is invoked at the server when an client calls its 'get variable' service
	// bool CB_get(messageType & req, messageType & res)
	// {
	// 	ROS_INFO_NAMED(ROS_NAME, "Received shared variable 'get' service request.");
	// 	res = ros::conversion::convert<T>().get(shareable_);
	// 	return true;
	// } 

	// // Is invoked at the client when the server sends an update of this variable via pub/sub
	// void CB_update(const boost::shared_ptr<messageType const>&  update)
	// {
	// 	ROS_INFO_NAMED(ROS_NAME, "Update of shared variable received from server.");
	// 	shareable_ = ros::conversion::convert<T>().get(*update);

	// 	// Store the time of the last update
	// 	last_update_received_ = ros::Time::now();
	// }


	// ros::ServiceClient createServiceClient(ros::NodeHandle& n, const std::string& service_name)
	// {
	// 	bool persistent = false;
	// 	ros::ServiceClientOptions ops(service_name,  ros::message_traits::MD5Sum<messageType>::value(), persistent, ros::M_string());
 //     	return n.serviceClient(ops);
	// }

	// ros::ServiceServer createServiceServer(ros::NodeHandle& n, const std::string& service_name, const boost::function<bool(messageType&, messageType&)>& _callback)
	// {
	// 	ros::AdvertiseServiceOptions ops;

 //    	ops.service 		= service_name;
	//     ops.md5sum 			= ros::message_traits::MD5Sum<messageType>::value();
	//     ops.datatype 		= ros::message_traits::datatype<messageType>();
	//     ops.req_datatype 	= ros::message_traits::datatype<messageType>();
	//     ops.res_datatype 	= ros::message_traits::datatype<messageType>();
	//     ops.helper 			= ros::ServiceCallbackHelperPtr(new ros::ServiceCallbackHelperT<ros::ServiceSpec<messageType, messageType> >(_callback));
	//     ops.tracked_object 	= ros::VoidConstPtr();
	    
	//     return n.advertiseService(ops);
	// }

	// For moving one sharedVariable to another.
	void move(SharedVariable<T>& to, SharedVariable<T>& from)
	{
		to.is_server_ 				= std::move(from.is_server_);
		to.is_client_ 				= std::move(from.is_client_);
		to.use_updates_ 			= std::move(from.use_updates_);
		to.read_only_ 				= std::move(from.read_only_);

		to.variable_name_ 			= std::move(from.variable_name_);
		to.shared_name_ 			= std::move(from.shared_name_);
		to.service_name_get_ 		= std::move(from.service_name_get_);
		to.service_name_set_ 		= std::move(from.service_name_set_);
		to.topic_name_updates_ 		= std::move(from.topic_name_updates_);

		to.shareable_ 				= std::move(from.shareable_);

		to.n_ 						= std::move(from.n_);
		to.service_client_get_ 		= std::move(from.service_client_get_);
		to.service_client_set_ 		= std::move(from.service_client_set_);
		to.service_server_get_ 		= std::move(from.service_server_get_);
		to.service_server_set_ 		= std::move(from.service_server_set_);

		to.updates_subscriber_ 		= std::move(from.updates_subscriber_);
		to.updates_publisher_ 		= std::move(from.updates_publisher_);

		to.last_update_received_ 	= std::move(from.last_update_received_);
	}

private:
	bool 				is_server_;
	bool 				is_client_;
	bool 				use_updates_;
	bool 				read_only_;

	std::string 		variable_name_;
	std::string 		shared_name_;
	std::string 		service_name_get_;
	std::string 		service_name_set_;
	std::string 		topic_name_updates_;

	// Shareable<nativeType> shareable_;
	std::shared_ptr<T> 	shared_value_;
	
	ros::NodeHandle 	n_;
	ros::ServiceClient 	service_client_get_;
	ros::ServiceClient 	service_client_set_;
	ros::ServiceServer 	service_server_get_;
	ros::ServiceServer 	service_server_set_;

	ros::Subscriber 	updates_subscriber_;
	ros::Publisher 		updates_publisher_;

	ros::Time 			last_update_received_;
	ros::Duration 		max_age_;

	std::shared_ptr<UpdateEngine<E>> 		update_engine_;
};

// template <typename T>
// class SharedVariable<std::vector<T>> 
// {
// public:
// 	SharedVariable(const std::string& variable_name)
// 		: SharedVariable<T>(variable_name)
// 	{}

// 	~SharedVariable()
// 	{}

// 	// Copy assignment
// 	SharedVariable<std::vector<T>> operator=(SharedVariable<std::vector<T>> rhs)
// 	{
// 		ROS_INFO("Copy operator SharedVariable<std::vector<T>>");
// 		// No copy exists, only move
// 		move(*this, rhs);
		
// 		SharedVariable<T>::set();

// 		return *this;
// 	}

// 	// Assignment from native vector type
// 	SharedVariable<std::vector<T>>& operator=(const std::vector<T>& rhs)
// 	{
// 		ROS_INFO("Assignment operator SharedVariable<std::vector<T>>");
// 	 	shareables_.clear();
// 	 	for(const auto& elem : rhs)
// 	 		shareables_.push_back(Shareable<T>(elem));
	 	
// 	 	SharedVariable<T>::set();
// 		return *this;
// 	}

// 	// Conversion operator
// 	operator const std::vector<T>()
// 	{
// 		ROS_INFO("Conversion operator SharedVariable<std::vector<T>>");
// 		SharedVariable<T>::get();

// 		std::vector<T> vec(shareables_.size());
// 		for(const auto& elem : shareables_)
// 	 		vec.push_back(elem);

// 		return vec;
// 	}

// };

}; // namespace shared_variables 

#endif // shareable_HPP 
