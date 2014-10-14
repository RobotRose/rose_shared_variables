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
#ifndef SHARED_VARIABLE_HPP
#define SHARED_VARIABLE_HPP

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <boost/any.hpp>

#include <rose20_common/ros_name.hpp>

#include "shared_variables/common.hpp"

namespace shared_variables
{

template <typename T>
class SharedVariable
{
public:
	typedef typename ros::conversion::convert<T>::nativeType 	nativeType;
	typedef typename ros::conversion::convert<T>::messageType 	messageType;

	SharedVariable(const std::string& ns, const std::string& variable_name, const bool& is_server = false, const bool& read_only = true, const bool& use_updates = true)
		: ns_(ns)
		, variable_name_(variable_name)
		, is_server_(is_server)
		, use_updates_(use_updates)
		, read_only_(read_only)
	{
		n_ 						= ros::NodeHandle();
		service_name_get_ 		= getServiceGetName(ns_, variable_name_);
		service_name_set_ 		= getServiceSetName(ns_, variable_name_);
		topic_name_updates_   	= getUpdateTopicName(ns_, variable_name_);
		
		if(is_server_)
		{
			// Create a server side shared variable
			service_server_get_ = createServiceServer(n_, service_name_get_, boost::bind(&SharedVariable::CB_get ,this, _1, _2));
			ROS_INFO_NAMED(ROS_NAME, "Shared variable server created get service get server at '%s'.", service_name_get_.c_str());
			service_server_set_ = createServiceServer(n_, service_name_set_, boost::bind(&SharedVariable::CB_set ,this, _1, _2));
			ROS_INFO_NAMED(ROS_NAME, "Shared variable server created get service set server at '%s'.", service_name_set_.c_str());

			if(use_updates_)
			{
				updates_publisher_ = n_.advertise<messageType>(topic_name_updates_, 100, true);
				ROS_INFO_NAMED(ROS_NAME, "Shared variable server publisher advertised to topic '%s' for updates.", topic_name_updates_.c_str());
			}
		}
		else
		{
			// Create a client side shared variable
			service_client_get_ = createServiceClient(n_, service_name_get_);
			service_client_set_ = createServiceClient(n_, service_name_set_);
			ROS_INFO_NAMED(ROS_NAME, "Shared variable client created get client at '%s'.", service_name_get_.c_str());
			ROS_INFO_NAMED(ROS_NAME, "Shared variable client created set client at '%s'.", service_name_set_.c_str());

			if(use_updates_)
			{
				updates_subscriber_ = n_.subscribe<messageType>(topic_name_updates_, 100, &SharedVariable::CB_update, this);
				ROS_INFO_NAMED(ROS_NAME, "Shared variable client subscribed to topic '%s' for updates.", topic_name_updates_.c_str());
			}
		}
	}

	~SharedVariable()
	{};

	bool set(const nativeType& value)
	{
		shared_variable_ = value;
		if(is_server_)
		{
			publishUpdate();
		}
		else
		{
			if(!setRemote())
			{
				ROS_WARN_NAMED(ROS_NAME, "Could not set remote variable '%s', not setting value.", variable_name_.c_str());
				return false;
			}
		}

		return true;
	}

	// Call with a certain ros::Duration(x) in order to get a cached version of the variable if available.
	// The duration indeicates the max age of the cached variable
	nativeType get(ros::Duration max_age = ros::Duration(-1))
	{
		// The client has to update if the varaiable age is too high
		bool update_required = (ros::Time::now() - last_update_received_) > max_age;

		// The server always returns its local version
		if( not is_server_ and update_required )
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Timeout, aksing for update to shared variable server.");
			if(!getRemote())
				ROS_WARN_NAMED(ROS_NAME, "Could not get remote variable '%s', using old value.", variable_name_.c_str());
		}

		return shared_variable_;
	}

private:
	// This function will be invoked by a client in order to get the state of the variable from the server
	bool getRemote()
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Getting an shared variable using the 'get' service.");
		
		// Check if remote is available
		if(!remoteAvailable(service_client_get_))
			return false;

		auto ref  		= ros::conversion::convert<T>().get(shared_variable_);
		auto response 	= ros::conversion::convert<T>().get(shared_variable_);;
		if ( service_client_get_.call(ref, response, ros::message_traits::MD5Sum<messageType>::value()) )
		{
			shared_variable_ = ros::conversion::convert<T>().get(response);
			// Store the time of the last update
			last_update_received_ = ros::Time::now();
			return true;
		}
		else
			return false;
	}

	// This function will be invoked by a client in order to change the variable at the server
	bool setRemote()
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Setting an shared variable using the 'set' service.");

		// Check if remote is available
		if(!remoteAvailable(service_client_get_))
			return false;

		auto ref  						= ros::conversion::convert<T>().get(shared_variable_);
		auto response 					= ros::conversion::convert<T>().get(shared_variable_);
		if ( service_client_set_.call(ref, response, ros::message_traits::MD5Sum<messageType>::value()) )
		{
			shared_variable_ = ros::conversion::convert<T>().get(response);
			return true;
		}
		else
			return false;
	}

	bool remoteAvailable(ros::ServiceClient& service_client, const ros::Duration& timeout = ros::Duration(1.0))
	{
		if(!service_client_get_.waitForExistence(timeout))
		{
			ROS_WARN_NAMED(ROS_NAME, "Could not communicate with service client '%s', is it up and running?", service_client.getService().c_str());
			return false;
		}

		return true;
	}	

	// Is invoked at the server when an client calls its 'set variable' service
	bool CB_set(messageType & req, messageType & res)
	{
		// If this variable is read onyl do not alter it on a request by the client
		if(read_only_)
			return false;

		ROS_DEBUG_NAMED(ROS_NAME, "Received an shared variable 'set' service request.");
		shared_variable_ 	= ros::conversion::convert<T>().get(req);
		res 				= ros::conversion::convert<T>().get(shared_variable_); 
		
		return true;
	}

	// Is invoked at the server when an client calls its 'get variable' service
	bool CB_get(messageType & req, messageType & res)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Received an shared variable 'get' service request.");
		res = ros::conversion::convert<T>().get(shared_variable_);
		return true;
	} 

	// Is invoked at the client when the server sends an update of this variable via pub/sub
	void CB_update(const boost::shared_ptr<messageType const>&  update)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Update of shared variable received from server.");
		shared_variable_ = ros::conversion::convert<T>().get(*update);

		// Store the time of the last update
		last_update_received_ = ros::Time::now();
	}

	// The server uses this to publish updates of the shared variable
	void publishUpdate()
	{
		if(use_updates_)
			updates_publisher_.publish(ros::conversion::convert<T>().get(shared_variable_));
	}

	ros::ServiceClient createServiceClient(ros::NodeHandle& n, const std::string& service_name)
	{
		bool persistent = false;
		ros::ServiceClientOptions ops(service_name,  ros::message_traits::MD5Sum<messageType>::value(), persistent, ros::M_string());
     	return n.serviceClient(ops);
	}

	ros::ServiceServer createServiceServer(ros::NodeHandle& n, const std::string& service_name, const boost::function<bool(messageType&, messageType&)>& _callback)
	{
		ros::AdvertiseServiceOptions ops;

    	ops.service 		= service_name;
	    ops.md5sum 			= ros::message_traits::MD5Sum<messageType>::value();
	    ops.datatype 		= ros::message_traits::datatype<messageType>();
	    ops.req_datatype 	= ros::message_traits::datatype<messageType>();
	    ops.res_datatype 	= ros::message_traits::datatype<messageType>();
	    ops.helper 			= ros::ServiceCallbackHelperPtr(new ros::ServiceCallbackHelperT<ros::ServiceSpec<messageType, messageType> >(_callback));
	    ops.tracked_object 	= ros::VoidConstPtr();
	    
	    return n.advertiseService(ops);
	}

private:
	bool is_server_;
	bool use_updates_;
	bool read_only_;

	std::string ns_;
	std::string variable_name_;
	std::string service_name_get_;
	std::string service_name_set_;
	std::string topic_name_updates_;

	nativeType			shared_variable_;
	
	ros::NodeHandle 	n_;
	ros::ServiceClient 	service_client_get_;
	ros::ServiceClient 	service_client_set_;
	ros::ServiceServer 	service_server_get_;
	ros::ServiceServer 	service_server_set_;

	ros::Subscriber 	updates_subscriber_;
	ros::Publisher 		updates_publisher_;

	ros::Time 			last_update_received_;
};

}; // namespace shared_variables 

#endif // SHARED_VARIABLE_HPP 
