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
#include "shared_variables/shareable.hpp"


namespace shared_variables
{

template <typename T>
class SharedVariable : public Shareable<T>
{
public:
	// Typedefs of the shareableTypes Shareable<T>
	typedef typename Shareable<T>::shareableType  			shareableType;
	typedef typename Shareable<T>::shareableTypeConstPtr  	shareableTypeConstPtr;

	SharedVariable(const std::string& ns, const std::string& variable_name, const bool& is_server = false)
		: ns_(ns)
		, variable_name_(variable_name)
		, is_server_(is_server)
	{
		n_ = ros::NodeHandle();
		service_name_get_ 		= getServiceGetName(ns_, variable_name_);
		service_name_set_ 		= getServiceSetName(ns_, variable_name_);
		topic_name_updates_   	= getUpdateTopicName(ns_, variable_name_);
		
		if(is_server_)
		{
			// Create a server side shared variable
			service_server_get_ = createServiceServer(n_, service_name_get_, boost::bind(&SharedVariable::CB_get ,this, _1, _2));
			ROS_INFO_NAMED(ROS_NAME, "Created get service get server at '%s'.", service_name_get_.c_str());
			service_server_set_ = createServiceServer(n_, service_name_set_, boost::bind(&SharedVariable::CB_set ,this, _1, _2));
			ROS_INFO_NAMED(ROS_NAME, "Created get service set server at '%s'.", service_name_set_.c_str());

			updates_publisher_ = n_.advertise<shareableType>(topic_name_updates_, 100, true);
			ROS_INFO_NAMED(ROS_NAME, "Client subscribed to topic '%s' for updates.", topic_name_updates_.c_str());
		}
		else
		{
			// Create a client side shared variable
			service_client_get_ = createServiceClient(n_, service_name_get_);
			service_client_set_ = createServiceClient(n_, service_name_set_);
			ROS_INFO_NAMED(ROS_NAME, "Created get service get client at '%s'.", service_name_get_.c_str());
			ROS_INFO_NAMED(ROS_NAME, "Created get service set client at '%s'.", service_name_set_.c_str());

			updates_subscriber_ = n_.subscribe<shareableType>(topic_name_updates_, 100, &SharedVariable::CB_update, this);
			ROS_INFO_NAMED(ROS_NAME, "Client subscribed to topic '%s' for updates.", topic_name_updates_.c_str());
		}
	}

	~SharedVariable()
	{};

	bool set(const T& value)
	{
		if(is_server_)
		{
			shared_variable_.set(value);
			updates_publisher_.publish(shared_variable_.getRef());
			ROS_INFO("Send shared_variable_ %d update.", value);
		}
		else
		{
			shared_variable_.set(value);
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
	T get(ros::Duration max_age = ros::Duration(-1))
	{
		// The client has to update if the varaiable age is too high
		bool update_required = (ros::Time::now() - max_age) > last_update_received_;

		// The server always returns its local version
		if( not is_server_ and update_required)
		{
			if(!getRemote())
				ROS_WARN_NAMED(ROS_NAME, "Could not get remote variable '%s', using old value.", variable_name_.c_str());
		}

		return shared_variable_.get();
	}

private:
	// This function will be invoked by a client in order to get the state of the variable from the server
	bool getRemote()
	{
		ROS_INFO("getRemote shared_variable_ %d", shared_variable_.get());
		
		// Check if remote is available
		if(!remoteAvailable(service_client_get_))
			return false;

		auto ref  = shared_variable_.getRef();
		auto response = shared_variable_.getRef();
		if ( service_client_get_.call(ref, ref, ros::message_traits::MD5Sum<shareableType>::value()) )
		{
			ROS_INFO("Called get %d", shared_variable_.get());
			return true;
		}
		else
		{
			ROS_INFO("Could not call get %d", shared_variable_.get());	
			return false;
		}
	}

	// This function will be invoked by a client in order to change the variable at the server
	bool setRemote()
	{
		ROS_INFO("setRemote shared_variable_ %d", shared_variable_.get());
		
		// Check if remote is available
		if(!remoteAvailable(service_client_get_))
			return false;

		auto ref  = shared_variable_.getRef();
		auto response = shared_variable_.getRef();
		if ( service_client_set_.call(ref, ref, ros::message_traits::MD5Sum<shareableType>::value()) )
		{
			ROS_INFO("Called set %d", shared_variable_.get());
			return true;
		}
		else
		{
			ROS_INFO("Could not call set %d", shared_variable_.get());	
			return false;
		}
	}

	bool remoteAvailable(ros::ServiceClient& service_client, const ros::Duration& timeout = ros::Duration(1.0))
	{
		if(!service_client_get_.waitForExistence(timeout))
		{
			ROS_INFO_NAMED(ROS_NAME, "Could not communicate with service client '%s', is it up and running?", service_client.getService().c_str());
			return false;
		}

		return true;
	}	

	// Is invoked at the server when an client calls its set variable service
	bool CB_set(shareableType & req, shareableType & res)
	{
		ROS_INFO_NAMED(ROS_NAME, "CB_set %d", req.data);
		shared_variable_.getRef() = req;
		res = shared_variable_.getRef(); 
		updates_publisher_.publish(shared_variable_.getRef());
		return true;
	}

	// Is invoked at the server when an client calls its get variable service
	bool CB_get(shareableType & req, shareableType & res)
	{
		ROS_INFO_NAMED(ROS_NAME, "CB_get %d", req.data);
		res = shared_variable_.getRef();
		return true;
	} 

	// Is invoked at the client when the server sends an update of this variable via pub/sub
	void CB_update(const shareableTypeConstPtr & update)
	{
		ROS_INFO_NAMED(ROS_NAME, "CB_update");
		shared_variable_.getRef() = *update;

		// Store the time of the last update
		last_update_received_ = ros::Time::now();
	}

	ros::ServiceClient createServiceClient(ros::NodeHandle& n, const std::string& service_name)
	{
		bool persistent = false;
		ros::ServiceClientOptions ops(service_name,  ros::message_traits::MD5Sum<shareableType>::value(), persistent, ros::M_string());
     	return n.serviceClient(ops);
	}

	ros::ServiceServer createServiceServer(ros::NodeHandle& n, const std::string& service_name, const boost::function<bool(shareableType&, shareableType&)>& _callback)
	{
		ros::AdvertiseServiceOptions ops;

    	ops.service 		= service_name;
	    ops.md5sum 			= ros::message_traits::MD5Sum<shareableType>::value();
	    ops.datatype 		= ros::message_traits::datatype<shareableType>();
	    ops.req_datatype 	= ros::message_traits::datatype<shareableType>();
	    ops.res_datatype 	= ros::message_traits::datatype<shareableType>();
	    ops.helper 			= ros::ServiceCallbackHelperPtr(new ros::ServiceCallbackHelperT<ros::ServiceSpec<shareableType, shareableType> >(_callback));
	    ops.tracked_object 	= ros::VoidConstPtr();
	    
	    return n.advertiseService(ops);
	}

private:
	bool is_server_;

	std::string ns_;
	std::string variable_name_;
	std::string service_name_get_;
	std::string service_name_set_;
	std::string topic_name_updates_;

	Shareable<T> shared_variable_;
	
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
