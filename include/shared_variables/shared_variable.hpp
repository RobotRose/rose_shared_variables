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
#include "roscomm/conversion_bool.hpp"
#include "roscomm/conversion_bool_list.hpp"
#include "roscomm/conversion_int32_t.hpp"
#include "roscomm/conversion_int32_t_list.hpp"

namespace shared_variables
{

template <typename T>
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
	{}



	SharedVariable(const std::string& variable_name)
		: variable_name_(variable_name)
		, is_server_(false)
		, is_client_(false)
		, use_updates_(true)
		, read_only_(false)
	{

		n_ 						= ros::NodeHandle();
		shared_name_			= getSharedVariableName(n_, variable_name_);
		service_name_get_ 		= getServiceGetName(n_, variable_name_);
		service_name_set_ 		= getServiceSetName(n_, variable_name_);
		topic_name_updates_   	= getUpdateTopicName(n_, variable_name_);
	}

	~SharedVariable()
	{};


	bool host(const bool& read_only = true, const bool& use_updates = true)
	{
		if(is_client_)
		{
			ROS_ERROR_NAMED(ROS_NAME, "Cannot host the shared variable '%s' if already connected as client to the same shared variable.", shared_name_.c_str());
			return false;
		}

		// Check if another server is already advertising this variable name in this namespace
		if(ros::service::exists(service_name_get_, false))
		{
			ROS_ERROR_NAMED(ROS_NAME, "A shared variable '%s' already exists, cannot host another.", shared_name_.c_str());
			return false;
		}

		// Create a server side shared variable
		service_server_get_ = createServiceServer(n_, service_name_get_, boost::bind(&SharedVariable::CB_get ,this, _1, _2));
		ROS_DEBUG_NAMED(ROS_NAME, "Shared variable server created get service get server at '%s'.", service_name_get_.c_str());
		service_server_set_ = createServiceServer(n_, service_name_set_, boost::bind(&SharedVariable::CB_set ,this, _1, _2));
		ROS_DEBUG_NAMED(ROS_NAME, "Shared variable server created get service set server at '%s'.", service_name_set_.c_str());

		// Settings
		read_only_ 		= read_only;
		use_updates_ 	= use_updates;
		is_server_ 		= true;
		
		// Create publisher if we use the updates topic
		if(use_updates_)
		{
			updates_publisher_ = n_.advertise<messageType>(topic_name_updates_, 100, true);
			ROS_DEBUG_NAMED(ROS_NAME, "Shared variable server publisher advertised to topic '%s' for updates.", topic_name_updates_.c_str());
		}		

		ROS_INFO_NAMED(ROS_NAME, "Shared variable '%s' server created.", shared_name_.c_str());

		return true;
	}

	bool connect(const bool& use_updates = true)
	{
		if(is_client_)
		{
			ROS_WARN_NAMED(ROS_NAME, "Already connected to a shared variable '%s', doing nothing.", shared_name_.c_str());
			return false;
		}

		if(is_server_)
		{
			ROS_ERROR_NAMED(ROS_NAME, "Cannot connected to the shared variable '%s' if already hosting it.", shared_name_.c_str());
			return false;
		}

		// Become a client side shared variable
		service_client_get_ = createServiceClient(n_, service_name_get_);
		ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client created get client at '%s'.", service_client_get_.getService().c_str());
		service_client_set_ = createServiceClient(n_, service_name_set_);
		ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client created set client at '%s'.", service_client_set_.getService().c_str());
		
		// Settings
		use_updates_ 	= use_updates;
		is_client_		= true;

		// Create subscriber if we use the updates topic
		if(use_updates_)
		{
			updates_subscriber_ = n_.subscribe<messageType>(topic_name_updates_, 100, &SharedVariable::CB_update, this);
			ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client subscribed to topic '%s' for updates.", topic_name_updates_.c_str());
		}

		ROS_INFO_NAMED(ROS_NAME, "Shared variable '%s' client created.", shared_name_.c_str());

		return true;
	}

	bool set(const nativeType& value)
	{
		if( not is_server_ and not is_client_ )
		{
			ROS_WARN_NAMED(ROS_NAME, "Cannot set shared variable '%s' while not connected-to, or hosting it.", shared_name_.c_str());
			return false;
		}

		shared_variable_ = value;
		if(is_server_)
		{
			publishUpdate();
		}
		else
		{
			if(!setRemote())
			{
				ROS_WARN_NAMED(ROS_NAME, "Could not set shared variable '%s', not setting value.", shared_name_.c_str());
				return false;
			}
		}

		return true;
	}

	// Call with a certain ros::Duration(x) in order to get a cached version of the variable if available.
	// The duration indicates the max age of the cached variable
	nativeType get(ros::Duration max_age = ros::Duration(-1))
	{
		if( not is_server_ and not is_client_ )
		{
			ROS_WARN_NAMED(ROS_NAME, "Cannot get shared variable '%s' while not connected-to, or hosting it.", shared_name_.c_str());
			return false;
		}

		// The client has to update if the last update was too long ago
		bool update_required = (ros::Time::now() - last_update_received_) > max_age;

		// The server always returns its local version
		if( not is_server_ and update_required )
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Timeout, aksing for update to shared variable server.");
			if(!getRemote())
				ROS_WARN_NAMED(ROS_NAME, "Could not get shared variable '%s', using old value.", shared_name_.c_str());
		}

		return shared_variable_;
	}

	// Copy assignment
	SharedVariable<T> operator=(SharedVariable<T> rhs)
	{
		// No copy exists, only move
		move(*this, rhs);
		
		if(is_server_)
			this->publishUpdate();

		return *this;
	}

	// Set from native type
	SharedVariable<T>& operator=(const nativeType& rhs)
	{
		// The server annouches that the variable has changed via an publish
	 	if(is_server_)
	 	{
	 		this->shared_variable_ = rhs;
	 		this->publishUpdate();
	 	}
	 	else
	 	{
	 		this->set(this->shared_variable_ = rhs);
	 	}

		return *this;
	}

	// Conversion operator
	operator T() const
	{
		return this->shared_variable_;
	}

private:
	// This function will be invoked by a client in order to get the state of the variable from the server
	bool getRemote()
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Getting shared variable '%s' using the 'get' service.", shared_name_.c_str());
		
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
		ROS_DEBUG_NAMED(ROS_NAME, "Setting shared variable '%s' using the 'set' service.", shared_name_.c_str());

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
		// If this variable is read only so do not alter it on a request by the client
		if(read_only_)
		{
			ROS_WARN_NAMED(ROS_NAME, "Someone tried to set read-only shared variable '%s'", shared_name_.c_str());
			return false;
		}

		ROS_DEBUG_NAMED(ROS_NAME, "Received shared variable 'set' service request.");
		shared_variable_ 	= ros::conversion::convert<T>().get(req);
		res 				= ros::conversion::convert<T>().get(shared_variable_); 
		
		return true;
	}

	// Is invoked at the server when an client calls its 'get variable' service
	bool CB_get(messageType & req, messageType & res)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Received shared variable 'get' service request.");
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
	bool publishUpdate()
	{
		if(not is_server_ )
		{
			ROS_WARN_NAMED(ROS_NAME, "Cannot publish shared variable '%s' while not hosting it.", shared_name_.c_str());
			return false;
		}

		ROS_DEBUG_NAMED(ROS_NAME, "Publishing shared variable update via '%s'.", topic_name_updates_.c_str());
		if(use_updates_)
			updates_publisher_.publish(ros::conversion::convert<T>().get(shared_variable_));

		return true;
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

		to.shared_variable_ 		= std::move(from.shared_variable_);

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
