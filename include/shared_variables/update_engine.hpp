/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/10/16
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/

#ifndef UPDATE_ENGINE_HPP
#define UPDATE_ENGINE_HPP

namespace shared_variables
{


template < typename T >
class UpdateEngine
{
public:
	typedef typename ros::conversion::convert<T>::nativeType 	nativeType;
	typedef typename ros::conversion::convert<T>::messageType 	messageType;
	typedef typename boost::function<void (const T& update) > 	changeCallbackType;

	UpdateEngine(const std::string& variable_name, T& value)
		: variable_name_(variable_name)
		, value_(value)
		, is_server_(false)
		, is_client_(false)
		, connected_(false)
		, read_only_(true)
		, use_updates_(false)
		, publish_rate_(ros::Rate(0.0))
		, value_changed_CB_(NULL)
	{
		n_ 						= ros::NodeHandle();
		shared_name_			= getSharedVariableName(n_, variable_name_);
		service_name_get_ 		= getServiceGetName(n_, variable_name_);
		service_name_set_ 		= getServiceSetName(n_, variable_name_);
		topic_name_updates_   	= getUpdateTopicName(n_, variable_name_);
	}

	~UpdateEngine()
	{}

	bool host(const bool& read_only, const bool& use_updates, const ros::Rate& publish_rate)
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
		service_server_get_ = createServiceServer(n_, service_name_get_, boost::bind(&UpdateEngine::CB_get ,this, _1, _2));
		ROS_DEBUG_NAMED(ROS_NAME, "Shared variable server created get service get server at '%s'.", service_name_get_.c_str());
		service_server_set_ = createServiceServer(n_, service_name_set_, boost::bind(&UpdateEngine::CB_set ,this, _1, _2));
		ROS_DEBUG_NAMED(ROS_NAME, "Shared variable server created get service set server at '%s'.", service_name_set_.c_str());

		// Settings
		read_only_ 		= read_only;
		use_updates_ 	= use_updates;
		is_server_ 		= true;
		publish_rate_ 	= publish_rate;
		
		// Create publisher if we use the updates topic
		if(use_updates_)
		{
			updates_publisher_ = n_.advertise<messageType>(topic_name_updates_, 100, true);
			ROS_DEBUG_NAMED(ROS_NAME, "Shared variable server publisher advertised to topic '%s' for updates.", topic_name_updates_.c_str());
		}		

		ROS_INFO_NAMED(ROS_NAME, "Shared variable '%s' server created.", shared_name_.c_str());

		return true;
	}

	bool connect(const ros::Duration& max_age = ros::Duration(-1))
	{
		return connect(true, max_age);
	}

	bool connect(const bool& use_updates = true, const ros::Duration& max_age = ros::Duration(-1))
	{
		if(is_client_ and connected_ )
		{
			ROS_WARN_NAMED(ROS_NAME, "Already connected to a shared variable '%s', doing nothing.", shared_name_.c_str());
			return false;
		}

		if(is_server_)
		{
			ROS_ERROR_NAMED(ROS_NAME, "Cannot connected to the shared variable '%s' if already hosting it.", shared_name_.c_str());
			return false;
		}

		max_age_ = max_age;

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
			updates_subscriber_ = n_.subscribe<messageType>(topic_name_updates_, 100, &UpdateEngine::CB_update, this);
			ROS_DEBUG_NAMED(ROS_NAME, "Shared variable client subscribed to topic '%s' for updates.", topic_name_updates_.c_str());
		}

		ROS_INFO_NAMED(ROS_NAME, "Shared variable '%s' client created.", shared_name_.c_str());

		connected_ = getRemote();

		return connected_;
	}

	bool registerChangeCallback(const UpdateEngine::changeCallbackType callback)
	{
		if( not is_client_ )
		{
			ROS_ERROR_NAMED(ROS_NAME, "Registering a change callback client is only possible when connected to a shared variable as a client.");
			return false;
		}

		value_changed_CB_ = callback;
		return true;
	}

	bool unregisterChangeCallback()
	{
		value_changed_CB_ = NULL;
		return true;
	}

	bool set()
	{
		if(is_server_)
		{
			publishUpdate();
		}
		else
		{
			if( not connected_ )
			{
				ROS_WARN_NAMED(ROS_NAME, "Could not set remote variable '%s', bacause not connected.", shared_name_.c_str());
				return false;
			}

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
	bool get()
	{
		if( not connected_ )
		{
			ROS_WARN_NAMED(ROS_NAME, "Could not get remote variable '%s', because not connected.", shared_name_.c_str());
			return false;
		}

		// The client has to update if the last update was too long ago
		bool update_required = (ros::Time::now() - last_update_received_) > max_age_;

		// The server always returns its local version
		if( not is_server_ and update_required )
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Timeout, aksing for update to shared variable server.");
			if(!getRemote())
				ROS_WARN_NAMED(ROS_NAME, "Could not get shared variable '%s', using old value.", shared_name_.c_str());
		}

		return true;
	}

private:
	// This function will be invoked by a client in order to get the state of the variable from the server
	bool getRemote()
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Getting shared variable '%s' using the 'get' service.", shared_name_.c_str());
		
		// Check if remote is available
		if(!remoteAvailable(service_client_get_))
			return false;

		auto ref  		= ros::conversion::convert<T>().get(value_);
		auto response 	= ros::conversion::convert<T>().get(value_);
		if ( service_client_get_.call(ref, response, ros::message_traits::MD5Sum<messageType>::value()) )
		{
			// Read the new value from the response
			value_ = ros::conversion::convert<T>().get(response);
			
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

		auto ref  						= ros::conversion::convert<T>().get(value_);
		auto response 					= ros::conversion::convert<T>().get(value_);
		if ( service_client_set_.call(ref, response, ros::message_traits::MD5Sum<messageType>::value()) )
		{
			value_ = ros::conversion::convert<T>().get(response);
			return true;
		}
		else
			return false;
	}

	// The server uses this to publish updates of the shared variable
	bool publishUpdate()
	{
		// Only publish if enabled and mange the publishing rate
		if(use_updates_ && ros::Time::now() - prev_publish_time_ > publish_rate_.expectedCycleTime())
        {
        	prev_publish_time_ = ros::Time::now();
            ROS_DEBUG_NAMED(ROS_NAME, "Publishing shared variable update via '%s'.", topic_name_updates_.c_str());
			updates_publisher_.publish(ros::conversion::convert<T>().get(value_));
        }

		return true;
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

private:
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
		value_ 			= ros::conversion::convert<T>().get(req);
		res 				= ros::conversion::convert<T>().get(value_); 
		
		return true;
	}

	// Is invoked at the server when an client calls its 'get variable' service
	bool CB_get(messageType & req, messageType & res)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Received shared variable 'get' service request.");
		res = ros::conversion::convert<T>().get(value_);
		return true;
	} 

	// Is invoked at the client when the server sends an update of this variable via pub/sub
	void CB_update(const boost::shared_ptr<messageType const>&  update)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Update of shared variable received from server.");
		auto prev_value_ = value_;
		value_ = ros::conversion::convert<T>().get(*update);

		// Call custom callback if registered, if the value changed
		if(prev_value_ != value_ and value_changed_CB_ != NULL)
			value_changed_CB_(value_);


		// Store the time of the last update
		last_update_received_ = ros::Time::now();
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
public:
	T& 					value_;
private:
	bool 				is_server_;
	bool 				is_client_;
	bool 				connected_;
	bool 				use_updates_;
	bool 				read_only_;

	std::string 		variable_name_;
	std::string 		shared_name_;
	std::string 		service_name_get_;
	std::string 		service_name_set_;
	std::string 		topic_name_updates_;


	ros::NodeHandle 	n_;
	ros::ServiceClient 	service_client_get_;
	ros::ServiceClient 	service_client_set_;
	ros::ServiceServer 	service_server_get_;
	ros::ServiceServer 	service_server_set_;

	ros::Subscriber 	updates_subscriber_;
	ros::Publisher 		updates_publisher_;

	ros::Time 			last_update_received_;
	ros::Time 			prev_publish_time_;
	ros::Duration 		max_age_;
	ros::Rate 			publish_rate_;

	changeCallbackType 	value_changed_CB_;

};

} // namespace shared_variables

#endif // UPDATE_ENGINE_HPP 
