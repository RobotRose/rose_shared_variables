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

#ifndef SHARED_LIST_HPP
#define SHARED_LIST_HPP

#include "shared_variables/shared_variable.hpp"
#include <vector>

namespace shared_variables
{

template <typename T>
class Shareable<std::vector<T>>
{
public:
	Shareable()
	{}

	ros::ServiceClient createServiceClient(ros::NodeHandle& n, std::string service_name)
	{
		ROS_INFO_NAMED(ROS_NAME, "Creating service client std::vector<T>");

		bool persistent = false;
		ros::ServiceClientOptions ops(service_name,  md5sum(), persistent, ros::M_string());
     	return n.serviceClient(ops);
	}

	std::vector<Shareable<T>> serviceRequest(const std::vector<T>& request)
	{
		std::vector<Shareable<T>> req;

		for(const auto& item : request)
			req.push_back(Shareable<T>::serviceRequest(item));

		ROS_INFO_NAMED(ROS_NAME, "serviceRequest std::vector<T>");
		return req;
	}

	std::vector<Shareable<T>> serviceResponse()
	{
		std::vector<Shareable<T>> res;
		ROS_INFO_NAMED(ROS_NAME, "serviceResponse std::vector<Shareable<T>>");
		return res;
	}

	std::vector<T> translate(const std::vector<Shareable<T>>& response)
	{
		std::vector<T> translation;

		for(const auto& item : response)
			translation.push_back(Shareable<T>::translate(item));

		ROS_INFO_NAMED(ROS_NAME, "translate std::vector<Shareable<T>>");
		return response.data;
	}

	static const char* md5sum()
	{
		return Shareable<T>::md5sum();
	}
};

}; // namespace shared_variables

#endif // SHARED_LIST_HPP 
