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

#ifndef SHAREABLE_HPP
#define SHAREABLE_HPP

template<typename nativeType>
class Shareable
{
public:
	typedef boost::function<void()> SharedVariableGetCallback;
	typedef boost::function<void()> SharedVariableSetCallback;
	
	Shareable()
	{}

	Shareable(const nativeType& value)
		: value_(value)
	{}

	~Shareable()
	{}

	// Copy assignment
	Shareable<nativeType> operator=(Shareable<nativeType> rhs)
	{
		ROS_INFO("Copy operator Shareable");
		// Copy
		swap(*this, rhs);

		return *this;
	}

	// Assignment from native type 			
	Shareable<nativeType>& operator=(const nativeType& rhs)
	{
		ROS_INFO("Assignement operator Shareable");
		this->value_ = rhs;
		return *this;
	}

	// Conversion operator
	operator const nativeType&() const
	{
        ROS_INFO("Conversion operator Shareable");
		return this->value_;
	}

public:
	nativeType						value_;

private:
	void swap(Shareable<nativeType>& to, Shareable<nativeType>& from)
	{
		std::swap(to.value_				, from.value_);
		std::swap(to.get_cb_			, from.get_cb_);
		std::swap(to.set_cb_			, from.set_cb_);
	}

	SharedVariableGetCallback		get_cb_;
	SharedVariableSetCallback		set_cb_;
};

#endif // SHAREABLE_HPP 
