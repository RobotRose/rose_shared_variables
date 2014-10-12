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

#ifndef SHAREABLE_HPP
#define SHAREABLE_HPP

namespace shared_variables
{

#define SHAREABLE_SPECIALIZATION(l, s) 	    \
	typedef l 				localType; 			\
	typedef s 				shareableType;			\
	typedef s::ConstPtr 	shareableTypeConstPtr;

// Base Class
class ShareableBaseClass
{
	public:
		ShareableBaseClass()
		{}
};

template<typename T>
class Shareable : public ShareableBaseClass
{
public:
	Shareable()
	{}
};



}; // namespace shared_variables 

#endif // SHAREABLE_HPP
