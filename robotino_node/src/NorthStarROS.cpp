/*
 * NorthStarROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "NorthStarROS.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

NorthStarROS::NorthStarROS()
{
	north_star_pub_ = nh_.advertise<robotino_msgs::NorthStarReadings>("north_star", 1, true);
	set_NS_calibrat_param_ = nh_.advertiseService("set_NS_calibrat_param", &NorthStarROS::setNsCalibratParamCallback, this);
}

NorthStarROS::~NorthStarROS()
{
	north_star_pub_.shutdown();
}

void NorthStarROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void NorthStarROS::readingsEvent( const rec::robotino::api2::NorthStarReadings& readings )
{
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(readings.posTheta);

	// Build the NorthStarReadings msg
	north_star_msg_.stamp 				= stamp_;
	north_star_msg_.seq 				= readings.sequenceNumber;
	north_star_msg_.roomId 				= readings.roomId;
	north_star_msg_.numSpotsVisible 	= readings.numSpotsVisible;


/*	north_star_msg_.pose.position.x 	= readings.posX;
	north_star_msg_.pose.position.y 	= readings.posY;
*/	north_star_msg_.pose.position.x 	= -readings.posY;
	north_star_msg_.pose.position.y 	= readings.posX;
	north_star_msg_.pose.position.z 	= 0.0;
	north_star_msg_.pose.orientation	= quat;

	// Publish the message
	north_star_pub_.publish( north_star_msg_ );
}

bool NorthStarROS::setNsCalibratParamCallback(robotino_msgs::SetNsCalibratParam::Request &req, robotino_msgs::SetNsCalibratParam::Response &res){
	if(req.value > 0.0)  	
		setCeilingCal(req.value);	
	return true;
}
