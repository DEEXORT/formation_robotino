/*
 * NorthStarROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef NORTHSTARROS_H_
#define NORTHSTARROS_H_

#include "rec/robotino/api2/NorthStar.h"

#include <ros/ros.h>
#include "robotino_msgs/NorthStarReadings.h"
#include "robotino_msgs/SetNsCalibratParam.h"

class NorthStarROS : public rec::robotino::api2::NorthStar
{
public:
	NorthStarROS();
	~NorthStarROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher north_star_pub_;
	
	/*!!!*/
	ros::ServiceServer set_NS_calibrat_param_;
	/**/
	robotino_msgs::NorthStarReadings north_star_msg_;

	ros::Time stamp_;
	/*!!!*/
	bool setNsCalibratParamCallback(robotino_msgs::SetNsCalibratParam::Request &req, robotino_msgs::SetNsCalibratParam::Response &res);
	/**/
	void readingsEvent( const rec::robotino::api2::NorthStarReadings& readings );
};

#endif /* NORTHSTARROS_H_ */
