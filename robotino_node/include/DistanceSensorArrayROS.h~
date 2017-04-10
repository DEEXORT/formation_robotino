/*
 * DistanceSensorArrayROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef DISTANCESENSORARRAYROS_H_
#define DISTANCESENSORARRAYROS_H_

#include "rec/robotino/api2/DistanceSensorArray.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

class DistanceSensorArrayROS: public rec::robotino::api2::DistanceSensorArray
{
public:
	DistanceSensorArrayROS();
	~DistanceSensorArrayROS();

	void setTimeStamp(ros::Time stamp);
	//**This was written in CSI Department (ITMO University) {
	void setTfPrefix(std::string &prefix);
	//** }
	
private:
	ros::NodeHandle nh_;

	ros::Publisher distances_pub_;
	
	//**This was written in CSI Department (ITMO University) {
	std::string tf_prefix;
	//** }

	sensor_msgs::PointCloud distances_msg_;

	ros::Time stamp_;

	void distancesChangedEvent(const float* distances, unsigned int size);

};


#endif /* DISTANCESENSORARRAYROS_H_ */
