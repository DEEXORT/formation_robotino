/*
 * MotorArrayROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "MotorArrayROS.h"

MotorArrayROS::MotorArrayROS()
{
	motor_pub_ = nh_.advertise<robotino_msgs::MotorReadings>("motor_readings", 1, true);
}

MotorArrayROS::~MotorArrayROS()
{
	motor_pub_.shutdown();
}

void MotorArrayROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void MotorArrayROS::getMotorReadings(std::vector<float> &velocities, std::vector<int> &positions )
{
	velocities = motor_msg_.velocities;
	positions = motor_msg_.positions;
}

void MotorArrayROS::velocitiesChangedEvent( const float* velocities, unsigned int size )
{
	// Build the MotorReadings msg
	motor_msg_.velocities.resize( size, 0.0 );

	if( velocities != NULL )
	{
		//**This was written by indorewala@servicerobotics.eu (commented by CSI Department){	
		//memcpy( motor_msg_.velocities.data(), velocities, size * sizeof(float) );
		//** }
		//**This was written in CSI Department (ITMO University) {
		unsigned int i;
		for(i = 0; i < size; i++)
			motor_msg_.velocities[i] = velocities[i];	
		//** }
	}
}

//**This was written by indorewala@servicerobotics.eu (commented by CSI Department){	
//void MotorArrayROS::positionsChangedEvent( const float* positions, unsigned int size )
//** }
//**This was written in CSI Department (ITMO University) {
void MotorArrayROS::positionsChangedEvent( const int *positions, unsigned int size )
//** }
{
	// Build the MotorReadings msg
	motor_msg_.positions.resize( size, 0.0 );

	if( positions != NULL )
	{
		//**This was written by indorewala@servicerobotics.eu (commented by CSI Department){	
		//memcpy( motor_msg_.positions.data(), positions, size * sizeof(int) );
		//** }
		//**This was written in CSI Department (ITMO University) {
		unsigned int i;
		for(i = 0; i < size; i++)
			 motor_msg_.positions[i] = positions[i];	
		//** }
		
	}
}

void MotorArrayROS::currentsChangedEvent( const float* currents, unsigned int size )
{
	// Build the MotorReadings msg
	motor_msg_.stamp = stamp_;
	motor_msg_.currents.resize( size );

	if( currents != NULL )
	{
		//**This was written by indorewala@servicerobotics.eu (commented by CSI Department){	
		//memcpy( motor_msg_.currents.data(), currents, size * sizeof(float) );
		//** }
		//**This was written in CSI Department (ITMO University) {
		unsigned int i;
		for(i = 0; i < size; i++)
			motor_msg_.currents[i] = currents[i];	
		//** }
	}

	// Publish the msg
	motor_pub_.publish( motor_msg_ );
}
