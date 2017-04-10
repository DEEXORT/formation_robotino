/*
 * DigitalOutputArrayROS.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "DigitalOutputArrayROS.h"

DigitalOutputArrayROS::DigitalOutputArrayROS()
{
	digital_sub_ = nh_.subscribe("set_digital_values", 1,
			&DigitalOutputArrayROS::setDigitalValuesCallback, this);
}

DigitalOutputArrayROS::~DigitalOutputArrayROS()
{
	digital_sub_.shutdown();
}

void DigitalOutputArrayROS::setDigitalValuesCallback( const robotino_msgs::DigitalReadingsConstPtr& msg)
{
	//**This was written by indorewala@servicerobotics.eu (commented by CSI Department){	
	//int numValues = msg->values.size();
	//if( numValues > 0 )
	//{
	//	bool values[numValues];
	//
	//	memcpy( values, msg->values.data(), numValues * sizeof(bool) );
	//	//setValues( values, numValues );
	//}
	//** }
	
	//**This was written in CSI Department (ITMO University) {	
	if(msg->values.size() == numDigitalOutputs()){
		
		int values[numDigitalOutputs()];
		unsigned int i;

		for(i = 0; i < numDigitalOutputs(); i++)
			values[i] = msg->values[i];

		setValues(values, numDigitalOutputs());
	}
	//** }
}
