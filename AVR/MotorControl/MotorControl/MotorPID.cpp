/*
 * MotorPID.cpp
 *
 * Created: 5/4/2016 8:49:38 PM
 *  Author: HeZ
 */ 

#include "MotorPID.h"
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

MotorPID::MotorPID(volatile uint8_t *_OCR, volatile uint8_t *_dirPort, uint8_t _fwPin, uint8_t _rvPin, PID _velocityPID)
{
	// Set the output compare register for this motor
	OCR = _OCR;
	// Set the port register and pins for direction
	dirPort = _dirPort;
	fwPin = _fwPin;
	rvPin = _rvPin;
	
	// Initialize PID Terms
	Perror = 0;
	PrevError = 0;
	Ierror = 0;
	dCounts = 0;
	lastCounts = 0;
	temperror = 0;
	output = 0;
	
	setVelPID(_velocityPID);
	
	// Motor Parameters
	motor.Velocity = 0;
	motor.counts = 0;
	motor.Pos = 0;
}

void MotorPID::setVelPID(PID _velocityPID)
{
	// PID parameters
	velocityPID.Kp = _velocityPID.Kp;
	velocityPID.Ki = _velocityPID.Ki;
	velocityPID.Kd = _velocityPID.Kd;
	velocityPID.K0 = _velocityPID.K0;
}

void MotorPID::snapshot(uint16_t _Td)
{
	dCounts = motor.counts - lastCounts; // Get number of encoder counts detected from last PID loop
	lastCounts = motor.counts; // Remember this counts
	motor.Pos = lastCounts;
	Td = _Td;
}

void MotorPID::spin()
{
	// Check the direction. This could be more elagent
	if (motor.Velocity > 0) {
		if (motor.state != Forward)
		{ // If we are switching directions, reset error
			PrevError = 0;
			Ierror = 0;
		}
		motor.state = Forward;
	}
	else if (motor.Velocity < 0) {
		if (motor.state != Reverse)
		{ // If we are switching directions, reset error
			PrevError = 0;
			Ierror = 0;
		}
		motor.state = Reverse;
	}
	else motor.state = Off;
	
	// Calculate motor velocity
	motor.vel = dCounts * 1000000 / (Td * 4); // Counts/s. 16MHz with 1/64 prescaler gives 4 µs per count. 
	// Calculate proportional error
	Perror = labs(motor.Velocity) - labs(motor.vel);
	
	temperror = (velocityPID.Kp*Perror + velocityPID.Kd*(Perror - PrevError) + velocityPID.Ki*Ierror)/velocityPID.K0;
	PrevError = Perror;
	output += temperror;
	
	// Update the integral error and limit it
	if (motor.Velocity == 0) {
		Ierror = 0; // If the motor velocity is zero then reset the errors
		PrevError = 0;
	}
	else Ierror += Perror;
	if (Ierror > 5000) Ierror = 5000;
	if (Ierror < -5000) Ierror = -5000;
	
	// Limit output to maximum
	if (output > 255) output = 255;
	if (output < 0) output = 0;
	// Set the direction
	switch (motor.state)
	{
		case Forward:
		*dirPort = (*dirPort & ~(fwPin | rvPin)) | fwPin;
		break;
		case Reverse:
		*dirPort = (*dirPort & ~(fwPin | rvPin)) | rvPin;
		break;
		case Off:
		*dirPort = (*dirPort & ~(fwPin | rvPin));
		break;
	}
	
	*OCR = output;
	
}