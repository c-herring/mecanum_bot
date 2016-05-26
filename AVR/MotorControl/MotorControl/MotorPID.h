/*
 * MotorPID.h
 *
 * Created: 5/4/2016 9:01:51 PM
 *  Author: HeZ
 */ 


#ifndef MOTORPID_H_
#define MOTORPID_H_
#include <stdio.h>

class MotorPID
{
	public:
		typedef struct {
			uint16_t Kp; // PID Proportional term
			uint16_t Ki; // PID integral term
			uint16_t Kd; // PID differential term
			uint16_t K0; // Scaling term
		} PID;
		PID velocityPID;
		
		typedef enum {
			Forward,
			Reverse,
			Off
		} MotorState;
		
		typedef struct {
			MotorState state;
			int32_t Pos;
			int32_t Velocity; // Current velocity set point (will be accelerating or decelerating towards VelocitySet)
			uint16_t Acceleration; // Acceleration
			uint16_t Deceleration; // Deceleration
			int32_t VelocitySet; // Velocity set point
			volatile int32_t counts; // Number of counts that encoder has detected since last PID loop
			int32_t vel; // Current velocity. Need to
		} MotorInfo;
		MotorInfo motor;
				
		MotorPID(volatile uint8_t *_OCR, volatile uint8_t *_dirPort, uint8_t _fwPin, uint8_t _rvPin, PID _velocityPID);
		void spin(); // Spin the PID wheels
		void snapshot(uint16_t _Td);
		void setVelPID(PID _velocityPID);

		int32_t Perror;		// Proportional Error
		int32_t PrevError;	// Previous Proportional Error
		int32_t Ierror;		// Integral Error
		uint16_t Td;		// PID Loop Time
		int32_t dCounts;	// Number of encoder counts detected from last PID loop
		int32_t lastCounts;	// Number of encoder counts at last PID loop
		int32_t temperror;
		int32_t output;
		
	private:
		volatile uint8_t *OCR;
		volatile uint8_t *dirPort;
		char fwPin;
		char rvPin;
		
		
};



#endif /* MOTORPID_H_ */