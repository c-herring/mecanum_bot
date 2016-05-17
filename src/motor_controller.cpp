#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include "mecanum_bot/motors.h"
#include <geometry_msgs/Twist.h>

class MotorController
{
	public:
		MotorController();
		void PublishEncoders();
		void PublishVelocities();
	
	private:
		// I2C transfer buffers
		char TXbuf[50];
		char RXbuf1[50];
		char RXbuf2[50];
		int TWIDelay;
		// File pointer for I2C motor controllers
		int fdL; // Left
		int fdR; // Right
		// Ros nodehandle. Main access point for all ROS comms
		ros::NodeHandle nh;
		
		// Message objects
		std_msgs::String test_msg;
		mecanum_bot::motors enc_msg;
		mecanum_bot::motors vel_msg;
		geometry_msgs::Twist twist_msg;
		
		// Pubs
		ros::Publisher testPub;
		ros::Publisher encoderPub;
		ros::Publisher velPub;
		
		// Subs
		ros::Subscriber L_cmd; // Send a null terminated ASCII command string to left motor
		ros::Subscriber R_cmd; // ** to right motor
		ros::Subscriber set_vel; // Set velocities of each individual motor. Mostly for debug use.
		ros::Subscriber twist_vel; // Set velocities to achieve twist
		
		// Callbacks
		void L_cmd_callback(const std_msgs::String::ConstPtr& msg);
		void R_cmd_callback(const std_msgs::String::ConstPtr& msg);
		void set_vel_callback(const mecanum_bot::motors::ConstPtr& msg);
		void set_twist_callback(const geometry_msgs::Twist msg);
				
		// Robot Geometry
		float wheelGeometryX[4];
		float wheelGeometryY[4];
		float wheelRad;
		int ticksPerRev;
		
};

MotorController::MotorController()
{
	TWIDelay = 500;
	
	// Initialise geometry
	wheelGeometryX[0] = 160;
	wheelGeometryX[1] = 160;
	wheelGeometryX[2] = -160;
	wheelGeometryX[3] = -160;
	
	wheelGeometryY[0] = 135;
	wheelGeometryY[1] = -135;
	wheelGeometryY[2] = -135;
	wheelGeometryY[3] = 135;
	
	wheelRad = 66;
	ticksPerRev = 768;
	
	// Subscribe to the command topics
	L_cmd = nh.subscribe<std_msgs::String>("L_cmd", 100, &MotorController::L_cmd_callback, this);
	R_cmd = nh.subscribe<std_msgs::String>("R_cmd", 100, &MotorController::R_cmd_callback, this);
	set_vel = nh.subscribe<mecanum_bot::motors>("vel_set", 100, &MotorController::set_vel_callback, this);
	twist_vel = nh.subscribe<geometry_msgs::Twist>("twist_set", 100, &MotorController::set_twist_callback, this);
	
	// -------- Create the publishers --------
	testPub = nh.advertise<std_msgs::String>("test_pub", 100);
	encoderPub = nh.advertise<mecanum_bot::motors>("encoder_pos", 100);	
	velPub = nh.advertise<mecanum_bot::motors>("encoder_vel", 100);
	
	// -------- Set up wiringPi --------
	wiringPiSetupSys();
	if ( (fdL = wiringPiI2CSetup(0x04)) == -1)
	{
		printf("Failed to connect to device 0x04 - Exiting\n");
	}
	if ( (fdR = wiringPiI2CSetup(0x08)) == -1)
	{
		printf("Failed to connect to device 0x08 - Exiting\n");
	}
}

void MotorController::PublishEncoders()
{
	// Set controllers to output positions
	write(fdL, "?P", 2);
	delayMicroseconds(TWIDelay);
	write(fdR, "?P", 2);	
	delayMicroseconds(TWIDelay);
	
	// Read from each controller
	read(fdL, RXbuf1, 50);
	delayMicroseconds(TWIDelay);
	read(fdR, RXbuf2, 50);
	delayMicroseconds(TWIDelay);
	
	// Pull data out from the strings
	sscanf(RXbuf1, "PA%ldB%ld", &(enc_msg.LeftRear), &(enc_msg.LeftFront));	
	sscanf(RXbuf2, "PA%ldB%ld", &(enc_msg.RightFront), &(enc_msg.RightRear));

	// Publish
	encoderPub.publish(enc_msg);
}

void MotorController::PublishVelocities()
{	
	// Set controllers to output velocities
	write(fdL, "?V", 2);
	delayMicroseconds(TWIDelay);
	write(fdR, "?V", 2);	
	delayMicroseconds(TWIDelay);
	
	// Read from each controller
	read(fdL, RXbuf1, 50);
	delayMicroseconds(TWIDelay);
	read(fdR, RXbuf2, 50);
	delayMicroseconds(TWIDelay);
	
	// Pull data out from the strings
	sscanf(RXbuf1, "VA%ldB%ld", &(vel_msg.LeftRear), &(vel_msg.LeftFront));	
	sscanf(RXbuf2, "VA%ldB%ld", &(vel_msg.RightFront), &(vel_msg.RightRear));
	
	// Publish
	velPub.publish(vel_msg);
}

void MotorController::L_cmd_callback(const std_msgs::String::ConstPtr& msg)
{
	write(fdL, msg->data.c_str(), msg->data.length() + 1);
	delayMicroseconds(TWIDelay);
	ROS_INFO("Command sent to Left Controller: [%s]", msg->data.c_str());
}

void MotorController::R_cmd_callback(const std_msgs::String::ConstPtr& msg)
{
	write(fdR, msg->data.c_str(), msg->data.length() + 1);
	delayMicroseconds(TWIDelay);
	ROS_INFO("Command sent to Right Controller: [%s]", msg->data.c_str());
}

void MotorController::set_vel_callback(const mecanum_bot::motors::ConstPtr& msg)
{
	// Construct and send the command strings
	
	sprintf(TXbuf, "VA%ldB%ld", msg->LeftRear, msg->LeftFront);
	write(fdL, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
	
	sprintf(TXbuf, "VA%ldB%ld", msg->RightFront, msg->RightRear);
	write(fdR, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
}

void MotorController::set_twist_callback(const geometry_msgs::Twist msg)
{
	int32_t RightFront;
	int32_t LeftFront;
	int32_t RightRear;
	int32_t LeftRear;
	
	RightFront = (int32_t) -((msg.linear.y - msg.linear.x + msg.angular.z*(wheelGeometryX[0] + wheelGeometryY[0])) / wheelRad * ticksPerRev);
	RightRear = (int32_t) -((msg.linear.y + msg.linear.x + msg.angular.z*(wheelGeometryX[1] - wheelGeometryY[1])) / wheelRad * ticksPerRev);
	LeftRear = (int32_t) ((msg.linear.y - msg.linear.x + msg.angular.z*(wheelGeometryX[2] + wheelGeometryY[2])) / wheelRad * ticksPerRev);	
	LeftFront = (int32_t) ((msg.linear.y + msg.linear.x + msg.angular.z*(wheelGeometryX[3] - wheelGeometryY[3])) / wheelRad * ticksPerRev);
	
//	printf("linear x: %f, y: %f, z: %f\nangular x: %f, y: %f, z: %f\n", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
//	printf("VA%ldB%ld\t", LeftRear, LeftFront);
//	printf("VA%ldB%ld\n", RightFront, RightRear);
	
	// Construct and send the command strings
	sprintf(TXbuf, "VA%ldB%ld", LeftRear, LeftFront);
	write(fdL, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
	
	sprintf(TXbuf, "VA%ldB%ld", RightFront, RightRear);
	write(fdR, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
}


int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "controller_node");
	
	// Create the motor controller object
	MotorController controller;
	ros::Rate loop_rate(10);
	
	
	
	while (ros::ok())
	{
		
		// Get raw encoder positions and publish them
		controller.PublishEncoders();
		// Get raw encoder velocities and publish them
		controller.PublishVelocities();
		
		
		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();		
	}
}



















