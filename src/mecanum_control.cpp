#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include "mecanum_bot/motors.h"
#include "../util/PiPS2.h"
#include <wiringPi.h> 

class MecanumControl
{
	public:
		MecanumControl();
		void SetVelocities(float x, float y, float w);
		
	private:
		float wheelGeometryX[4];
		float wheelGeometryY[4];
		float wheelRad;
		int ticksPerRev;
		
		// Ros nodehandle. Main access point for all ROS comms
		ros::NodeHandle nh;
		
		// Message Objects
		mecanum_bot::motors motor_vels;
		
		// Pubs
		ros::Publisher velPub;
};

MecanumControl::MecanumControl()
{
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
	
	// -------- Create the publishers --------
	velPub = nh.advertise<mecanum_bot::motors>("vel_set", 100);
}

void MecanumControl::SetVelocities(float vx, float vy, float w)
{
	motor_vels.RightFront = (int32_t) -((vy - vx + w*(wheelGeometryX[0] + wheelGeometryY[0])) / wheelRad * ticksPerRev);
	motor_vels.RightRear = (int32_t) -((vy + vx + w*(wheelGeometryX[1] - wheelGeometryY[1])) / wheelRad * ticksPerRev);
	motor_vels.LeftRear = (int32_t) ((vy - vx + w*(wheelGeometryX[2] + wheelGeometryY[2])) / wheelRad * ticksPerRev);	
	motor_vels.LeftFront = (int32_t) ((vy + vx + w*(wheelGeometryX[3] - wheelGeometryY[3])) / wheelRad * ticksPerRev);
	
	// Publish
	velPub.publish(motor_vels);
}

int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "mecanum_control_node");
	
	// create the mecanum control object
	MecanumControl control;
	ros::Rate loop_rate(10);
	/*
	if (wiringPiSetupPhys () == -1)
	{
		fprintf (stdout, "Unable to start wiringPi\n") ;
	}*/
	wiringPiSetupSys();
	system("gpio -g mode 10 out");
	system("gpio -g mode 9 in");
	system("gpio -g mode 11 out");
	system("gpio -g mode 8 out");
	
	// Create a PIPS2 object
	PIPS2 pips2;
	//int nextRead = READDELAYMS;
	if (!pips2.initializeController(10, 9, 11, 8))
	{
		fprintf(stderr, "Failed to configure gamepad\nController is not responding.\nExiting ...\n");
	}
	int returnVal = pips2.reInitializeController(ALLPRESSUREMODE);
	if (returnVal == -1)
	{
		printf("Invalid Mode\n");
	} else if (returnVal == -2)
	{
		printf("Took too many tries to reinit.\n");
	}
	
	while (ros::ok())
	{
		// Read the controller.
		pips2.readPS2();		
		printf("Right Joy   Horizontal = %d\tVertical = %d\n", pips2.PS2data[5], pips2.PS2data[6]);
		printf("Right Joy   Horizontal = %d\tVertical = %d\n", pips2.PS2data[7], pips2.PS2data[8]);
		
		control.SetVelocities(200.0 * (pips2.PS2data[5]-128)/128.0, 200.0 * (127 - pips2.PS2data[6])/128.0, 0.314 * (pips2.PS2data[7] - 128)/128.0);
		ros::spinOnce();
		loop_rate.sleep();
	}
}