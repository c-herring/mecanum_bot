#include "ros/ros.h"
#include <stdlib.h>
#include "../util/PiPS2.h"
#include <wiringPi.h> 
#include <geometry_msgs/Twist.h>

// Maximum velocity (magnitude) defines.
#define MAX_X 200.0 //In mm/s
#define MAX_Y 200.0 // In mm/s
#define MAX_W 0.314 // in rad/sec


class PlaystationTeleop
{
	public:
		PlaystationTeleop();
		void Teleop(char LeftHoriz, char RightHoriz, char RightVert);
		
	private:
		// Ros nodehandle. Main access point for all ROS comms
		ros::NodeHandle nh;
		
		// Message objects
		geometry_msgs::Twist twist_msg;
		
		// Pubs
		ros::Publisher twistPub;
};

PlaystationTeleop::PlaystationTeleop()
{
	// -------- Create the publishers --------
	twistPub = nh.advertise<geometry_msgs::Twist>("twist_set", 100);
}


void PlaystationTeleop::Teleop(char LeftHoriz, char RightHoriz, char RightVert)
{
	twist_msg.linear.x = MAX_X * (RightHoriz-128)/128.0;
	twist_msg.linear.y = MAX_Y * (127 - RightVert)/128.0;
	twist_msg.angular.z = MAX_W * (LeftHoriz - 128)/128.0;
	
	twistPub.publish(twist_msg);
}

int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "mecanum_bot_teleop_ps2");
	
	// create the mecanum control object
	PlaystationTeleop teleop;
	ros::Rate loop_rate(10);
	
	// Set up witing pi. NOTE: MUST RUN scripts/setPiGpio.sh first to export system pins
	wiringPiSetupSys();
	system("gpio -g mode 10 out");
	system("gpio -g mode 9 in");
	system("gpio -g mode 11 out");
	system("gpio -g mode 8 out");
	
	// Create a PIPS2 object
	PIPS2 pips2;
	// Initialise controller
	if (!pips2.initializeController(10, 9, 11, 8))
	{
		fprintf(stderr, "Failed to configure gamepad\nController is not responding.\nExiting ...\n");
		return -1;
	}
	// Now do a re-init to set the mode to all pressures returned
	int returnVal = pips2.reInitializeController(ALLPRESSUREMODE);
	if (returnVal == -1)
	{
		printf("Invalid Mode\n");
	} else if (returnVal == -2)
	{
		printf("Took too many tries to reinit.\n");
	}
	
	// Loop
	while (ros::ok())
	{
		// Read the controller.
		pips2.readPS2();		
		//printf("Right Joy   Horizontal = %d\tVertical = %d\n", pips2.PS2data[5], pips2.PS2data[6]);
		//printf("Right Joy   Horizontal = %d\tVertical = %d\n", pips2.PS2data[7], pips2.PS2data[8]);
		
		teleop.Teleop(pips2.PS2data[7], pips2.PS2data[5], pips2.PS2data[6]);
		ros::spinOnce();
		loop_rate.sleep();
	}
}