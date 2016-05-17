#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdlib.h>


#include <sstream>

char buff[50];
int fdL;
int fdR;
int32_t testNum;
#define TWIdelay 800

char TXbuf[20];
char RXbuf[20];
char outBuf[100];
int RXindex;

//int32_t outA;
//int32_t outB;
int32_t Velocity = 2500;

void pollController(int fd, int32_t *posA, int32_t *velA, int32_t *posB, int32_t *velB)
{
	// Get positions
	write(fd, "?P", 2);
	delayMicroseconds(TWIdelay);
	read(fd, buff, 50);
	delayMicroseconds(TWIdelay);
	// Pull out the numbers
	sscanf(buff, "PA%ldB%ld", posA, posB);
	delayMicroseconds(TWIdelay);
	
	// Get velocities
	write(fd, "?V", 2);
	delayMicroseconds(TWIdelay);
	read(fd, buff, 50);
	delayMicroseconds(TWIdelay);
	// Pull out the numbers
	sscanf(buff, "VA%ldB%ld", velA, velB);
	delayMicroseconds(TWIdelay);
		/*
	 // Get debug string
	 write(fd, "?t", 2);
	delayMicroseconds(500);
	read(fd, buff, 50);
	delayMicroseconds(500);
	// Pull out the numbers
	sscanf(buff, "x%ldx%ld", &out, &out);
	*/

}

void pollPID(int fd, uint16_t *Kp, uint16_t *Ki, uint16_t *Kd, uint16_t *K0)
{
	// Set output mode to PID
	write(fd, "?^", 2);
	delayMicroseconds(TWIdelay);
	read(fd, buff, 50);
	delayMicroseconds(TWIdelay);
	//sprintf(buff, "p%u i%u d%u o%u", 800, 600, 40, 1000);
	//int read = sscanf(buff, "a%u\ta%u\ta%u\ta%u", Kp, Ki, Kd, K0);
	// Well, this is annoying....
	sscanf(buff, "p%*u\ti%*u\td%*u\to%u", K0);
	sscanf(buff, "p%*u\ti%*u\td%u\to%*u", Kd);
	sscanf(buff, "p%*u\ti%u\td%*u\to%*u", Ki);
	sscanf(buff, "p%u\ti%*u\td%*u\to%*u", Kp);
	//printf("%read: %d\t%s\t(%d, %d, %d, %d)\n", read, buff, *Kp, *Ki, *Kd, *K0);
	//printf("%s\n", buff);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	write(fdL, msg->data.c_str(), msg->data.length() + 1);
	delayMicroseconds(TWIdelay);
	write(fdR, msg->data.c_str(), msg->data.length() + 1);
	delayMicroseconds(TWIdelay);
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "test_node");
	
	// Grab the node handle for this node. It is the main access point for all communications tis the ROS system
	ros::NodeHandle n;
	
	// Subscribe to the chatter topic
	ros::Subscriber sub = n.subscribe("send_cmd", 1000, chatterCallback);
	
	// vars
	int32_t velLA;
	int32_t posLA;
	int32_t velLB;
	int32_t posLB;
	int32_t velRA;
	int32_t posRA;
	int32_t velRB;
	int32_t posRB;
	
	uint16_t LKp;
	uint16_t LKi;
	uint16_t LKd;
	uint16_t LK0;
	uint16_t RKp;
	uint16_t RKi;
	uint16_t RKd;
	uint16_t RK0;
	
	// Create a publisher
	ros::Publisher pubStr = n.advertise<std_msgs::String>("ret_str", 1000);
	
	ros::Publisher pubVelLA = n.advertise<std_msgs::Int32>("ret_velLA", 1000);		
	ros::Publisher pubPosLA = n.advertise<std_msgs::Int32>("ret_posLA", 1000);
	ros::Publisher pubVelLB = n.advertise<std_msgs::Int32>("ret_velLB", 1000);		
	ros::Publisher pubPosLB = n.advertise<std_msgs::Int32>("ret_posLB", 1000);
	
	ros::Publisher pubVelRA = n.advertise<std_msgs::Int32>("ret_velRA", 1000);		
	ros::Publisher pubPosRA = n.advertise<std_msgs::Int32>("ret_posRA", 1000);
	ros::Publisher pubVelRB = n.advertise<std_msgs::Int32>("ret_velRB", 1000);		
	ros::Publisher pubPosRB = n.advertise<std_msgs::Int32>("ret_posRB", 1000);
	
	//ros::Publisher pubOutA = n.advertise<std_msgs::Int32>("ret_outLA", 1000);
	//ros::Publisher pubOutLB = n.advertise<std_msgs::Int32>("ret_outB", 1000);
	ros::Rate loop_rate(10);
	
	// Set up wiringPi
	wiringPiSetupSys();
	if ( (fdL = wiringPiI2CSetup(0x04)) == -1)
	{
		printf("Failed to connect to device - Exiting\n");
		return 1;
	}
	if ( (fdR = wiringPiI2CSetup(0x08)) == -1)
	{
		printf("Failed to connect to device - Exiting\n");
		return 1;
	}
	system("gpio -g mode 21 out"); // Use a system call to set output mode
	buff[11] = '\0';
	int output = 1;
	
	
	while (ros::ok())
	{
		std::stringstream ss;
		std::string tempstr;

		// Spin once - (check for callbacks)
		ros::spinOnce();
		digitalWrite(21, output);
		output = (output == 1) ? 0 : 1;
		
		
		// Create the message object
		std_msgs::String msg_str;
		std_msgs::Int32 msg_velLA;		
		std_msgs::Int32 msg_posLA;		
		std_msgs::Int32 msg_velLB;		
		std_msgs::Int32 msg_posLB;
		
		std_msgs::Int32 msg_velRA;		
		std_msgs::Int32 msg_posRA;		
		std_msgs::Int32 msg_velRB;		
		std_msgs::Int32 msg_posRB;
		
		//std_msgs::Int32 msg_outA;
		//std_msgs::Int32 msg_outB;
		
		// -------- Set the velocity of left motors --------
		sprintf(buff, "VA2000B2200");
		write(fdL, buff, 11);
		delayMicroseconds(TWIdelay);
		
		// -------- Set the velocity of right motors --------
		sprintf(buff, "VA2400B2600");
		write(fdR, buff, 11);
		delayMicroseconds(TWIdelay);
		
		// Get position and velocities of each motors
		pollController(fdR, &posRA, &velRA, &posRB, &velRB);
		pollController(fdL, &posLA, &velLA, &posLB, &velLB);
		
		
		
		
		// Set PID parameters
		//sprintf(buff, "^p4i2d3o4");
		//write(fdL, buff, 11);
		//delayMicroseconds(500);
		// Get PID parameters
		
		pollPID(fdR, &RKp, &RKi, &RKd, &RK0);
		pollPID(fdL, &LKp, &LKi, &LKd, &LK0);
		
		printf("Pos:%ld\t%ld\t%ld\t%ld\tVel:%ld\t%ld\t%ld\t%ld\n", posLA, posLB, posRA, posRB, velLA, velLB, velRA, velRB);
		printf("Test: %ld\n", posRB);
		printf("p%u-%u\ti%u-%u\t", LKp, RKp, LKi, RKi);
		printf("d%u-%u\to%u-%u\n", LKd, RKd, LK0, RK0);
		//printf("p%u\ti%u\td%u\to%u\n", Kp, Ki, Kd, K0);
		
		/*
		// Get the allcall string
		write(fd, "??", 2);
		delayMicroseconds(500);
		read(fd, buff, 50);
		delayMicroseconds(500);
		sscanf(buff, "PA%ldB%ldVA%ldB%ld", &posA, &posB, &velA, &velB);
		*/
	/*	
		// Get positions
		write(fd, "?P", 2);
		delayMicroseconds(500);
		read(fd, buff, 50);
		delayMicroseconds(500);
		// Pull out the numbers
		sscanf(buff, "PA%ldB%ld", &posA, &posB);
		
		// Get velocities
		write(fd, "?V", 2);
		delayMicroseconds(500);
		read(fd, buff, 50);
		delayMicroseconds(500);
		// Pull out the numbers
		sscanf(buff, "VA%ldB%ld", &velA, &velB);
		
		 // Get debug string
		 write(fd, "?t", 2);
		delayMicroseconds(500);
		read(fd, buff, 50);
		delayMicroseconds(500);
		// Pull out the numbers
		sscanf(buff, "x%ldx%ld", &outA, &outB);
	
		// Print the string to the console
		printf("outA=%ld\toutB=%ld\tVA=%ld\tVB=%ld\n\n", outA, outB, velA, velB);
		// Pull out the position and velocity
		//sscanf(buff, "PA%ldVA%ld", &pos, &vel);
	*/	
		// Place data in messages
		//msg_str.data = buff;
		
		msg_posLA.data = posLA;
		msg_velLA.data = velLA;		
		msg_posLB.data = posLB;
		msg_velLB.data = velLB;
		
		msg_posRA.data = posRA;
		msg_velRA.data = velRA;		
		msg_posRB.data = posRB;
		msg_velRB.data = velRB;
		
		//msg_outA.data = outA;
		//msg_outB.data = outB;
		// Publish
		//pubStr.publish(msg_str);
		
		pubVelLA.publish(msg_velLA);
		pubPosLA.publish(msg_posLA);		
		pubVelLB.publish(msg_velLB);
		pubPosLB.publish(msg_posLB);
		
		pubVelRA.publish(msg_velRA);
		pubPosRA.publish(msg_posRA);		
		pubVelRB.publish(msg_velRB);
		pubPosRB.publish(msg_posRB);
		
		//pubOutA.publish(msg_outA);
		//pubOutB.publish(msg_outB);
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		// -------- SET VEL --------
	//	char *velp = (char*)&Velocity;
	//	sprintf(TXbuf, "V%c%c", velp[1], velp[2]);
	//	write(fd, TXbuf, 3);
		  /*
		// -------- VELOCITY --------
		// Set output mode to velocity
		sprintf(TXbuf, "?V");
		write(fd, TXbuf, 2);
		delayMicroseconds(500);
		// Read the velocity
		read(fd, RXbuf, 4);
		delayMicroseconds(500);
		vel = (RXbuf[3] << 24) | (RXbuf[2] << 16) | (RXbuf[1] << 8) | RXbuf[0];
		  
		// -------- VELOCITY --------
		// Set output mode to velocity
		sprintf(TXbuf, "?P");
		write(fd, TXbuf, 2);
		delayMicroseconds(500);

		// Read the velocity
		read(fd, RXbuf, 4);
		delayMicroseconds(50000);
		pos = (RXbuf[3] << 24) | (RXbuf[2] << 16) | (RXbuf[1] << 8) | RXbuf[0];
		  
		  
		// Output
		printf("pos = %d\tvel = %d\n", pos, vel);
			*/
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		/*
		
		// Create the message object
		std_msgs::String msg_str;
		std_msgs::Int32 msg_vel;
		std_msgs::Int16 msg_out;
		std_msgs::Int32 msg_pos;

		sprintf(buff, "V2600");
		write(fd, buff, 6);
		delayMicroseconds(500);*/
/*		
		write(fd, "?P", 2);
		delayMicroseconds(500);
		// Read the velocity and publish it
		int16_t tempvel = 4002;
		//char *cvel = (char*)&tempvel;
		//char tempbuff[] = {'V', cvel[1], cvel[0]};
		//write(fd, tempbuff, 3);
		
		delayMicroseconds(500);
		read(fd, buff, 30);
		delayMicroseconds(500);
		printf("%s\n", buff);
		testNum = 0;
		//testNum = (buff[1] << 8) | buff[0];
		testNum = (buff[3] << 24) | (buff[2] << 16) | (buff[1] << 8) | buff[0];
		printf("%x%x%x%x\n", buff[0], buff[1], buff[2], buff[3]);
		printf("testnum = %ld\n", testNum);
		
		// Put the message into a string stream
		//ss << buff;
		// Pull out values using string stream
		//ss >> tempstr >> val;
		
		// Read velocity and publish it
		write(fd, "?V", 2);
		delayMicroseconds(500);
		read(fd, buff, 4);
		delayMicroseconds(500);
		vel = (buff[3] << 24) | (buff[2] << 16) | (buff[1] << 8) | buff[0];
		msg_vel.data = vel;
		pubVel.publish(msg_vel);
		*/
		/*
		sscanf(buff, "V%dP%ldO%d", &vel, &pos, &out);
		msg_vel.data = vel;
		msg_out.data = out*100;
		msg_pos.data = pos;
		pubVel.publish(msg_vel);
		pubOut.publish(msg_out);
		pubPos.publish(msg_pos);
		*/
		
		/*
		// Read the debug string and publish
		write(fd, "??", 2);
		delayMicroseconds(500);
		read(fd, buff, 50);
		delayMicroseconds(50);
		//testNum = (buff[3] << 24) | (buff[2] << 16) | (buff[1] << 8) | buff[0];
		//sprintf(buff, "vel = %l\n", testNum);
		msg_str.data = buff;
		//msg_vel.data = testNum;
		pubStr.publish(msg_str);
		//pubVel.publish(msg_vel);
		*/
		
		loop_rate.sleep();
		/*
		write(fd, "?V", 2);
		delayMicroseconds(50);
		read(fd, buff, 10);
		msg.data = buff;
		pub.publish(msg);
		loop_rate.sleep();
		
		write(fd, "?kpV", 4);
		delayMicroseconds(50);
		read(fd, buff, 10);
		msg.data = buff;
		pub.publish(msg);
		loop_rate.sleep();
		*/
		
		//read(fd, buff, 10);
		//delay(10);
		//printf("out = %d Read: %s\n", output, buff);

	}
	
	
	
	return 0;
	
}