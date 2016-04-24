#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>


#include <sstream>

char buff[50];
int fd;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	write(fd, msg->data.c_str(), msg->data.length() + 1);
	delayMicroseconds(500);
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
	
	// Create a publisher
	ros::Publisher pubStr = n.advertise<std_msgs::String>("ret_str", 1000);
	ros::Publisher pubVel = n.advertise<std_msgs::Int16>("ret_vel", 1000);	
	ros::Publisher pubOut = n.advertise<std_msgs::Int16>("ret_out", 1000);
	ros::Publisher pubPos = n.advertise<std_msgs::Int32>("ret_pos", 1000);
	
	ros::Rate loop_rate(10);
	
	// Set up wiringPi
	wiringPiSetupSys();
	if ( (fd = wiringPiI2CSetup(0x04)) == -1)
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
		int vel;
		int out;
		long int pos;
		
		// Spin once - (check for callbacks)
		ros::spinOnce();
		digitalWrite(21, output);
		output = (output == 1) ? 0 : 1;
		
		// Create the message object
		std_msgs::String msg_str;
		std_msgs::Int16 msg_vel;
		std_msgs::Int16 msg_out;
		std_msgs::Int32 msg_pos;

		// Read the velocity and publish it
		write(fd, "?!", 2);
		delayMicroseconds(500);
		read(fd, buff, 30);
		delayMicroseconds(50);
		printf("%s\n", buff);
		// Put the message into a string stream
		//ss << buff;
		// Pull out values using string stream
		//ss >> tempstr >> val;
		sscanf(buff, "V%dP%ldO%d", &vel, &pos, &out);
		msg_vel.data = vel;
		msg_out.data = out*100;
		msg_pos.data = pos;
		pubVel.publish(msg_vel);
		pubOut.publish(msg_out);
		pubPos.publish(msg_pos);
		
		
		
		// Read the debug string and publish
		write(fd, "??", 2);
		delayMicroseconds(500);
		read(fd, buff, 30);
		delayMicroseconds(50);
		msg_str.data = buff;
		pubStr.publish(msg_str);
		
		
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