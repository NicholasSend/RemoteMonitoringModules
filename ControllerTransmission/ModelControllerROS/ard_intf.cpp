// Arduino Intf file

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


std_msgs::String tmp_str;

void messageCb (const std_msgs::String& msg)
{
  ROS_INFO("I heard: [%s]", msg->data);
  tmp_str.data = msg->data;
  
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ard_intf");


	ros::NodeHandle n;


	ros::Publisher ard_pub = n.advertise<std_msgs::String>("ard_in", 1000);
	//ros::Subscriber ard_sub = n.subscribe("ard_out", 1000, messageCb);
	ros::Subscriber udp_sub = n.subscribe("udp_out", 1000, messageCb);
	

	ros::Rate loop_rate(10);

	std_msgs::String ard_msg;

	while (ros::ok())
	{

		ard_msg.data = tmp_str.data;

		ROS_INFO("%s", ard_msg.data.c_str());


		chatter_pub.publish(ard_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}


  return 0;
}