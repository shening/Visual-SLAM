#include <ModeHeader.h>

void mode_PX4FLOW_ON()
{
	//switch bool to true
	testFlow = true;
	desired_mode = "0";
	ROS_INFO("Turned PX4FLOW ON!!");
}