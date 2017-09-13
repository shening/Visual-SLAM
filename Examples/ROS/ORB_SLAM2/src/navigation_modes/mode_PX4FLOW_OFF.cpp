#include <ModeHeader.h>

void mode_PX4FLOW_OFF()
{
	//switch bool to true
	testFlow = false;
	desired_mode = "0";
	ROS_INFO("Turned PX4FLOW off");

}