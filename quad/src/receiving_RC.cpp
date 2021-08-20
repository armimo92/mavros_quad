#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>

mavros_msgs::RCIn RC_values;
mavros_msgs::State current_state;

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	RC_values = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "receiving_RC");

	ros::NodeHandle nh;
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber RC_reading_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rc_cb);
	
	ros::Rate rate(100);
	
	//wait for FCU connection
	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	
	while(ros::ok())
	{
	  ros::spinOnce();
	  rate.sleep();
	  
	}
	
	return 0;
}
