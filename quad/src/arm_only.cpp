#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

bool flag = false;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "to_do");

	ros::NodeHandle nh;
	
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::Publisher setpoint_raw_attitude_pub  = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
	
	ros::Rate rate(100);
	
	//wait for FCU connection
	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	
	mavros_msgs::AttitudeTarget att_sp;
	att_sp.type_mask = 0b00000111;
	att_sp.orientation.x = 0;
	att_sp.orientation.y = 0;
	att_sp.orientation.z = 0.05;
	att_sp.orientation.w = 0.9988;

	att_sp.thrust = 0.3;
	
	//send a few Setpoints before starting
	for(int i = 100; ros::ok() && i > 0; i--)
	{
		//send some attitude or actuator control data HERE!!
       
	  setpoint_raw_attitude_pub.publish(att_sp);
	  
	  ros::spinOnce();
	  rate.sleep();
	}
	
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	
	
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	
	ros::Time last_request = ros::Time::now();
	
	
	
	while(ros::ok())
	{

	  if(current_state.mode != "OFFBOARD" && flag == false && (ros::Time::now() - last_request > ros::Duration(5.0)))
	  {
	    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	      {
		ROS_INFO("Offboard enabled");
		flag = true;
	      }
	    
	    last_request = ros::Time::now();
	  }
	  
	  if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
	  {
	    if(arming_client.call(arm_cmd) && arm_cmd.response.success)
	    {
	      ROS_INFO("Vehicle armed");
	    }
	    
	    last_request = ros::Time::now();
	  }


	  setpoint_raw_attitude_pub.publish(att_sp);
	  ros::spinOnce();
	  rate.sleep();
	  
	}
	
	return 0;
}
