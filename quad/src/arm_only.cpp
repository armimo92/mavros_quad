#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>

mavros_msgs::State current_state;
mavros_msgs::RCIn RC_values;

mavros_msgs::SetMode offb_set_mode;
mavros_msgs::SetMode stabilize_set_mode;
mavros_msgs::SetMode althold_set_mode;

int switch_value;
float battery_voltage;

void rc_cb(const mavros_msgs::RCIn::ConstPtr& RCmsg)
{
	switch_value = RCmsg->channels[4];
}


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void battery_cb(const sensor_msgs::BatteryState::ConstPtr& Battmsg)
{
	battery_voltage = Battmsg->voltage;
}


bool Stabilize_flag = false;
bool Offboard_flag = false;
bool AltHold_flag = false;
bool offboard_selected = false;
bool offboard_active = false;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "quad_code");

	ros::NodeHandle nh;
	
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber RC_reading_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rc_cb);
	ros::Subscriber battery_level_sub = nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, battery_cb);

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
	


	offb_set_mode.request.custom_mode = "OFFBOARD";
	stabilize_set_mode.request.custom_mode = "STABILIZED";
	//althold_set_mode.request.custom_mode = "ALTCTL";
		
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	
	ros::Time last_request = ros::Time::now();
	
		
	
	
	
	while(ros::ok() && current_state.connected == true)
	{
	
	
		//READING FLIGHT MODE SWITCH FROM CHANNEL 5
		
		if(switch_value == 982)				//STABILIZED FLIGHT MODE SELECTED
		{
			Stabilize_flag = true;
			Offboard_flag = false;
			AltHold_flag = false;	
			
			offboard_selected = false;
		
		}
	
		if(switch_value == 1494)	//OFFBOARD FLIGHT MODE SELECTED
		{
			Stabilize_flag = false;
			Offboard_flag = true;
			AltHold_flag = false;
			
			offboard_selected = true;
			
			
		}
	
	
		if(switch_value == 2006)	//OFFBOARD FLIGHT MODE SELECTED
		{
			Stabilize_flag = false;
			Offboard_flag = false;
			AltHold_flag = true;	
			
			offboard_selected = false;
			
			
		}
		
		/*
		if(RC_values.channels[4] >= 1700 && RC_values.channels[4] < 2100) 	//ALTITUDE HOLD FLIGHT MODE SELECTED
		{
			Stabilize_flag = false;
			Offboard_flag = false;
			AltHold_flag = true;	
			
			offboard_selected = false;
			
		}
		*/
		
		if(offboard_selected == true && offboard_active == false)
		{		
			att_sp.type_mask = 0b00000111;
			att_sp.orientation.x = 0;
			att_sp.orientation.y = 0;
			att_sp.orientation.z = 0.05;
			att_sp.orientation.w = 0.9988;
		
			att_sp.thrust = 0.2;
			//send a few Setpoints before starting
			for(int i = 100; i > 0; i--)
			{
				//send some attitude or actuator control data HERE!!
       
			 	setpoint_raw_attitude_pub.publish(att_sp);
			 	ros::spinOnce();
			 	rate.sleep();
			}
		}
		
		
	
		//SETTING FLIGHT MODE FROM CODE
		
		if(current_state.mode != "STABILIZED" && Stabilize_flag == true && (ros::Time::now() - last_request > ros::Duration(1.0)) )
	  	{
	    	if(set_mode_client.call(stabilize_set_mode) && stabilize_set_mode.response.mode_sent)
	      {
					ROS_INFO("Stabilized Flight Mode Enabled");
					offboard_active = false;

	      }
	    
	    	last_request = ros::Time::now();
	  	}
	  
	  
	  if(current_state.mode != "OFFBOARD" && Offboard_flag == true && (ros::Time::now() - last_request > ros::Duration(3.0)))
	  	{
	  		
			
	    	if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	   		{
						ROS_INFO("Offboard Flight Mode Enabled");
						offboard_active = true;
						
	    	  }
	    	
	    		last_request = ros::Time::now();
	  	}
	  	
	  
	  if(current_state.mode != "ALTCTL" && AltHold_flag == true && (ros::Time::now() - last_request > ros::Duration(1.0)))
	  	{
			
	    	if(set_mode_client.call(althold_set_mode) && althold_set_mode.response.mode_sent)
	   		{
						ROS_INFO("AltHold Flight Mode Enabled");
						offboard_active = false;
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

	  

		if(current_state.mode == "OFFBOARD")
		{
			ROS_INFO("Flight mode: Offboard");
	  	setpoint_raw_attitude_pub.publish(att_sp);
	  }
	  
	  if(current_state.mode == "STABILIZED")
	  {
	  	ROS_INFO("Flight mode: Stabilized");
	  }
	  
	  if(current_state.mode == "ALTCTL")
	  {
	  	ROS_INFO("Flight mode: AltitudeControl");
	  }
	  
	  ROS_INFO("Battery level: %4f", battery_voltage);
	  ros::spinOnce();
	  rate.sleep();
	  
	}
	
	return 0;
}
