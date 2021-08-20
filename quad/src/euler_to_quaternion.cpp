#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>

Eigen::Vector3f euler_data;	//roll, pitch, yaw
float thrust;
float normalized_thrust;

void attDesCallback(const geometry_msgs::Vector3::ConstPtr& attD)
{
	euler_data(0) = attD -> x;
	euler_data(1) = attD -> y;
	euler_data(2) = attD -> z;
}

void ForceInputCallback(const geometry_msgs::Vector3::ConstPtr& thr)
{
	thrust = thr->x;
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "euler_to_quaternion");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber eulerAtt_sub = nh.subscribe("Desired_attitude_ibvs",10, &attDesCallback);
	ros::Subscriber thrust_sub = nh.subscribe("Thrust_and_des_att",10, &ForceInputCallback);
	
	ros::Publisher normalized_thrust_pub = nh.advertise<std_msgs::Float64>("Thrust2MAV",10);
	ros::Publisher quaternion_pub = nh.advertise<geometry_msgs::Quaternion>("Quat2MAV",10);
	
	tf2::Quaternion myQuaternion;
	tf2::Quaternion q;
	tf2::Quaternion q_rot;
	
	std_msgs::Float64 norm_thrust;
	geometry_msgs::Quaternion quat;
	
	while(ros::ok())
	{
		/*Conversion from
						x		^
								|
								|
								|
				y	<------
				
				
			To
						x		^
								|
								|
								|
								---------->  y
			
		*/
		
		myQuaternion.setRPY(euler_data(0),euler_data(1),euler_data(2));
		q_rot.setRPY(3.141592,0,0);	//Axis frame rotation around the x-axis
		q = q_rot * myQuaternion;
		q.normalize(); //Normalize the quaternion i.e ||q|| = 1
		
		norm_thrust.data = thrust/32;	//Maximum thrust for the system is 32 N i.e. 8 N for each motor. 
		
		quat.x = q.x();
		quat.y = q.y();
		quat.z = q.z();
		quat.w = q.w();
		                                                                                                                         
		normalized_thrust_pub.publish(norm_thrust);
		quaternion_pub.publish(quat);
		
	
		
		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	
	
	
	return 0;
}

