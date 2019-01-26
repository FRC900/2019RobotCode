#include <ros/ros.h>

/*
 * Publishes orient state, orient setpoint, y state, y setpoint, and pid_enable
 */

#include <screen_to_world/WorldVector.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

std_msgs::Float64 orient_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 orient_target_msg;
std_msgs::Float64 y_target_msg;
std_msgs::Bool pid_enable_msg;

ros::Subscriber world_vector_sub;
ros::Publisher orient_setpoint;
ros::Publisher orient_state;
ros::Publisher y_setpoint;
ros::Publisher y_state;
ros::Publisher pid_enable;

double orient_target_value;
double y_target_value;

void world_cb(screen_to_world::WorldVector msg)
{
	orient_state_msg.data = msg.slope;
	orient_state.publish(orient_state_msg);

	orient_state_msg.data = msg.slope;
	orient_state.publish(orient_state_msg);

	orient_target_msg.data = orient_target_value;
	orient_setpoint.publish(orient_target_msg);
	
	y_target_msg.data = y_target_value;
	y_setpoint.publish(y_target_msg);

	pid_enable_msg.data = true;
	pid_enable.publish(pid_enable_msg);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "tape_setpoint_pub");
	ros::NodeHandle nh;


	nh.getParam("orient_target_value", orient_target_value);
	nh.getParam("y_target_value", y_target_value);

	ros::Subscriber world_vector_sub = nh.subscribe("world", 1, world_cb);
	orient_setpoint = nh.advertise<std_msgs::Float64>("orient_setpoint", 1);
	orient_state = nh.advertise<std_msgs::Float64>("orient_state", 1);
	y_setpoint = nh.advertise<std_msgs::Float64>("y_setpoint", 1);
	y_state = nh.advertise<std_msgs::Float64>("y_state", 1);
	pid_enable = nh.advertise<std_msgs::Bool>("pid_enable", 1);

	ros::spin();

	return 0;
}
