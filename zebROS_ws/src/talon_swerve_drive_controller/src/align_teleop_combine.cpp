#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_srvs/SetBool.h"
#include <vector>

geometry_msgs::Twist processed_msg;

ros::Publisher processed_data_pub;

ros::ServiceServer hold_align_button;

bool button_hold = false;
double button_hold_start_time = -1;

bool holdAlignButtonServer(	std_srvs::SetBool::Request &req,
							std_srvs::SetBool::Response &res)
{
	button_hold = req.data;

	if(button_hold)
		button_hold_start_time = ros::Time::now().toSec();

	return true;
}

bool buttonHeld(void)
{
	//TODO make all these 0.25 seconds for press vs. hold config values (should pull from same config as align server)
	if(button_hold)
		return ((ros::Time::now().toSec() - button_hold_start_time) >= 0.25);

	return false;
}

void teleopDataCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if(!buttonHeld())
		return;

	processed_msg.linear.x = msg->linear.x;
	processed_msg.linear.y = msg->linear.y;

	processed_data_pub.publish(processed_msg);
}

void alignPidDataCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if(!buttonHeld())
		return;

	processed_msg.angular.z = msg->angular.z;

	processed_data_pub.publish(processed_msg);
}

int main(int argc, char ** argv)
{
	processed_msg.linear.x = 0;
	processed_msg.linear.y = 0;
	processed_msg.linear.z = 0;

	processed_msg.angular.x = 0;
	processed_msg.angular.y = 0;
	processed_msg.angular.z = 0;

	ros::init(argc, argv, "align_teleop_combine");
	ros::NodeHandle n;

	ros::Subscriber teleop_data_sub = n.subscribe("/teleop/swerve_drive_controller/cmd_vel", 5, teleopDataCallback);

	std::vector <ros::Subscriber> align_subscriber_array;

	align_subscriber_array.push_back(n.subscribe("/align_hatch/align_hatch/swerve_drive_controller/cmd_vel", 5, alignPidDataCallback));
	align_subscriber_array.push_back(n.subscribe("/align_cargo_rocketship/align_pid/swerve_drive_controller/cmd_vel", 5, alignPidDataCallback));
	align_subscriber_array.push_back(n.subscribe("/align_server/align_pid/swerve_drive_controller/cmd_vel", 5, alignPidDataCallback));
	align_subscriber_array.push_back(n.subscribe("/hatch_outtake/swerve_drive_controller/cmd_vel", 5, alignPidDataCallback));

	processed_data_pub = n.advertise<geometry_msgs::Twist>("align_teleop_combine/swerve_drive_controller/cmd_vel", 5);

	hold_align_button = n.advertiseService("swerve_drive_controller/hold_align_button", holdAlignButtonServer);

	ros::spin();

	return 0;
}
