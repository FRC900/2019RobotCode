/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on lookahead distance)
 * 4. transform goal point to vehicle coordinates and drive to that position
 */
#include "pure_pursuit/pure_pursuit.h"

void PurePursuit::setup()
{
	// Read config values
	nh_.getParam("/frcrobot_jetson/pure_pursuit/lookahead_distance", lookahead_distance_);
	nh_.getParam("/frcrobot_jetson/swerve_drive_controller/max_speed", max_velocity_);
	nh_.getParam("/frcrobot_jetson/swerve_drive_controller/max_accel", max_accel_);
	nh_.getParam("/frcrobot_jetson/pure_pursuit/pos_tol", pos_tol_);
	nh_.getParam("/frcrobot_jetson/pure_pursuit/final_pos_tol", final_pos_tol_);
	nh_.getParam("/frcrobot_jetson/pure_pursuit/deceleration", deceleration_);
}

// TODO : for large function parameters, making them const T & is more efficient
void PurePursuit::loadPath(nav_msgs::Path path)
{
	path_ = path;
	num_waypoints_ = path_.poses.size();
	last_idx_ = 0;
}

// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a Twist (or just x,y
// velocity) and whoever called run would be responsible for sending that
// where it needs to go.
geometry_msgs::Twist PurePursuit::run(nav_msgs::Odometry odom)
{
	ROS_INFO_STREAM("----------------------------------------------");
	ROS_INFO_STREAM("current_position = " << odom.pose.pose.position.x
			<< " " << odom.pose.pose.position.y);

	geometry_msgs::PoseStamped next_waypoint;

	// Find posize_t in path closest to odometry reading
	double minimum_distance = std::numeric_limits<double>::max();
	size_t minimum_idx = 0;
	for(size_t i = last_idx_; i < num_waypoints_; i++)
	{
		ROS_INFO_STREAM("waypoint " << i << " = " << path_.poses[i].pose.position.x << ", " << path_.poses[i].pose.position.y);
		ROS_INFO_STREAM("distance from waypoint " << i << " = " << hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y));
		if(hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y) < minimum_distance)
		{
			minimum_distance = hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y);
			minimum_idx = i;
		}
	}
	ROS_INFO_STREAM("minimum_distance = " << minimum_distance);
	last_idx_ = minimum_idx;

	double dist_from_endpoint = hypot(path_.poses[num_waypoints_ - 1].pose.position.x - odom.pose.pose.position.x, path_.poses[num_waypoints_ - 1].pose.position.y- odom.pose.pose.position.y);
	//double last_velocity = hypot(cmd_vel.linear.x, cmd_vel.linear.y);
	//double distance_to_decelerate = pow(last_velocity, 2)/deceleration_/2;

	if(dist_from_endpoint > lookahead_distance /*distance_to_decelerate*/)
	{
		for(size_t i = minimum_idx; i < path_.poses.size(); i++)
		{
			if(hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y- odom.pose.pose.position.y) > lookahead_distance_)
			{
				next_waypoint = path_.poses[i - 1];
				break;
			}
		}
	}
	else
	{
		/*cmd_vel_.linear.x = 0;
		cmd_vel_.linear.y = 0;
		cmd_vel_.linear.z = 0;
		cmd_vel_.angular.x = 0;
		cmd_vel_.angular.y = 0;
		cmd_vel_.angular.z = 0;*/
		next_waypoint = path_.poses[num_waypoints_ - 1];
	}


	ROS_INFO_STREAM("x-error: " << fabs(odom.pose.pose.position.x - next_waypoint.pose.position.x) << " y-error: " << fabs(odom.pose.pose.position.y - next_waypoint.pose.position.y) << " final_pos_tol: " << final_pos_tol_);
	if(minimum_idx == num_waypoints_ - 1 && fabs(odom.pose.pose.position.x - path_.poses[num_waypoints_ - 1].pose.position.x) < final_pos_tol_ && fabs(odom.pose.pose.position.y - path_.poses[num_waypoints_ - 1].pose.position.y) < final_pos_tol_)
	{
		cmd_vel_.linear.x = 0;
		cmd_vel_.linear.y = 0;
		cmd_vel_.linear.z = 0;
		cmd_vel_.angular.x = 0;
		cmd_vel_.angular.y = 0;
		cmd_vel_.angular.z = 0;

		return cmd_vel_;
	}

	ROS_INFO_STREAM("next_waypoint = " << next_waypoint.pose.position.x
			<< " " << next_waypoint.pose.position.y
			<< " " << next_waypoint.pose.position.z);

	// Set the angle of the velocity
	geometry_msgs::Point32 base_link_waypoint;
	base_link_waypoint.x = next_waypoint.pose.position.x - odom.pose.pose.position.x;
	base_link_waypoint.y = next_waypoint.pose.position.y - odom.pose.pose.position.y;
	double mag = hypot(base_link_waypoint.x, base_link_waypoint.y);
	ROS_INFO_STREAM("distance to drive to next waypoint = " << mag);
	ROS_INFO_STREAM("max_velocity = " << max_velocity_);
	ROS_INFO_STREAM("distance to drive x = " << base_link_waypoint.x << "; distance to drive y = " << base_link_waypoint.y);
	cmd_vel_.linear.x = max_velocity_ * base_link_waypoint.x / mag;
	cmd_vel_.linear.y = max_velocity_ * base_link_waypoint.y / mag;
	cmd_vel_.linear.z = 0;
	cmd_vel_.angular.x = 0;
	cmd_vel_.angular.y = 0;
	cmd_vel_.angular.z = 0;

	return cmd_vel_;
}
