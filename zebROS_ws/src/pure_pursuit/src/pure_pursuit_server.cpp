#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pure_pursuit/PurePursuitAction.h>
#include <pure_pursuit/pure_pursuit.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
//tf stuff
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

double time_to_path;

class PurePursuitAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<pure_pursuit::PurePursuitAction> as_;
		std::string action_name_;
		nav_msgs::Odometry odom_msg_;
		tf2_ros::Buffer buffer_;
		std::string target_frame_;
		std::string odometry_topic_;
		ros::Publisher cmd_vel_pub_;

		//tf stuff
		tf2_ros::TransformListener tf2_;
		message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
		tf2_ros::MessageFilter<nav_msgs::Odometry> tf2_filter_;
		
		PurePursuit pure_pursuit_controller_;
	public:
		PurePursuitAction(const std::string &name) :
			as_(nh_, name, boost::bind(&PurePursuitAction::executeCB, this, _1), false),
			action_name_(name),
			pure_pursuit_controller_(nh_),
			odom_sub_(nh_, "pointstamped_goal_msg", 1),
			tf2_(buffer_),
			tf2_filter_(buffer_, target_frame_, 10, nh_)
	{
		as_.start();

		// Subscribing to odometry/odom transforms
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

		tf2_filter_.registerCallback(boost::bind(&PurePursuitAction::odomCallback, this, _1));

		nh_.getParam("/pure_pursuit/odometry_topic", odometry_topic_);
		nh_.getParam("/pure_pursuit/target_frame", target_frame_); //position of the robot at the beginning of the path
	}

		~PurePursuitAction(void) {}

		void odomCallback(nav_msgs::OdometryConstPtr odom)
		{
			odom_msg_ = *odom;
		}


		void executeCB(const pure_pursuit::PurePursuitGoalConstPtr &goal)
		{
			ROS_INFO("Hatch Panel PurePursuit Server Running");

			ros::Rate r(50);

			bool preempted = false;
			bool timed_out = false; 
			bool success = false;
			double percent_complete = 0;

			const double start_time = ros::Time::now().toSec();

			pure_pursuit_controller_.setup();

			pure_pursuit_controller_.loadPath(goal->path);

			//publish the current location, relative to the starting location of the robot, as a dynamic transform
			tf2_ros::TransformBroadcaster br;
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = ros::Time::now();
			transform.header.frame_id = "initial_pose";
			transform.child_frame_id = "base_link";
			transform.transform.translation.x = odom_msg_.pose.pose.position.x;
			transform.transform.translation.y = odom_msg_.pose.pose.position.y;
			transform.transform.translation.z = 0.0;
			transform.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
			transform.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
			transform.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
			transform.transform.rotation.w = odom_msg_.pose.pose.orientation.w;
			br.sendTransform(transform);

			geometry_msgs::Twist cmd_vel;
			nav_msgs::Odometry transformed_odom;
			while(ros::ok() && !timed_out && !preempted && !success)
			{
				try
				{
					buffer_.transform(odom_msg_, transformed_odom, target_frame_);
				}
				catch (tf2::TransformException &ex)
				{
					ROS_WARN("Failed %s\n", ex.what());
				}
				cmd_vel = pure_pursuit_controller_.run(transformed_odom);
				cmd_vel_pub_.publish(cmd_vel);

				preempted = as_.isPreemptRequested();
				timed_out = (start_time - ros::Time::now().toSec() > time_to_path);
			}

			pure_pursuit::PurePursuitResult result;
			if(timed_out)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result.success = false;
				as_.setSucceeded(result);
			}
			else if(preempted)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result.success = false;
				as_.setPreempted(result);
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result.success = true;
				as_.setSucceeded(result);
			}
			return;
		}
};


int main(int argc, char** argv)
{
	//create node
	ros::init(argc, argv, "pure_pursuit_server");

	PurePursuitAction pure_pursuit_server("pure_pursuit_server");

	ros::spin();

	return 0;
}
