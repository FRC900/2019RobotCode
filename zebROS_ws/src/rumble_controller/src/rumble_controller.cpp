#include "rumble_controller/rumble_controller.h"

namespace rumble_controller
{
	bool RumbleController::init(hardware_interface::PositionJointInterface *hw,
			ros::NodeHandle                 &/*root_nh*/,
			ros::NodeHandle                 &controller_nh)
	{
		rumble_joint_ = hw->getHandle("rumble");

		rumble_service_ = controller_nh.advertiseService("rumble_command", &RumbleController::cmdService, this);

		return true;
	}

	double RumbleController::convert(int leftRumble, int rightRumble)
	{
		const unsigned int rumble = ((leftRumble & 0xFFFF) << 16) | (rightRumble & 0xFFFF);
		const double rumble_val = *((double *)&rumble);
		return rumble_val;
	}

	void RumbleController::starting(const ros::Time &/*time*/) {
		//claw not released, mech not extended
		rumble_cmd_ = RumbleController::convert(0,0);
	}

	void RumbleController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		rumble_joint_.setCommand(rumble_cmd_);
	}

	void RumbleController::stopping(const ros::Time &/*time*/) {
	}

	bool RumbleController::cmdService(rumble_controller::RumbleSrv::Request &req, rumble_controller::RumbleSrv::Response &/*response*/) {
		if(isRunning())
		{
			rumble_cmd_ = RumbleController::convert(req.leftRumble,req.rightRumble);
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. RumbleController is not running.");
			return false;
		}
		return true;
	}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(rumble_controller::RumbleController, controller_interface::ControllerBase)
