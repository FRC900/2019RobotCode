#ifndef RUMBLE_INTAKE_CONTROLLER
#define RUMBLE_INTAKE_CONTROLLER

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include "rumble_controller/RumbleSrv.h"
#include "sensor_msgs/JointState.h"

namespace rumble_controller
{

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class RumbleController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            RumbleController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

	    virtual double convert(int leftRumble, int rightRumble);

            virtual bool cmdService(rumble_controller::RumbleSrv::Request &req,
					                rumble_controller::RumbleSrv::Response &res);

			void jointStateCallback(const sensor_msgs::JointState &joint_state);

        private:
            hardware_interface::JointHandle rumble_joint_;

			realtime_tools::RealtimeBuffer<double> rumble_cmd_; //buffer for clamp and extend commands

            ros::ServiceServer rumble_service_; //service for receiving commands
}; //class

} //namespace
#endif
