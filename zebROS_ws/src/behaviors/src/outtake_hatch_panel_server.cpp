#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <panel_intake_controller/PanelIntakeSrv.h>
#include <behaviors/PlaceAction.h>
#include <behaviors/ElevatorAction.h>
#include <frc_msgs/JoystickState.h>
#include <behaviors/FinishActionlib.h> //teleop joystick comp calls this to say finish the action after it's paused

//define global variables that will be defined based on config values
double elevator_timeout = 3;
double pause_time_between_pistons = 1;
double wait_for_server_timeout = 5;

class OuttakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::PlaceAction> as_;
		std::string action_name_;


		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient panel_controller_client_;

		ros::ServiceServer finish_actionlib_server_;
		bool finish_command_sent_; //stores whether teleop joystick comp sent a request to finish the action, which will pause halfway through. (on button release)

	public:
		OuttakeHatchPanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&OuttakeHatchPanelAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true)
	{
		as_.start();

		//do networking stuff?g
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		panel_controller_client_ = nh_.serviceClient<panel_intake_controller::PanelIntakeSrv>("/frcrobot_jetson/panel_intake_controller/panel_command", false, service_connection_header);

		//initialize subscriber for joint states
		finish_actionlib_server_ = nh_.advertiseService("finish_actionlib", &OuttakeHatchPanelAction::finishActionlibCB, this); //namespaces makes this name work
	}

		~OuttakeHatchPanelAction(void) {}

		void executeCB(const behaviors::PlaceGoalConstPtr &goal)
		{
			ROS_WARN("hatch panel outtake server running");

			//make sure the elevator server exists
			bool elevator_server_found = ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout));

			if(!elevator_server_found)
			{
				ROS_ERROR_STREAM("The elevator server was not loaded before the panel intake server needed it");
				as_.setPreempted();
				return;
			}

			ros::Rate r(10);
			//define variables that will be re-used for each call to a controller
			double start_time = ros::Time::now().toSec();

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

			finish_command_sent_ = false; //make sure this is what we think it should be

			/*/move elevator to outtake location
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = goal->setpoint_index;
			elev_goal.place_cargo = false;
			elev_goal.raise_intake_after_success = true;
			ac_elevator_.sendGoal(elev_goal);

			//wait to see if elevator server finishes before timeout
			bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(elevator_timeout - (ros::Time::now().toSec() - start_time), 0.001)));
			//determine final state of elevator server
			//if finished before timeout then it has either succeeded or failed, not timed out
			if(finished_before_timeout && !ac_elevator_.getResult()->timed_out) {
				//determine final state of elevator server
				actionlib::SimpleClientGoalState state = ac_elevator_.getState();
				//test if failed
				if(state.toString() != "SUCCEEDED") {
					ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
				}
				else {
					ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
				}
			}
			else {
				ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
			}


			//test if we got a preempt while waiting
			if(as_.isPreemptRequested())
			{
				preempted = true;
			}*/

			//send commands to panel_intake_controller to grab the panel ---------------------------------------
			if(!preempted && ros::ok())
			{
				//extend panel mechanism
				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_release = false;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv))
				{
					ROS_ERROR("Panel controller call failed in panel outtake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything


				//wait until the panel outtake button is released before retracting mech and lowering elevator
				while(ros::ok() && !preempted)
				{
					ROS_WARN("At loop!!!!!");
					//check if B button (panel outtake) was released
					if(finish_command_sent_)
					{
						ROS_ERROR("Boom!");
						break; //exit loop when button is released
					}
					//check if preempted
					if(as_.isPreemptRequested())
					{
						preempted = true;
					}
					else
					{
						//wait a bit before iterating the loop again
						r.sleep();
					}
				}
				finish_command_sent_ = false; //we're done processing this, so set it to false



				//release the panel - we can reuse the srv variable
				srv.request.claw_release = true;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Panel controller call failed in panel outtake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything


				//retract the panel mechanism and clamp
				srv.request.claw_release = true; //we can reuse the srv variable
				srv.request.push_extend = false;
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Panel controller call failed in panel outtake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything

			}

			ros::Duration(1).sleep();

			/*/lower elevator
			elev_goal.setpoint_index = goal->end_setpoint_index;
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);

			finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(elevator_timeout - (ros::Time::now().toSec() - start_time), 0.001)));
			if(finished_before_timeout && !ac_elevator_.getResult()->timed_out) {
				actionlib::SimpleClientGoalState state = ac_elevator_.getState();
				if(state.toString() != "SUCCEEDED") {
					ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
					preempted = true;
				}
				else {
					ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
				}
			}
			else {
				ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
				timed_out = true;
			}*/

			//set final state of mechanism - pulled in, clamped (to stay within frame perimeter)
			//it doesn't matter if timed out or preempted, do anyways
			//extend panel mechanism
			panel_intake_controller::PanelIntakeSrv srv;
			srv.request.claw_release = false;
			srv.request.push_extend = false;
			//send request to controller
			if(!panel_controller_client_.call(srv)) //call will happen no matter preempt outcome 
			{
				ROS_ERROR("Panel controller call failed in panel outtake server, final state of mechanism call.");
				preempted = true;
			}
			ros::spinOnce(); //update everything

			//log state of action and set result of action
			behaviors::PlaceResult result;
			result.timed_out = timed_out;

			if(timed_out)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result.success = false;
			}
			else if(preempted)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result.success = false;
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result.success = true;
			}
			as_.setSucceeded(result);
			return;

		}

		bool finishActionlibCB(behaviors::FinishActionlib::Request &req, behaviors::FinishActionlib::Response &/*res*/)
		{
			if(req.finish)
			{
				finish_command_sent_ = true;
			}
			return true;
		}
};

int main(int argc, char** argv)
{
	//create node
	ros::init(argc, argv, "panel_outtake_server");

	OuttakeHatchPanelAction outtake_hatch_panel_server("outtake_hatch_panel_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_outtake_params");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR("Could not read elevator_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("pause_time_between_pistons", pause_time_between_pistons))
		ROS_ERROR("Could not read pause_time_between_pistons in panel_outtake_sever");

	ros::spin();

	return 0;
}
