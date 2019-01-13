#pragma once
#include <spark_max_interface/spark_max_state_interface.h>

namespace hardware_interface
{

class SparkMaxHWCommand
{
	public:
		SparkMaxHWCommand()
			: set_point_(0)
			, set_point_changed_(false)
			, inverted_(false)
			, inverted_changed_(false)
			, p_gain_{0}
			, i_gain_{0}
			, d_gain_{0}
			, f_gain_{0}
			, i_zone_{0}
			, d_filter_{0}
			, pidf_constants_changed_{false}
			, pidf_output_min_{-1}
			, pidf_output_max_{1}
			, pidf_reference_value_{0}
			, pidf_reference_value_changed_{false}
			, pidf_reference_ctrl_(kDutyCycle)
			, pidf_reference_ctrl_changed_(false)
			, pidf_reference_slot_(0)
			, pidf_reference_slot_changed_(false)
			, pidf_arb_feed_forward_{0}
			, pidf_config_changed_{false}
			, forward_limit_switch_polarity_(kNormallyOpen)
			, forward_limit_switch_enabled_(false)
			, forward_limit_switch_changed_(false)
			, reverse_limit_switch_polarity_(kNormallyOpen)
			, reverse_limit_switch_enabled_(false)
			, reverse_limit_switch_changed_(false)
			, current_limit_(0) // TODO: Better defaults
			, current_limit_stall_(0)
			, current_limit_free_(0)
			, current_limit_rpm_(0)
			, current_limit_changed_(false)
			, secondary_current_limit_(0)
			, secondary_current_limit_cycles_(0)
			, secondary_current_limit_changed_(false)

			, idle_mode_(kCoast)
			, idle_mode_changed_(false)
			, ramp_rate_(0)
			, ramp_rate_changed_(false)
			, follower_type_(kFollowerDisabled)
			, follower_id_(-1)
			, follower_invert_(false)
			, follower_changed_(false)
		{
		}

		void setSetPoint(double set_point)
		{
			if (set_point_ != set_point)
			{
				set_point_ = set_point;
				set_point_changed_ = true;
			}
		}
		double getSetPoint(void) const
		{
			return set_point_;
		}
		bool changedSetPoint(double &set_point)
		{
			set_point = set_point_;
			if (set_point_changed_)
			{
				set_point_changed_ = false;
				return true;
			}
			return false;
		}

		void setInverted(bool inverted)
		{
			if (inverted_ != inverted)
			{
				inverted_ = inverted;
				inverted_changed_ = true;
			}
		}
		bool getInverted(void) const
		{
			return inverted_;
		}
		bool changedInverted(bool &inverted)
		{
			inverted = inverted_;
			if (inverted_changed_)
			{
				inverted_changed_ = false;
				return true;
			}
			return false;
		}

		void setPGain(size_t slot, double p_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPGain() : invalid slot " << slot);
				return;
			}
			if (p_gain != p_gain_[slot])
			{
				p_gain_[slot] = p_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getPGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPGain() : invalid slot " << slot);
				return -1;
			}
			return p_gain_[slot];
		}

		void setIGain(size_t slot, double i_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setIGain() : invalid slot " << slot);
				return;
			}
			if (i_gain != i_gain_[slot])
			{
				i_gain_[slot] = i_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getIGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getIGain() : invalid slot " << slot);
				return -1;
			}
			return i_gain_[slot];
		}
		void setDGain(size_t slot, double d_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setDGain() : invalid slot " << slot);
				return;
			}
			if (d_gain != d_gain_[slot])
			{
				d_gain_[slot] = d_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getDGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getDGain() : invalid slot " << slot);
				return -1;
			}
			return d_gain_[slot];
		}

		void setFGain(size_t slot, double f_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setFGain() : invalid slot " << slot);
				return;
			}
			if (f_gain != f_gain_[slot])
			{
				f_gain_[slot] = f_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getFGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getFGain() : invalid slot " << slot);
				return -1;
			}
			return f_gain_[slot];
		}

		void setIZone(size_t slot, double i_zone)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setIZone() : invalid slot " << slot);
				return;
			}
			if (i_zone != i_zone_[slot])
			{
				i_zone_[slot] = i_zone;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getIZone(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getIZone() : invalid slot " << slot);
				return -1;
			}
			return i_zone_[slot];
		}

		void setDFilter(size_t slot, double d_filter)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setDFilter() : invalid slot " << slot);
				return;
			}
			if (d_filter != d_filter_[slot])
			{
				d_filter_[slot] = d_filter;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getDFilter(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getDFilter() : invalid slot " << slot);
				return -1;
			}
			return d_filter_[slot];
		}

		bool changedPIDFConstants(size_t slot,
				double &p_gain, double &i_gain,
				double &d_gain, double &f_gain,
				double &i_zone, double &d_filter)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDConstants() : invalid slot " << slot);
				return false;
			}
			p_gain = p_gain_[slot];
			i_gain = i_gain_[slot];
			d_gain = d_gain_[slot];
			f_gain = f_gain_[slot];
			i_zone = i_zone_[slot];
			d_filter = d_filter_[slot];
			if (pidf_constants_changed_[slot])
			{
				pidf_constants_changed_[slot] = false;
				return true;
			}
			return false;
		}

		void setPIDFOutputMin(size_t slot, double pidf_output_min)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFOutputMin() : invalid slot " << slot);
				return;
			}
			if (pidf_output_min != pidf_output_min_[slot])
			{
				pidf_output_min_[slot] = pidf_output_min;
				pidf_config_changed_[slot] = true;
			}
		}
		double getPIDFOutputMin(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFOutputMin() : invalid slot " << slot);
				return std::numeric_limits<double>::max();
			}
			return pidf_output_min_[slot];
		}

		void setPIDFOutputMax(size_t slot, double pidf_output_max)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFOutputMax() : invalid slot " << slot);
				return;
			}
			if (pidf_output_max != pidf_output_max_[slot])
			{
				pidf_output_max_[slot] = pidf_output_max;
				pidf_config_changed_[slot] = true;
			}
		}
		double getPIDFOutputMax(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFOutputMax() : invalid slot " << slot);
				return -std::numeric_limits<double>::max();
			}
			return pidf_output_max_[slot];
		}

		void setPIDFReferenceCtrl(ControlType pidf_reference_ctrl)
		{
			if (pidf_reference_ctrl != pidf_reference_ctrl_)
			{
				pidf_reference_ctrl_ = pidf_reference_ctrl;
				pidf_reference_ctrl_changed_ = true;
			}
		}
		ControlType getPIDFReferenceCtrl(void) const
		{
			return pidf_reference_ctrl_;
		}
		bool changedPIDFReferenceCtrl(ControlType &pidf_reference_ctrl)
		{
			pidf_reference_ctrl = pidf_reference_ctrl_;
			if (pidf_reference_ctrl_changed_)
			{
				pidf_reference_ctrl_changed_ = false;
				return true;
			}
		}

		void setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFArbFeedForward() : invalid slot " << slot);
				return;
			}
			if (pidf_arb_feed_forward != pidf_arb_feed_forward_[slot])
			{
				pidf_arb_feed_forward_[slot] = pidf_arb_feed_forward;
				pidf_config_changed_[slot] = true;
			}
		}
		double getPIDFArbFeedForward(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFArbFeedForward() : invalid slot " << slot);
				return -1;
			}
			return pidf_arb_feed_forward_[slot];
		}

		bool changedPIDFConfig(size_t slot,
				double &output_min,
				double &output_max,
				double &arb_feed_forward)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::changedPIDFConfig() : invalid slot " << slot);
				return false;
			}
			output_min = pidf_output_min_[slot];
			output_max = pidf_output_max_[slot];
			arb_feed_forward = pidf_arb_feed_forward_[slot];
			if (pidf_config_changed_[slot])
			{
				pidf_config_changed_[slot] = false;
				return true;
			}
			return false;
		}

		void setPIDFReferenceSlot(int slot)
		{
			if ((slot < 0) || (slot >= SPARK_MAX_PID_SLOTS))
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFReferenceSlot() : invalid slot " << slot);
				return;
			}
			if (slot != pidf_reference_slot_)
			{
				pidf_reference_slot_ = slot;
				pidf_reference_slot_changed_ = true;
			}
		}
		int getPIDFReferenceSlot(void) const
		{
			return pidf_reference_slot_;
		}

		bool changedPIDFReferenceSlot(int &pidf_reference_slot)
		{
			pidf_reference_slot = pidf_reference_slot_;
			if (pidf_reference_slot_changed_)
			{
				pidf_reference_slot_changed_ = false;
				return true;
			}
			return false;
		}


		void setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity)
		{
			if (forward_limit_switch_polarity_ != forward_limit_switch_polarity)
			{
				forward_limit_switch_polarity_ = forward_limit_switch_polarity;
				forward_limit_switch_changed_ = true;
			}
		}
		LimitSwitchPolarity getForwardLimitSwitchPolarity(void) const
		{
			return forward_limit_switch_polarity_;
		}

		void setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled)
		{
			if (forward_limit_switch_enabled_ != forward_limit_switch_enabled)
			{
				forward_limit_switch_enabled_ = forward_limit_switch_enabled;
				forward_limit_switch_changed_ = true;
			}
		}
		bool getForwardLimitSwitchEnabled(void) const
		{
			return forward_limit_switch_enabled_;
		}

		bool changedForwardLimitSwitch(
				LimitSwitchPolarity &forward_limit_switch_polarity,
				bool &forward_limit_switch_enabled)
		{
			forward_limit_switch_polarity = forward_limit_switch_polarity_;
			forward_limit_switch_enabled = forward_limit_switch_enabled_;
			if (!forward_limit_switch_changed_)
			{
				forward_limit_switch_changed_ = false;
				return true;
			}
			return false;
		}

		void setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity)
		{
			if (reverse_limit_switch_polarity_ != reverse_limit_switch_polarity)
			{
				reverse_limit_switch_polarity_ = reverse_limit_switch_polarity;
				reverse_limit_switch_changed_ = true;
			}
		}
		LimitSwitchPolarity getReverseLimitSwitchPolarity(void) const
		{
			return reverse_limit_switch_polarity_;
		}

		void setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled)
		{
			if (reverse_limit_switch_enabled_ != reverse_limit_switch_enabled)
			{
				reverse_limit_switch_enabled_ = reverse_limit_switch_enabled;
				reverse_limit_switch_changed_ = true;
			}
		}
		bool getReverseLimitSwitchEnabled(void) const
		{
			return reverse_limit_switch_enabled_;
		}

		bool changedReverseLimitSwitch(
				LimitSwitchPolarity &reverse_limit_switch_polarity,
				bool &reverse_limit_switch_enabled)
		{
			reverse_limit_switch_polarity = reverse_limit_switch_polarity_;
			reverse_limit_switch_enabled = reverse_limit_switch_enabled_;
			if (!reverse_limit_switch_changed_)
			{
				reverse_limit_switch_changed_ = false;
				return true;
			}
			return false;
		}

		void setCurrentLimit(unsigned int current_limit)
		{
			if (current_limit_ != current_limit)
			{
				current_limit_ = current_limit;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimit(void) const
		{
			return current_limit_;
		}

		void setCurrentLimitStall(unsigned int current_limit_stall)
		{
			if (current_limit_stall_ != current_limit_stall)
			{
				current_limit_stall_ = current_limit_stall;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimitStall(void) const
		{
			return current_limit_stall_;
		}

		void setCurrentLimitFree(unsigned int current_limit_free)
		{
			if (current_limit_free_ != current_limit_free)
			{
				current_limit_free_ = current_limit_free;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimitFree(void) const
		{
			return current_limit_free_;
		}

		void setCurrentLimitRPM(unsigned int current_limit_rpm)
		{
			if (current_limit_rpm_ != current_limit_rpm)
			{
				current_limit_rpm_ = current_limit_rpm;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimitRPM(void) const
		{
			return current_limit_rpm_;
		}

		bool changedCurrentLimit(unsigned int &current_limit,
				unsigned int &current_limit_stall,
				unsigned int &current_limit_rpm)
		{
			current_limit = current_limit_;
			current_limit_stall = current_limit_stall_;
			current_limit_rpm = current_limit_rpm_;
			if (current_limit_changed_)
			{
				current_limit_changed_ = false;
				return true;
			}
			return false;
		}

		void setSecondaryCurrentLimit(unsigned int secondary_current_limit)
		{
			if (secondary_current_limit_ != secondary_current_limit)
			{
				secondary_current_limit_ = secondary_current_limit;
				secondary_current_limit_changed_ = true;
			}
		}
		unsigned int getSecondaryCurrentLimit(void) const
		{
			return secondary_current_limit_;
		}

		void setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles)
		{
			if (secondary_current_limit_cycles_ != secondary_current_limit_cycles)
			{
				secondary_current_limit_cycles_ = secondary_current_limit_cycles;
				secondary_current_limit_changed_ = true;
			}
		}
		unsigned int getSecondaryCurrentLimitCycles(void) const
		{
			return secondary_current_limit_cycles_;
		}

		bool changedSecondaryCurrentLimits(unsigned &secondary_current_limit,
				unsigned int &secondary_current_limit_cycles)
		{
			secondary_current_limit = secondary_current_limit_;
			secondary_current_limit_cycles = secondary_current_limit_cycles_;
			if (secondary_current_limit_changed_)
			{
				secondary_current_limit_changed_ = false;
				return true;
			}
			return false;
		}


		void setIdleMode(IdleMode idle_mode)
		{
			if (idle_mode_ != idle_mode)
			{
				idle_mode_ = idle_mode;
				idle_mode_changed_ = true;
			}
		}
		IdleMode getIdleMode(void) const
		{
			return idle_mode_;
		}
		bool changedIdleMode(IdleMode &idle_mode)
		{
			idle_mode = idle_mode_;
			if (idle_mode_changed_)
			{
				idle_mode_changed_ = false;
				return true;
			}
			return false;
		}

		void setRampRate(double ramp_rate)
		{
			if (ramp_rate_ != ramp_rate)
			{
				ramp_rate_ = ramp_rate;
				ramp_rate_changed_ = true;
			}
		}
		double getRampRate(void) const
		{
			return ramp_rate_;
		}
		bool changedRampRate(double &ramp_rate)
		{
			ramp_rate = ramp_rate_;
			if (ramp_rate_changed_)
			{
				ramp_rate_changed_ = false;
				return true;
			}
			return false;
		}

		void setFollowerType(ExternalFollower follower_type)
		{
			if (follower_type_ != follower_type)
			{
				follower_type_ = follower_type;
				follower_changed_ = true;
			}
		}
		ExternalFollower getFollowerType(void) const
		{
			return follower_type_;
		}

		void setFollowerID(double follower_id)
		{
			if (follower_id_ != follower_id)
			{
				follower_id_ = follower_id;
				follower_changed_ = true;
			}
		}
		double getFollowerID(void) const
		{
			return follower_id_;
		}

		void setFollowerInvert(double follower_invert)
		{
			if (follower_invert_ != follower_invert)
			{
				follower_invert_ = follower_invert;
				follower_changed_ = true;
			}
		}
		double getFollowerInvert(void) const
		{
			return follower_invert_;
		}

		bool changedFollower(ExternalFollower &follower_type,
				int &follower_id,
				bool &follower_invert)
		{
			follower_type = follower_type_;
			follower_id = follower_id_;
			follower_invert = follower_invert_;
			if (follower_changed_)
			{
				follower_changed_ = false;
				return true;
			}
			return false;
		}

	private:
		double              set_point_;
		bool                set_point_changed_;
		bool                inverted_;
		bool                inverted_changed_;

		// PID Controller
		double              p_gain_[SPARK_MAX_PID_SLOTS];
		double              i_gain_[SPARK_MAX_PID_SLOTS];
		double              d_gain_[SPARK_MAX_PID_SLOTS];
		double              f_gain_[SPARK_MAX_PID_SLOTS];
		double              i_zone_[SPARK_MAX_PID_SLOTS];
		double              d_filter_[SPARK_MAX_PID_SLOTS];
		bool                pidf_constants_changed_[SPARK_MAX_PID_SLOTS];

		double              pidf_output_min_[SPARK_MAX_PID_SLOTS];
		double              pidf_output_max_[SPARK_MAX_PID_SLOTS];

		double              pidf_reference_value_[SPARK_MAX_PID_SLOTS];
		bool                pidf_reference_value_changed_[SPARK_MAX_PID_SLOTS];

		ControlType         pidf_reference_ctrl_;
		bool                pidf_reference_ctrl_changed_;
		int                 pidf_reference_slot_;
		bool                pidf_reference_slot_changed_;
		double              pidf_arb_feed_forward_[SPARK_MAX_PID_SLOTS];
		bool                pidf_config_changed_[SPARK_MAX_PID_SLOTS];

		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_;
		bool                forward_limit_switch_enabled_;
		bool                forward_limit_switch_changed_;
		LimitSwitchPolarity reverse_limit_switch_polarity_;
		bool                reverse_limit_switch_enabled_;
		bool                reverse_limit_switch_changed_;

		// Something for current limit mode?
		unsigned int        current_limit_;
		unsigned int        current_limit_stall_;
		unsigned int        current_limit_free_;
		unsigned int        current_limit_rpm_;
		bool                current_limit_changed_;
		double              secondary_current_limit_;
		int                 secondary_current_limit_cycles_;
		bool                secondary_current_limit_changed_;

		IdleMode            idle_mode_;
		bool                idle_mode_changed_;
		double              ramp_rate_;
		bool                ramp_rate_changed_;

		ExternalFollower    follower_type_;
		int                 follower_id_;
		bool                follower_invert_;
		bool                follower_changed_;
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a SparkMax
class SparkMaxCommandHandle: public SparkMaxStateHandle
{
	public:
		SparkMaxCommandHandle(void) :
			SparkMaxStateHandle(),
			cmd_(0)
		{
		}

		SparkMaxCommandHandle(const SparkMaxStateHandle &js, SparkMaxHWCommand *cmd) :
			SparkMaxStateHandle(js),
			cmd_(cmd)
		{
			if (!cmd_)
				throw HardwareInterfaceException("Cannot create SparkMax handle '" + js.getName() + "'. command pointer is null.");
		}

		// Operator which allows access to methods from
		// the SparkMaxHWCommand member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWState
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWState method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		//
		SparkMaxHWCommand *operator->()
		{
			assert(cmd_);
			return cmd_;
		}

		// Get a pointer to the HW state associated with
		// this SparkMax.  Since CommandHandle is derived
		// from StateHandle, there's a state embedded
		// in each instance of a CommandHandle. Use
		// this method to access it.
		//
		// handle->state()->getCANID();
		//
		const SparkMaxHWState *state(void) const
		{
			return SparkMaxStateHandle::operator->();
		}

	private:
		SparkMaxHWCommand *cmd_;
};

// Use ClaimResources here since we only want 1 controller
// to be able to access a given SparkMax at any particular time
class SparkMaxCommandInterface : public HardwareResourceManager<SparkMaxCommandHandle, ClaimResources> {};

}
