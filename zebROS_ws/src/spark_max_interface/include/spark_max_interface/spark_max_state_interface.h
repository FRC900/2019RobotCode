#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

enum SensorType { kNoSensor, kHallSensor, kEncoder, kSensorless };

enum IdleMode { kCoast, kBrake };

// TODO : Fault to string?
enum FaultID {
	kBrownout, kOvercurrent, kOvervoltage, kMotorFault,
	kSensorFault, kStall, kEEPROMCRC, kCANTX,
	kCANRX, kHasReset, kDRVFault, kOtherFault,
	kSoftLimitFwd, kSoftLimitRev, kHardLimitFwd, kHardLimitRev
};

enum LimitSwitchPolarity { kNormallyOpen, kNormallyClosed };
enum MotorType { kBrushed, kBrushless };
enum ControlType { kDutyCycle, kVelocity, kVoltage, kPosition };
enum ExternalFollower { kFollowerDisabled, kFollowerSparkMax, kFollowerPhoenix };

const size_t SPARK_MAX_PID_SLOTS = 4;

class SparkMaxHWState
{
	public:
		SparkMaxHWState(int device_id, MotorType motor_type)
			: device_id_(device_id)
			, motor_type_(motor_type)
			, set_point_(0)
			, inverted_(false)
			, position_(0)
			, velocity_(0)
			, p_gain_{0}
			, i_gain_{0}
			, d_gain_{0}
			, f_gain_{0}
			, i_zone_{0}
			, d_filter_{0}
			, pidf_output_min_{-1}
			, pidf_output_max_{1}
			, pidf_reference_value_{0}
			, pidf_reference_ctrl_(kDutyCycle)
			, pidf_reference_slot_(0)
			, pidf_arb_feed_forward_{0}
			, forward_limit_switch_polarity_(kNormallyOpen)
			, forward_limit_switch_enabled_(false)
			, forward_limit_switch_(false)
			, reverse_limit_switch_polarity_(kNormallyOpen)
			, reverse_limit_switch_enabled_(false)
			, reverse_limit_switch_(false)
			, current_limit_(0) // TODO : better defaults
			, current_limit_stall_(0)
			, current_limit_free_(0)
			, current_limit_rpm_(0)
			, secondary_current_limit_(0)
			, secondary_current_limit_cycles_(0)
			, idle_mode_(kCoast)
			, ramp_rate_(0)
			, follower_type_(kFollowerDisabled)
			, follower_id_(-1)
			, follower_invert_(false)
			, faults_(0)
			, sticky_faults_(0)
			, bus_voltage_(0)
			, applied_output_(0)
			, output_current_(0)
			, motor_temperature_(0)
		{
		}

	int getDeviceId(void) const
	{
		return device_id_;
	}
	MotorType getMotorType(void) const
	{
		return motor_type_;
	}
	void setSetPoint(double set_point)
	{
		set_point_ = set_point;
	}
	double getSetPoint(void) const
	{
		return set_point_;
	}

	void setInverted(bool inverted)
	{
		inverted_ = inverted;
	}
	bool getInverted(void) const
	{
		return inverted_;
	}

	void setPosition(double position)
	{
		position_ = position;
	}
	double getPosition(void) const
	{
		return position_;
	}

	void setVelocity(double velocity)
	{
		velocity_ = velocity;
	}
	double getVelocity(void) const
	{
		return velocity_;
	}

	void setPGain(size_t slot, double p_gain)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setPGain() : invalid slot " << slot);
			return;
		}
		p_gain_[slot] = p_gain;
	}
	double getPGain(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getPGain() : invalid slot " << slot);
			return -1;
		}
		return p_gain_[slot];
	}

	void setIGain(size_t slot, double i_gain)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setIGain() : invalid slot " << slot);
			return;
		}
		i_gain_[slot] = i_gain;
	}
	double getIGain(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getIGain() : invalid slot " << slot);
			return -1;
		}
		return i_gain_[slot];
	}

	void setDGain(size_t slot, double d_gain)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setDGain() : invalid slot " << slot);
			return;
		}
		d_gain_[slot] = d_gain;
	}
	double getDGain(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getDGain() : invalid slot " << slot);
			return -1;
		}
		return d_gain_[slot];
	}

	void setFGain(size_t slot, double f_gain)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setFGain() : invalid slot " << slot);
			return;
		}
		f_gain_[slot] = f_gain;
	}
	double getFGain(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getFGain() : invalid slot " << slot);
			return -1;
		}
		return f_gain_[slot];
	}

	void setIZone(size_t slot, double i_zone)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setIZone() : invalid slot " << slot);
			return;
		}
		i_zone_[slot] = i_zone;
	}
	double getIZone(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getIZone() : invalid slot " << slot);
			return -1;
		}
		return i_zone_[slot];
	}

	void setDFilter(size_t slot, double d_filter)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setDFilter() : invalid slot " << slot);
			return;
		}
		d_filter_[slot] = d_filter;
	}
	double getDFilter(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getDFilter() : invalid slot " << slot);
			return -1;
		}
		return d_filter_[slot];
	}

	void setPIDFOutputMin(size_t slot, double pidf_output_min)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setPIDFOutputMin() : invalid slot " << slot);
			return;
		}
		pidf_output_min_[slot] = pidf_output_min;
	}
	double getPIDFOutputMin(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getPIDFOutputMin() : invalid slot " << slot);
			return std::numeric_limits<double>::max();
		}
		return pidf_output_min_[slot];
	}

	void setPIDFOutputMax(size_t slot, double pidf_output_max)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setPIDFOutputMax() : invalid slot " << slot);
			return;
		}
		pidf_output_max_[slot] = pidf_output_max;
	}
	double getPIDFOutputMax(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getPIDFOutputMax() : invalid slot " << slot);
			return -std::numeric_limits<double>::max();
		}
		return pidf_output_max_[slot];
	}

	void setPIDFReferenceOutput(size_t slot, double pidf_reference_value)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceOutput() : invalid slot " << slot);
			return;
		}
		pidf_reference_value_[slot] = pidf_reference_value;
	}
	double getPIDFReferenceOutput(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceOutput() : invalid slot " << slot);
			return 0;
		}
		return pidf_reference_value_[slot];
	}

	void setPIDFReferenceCtrl(ControlType pidf_reference_ctrl)
	{
		pidf_reference_ctrl_ = pidf_reference_ctrl;
	}
	ControlType getPIDFReferenceCtrl(void) const
	{
		return pidf_reference_ctrl_;
	}
	void setPIDFReferenceSlot(size_t slot)
	{
		if ((slot < 0) || (slot >= SPARK_MAX_PID_SLOTS))
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceSlot() : invalid slot " << slot);
			return;
		}
		pidf_reference_slot_ = slot;
	}
	int getPIDFReferenceSlot(void) const
	{
		return pidf_reference_slot_;
	}

	void setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward)
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::setPIDFArbFeedForward() : invalid slot " << slot);
			return;
		}
		pidf_arb_feed_forward_[slot] = pidf_arb_feed_forward;
	}
	double getPIDFArbFeedForward(size_t slot) const
	{
		if (slot >= SPARK_MAX_PID_SLOTS)
		{
			ROS_ERROR_STREAM("SparkMaxHWState::getPIDFArbFeedForward() : invalid slot " << slot);
			return -1;
		}
		return pidf_arb_feed_forward_[slot];
	}

	void setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity)
	{
		forward_limit_switch_polarity_ = forward_limit_switch_polarity;
	}
	LimitSwitchPolarity getForwardLimitSwitchPolarity(void) const
	{
		return forward_limit_switch_polarity_;
	}

	void setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled)
	{
		forward_limit_switch_enabled_ = forward_limit_switch_enabled;
	}
	bool getForwardLimitSwitchEnabled(void) const
	{
		return forward_limit_switch_enabled_;
	}

	void setForwardLimitSwitch(bool forward_limit_switch)
	{
		forward_limit_switch_ = forward_limit_switch;
	}
	bool getForwardLimitSwitch(void) const
	{
		return forward_limit_switch_;
	}

	void setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity)
	{
		reverse_limit_switch_polarity_ = reverse_limit_switch_polarity;
	}
	LimitSwitchPolarity getReverseLimitSwitchPolarity(void) const
	{
		return reverse_limit_switch_polarity_;
	}

	void setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled)
	{
		reverse_limit_switch_enabled_ = reverse_limit_switch_enabled;
	}
	bool getReverseLimitSwitchEnabled(void) const
	{
		return reverse_limit_switch_enabled_;
	}

	void setReverseLimitSwitch(bool reverse_limit_switch)
	{
		reverse_limit_switch_ = reverse_limit_switch;
	}
	bool getReverseLimitSwitch(void) const
	{
		return reverse_limit_switch_;
	}

	void setCurrentLimit(unsigned int current_limit)
	{
		current_limit_ = current_limit;
	}
	unsigned int getCurrentLimit(void) const
	{
		return current_limit_;
	}

	void setCurrentLimitStall(unsigned int current_limit_stall)
	{
		current_limit_stall_ = current_limit_stall;
	}
	unsigned int getCurrentLimitStall(void) const
	{
		return current_limit_stall_;
	}

	void setCurrentLimitRPM(unsigned int current_limit_rpm)
	{
		current_limit_rpm_ = current_limit_rpm;
	}
	unsigned int getCurrentLimitRPM(void) const
	{
		return current_limit_rpm_;
	}

	void setSecondaryCurrentLimit(unsigned int secondary_current_limit)
	{
		secondary_current_limit_ = secondary_current_limit;
	}
	unsigned int getSecondaryCurrentLimit(void) const
	{
		return secondary_current_limit_;
	}

	void setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles)
	{
		secondary_current_limit_cycles_ = secondary_current_limit_cycles;
	}
	unsigned int getSecondaryCurrentLimitCycles(void) const
	{
		return secondary_current_limit_cycles_;
	}

	void setIdleMode(IdleMode idle_mode)
	{
		idle_mode_ = idle_mode;
	}
	bool getIdleMode(void) const
	{
		return idle_mode_;
	}

	void setRampRate(double ramp_rate)
	{
		ramp_rate_ = ramp_rate;
	}
	double getRampRate(void) const
	{
		return ramp_rate_;
	}

	void setFollowerType(ExternalFollower follower_type)
	{
		follower_type_ = follower_type;
	}
	ExternalFollower getFollowerType(void) const
	{
		return follower_type_;
	}

	void setFollowerID(int follower_id)
	{
		follower_id_ = follower_id;
	}
	int getFollowerID(void) const
	{
		return follower_id_;
	}

	void setFollowerInvert(bool follower_invert)
	{
		follower_invert_ = follower_invert;
	}
	bool getFollowerInvert(void) const
	{
		return follower_invert_;
	}

	void setFaults(uint16_t faults)
	{
		faults_ = faults;
	}
	uint16_t getFaults(void) const
	{
		return faults_;
	}

	void setStickyFaults(uint16_t sticky_faults)
	{
		sticky_faults_ = sticky_faults;
	}
	uint16_t getStickyFaults(void) const
	{
		return sticky_faults_;
	}

	void setBusVoltage(double bus_voltage)
	{
		bus_voltage_ = bus_voltage;
	}
	double getBusVoltage(void) const
	{
		return bus_voltage_;
	}

	void setAppliedOutput(double applied_output)
	{
		applied_output_ = applied_output;
	}
	double getAppliedOutput(void) const
	{
		return applied_output_;
	}

	void setOutputCurrent(double output_current)
	{
		output_current_ = output_current;
	}
	double getOutputCurrent(void) const
	{
		return output_current_;
	}

	void setMotorTemperature(double motor_temperature)
	{
		motor_temperature_ = motor_temperature;
	}
	double getMotorTemperature(void) const
	{
		return motor_temperature_;
	}
	private:
		int                 device_id_;
		MotorType           motor_type_;
		double              set_point_;
		bool                inverted_;

		// Encoder
		double              position_;
		double              velocity_;

		// PID Controller
		double              p_gain_[SPARK_MAX_PID_SLOTS];
		double              i_gain_[SPARK_MAX_PID_SLOTS];
		double              d_gain_[SPARK_MAX_PID_SLOTS];
		double              f_gain_[SPARK_MAX_PID_SLOTS];
		double              i_zone_[SPARK_MAX_PID_SLOTS];
		double              d_filter_[SPARK_MAX_PID_SLOTS];
		double              pidf_output_min_[SPARK_MAX_PID_SLOTS];
		double              pidf_output_max_[SPARK_MAX_PID_SLOTS];
		double              pidf_reference_value_[SPARK_MAX_PID_SLOTS];
		ControlType         pidf_reference_ctrl_;
		int                 pidf_reference_slot_;
		double              pidf_arb_feed_forward_[SPARK_MAX_PID_SLOTS];

		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_;
		bool                forward_limit_switch_enabled_;
		bool                forward_limit_switch_;
		LimitSwitchPolarity reverse_limit_switch_polarity_;
		bool                reverse_limit_switch_enabled_;
		bool                reverse_limit_switch_;

		// Something for current limit mode?
		unsigned int        current_limit_;
		unsigned int        current_limit_stall_;
		unsigned int        current_limit_free_;
		unsigned int        current_limit_rpm_;
		double              secondary_current_limit_;
		int                 secondary_current_limit_cycles_;

		IdleMode            idle_mode_;
		double              ramp_rate_;

		ExternalFollower    follower_type_;
		int                 follower_id_;
		bool                follower_invert_;

		uint16_t            faults_;
		uint16_t            sticky_faults_;

		double              bus_voltage_;
		double              applied_output_;
		double              output_current_;
		double              motor_temperature_;
};

typedef StateHandle<const SparkMaxHWState> SparkMaxStateHandle;
typedef StateHandle<SparkMaxHWState> SparkMaxWritableStateHandle;

class SparkMaxStateInterface : public HardwareResourceManager<SparkMaxStateHandle> {};

} // namespace

