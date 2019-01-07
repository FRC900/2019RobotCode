#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

enum SensorType { kNoSensor = 0, kHallSensor = 1, kEncoder = 2, kSensorless = 3 }

enum IdleMode { kCoast = 0, kBrake = 1 }

// TODO : Fault to string?
enum FaultID {
	kBrownout = 0, kOvercurrent = 1, kOvervoltage = 2, kMotorFault = 3,
	kSensorFault = 4, kStall = 5, kEEPROMCRC = 6, kCANTX = 7,
	kCANRX = 8, kHasReset = 9, kDRVFault = 10, kOtherFault = 11,
	kSoftLimitFwd = 12, kSoftLimitRev = 13, kHardLimitFwd = 14, kHardLimitRev = 15
}

enum LimitSwitchPolarity { kNormallyOpen = 0, kNormallyClosed = 1 }

const size_t NUM_PID_SLOTS = 4;

class SparkMaxHWState
{
	public:
		SparkMaxHWState(int device_id, MotorType motor_type)
			: device_id_(device_id)
			, motor_type_(motor_type)
	{}

	private:
		int                 device_id_;
		MotorType           motor_type_;
		double              set_point_;
		bool                inverted_;

		// Encoder
		double              position_;
		double              velocity_;

		// PID Controller
		double              p_gain_[NUM_PID_SLOTS];
		double              i_gain_[NUM_PID_SLOTS];
		double              d_gain_[NUM_PID_SLOTS];
		double              f_gain_[NUM_PID_SLOTS];
		double              i_zone_[NUM_PID_SLOTS];
		double              d_filter_[NUM_PID_SLOTS];
		double              pidf_output_min_[NUM_PID_SLOTS];
		double              pidf_output_max_[NUM_PID_SLOTS];
		double              pidf_reference_value_[NUM_PID_SLOTS];
		ControlType         pidf_reference_ctrl_[NUM_PID_SLOTS];
		double              pidf_arb_feed_forward_[NUM_PID_SLOTS];


		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_;
		double              forward_limit_switch_enabled_;
		double              forward_limit_switch_;
		LimitSwitchPolarity reverse_limit_switch_polarity_;
		double              reverse_limit_switch_enabled_;
		double              reverse_limit_switch_;

		// Something for current limit mode?
		unsigned int        current_limit_;
		unsigned int        current_limit_stall_;
		unsigned int        current_limit_free_;
		unsigned int        current_limit_RPM_;
		double              secondary_current_limit_;
		int                 secondary_current_limit_cycles_;

		IdleMode            idle_mode_;
		double              ramp_rate_;

		ExternalFollower    follower_type_;
		int                 follower_id_;

		uint16_t            faults_;
		uint16_t            sticky_faults_;

		double              bus_voltage_;
		double              applied_output_;
		double              output_current_;
		double              motor_temperature_;
};

} // namespace




