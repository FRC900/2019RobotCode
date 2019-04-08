/*
 * Copyright (c) 2018-2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rev/CANSparkMax.h"

#include "rev/CANSparkMaxFrames.h"

using namespace rev;

constexpr CANSparkMax::ExternalFollower CANSparkMax::kFollowerDisabled;
constexpr CANSparkMax::ExternalFollower CANSparkMax::kFollowerSparkMax;
constexpr CANSparkMax::ExternalFollower CANSparkMax::kFollowerPhoenix;

CANSparkMax::CANSparkMax(int deviceID, MotorType type)
    : CANSparkMaxLowLevel(deviceID, type) {
    // Initialize conversion factors to 1
    GetEncoder().SetPositionConversionFactor(1);
    GetEncoder().SetVelocityConversionFactor(1);
    ClearFaults();
}

void CANSparkMax::Set(double speed) {
    // Only for 'get' api
    m_setpoint = speed;
    SetpointCommand(speed, ControlType::kDutyCycle);
}

double CANSparkMax::Get() const { return m_setpoint; }

void CANSparkMax::SetInverted(bool isInverted) { m_inverted = isInverted; }

bool CANSparkMax::GetInverted() const { return m_inverted; }

void CANSparkMax::Disable() { Set(0); }

void CANSparkMax::StopMotor() { Set(0); }

void CANSparkMax::PIDWrite(double output) { Set(output); }

CANEncoder CANSparkMax::GetEncoder() { return CANEncoder{*this}; }

CANPIDController CANSparkMax::GetPIDController() {
    return CANPIDController{*this};
}

CANDigitalInput CANSparkMax::GetForwardLimitSwitch(
    CANDigitalInput::LimitSwitchPolarity polarity) {
    return CANDigitalInput{*this, CANDigitalInput::LimitSwitch::kForward,
                           polarity};
}

CANDigitalInput CANSparkMax::GetReverseLimitSwitch(
    CANDigitalInput::LimitSwitchPolarity polarity) {
    return CANDigitalInput{*this, CANDigitalInput::LimitSwitch::kReverse,
                           polarity};
}

CANError CANSparkMax::SetSmartCurrentLimit(unsigned int limit) {
    return SetSmartCurrentLimit(limit, 0, 20000);
}

CANError CANSparkMax::SetSmartCurrentLimit(unsigned int stallLimit,
                                           unsigned int freeLimit,
                                           unsigned int limitRPM) {
    auto status =
        SetParameter(ConfigParameter::kSmartCurrentStallLimit, stallLimit);
    if (status != ParameterStatus::kOK) {
        return CANError::kError;
    }

    status = SetParameter(ConfigParameter::kSmartCurrentFreeLimit, freeLimit);
    if (status != ParameterStatus::kOK) {
        return CANError::kError;
    }

    status = SetParameter(ConfigParameter::kSmartCurrentConfig, limitRPM);
    if (status == ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANError CANSparkMax::SetSecondaryCurrentLimit(double limit, int chopCycles) {
    auto status = SetParameter(ConfigParameter::kCurrentChop, limit);
    if (status != ParameterStatus::kOK) {
        return CANError::kError;
    }

    status = SetParameter(ConfigParameter::kCurrentChopCycles,
                          static_cast<uint32_t>(chopCycles));
    if (status == ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANError CANSparkMax::SetIdleMode(IdleMode idleMode) {
    auto status = SetParameter(ConfigParameter::kIdleMode,
                               static_cast<uint32_t>(idleMode));
    if (status == ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANSparkMax::IdleMode CANSparkMax::GetIdleMode() {
    uint32_t value;
    auto status = GetParameter(ConfigParameter::kIdleMode, value);
    if (status == ParameterStatus::kOK) {
        return static_cast<IdleMode>(value);
    } else {
        return IdleMode::kBrake;
    }
}

CANError CANSparkMax::EnableVoltageCompensation(double nominalVoltage) {
    bool statusOkay = SetParameter(ConfigParameter::kCompensatedNominalVoltage,
                                   nominalVoltage) == ParameterStatus::kOK;
    statusOkay &= SetParameter(ConfigParameter::kVoltageCompMode, 2U) ==
                  ParameterStatus::kOK;
    if (statusOkay) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANError CANSparkMax::DisableVoltageCompensation() {
    bool statusOkay = SetParameter(ConfigParameter::kVoltageCompMode, 0) ==
                      ParameterStatus::kOK;
    statusOkay &= SetParameter(ConfigParameter::kCompensatedNominalVoltage,
                               0) == ParameterStatus::kOK;
    if (statusOkay) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

double CANSparkMax::GetVoltageCompensationNominalVoltage() {
    double value;
    auto status =
        GetParameter(ConfigParameter::kCompensatedNominalVoltage, value);
    if (status == ParameterStatus::kOK) {
        return value;
    } else {
        return 0.0;
    }
}

CANError CANSparkMax::SetOpenLoopRampRate(double rate) {
    if (rate != 0) {
        rate = 1.0 / rate;
    }
    auto status = SetParameter(ConfigParameter::kOpenLoopRampRate,
                               static_cast<float>(rate));
    if (status == ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANError CANSparkMax::SetClosedLoopRampRate(double rate) {
    if (rate != 0) {
        rate = 1.0 / rate;
    }
    auto status = SetParameter(ConfigParameter::kClosedLoopRampRate,
                               static_cast<float>(rate));
    if (status == ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

double CANSparkMax::GetOpenLoopRampRate() {
    double value;
    auto status = GetParameter(ConfigParameter::kOpenLoopRampRate, value);
    if (status == ParameterStatus::kOK) {
        // Convert to 'time from 0 to full throttle'
        if (value != 0) {
            return 1.0 / value;
        }
        return value;
    } else {
        return 0.0;
    }
}

double CANSparkMax::GetClosedLoopRampRate() {
    double value;
    auto status = GetParameter(ConfigParameter::kClosedLoopRampRate, value);
    if (status == ParameterStatus::kOK) {
        // Convert to 'time from 0 to full throttle'
        if (value != 0) {
            return 1.0 / value;
        }
        return value;
    } else {
        return 0.0;
    }
}

CANError CANSparkMax::Follow(const CANSparkMax& leader, bool invert) {
    return Follow(kFollowerSparkMax, leader.GetDeviceId(), invert);
}

CANError CANSparkMax::Follow(ExternalFollower leader, int deviceID,
                             bool invert) {
    FollowConfig maxFollower;
    maxFollower.leaderArbId = leader.arbId | deviceID;
    maxFollower.config.predefined = leader.configId;
    maxFollower.config.invert = invert;
    return SetFollow(maxFollower);
}

bool CANSparkMax::IsFollower() { return GetPeriodicStatus0().isFollower; }

uint16_t CANSparkMax::GetFaults() { return GetPeriodicStatus0().faults; }

uint16_t CANSparkMax::GetStickyFaults() {
    return GetPeriodicStatus0().stickyFaults;
}

bool CANSparkMax::GetFault(FaultID faultID) {
    uint16_t val = GetFaults() & (1 << static_cast<uint16_t>(faultID));
    return val != 0;
}

bool CANSparkMax::GetStickyFault(FaultID faultID) {
    uint16_t val = GetStickyFaults() & (1 << static_cast<uint16_t>(faultID));
    return val != 0;
}

double CANSparkMax::GetBusVoltage() { return GetPeriodicStatus1().busVoltage; }

double CANSparkMax::GetAppliedOutput() {
    return GetPeriodicStatus0().appliedOutput;
}

double CANSparkMax::GetOutputCurrent() {
    return GetPeriodicStatus1().outputCurrent;
}

double CANSparkMax::GetMotorTemperature() {
    return static_cast<double>(GetPeriodicStatus1().motorTemperature);
}

CANError CANSparkMax::ClearFaults() {
    frc::CANData frame;
    m_can.WritePacket(frame.data, 0, CMD_API_CLEAR_FAULTS);
    return CANError::kOK;
}

CANError CANSparkMax::BurnFlash() {
    frc::CANData frame;
    frame.data[0] = 0xA3;
    frame.data[1] = 0x3A;
    m_can.WritePacket(frame.data, 2, CMD_API_BURN_FLASH);
    return CANError::kOK;
}

CANError CANSparkMax::SetCANTimeout(int milliseconds) {
    m_canTimeoutMs = milliseconds;
    return CANError::kOK;
}
