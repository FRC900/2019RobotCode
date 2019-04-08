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

#include "rev/CANSparkMaxLowLevel.h"

#include <cstring>
#include <thread>

#include <frc/CAN.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <hal/CANAPI.h>
// #include <hal/FRCUsageReporting.h>
#include <hal/HAL.h>
#include <wpi/SmallString.h>
#include <wpi/raw_ostream.h>

#include "rev/CANSparkMaxFrames.h"
//#include "rev/CANSparkMaxHeartbeat.h"
#include "rev/CANSparkMaxSetDriver.h"

static constexpr uint8_t kNumFirmwareRetries = 10;
static constexpr int kDefaultCANTimeoutMs = 20;
static constexpr int kDefaultStatus0PeriodMs = 10;
static constexpr int kDefaultStatus1PeriodMs = 20;
static constexpr int kDefaultStatus2PeriodMs = 50;
static constexpr uint32_t kMinFirmwareVersion = 0x101001C;

static_assert(sizeof(float) == sizeof(uint32_t), "float isn't 32 bits wide");

using namespace rev;

// kDutyCycle = 0, kVelocity = 1, kVoltage = 2, kPosition = 3, kSmartMotion = 4,
// kCurrent = 5, kSmartVelocity = 6
static constexpr int kNumControlTypes = 7;
static constexpr int kControlTypeFrames[kNumControlTypes] = {
    CMD_API_DC_SET,  CMD_API_SPD_SET,         CMD_API_VOLT_SET,
    CMD_API_POS_SET, CMD_API_SMARTMOTION_SET, CMD_API_CURRENT_SET,
    CMD_API_SMART_VEL_SET};

class CANSparkMaxLowLevel::Daemon : public wpi::SafeThread {
private:
    void Main() override {
        //REV_CANSparkMaxHeartbeatInit();
        //REV_CANSparkMaxRunHeartbeat();

        REV_CANSparkMaxSetDriverInit();
        REV_CANSparkMaxRunSetDriver();
    }
};

CANSparkMaxLowLevel::CANSparkMaxLowLevel(int deviceID, MotorType type)
    : m_can(deviceID, HAL_CAN_Man_kREV, HAL_CAN_Dev_kMotorController),
      m_owner(&GetThreadOwner()),
      m_deviceID(deviceID) {
    m_firmwareVersion = GetFirmwareVersion(m_isFirmwareDebug);

    m_canTimeoutMs = kDefaultCANTimeoutMs;

    m_status0PeriodMs = kDefaultStatus0PeriodMs;
    m_status1PeriodMs = kDefaultStatus1PeriodMs;
    m_status2PeriodMs = kDefaultStatus2PeriodMs;

    m_activeSetpointApi = CMD_API_DC_SET;

    m_inverted = false;

    if (m_firmwareVersion == 0) {
        wpi::SmallString<255> buf;
        wpi::raw_svector_ostream firmwareString{buf};
        firmwareString << "Unable to retrieve SPARK MAX firmware "
                       << "version for CAN ID: "
                       << m_deviceID
                       << ". Please verify the deviceID field matches the "
                       << "configured CAN ID of the controller, and that "
                       << "the controller is connected to the CAN Bus.";
        frc::DriverStation::ReportError(firmwareString.str());
    } else if (m_firmwareVersion < kMinFirmwareVersion) {
        wpi::SmallString<255> buf;
        wpi::raw_svector_ostream firmwareString{buf};
        firmwareString << "The firmware on SPARK MAX with CAN ID: "
                       << m_deviceID
                       << " is too old and needs to be updated. Refer "
                       << "to www.RevRobotics.com/sparkmax for details.";
        frc::DriverStation::ReportError(firmwareString.str());
    }

    SetMotorType(type);

    REV_CANSparkMaxRegisterDevice(deviceID);

    //Set default control frame period
    REV_CANSparkMaxSetDriverSetCtrlFramePeriodMs(deviceID, 10);

    HAL_Report(HALUsageReporting::kResourceType_RevSparkMaxCAN, deviceID);
}

uint32_t CANSparkMaxLowLevel::GetFirmwareVersion() {
    bool isDebugBuild;
    return GetFirmwareVersion(isDebugBuild);
}

uint32_t CANSparkMaxLowLevel::GetFirmwareVersion(bool& isDebugBuild) {
    frc_dataframe_t frame;
    uint32_t firmwareVersionFlattened;
    frc::CANData data;

    m_can.WritePacket(frame.data, 0, CMD_API_FIRMWARE);

    int retries = 0;
    while (retries < kNumFirmwareRetries) {
        if (m_can.ReadPacketTimeout(CMD_API_FIRMWARE, 10, &data)) {
            // Firmware read successful
            break;
        }
        retries++;
    }

    if (retries == kNumFirmwareRetries) {
        // Firmware version not recieved
        return 0;
    }

    std::memcpy(frame.data, data.data, data.length);

    uint16_t firmwareBuild = (frame.firmwareIn.firmwareBuild >> 8) |
                             (frame.firmwareIn.firmwareBuild << 8);

    isDebugBuild = frame.firmwareIn.debugBuild ? true : false;
    firmwareVersionFlattened =
        static_cast<uint32_t>(frame.firmwareIn.firmwareMajor) << 24 |
        static_cast<uint32_t>(frame.firmwareIn.firmwareMinor) << 16 |
        firmwareBuild;

    wpi::SmallString<128> buf;
    wpi::raw_svector_ostream firmwareString{buf};
    firmwareString << "v"
                   << static_cast<uint32_t>(frame.firmwareIn.firmwareMajor)
                   << "."
                   << static_cast<uint32_t>(frame.firmwareIn.firmwareMinor)
                   << "." << firmwareBuild;
    if (isDebugBuild) {
        firmwareString << " Debug Build";
    }

    m_firmwareString = firmwareString.str();

    return firmwareVersionFlattened;
}

std::string CANSparkMaxLowLevel::GetFirmwareString() {
    return m_firmwareString;
}

std::vector<uint8_t> CANSparkMaxLowLevel::GetSerialNumber() { return {}; }

int CANSparkMaxLowLevel::GetDeviceId() const { return m_deviceID; }

CANError CANSparkMaxLowLevel::SetMotorType(MotorType type) {
    auto status =
        SetParameter(ConfigParameter::kMotorType, static_cast<uint32_t>(type));
    if (status == ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANSparkMaxLowLevel::MotorType CANSparkMaxLowLevel::GetMotorType() {
    uint32_t value;
    auto status = GetParameter(ConfigParameter::kMotorType, value);
    if (status == ParameterStatus::kOK) {
        return static_cast<MotorType>(value);
    } else {
        return MotorType::kBrushless;
    }
}

CANError CANSparkMaxLowLevel::SetPeriodicFramePeriod(PeriodicFrame frameID,
                                                     int periodMs) {
    int apiID;

    switch (frameID) {
        case PeriodicFrame::kStatus0:
            apiID = CMD_API_STAT0;
            m_status0PeriodMs = periodMs;
            break;
        case PeriodicFrame::kStatus1:
            apiID = CMD_API_STAT1;
            m_status1PeriodMs = periodMs;
            break;
        case PeriodicFrame::kStatus2:
            apiID = CMD_API_STAT2;
            m_status2PeriodMs = periodMs;
            break;
        default:
            return CANError::kError;
    }

    frc_dataframe_t frame;
    std::memset(frame.data, 0, 8);
    frame.statusConfigOut.updateRate = static_cast<uint16_t>(periodMs);
    m_can.WritePacket(frame.data, 2, apiID);

    return CANError::kOK;
}

void CANSparkMaxLowLevel::SetControlFramePeriodMs(int periodMs) {
    REV_CANSparkMaxSetDriverSetCtrlFramePeriodMs(m_deviceID, periodMs);
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::SetParameter(
    ConfigParameter parameterID, double value) {
    uint32_t tmp;
    float value_float32 = value;

    std::memcpy(&tmp, &value_float32, sizeof(tmp));
    return SetParameterCore(parameterID, ParameterType::kFloat32, tmp);
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::SetParameter(
    ConfigParameter parameterID, uint32_t value) {
    return SetParameterCore(parameterID, ParameterType::kUint32, value);
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::SetParameter(
    ConfigParameter parameterID, int32_t value) {
    uint32_t tmp;

    std::memcpy(&tmp, &value, sizeof(tmp));
    return SetParameterCore(parameterID, ParameterType::kInt32, tmp);
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::SetParameter(
    ConfigParameter parameterID, bool value) {
    return SetParameterCore(parameterID, ParameterType::kBool, value ? 1 : 0);
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::GetParameter(
    ConfigParameter parameterID, double& value) {
    uint32_t tmp;
    float value_float32;
    ParameterStatus status =
        GetParameterCore(parameterID, ParameterType::kFloat32, tmp);

    std::memcpy(&value_float32, &tmp, sizeof(value_float32));

    value = static_cast<double>(value_float32);
    return status;
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::GetParameter(
    ConfigParameter parameterID, uint32_t& value) {
    return GetParameterCore(parameterID, ParameterType::kFloat32, value);
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::GetParameter(
    ConfigParameter parameterID, int32_t& value) {
    uint32_t tmp;
    int32_t returnValue = 0;
    ParameterStatus status =
        GetParameterCore(parameterID, ParameterType::kFloat32, tmp);

    std::memcpy(&returnValue, &tmp, sizeof(returnValue));
    return status;
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::GetParameter(
    ConfigParameter parameterID, bool& value) {
    uint32_t tmp;
    ParameterStatus status =
        GetParameterCore(parameterID, ParameterType::kFloat32, tmp);

    value = (tmp == 0) ? false : true;
    return status;
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::SetParameterCore(
    ConfigParameter parameterID, ParameterType type, uint32_t value) {
    frc_dataframe_t frame;
    frame.setParamOut.parameter = value;
    frame.setParamOut.parameterType = static_cast<uint8_t>(type);

    m_can.WritePacket(reinterpret_cast<uint8_t*>(&frame), 5,
                      CMD_API_PARAM_ACCESS | static_cast<int>(parameterID));

    return ParameterStatus::kOK;
}

CANSparkMaxLowLevel::ParameterStatus CANSparkMaxLowLevel::GetParameterCore(
    ConfigParameter parameterID, ParameterType expectedType, uint32_t& value) {
    // if (GetParameterType(parameterID) != expectedType) {
    //    return ParameterStatus::kMismatchType;
    //}

    // Request parameter
    frc_dataframe_t frame;
    int apiID = CMD_API_PARAM_ACCESS | static_cast<int>(parameterID);
    m_can.WritePacket(frame.data, 0, apiID);

    // Wait for reply containing parameter
    frc::CANData data;
    if (!m_can.ReadPacketTimeout(apiID, m_canTimeoutMs, &data)) {
        return ParameterStatus::kInvalid;
    }

    // Extract parameter from payload
    auto payload = reinterpret_cast<frc_dataframe_getParam_in_t*>(&data.data);
    value = payload->parameter0;

    return ParameterStatus::kOK;
}

CANSparkMaxLowLevel::ParameterType CANSparkMaxLowLevel::GetParameterType(
    ConfigParameter parameterID) {
    ConfigParameterType configType =
        static_cast<ConfigParameterType>(parameterID);
    return static_cast<ParameterType>(configType);
}

CANError CANSparkMaxLowLevel::SetEncPosition(double value) {
    uint32_t tmp;
    float value_float32 = value;

    if (m_inverted) {
        value_float32 = value_float32 * -1;
    }

    std::memcpy(&tmp, &value_float32, sizeof(tmp));
    frc_dataframe_t frame;
    frame.setParamOut.parameter = tmp;
    frame.setParamOut.parameterType =
        static_cast<uint8_t>(ParameterType::kFloat32);

    m_can.WritePacket(reinterpret_cast<uint8_t*>(&frame), 5, CMD_API_MECH_POS);

    return CANError::kOK;
}

CANError CANSparkMaxLowLevel::SetIAccum(double value) {
    uint32_t tmp;
    float value_float32 = value;

    if (m_inverted) {
        value_float32 = value_float32 * -1;
    }

    std::memcpy(&tmp, &value_float32, sizeof(tmp));
    frc_dataframe_t frame;
    frame.setParamOut.parameter = tmp;
    frame.setParamOut.parameterType =
        static_cast<uint8_t>(ParameterType::kFloat32);

    m_can.WritePacket(reinterpret_cast<uint8_t*>(&frame), 5, CMD_API_I_ACCUM);

    return CANError::kOK;
}

CANSparkMaxLowLevel::PeriodicStatus0 CANSparkMaxLowLevel::GetPeriodicStatus0() {
    PeriodicStatus0 status0;
    frc::CANData frame;
    m_can.ReadPeriodicPacket(CMD_API_STAT0,
                             m_status0PeriodMs * 2 + m_canTimeoutMs,
                             m_status0PeriodMs, &frame);

    auto status0Frame =
        reinterpret_cast<frc_dataframe_status0_in_t*>(&frame.data);

    status0.appliedOutput =
        (static_cast<double>(status0Frame->appliedOutput) / 32767.0) *
        (m_inverted ? -1 : 1);
    status0.faults = status0Frame->faults;
    status0.idleMode = status0Frame->idleMode;
    status0.isFollower = status0Frame->rsvdBit;
    status0.motorType = static_cast<MotorType>(status0Frame->mtrType);
    status0.stickyFaults = status0Frame->stickyFaults;

    return status0;
}

static inline float unpackFloat32ToInt32(int32_t val) {
    float f;
    std::memcpy(&f, &val, sizeof(f));
    return f;
}

CANSparkMaxLowLevel::PeriodicStatus1 CANSparkMaxLowLevel::GetPeriodicStatus1() {
    PeriodicStatus1 status1;
    frc::CANData frame;
    m_can.ReadPeriodicPacket(CMD_API_STAT1,
                             m_status1PeriodMs * 2 + m_canTimeoutMs,
                             m_status1PeriodMs, &frame);

    auto status1Frame =
        reinterpret_cast<frc_dataframe_status1_in_t*>(&frame.data);

    status1.outputCurrent = static_cast<double>(status1Frame->mtrCurrent) / 32;
    status1.busVoltage = static_cast<double>(status1Frame->mtrVoltage) / 128;
    status1.motorTemperature = status1Frame->mtrTemp;
    status1.sensorVelocity =
        static_cast<double>(unpackFloat32ToInt32(status1Frame->sensorVel)) *
        (m_inverted ? -1 : 1);

    return status1;
}

CANSparkMaxLowLevel::PeriodicStatus2 CANSparkMaxLowLevel::GetPeriodicStatus2() {
    PeriodicStatus2 status2;
    frc::CANData frame;
    m_can.ReadPeriodicPacket(CMD_API_STAT2,
                             m_status2PeriodMs * 2 + m_canTimeoutMs,
                             m_status2PeriodMs, &frame);

    auto status2Frame =
        reinterpret_cast<frc_dataframe_status2_in_t*>(&frame.data);

    status2.sensorPosition =
        static_cast<double>(unpackFloat32ToInt32(status2Frame->sensorPos)) *
        (m_inverted ? -1 : 1);

    status2.iAccum =
        static_cast<double>(unpackFloat32ToInt32(status2Frame->iAccum)) *
        (m_inverted ? -1 : 1);

    return status2;
}

CANError CANSparkMaxLowLevel::SetFollow(FollowConfig follower) {
    static const int kFollowFrameLength = 8;
    frc::CANData canReturnFrame;
    frc_dataframe_t frame;
    frame.followerOut.followerCfg = follower.configRaw;
    frame.followerOut.followerID = follower.leaderArbId;
    m_can.WritePacket(frame.data, kFollowFrameLength, CMD_API_SET_FOLLOWER);

    // Confim good setup checking response
    if (m_can.ReadPacketTimeout(CMD_API_SET_FOLLOWER, m_canTimeoutMs,
                                &canReturnFrame) == false) {
        return CANError::kTimeout;
    }

    if (canReturnFrame.length < kFollowFrameLength) {
        return CANError::kError;
    }

    for (int i = 0; i < kFollowFrameLength; i++) {
        if (frame.data[i] != canReturnFrame.data[i]) {
            return CANError::kError;
        }
    }

    return CANError::kOK;
}

CANError CANSparkMaxLowLevel::SetpointCommand(double value, ControlType ctrl,
                                              int pidSlot,
                                              double arbFeedforward) {
    int apiId = 0;
    int cntrlIdx = static_cast<int>(ctrl);

    if (cntrlIdx >= 0 && cntrlIdx < kNumControlTypes) {
        apiId = kControlTypeFrames[cntrlIdx];
    } else {
        return CANError::kError;
    }

    double shiftedArbFF = (arbFeedforward * 1024);
    int16_t packedFF;

    if (shiftedArbFF > 32767) {
        packedFF = 32767;
    } else if (shiftedArbFF < -32767) {
        packedFF = -32767;
    } else {
        packedFF = static_cast<int16_t>(shiftedArbFF);
    }

    m_activeSetpointApi = apiId;
    REV_CANSparkMaxSetDriverSet4(m_deviceID, GetSafeFloat((float) (m_inverted ? -value : value)), apiId, (uint8_t) pidSlot, (int16_t)(m_inverted ? -packedFF : packedFF));

    return CANError::kOK;
}

float CANSparkMaxLowLevel::GetSafeFloat(float f) {
    if (std::isinf(f) || std::isnan(f))
        return 0;
    return f;
}

CANError CANSparkMaxLowLevel::RestoreFactoryDefaults(bool persist) {
    frc_dataframe_t frame;
    frame.setParamOut.parameter = persist ? 1 : 0;
    frame.setParamOut.parameterType =
        static_cast<uint8_t>(ParameterType::kBool);

    m_can.WritePacket(reinterpret_cast<uint8_t*>(&frame), 5,
                      CMD_API_FACTORY_DEFAULT);

    return CANError::kOK;
}

wpi::SafeThreadOwner<CANSparkMaxLowLevel::Daemon>&
CANSparkMaxLowLevel::GetThreadOwner() {
    static wpi::SafeThreadOwner<Daemon> inst = [] {
        wpi::SafeThreadOwner<CANSparkMaxLowLevel::Daemon> inst;
        inst.Start();
        return inst;
    }();
    return inst;
}
