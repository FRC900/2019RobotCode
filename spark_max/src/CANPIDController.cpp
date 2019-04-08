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

#include "rev/CANPIDController.h"

#include "rev/CANSparkMax.h"

using namespace rev;

CANPIDController::CANPIDController(CANSparkMax& device) : m_device(&device) {}

CANError CANPIDController::SetReference(double value, ControlType ctrl,
                                        int pidSlot, double arbFeedforward) {
    return m_device->SetpointCommand(value, ctrl, pidSlot, arbFeedforward);
}

static CANError SetPIDLimitParamsRaw(CANSparkMax* device,
                                     CANSparkMax::ConfigParameter kParamSlot0,
                                     int slotID, double value) {
    if (slotID < 0 || slotID > 3 ||
        kParamSlot0 < CANSparkMax::ConfigParameter::kIMaxAccum_0 ||
        kParamSlot0 > CANSparkMax::ConfigParameter::kSlot3Placeholder3_0) {
        return CANError::kError;
    }

    int offset = slotID * 4 + static_cast<int>(kParamSlot0);
    CANSparkMax::ConfigParameter param =
        static_cast<CANSparkMax::ConfigParameter>(offset);

    auto status = device->SetParameter(param, value);
    if (status == CANSparkMax::ParameterStatus::kOK) {
        return CANError::kOK;
    }
    return CANError::kError;
}

static double GetPIDLimitParamsRaw(CANSparkMax* device,
                                   CANSparkMax::ConfigParameter kParamSlot0,
                                   int slotID) {
    double value;

    if (slotID < 0 || slotID > 3 ||
        kParamSlot0 < CANSparkMax::ConfigParameter::kIMaxAccum_0 ||
        kParamSlot0 > CANSparkMax::ConfigParameter::kSlot3Placeholder3_0) {
        return 0;
    }

    int offset = slotID * 4 + static_cast<int>(kParamSlot0);
    CANSparkMax::ConfigParameter param =
        static_cast<CANSparkMax::ConfigParameter>(offset);

    auto status = device->GetParameter(param, value);
    if (status == CANSparkMax::ParameterStatus::kOK) {
        return value;
    }
    return 0;
}

static CANError SetMotionParamRaw(CANSparkMax* device,
                                  CANSparkMax::ConfigParameter kParamSlot0,
                                  int slotID, double value) {
    if (slotID < 0 || slotID > 3 ||
        kParamSlot0 < CANSparkMax::ConfigParameter::kSmartMotionMaxVelocity_0 ||
        kParamSlot0 >
            CANSparkMax::ConfigParameter::kSmartMotionAccelStrategy_0) {
        return CANError::kError;
    }

    int offset = slotID * 5 + static_cast<int>(kParamSlot0);
    CANSparkMax::ConfigParameter param =
        static_cast<CANSparkMax::ConfigParameter>(offset);

    auto status = device->SetParameter(param, value);
    if (status == CANSparkMax::ParameterStatus::kOK) {
        return CANError::kOK;
    }
    return CANError::kError;
}

static double GetMotionParamRaw(CANSparkMax* device,
                                CANSparkMax::ConfigParameter kParamSlot0,
                                int slotID) {
    double value;

    if (slotID < 0 || slotID > 3 ||
        kParamSlot0 < CANSparkMax::ConfigParameter::kSmartMotionMaxVelocity_0 ||
        kParamSlot0 >
            CANSparkMax::ConfigParameter::kSmartMotionAccelStrategy_0) {
        return 0;
    }

    int offset = slotID * 5 + static_cast<int>(kParamSlot0);
    CANSparkMax::ConfigParameter param =
        static_cast<CANSparkMax::ConfigParameter>(offset);

    auto status = device->GetParameter(param, value);
    if (status == CANSparkMax::ParameterStatus::kOK) {
        return value;
    }
    return 0;
}

static CANError SetGainRaw(CANSparkMax* device,
                           CANSparkMax::ConfigParameter kParamSlot0, int slotID,
                           double value) {
    if (slotID < 0 || slotID > 3 ||
        kParamSlot0 < CANSparkMax::ConfigParameter::kP_0 ||
        kParamSlot0 > CANSparkMax::ConfigParameter::kOutputMax_0) {
        return CANError::kError;
    }

    int offset = slotID * 8 + static_cast<int>(kParamSlot0);
    CANSparkMax::ConfigParameter param =
        static_cast<CANSparkMax::ConfigParameter>(offset);

    auto status = device->SetParameter(param, value);
    if (status == CANSparkMax::ParameterStatus::kOK) {
        return CANError::kOK;
    }
    return CANError::kError;
}

static double GetGainRaw(CANSparkMax* device,
                         CANSparkMax::ConfigParameter kParamSlot0, int slotID) {
    double value;

    if (slotID < 0 || slotID > 3 ||
        kParamSlot0 < CANSparkMax::ConfigParameter::kP_0 ||
        kParamSlot0 > CANSparkMax::ConfigParameter::kOutputMax_3) {
        return 0;
    }

    int offset = slotID * 8 + static_cast<int>(kParamSlot0);
    CANSparkMax::ConfigParameter param =
        static_cast<CANSparkMax::ConfigParameter>(offset);

    auto status = device->GetParameter(param, value);
    if (status == CANSparkMax::ParameterStatus::kOK) {
        return value;
    }
    return 0;
}

CANError CANPIDController::SetP(double gain, int slotID) {
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kP_0, slotID,
                      gain);
}

CANError CANPIDController::SetI(double gain, int slotID) {
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kI_0, slotID,
                      gain);
}

CANError CANPIDController::SetD(double gain, int slotID) {
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kD_0, slotID,
                      gain);
}

CANError CANPIDController::SetDFilter(double gain, int slotID) {
    gain = gain < 0 ? 0 : gain;
    gain = gain > 1 ? 1 : gain;
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kDFilter_0,
                      slotID, gain);
}

CANError CANPIDController::SetFF(double gain, int slotID) {
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kF_0, slotID,
                      gain);
}

CANError CANPIDController::SetIZone(double IZone, int slotID) {
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kIZone_0, slotID,
                      IZone);
}

CANError CANPIDController::SetOutputRange(double min, double max, int slotID) {
    if (m_device->GetInverted()) {
        double tmp = min;
        min = max * -1;
        max = tmp * -1;
    }

    CANError status = SetGainRaw(
        m_device, CANSparkMax::ConfigParameter::kOutputMin_0, slotID, min);
    if (status != CANError::kOK) {
        return status;
    }
    return SetGainRaw(m_device, CANSparkMax::ConfigParameter::kOutputMax_0,
                      slotID, max);
}

double CANPIDController::GetP(int slotID) {
    return GetGainRaw(m_device, CANSparkMax::ConfigParameter::kP_0, slotID);
}

double CANPIDController::GetI(int slotID) {
    return GetGainRaw(m_device, CANSparkMax::ConfigParameter::kI_0, slotID);
}

double CANPIDController::GetD(int slotID) {
    return GetGainRaw(m_device, CANSparkMax::ConfigParameter::kD_0, slotID);
}

double CANPIDController::GetDFilter(int slotID) {
    return GetGainRaw(m_device, CANSparkMax::ConfigParameter::kDFilter_0,
                      slotID);
}

double CANPIDController::GetFF(int slotID) {
    return GetGainRaw(m_device, CANSparkMax::ConfigParameter::kF_0, slotID);
}

double CANPIDController::GetIZone(int slotID) {
    return GetGainRaw(m_device, CANSparkMax::ConfigParameter::kIZone_0, slotID);
}

double CANPIDController::GetOutputMin(int slotID) {
    if (m_device->GetInverted()) {
        return -1 * GetGainRaw(m_device,CANSparkMax::ConfigParameter::kOutputMax_0, slotID);
    } else {
        return GetGainRaw(m_device,CANSparkMax::ConfigParameter::kOutputMin_0, slotID);
    }
}

double CANPIDController::GetOutputMax(int slotID) {
    if (m_device->GetInverted()) {
        return -1 * GetGainRaw(m_device,CANSparkMax::ConfigParameter::kOutputMin_0, slotID);
    } else {
        return GetGainRaw(m_device,CANSparkMax::ConfigParameter::kOutputMax_0, slotID);
    }
}

CANError CANPIDController::SetSmartMotionMaxVelocity(double maxVel,
                                                     int slotID) {
    return SetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionMaxVelocity_0,
        slotID, maxVel);
}

CANError CANPIDController::SetSmartMotionMaxAccel(double maxAccel, int slotID) {
    return SetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionMaxAccel_0, slotID,
        maxAccel);
}

CANError CANPIDController::SetSmartMotionMinOutputVelocity(double minVel,
                                                           int slotID) {
    return SetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionMinVelOutput_0,
        slotID, minVel);
}

CANError CANPIDController::SetSmartMotionAllowedClosedLoopError(
    double allowedErr, int slotID) {
    return SetMotionParamRaw(
        m_device,
        CANSparkMax::ConfigParameter::kSmartMotionAllowedClosedLoopError_0,
        slotID, allowedErr);
}

CANError CANPIDController::SetSmartMotionAccelStrategy(
    CANPIDController::AccelStrategy accelStrategy, int slotID) {
    return SetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionAccelStrategy_0,
        slotID, static_cast<double>(accelStrategy));
}

double CANPIDController::GetSmartMotionMaxVelocity(int slotID) {
    return GetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionMaxVelocity_0,
        slotID);
}

double CANPIDController::GetSmartMotionMaxAccel(int slotID) {
    return GetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionMaxAccel_0, slotID);
}

double CANPIDController::GetSmartMotionMinOutputVelocity(int slotID) {
    return GetMotionParamRaw(
        m_device, CANSparkMax::ConfigParameter::kSmartMotionMinVelOutput_0,
        slotID);
}

double CANPIDController::GetSmartMotionAllowedClosedLoopError(int slotID) {
    return GetMotionParamRaw(
        m_device,
        CANSparkMax::ConfigParameter::kSmartMotionAllowedClosedLoopError_0,
        slotID);
}

CANPIDController::AccelStrategy CANPIDController::GetSmartMotionAccelStrategy(
    int slotID) {
    return static_cast<CANPIDController::AccelStrategy>(
        static_cast<int>(GetMotionParamRaw(
            m_device, CANSparkMax::ConfigParameter::kSmartMotionAccelStrategy_0,
            slotID)));
}

CANError CANPIDController::SetIMaxAccum(double iMaxAccum, int slotID) {
    return SetPIDLimitParamsRaw(m_device,
                                CANSparkMax::ConfigParameter::kIMaxAccum_0,
                                slotID, iMaxAccum);
}

double CANPIDController::GetIMaxAccum(int slotID) {
    return GetPIDLimitParamsRaw(
        m_device, CANSparkMax::ConfigParameter::kIMaxAccum_0, slotID);
}

CANError CANPIDController::SetIAccum(double iAccum) {
    return m_device->SetIAccum(iAccum);
}

double CANPIDController::GetIAccum() {
    return m_device->GetPeriodicStatus2().iAccum;
}
