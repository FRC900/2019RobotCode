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

#include "rev/CANEncoder.h"

#include "rev/CANError.h"
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"

using namespace rev;

CANEncoder::CANEncoder(CANSparkMax& device) : m_device(&device) {}

double CANEncoder::GetPosition() {
    return m_device->GetPeriodicStatus2().sensorPosition;
}

double CANEncoder::GetVelocity() {
    return m_device->GetPeriodicStatus1().sensorVelocity;
}

CANError CANEncoder::SetPosition(double position) {
    return m_device->SetEncPosition(position);
}

CANError CANEncoder::SetPositionConversionFactor(double factor) {
    auto status = m_device->SetParameter(
        CANSparkMaxLowLevel::ConfigParameter::kPositionConversionFactor,
        static_cast<float>(factor));
    if (status == CANSparkMaxLowLevel::ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

CANError CANEncoder::SetVelocityConversionFactor(double factor) {
    auto status = m_device->SetParameter(
        CANSparkMaxLowLevel::ConfigParameter::kVelocityConversionFactor,
        static_cast<float>(factor));
    if (status == CANSparkMaxLowLevel::ParameterStatus::kOK) {
        return CANError::kOK;
    } else {
        return CANError::kError;
    }
}

double CANEncoder::GetPositionConversionFactor() {
    double value;
    auto status = m_device->GetParameter(
        CANSparkMaxLowLevel::ConfigParameter::kPositionConversionFactor, value);
    if (status == CANSparkMaxLowLevel::ParameterStatus::kOK) {
        return value;
    } else {
        return 0.0;
    }
}

double CANEncoder::GetVelocityConversionFactor() {
    double value;
    auto status = m_device->GetParameter(
        CANSparkMaxLowLevel::ConfigParameter::kVelocityConversionFactor, value);
    if (status == CANSparkMaxLowLevel::ParameterStatus::kOK) {
        return value;
    } else {
        return 0.0;
    }
}
