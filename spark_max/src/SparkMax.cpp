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

#include "rev/SparkMax.h"

#include <hal/HAL.h>

using namespace rev;

SparkMax::SparkMax(int channel) : frc::PWMSpeedController(channel) {
    /* Note that the Spark Max uses the following bounds for PWM values. These
     * values should work reasonably well for most controllers, but if users
     * experience issues such as asymmetric behavior around the deadband or
     * inability to saturate the controller in either direction, calibration is
     * recommended. The calibration procedure can be found in the Spark Max User
     * Manual available from REV Robotics.
     *
     *   2.003ms = full "forward"
     *   1.55ms = the "high end" of the deadband range
     *   1.50ms = center of the deadband range (off)
     *   1.46ms = the "low end" of the deadband range
     *   0.999ms = full "reverse"
     */
    SetBounds(2.003, 1.55, 1.50, 1.46, .999);
    SetPeriodMultiplier(kPeriodMultiplier_1X);
    SetSpeed(0.0);
    SetZeroLatch();

    HAL_Report(HALUsageReporting::kResourceType_RevSparkMaxPWM, GetChannel());
    SetName("SparkMax", GetChannel());
}
