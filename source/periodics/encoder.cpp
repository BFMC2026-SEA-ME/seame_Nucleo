/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#include <periodics/encoder.hpp>
#include <cstring>
#include <cstdio>
#include <cmath>

#define BUFFER_SIZE 64

namespace periodics
{
    /**
     * @brief Constructor - initializes encoder hardware and interrupts
     */
    CEncoder::CEncoder(
            std::chrono::milliseconds f_period,
            UnbufferedSerial& f_serial,
            PinName f_pinA,
            PinName f_pinB,
            PinName f_pinX)
        : utils::CTask(f_period)
        , m_isActive(false)
        , m_serial(f_serial)
        , m_encoderA(f_pinA)
        , m_encoderB(f_pinB)
        , m_encoderX(f_pinX)
        , m_pulseCount(0)
        , m_revCount(0)
        , m_totalDistance(0.0f)
        , m_prevPulseCount(0)
        , m_prevTime_us(0)
        , m_prevA(0)
        , m_prevX(0)
        , m_prevAB(0)
        , m_rpmFiltered(0.0f)
        , m_filterInit(false)
        , m_rpmBuf{0}
        , m_rpmBufIdx(0)
        , m_rpmBufCount(0)
        , m_rpmSum(0.0f)
        , m_noPulseTime_us(0)
    {
        m_encoderA.mode(PullUp);
        m_encoderB.mode(PullUp);
        m_encoderX.mode(PullUp);
        m_encoderA.rise(mbed::callback(this, &CEncoder::encoderISR));
        m_encoderA.fall(mbed::callback(this, &CEncoder::encoderISR));
        m_encoderB.rise(mbed::callback(this, &CEncoder::encoderISR));
        m_encoderB.fall(mbed::callback(this, &CEncoder::encoderISR));
        m_encoderX.rise(mbed::callback(this, &CEncoder::indexISR));
        m_timer.start();
        m_prevTime_us = m_timer.elapsed_time().count();
        m_prevA = m_encoderA.read();
        m_prevX = m_encoderX.read();
        m_prevAB = (uint8_t)((m_encoderA.read() << 1) | m_encoderB.read());
    }

    /**
     * @brief Destructor
     */
    CEncoder::~CEncoder()
    {
        m_timer.stop();
    }

    /**
     * @brief Encoder interrupt service routine
     *
     * Called on each edge of channel A. Reads channel B to determine direction.
     * If A rises and B is low -> forward
     * If A rises and B is high -> reverse
     */
    void CEncoder::encoderISR()
    {
        // 4x quadrature decoding using state transition table
        static const int8_t s_transition[16] = {
            0, -1, +1,  0,
            +1,  0,  0, -1,
            -1,  0,  0, +1,
            0, +1, -1,  0
        };

        uint8_t a = (uint8_t)m_encoderA.read();
        uint8_t b = (uint8_t)m_encoderB.read();
        uint8_t state = (uint8_t)((a << 1) | b);
        uint8_t idx = (uint8_t)((m_prevAB << 2) | state);
        int8_t delta = s_transition[idx];
        m_prevAB = state;
        m_pulseCount += (int32_t)ENCODER_DIR * (int32_t)delta;
    }

    /**
     * @brief Index pin interrupt service routine
     *
     * Called once per revolution. Used for accurate revolution counting
     * and odometry distance calculation.
     */
    void CEncoder::indexISR()
    {
        // Determine direction from channel B
        if (m_encoderB.read()) {
            m_revCount -= ENCODER_DIR;  // Reverse
        } else {
            m_revCount += ENCODER_DIR;  // Forward
        }
    }

    /**
     * @brief Calculate RPM from pulse count difference
     *
     * @return Wheel RPM (accounting for gear ratio)
     */
    float CEncoder::calculateRPM()
    {
        // Get current time
        uint64_t currentTime_us = m_timer.elapsed_time().count();
        uint64_t deltaTime_us = currentTime_us - m_prevTime_us;

        // Prevent division by zero
        if (deltaTime_us == 0) {
            return 0.0f;
        }

        // Get pulse difference (atomic read)
        __disable_irq();
        int32_t currentPulses = m_pulseCount;
        __enable_irq();

        int32_t deltaPulses = currentPulses - m_prevPulseCount;

        // Update previous values
        m_prevPulseCount = currentPulses;
        m_prevTime_us = currentTime_us;

        if (deltaPulses == 0) {
            m_noPulseTime_us += deltaTime_us;
        } else {
            m_noPulseTime_us = 0;
        }

        // Calculate encoder shaft RPM
        // RPM = (pulses / CPR) * (60,000,000 / deltaTime_us)
        float encoderRPM = (float)deltaPulses / (float)ENCODER_CPR * (60000000.0f / (float)deltaTime_us);

        // Apply gear ratio to get wheel RPM
        // Wheel turns (drive / driven) per 1 encoder turn
        float wheelRPM = encoderRPM * GEAR_RATIO;

        return wheelRPM;
    }

    /**
     * @brief Calculate linear velocity from RPM
     *
     * @param rpm Wheel RPM
     * @return Linear velocity in m/s
     */
    float CEncoder::calculateVelocity(float rpm)
    {
        // velocity = RPM * circumference / 60
        // velocity (m/s) = RPM * 0.207345 / 60
        return rpm * WHEEL_CIRCUMFERENCE_M / 60.0f;
    }

    /**
     * @brief Serial callback to activate/deactivate encoder
     *
     * @param a Input: "1" to activate, "0" to deactivate
     * @param b Output: Response message
     */
    void CEncoder::serialCallbackENCODERcommand(char const* a, char* b)
    {
        unsigned int l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%u", &l_isActivate);

        if (1 == l_res) {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30) {
                m_isActive = (l_isActivate >= 1);
                sprintf(b, "1");
            } else {
                sprintf(b, "kl 15/30 is required!!");
            }
        } else {
            sprintf(b, "syntax error");
        }
    }

    void CEncoder::poll()
    {
        // Poll encoder channel A - rising edge only (1x counting)
        int currentA = m_encoderA.read();
        if (currentA == 1 && m_prevA == 0) {
            // Rising edge on A: read B for direction
            if (m_encoderB.read()) {
                m_pulseCount--;   // Reverse
            } else {
                m_pulseCount++;   // Forward
            }
        }
        m_prevA = currentA;

        // Poll index pin for revolution counting
        int currentX = m_encoderX.read();
        if (currentX && !m_prevX) {
            // Rising edge on index
            if (m_encoderB.read()) {
                m_revCount++;
            } else {
                m_revCount--;
            }
        }
        m_prevX = currentX;
    }

    /**
     * @brief Periodic run method - reads encoder and sends data via UART
     *
     * Data format: @encoder:rpm;velocity;distance;;\r\n
     * Example: @encoder:150.50;0.512;12.345;;\r\n
     */
    void CEncoder::_run()
    {
        if (bool_globalsV_imuenc_isActive) {
            return;
        }
        if (!bool_globalsV_separateSensors_isActive) {
            return;
        }
        if (uint8_globalsV_value_of_kl != 15 && uint8_globalsV_value_of_kl != 30) {
            return;
        }
        sendData();
    }

    void CEncoder::sample(EncoderSample& out)
    {
        // Calculate RPM and velocity (signed)
        float rpm_raw = calculateRPM();
        bool no_pulse_timeout = (m_noPulseTime_us >= RPM_ZERO_TIMEOUT_US);
        bool no_pulse = (m_noPulseTime_us > 0);
        if (no_pulse_timeout) {
            // Snap to zero and clear filter history to avoid slow decay at standstill.
            m_rpmSum = 0.0f;
            m_rpmBufIdx = 0;
            m_rpmBufCount = 0;
            memset(m_rpmBuf, 0, sizeof(m_rpmBuf));
            m_rpmFiltered = 0.0f;
            m_filterInit = false;
            rpm_raw = 0.0f;
        }
        // Moving average
        if (!no_pulse_timeout && !no_pulse && m_rpmBufCount < RPM_AVG_WINDOW) {
            m_rpmBuf[m_rpmBufIdx] = rpm_raw;
            m_rpmSum += rpm_raw;
            m_rpmBufCount++;
        } else if (!no_pulse_timeout && !no_pulse) {
            m_rpmSum -= m_rpmBuf[m_rpmBufIdx];
            m_rpmBuf[m_rpmBufIdx] = rpm_raw;
            m_rpmSum += rpm_raw;
        }
        if (!no_pulse_timeout && !no_pulse) {
            m_rpmBufIdx = (m_rpmBufIdx + 1) % RPM_AVG_WINDOW;
        }
        float rpm_avg = (m_rpmBufCount > 0) ? (m_rpmSum / (float)m_rpmBufCount) : rpm_raw;

        // IIR filter on top of moving average
        float rpm = 0.0f;
        if (no_pulse_timeout) {
            m_rpmFiltered = 0.0f;
            m_filterInit = false;
            rpm = 0.0f;
        } else if (no_pulse) {
            // Hold last filtered value to avoid low-speed zero jitter.
            rpm = m_filterInit ? m_rpmFiltered : 0.0f;
        } else if (!m_filterInit) {
            m_rpmFiltered = rpm_avg;
            m_filterInit = true;
            rpm = m_rpmFiltered;
        } else {
            m_rpmFiltered += RPM_FILTER_ALPHA * (rpm_avg - m_rpmFiltered);
            rpm = m_rpmFiltered;
        }
        float velocity = calculateVelocity(rpm);

        __disable_irq();
        int32_t pulses = m_pulseCount;
        __enable_irq();
        m_totalDistance = (float)pulses / (float)ENCODER_CPR * WHEEL_CIRCUMFERENCE_M * GEAR_RATIO;

        int32_t rpm_int = (int32_t)(rpm * 100.0f);
        int32_t vel_int = (int32_t)(velocity * 1000.0f);
        int32_t dist_int = (int32_t)(m_totalDistance * 1000.0f);

        out.rpm_centi = rpm_int;
        out.vel_mms = vel_int;
        out.dist_mm = dist_int;
        out.dist_negative = (m_totalDistance < 0.0f);
    }

    void CEncoder::sendData()
    {
        EncoderSample sampleData;
        sample(sampleData);

        const char* rpm_sign = (sampleData.rpm_centi < 0) ? "-" : "";
        int32_t abs_rpm = abs(sampleData.rpm_centi);
        const char* vel_sign = (sampleData.vel_mms < 0) ? "-" : "";
        int32_t abs_vel = abs(sampleData.vel_mms);

        // Handle negative sign for distance (-0.xxx case)
        const char* dist_sign = sampleData.dist_negative ? "-" : "";
        int32_t abs_dist = abs(sampleData.dist_mm);

        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "@encoder:%s%d.%02d;%s%d.%03d;%s%d.%03d;;\r\n",
                 rpm_sign, (int)(abs_rpm / 100), (int)(abs_rpm % 100),
                 vel_sign, (int)(abs_vel / 1000), (int)(abs_vel % 1000),
                 dist_sign, (int)(abs_dist / 1000), (int)(abs_dist % 1000));

        m_serial.write(buffer, strlen(buffer));
    }

}; // namespace periodics
