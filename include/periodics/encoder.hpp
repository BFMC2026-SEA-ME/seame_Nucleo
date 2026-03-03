/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
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

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <mbed.h>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <chrono>

namespace periodics
{
    // TODO: Verify these constants for your encoder/wheel setup.
    static constexpr int32_t ENCODER_PPR = 1024;
    // Counting mode: 1x = rising edge on A, 2x = both edges on A, 4x = both edges on A/B.
    static constexpr int32_t ENCODER_COUNT_MULTIPLIER = 4;
    static constexpr int32_t ENCODER_CPR = ENCODER_PPR * ENCODER_COUNT_MULTIPLIER;
    // Direction flip: set to -1 to invert sign, +1 for normal.
    static constexpr int8_t ENCODER_DIR = -1;
    // Motor shaft gear (drive) : wheel gear (driven)
    static constexpr float DRIVE_GEAR_TEETH = 11.0f;
    static constexpr float WHEEL_GEAR_TEETH = 34.0f;
    // Wheel RPM = encoder RPM * (drive / driven)
    static constexpr float GEAR_RATIO = DRIVE_GEAR_TEETH / WHEEL_GEAR_TEETH;
    // Wheel diameter = 6.6 cm => circumference = pi * d
    static constexpr float WHEEL_CIRCUMFERENCE_M = 0.207345f;
    // Simple low-pass filter for RPM (0..1). Higher = more responsive, lower = smoother.
    static constexpr float RPM_FILTER_ALPHA = 0.1f;
    // Moving average window to reduce noise.
    static constexpr int RPM_AVG_WINDOW = 5;
    // Snap RPM to zero if no pulses for this long (us).
    static constexpr uint64_t RPM_ZERO_TIMEOUT_US = 150000ULL;

    struct EncoderSample
    {
        int32_t rpm_centi;
        int32_t vel_mms;
        int32_t dist_mm;
        bool dist_negative;
    };

    class CEncoder : public utils::CTask
    {
        public:
            CEncoder(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                PinName f_pinA,
                PinName f_pinB,
                PinName f_pinX
            );
            ~CEncoder();

            void serialCallbackENCODERcommand(char const* a, char* b);
            void poll();
            void sample(EncoderSample& out);
            void sendData();

        private:
            virtual void _run() override;
            void encoderISR();
            void indexISR();
            float calculateRPM();
            float calculateVelocity(float rpm);

            bool m_isActive;
            UnbufferedSerial& m_serial;
            InterruptIn m_encoderA;
            InterruptIn m_encoderB;
            InterruptIn m_encoderX;
            Timer m_timer;

            volatile int32_t m_pulseCount;
            volatile int32_t m_revCount;
            float m_totalDistance;
            int32_t m_prevPulseCount;
            uint64_t m_prevTime_us;
            int m_prevA;
            int m_prevX;
            uint8_t m_prevAB;
            float m_rpmFiltered;
            bool m_filterInit;
            float m_rpmBuf[RPM_AVG_WINDOW];
            int m_rpmBufIdx;
            int m_rpmBufCount;
            float m_rpmSum;
            uint64_t m_noPulseTime_us;
    }; // class CEncoder

}; // namespace periodics

#endif // ENCODER_HPP
