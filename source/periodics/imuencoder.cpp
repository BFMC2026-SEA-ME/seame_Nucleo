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

#include <periodics/imuencoder.hpp>
#include <cstring>
#include <cstdio>
#include <cmath>

#define BUFFER_SIZE 1024

namespace periodics
{
    static inline void split_milli(int32_t value, const char** sign, int32_t* ipart, int32_t* fpart)
    {
        int64_t abs_value = value;
        if (abs_value < 0) {
            *sign = "-";
            abs_value = -abs_value;
        } else {
            *sign = "";
        }
        *ipart = (int32_t)(abs_value / 1000);
        *fpart = (int32_t)(abs_value % 1000);
    }

    CImuEncoderSync::CImuEncoderSync(
            std::chrono::milliseconds f_period,
            UnbufferedSerial& f_serial,
            CImu& f_imu,
            CEncoder& f_encoder)
        : utils::CTask(f_period)
        , m_serial(f_serial)
        , m_imu(f_imu)
        , m_encoder(f_encoder)
        , m_isActive(true)
    {
        bool_globalsV_imuenc_isActive = true;
        m_timer.start();
    }

    void CImuEncoderSync::serialCallbackIMUENCcommand(char const* a, char* b)
    {
        unsigned int l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%u", &l_isActivate);

        if (1 == l_res) {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30) {
                m_isActive = (l_isActivate >= 1);
                bool_globalsV_imuenc_isActive = m_isActive;
                sprintf(b, "1");
            } else {
                sprintf(b, "kl 15/30 is required!!");
            }
        } else {
            sprintf(b, "syntax error");
        }
    }

    void CImuEncoderSync::_run()
    {
        if (!m_isActive) return;
        if (uint8_globalsV_value_of_kl != 15 && uint8_globalsV_value_of_kl != 30) {
            return;
        }
        if (!bool_globalsV_imu_isActive) {
            return;
        }

        uint64_t ts_us = m_timer.elapsed_time().count();

        ImuSample imuSample;
        if (!m_imu.sample(imuSample)) {
            return;
        }

        EncoderSample encSample;
        m_encoder.sample(encSample);

        const char* rpm_sign = (encSample.rpm_centi < 0) ? "-" : "";
        int32_t abs_rpm = abs(encSample.rpm_centi);
        const char* vel_sign = (encSample.vel_mms < 0) ? "-" : "";
        int32_t abs_vel = abs(encSample.vel_mms);
        const char* dist_sign = encSample.dist_negative ? "-" : "";
        int32_t abs_dist = abs(encSample.dist_mm);

        const char* er_sign;
        const char* ep_sign;
        const char* eh_sign;
        const char* gx_sign;
        const char* gy_sign;
        const char* gz_sign;
        const char* ax_sign;
        const char* ay_sign;
        const char* az_sign;
        const char* vx_sign;
        const char* vy_sign;
        const char* vz_sign;
        const char* mx_sign;
        const char* my_sign;
        const char* mz_sign;
        const char* qx_sign;
        const char* qy_sign;
        const char* qz_sign;
        const char* qw_sign;
        int32_t er_int;
        int32_t ep_int;
        int32_t eh_int;
        int32_t gx_int;
        int32_t gy_int;
        int32_t gz_int;
        int32_t ax_int;
        int32_t ay_int;
        int32_t az_int;
        int32_t vx_int;
        int32_t vy_int;
        int32_t vz_int;
        int32_t mx_int;
        int32_t my_int;
        int32_t mz_int;
        int32_t qx_int;
        int32_t qy_int;
        int32_t qz_int;
        int32_t qw_int;
        int32_t er_frac;
        int32_t ep_frac;
        int32_t eh_frac;
        int32_t gx_frac;
        int32_t gy_frac;
        int32_t gz_frac;
        int32_t ax_frac;
        int32_t ay_frac;
        int32_t az_frac;
        int32_t vx_frac;
        int32_t vy_frac;
        int32_t vz_frac;
        int32_t mx_frac;
        int32_t my_frac;
        int32_t mz_frac;
        int32_t qx_frac;
        int32_t qy_frac;
        int32_t qz_frac;
        int32_t qw_frac;

        split_milli(imuSample.euler_r_mdeg, &er_sign, &er_int, &er_frac);
        split_milli(imuSample.euler_p_mdeg, &ep_sign, &ep_int, &ep_frac);
        split_milli(imuSample.euler_h_mdeg, &eh_sign, &eh_int, &eh_frac);
        split_milli(imuSample.gyro_x_mrads, &gx_sign, &gx_int, &gx_frac);
        split_milli(imuSample.gyro_y_mrads, &gy_sign, &gy_int, &gy_frac);
        split_milli(imuSample.gyro_z_mrads, &gz_sign, &gz_int, &gz_frac);
        split_milli(imuSample.accel_x_mms2, &ax_sign, &ax_int, &ax_frac);
        split_milli(imuSample.accel_y_mms2, &ay_sign, &ay_int, &ay_frac);
        split_milli(imuSample.accel_z_mms2, &az_sign, &az_int, &az_frac);
        split_milli(imuSample.vel_x_mms, &vx_sign, &vx_int, &vx_frac);
        split_milli(imuSample.vel_y_mms, &vy_sign, &vy_int, &vy_frac);
        split_milli(imuSample.vel_z_mms, &vz_sign, &vz_int, &vz_frac);
        split_milli(imuSample.mag_x_nT, &mx_sign, &mx_int, &mx_frac);
        split_milli(imuSample.mag_y_nT, &my_sign, &my_int, &my_frac);
        split_milli(imuSample.mag_z_nT, &mz_sign, &mz_int, &mz_frac);
        split_milli(imuSample.quat_x_milli, &qx_sign, &qx_int, &qx_frac);
        split_milli(imuSample.quat_y_milli, &qy_sign, &qy_int, &qy_frac);
        split_milli(imuSample.quat_z_milli, &qz_sign, &qz_int, &qz_frac);
        split_milli(imuSample.quat_w_milli, &qw_sign, &qw_int, &qw_frac);

        // Covariances are emitted as fixed-point with 6 fractional digits.
        const int32_t cov_zero_int = 0;
        const int32_t cov_zero_frac = 0;

        const int32_t cov_ori_micro = 1220;   // 0.001220 (rad^2)
        const int32_t cov_ang_micro = 400;    // 0.000400 ((rad/s)^2)
        const int32_t cov_lin_micro = 40000;  // 0.040000 ((m/s^2)^2)

        const int32_t cov_ori_int = cov_ori_micro / 1000000;
        const int32_t cov_ori_frac = cov_ori_micro % 1000000;
        const int32_t cov_ang_int = cov_ang_micro / 1000000;
        const int32_t cov_ang_frac = cov_ang_micro % 1000000;
        const int32_t cov_lin_int = cov_lin_micro / 1000000;
        const int32_t cov_lin_frac = cov_lin_micro % 1000000;

        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer),
            "@imuenc:%llu;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%02d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;"
            "%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;"
            "%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;"
            "%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;%d.%06d;;\r\n",
            (unsigned long long)ts_us,
            er_sign, er_int, er_frac,
            ep_sign, ep_int, ep_frac,
            eh_sign, eh_int, eh_frac,
            gx_sign, gx_int, gx_frac,
            gy_sign, gy_int, gy_frac,
            gz_sign, gz_int, gz_frac,
            ax_sign, ax_int, ax_frac,
            ay_sign, ay_int, ay_frac,
            az_sign, az_int, az_frac,
            vx_sign, vx_int, vx_frac,
            vy_sign, vy_int, vy_frac,
            vz_sign, vz_int, vz_frac,
            rpm_sign, (int)(abs_rpm / 100), (int)(abs_rpm % 100),
            vel_sign, (int)(abs_vel / 1000), (int)(abs_vel % 1000),
            dist_sign, (int)(abs_dist / 1000), (int)(abs_dist % 1000),
            mx_sign, mx_int, mx_frac,
            my_sign, my_int, my_frac,
            mz_sign, mz_int, mz_frac,
            qx_sign, qx_int, qx_frac,
            qy_sign, qy_int, qy_frac,
            qz_sign, qz_int, qz_frac,
            qw_sign, qw_int, qw_frac,
            // orientation_covariance (row-major 3x3)
            cov_ori_int, cov_ori_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_ori_int, cov_ori_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_ori_int, cov_ori_frac,
            // angular_velocity_covariance (row-major 3x3)
            cov_ang_int, cov_ang_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_ang_int, cov_ang_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_ang_int, cov_ang_frac,
            // linear_acceleration_covariance (row-major 3x3)
            cov_lin_int, cov_lin_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_lin_int, cov_lin_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_zero_int, cov_zero_frac,
            cov_lin_int, cov_lin_frac);

        m_serial.write(buffer, strlen(buffer));
    }

} // namespace periodics
