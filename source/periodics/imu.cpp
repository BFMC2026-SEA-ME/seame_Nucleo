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


#include <periodics/imu.hpp>
#include "imu.hpp"
#include <cmath>

#define IMU_BUFFER_SIZE                 240
#define BNO055_EULER_DIV_DEG_int        16
#define BNO055_LINEAR_ACCEL_DIV_MSQ_int 100
#define BNO055_GYRO_DIV_RPS_int         900
#define BNO055_MAG_DIV_UT_int           16
#define precision_scaling_factor        1000
#define IMU_MIN_PERIOD_MS               20
#define IMU_MAX_DT_US                   200000

namespace periodics{
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
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */

    /*--------------------------------------------------------------------------------------------------*
    *  Before initializiting with another value, the i2c_instance static pointer variable should be
    *  initialized with a nullptr.
    *---------------------------------------------------------------------------------------------------*/
    I2C* periodics::CImu::i2c_instance = nullptr;

    CImu::CImu(
            std::chrono::milliseconds    f_period, 
            UnbufferedSerial& f_serial,
            PinName SDA,
            PinName SCL)
        : utils::CTask(f_period)
        , m_isActive(false)
        , m_serial(f_serial)
        , m_velocityX(0)
        , m_velocityY(0)
        , m_velocityZ(0)
        , m_stationaryTime_ms(0)
        , m_delta_time_ms(f_period.count())
        , m_lastTime_us(0)
        , m_biasX(0)
        , m_biasY(0)
        , m_biasZ(0)
    {
        if(m_delta_time_ms < IMU_MIN_PERIOD_MS){
            setNewPeriod(IMU_MIN_PERIOD_MS);
            m_delta_time_ms = IMU_MIN_PERIOD_MS;
        }
        
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;

        /*---------------------------------------------------------------------------*
        *********************** START INITIALIZATION ************************
        *--------------------------------------------------------------------------*/

        /*--------------------------------------------------------------------------------------------------*
        *  i2c_instance variable member will be initialized with the actual I2C of the target board.
        *---------------------------------------------------------------------------------------------------*/      
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);

        ThisThread::sleep_for(chrono::milliseconds(300));

        /*  Based on the user need configure I2C interface.
        *  It is example code to explain how to use the bno055 API*/
        I2C_routine();

        /*--------------------------------------------------------------------------*
        *  This API used to assign the value/reference of
        *  the following parameters
        *  I2C address
        *  Bus Write
        *  Bus read
        *  Chip id
        *  Page id
        *  Accel revision id
        *  Mag revision id
        *  Gyro revision id
        *  Boot loader revision id
        *  Software revision id
        *-------------------------------------------------------------------------*/
        comres = bno055_init(&bno055);

        /*  For initializing the BNO sensor it is required to the operation mode
        * of the sensor as NORMAL
        * Normal mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_NORMAL;

        /* set the power mode as NORMAL*/
        comres += bno055_set_power_mode(power_mode);

        /*----------------------------------------------------------------*
        *  Force units and bandwidth/range to keep /Imu interpretation
        *  and covariance stable across boots.
        *-----------------------------------------------------------------*/
        comres += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
        comres += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
        comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
        comres += bno055_set_tilt_unit(0x00); /* 0x00 = degrees */
        comres += bno055_set_temp_unit(BNO055_TEMP_UNIT_CELSIUS);
        comres += bno055_set_data_output_format(0x01); /* 0x01 = Android */

        comres += bno055_set_accel_bw(BNO055_ACCEL_BW_62_5HZ);
        comres += bno055_set_accel_range(BNO055_ACCEL_RANGE_4G);
        comres += bno055_set_gyro_bw(BNO055_GYRO_BW_32HZ);
        comres += bno055_set_gyro_range(BNO055_GYRO_RANGE_500DPS);

        /************************* START READ RAW DATA ********
        * operation modes of the sensor
        * operation mode can set from the register
        * page - page0
        * register - 0x3D
        * bit - 0 to 3
        * for sensor data read following operation mode have to set
        * SENSOR MODE
        * 0x01 - BNO055_OPERATION_MODE_ACCONLY
        * 0x02 - BNO055_OPERATION_MODE_MAGONLY
        * 0x03 - BNO055_OPERATION_MODE_GYRONLY
        * 0x04 - BNO055_OPERATION_MODE_ACCMAG
        * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
        * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
        * 0x07 - BNO055_OPERATION_MODE_AMG
        * based on the user need configure the operation mode*/

        /************************* START READ RAW FUSION DATA ********
         * For reading fusion data it is required to set the
         * operation modes of the sensor
         * operation mode can set from the register
         * page - page0
         * register - 0x3D
         * bit - 0 to 3
         * for sensor data read following operation mode have to set
         * FUSION MODE
         * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
         * 0x09 - BNO055_OPERATION_MODE_COMPASS
         * 0x0A - BNO055_OPERATION_MODE_M4G
        * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
        * 0x0C - BNO055_OPERATION_MODE_NDOF
        * based on the user need configure the operation mode*/
        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

        /*----------------------------------------------------------------*
        *  Readback log for verification
        *-----------------------------------------------------------------*/
        u8 accel_unit_u8 = BNO055_INIT_VALUE;
        u8 gyro_unit_u8 = BNO055_INIT_VALUE;
        u8 euler_unit_u8 = BNO055_INIT_VALUE;
        u8 tilt_unit_u8 = BNO055_INIT_VALUE;
        u8 temp_unit_u8 = BNO055_INIT_VALUE;
        u8 data_output_format_u8 = BNO055_INIT_VALUE;
        u8 accel_bw_u8 = BNO055_INIT_VALUE;
        u8 accel_range_u8 = BNO055_INIT_VALUE;
        u8 gyro_bw_u8 = BNO055_INIT_VALUE;
        u8 gyro_range_u8 = BNO055_INIT_VALUE;
        u8 operation_mode_u8 = BNO055_INIT_VALUE;
        u8 power_mode_u8 = BNO055_INIT_VALUE;

        comres += bno055_get_accel_unit(&accel_unit_u8);
        comres += bno055_get_gyro_unit(&gyro_unit_u8);
        comres += bno055_get_euler_unit(&euler_unit_u8);
        comres += bno055_get_tilt_unit(&tilt_unit_u8);
        comres += bno055_get_temp_unit(&temp_unit_u8);
        comres += bno055_get_data_output_format(&data_output_format_u8);
        comres += bno055_get_accel_bw(&accel_bw_u8);
        comres += bno055_get_accel_range(&accel_range_u8);
        comres += bno055_get_gyro_bw(&gyro_bw_u8);
        comres += bno055_get_gyro_range(&gyro_range_u8);
        comres += bno055_get_operation_mode(&operation_mode_u8);
        comres += bno055_get_power_mode(&power_mode_u8);

        char config_buffer[180];
        snprintf(
            config_buffer,
            sizeof(config_buffer),
            "@imu_cfg:unit(acc=%u,gyro=%u,euler=%u,tilt=%u,temp=%u,fmt=%u);bw(acc=%u,gyro=%u);range(acc=%u,gyro=%u);op=%u;pwr=%u;;\r\n",
            accel_unit_u8,
            gyro_unit_u8,
            euler_unit_u8,
            tilt_unit_u8,
            temp_unit_u8,
            data_output_format_u8,
            accel_bw_u8,
            gyro_bw_u8,
            accel_range_u8,
            gyro_range_u8,
            operation_mode_u8,
            power_mode_u8);
        m_serial.write(config_buffer, strlen(config_buffer));

        m_timer.start();
        m_lastTime_us = m_timer.elapsed_time().count();
    }

    /** @brief  CImu class destructor
     */
    CImu::~CImu()
    {
        m_timer.stop();
        /*-----------------------------------------------------------------------*
        ************************* START DE-INITIALIZATION ***********************
        *-------------------------------------------------------------------------*/
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;
        /*  For de - initializing the BNO sensor it is required
        * to the operation mode of the sensor as SUSPEND
        * Suspend mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_SUSPEND;

        /* set the power mode as SUSPEND*/
        comres += bno055_set_power_mode(power_mode);

        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
        /*---------------------------------------------------------------------*
        ************************* END DE-INITIALIZATION **********************
        *---------------------------------------------------------------------*/
    };

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
    void CImu::serialCallbackIMUcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                bool activate = (l_isActivate >= 1);
                m_isActive = activate;
                bool_globalsV_imu_isActive = activate;
                if (activate) {
                    m_lastTime_us = m_timer.elapsed_time().count();
                    m_stationaryTime_ms = 0;
                }
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }

    /* This API is an example for reading sensor data
    *  \param: None
    *  \return: communication result
    */
    // s32 CImu::bno055_data_readout_template(void)
    // {
    //     /* Variable used to return value of
    //     * communication routine*/
    //     s32 comres = BNO055_ERROR;

    //     /************************* START READ RAW SENSOR DATA****************/

    //     /*  Using BNO055 sensor we can read the following sensor data and
    //     * virtual sensor data
    //     * Sensor data:
    //     * Accel
    //     * Mag
    //     * Gyro
    //     * Virtual sensor data
    //     * Euler
    //     * Quaternion
    //     * Linear acceleration
    //     * Gravity sensor */

    //     /*********read raw accel data***********/
    //     /* variable used to read the accel x data */
    //     s16 accel_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the accel y data */
    //     s16 accel_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the accel z data */
    //     s16 accel_dataz = BNO055_INIT_VALUE;

    //     /* variable used to read the accel xyz data */
    //     struct bno055_accel_t accel_xyz;

    //     /*  Raw accel X, Y and Z data can read from the register
    //     * page - page 0
    //     * register - 0x08 to 0x0D*/
    //     comres += bno055_read_accel_x(&accel_datax);
    //     comres += bno055_read_accel_y(&accel_datay);
    //     comres += bno055_read_accel_z(&accel_dataz);
    //     comres += bno055_read_accel_xyz(&accel_xyz);

    //     /*********read raw mag data***********/
    //     /* variable used to read the mag x data */
    //     s16 mag_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the mag y data */
    //     s16 mag_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the mag z data */
    //     s16 mag_dataz = BNO055_INIT_VALUE;

    //     /* structure used to read the mag xyz data */
    //     struct bno055_mag_t mag_xyz;

    //     /*  Raw mag X, Y and Z data can read from the register
    //     * page - page 0
    //     * register - 0x0E to 0x13*/
    //     comres += bno055_read_mag_x(&mag_datax);
    //     comres += bno055_read_mag_y(&mag_datay);
    //     comres += bno055_read_mag_z(&mag_dataz);
    //     comres += bno055_read_mag_xyz(&mag_xyz);

    //     /***********read raw gyro data***********/
    //     /* variable used to read the gyro x data */
    //     s16 gyro_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the gyro y data */
    //     s16 gyro_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the gyro z data */
    //     s16 gyro_dataz = BNO055_INIT_VALUE;

    //     /* structure used to read the gyro xyz data */
    //     struct bno055_gyro_t gyro_xyz;

    //     /*  Raw gyro X, Y and Z data can read from the register
    //     * page - page 0
    //     * register - 0x14 to 0x19*/
    //     comres += bno055_read_gyro_x(&gyro_datax);
    //     comres += bno055_read_gyro_y(&gyro_datay);
    //     comres += bno055_read_gyro_z(&gyro_dataz);
    //     comres += bno055_read_gyro_xyz(&gyro_xyz);

    //     /*************read raw Euler data************/
    //     /* variable used to read the euler h data */
    //     s16 euler_data_h = BNO055_INIT_VALUE;

    //     /* variable used to read the euler r data */
    //     s16 euler_data_r = BNO055_INIT_VALUE;

    //     /* variable used to read the euler p data */
    //     s16 euler_data_p = BNO055_INIT_VALUE;

    //     /* structure used to read the euler hrp data */
    //     struct bno055_euler_t euler_hrp;

    //     /*  Raw Euler H, R and P data can read from the register
    //     * page - page 0
    //     * register - 0x1A to 0x1E */
    //     comres += bno055_read_euler_h(&euler_data_h);
    //     comres += bno055_read_euler_r(&euler_data_r);
    //     comres += bno055_read_euler_p(&euler_data_p);
    //     comres += bno055_read_euler_hrp(&euler_hrp);

    //     /************read raw quaternion data**************/
    //     /* variable used to read the quaternion w data */
    //     s16 quaternion_data_w = BNO055_INIT_VALUE;

    //     /* variable used to read the quaternion x data */
    //     s16 quaternion_data_x = BNO055_INIT_VALUE;

    //     /* variable used to read the quaternion y data */
    //     s16 quaternion_data_y = BNO055_INIT_VALUE;

    //     /* variable used to read the quaternion z data */
    //     s16 quaternion_data_z = BNO055_INIT_VALUE;

    //     /* structure used to read the quaternion wxyz data */
    //     struct bno055_quaternion_t quaternion_wxyz;

    //     /*  Raw Quaternion W, X, Y and Z data can read from the register
    //     * page - page 0
    //     * register - 0x20 to 0x27 */
    //     comres += bno055_read_quaternion_w(&quaternion_data_w);
    //     comres += bno055_read_quaternion_x(&quaternion_data_x);
    //     comres += bno055_read_quaternion_y(&quaternion_data_y);
    //     comres += bno055_read_quaternion_z(&quaternion_data_z);
    //     comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);

    //     /************read raw linear acceleration data***********/
    //     /* variable used to read the linear accel x data */
    //     s16 linear_accel_data_x = BNO055_INIT_VALUE;

    //     /* variable used to read the linear accel y data */
    //     s16 linear_accel_data_y = BNO055_INIT_VALUE;

    //     /* variable used to read the linear accel z data */
    //     s16 linear_accel_data_z = BNO055_INIT_VALUE;

    //     /* structure used to read the linear accel xyz data */
    //     struct bno055_linear_accel_t linear_acce_xyz;

    //     /*  Raw Linear accel X, Y and Z data can read from the register
    //     * page - page 0
    //     * register - 0x28 to 0x2D */
    //     comres += bno055_read_linear_accel_x(&linear_accel_data_x);
    //     comres += bno055_read_linear_accel_y(&linear_accel_data_y);
    //     comres += bno055_read_linear_accel_z(&linear_accel_data_z);
    //     comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);

    //     /*****************read raw gravity sensor data****************/
    //     /* variable used to read the gravity x data */
    //     s16 gravity_data_x = BNO055_INIT_VALUE;

    //     /* variable used to read the gravity y data */
    //     s16 gravity_data_y = BNO055_INIT_VALUE;

    //     /* variable used to read the gravity z data */
    //     s16 gravity_data_z = BNO055_INIT_VALUE;

    //     /* structure used to read the gravity xyz data */
    //     struct bno055_gravity_t gravity_xyz;

    //     /*  Raw Gravity sensor X, Y and Z data can read from the register
    //     * page - page 0
    //     * register - 0x2E to 0x33 */
    //     comres += bno055_read_gravity_x(&gravity_data_x);
    //     comres += bno055_read_gravity_y(&gravity_data_y);
    //     comres += bno055_read_gravity_z(&gravity_data_z);
    //     comres += bno055_read_gravity_xyz(&gravity_xyz);

    //     /************************* END READ RAW SENSOR DATA****************/

    //     /******************START READ CONVERTED SENSOR DATA****************/

    //     /*************read accel converted data***************/
    //     /* variable used to read the accel x data output as m/s2 or mg */
    //     double d_accel_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the accel y data output as m/s2 or mg */
    //     double d_accel_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the accel z data output as m/s2 or mg */
    //     double d_accel_dataz = BNO055_INIT_VALUE;

    //     /* structure used to read the accel xyz data output as m/s2 or mg */
    //     struct bno055_accel_double_t d_accel_xyz;

    //     /*  API used to read accel data output as double  - m/s2 and mg
    //     * float functions also available in the BNO055 API */
    //     comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
    //     comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
    //     comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
    //     comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
    //     comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
    //     comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
    //     comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
    //     comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);

    //     /******************read mag converted data********************/
    //     /* variable used to read the mag x data output as uT*/
    //     double d_mag_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the mag y data output as uT*/
    //     double d_mag_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the mag z data output as uT*/
    //     double d_mag_dataz = BNO055_INIT_VALUE;

    //     /* structure used to read the mag xyz data output as uT*/
    //     struct bno055_mag_double_t d_mag_xyz;

    //     /*  API used to read mag data output as double  - uT(micro Tesla)
    //     * float functions also available in the BNO055 API */
    //     comres += bno055_convert_double_mag_x_uT(&d_mag_datax);
    //     comres += bno055_convert_double_mag_y_uT(&d_mag_datay);
    //     comres += bno055_convert_double_mag_z_uT(&d_mag_dataz);
    //     comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);

    //     /*****************read gyro converted data************************/
    //     /* variable used to read the gyro x data output as dps or rps */
    //     double d_gyro_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the gyro y data output as dps or rps */
    //     double d_gyro_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the gyro z data output as dps or rps */
    //     double d_gyro_dataz = BNO055_INIT_VALUE;

    //     /* structure used to read the gyro xyz data output as dps or rps */
    //     struct bno055_gyro_double_t d_gyro_xyz;

    //     /*  API used to read gyro data output as double  - dps and rps
    //     * float functions also available in the BNO055 API */
    //     comres += bno055_convert_double_gyro_x_dps(&d_gyro_datax);
    //     comres += bno055_convert_double_gyro_y_dps(&d_gyro_datay);
    //     comres += bno055_convert_double_gyro_z_dps(&d_gyro_dataz);
    //     comres += bno055_convert_double_gyro_x_rps(&d_gyro_datax);
    //     comres += bno055_convert_double_gyro_y_rps(&d_gyro_datay);
    //     comres += bno055_convert_double_gyro_z_rps(&d_gyro_dataz);
    //     comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
    //     comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);

    //     /*******************read euler converted data*******************/

    //     /* variable used to read the euler h data output
    //     * as degree or radians*/
    //     double d_euler_data_h = BNO055_INIT_VALUE;

    //     /* variable used to read the euler r data output
    //     * as degree or radians*/
    //     double d_euler_data_r = BNO055_INIT_VALUE;

    //     /* variable used to read the euler p data output
    //     * as degree or radians*/
    //     double d_euler_data_p = BNO055_INIT_VALUE;

    //     /* structure used to read the euler hrp data output
    //     * as as degree or radians */
    //     struct bno055_euler_double_t d_euler_hpr;

    //     /*  API used to read Euler data output as double  - degree and radians
    //     * float functions also available in the BNO055 API */
    //     comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
    //     comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
    //     comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);
    //     comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
    //     comres += bno055_convert_double_euler_r_rad(&d_euler_data_r);
    //     comres += bno055_convert_double_euler_p_rad(&d_euler_data_p);
    //     comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
    //     comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);


    //     /*********read linear acceleration converted data**********/
    //     /* variable used to read the linear accel x data output as m/s2*/
    //     double d_linear_accel_datax = BNO055_INIT_VALUE;

    //     /* variable used to read the linear accel y data output as m/s2*/
    //     double d_linear_accel_datay = BNO055_INIT_VALUE;

    //     /* variable used to read the linear accel z data output as m/s2*/
    //     double d_linear_accel_dataz = BNO055_INIT_VALUE;

    //     /* structure used to read the linear accel xyz data output as m/s2*/
    //     struct bno055_linear_accel_double_t d_linear_accel_xyz;

    //     /*  API used to read Linear acceleration data output as m/s2
    //     * float functions also available in the BNO055 API */
    //     comres += bno055_convert_double_linear_accel_x_msq(&d_linear_accel_datax);
    //     comres += bno055_convert_double_linear_accel_y_msq(&d_linear_accel_datay);
    //     comres += bno055_convert_double_linear_accel_z_msq(&d_linear_accel_dataz);
    //     comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);



    //     /********************Gravity converted data**********************/
    //     /* variable used to read the gravity sensor x data output as m/s2*/
    //     double d_gravity_data_x = BNO055_INIT_VALUE;

    //     /* variable used to read the gravity sensor y data output as m/s2*/
    //     double d_gravity_data_y = BNO055_INIT_VALUE;

    //     /* variable used to read the gravity sensor z data output as m/s2*/
    //     double d_gravity_data_z = BNO055_INIT_VALUE;

    //     /* structure used to read the gravity xyz data output as m/s2*/
    //     struct bno055_gravity_double_t d_gravity_xyz;

    //     /*  API used to read Gravity sensor data output as m/s2
    //     * float functions also available in the BNO055 API */
    //     comres += bno055_convert_gravity_double_x_msq(&d_gravity_data_x);
    //     comres += bno055_convert_gravity_double_y_msq(&d_gravity_data_y);
    //     comres += bno055_convert_gravity_double_z_msq(&d_gravity_data_z);
    //     comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
        
    //     return comres;
    // }

    /**
    * \brief Writes data to the device over the I2C bus.
    * 
    * This function serves as a low-level I2C write routine tailored for the BNO055 sensor. 
    * It packages the register address and data to be written into a single buffer and then 
    * dispatches the write operation.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register where the data needs to be written.
    * \param reg_data   : Pointer to an array containing the data bytes to be written to the sensor.
    * \param cnt        : Number of data bytes to write from the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful write operation.
    * \return BNO055_ERROR (-1) if the write operation encounters an error.
    * 
    * \note The actual data writing starts from the second position in the array, as the 
    *       first position is reserved for the register address.
    */
    s8 CImu::BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        s32 BNO055_iERROR = BNO055_INIT_VALUE;
        u8 array[I2C_BUFFER_LEN];
        u8 stringpos = BNO055_INIT_VALUE;

        array[BNO055_INIT_VALUE] = reg_addr;
        for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
        {
            array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
        }

        if (i2c_instance->write(dev_addr, (const char*)array, cnt + 1) == 0)
        {
            BNO055_iERROR = BNO055_SUCCESS; // Return success (0)
        }
        else
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
        }

        return (s8)BNO055_iERROR;
    }

    /**
    * \brief Reads data from the device over the I2C bus.
    * 
    * This function facilitates reading data from a specific register of the BNO055 sensor 
    * over the I2C communication protocol. It sends the desired register address to the sensor, 
    * then reads back the requested amount of data bytes into a provided buffer.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register from which the data needs to be read.
    * \param reg_data   : Pointer to an array where the read data bytes will be stored.
    * \param cnt        : Number of data bytes to read into the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful read operation.
    * \return BNO055_ERROR (-1) if the read operation encounters an error.
    * 
    * \note The function first writes the register address to the sensor to set the pointer 
    *       to the desired location and then initiates the I2C read operation.
    */
    s8 CImu::BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        s32 BNO055_iERROR = BNO055_INIT_VALUE;
        u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
        u8 stringpos = BNO055_INIT_VALUE;

        array[BNO055_INIT_VALUE] = reg_addr;

        /* Please take the below API as your reference
        * for read the data using I2C communication
        * add your I2C read API here.
        * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
        * ARRAY, ARRAY, 1, CNT)"
        * BNO055_iERROR is an return value of SPI write API
        * Please select your valid return value
        * In the driver BNO055_SUCCESS defined as 0
        * and FAILURE defined as -1
        */
        for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
        {
            *(reg_data + stringpos) = array[stringpos];
        }

        // Write the register address to set the pointer for reading
        if (i2c_instance->write(dev_addr, (const char*)&reg_addr, 1) != 0)
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
            return (s8)BNO055_iERROR;
        }

        // Read the data from the specified register address
        if (i2c_instance->read(dev_addr, (char*)reg_data, cnt) == 0)
        {
            BNO055_iERROR = BNO055_SUCCESS; // Return success (0)
        }
        else
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
        }
        return (s8)BNO055_iERROR;
    }
    
    /*-------------------------------------------------------------------------*
     *  By using bno055 the following structure parameter can be accessed
     *  Bus write function pointer: BNO055_WR_FUNC_PTR
     *  Bus read function pointer: BNO055_RD_FUNC_PTR
     *  Delay function pointer: delay_msec
     *  I2C address: dev_addr
     *--------------------------------------------------------------------------*/
    void CImu::I2C_routine(void)
    {
        bno055.bus_write = BNO055_I2C_bus_write;
        bno055.bus_read = BNO055_I2C_bus_read;
        bno055.delay_msec = BNO055_delay_msek;
        bno055.dev_addr = BNO055_I2C_ADDR1 << 1;
        // bno055.dev_addr = BNO055_I2C_ADDR2 << 1;

        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    /**
    * \brief Introduces a delay for the specified duration in milliseconds.
    * 
    * This function relies on the `ThisThread::sleep_for` method to create 
    * a delay, making the current thread sleep for the specified duration.
    * 
    * \param msek The delay duration in milliseconds.
    */
    void CImu::BNO055_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    /** 
    * \brief  Retrieves and processes IMU sensor values.
    * 
    * This method is invoked periodically and handles:
    * 1. Reading Euler angles (roll, pitch, yaw) from the BNO055 sensor.
    * 2. Reading linear acceleration values in the x, y, and z axes from the sensor.
    * 3. Based on the linear acceleration, it updates the current velocity of the device.
    * 4. If the device appears to be stationary (based on x/y/z acceleration thresholds),
    *    a timer is accumulated. If the device remains stationary for a certain duration
    *    (500 ms), the velocity is reset.
    * 5. Outputs the computed values via the out parameter.
    * 
    * \note If there are any issues reading from the BNO055 sensor, the method will exit early and return false.
    */
    bool CImu::sample(ImuSample& out)
    {
        uint64_t now_us = m_timer.elapsed_time().count();
        uint64_t dt_us = now_us - m_lastTime_us;
        if (dt_us == 0) {
            return false;
        }
        if (dt_us > IMU_MAX_DT_US) {
            dt_us = IMU_MAX_DT_US;
        }
        m_lastTime_us = now_us;
        m_delta_time_ms = (uint32_t)(dt_us / 1000);
        
        s8 comres = BNO055_SUCCESS;

        s16 s16_euler_h_raw = BNO055_INIT_VALUE;
        s16 s16_euler_p_raw = BNO055_INIT_VALUE;
        s16 s16_euler_r_raw = BNO055_INIT_VALUE;

        s16 s16_linear_accel_x_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_y_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_z_raw = BNO055_INIT_VALUE;

        s16 s16_gyro_x_raw = BNO055_INIT_VALUE;
        s16 s16_gyro_y_raw = BNO055_INIT_VALUE;
        s16 s16_gyro_z_raw = BNO055_INIT_VALUE;
        struct bno055_mag_t mag_raw;
        struct bno055_quaternion_t quat_raw;

        comres += bno055_read_euler_h(&s16_euler_h_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres += bno055_read_euler_p(&s16_euler_p_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres += bno055_read_euler_r(&s16_euler_r_raw);

        if(comres != BNO055_SUCCESS) return false;

        s32 s16_euler_h_deg = (s16_euler_h_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;
        s32 s16_euler_p_deg = (s16_euler_p_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;
        s32 s16_euler_r_deg = (s16_euler_r_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;

        comres = bno055_read_gyro_x(&s16_gyro_x_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres = bno055_read_gyro_y(&s16_gyro_y_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres = bno055_read_gyro_z(&s16_gyro_z_raw);

        if(comres != BNO055_SUCCESS) return false;

        s32 s16_gyro_x_mrads = (s16_gyro_x_raw * precision_scaling_factor) / BNO055_GYRO_DIV_RPS_int;
        s32 s16_gyro_y_mrads = (s16_gyro_y_raw * precision_scaling_factor) / BNO055_GYRO_DIV_RPS_int;
        s32 s16_gyro_z_mrads = (s16_gyro_z_raw * precision_scaling_factor) / BNO055_GYRO_DIV_RPS_int;

        comres = bno055_read_linear_accel_x(&s16_linear_accel_x_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres = bno055_read_linear_accel_y(&s16_linear_accel_y_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres = bno055_read_linear_accel_z(&s16_linear_accel_z_raw);

        if(comres != BNO055_SUCCESS) return false;

        s32 s16_linear_accel_x_msq = (s16_linear_accel_x_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        s32 s16_linear_accel_y_msq = (s16_linear_accel_y_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        s32 s16_linear_accel_z_msq = (s16_linear_accel_z_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;

        comres = bno055_read_mag_xyz(&mag_raw);

        if(comres != BNO055_SUCCESS) return false;

        comres = bno055_read_quaternion_wxyz(&quat_raw);

        if(comres != BNO055_SUCCESS) return false;

        s32 mag_x_nT = (s32)(((int64_t)mag_raw.x * precision_scaling_factor) / BNO055_MAG_DIV_UT_int);
        s32 mag_y_nT = (s32)(((int64_t)mag_raw.y * precision_scaling_factor) / BNO055_MAG_DIV_UT_int);
        s32 mag_z_nT = (s32)(((int64_t)mag_raw.z * precision_scaling_factor) / BNO055_MAG_DIV_UT_int);

        int64_t quat_norm_sq =
            (int64_t)quat_raw.w * (int64_t)quat_raw.w +
            (int64_t)quat_raw.x * (int64_t)quat_raw.x +
            (int64_t)quat_raw.y * (int64_t)quat_raw.y +
            (int64_t)quat_raw.z * (int64_t)quat_raw.z;
        if (quat_norm_sq <= 0) {
            return false;
        }

        double quat_norm = std::sqrt((double)quat_norm_sq);
        if (quat_norm <= 0.0) {
            return false;
        }

        double qx = (double)quat_raw.x / quat_norm;
        double qy = (double)quat_raw.y / quat_norm;
        double qz = (double)quat_raw.z / quat_norm;
        double qw = (double)quat_raw.w / quat_norm;

        s32 quat_x_milli = (s32)(qx * precision_scaling_factor);
        s32 quat_y_milli = (s32)(qy * precision_scaling_factor);
        s32 quat_z_milli = (s32)(qz * precision_scaling_factor);
        s32 quat_w_milli = (s32)(qw * precision_scaling_factor);

        const int32_t kStationaryThresh = 120; // milli m/s^2
        bool is_stationary =
            (abs(s16_linear_accel_x_msq) <= kStationaryThresh) &&
            (abs(s16_linear_accel_y_msq) <= kStationaryThresh) &&
            (abs(s16_linear_accel_z_msq) <= kStationaryThresh);

        if (is_stationary) {
            // Update bias estimate when stationary (EMA with 1/32 gain)
            m_biasX += (s16_linear_accel_x_msq - m_biasX) / 32;
            m_biasY += (s16_linear_accel_y_msq - m_biasY) / 32;
            m_biasZ += (s16_linear_accel_z_msq - m_biasZ) / 32;
            m_stationaryTime_ms += m_delta_time_ms;

            if (m_stationaryTime_ms >= 500) {
                m_velocityX = 0;
                m_velocityY = 0;
                m_velocityZ = 0;
                m_stationaryTime_ms = 0;
            }
        } else {
            m_stationaryTime_ms = 0;
        }

        // NOTE: Velocity integration disabled on request.
        // s32 ax = s16_linear_accel_x_msq - m_biasX;
        // s32 ay = s16_linear_accel_y_msq - m_biasY;
        // s32 az = s16_linear_accel_z_msq - m_biasZ;
        //
        // m_velocityX += (s32)(((int64_t)ax * (int64_t)dt_us) / 1000000); // Δt in us
        // m_velocityY += (s32)(((int64_t)ay * (int64_t)dt_us) / 1000000);
        // m_velocityZ += (s32)(((int64_t)az * (int64_t)dt_us) / 1000000);

        if(comres != BNO055_SUCCESS) return false;

        out.euler_r_mdeg = s16_euler_r_deg;
        out.euler_p_mdeg = s16_euler_p_deg;
        out.euler_h_mdeg = s16_euler_h_deg;
        out.gyro_x_mrads = s16_gyro_x_mrads;
        out.gyro_y_mrads = s16_gyro_y_mrads;
        out.gyro_z_mrads = s16_gyro_z_mrads;
        out.accel_x_mms2 = s16_linear_accel_x_msq;
        out.accel_y_mms2 = s16_linear_accel_y_msq;
        out.accel_z_mms2 = s16_linear_accel_z_msq;
        out.vel_x_mms = m_velocityX;
        out.vel_y_mms = m_velocityY;
        out.vel_z_mms = m_velocityZ;
        out.mag_x_nT = mag_x_nT;
        out.mag_y_nT = mag_y_nT;
        out.mag_z_nT = mag_z_nT;
        out.quat_w_milli = quat_w_milli;
        out.quat_x_milli = quat_x_milli;
        out.quat_y_milli = quat_y_milli;
        out.quat_z_milli = quat_z_milli;

        return true;
    }

    void CImu::_run()
    {
        if (bool_globalsV_imuenc_isActive) return;
        if (!bool_globalsV_separateSensors_isActive) return;
        if(!m_isActive) return;

        ImuSample sampleData;
        if (!sample(sampleData)) {
            return;
        }

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

        split_milli(sampleData.euler_r_mdeg, &er_sign, &er_int, &er_frac);
        split_milli(sampleData.euler_p_mdeg, &ep_sign, &ep_int, &ep_frac);
        split_milli(sampleData.euler_h_mdeg, &eh_sign, &eh_int, &eh_frac);
        split_milli(sampleData.gyro_x_mrads, &gx_sign, &gx_int, &gx_frac);
        split_milli(sampleData.gyro_y_mrads, &gy_sign, &gy_int, &gy_frac);
        split_milli(sampleData.gyro_z_mrads, &gz_sign, &gz_int, &gz_frac);
        split_milli(sampleData.accel_x_mms2, &ax_sign, &ax_int, &ax_frac);
        split_milli(sampleData.accel_y_mms2, &ay_sign, &ay_int, &ay_frac);
        split_milli(sampleData.accel_z_mms2, &az_sign, &az_int, &az_frac);
        split_milli(sampleData.vel_x_mms, &vx_sign, &vx_int, &vx_frac);
        split_milli(sampleData.vel_y_mms, &vy_sign, &vy_int, &vy_frac);
        split_milli(sampleData.vel_z_mms, &vz_sign, &vz_int, &vz_frac);

        char buffer[IMU_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "@imu:%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;;\r\n",
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
            vz_sign, vz_int, vz_frac);
        m_serial.write(buffer,strlen(buffer));
    }

}; // namespace periodics
