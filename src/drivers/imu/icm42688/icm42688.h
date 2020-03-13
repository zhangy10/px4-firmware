/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#include <perf/perf_counter.h>
#include <systemlib/conversions.h>

#include <drivers/drv_hrt.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/conversion/rotation.h>
#include <systemlib/err.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/uORB.h>


#define ICM42688_BANK_0                  (0 << 7)                    /**< Register bank 0 */
#define ICM42688_REG_WHO_AM_I            (ICM42688_BANK_0 | 0x75)    /**< Device ID register */
#define ICM42688_DEVICE_ID               0x47                        /**< ICM42688 Device ID value  */

/* Bank 0 */
#define MPUREG_DEVICE_CONFIG      0x11
#define MPUREG_CHIP_CONFIG        MPUREG_DEVICE_CONFIG // Retro-compatibility
#define MPUREG_DRIVE_CONFIG       0x13
#define MPUREG_INT_CONFIG         0x14
#define MPUREG_FIFO_CONFIG        0x16
#define MPUREG_TEMP_DATA0_UI      0x1D
#define MPUREG_ACCEL_DATA_X0_UI   0x1F
#define MPUREG_GYRO_DATA_X0_UI    0x25
#define MPUREG_TMST_FSYNCH        0x2B
#define MPUREG_TMST_FSYNC1        MPUREG_TMST_FSYNCH // Retro-compatibility
#define MPUREG_INT_STATUS         0x2D
#define MPUREG_FIFO_COUNTH        0x2E
#define MPUREG_FIFO_BYTE_COUNT1   MPUREG_FIFO_COUNTH // Retro-compatibility
#define MPUREG_FIFO_COUNTL        0x2F
#define MPUREG_FIFO_BYTE_COUNT2   MPUREG_FIFO_COUNTL // Retro-compatibility
#define MPUREG_FIFO_DATA          0x30
#define MPUREG_APEX_DATA0         0x31
#define MPUREG_APEX_DATA1         0x32
#define MPUREG_APEX_DATA2         0x33
#define MPUREG_APEX_DATA3         0x34
#define MPUREG_APEX_DATA4         0x35
#define MPUREG_APEX_DATA5         0x36
#define MPUREG_INT_STATUS2        0x37
#define MPUREG_INT_STATUS3        0x38
#define MPUREG_SIGNAL_PATH_RESET  0x4B
#define MPUREG_INTF_CONFIG0       0x4C
#define MPUREG_INTF_CONFIG1       0x4D
#define MPUREG_PWR_MGMT_0         0x4E
#define MPUREG_GYRO_CONFIG0       0x4F
#define MPUREG_ACCEL_CONFIG0      0x50
#define MPUREG_GYRO_CONFIG1       0x51
#define MPUREG_ACCEL_GYRO_CONFIG0 0x52
#define MPUREG_ACCEL_CONFIG1      0x53
#define MPUREG_TMST_CONFIG        0x54
#define MPUREG_APEX_CONFIG0       0x56
#define MPUREG_SMD_CONFIG         0x57
#define MPUREG_FIFO_CONFIG1       0x5F
#define MPUREG_FIFO_CONFIG2       0x60
#define MPUREG_FSYNC_CONFIG       0x62
#define MPUREG_INT_CONFIG0        0x63
#define MPUREG_INT_CONFIG1        0x64
#define MPUREG_INT_SOURCE0        0x65
#define MPUREG_INT_SOURCE1        0x66
#define MPUREG_INT_SOURCE2        0x67
#define MPUREG_INT_SOURCE3        0x68
#define MPUREG_INT_SOURCE4        0x69
#define MPUREG_INT_SOURCE5        0x6A
#define MPUREG_FIFO_LOST_PKT0     0x6C
#define MPUREG_SELF_TEST_CONFIG   0x70
#define MPUREG_WHO_AM_I           0x75
#define MPUREG_REG_BANK_SEL       0x76

/* Bank 1 */
#define MPUREG_SENSOR_CONFIG0_B1      0x03 // Added
#define MPUREG_GYRO_CONFIG_STATIC2_B1 0x0B
#define MPUREG_GYRO_CONFIG_STATIC3_B1 0x0C
#define MPUREG_GYRO_CONFIG_STATIC4_B1 0x0D
#define MPUREG_GYRO_CONFIG_STATIC5_B1 0x0E
#define MPUREG_XG_ST_DATA_B1          0x5F
#define MPUREG_YG_ST_DATA_B1          0x60
#define MPUREG_ZG_ST_DATA_B1          0x61
#define MPUREG_TMST_VAL0_B1           0x62
#define MPUREG_INTF_CONFIG4_B1        0x7A
#define MPUREG_INTF_CONFIG5_B1        0x7B
#define MPUREG_INTF_CONFIG6_B1        0x7C

/* Bank 2 */
#define MPUREG_ACCEL_CONFIG_STATIC2_B2 0x03
#define MPUREG_ACCEL_CONFIG_STATIC3_B2 0x04
#define MPUREG_ACCEL_CONFIG_STATIC4_B2 0x05
#define MPUREG_ACCEL_CONFIG_STATIC0_B2 0x39
#define MPUREG_XA_ST_DATA_B2           0x3B
#define MPUREG_YA_ST_DATA_B2           0x3C
#define MPUREG_ZA_ST_DATA_B2           0x3D
/* Only accessible from AUX1 */
#define MPUREG_OIS1_CONFIG1_B2         0x44
#define MPUREG_OIS1_CONFIG2_B2         0x45
#define MPUREG_OIS1_CONFIG3_B2         0x46
#define MPUREG_ACCEL_DATA_X0_OIS1_B2   0x49
#define MPUREG_GYRO_DATA_X0_OIS1_B2    0x4F
#define MPUREG_INT_STATUS_OIS1_B2      0x57
/* End of Only accessible from AUX1 */
/* Only accessible from AUX2 */
#define MPUREG_OIS2_CONFIG1_B2         0x59
#define MPUREG_OIS2_CONFIG2_B2         0x5A
#define MPUREG_OIS2_CONFIG3_B2         0x5B
#define MPUREG_ACCEL_DATA_X0_OIS2_B2   0x5E
#define MPUREG_GYRO_DATA_X0_OIS2_B2    0x64
#define MPUREG_INT_STATUS_OIS2_B2      0x6C
/* End of Only accessible from AUX2 */


/* Bank 4 */
#define MPUREG_APEX_CONFIG1_B4    0x40
#define MPUREG_APEX_CONFIG2_B4    0x41
#define MPUREG_APEX_CONFIG3_B4    0x42
#define MPUREG_APEX_CONFIG4_B4    0x43
#define MPUREG_APEX_CONFIG5_B4    0x44
#define MPUREG_APEX_CONFIG6_B4    0x45
#define MPUREG_APEX_CONFIG7_B4    0x46
#define MPUREG_APEX_CONFIG8_B4    0x47
#define MPUREG_APEX_CONFIG9_B4    0x48
#define MPUREG_ACCEL_WOM_X_THR_B4 0x4A
#define MPUREG_ACCEL_WOM_Y_THR_B4 0x4B
#define MPUREG_ACCEL_WOM_Z_THR_B4 0x4C
#define MPUREG_INT_SOURCE6_B4     0x4D
#define MPUREG_INT_SOURCE7_B4     0x4E
#define MPUREG_INT_SOURCE8_B4     0x4F
#define MPUREG_INT_SOURCE9_B4     0x50
#define MPUREG_INT_SOURCE10_B4    0x51
#define MPUREG_OFFSET_USER_0_B4   0x77
#define MPUREG_OFFSET_USER_1_B4   0x78
#define MPUREG_OFFSET_USER_2_B4   0x79
#define MPUREG_OFFSET_USER_3_B4   0x7A
#define MPUREG_OFFSET_USER_4_B4   0x7B
#define MPUREG_OFFSET_USER_5_B4   0x7C
#define MPUREG_OFFSET_USER_6_B4   0x7D
#define MPUREG_OFFSET_USER_7_B4   0x7E
#define MPUREG_OFFSET_USER_8_B4   0x7F

#define DEG_TO_RAD(x) ((x) * (M_PI) / 180.0)
#define G_TO_MS2(x)   ((x) * 9.80665)


class IICM42688
{
public:
	virtual ~IICM42688() = default;

	virtual int init() = 0;

	virtual int 	set_reg_bank(uint8_t bank) = 0;
	virtual uint8_t get_reg(uint8_t addr) = 0;
	virtual int	get_reg_bulk(uint8_t addr, void *data, uint8_t len);
	virtual int 	write(uint8_t addr, void *data, uint8_t len) = 0;
	virtual int 	write_reg(uint8_t addr, uint8_t val) = 0;

	virtual uint32_t get_device_id() const = 0;
};

class ICM42688 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	ICM42688(IICM42688 *interface, const char *path);
	virtual ~ICM42688();
	virtual int		init();
	virtual int		sanity_test();

private:
	IICM42688		*_interface;
	void 			Run() override;
	int			write_checked(uint8_t addr, uint8_t val);
};


/* interface factories */
extern IICM42688 *icm42688_spi_interface(uint8_t busnum, uint32_t device);
typedef IICM42688 *(*ICM42688_constructor)(uint8_t, uint32_t);
