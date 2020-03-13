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

#include "icm42688.h"

ICM42688::ICM42688(IICM42688 *interface, const char *path) :
	CDev(path),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface)
{
}

ICM42688::~ICM42688()
{
	delete _interface;
}

int
ICM42688::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	if (_interface->set_reg_bank(0) < 0) {
		PX4_ERR("Failed to set bank 0");
	}

	uint8_t who_am_i = _interface->get_reg(ICM42688_REG_WHO_AM_I);

	if (who_am_i != ICM42688_DEVICE_ID) {
		PX4_WARN("id of your accel is not: 0x%02x", ICM42688_DEVICE_ID);
		return -EIO;

	} else {
		PX4_INFO("detected ICM42688!");
	}

	/*
	 * Reset
	 */
	uint8_t SOFT_RESET_CONFIG = (1 << 0);
	uint8_t DEVICE_CONFIG_REG = SOFT_RESET_CONFIG;
	_interface->write_reg(MPUREG_DEVICE_CONFIG, DEVICE_CONFIG_REG);
	px4_usleep(10000);

	uint8_t INT_ASYNC_RESET = (0 << 4);
	uint8_t INT_CONFIG_REG = INT_ASYNC_RESET;
	_interface->write_reg(MPUREG_INT_CONFIG1, INT_CONFIG_REG);
	px4_usleep(10000);

	uint8_t ABORT_AND_RESET  = (1 << 3);
	uint8_t SIGNAL_PATH_RESET_REG = ABORT_AND_RESET;
	_interface->write_reg(MPUREG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_REG);
	px4_usleep(100000);



	if (_interface->set_reg_bank(1) < 0) {
		PX4_ERR("Failed to set bank 1");
	}

	px4_usleep(1000);

	uint8_t XA_DISABLE = 0 << 0;
	uint8_t YA_DISABLE = 0 << 1;
	uint8_t ZA_DISABLE = 0 << 2;
	uint8_t XG_DISABLE = 0 << 3;
	uint8_t YG_DISABLE = 0 << 4;
	uint8_t ZG_DISABLE = 0 << 5;

	uint8_t SENSOR_CONFIG0_B1_REG = XA_DISABLE |  YA_DISABLE | ZA_DISABLE | XG_DISABLE | YG_DISABLE | ZG_DISABLE;
	_interface->write_reg(MPUREG_SENSOR_CONFIG0_B1, SENSOR_CONFIG0_B1_REG);
	px4_usleep(50000);

	uint8_t GYRO_AAF_DIS = 1 << 1; // Disable gyro Anti-aliasing filter
	uint8_t GYRO_NF_DIS  = 1 << 0; // Disable gyro Notch filter
	uint8_t GYRO_CONFIG_STATIC2_B1_REG = GYRO_AAF_DIS | GYRO_NF_DIS;
	_interface->write_reg(MPUREG_GYRO_CONFIG_STATIC2_B1, GYRO_CONFIG_STATIC2_B1_REG);
	px4_usleep(10000);



	if (_interface->set_reg_bank(0) < 0) {
		PX4_ERR("Failed to set bank 0");
	}

	px4_usleep(1000);

	uint8_t UI_SIFS_CFG = (3 << 0);        // disable I2C
	//uint8_t SENSOR_DATA_ENDIAN = (1 << 4); // Big Endian
	uint8_t INTF_CONFIG0_REG = _interface->get_reg(MPUREG_INTF_CONFIG0);
	INTF_CONFIG0_REG |= UI_SIFS_CFG;
	_interface->write_reg(MPUREG_INTF_CONFIG0, INTF_CONFIG0_REG);
	px4_usleep(10000);

	// GYRO_CONFIG
	uint8_t GYRO_FS_SEL = 0 << 5;   // 0 --> 2000 dps
	uint8_t GYRO_ODR    = 6 << 0;   // 6 --> 1000Hz
	uint8_t GYRO_CONFIG0_REG = GYRO_FS_SEL | GYRO_ODR;

	if (write_checked(MPUREG_GYRO_CONFIG0, GYRO_CONFIG0_REG) < 0) {
		PX4_ERR("Failed to set MPUREG_GYRO_CONFIG0");
	}

	px4_usleep(10000);


	// ACCEL_CONFIG0
	// 16g, 1kHz
	uint8_t ACCEL_FS_SEL = 0 << 5;  // 0 --> 16g
	uint8_t ACCEL_ODR    = 6 << 0;  // 6 --> 1000Hz
	uint8_t ACCEL_CONFIG0_REG = ACCEL_FS_SEL | ACCEL_ODR;

	if (write_checked(MPUREG_ACCEL_CONFIG0, ACCEL_CONFIG0_REG) < 0) {
		PX4_ERR("Failed to set MPUREG_ACCEL_CONFIG0");
	}

	px4_usleep(10000);

	// disable FIFO
	if (write_checked(MPUREG_FIFO_CONFIG, 0) < 0) {
		PX4_ERR("Failed to set MPUREG_FIFO_CONFIG");
	}

	px4_usleep(10000);

	uint8_t ACCEL_UI_FILT_BW = 0; // Bandwidth = ODR / 2
	uint8_t GYRO_UI_FILT_BW  = 0; // Bandwidth = ODR / 2
	uint8_t ACCEL_GYRO_CONFIG0_REG = ACCEL_UI_FILT_BW | GYRO_UI_FILT_BW;

	if (write_checked(MPUREG_ACCEL_GYRO_CONFIG0, ACCEL_GYRO_CONFIG0_REG) < 0) {
		PX4_ERR("Failed to set MPUREG_ACCEL_GYRO_CONFIG0");
	}

	px4_usleep(10000);

	// MPUREG_PWR_MGMT_0
	uint8_t ACCEL_MODE     = (3 << 0); // Low Noise mode
	uint8_t GYRO_MODE      = (3 << 2); // Low Noise Mode
	uint8_t IDLE           = (0 << 4); // Turn off RC osc when accel and gyro turned off
	uint8_t TEMP_DIS       = (0 << 5); // Enable temperature
	uint8_t PWR_MGMT_0_REG = ACCEL_MODE | GYRO_MODE | IDLE | TEMP_DIS;

	if (write_checked(MPUREG_PWR_MGMT_0, PWR_MGMT_0_REG) < 0) {
		PX4_ERR("Failed to set MPUREG_PWR_MGMT_0");
	}

	px4_usleep(10000);

	return PX4_OK;
}

int
ICM42688::write_checked(uint8_t addr, uint8_t val)
{
	_interface->write_reg(addr, val);
	px4_usleep(10000);
	uint8_t check = _interface->get_reg(addr);

	if (val != check) {
		return -1;
	}

	return 0;
}

int
ICM42688::sanity_test()
{
	/* Get Temperature Data
	 */
	_interface->set_reg_bank(0);

	uint8_t temp_h = _interface->get_reg(MPUREG_TEMP_DATA0_UI);
	uint8_t temp_l = _interface->get_reg(MPUREG_TEMP_DATA0_UI + 1);
	uint16_t temp_16 = (temp_h << 8) + temp_l;
	double temp_c = ((double)temp_16 / 132.48) + 25.0;
	PX4_INFO("Temperature: %f", temp_c);

	uint8_t buf[12];
	memset(buf, 0x00, 12);
	_interface->get_reg_bulk(MPUREG_ACCEL_DATA_X0_UI, buf, 12);

	uint16_t accel_ux_16 = (buf[0] << 8) + buf[1];
	uint16_t accel_uy_16 = (buf[2] << 8) + buf[3];
	uint16_t accel_uz_16 = (buf[4] << 8) + buf[5];

	if (buf[0] == 0x80 || buf[2] == 0x80 || buf[4] == 0x80) {
		PX4_ERR("Read reset values from accel data registers");
		//return -1;
	}

	int16_t accel_x_16 = 0;
	memcpy(&accel_x_16, &accel_ux_16, sizeof(int16_t));
	int16_t accel_y_16 = 0;
	memcpy(&accel_y_16, &accel_uy_16, sizeof(int16_t));
	int16_t accel_z_16 = 0;
	memcpy(&accel_z_16, &accel_uz_16, sizeof(int16_t));

	double accel_g_per_bit = ((32.0) / (double)0xFFFF);

	double accel_x = ((double)(accel_x_16) * accel_g_per_bit);
	double accel_y = ((double)(accel_y_16) * accel_g_per_bit);
	double accel_z = ((double)(accel_z_16) * accel_g_per_bit);

	PX4_INFO("Accel X: %f", accel_x);
	PX4_INFO("Accel Y: %f", accel_y);
	PX4_INFO("Accel Z: %f", accel_z);

	uint16_t gyro_ux_16 = (buf[6] << 8) + buf[7];
	uint16_t gyro_uy_16 = (buf[8] << 8) + buf[9];
	uint16_t gyro_uz_16 = (buf[10] << 8) + buf[11];

	if (buf[6] == 0x80 || buf[8] == 0x80 || buf[0] == 0x80) {
		PX4_ERR("Read reset values from gryo data registers");
		//return -1;
	}

	int16_t gyro_x_16 = 0;
	memcpy(&gyro_x_16, &gyro_ux_16, sizeof(int16_t));
	int16_t gyro_y_16 = 0;
	memcpy(&gyro_y_16, &gyro_uy_16, sizeof(int16_t));
	int16_t gyro_z_16 = 0;
	memcpy(&gyro_z_16, &gyro_uz_16, sizeof(int16_t));

	if (gyro_ux_16 == 0xF000 || gyro_uy_16 == 0xF000 || gyro_uz_16 == 0xF000) {
		PX4_ERR("Read 0 from gyro data registers");
		//return -1;
	}

	double gyro_dps_per_bit = ((4000.0) / (double)0xFFFF);

	double gyro_x = ((double)(gyro_x_16) * gyro_dps_per_bit);
	double gyro_y = ((double)(gyro_y_16) * gyro_dps_per_bit);
	double gyro_z = ((double)(gyro_z_16) * gyro_dps_per_bit);

	PX4_INFO("Gyro X: %f", gyro_x);
	PX4_INFO("Gyro Y: %f", gyro_y);
	PX4_INFO("Gyro Z: %f", gyro_z);

	return PX4_OK;
}

/*
 * ScheduledWorkItem override
 */
void
ICM42688::Run()
{
	// TODO
}
