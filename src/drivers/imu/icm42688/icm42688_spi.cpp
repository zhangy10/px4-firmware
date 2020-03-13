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

/**
 * @file icm42688_spi.cpp
 *
 * SPI interface for ICM42688
 */

#include <drivers/device/spi.h>

#include "icm42688.h"

/* SPI protocol address bits */
#define DIR_READ			0x80
#define DIR_WRITE			0x00

class ICM42688_SPI: public device::SPI, public IICM42688
{
public:
	ICM42688_SPI(uint8_t bus, uint32_t device);
	virtual ~ICM42688_SPI() = default;

	int init();

	int 		set_reg_bank(uint8_t bank);
	uint8_t 	get_reg(uint8_t addr);
	int 		get_reg_bulk(uint8_t addr, void *data, uint8_t len);
	int 		write(uint8_t addr, void *data, uint8_t len);
	int	 	write_reg(uint8_t addr, uint8_t val);

	uint32_t get_device_id() const override { return device::SPI::get_device_id(); }

private:
};

IICM42688 *icm42688_spi_interface(uint8_t busnum, uint32_t device)
{
	return new ICM42688_SPI(busnum, device);
}

ICM42688_SPI::ICM42688_SPI(uint8_t bus, uint32_t device) :
	SPI("ICM42688_SPI", nullptr, bus, device, SPIDEV_MODE3, 24 * 1000 * 1000)
{
}

int ICM42688_SPI::init()
{
	return SPI::init();
}

int ICM42688_SPI::set_reg_bank(uint8_t bank)
{
	return write(MPUREG_REG_BANK_SEL, &bank, 1);
}

uint8_t ICM42688_SPI::get_reg(uint8_t addr)
{
	uint8_t buf[2] = { (uint8_t)(addr | DIR_READ), 0}; //set MSB bit
	transfer(&buf[0], &buf[0], 2);

	return buf[1];
}

int ICM42688_SPI::get_reg_bulk(uint8_t addr, void *data, uint8_t len)
{
	uint8_t buf[32] = {};

	if (32 < len + 1) {
		return -EIO;
	}

	buf[0] = (addr | DIR_READ);
	transfer(&buf[0], &buf[0], len);
	memcpy(data, &buf[1], len);

	return 1;
}

int ICM42688_SPI::write(uint8_t addr, void *data, uint8_t len)
{
	uint8_t buf[32];

	if (32 < len + 1) {
		return -EIO;
	}

	buf[0] = (addr | DIR_WRITE);
	buf[1] = *(uint8_t *)data;

	return transfer(&buf[0], &buf[0], len + 1);
}

int ICM42688_SPI::write_reg(uint8_t addr, uint8_t val)
{
	uint8_t buf[2];

	buf[0] = (addr | DIR_WRITE);
	buf[1] = val;

	return transfer(&buf[0], &buf[0], 2);
}
