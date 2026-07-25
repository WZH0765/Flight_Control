/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "inv_imu_transport.h"
#include "inv_imu_defs.h"

/* Static function definition */

/* For Direct access REGisters */
static int write_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, const uint8_t *buf);
static int read_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, uint8_t *buf);

/* For Mclk REGisters */
static int check_out_of_bounds_mreg(uint32_t reg, uint32_t len);
static int write_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, const uint8_t *buf);
static int read_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, uint8_t *buf);

int inv_imu_read_reg(void *t, uint32_t reg, uint32_t len, uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	if (reg > 0xFF)
		return read_mreg(tr, reg & 0xFFFF, len, buf);
	else
		return read_dreg(tr, reg, len, buf);
}

int inv_imu_write_reg(void *t, uint32_t reg, uint32_t len, const uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	if (reg > 0xFF)
		return write_mreg(tr, reg, len, buf);
	else
		return write_dreg(tr, reg, len, buf);
}

int inv_imu_read_sram(void *t, uint32_t addr, uint32_t len, uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	return read_mreg(tr, addr, len, buf);
}

int inv_imu_write_sram(void *t, uint32_t addr, uint32_t len, const uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	return write_mreg(tr, addr, len, buf);
}

/*
 * Static functions implementation 
 */

static int read_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, uint8_t *buf)
{
	if (t->read_reg(reg, buf, len) != 0)
		return INV_IMU_ERROR_TRANSPORT;

	return INV_IMU_OK;
}

static int write_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, const uint8_t *buf)
{
	if (t->write_reg(reg, buf, len) != 0)
		return INV_IMU_ERROR_TRANSPORT;

	return INV_IMU_OK;
}

static int check_out_of_bounds_mreg(uint32_t reg, uint32_t len)
{
	uint32_t min_addr = reg;
	uint32_t max_addr = reg + len - 1;

	/* AN-000364
	 * Users must not access the following register map address space to prevent stalling the device.
	 * - From 0x000023FF to 0x00003FFF
	 * - From 0x000083FF to 0x00009FFF
	 * - From 0x0000AFFF to 0xFFFFFFFF
	 * If user happens to access this space, soft reset is needed after the access to recover from stall.
	 */
	if (((min_addr > 0x000023FF) && (min_addr <= 0x00003FFF)) ||
	    ((max_addr > 0x000023FF) && (max_addr <= 0x00003FFF)) ||
	    ((min_addr <= 0x000023FF) && (max_addr > 0x00003FFF)))
		return INV_IMU_ERROR_TRANSPORT;
	if (((min_addr > 0x000083FF) && (min_addr <= 0x00009FFF)) ||
	    ((max_addr > 0x000083FF) && (max_addr <= 0x00009FFF)) ||
	    ((min_addr <= 0x000083FF) && (max_addr > 0x00009FFF)))
		return INV_IMU_ERROR_TRANSPORT;
	if (max_addr > 0x0000AFFF)
		return INV_IMU_ERROR_TRANSPORT;

	return INV_IMU_OK;
}

static int read_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, uint8_t *buf)
{
	int     status = INV_IMU_OK;
	uint8_t data[2];

	status |= check_out_of_bounds_mreg(reg, len);
	if (status)
		return status;

	/* Write address first */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	t->sleep_us(4);
	status |= write_dreg(t, IREG_ADDR_15_8, 2, data);

	if (status)
		return status;

	/* Read all bytes one by one */
	for (uint32_t i = 0; i < len; i++) {
		t->sleep_us(4);
		status |= read_dreg(t, IREG_DATA, 1, &buf[i]);
	}

	return status;
}

static int write_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, const uint8_t *buf)
{
	int     status = INV_IMU_OK;
	uint8_t data[3];

	status |= check_out_of_bounds_mreg(reg, len);
	if (status)
		return status;

	/* First two bytes are the address where we want to write */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	/* 3rd byte is the first data to write*/
	data[2] = buf[0];

	/* Burst write address and first byte */
	t->sleep_us(4);
	status |= write_dreg(t, IREG_ADDR_15_8, 3, data);
	t->sleep_us(4);

	if (status)
		return status;

	/* Loop on the remaining bytes */
	for (uint32_t i = 1; i < len; i++) {
		status |= write_dreg(t, IREG_DATA, 1, &buf[i]);
		t->sleep_us(4);
	}

	return status;
}
