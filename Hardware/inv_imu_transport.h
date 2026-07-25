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

/** @defgroup Transport Transport
 *  @brief    Abstraction layer to communicate with device.
 *  @{
 */

/** @file inv_imu_transport.h */

#ifndef _INV_IMU_TRANSPORT_H_
#define _INV_IMU_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Function pointer to read register(s).
 *  @param[in] reg   Register address to be read.
 *  @param[out] buf  Output data from the register.
 *  @param[in] len   Number of byte to be read.
 *  @return          0 on success, negative value on error.
 */
typedef int (*inv_imu_read_reg_t)(uint8_t reg, uint8_t *buf, uint32_t len);

/** @brief Function pointer to write register(s).
 *  @param[in] reg  Register address to be written.
 *  @param[in] buf  Input data to write.
 *  @param[in] len  Number of byte to be written.
 *  @return         0 on success, negative value on error.
 */
typedef int (*inv_imu_write_reg_t)(uint8_t reg, const uint8_t *buf, uint32_t len);

/* Available serial interface type. */
#define UI_I2C  0 /**< identifies I2C interface. */
#define UI_SPI4 1 /**< identifies 4-wire SPI interface. */
#define UI_SPI3 2 /**< identifies 3-wire SPI interface. */

/** @brief Serif type definition.
 *  @deprecated Kept for retrocompatibility. Replaced with `uint32_t` type
 *              in `inv_imu_transport_t` struct.
 */
typedef uint32_t inv_imu_serif_type_t;

/** @brief Structure dedicated to transport layer transport interface. */
typedef struct {
	/* Serial interface variables (should be initialized by application) */
	inv_imu_read_reg_t  read_reg; /**< Function pointer to read register(s). */
	inv_imu_write_reg_t write_reg; /**< Function pointer to write register(s). */
	uint32_t            serif_type; /**< Serial interface type. */

	/** @brief Callback to sleep function.
	 *  @param[in] us  Time to sleep in microseconds.
	 */
	void (*sleep_us)(uint32_t us);
} inv_imu_transport_t;

/** @brief Reads data from a register on IMU.
 *  @param[in] t     Pointer to transport (as void * so it can be called from any module).
 *  @param[in] reg   Register address to be read.
 *  @param[in] len   Number of byte to be read.
 *  @param[out] buf  Output data from the register.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_read_reg(void *t, uint32_t reg, uint32_t len, uint8_t *buf);

/** @brief Writes data to a register on IMU.
 *  @param[in] t    Pointer to transport (as void * so it can be called from any module).
 *  @param[in] reg  Register address to be written.
 *  @param[in] len  Number of byte to be written.
 *  @param[in] buf  Input data to write.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_write_reg(void *t, uint32_t reg, uint32_t len, const uint8_t *buf);

/** @brief Reads data from SRAM on IMU.
 *  @param[in] t     Pointer to transport (as void * so it can be called from any module). 
 *  @param[in] addr  Address to be read.
 *  @param[in] len   Number of byte to be read.
 *  @param[out] buf  Output data from the register.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_read_sram(void *t, uint32_t addr, uint32_t len, uint8_t *buf);

/** @brief Writes data to SRAM on IMU.
 *  @param[in] t     Pointer to transport (as void * so it can be called from any module).
 *  @param[in] addr  Address to be written.
 *  @param[in] len   Number of byte to be written.
 *  @param[in] buf   Input data to write.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_write_sram(void *t, uint32_t addr, uint32_t len, const uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_TRANSPORT_H_ */

/** @} */
