/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2023 Daniel Austin <me@dan.me.uk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* 16-bit commands */
#define	SHT30_CMD_FETCH_DATA	0xe000
#define SHT30_CMD_USE_ART		0x2b32
#define SHT30_CMD_CLEAR_STATUS	0x3041
#define SHT30_CMD_HEATER_OFF	0x3066
#define SHT30_CMD_HEATER_ON		0x306d
#define SHT30_CMD_BREAK			0x3093
#define SHT30_CMD_SOFT_RESET	0x30a2
#define SHT30_CMD_READ_STATUS	0xf32d
	
	typedef struct _sht30_t
	{
		i2c_port_t			i2c_port;		/*!< I2C port */
		uint8_t				addr;			/*!< I2C address (default 0x44) */
		float				_temp;			/*!< (internal) temperature value */
		float				_humidity;		/*!< (internal) humidity value */
		SemaphoreHandle_t	_sem;			/*!< (internal) semaphore */
	} sht30_t;

	/**
	 * @brief Initialise SHT30 Temperature/Humidity Sensor
	 *
	 * @param[in] dev sht30_t object pre-configured
	 * @return
	 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
	 *          - ESP_ERR_NO_MEM        if out of memory when creating semaphore
	 *          - ESP_OK                on success
	 */
	esp_err_t sht30_init(sht30_t *dev);

	/**
	 * @brief Get latest temperature reading
	 * 
	 * @param[in] dev sht30_t object already initialised by sht30_init()
	 * @param[in] update whether to update readings now, or use previous readings
	 * @return temperature degrees celcius (float)
	 */
	float sht30_get_temperature(sht30_t *dev, bool update);

	/**
	 * @brief Get latest humidity reading
	 * 
	 * @param[in] dev sht30_t object already initialised by sht30_init()
	 * @param[in] update whether to update readings now, or use previous readings
	 * @return humidity in percent (float)
	 */
	float sht30_get_humidity(sht30_t *dev, bool update);

#ifdef __cplusplus
}
#endif