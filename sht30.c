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

#include "esp_log.h"
#include "sht30.h"

static const char *TAG = "sht30";

static esp_err_t _sht30_write_command(sht30_t *dev, uint16_t cmd)
{
	uint8_t		data[2];

	data[0] = cmd >> 8;
	data[1] = (uint8_t)cmd;
	return i2c_master_write_to_device(dev->i2c_port, dev->addr, data, 2, pdMS_TO_TICKS(1000));
}

static uint8_t _sht30_crc8(uint8_t *data, int len)
{
	const uint8_t	POLYNOMIAL = 0x31;
	uint8_t			crc = 0xFF;
	int				i, j;
	
	for (i = 0; i < len; ++i)
	{
		crc ^= *data++;
		for (j = 0; j < 8; ++j)
			crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
	}
	return crc;
}

static uint16_t _sht30_read_reg16(sht30_t *dev, uint16_t reg)
{
	esp_err_t	ret;
	uint8_t		data[3];
	uint16_t	val;
	int			i;

	/* 3 retries */
	for (i = 0; i < 3; i++)
	{
		data[0] = reg >> 8;
		data[1] = (uint8_t)reg;
		ret = i2c_master_write_read_device(dev->i2c_port, dev->addr, data, 2, data, 3, pdMS_TO_TICKS(1000));
		if (ret == ESP_OK)
		{
			val = data[0] << 8;
			val += data[1];
			if (_sht30_crc8(data, 2) == data[2])
				return val;
			/* else CRC error */
			ESP_LOGE(TAG, "Invalid CRC received (0x%02x != 0x%02x)", data[2], _sht30_crc8(data, 2));
		}
		/* retry in 10ms either due to failed read, or invalid CRC */
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	return 0;
}

static void _sht30_send_break(sht30_t *dev)
{
	_sht30_write_command(dev, SHT30_CMD_BREAK);
	vTaskDelay(pdMS_TO_TICKS(150));
}

static void _sht30_soft_reset(sht30_t *dev)
{
	_sht30_send_break(dev);
	_sht30_write_command(dev, SHT30_CMD_SOFT_RESET);
	vTaskDelay(pdMS_TO_TICKS(150));
}

void sht30_heater_enable(sht30_t *dev, bool enabled)
{
	if (!dev)
		return;
	xSemaphoreTake(dev->_sem, portMAX_DELAY);
	_sht30_write_command(dev, (enabled ? SHT30_CMD_HEATER_ON : SHT30_CMD_HEATER_OFF));
	xSemaphoreGive(dev->_sem);
}

void sht30_clear_status(sht30_t *dev)
{
	if (!dev)
		return;
	xSemaphoreTake(dev->_sem, portMAX_DELAY);
	_sht30_write_command(dev, SHT30_CMD_CLEAR_STATUS);
	xSemaphoreGive(dev->_sem);
	vTaskDelay(pdMS_TO_TICKS(50));
}

static void _sht30_update(sht30_t *dev)
{
	esp_err_t	ret;
	uint8_t		data[6];
	uint16_t	val;

	xSemaphoreTake(dev->_sem, portMAX_DELAY);
	data[0] = SHT30_CMD_FETCH_DATA >> 8;
	data[1] = SHT30_CMD_FETCH_DATA & 0xff;
	ret = i2c_master_write_read_device(dev->i2c_port, dev->addr, data, 2, data, 6, pdMS_TO_TICKS(1000));
	if (ret == ESP_OK)
	{
		/* if all 0xff, no data available (probably didn't wait long enough) */
		if (data[0] == 0xff && data[1] == 0xff && data[2] == 0xff &&
		    data[3] == 0xff && data[4] == 0xff && data[5] == 0xff)
		{
			xSemaphoreGive(dev->_sem);
			return;			
		}
		if (_sht30_crc8(data, 2) != data[2])
		{
			ESP_LOGE(TAG, "CRC mismatch on temperature reading");
		} else {
			val = data[0] << 8;
			val += data[1];
			dev->_temp = -45.0f + 175.0f * ((float)val / 65535.0f);
		}
		data[0] = data[3];
		data[1] = data[4];
		if (_sht30_crc8(data, 2) != data[5])
		{
			ESP_LOGE(TAG, "CRC mismatch on humidity reading");
		} else {
			val = data[0] << 8;
			val += data[1];
			dev->_humidity = 100.0f * ((float)val / 65535.0f);
		}
	}
	xSemaphoreGive(dev->_sem);
	return;
}

esp_err_t sht30_init(sht30_t *dev)
{
	uint16_t	val;

	if (!dev)
	{
		ESP_LOGE(TAG, "Unable to initialise (dev is NULL)");
		return ESP_ERR_INVALID_ARG;
	}
	
	if (dev->i2c_port < 0 || dev->i2c_port >= I2C_NUM_MAX)
	{
		ESP_LOGE("sht30", "Unable to initialise (I2C port invalid)");
		return ESP_ERR_INVALID_ARG;
	}

	if (dev->addr > 0x7F)
	{
		ESP_LOGE(TAG, "Unable to initialise (addr provided invalid)");
		return ESP_ERR_INVALID_ARG;
	}

	if (dev->addr == 0x00)
		dev->addr = 0x44;

	dev->_sem = xSemaphoreCreateMutex();
	if (!dev->_sem)
	{
		ESP_LOGE(TAG, "Unable to initialise (unable to create mutex)");
		return ESP_ERR_NO_MEM;
	}
	
	ESP_LOGI(TAG, "Initialising SHT30 at 0x%02x", dev->addr);
	_sht30_soft_reset(dev);
	/* read status register */
	val = _sht30_read_reg16(dev, SHT30_CMD_READ_STATUS);
	if (val == 0)
	{
		ESP_LOGE(TAG, "Unable to initialise (SHT30 not found or unexpected response from status register)");
		return ESP_ERR_NOT_FOUND;
	}
	sht30_clear_status(dev);
	_sht30_write_command(dev, 0x2130);		/* high repeatability, 1 update per second */
	vTaskDelay(pdMS_TO_TICKS(50));
	ESP_LOGI(TAG, "Initialised SHT30");
	return ESP_OK;
}

float sht30_get_temperature(sht30_t *dev, bool update)
{
	if (!dev)
		return 0.0f;
	if (update)
		_sht30_update(dev);
	return dev->_temp;
}

float sht30_get_humidity(sht30_t *dev, bool update)
{
	if (!dev)
		return 0.0f;
	if (update)
		_sht30_update(dev);
	return dev->_humidity;
}