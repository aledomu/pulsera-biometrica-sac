/** @file
 *  @brief HTS Service sample
 */

/*
 * Copyright (c) 2020 SixOctets Systems
 * Copyright (c) 2019 Aaron Tsui <aaron.tsui@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/hrs.h>

#include "hds.h"

#include "getData.h"
#include "heartRate.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"

static const struct i2c_dt_spec i2cSpec = I2C_DT_SPEC_GET(DT_NODELABEL(max30105));

static MAX30105 sensor = { .i2cDriver = &i2cSpec };

const float calibrate_temp = 1.2;

int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

static uint8_t simulate_htm;
static uint8_t indicating;

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	simulate_htm = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	printk("Indication complete\n");
	indicating = 0U;
}

/* Health Thermometer Service Declaration */
BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* more optional Characteristics */
);

void bt_hts_notify(double temperature) {
	uint32_t mantissa = (uint32_t)(temperature * 100);
	uint8_t exponent = (uint8_t)-2;

	uint8_t htm[5];
	htm[0] = 0; /* temperature in celsius */
	sys_put_le24(mantissa, (uint8_t *)&htm[1]);
	htm[4] = exponent;

	struct bt_gatt_indicate_params ind_params = {
		.attr = &hts_svc.attrs[2],
		.func = indicate_cb,
		.destroy = indicate_destroy,
		.data = &htm,
		.len = sizeof(htm)
	};

	if (bt_gatt_indicate(NULL, &ind_params) == 0) {
		indicating = 1U;
	}
}

void hds_init(void)
{
	MAX30105_init(&sensor);

	byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  	byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  	byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  	byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  	int pulseWidth = 411; //Options: 69, 118, 215, 411
  	int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
	MAX30105_setup(&sensor, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

	MAX30105_enableDIETEMPRDY(&sensor); //Enable the temp ready interrupt. This is required.

	getSpo2AndHeartRateInit(&sensor, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void hds_indicate(void)
{
	uint32_t ir = MAX30105_getIR(&sensor);
	getSpo2AndHeartRateContinuous(&sensor, &spo2, &validSPO2, &heartRate, &validHeartRate);
	float temp = MAX30105_readTemperature(&sensor) + calibrate_temp;

	printk(
		"temperature=%.2f ºC, BPM=%d, validBPM=%d, SpO2=%d, validSpO2=%d",
		temp,
		heartRate,
		validHeartRate,
		spo2,
		validSPO2
	);

	if (ir < 50000) printk(" No finger?");

	bt_hts_notify((double) temp);
	if (validHeartRate)	bt_hrs_notify((uint16_t) heartRate);

  	printk("\r\n");
}
