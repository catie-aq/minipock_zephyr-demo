/*
 * Copyright (c) 2025, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_PAA5160E1_PAA5160E1_GLOBAL_H_
#define ZEPHYR_DRIVERS_SENSOR_PAA5160E1_PAA5160E1_GLOBAL_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>

enum paa5160e1_sensor_channel {

	PAA5160E1_SENSOR_CHAN_X = SENSOR_CHAN_PRIV_START + 0,
	PAA5160E1_SENSOR_CHAN_Y = SENSOR_CHAN_PRIV_START + 1,
	PAA5160E1_SENSOR_CHAN_H = SENSOR_CHAN_PRIV_START + 2,

	PAA5160E1_SENSOR_CHAN_VEL_X = SENSOR_CHAN_PRIV_START + 3,
	PAA5160E1_SENSOR_CHAN_VEL_Y = SENSOR_CHAN_PRIV_START + 4,
	PAA5160E1_SENSOR_CHAN_VEL_H = SENSOR_CHAN_PRIV_START + 5,

	PAA5160E1_SENSOR_CHAN_ACC_X = SENSOR_CHAN_PRIV_START + 6,
	PAA5160E1_SENSOR_CHAN_ACC_Y = SENSOR_CHAN_PRIV_START + 7,
	PAA5160E1_SENSOR_CHAN_ACC_H = SENSOR_CHAN_PRIV_START + 8,

	PAA5160E1_SENSOR_CHAN_STD_X = SENSOR_CHAN_PRIV_START + 9,
	PAA5160E1_SENSOR_CHAN_STD_Y = SENSOR_CHAN_PRIV_START + 10,
	PAA5160E1_SENSOR_CHAN_STD_H = SENSOR_CHAN_PRIV_START + 11,

	PAA5160E1_SENSOR_CHAN_STD_VEL_X = SENSOR_CHAN_PRIV_START + 12,
	PAA5160E1_SENSOR_CHAN_STD_VEL_Y = SENSOR_CHAN_PRIV_START + 13,
	PAA5160E1_SENSOR_CHAN_STD_VEL_H = SENSOR_CHAN_PRIV_START + 14,

	PAA5160E1_SENSOR_CHAN_STD_ACC_X = SENSOR_CHAN_PRIV_START + 15,
	PAA5160E1_SENSOR_CHAN_STD_ACC_Y = SENSOR_CHAN_PRIV_START + 16,
	PAA5160E1_SENSOR_CHAN_STD_ACC_H = SENSOR_CHAN_PRIV_START + 17,
};

struct paa5160e1_data {
	uint32_t offsetX; // Offset X
	uint32_t offsetY; // Offset Y
	uint32_t offsetH; // Offset Heading angle

	uint16_t prev_positionX; // Previous Position X
	uint16_t prev_positionY; // Previous Position Y
	uint16_t prev_positionH; // Previous Position Heading angle

	uint16_t positionX; // Position X from 0 to 10000
	uint16_t positionY; // Position Y from 0 to 10000
	uint16_t positionH; // Heading angle from 0 to 36000

	uint32_t absolute_positionX; // Absolute Position X in mm
	uint32_t absolute_positionY; // Absolute Position Y in mm
	uint32_t absolute_positionH; // Absolute Position Heading angle in deg

	uint32_t velocityX; // Velocity X in mm/s
	uint32_t velocityY; // Velocity Y in mm/s
	uint32_t velocityH; // Velocity Heading in deg/s

	uint32_t accelerationX; // Acceleration X in mm/s^2
	uint32_t accelerationY; // Acceleration Y in mm/s^2
	uint32_t accelerationH; // Acceleration Heading angle

	uint16_t std_positionX; // Standard deviation of position X
	uint16_t std_positionY; // Standard deviation of position Y
	uint16_t std_positionH; // Standard deviation of heading angle

	uint16_t std_velocityX; // Standard deviation of velocity X
	uint16_t std_velocityY; // Standard deviation of velocity Y
	uint16_t std_velocityH; // Standard deviation of velocity Heading angle

	uint16_t std_accelerationX; // Standard deviation of acceleration X
	uint16_t std_accelerationY; // Standard deviation of acceleration Y
	uint16_t std_accelerationH; // Standard deviation of acceleration Heading angle
};

#endif /* ZEPHYR_DRIVERS_SENSOR_PAA5160E1_PAA5160E1_GLOBAL_H_ */
