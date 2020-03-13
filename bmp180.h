
/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-bmp180>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-12     panrui      the first version
 */

#ifndef _BMP180_H_
#define _BMP180_H_

#include "sensor.h"

extern int rt_hw_bmp180_init(const char *name, struct rt_sensor_config *cfg);

#endif /* _BMP180_H_ */
