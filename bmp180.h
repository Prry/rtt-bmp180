
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

#include <rtthread.h>
#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif

    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 1, 0))
        #include "drivers/sensor.h"
    #else
        #include "sensor.h"
    #endif
#endif

extern int rt_hw_bmp180_init(const char *name, struct rt_sensor_config *cfg);

#endif /* _BMP180_H_ */
