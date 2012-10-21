/* board-htcleo-ts.h
 *
 * Copyright (C) 2010 Cotulla
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef HTCLEO_TS_H
#define HTCLEO_TS_H

#include <linux/types.h>

#define LEO_TOUCH_DRV_NAME  "leo_touch_name"

struct htcleo_ts_i2c_platform_data
{
    uint16_t version;
    int abs_x_min;
    int abs_x_max;
    int abs_y_min;
    int abs_y_max;
    int abs_pressure_min;
    int abs_pressure_max;
    int abs_width_min;
    int abs_width_max;
};

#endif // HTCLEO_TS_H


