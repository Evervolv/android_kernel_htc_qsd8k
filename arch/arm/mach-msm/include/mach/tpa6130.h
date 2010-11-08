/*
 * Definitions for tpa6130a headset amp chip.
 */
#ifndef TPA6130_H
#define TPA6130_H

#include <linux/ioctl.h>

#define TPA6130_I2C_NAME "tpa6130"
void set_headset_amp(int on);

struct tpa6130_platform_data {
	int gpio_hp_sd;
	int enable_rpc_server;
};

struct rpc_headset_amp_ctl_args {
	int on;
};
#endif

