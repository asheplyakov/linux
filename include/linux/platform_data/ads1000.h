// SPDX-License-Identifier: GPL-2.0
/*
 * Platform Data for ADS1000/ADS1100 12-16-bit ADC
 *
 * Copyright (C) 2018 T-platforms JSC (fancer.lancer@gmail.com)
 */

#ifndef LINUX_ADS1000_H
#define LINUX_ADS1000_H

#include <linux/regulator/consumer.h>

struct ads1000_platform_data {
	unsigned int pga;
	unsigned int data_rate;
	struct regulator *vdd;
	unsigned int divider[2];
};

#endif /* LINUX_ADS1000_H */
