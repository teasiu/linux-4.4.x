#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Synology NAS Board GPIO Setup
 *
 * Maintained by:  Comsumer Platform Team <cpt@synology.com>
 *
 * Copyright 2009-2015 Synology, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/synobios.h>
#include <linux/syno_gpio.h>

#ifdef MY_DEF_HERE
#include <linux/synolib.h>
#include <linux/of.h>
#include <linux/string.h>
#endif /* MY_DEF_HERE */

SYNO_GPIO syno_gpio = {
	.fan_ctrl =NULL,
	.fan_fail =NULL,
	.hdd_fail_led =NULL,
	.hdd_present_led =NULL,
	.hdd_act_led =NULL,
	.hdd_detect =NULL,
	.hdd_enable =NULL,
	.model_id =NULL,
	.alarm_led =NULL,
	.power_led =NULL,
	.disk_led_ctrl =NULL,
	.phy_led_ctrl =NULL,
	.copy_button_detect =NULL,
	.redundant_power_detect =NULL,
};
EXPORT_SYMBOL(syno_gpio);

#ifdef MY_ABC_HERE
extern int giSynoSpinupGroupDebug;
#endif /* MY_ABC_HERE */

void syno_gpio_direction_output(int pin, int pValue)
{
	int iErr = 0;
	iErr = gpio_request(pin, NULL);
	if (iErr) {
		printk("%s:%s(%d) gpio_request pin %d fail!\n", __FILE__, __FUNCTION__, __LINE__, pin);
		goto END;
	}
	iErr = gpio_direction_output(pin, pValue);
	if (iErr) {
		printk("%s:%s(%d) set gpio pin %d value %d fail!\n", __FILE__, __FUNCTION__, __LINE__, pin, pValue);
		goto UNLOCK;
	}
UNLOCK:
	gpio_free(pin);
END:
	return;
}
EXPORT_SYMBOL(syno_gpio_direction_output);

void syno_gpio_direction_input(int pin)
{
	int iErr = 0;
	iErr = gpio_request(pin, NULL);
	if (iErr) {
		printk("%s:%s(%d) gpio_request pin %d fail!\n", __FILE__, __FUNCTION__, __LINE__, pin);
		goto END;
	}
	iErr = gpio_direction_input(pin);
	if (iErr) {
		printk("%s:%s(%d) set gpio pin %d input fail!\n", __FILE__, __FUNCTION__, __LINE__, pin);
		goto UNLOCK;
	}
UNLOCK:
	gpio_free(pin);
END:
	return;
}
EXPORT_SYMBOL(syno_gpio_direction_input);

int syno_gpio_to_irq(int pin)
{
	return gpio_to_irq(pin);
}
EXPORT_SYMBOL(syno_gpio_to_irq);

int SYNO_GPIO_READ(int pin)
{
#if defined(MY_DEF_HERE)
	int iVal=0;
	syno_gpio_value_get(pin, &iVal);
	return iVal;
#else
#if defined(MY_DEF_HERE)
	/*
	 * pinctl-nb range 476 to 511
	 * pinctl-sb range 446 to 475
	 */
	pin = pin < 36 ? (512 - 36 + pin) : (512 - 36 - 30) + pin - 36;
#endif /* MY_DEF_HERE */
	return gpio_get_value(pin);
#endif
}
EXPORT_SYMBOL(SYNO_GPIO_READ);

void SYNO_GPIO_WRITE(int pin, int pValue)
{
#if defined(MY_DEF_HERE)
	syno_gpio_value_set(pin, pValue);
#elif defined(MY_ABC_HERE)
	syno_gpio_direction_output(pin, pValue);
#else
#if defined(MY_DEF_HERE)
	/*
	 * pinctl-nb range 476 to 511
	 * pinctl-sb range 446 to 475
	 */
	pin = pin < 36 ? (512 - 36 + pin) : (512 - 36 - 30) + pin - 36;
#endif /* MY_DEF_HERE */
	gpio_set_value(pin, pValue);
#endif
}
EXPORT_SYMBOL(SYNO_GPIO_WRITE);

#ifdef MY_ABC_HERE
int SynoHaveRPDetectPin(void)
{
	if (syno_gpio.redundant_power_detect &&
		HAVE_RP_DETECT(1) &&
		HAVE_RP_DETECT(2)) {
		return 1;
	}
	return 0;
}
int SynoAllRedundantPowerDetected(void)
{
	if (syno_gpio.redundant_power_detect && 2 == syno_gpio.redundant_power_detect->nr_gpio &&
		!(SYNO_GPIO_READ(RP_DETECT_PIN(1)) ^ SYNO_GPIO_READ(RP_DETECT_PIN(2)))) {
		return 1;
	}
	return 0;
}
void DBG_SpinupGroupListGpio(void)
{
	int i = 0;
	if (giSynoSpinupGroupDebug && NULL != syno_gpio.hdd_detect) {
		for (i = 0; i < syno_gpio.hdd_detect->nr_gpio; i++) {
			printk("gpio debug: hdd detect pin %d, value= %d\n", HDD_DETECT_PIN(i + 1), SYNO_GPIO_READ(HDD_DETECT_PIN(i + 1)));
		}
		for (i = 0; i < syno_gpio.hdd_enable->nr_gpio; i++) {
			printk("gpio debug: hdd enable pin %d, value= %d\n", HDD_ENABLE_PIN(i + 1), SYNO_GPIO_READ(HDD_ENABLE_PIN(i + 1)));
		}
		if (syno_gpio.redundant_power_detect && 2 == syno_gpio.redundant_power_detect->nr_gpio) {
			printk("gpio debug: redundant power detect pin %d, value= %d\n", RP_DETECT_PIN(1), SYNO_GPIO_READ(RP_DETECT_PIN(1)));
			printk("gpio debug: redundant power detect pin %d, value= %d\n", RP_DETECT_PIN(2), SYNO_GPIO_READ(RP_DETECT_PIN(2)));
		}
	}
}
EXPORT_SYMBOL(DBG_SpinupGroupListGpio);
#endif /* MY_ABC_HERE */
