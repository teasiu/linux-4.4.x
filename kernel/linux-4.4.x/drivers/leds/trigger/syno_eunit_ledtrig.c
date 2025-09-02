#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
// Copyright (c) 2000-2014 Synology Inc. All rights reserved.
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/synolib.h>
#include <linux/device.h>
#include <linux/slab.h>

#ifdef MY_DEF_HERE
#include <linux/of.h>
#include <linux/syno_fdt.h>
#endif /* MY_DEF_HERE */

typedef struct _tag_SYNO_EUNIT_LED_TRIGGER_TIMER {
	struct timer_list Timer;
	int DiskActivity;
	int DiskLastActivity;
	int DiskFaulty;
} SYNO_EUNIT_LED_TRIGGER_TIMER;

#define SYNO_MAX_EUNIT 4
#define SYNO_MAX_LED 255

static SYNO_EUNIT_LED_TRIGGER_TIMER syno_eunit_led_trigger_timer[SYNO_MAX_EUNIT][SYNO_MAX_LED];
static struct led_trigger syno_eunit_led_ledtrig[SYNO_MAX_EUNIT][SYNO_MAX_LED];
char syno_eunit_led_trigger_name[SYNO_MAX_EUNIT][SYNO_MAX_LED][64];
int syno_eunit_led_trigger_number[SYNO_MAX_EUNIT] = {0, 0, 0, 0};
int ActiveLedMap[SYNO_MAX_EUNIT][SYNO_MAX_LED];
int FaultyLedMap[SYNO_MAX_EUNIT][SYNO_MAX_LED];

char* syno_eunit_ledtrig_name_get(int eunit_index, int led_index)
{
	char *pRet = NULL;
	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		goto END;
	}
	if (0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		goto END;
	}
	pRet = syno_eunit_led_trigger_name[eunit_index][led_index];
END:
	return pRet;
}
EXPORT_SYMBOL(syno_eunit_ledtrig_name_get);
void syno_eunit_ledtrig_set(int eunit_index, int led_index, enum led_brightness brightness)
{
	if (0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		return;
	}
	if (0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		return;
	}
	led_trigger_event(&syno_eunit_led_ledtrig[eunit_index][led_index], brightness);
}
EXPORT_SYMBOL(syno_eunit_ledtrig_set);

void syno_eunit_ledtrig_active_set(int eunit_index, int disk_index)
{
	int led_index;
	SYNO_EUNIT_LED_TRIGGER_TIMER *pTriggerTimer = NULL;

	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		return;
	}

	led_index = ActiveLedMap[eunit_index][disk_index];
	if (0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		return;
	}

	pTriggerTimer = &syno_eunit_led_trigger_timer[eunit_index][led_index];
	if (1 == pTriggerTimer->DiskFaulty){
		return;
	}

	pTriggerTimer->DiskActivity++;
	if (!timer_pending(&pTriggerTimer->Timer)){
		mod_timer(&pTriggerTimer->Timer, jiffies + msecs_to_jiffies(100));
	}
}
EXPORT_SYMBOL(syno_eunit_ledtrig_active_set);

void syno_eunit_ledtrig_faulty_set(int eunit_index, int led_index, int iFaulty)
{
	SYNO_EUNIT_LED_TRIGGER_TIMER *pTriggerTimer = NULL;

	if (0 > iFaulty) {
		return;
	}

	if (0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		return;
	}

	if (0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		return;
	}

	pTriggerTimer = &syno_eunit_led_trigger_timer[eunit_index][led_index];
	pTriggerTimer->DiskFaulty = iFaulty;
}
EXPORT_SYMBOL(syno_eunit_ledtrig_faulty_set);

static void syno_active_eunit_ledtrig_timerfunc(unsigned long index)
{
	int led_index = index%1000;
	int eunit_index = index/1000;
	SYNO_EUNIT_LED_TRIGGER_TIMER *pTriggerTimer = NULL;

	if (0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		return;
	}
	if (0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		return;
	}

	pTriggerTimer = &syno_eunit_led_trigger_timer[eunit_index][led_index];

	if (pTriggerTimer->DiskLastActivity != pTriggerTimer->DiskActivity) {
		pTriggerTimer->DiskLastActivity = pTriggerTimer->DiskActivity;
		led_trigger_event(&syno_eunit_led_ledtrig[eunit_index][led_index], LED_HALF);
		mod_timer(&pTriggerTimer->Timer, jiffies + msecs_to_jiffies(150));
	} else if ( 1 == pTriggerTimer->DiskFaulty){
		led_trigger_event(&syno_eunit_led_ledtrig[eunit_index][led_index], LED_OFF);
	} else {
		led_trigger_event(&syno_eunit_led_ledtrig[eunit_index][led_index], LED_FULL);
	}
}

int syno_eunit_led_number_fill(int eunit_index, int led_num)
{
	int iRet = -1;
	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		goto END;
	}
	if(0 > led_num || SYNO_MAX_LED <= led_num){
		goto END;
	}
	syno_eunit_led_trigger_number[eunit_index] = led_num;
	iRet = 0;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_eunit_led_number_fill);

int syno_eunit_led_number_clear(int eunit_index)
{
	int iRet = -1;
	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		goto END;
	}

	syno_eunit_led_trigger_number[eunit_index] = 0;
	iRet = 0;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_eunit_led_number_clear);

int syno_eunit_active_led_remap_fill(int eunit_index, int disk_index, int led_index)
{
	int iRet = -1;
	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		goto END;
	}
	if(0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		goto END;
	}
	ActiveLedMap[eunit_index][disk_index] = led_index;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_eunit_active_led_remap_fill);
int syno_eunit_faulty_led_remap_fill(int eunit_index, int disk_index, int led_index)
{
	int iRet = -1;
	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		goto END;
	}
	if(0 > led_index || syno_eunit_led_trigger_number[eunit_index] <= led_index){
		goto END;
	}
	FaultyLedMap[eunit_index][disk_index] = led_index;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_eunit_faulty_led_remap_fill);
int syno_eunit_led_remap_clear(int eunit_index)
{
	int iRet = -1;
	if(0 > eunit_index || SYNO_MAX_EUNIT <= eunit_index){
		goto END;
	}

	memset(ActiveLedMap[eunit_index], -1, sizeof(ActiveLedMap[eunit_index]));
	memset(FaultyLedMap[eunit_index], -1, sizeof(FaultyLedMap[eunit_index]));
END:
	return iRet;
}
EXPORT_SYMBOL(syno_eunit_led_remap_clear);

static int __init syno_eunit_ledtrig_init(void)
{
	int led_index = 0, eunit_index;
	int err = 0;
	SYNO_EUNIT_LED_TRIGGER_TIMER *pTriggerTimer = NULL;
	unsigned long index;

	memset(syno_eunit_led_trigger_name, 0, sizeof(syno_eunit_led_trigger_name));
	memset(syno_eunit_led_ledtrig, 0, sizeof(syno_eunit_led_ledtrig));
	memset(syno_eunit_led_trigger_timer, 0, sizeof(syno_eunit_led_trigger_timer));

	/*register all triggers used in DSM*/
	for(eunit_index = 0; eunit_index < SYNO_MAX_EUNIT; eunit_index++) {
		for(led_index = 0; led_index < SYNO_MAX_LED; led_index++) {
			snprintf(syno_eunit_led_trigger_name[eunit_index][led_index], 64, "syno_eunit%d_led%d_ledtrig", eunit_index, led_index);
			syno_eunit_led_ledtrig[eunit_index][led_index].name = syno_eunit_led_trigger_name[eunit_index][led_index];

			err = led_trigger_register(&syno_eunit_led_ledtrig[eunit_index][led_index]);
			if (0 != err ){
				printk("fail to regist tirgger Num %d \n", led_index);
				break;
			}
			pTriggerTimer = &syno_eunit_led_trigger_timer[eunit_index][led_index];
			pTriggerTimer->DiskFaulty = 0;
			// since timer data only accept unsigned long
			// combine eunit and led index by following formular
			index = led_index + eunit_index*1000;
			setup_timer(&pTriggerTimer->Timer, syno_active_eunit_ledtrig_timerfunc, index);
		}
	}
	return err;
}
module_init(syno_eunit_ledtrig_init);

static void __exit syno_eunit_ledtrig_exit(void)
{
	int led_index = 0, eunit_index;

	/*unregister triggers*/
	for(eunit_index = 0; eunit_index < SYNO_MAX_EUNIT; eunit_index++) {
		for(led_index = 0 ; led_index < SYNO_MAX_LED ; led_index++){
			led_trigger_unregister_simple(&syno_eunit_led_ledtrig[eunit_index][led_index]);
		}
	}
}
module_exit(syno_eunit_ledtrig_exit);
