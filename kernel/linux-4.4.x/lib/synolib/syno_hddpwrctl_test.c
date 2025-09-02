#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
#include <linux/init.h>
#include <linux/module.h>
#include <linux/libata.h>
#ifdef MY_ABC_HERE
#include <linux/syno_gpio.h>
#endif /* MY_ABC_HERE */
#if defined(MY_DEF_HERE) || defined(MY_DEF_HERE)
#include <linux/synolib.h>
#endif /* defined(MY_DEF_HERE) || defined(MY_DEF_HERE) */

static struct kobject *SynoHddPwrCtlObject = NULL;
int iSlot = 0;
int iMaxHddNum = INT_MAX;

#ifdef MY_DEF_HERE
extern long g_smbus_hdd_powerctl;
extern int gSynoSmbusHddAdapter;
extern int gSynoSmbusHddAddress;
extern void syno_smbus_hdd_powerctl_init(void);
extern SYNO_SMBUS_HDD_POWERCTL SynoSmbushddpwrctl;
#endif /* MY_DEF_HERE */
extern int SYNO_SUPPORT_HDD_DYNAMIC_ENABLE_POWER(void);
extern int SYNO_CTRL_HDD_POWERON(int index, int value);
extern int SYNO_CHECK_HDD_ENABLE(int index);
#ifdef MY_ABC_HERE
extern int SYNO_CHECK_HDD_DETECT(int index);
#else
extern int SYNO_CHECK_HDD_PRESENT(int index);
#endif /* MY_ABC_HERE */


static ssize_t pwrctl_slot_show (struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf (buf, PAGE_SIZE, "%d\n", iSlot);
}
static ssize_t pwrctl_slot_store (struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    if (0 > kstrtoint(buf, 0, &iSlot)) {
		printk(KERN_WARNING "Failed to convert string to unsigned int.\n");
		goto END;
	}
    if (1 > iSlot || iMaxHddNum < iSlot) {
		printk(KERN_WARNING "Invalid Slot Number\n");
		iSlot = 0;
		goto END;
    }

END:
	return count;
}

static ssize_t pwrctl_present_show (struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int iRet = 0;
    int iPrzPinVal = 1;
    
	if (1 > iSlot || iMaxHddNum < iSlot) {
		printk(KERN_WARNING "Invalid Slot Number\n");
		goto END;
    }

#ifdef MY_ABC_HERE
		iPrzPinVal = SYNO_CHECK_HDD_DETECT(iSlot);
#else
		iPrzPinVal = SYNO_CHECK_HDD_PRESENT(iSlot);
#endif /* MY_ABC_HERE */

	iRet += scnprintf (buf, PAGE_SIZE, "Slot%d is %sPresent\n", iSlot, iPrzPinVal?"":"Not ");

END:
	return iRet;
}
static ssize_t pwrctl_present_store (struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    printk(KERN_WARNING "Present pin is not writable\n");
	return count;
}

static ssize_t pwrctl_enable_show (struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int iRet = 0;
	/* defult is enable */
    int iEnPinVal = 1;


	if (1 > iSlot || iMaxHddNum < iSlot) {
		printk(KERN_WARNING "Invalid Slot Number\n");
		goto END;
    }

	iEnPinVal = SYNO_CHECK_HDD_ENABLE(iSlot);

	iRet += scnprintf (buf, PAGE_SIZE, "Slot%d is %sEnable\n", iSlot, iEnPinVal?"":"Not ");
END:
	return iRet;
}
static ssize_t pwrctl_enable_store (struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    int iValue = 0;

    if (0 > kstrtoint(buf, 0, &iValue)) {
		printk(KERN_WARNING "Failed to convert string to unsigned int.\n");
		goto END;
	}

    if (0 > iValue || 1 < iValue) {
        printk(KERN_WARNING "Invalid Input Value\n");
        goto END;
    }

	if (1 > iSlot || iMaxHddNum < iSlot) {
		printk(KERN_WARNING "Invalid Slot Number\n");
		goto END;
    }

    SYNO_CTRL_HDD_POWERON(iSlot, iValue);
END:
	return count;
}

// register function to attribute
static struct kobj_attribute pwrctl_slot = __ATTR( pwrctl_slot, 0640, pwrctl_slot_show, pwrctl_slot_store);
static struct kobj_attribute pwrctl_present = __ATTR( pwrctl_present, 0640, pwrctl_present_show, pwrctl_present_store);
static struct kobj_attribute pwrctl_enable = __ATTR( pwrctl_enable, 0640, pwrctl_enable_show, pwrctl_enable_store);

// put attribute to attribute group
static struct attribute *SynoHddPwrCtlAttr[] = {
	&pwrctl_slot.attr,
	&pwrctl_present.attr,
	&pwrctl_enable.attr,
	NULL,   /* NULL terminate the list*/
};
static struct attribute_group SynoHddPwrCtlGroup = {
	.attrs = SynoHddPwrCtlAttr
};

static int syno_hddpwrctl_test_init(void)
{
	int iRet = -1;
	SynoHddPwrCtlObject = kobject_create_and_add("syno_hddpwrctl_test", kernel_kobj);
	if (!SynoHddPwrCtlObject) {
		iRet = -ENOMEM;
		goto END;
	}

	//create attributes (files)
	if(sysfs_create_group(SynoHddPwrCtlObject, &SynoHddPwrCtlGroup)){
		iRet = -ENOMEM;
		goto END;
	}

	if (!SYNO_SUPPORT_HDD_DYNAMIC_ENABLE_POWER()){
		printk(KERN_ERR "Not support HDD dynamic power control\n");
		goto END;
	}

#ifdef MY_DEF_HERE
		iMaxHddNum = gSynoInternalHddNumber;
#else /* MY_DEF_HERE */
	if (g_syno_hdd_powerup_seq) {
		iMaxHddNum = g_syno_hdd_powerup_seq;
	} else {
		//FIXME: for those do not set spinup group (ihd_num=0), need more info to check max slot number
	}
#endif /* MY_DEF_HERE */

	iRet = 0;
END:
	if (0 != iRet) {
		if (SynoHddPwrCtlObject) {
			kobject_put(SynoHddPwrCtlObject);
		}
	}
	return iRet;
}

static void syno_hddpwrctl_test_exit(void)
{
	kobject_put(SynoHddPwrCtlObject);
}

MODULE_LICENSE("GPL");
module_init(syno_hddpwrctl_test_init);
module_exit(syno_hddpwrctl_test_exit);
