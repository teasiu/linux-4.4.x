#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
// Copyright (c) 2000-2015 Synology Inc. All rights reserved.
#ifndef __SYNOLIB_H_
#define __SYNOLIB_H_

#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/kobject.h>
#ifdef MY_DEF_HERE
#include <linux/of.h>
#endif /* MY_DEF_HERE */
#include <linux/genhd.h>

#ifdef  MY_ABC_HERE
extern int gSynoDebugFlag;
extern int gSynoAtaDebug;
extern int gSynoHibernationLogLevel;
extern struct mm_struct *syno_get_task_mm(struct task_struct *task);
void syno_do_hibernation_fd_log(const int fd);
void syno_do_hibernation_filename_log(const char __user *filename);
void syno_do_hibernation_inode_log(struct inode *inode);
void syno_do_hibernation_bio_log(const char *DeviceName);
void syno_do_hibernation_scsi_log(const char *DeviceName);
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
void syno_draw_auto_remap_buffer(char *buffer, int size);
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
int SynoSCSIGetDeviceIndex(struct gendisk *disk);
#endif
#ifdef MY_DEF_HERE
int SynoNVMeGetDeviceIndex(struct gendisk *disk);
#endif /* MY_DEF_HERE */
#ifdef MY_ABC_HERE
int SynoDiskGetDeviceIndex(struct block_device *bdev);
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
/**
 * How to use :
 * 1. module itself register the proprietary instance into the kernel
 *    by a predined MAGIC-key.
 * 2. Others can query the module registration by the same MAGIC-key
 *    and get the instance handle.
 * ********************************************************************
 * Beware of casting/handing "instance", you must know
 * what you are doing before accessing the instance.
 * ********************************************************************
 */
/* For plugin-instance registration */
int syno_plugin_register(int plugin_magic, void *instance);
int syno_plugin_unregister(int plugin_magic);
/* For getting the plugin-instance */
int syno_plugin_handle_get(int plugin_magic, void **hnd);
void * syno_plugin_handle_instance(void *hnd);
void syno_plugin_handle_put(void *hnd);

/* Magic definition */
#define EPIO_PLUGIN_MAGIC_NUMBER    0x20120815
#define RODSP_PLUGIN_MAGIC_NUMBER    0x20141111
#endif

#ifdef MY_ABC_HERE
/* Maximum number of MAC addresses */
#define SYNO_MAC_MAX_NUMBER 8
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
#define SATA_REMAP_MAX  64
#define SATA_REMAP_NOT_INIT 0xff
extern int g_syno_sata_remap[SATA_REMAP_MAX];
extern int g_use_sata_remap;
int syno_get_remap_idx(int origin_idx);
extern int g_syno_mv14xx_remap[SATA_REMAP_MAX];
extern int g_use_mv14xx_remap;
int syno_get_mv_14xx_remap_idx(int origin_idx);
#endif /* MY_ABC_HERE */

#ifdef MY_DEF_HERE
#define MAX_INTERNAL_ATA_PORT 60
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
#define DT_INTERNAL_SLOT "internal_slot"
#define DT_SYSTEM_SLOT "system_slot"
#define DT_ESATA_SLOT "esata_port"
#define DT_CX4_SLOT "cx4_port"
#define DT_PCIE_SLOT "pcie_slot"
#define DT_USB_SLOT "usb_slot"
#define DT_HUB_SLOT "usb_hub"
#define DT_POWER_PIN_GPIO "power_pin_gpio"
#define DT_DETECT_PIN_GPIO "detect_pin_gpio"
#define DT_SWITCH_NO "switch_no"
#define DT_HDD_LED_TYPE "led_type"
#define DT_HDD_ORANGE_LED "led_orange"
#define DT_HDD_GREEN_LED "led_green"
#define DT_HDD_LED_NAME "led_name"
#define DT_HDD_ACT_LED "led_activity"
#define DT_SYNO_GPIO "syno_gpio"
#define DT_PCIE_ROOT "pcie_root"
#define DT_ATA_PORT "ata_port"
#define DT_AHCI "ahci"
#define DT_AHCI_RTK "ahci_rtk"
#define DT_AHCI_MVEBU "ahci_mvebu"
#define DT_MV14XX "mv14xx"
#define DT_PHY "phy"
#define DT_USB2 "usb2"
#define DT_USB3 "usb3"
#define DT_USB_PORT "usb_port"
#define DT_USB_HUB "usb_hub"
#define DT_USB_TO_TTY "usb_to_tty"
#define DT_VBUS "vbus"
#define DT_SHARED "shared"
#define DT_SYNO_SPINUP_GROUP "syno_spinup_group"
#define DT_SYNO_SPINUP_GROUP_DELAY "syno_spinup_group_delay"
#define DT_HDD_POWERUP_SEQ "syno_hdd_powerup_seq"
#define DT_PROPERTY_SW_ACTIVITY "sw_activity"
#define DT_DISK_LED_TYPE_GPIO "gpio"
#define DT_FORM_FACTOR "form_factor"
#define DT_EXPANDER "expander"
#define DT_MODEL_NAME "model_name"
#define DT_SWITCHTEC "switchtec"
#define DT_LED_OFF_GPIO "led_off_gpio"
#define DT_I2C_BUS "i2c_bus"
#define DT_I2C_DEVICE "i2c_device"
#define DT_I2C_ADDRESS "i2c_address"
#define DT_I2C_DEVICE_NAME "i2c_device_name"
#define DT_DEVICE_INDEX "device_index"
#define DT_ACPI_HID "acpi_hid"
#define DT_ACPI_UID "acpi_uid"
#define DT_SET_SSC_OFF "set_ssc_off"
#define DT_NVME "nvme"
#define DT_PCIE_POSTFIX "pcie_postfix"
#define DT_PCIE_EUNIT_PORT "pcie_eunit_port"
#define DT_NUMBER_OF_LED_TRIGGER "number_of_led_trigger"
#define DT_M2_CARD "m2_card"
#define DT_EUNIT "eunit"
#define DT_I2C_BUS "i2c_bus"
#define DT_PCIID_LIST "pciid_list"
#define DT_PHY_ID "phy_id"
#define DT_MODEL_NAME "model_name"
#define DT_SAS "sas"
#define DT_SYNO_APIC_DRIVER "apic_driver"
#define DT_PMP_SLOT "pmp_slot"
#define DT_LIBATA "libata"
#define DT_PMP_LINK "pmp_link"
#define DT_POWER_SEQ "power_seq"

#ifdef MY_DEF_HERE
#define DT_AHCI_INTERNAL_MODE "internal_mode"
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
#define DT_PCIEHP_FORCE "pciehp_force"
#define DT_ROOT_LIST "root_list"
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
#define DT_SYNO_HDD_SMBUS_TYPE "syno_smbus_hdd_type"
#define DT_SYNO_HDD_SMBUS_ADAPTER "syno_smbus_hdd_adapter"
#define DT_SYNO_HDD_SMBUS_ADDRESS "syno_smbus_hdd_address"

#define SMBUS_SWITCH_MAX_COUNT 16
#define DT_SYNO_SMBUS_SWITCH_ADAPTERS "syno_smbus_switch_adapters"
#define DT_SYNO_SMBUS_SWITCH_ADDRS "syno_smbus_switch_addrs"
#define DT_SYNO_SMBUS_SWITCH_VALS "syno_smbus_switch_vals"
#endif /* MY_DEF_HERE */

#define DT_SYNO_PMBUS_ADAPTER "syno_pmbus_adapter"
#define DT_SYNO_PMBUS_ADDRESS "syno_pmbus_address"
#define DT_SYNO_PMBUS_PIN_REG "syno_pmbus_pin_register"
#define DT_SYNO_PMBUS_POUT_REG "syno_pmbus_pout_register"
#define DT_SYNO_PMBUS_TEMP1_REG "syno_pmbus_temp1_register"
#define DT_SYNO_PMBUS_TEMP2_REG "syno_pmbus_temp2_register"
#define DT_SYNO_PMBUS_TEMP3_REG "syno_pmbus_temp3_register"
#define DT_SYNO_PMBUS_FAN_REG "syno_pmbus_fan_register"
#define DT_SYNO_PMBUS_STATUS_REG "syno_pmbus_status_register"
#define DT_SYNO_PMBUS_PSU_OFF_BIT "syno_pmbus_psu_off_bit"
#define DT_SYNO_PMBUS_PSU_PRESENT_BIT "syno_pmbus_psu_present_bit"

#ifdef MY_DEF_HERE
#define SZ_DTS_AHCI_IRQ "ahci_irq"
#define SZ_AHCI_HARD_IRQ "hard_irq"
#define SZ_AHCI_THREADED_IRQ "threaded_irq"
#endif /* MY_DEF_HERE */

#ifdef MY_ABC_HERE
#define SZ_DTS_EBOX_I2C_PWR_BTN "power_btn"
#define SZ_DTS_EBOX_I2C_OFFSET "offset"
#define SZ_DTS_EBOX_I2C_MASK "mask"
#define SZ_DTS_EBOX_I2C_PWR_CTL "power_control"
#define SZ_DTS_EBOX_I2C_SN_READ "ebox_sn_read"
#define SZ_DTS_EBOX_RP "rp_power"
#define SZ_DTS_EBOX_RP_INFO "rp_power_info"
#define SZ_DTS_EBOX_I2C_DEEPSELLP_CTL "deep_sleep_control"
#define SZ_DTS_EBOX_I2C_DEEPSELLP_INDICATOR "deep_sleep_indicator"
#define SZ_DTS_EBOX_I2C_REG_MANUAL_ENABLE "reg_manual_enable"
#endif /* MY_ABC_HERE */


#ifdef MY_DEF_HERE

#define DT_EUNIT_STATUS_EXPSTATUS "expstatus"
#define DT_EUNIT_STATUS_EXPCTRL "expctrl"
#define DT_EUNIT_STATUS_FANPWM "fanpwm"
#define DT_EUNIT_STATUS_FANSPEED "fanspeed"
#define DT_EUNIT_STATUS_HDDCTRL "hddctrl"
#define DT_EUNIT_STATUS_DISKLED "diskled"
#define DT_EUNIT_STATUS_7SEGLED "7segled"
#define DT_EUNIT_STATUS_EXPIDSET "expidset"
#define DT_EUNIT_STATUS_EXPSNSET "expsnset"
#define DT_EUNIT_STATUS_UPVERSION "upversion"
#define DT_EUNIT_STATUS_HDDPRESENT "hddpresent"
#define DT_EUNIT_STATUS_HDDENABLE "hddenable"
#define DT_EUNIT_STATUS_MONCURRENT "moncurrent"
#define DT_EUNIT_STATUS_MONVOLTAGE "monvoltage"
#define DT_EUNIT_STATUS_MONTHERMAL "monthermal"
#define DT_EUNIT_CONTROL_METHOD "control_method"
#define DT_EUNIT_CONTROL_TYPE "control_type"

#define DT_EUNIT_COMMAND "command"
#define DT_EUNIT_OFFSET "offset"

#define DT_EUNIT_DISK_PRESENT "disk_present"
#define DT_EUNIT_DISK_POWER_ON "disk_power_on"

typedef enum _tag_EUNIT_STATUS_INDEX {
	EUNIT_STATUS_EXPCTRL = 0,
	EUNIT_STATUS_FANPWM,
	EUNIT_STATUS_FANSPEED,
	EUNIT_STATUS_HDDCTRL,
	EUNIT_STATUS_DISKLED,
	EUNIT_STATUS_7SEGLED,
	EUNIT_STATUS_EXPSNSET,
	EUNIT_STATUS_EXPIDSET,
	EUNIT_STATUS_UPVERSION,
	EUNIT_STATUS_HDDPRESENT,
	EUNIT_STATUS_HDDENABLE,
	EUNIT_STATUS_MONCURRENT,
	EUNIT_STATUS_MONVOLTAGE,
	EUNIT_STATUS_MONTHERMAL,
	EUNIT_STATUS_UNKNOWN,
	EUNIT_STATUS_INDEX_END,
} EUNIT_STATUS_INDEX;
#endif /* MY_DEF_HERE */

#define SYNO_DTS_PROPERTY_CONTENT_LENGTH 128 // If used to retrive PCIe path, can only accept 9 layer PCIe switch.
#define MAX_NODENAME_LEN 31

/* This enum must sync with synosdk/fs.h for user space having same DISK_PORT_TYPE mapping */
typedef enum _tag_DISK_PORT_TYPE{
	UNKNOWN_DEVICE = 0,
	INTERNAL_DEVICE,
	EXTERNAL_SATA_DEVICE,
	EUNIT_DEVICE,
	EXTERNAL_USB_DEVICE,
	SYNOBOOT_DEVICE,
	ISCSI_DEVICE,
	CACHE_DEVICE,
	USB_HUB_DEVICE,
	SDCARD_DEVICE,
	INVALID_DEVICE,
	SYSTEM_DEVICE,
	DISK_PORT_TYPE_END,
} DISK_PORT_TYPE;

#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
typedef struct _syno_smbus_hdd_powerctl {
        bool bl_init;
        int (*syno_smbus_hdd_enable_write)(int adapter, int address, int index, int val);
        int (*syno_smbus_hdd_enable_read)(int adapter, int address, int index);
        int (*syno_smbus_hdd_present_read)(int adapter, int address, int index);
        int (*syno_smbus_hdd_enable_write_all_once)(int adapter, int address);
} SYNO_SMBUS_HDD_POWERCTL;
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
#define PCI_ADDR_LEN_MAX 9
#define PCI_ADDR_NUM_MAX CONFIG_SYNO_MAX_PCI_SLOT
extern char gszPciAddrList[PCI_ADDR_NUM_MAX][PCI_ADDR_LEN_MAX];
extern int gPciAddrNum;
extern int syno_check_on_option_pci_slot(struct pci_dev *pdev);
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
/* Max 768 */
#define M2SATA_START_IDX 800
extern int gPciDeferStart;
extern int g_nvc_map_index;
extern int g_syno_nvc_index_map[SATA_REMAP_MAX];
void syno_insert_sata_index_remap(unsigned int idx, unsigned int num, unsigned int id_start);
#endif /* MY_DEF_HERE */
#ifdef MY_DEF_HERE
#define M2_HOST_LEN_MAX 128
#define M2_PORT_NO_MAX 16
extern char gSynoM2HostName[M2_HOST_LEN_MAX];
extern unsigned long gSynoM2PortNo;
extern unsigned long gSynoM2PortIndex[M2_PORT_NO_MAX];
#endif /* MY_DEF_HERE */
#ifdef MY_ABC_HERE
#define SYNO_SPINUP_GROUP_MAX 16
#define SYNO_SPINUP_GROUP_PIN_MAX_NUM 8
extern int g_syno_rp_detect_no;
extern int g_syno_rp_detect_list[SYNO_SPINUP_GROUP_PIN_MAX_NUM];
extern int g_syno_hdd_detect_no;
extern int g_syno_hdd_detect_list[SYNO_SPINUP_GROUP_PIN_MAX_NUM];
extern int g_syno_hdd_enable_no;
extern int g_syno_hdd_enable_list[SYNO_SPINUP_GROUP_PIN_MAX_NUM];
#endif /* MY_ABC_HERE */
#ifdef MY_DEF_HERE
#define SYSTEM_DEVICE_START_IDX 900
#define SYSTEM_DEVICE_NUM_MAX 10
#endif /* MY_DEF_HERE */

#if defined(MY_ABC_HERE) || defined(MY_ABC_HERE)
#define SYNOBIOS_EVENTDATA_NUM_MAX 8
typedef struct _synobios_event_parm_tag {
	unsigned long long data[SYNOBIOS_EVENTDATA_NUM_MAX];
} SYNOBIOS_EVENT_PARM;

typedef int (*FUNC_SYNOBIOS_EVENT)(SYNOBIOS_EVENT_PARM parms);

typedef struct _synobios_evnet_action_tag {
	FUNC_SYNOBIOS_EVENT *funcSynobiosEvent;
	SYNOBIOS_EVENT_PARM parms;
	struct list_head list;
} SYNOBIOS_EVENT_ACTION_LIST;
#endif /* MY_ABC_HERE || MY_ABC_HERE */

#ifdef MY_DEF_HERE
/*
 * Notice
 * ------
 *  Before calling syno_kexec_test() or reading kexex_test_flags, please
 *  ensure that syno_kexec_test_init() has been called.
 */
#define KEXEC_TEST_DECOMPRESSION	0	/* Did we skip compressed/head_64.S ? */
#define KEXEC_TEST_BOOTLOADER		1	/* Is bootloader type 0xD ? */
#define KEXEC_TEST_E820_TABLE		2	/* Is the minimal start address of usable memory in e820 table 0x100 ? */
#define KEXEC_TEST_SETUP_DATA		3	/* Did we receive setup_data with type SETUP_NONE or SETUP_EFI ? */

extern unsigned long kexec_test_flags;

/*
 * Test whether the above KEXEC_TEST_* bits are set.
 */
static __always_inline bool syno_kexec_test(int test)
{
	return 0 != test_bit(test, &kexec_test_flags);
}
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE

typedef enum _tag_SYNO_MPATH_SYSFS_AGGR_METHOD {
	/*
	 * The default option.
	 * If the attr values between two native device are the same, we will pick
	 * arbitrary one of them.
	 */
	MPATH_SYSFS_SHOW_AGGR_ARBITRARY,
	/* Report the minimum value as decimal unsigned long */
	MPATH_SYSFS_SHOW_AGGR_MIN_UL_DEC,
} SYNO_MPATH_SYSFS_SHOW_AGGR_METHOD;

typedef struct _syno_multipath_target_sysfs {
	struct kobject deviceKobj;
	struct kobj_type deviceKtype;
	struct kobject *parent;
	struct mapped_device *md;
	ssize_t (*funcTargetSysfsShow)(struct gendisk*, struct attribute*, char*);
	ssize_t (*funcTargetSysfsStore)(struct gendisk*, struct attribute*, const char*, size_t);
	SYNO_MPATH_SYSFS_SHOW_AGGR_METHOD (*funcTargetShowAggrMethod)(struct attribute*);
} SYNO_MPATH_TARGET_SYSFS;
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
struct asm2824_pdata {
	struct pci_dev *pci_dev;
};
int syno_pci_dev_to_i2c_bus(struct pci_dev*);
#endif /* CONFIG_SYNO_PCIEI2C */

#ifdef MY_DEF_HERE
int syno_nvme_index_get(struct pci_dev *pdev, char *syno_block_info);
bool syno_pciid_list_cmp(struct pci_dev *pdev, struct device_node *pDeviceNode);
struct device_node * syno_pci_dev_to_eunit_node(struct pci_dev* pdev);
int syno_eunit_disk_index_get(struct pci_dev *pdev, char *syno_block_info);
int syno_eunit_index_get(struct pci_dev *pdev, char *syno_block_info);
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
void syno_add_eunit_led_remap(struct pci_dev* pdev);
void syno_del_eunit_led_remap(struct pci_dev* pdev);
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
#define SYNO_ATMEGA_NUM_MAX 2
#endif /* MY_DEF_HERE */

struct syno_device_list {
	char disk_name[DISK_NAME_LEN];
	struct list_head device_list;
};

#ifdef MY_DEF_HERE

enum spinup_operation {
	SPINDOWN = 0,
	SPINUP_CHECK,
	SPINUP_NODELAY,
	SPINUP_DELAY,
};

struct syno_control_operations {
	const char control_method[SYNO_DTS_PROPERTY_CONTENT_LENGTH];
	int (*unique_get)(const int slot_type, const int slot_index, char *unique, int unique_size);
	int (*container_index_get_by_diskname)(int *slot_index, const char* diskname);
	int (*hdd_ctrl)(const int slot_type, const int slot_index, const int power_ctrl);
	int (*deep_sleep_indicator_ctrl)(const int slot_type, const int slot_index, const int ctrl);
	int (*power_control) (const int slot_type, const int slot_index);
	int (*disk_delay_waiting) (const int slot_type, const int slot_index, const int disk_id, int action);
	int (*disk_is_wait_power_on) (const int slot_type, const int slot_index, const int disk_slot_id);
};
#endif /* MY_DEF_HERE */

#endif //__SYNOLIB_H_
