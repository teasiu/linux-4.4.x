#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
#ifdef MY_DEF_HERE
#include <linux/synolib.h>
#include <linux/of.h>
#include <linux/syno_fdt.h>
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/syno_gpio.h>

extern int syno_compare_dts_pciepath(const struct pci_dev *pdev, const struct device_node *pDeviceNode);

#ifdef MY_DEF_HERE
int syno_pmbus_property_get(unsigned int *pmbus_property, const char *property_name, int index)
{
    int iRet = -1;
	if (NULL == pmbus_property || NULL == property_name) {
		goto END;
	}

    // if property name not exist, do nothing but return 0
    if (of_find_property(of_root, property_name, NULL)) {
        of_property_read_u32_index(of_root, property_name, index, pmbus_property);
    }

	iRet = 0;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_pmbus_property_get);
#endif /* MY_DEF_HERE */

struct device_node* syno_of_i2c_bus_match(struct device *dev, int* index)
{
	struct device_node *pI2CNode = NULL;
	struct pci_dev *pdev = NULL;
	char *i2c_hid = NULL;
	char *i2c_uid = NULL;
	struct acpi_device *acpi_dev = NULL;

	if(NULL == of_root || NULL == dev) {
		goto END;
	}

	for_each_child_of_node(of_root, pI2CNode) {
		if (pI2CNode->full_name && 1 == sscanf(pI2CNode->full_name, "/"DT_I2C_BUS"@%d", index)) {
			if (dev_is_pci(dev->parent)) {
				pdev = to_pci_dev(dev->parent);
				if (0 == syno_compare_dts_pciepath(pdev, pI2CNode)) {
					return pI2CNode;
				}
			} else if (is_acpi_device_node(dev->parent->fwnode)) {
				acpi_dev = ACPI_COMPANION(dev->parent);
				i2c_hid = (char *)of_get_property(pI2CNode, DT_ACPI_HID, NULL);
				i2c_uid = (char *)of_get_property(pI2CNode, DT_ACPI_UID, NULL);

				if (!i2c_hid || !i2c_uid) {
					continue;
				}

				if ( 0 == strncmp(i2c_hid, acpi_device_hid(acpi_dev), SYNO_DTS_PROPERTY_CONTENT_LENGTH) &&
						0 == strncmp(i2c_uid, acpi_dev->pnp.unique_id, SYNO_DTS_PROPERTY_CONTENT_LENGTH)) {
					return pI2CNode;
				}
			}
		}
	}
END:
	return NULL;
}
EXPORT_SYMBOL_GPL(syno_of_i2c_bus_match);

struct device_node* syno_of_i2c_device_match(struct i2c_client *client, const char *i2c_dev_name, struct device_node *pI2CNode)
{
	struct device_node *pI2CDevNode = NULL;
	char *device_name = NULL;
	char *device_address = NULL;
	unsigned short addr = 0;

	if (NULL == client || NULL == i2c_dev_name || NULL == pI2CNode) {
		goto END;
	}

	for_each_child_of_node(pI2CNode, pI2CDevNode) {
		device_name = (char *)of_get_property(pI2CDevNode, DT_I2C_DEVICE_NAME, NULL);
		device_address = (char *)of_get_property(pI2CDevNode, DT_I2C_ADDRESS, NULL);
		if (NULL != device_address && 0 == kstrtou16(device_address, 16, &addr)) {
			if ( 0 == strncmp(i2c_dev_name, device_name, SYNO_DTS_PROPERTY_CONTENT_LENGTH)
					&& client->addr == addr) {
				return pI2CDevNode;
			}
		}
	}

END:
	return NULL;
}
EXPORT_SYMBOL_GPL(syno_of_i2c_device_match);

struct device_node* syno_of_i2c_adapter_match(struct i2c_adapter *adap)
{
	struct device_node *pI2CNode = NULL;
	struct device_node *pRet = NULL;
	char szName[32] = {'\0'};
	int id;

	if (!adap || !of_root) {
		goto END;
	}

	id = i2c_adapter_id(adap);
	snprintf(szName, sizeof(szName), "/"DT_I2C_BUS"@%d", id);

	for_each_child_of_node(of_root, pI2CNode) {
		if (!pI2CNode->full_name) {
			continue;
		}
		if (0 == strncmp(pI2CNode->full_name, szName, strlen(szName))) {
			pRet = pI2CNode;
			goto END;
		}
	}
END:
	return pRet;
}
EXPORT_SYMBOL_GPL(syno_of_i2c_adapter_match);

bool syno_of_i2c_driver_match_device(struct device *dev, const struct device_driver *drv)
{
	struct i2c_client *client = NULL;
	struct i2c_driver *driver = NULL;
	struct device_node *pNode = NULL;
	struct device_node *pDevNode = NULL;
	bool iRet = false;

	if(NULL == dev || NULL == drv) {
		goto END;
	}

	client = i2c_verify_client(dev);
	driver = to_i2c_driver(drv);

	if (NULL == client || NULL == driver) {
		goto END;
	}

	if (NULL != (pNode = syno_of_i2c_adapter_match(client->adapter)) &&
			NULL != (pDevNode = syno_of_i2c_device_match(client, driver->driver.name, pNode))) {
		iRet = true;
	}

END:
	return iRet;
}
EXPORT_SYMBOL_GPL(syno_of_i2c_driver_match_device);

/**
 * syno_disk_gpio_pin_get - get the property content of internal slot
 * @diskPort [IN]:          internel slot number
 * @szPropertyName [IN]:
 * @propertyIndex [IN]: index of number need be read in szPropertyName
 *
 * return >=0: property number in device tree of internal slot
 *        -1: fail
 */
u32 syno_disk_gpio_pin_get(const int diskPort, const char *szPropertyName, const int propertyIndex)
{
	int index= 0;
	u32 synoGpioPin = U32_MAX;
	struct device_node *pSlotNode = NULL;

	if (NULL == szPropertyName || 0 > diskPort || 0 >propertyIndex) {
		goto END;
	}

	for_each_child_of_node(of_root, pSlotNode) {
		// get index number of internal_slot, e.g. /internal_slot@4 --> 4
		if (!pSlotNode->full_name || 1 != sscanf(pSlotNode->full_name, "/"DT_INTERNAL_SLOT"@%d", &index)) {
			continue;
		}
		if (diskPort == index) {
			break;
		}
	}

	if (NULL == pSlotNode) {
		goto END;
	}
	of_property_read_u32_index(pSlotNode, szPropertyName, propertyIndex, &synoGpioPin);
	of_node_put(pSlotNode);
END:
	return synoGpioPin;
}
EXPORT_SYMBOL(syno_disk_gpio_pin_get);

/**
 * syno_disk_gpio_pin_have - determine the szPropertyName of the internal slot is defined in device tree
 * @diskPort [IN]:          internel slot number
 * @szPropertyName [IN]:
 *
 * return 1: property exist
 *        0: property not exist
 */
int syno_disk_gpio_pin_have(const int diskPort, const char *szPropertyName)
{
	u32 synoGpioPin = U32_MAX;
	int ret = -1;

	synoGpioPin = syno_disk_gpio_pin_get(diskPort, szPropertyName, SYNO_GPIO_PIN);

	if (U32_MAX != synoGpioPin) {
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(syno_disk_gpio_pin_have);
/**
 * syno_led_pin_get - get the szLedName pin of target slot
 * @szSlotName [IN]:    slot name
 * @diskPort [IN]:      slot number
 * @szLedName [IN]:		LED node name in device node
 * @propertyIndex [IN]: index of number need be read in DT_SYNO_GPIO
 *
 * return >=0: property number in device tree of target slot
 *        -1: fail
 */
u32 syno_led_pin_get(const char* szSlotName, const int diskPort, const char *szLedName, const int propertyIndex)
{
	u32 synoGpioPin = U32_MAX;
	struct device_node *pSlotNode = NULL, *pLedNode = NULL;
	char szFullName[MAX_NODENAME_LEN] = {0};

	if (NULL == szSlotName || NULL == szLedName || 1 > diskPort || 0 > propertyIndex) {
		goto END;
	}
	if (0 > snprintf(szFullName, MAX_NODENAME_LEN - 1, "/%s@%d", szSlotName, diskPort)) {
		goto END;
}

	for_each_child_of_node(of_root, pSlotNode) {
		if (pSlotNode->full_name && 0 == strcmp(pSlotNode->full_name, szFullName)) {
			break;
		}
	}

	if (NULL == pSlotNode) {
		goto END;
	}
	pLedNode = of_get_child_by_name(pSlotNode, szLedName);
	of_node_put(pSlotNode);
	if (NULL == pLedNode) {
		goto END;
	}
	of_property_read_u32_index(pLedNode, DT_SYNO_GPIO, propertyIndex, &synoGpioPin);
	of_node_put(pLedNode);

END:
	return synoGpioPin;
}
EXPORT_SYMBOL(syno_led_pin_get);

/**
 * syno_led_pin_have - determine the szLedName of the target slot is defined in device tree
 * @szSlotName [IN]: slot name
 * @diskPort [IN]:   slot number
 * @szLedName [IN]:	LED node name in device node
 *
 * return 1: szLedName exist
 *        0: szLedName not exist
 */
int syno_led_pin_have(const char* szSlotName, const int diskPort, const char *szLedName)
{
	u32 synoGpioPin = U32_MAX;
	int ret = -1;

	if (szSlotName && szLedName) {
		synoGpioPin = syno_led_pin_get(szSlotName, diskPort, szLedName, SYNO_GPIO_PIN);
	}

	if (U32_MAX != synoGpioPin) {
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(syno_led_pin_have);

/**
 * syno_led_name_get - get the szLedName led_name of target slot
 * @szSlotName [IN]:    slot name
 * @diskPort [IN]:      slot number
 * @szLedType [IN]:		LED node name in device node
 *
 */
int  syno_led_name_get(const char* szSlotName, const int diskPort, const char *szLedType, char *szSynoLedName, unsigned int cbSynoLedName)
{
	int iRet = -1;
	struct device_node *pSlotNode = NULL, *pLedNode = NULL;
	char szFullName[MAX_NODENAME_LEN] = {0};
	const char *szLedName = NULL;

	if (NULL == szSlotName || NULL == szLedType || 1 > diskPort || NULL == szSynoLedName) {
		goto END;
	}
	if (0 > snprintf(szFullName, MAX_NODENAME_LEN - 1, "/%s@%d", szSlotName, diskPort)) {
		goto END;
	}

	for_each_child_of_node(of_root, pSlotNode) {
		if (pSlotNode->full_name && 0 == strcmp(pSlotNode->full_name, szFullName)) {
			break;
		}
	}

	if (NULL == pSlotNode) {
		goto END;
	}
	pLedNode = of_get_child_by_name(pSlotNode, szLedType);
	of_node_put(pSlotNode);
	if (NULL == pLedNode) {
		goto END;
	}
	of_property_read_string(pLedNode, DT_HDD_LED_NAME, &szLedName);
	of_node_put(pLedNode);
	if (0 > snprintf(szSynoLedName, cbSynoLedName, "%s", szLedName)) {
		goto END;
	}
	iRet = 0;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_led_name_get);

/**
 * syno_led_type_get - get the szLedType led_type of target slot
 * @szSlotName [IN]:    slot name
 * @diskPort [IN]:      slot number
 *
 */
int  syno_led_type_get(const char* szSlotName, const int diskPort, char *szSynoLedType, unsigned int cbSynoLedType)
{
	int iRet = -1;
	struct device_node *pSlotNode = NULL;
	char szFullName[MAX_NODENAME_LEN] = {0};
	const char *szLedType = NULL;

	if (NULL == szSlotName || 1 > diskPort || NULL == szSynoLedType) {
		goto END;
	}
	if (0 > snprintf(szFullName, MAX_NODENAME_LEN - 1, "/%s@%d", szSlotName, diskPort)) {
		goto END;
	}

	for_each_child_of_node(of_root, pSlotNode) {
		if (pSlotNode->full_name && 0 == strcmp(pSlotNode->full_name, szFullName)) {
			break;
		}
	}

	if (NULL == pSlotNode) {
		goto END;
	}

	of_property_read_string(pSlotNode, DT_HDD_LED_TYPE, &szLedType);
	of_node_put(pSlotNode);
	if (0 > snprintf(szSynoLedType, cbSynoLedType, "%s", szLedType)) {
		goto END;
	}
	iRet = 0;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_led_type_get);

struct i2c_adapter* syno_i2c_adapter_get_by_node(struct device_node *pI2CBusNode)
{
	struct i2c_adapter* adapter = NULL;
	int iBusIdx = 0;

	if (1 != sscanf(pI2CBusNode->full_name, "/"DT_I2C_BUS"@%d", &iBusIdx)) {
		printk("synobios: cannot parse i2c bus index\n");
		goto END;
	}

	adapter = i2c_get_adapter(iBusIdx);

END:
	return adapter;
}
EXPORT_SYMBOL(syno_i2c_adapter_get_by_node);

int syno_pmp_get_ebox_node_by_unique_id(u8 synoUniqueID, u8 isRP, struct device_node **pEBoxNode)
{
	int iRet = -1;
	int i = 0;
	char szUnique[SYNO_EBOX_UNIQUE_MAX_LEN] = {0};

	for (i=0; syno_ebox_unique_mapping[i].uniqueId != 0; i++) {
		if (syno_ebox_unique_mapping[i].uniqueId == (synoUniqueID & syno_ebox_unique_mapping[i].mask)){
			
			if (isRP) {
				snprintf(szUnique, SYNO_EBOX_UNIQUE_MAX_LEN, "%s", syno_ebox_unique_mapping[i].szUnique);
			} else {
				snprintf(szUnique, SYNO_EBOX_UNIQUE_MAX_LEN, "%s", syno_ebox_unique_mapping[i].szUniqueRp);
			}

			if (NULL == ((*pEBoxNode) = of_get_child_by_name(of_root, szUnique))) {
				printk("Get node %s failed\n", szUnique);
				goto END;
			}

			break;
		}
	}

	
	iRet = 0;
END:
	return iRet;
}
EXPORT_SYMBOL(syno_pmp_get_ebox_node_by_unique_id);

int syno_pmp_i2c_addr_get(struct device_node *pNode, unsigned int *addr)
{
	int iRet = -1;
	phandle ph;
	struct device_node *pI2cNode = NULL;

	if (!pNode || !addr) {
		goto END;
	}

	/* Get I2c Device PH */
	if(of_property_read_u32_index(pNode, DT_I2C_DEVICE, 0, &ph)) {
		printk("Get I2c ph failed\n");
		goto END;
	}

	/* Get I2c Device Node*/
	if (NULL == (pI2cNode = of_find_node_by_phandle(ph))) {
		printk("Get I2c Node failed, ph = %u\n", ph);
		goto END;
	}

	/* Get I2c Addr */
	if (0 != of_property_read_u32_index(pI2cNode, DT_I2C_ADDRESS, 0 , addr)) {
		printk("Read i2c_addr failed\n");
		goto END;
	}

	iRet = 0;
END:
	return iRet;
}

EXPORT_SYMBOL(syno_pmp_i2c_addr_get);

#endif /* MY_DEF_HERE */
