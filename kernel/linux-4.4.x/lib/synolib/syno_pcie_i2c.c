#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/* Copyright (c) 2000-2021 Synology Inc. All rights reserved. */
#include <linux/synolib.h>
#include <linux/of.h>
#include <linux/pci.h>

extern int syno_pciepath_dts_pattern_get(struct pci_dev *pdev, char *szPciePath, const int size);

/*
 * Check if i2c node support this eunit name
 *
 * @param pI2CNode   [IN] I2C device node
 * @param pEunitNode [IN] Eunit device node
 *
 * Return true : support
 *        false: unsupport
 */
static bool syno_eunit_name_cmp(struct device_node *pI2CNode, struct device_node *pEunitNode)
{
	int i, num;
	bool blRet = false;
	const char *strings;

	if (!pI2CNode || !pEunitNode) {
		goto END;
	}
	if (!of_find_property(pI2CNode, DT_EUNIT, NULL)) {
		goto END;
	}
	num = of_property_count_strings(pI2CNode, DT_EUNIT);
	for (i = 0; i < num; ++i) {
		if (0 != of_property_read_string_index(pI2CNode, DT_EUNIT, i, &strings))
			continue;
		if (0 == strncmp(strings, pEunitNode->name, strlen(pEunitNode->name))) {
			blRet = true;
			goto END;
		}
	}
END:
	return blRet;
}

/*
 * Return if the pci_dev support ASM2824 I2C and corresponding bus number.
 *
 * @param pdev [IN] PCI device structure
 *
 * Return -1 : not found
 *        others: found and return value is bus number
 */
int syno_pci_dev_to_i2c_bus(struct pci_dev *pdev)
{
	int ret = -1;
	int index;
	struct device_node *pDeviceNode = NULL;
	struct device_node *pEunitNode = NULL;
	char *pcie_root = NULL;

	if (NULL == pdev || NULL == of_root) {
		goto END;
	}

	for_each_child_of_node(of_root, pDeviceNode) {
		if (NULL == pDeviceNode->full_name) {
			continue;
		}
		if (0 != strncmp(pDeviceNode->full_name, "/"DT_I2C_BUS, strlen("/"DT_I2C_BUS))) {
			continue;
		}
		pcie_root = (char *)of_get_property(pDeviceNode, DT_PCIE_ROOT, NULL);
		if (NULL == pcie_root) {
			continue;
		}

		if (strncmp(pcie_root, kobject_name(&pdev->dev.kobj), strlen(pcie_root))) {
			continue;
		}
		pEunitNode = syno_pci_dev_to_eunit_node(pdev);
		if(NULL == pEunitNode) {
			continue;
		}
		if (!syno_pciid_list_cmp(pdev, pEunitNode)) {
			continue;
		}
		if (!syno_eunit_name_cmp(pDeviceNode, pEunitNode)) {
			continue;
		}
		if (1 == sscanf(pDeviceNode->full_name, "/"DT_I2C_BUS"@%d", &index)) {
			ret = index;
			goto END;
		}
	}
END:
	return ret;
}

/*
 * Return corresponding eunit index for pci dev
 *
 * @param pdev [IN] PCI device strct
 *
 * Return -1 : not found
 *        others: found and return pointer to device node
 */
int syno_pci_dev_to_eunit_index(struct pci_dev* pdev)
{
	int iRet = -1;
	int index;
	struct device_node *pDeviceNode = NULL;
	char *szPcieRoot = NULL;
	char szDevPciePath[SYNO_DTS_PROPERTY_CONTENT_LENGTH] = {'\0'};
	if(!pdev || !of_root) {
		goto END;
	}
	if (-1 == syno_pciepath_dts_pattern_get(pdev, szDevPciePath, SYNO_DTS_PROPERTY_CONTENT_LENGTH)) {
		goto END;
	}
	for_each_child_of_node(of_root, pDeviceNode) {
		if (!pDeviceNode->full_name) {
			continue;
		}

		if (strncmp(pDeviceNode->full_name, "/"DT_PCIE_SLOT, strlen("/"DT_PCIE_SLOT))) {
			continue;
		}
		szPcieRoot = (char *)of_get_property(pDeviceNode, DT_PCIE_ROOT, NULL);
		if(!szPcieRoot) {
			continue;
		}
		if (strncmp(szDevPciePath, szPcieRoot, strlen(szPcieRoot))) {
			continue;
		}
		if (0 == of_property_read_u32_index(pDeviceNode, DT_PCIE_EUNIT_PORT, 0, &index)) {
			iRet = index;
		}
	}
END:
	return iRet;
}

#ifdef MY_DEF_HERE
extern int syno_eunit_led_number_fill(int eunit_index, int iLedNum);
extern int syno_eunit_led_remap_clear(int eunit_index);
extern int syno_eunit_led_number_clear(int eunit_index);
extern int syno_eunit_active_led_remap_fill(int eunit_index, int disk_index, int iLedNum);
extern int syno_eunit_faulty_led_remap_fill(int iEunitNum, int disk_index, int iLedNum);

/*
 * Fill led remap info for specific disk
 *
 * @param eunit_index [IN] eunit index of disk
 * @param disk_index  [IN] disk index
 * @param pDeviceNode [IN] device node pointer to disk
 *
 */
static void syno_led_remap_do_fill(int eunit_index, int disk_index, struct device_node *pDiskNode)
{
	int led_index;
	char *led_name = NULL;
	const char *temp;
	struct device_node *pLedNode = NULL;
	if (!pDiskNode) {
		return;
	}
	for_each_child_of_node(pDiskNode, pLedNode) {
		if (!pLedNode->full_name) {
			continue;
		}
		led_name = (char *)of_get_property(pLedNode, DT_HDD_LED_NAME, NULL);
		if (!led_name) {
			continue;
		}
		// LED name use '%d' to indicate eunit index
		if(1 != sscanf(led_name, "syno_eunit%%d_led%d", &led_index)) {
			continue;
		}
		// ex: LED node /FX2422N/m2_card@1/led_green, disk node /FX2422N/m2_card@1
		// so LED node + string length + 1 = led_green
		temp = pLedNode->full_name+strlen(pDiskNode->full_name)+1;
		if (0 == strncmp(temp, DT_HDD_GREEN_LED, strlen(DT_HDD_GREEN_LED))) {
			syno_eunit_active_led_remap_fill(eunit_index, disk_index, led_index);
			continue;
		}
		if (0 == strncmp(temp, DT_HDD_ORANGE_LED, strlen(DT_HDD_ORANGE_LED))) {
			syno_eunit_faulty_led_remap_fill(eunit_index, disk_index, led_index);
			continue;
		}
	}
}

/*
 * Fill Eunit LED remap info
 *
 * @param pdev [IN] PCI device struct
 */
void syno_add_eunit_led_remap(struct pci_dev* pdev)
{
	const char *temp = NULL;
	int eunit_index, disk_index, led_num;
	struct device_node *pEunitNode = NULL;
	struct device_node *pDeviceNode;

	if(!pdev || !of_root) {
		return;
	}
	eunit_index = syno_pci_dev_to_eunit_index(pdev);
	if (eunit_index < 0) {
		return;
	}
	pEunitNode = syno_pci_dev_to_eunit_node(pdev);
	if (0 != of_property_read_u32_index(pEunitNode, DT_NUMBER_OF_LED_TRIGGER, 0, &led_num)) {
		return;
	}
	syno_eunit_led_number_fill(eunit_index, led_num);
	for_each_child_of_node(pEunitNode, pDeviceNode) {
		temp = pDeviceNode->full_name+strlen(pEunitNode->full_name)+1;
		if (strncmp(temp, DT_M2_CARD, strlen(DT_M2_CARD))) {
			continue;
		}
		if (1 != sscanf(temp, DT_M2_CARD"@%d", &disk_index)) {
			continue;
		}
		syno_led_remap_do_fill(eunit_index, disk_index, pDeviceNode);
	}
}

/*
 * Clear LED remap info
 *
 * @param pdev [IN] PCI device struct
 */
void syno_del_eunit_led_remap(struct pci_dev* pdev)
{
	int eunit_index;

	if(!pdev || !of_root) {
		return;
	}
	eunit_index = syno_pci_dev_to_eunit_index(pdev);
	if (eunit_index < 0) {
		return;
	}
	syno_eunit_led_remap_clear(eunit_index);
	syno_eunit_led_number_clear(eunit_index);
}

/*
 * Find Eunit index by i2c bus number
 *
 * @param pI2CNode [IN] device node structure to i2c node
 *
 * Return 0  : Failed to find
 *        >0 : success and return value is eunit index
 */
int syno_i2c_bus_to_eunit_index(struct device_node *pI2CNode)
{
	int bus, dev, fun;
	int iRet = 0, index;
	struct pci_dev *pdev = NULL;
	char *pcie_root = NULL;

	if (!pI2CNode || !of_root) {
		goto END;
	}

	pcie_root = (char*)of_get_property(pI2CNode, DT_PCIE_ROOT, NULL);
	if (!pcie_root) {
		goto END;
	}

	if (3 != sscanf(pcie_root, "0000:%x:%x.%x", &bus, &dev, &fun)) {
		goto END;
	}

	pdev = pci_get_bus_and_slot(bus, PCI_DEVFN(dev, fun));
	if(!pdev) {
		goto END;
	}
	index = syno_pci_dev_to_eunit_index(pdev);
	if (index < 0) {
		goto END;
	}
	iRet = index;
END:
	if (pdev) {
		pci_dev_put(pdev);
	}
	return iRet;
}
EXPORT_SYMBOL(syno_i2c_bus_to_eunit_index);
#endif /* MY_DEF_HERE */
