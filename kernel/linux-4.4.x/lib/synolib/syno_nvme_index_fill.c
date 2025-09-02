#include <linux/pci.h>
#include <linux/synolib.h>
#include <linux/of.h>
#include <linux/device.h>

extern int syno_compare_dts_pciepath(struct pci_dev *pdev, const struct device_node *pDeviceNode);

/*
 * Return nvme internal slot index for pci dev
 *
 * @param pdev            [IN] PCI device structure
 * @param syno_block_info [IN] synology block infomation
 *
 * Return -1 : not found
 *        others: found and return slot index
 */
int syno_nvme_index_get(struct pci_dev *pdev, char *syno_block_info)
{
	int iIndex = 0;
	struct device_node *pDeviceNode = NULL;

	if (NULL == pdev || NULL == of_root) {
		goto END;
	}

	for_each_child_of_node(of_root, pDeviceNode) {
		if (pDeviceNode->full_name && 0 == strncmp(pDeviceNode->full_name, "/"DT_INTERNAL_SLOT, strlen("/"DT_INTERNAL_SLOT))) {
			/* skip non-internal nvme device */
			if (!of_find_property(pDeviceNode, DT_PCIE_ROOT, NULL)) {
				continue;
			}

			if (0 == syno_compare_dts_pciepath(pdev, pDeviceNode)) {
				// get index number of nvme_slot, e.g. /internal_slot@4 --> 4
				sscanf(pDeviceNode->full_name, "/"DT_INTERNAL_SLOT"@%d", &iIndex);
				break;
			}
		}
	}
END:
	// convert one base to zero base
	return iIndex-1;
}

/*
 * Compare PCI ID in device node and pci dev
 *
 * @param pdev        [IN] PCI device structure
 * @param pDeviceNode [IN] device node to check
 *
 * Return true : the same
 *        false: different
 */
bool syno_pciid_list_cmp(struct pci_dev *pdev, struct device_node *pDeviceNode)
{
	int i, num;
	u32 id_list[4];
	bool blRet = false;

	if (!pdev || !pDeviceNode) {
		goto END;
	}
	num = of_property_count_u32_elems (pDeviceNode, DT_PCIID_LIST);
	if (num != 4) {
		goto END;;
	}
	for (i = 0; i < num; ++i) {
		if (of_property_read_u32_index(pDeviceNode, DT_PCIID_LIST, i, &id_list[i])) {
			goto END;
		}
	}
	if ((pdev->vendor == (id_list[0]&0xffff))
			&& (pdev->device == (id_list[1]&0xffff))
			&& (pdev->subsystem_vendor == (id_list[2]&0xffff))
			&& (pdev->subsystem_device == (id_list[3]&0xffff))) {
		blRet = true;
	}
END:
	return blRet;
}

/*
 * Return corresponding eunit for pci dev by comparing PCI ID
 *
 * @param pdev [IN] PCI device structure
 *
 * Return NULL : not found
 *        others: found and return pointer to device node
 */
struct device_node * syno_pci_dev_to_eunit_node(struct pci_dev* pdev)
{
	struct device_node *pDeviceNode = NULL;
	struct device_node *pRet = NULL;

	if(!pdev || !of_root) {
		goto END;
	}
	for_each_child_of_node(of_root, pDeviceNode) {
		if (!syno_pciid_list_cmp(pdev, pDeviceNode)) {
			continue;
		}
		pRet = pDeviceNode;
		goto END;
	}

END:
	return pRet;
}

/*
 * Compare syno_block_info and device node pcie_postfix
 *
 * @param syno_block_info [IN] synology block info
 * @param pEunitNode      [IN] device node to eunit
 *
 * Return true : the same
 *        false: different
 */
static bool syno_eunit_disk_pciepath_cmp(char *syno_block_info, const struct device_node *pEunitNode)
{
	struct device_node *pU2Node = NULL;
	bool blRet = false;
	char *szPciePostfix = NULL;

	if (!syno_block_info || !pEunitNode) {
		goto END;
	}
	for_each_child_of_node(pEunitNode, pU2Node) {
		if (!pU2Node->full_name) {
			continue;
		}
		if (strncmp(pU2Node->full_name+strlen(pEunitNode->full_name)+1, DT_NVME, strlen(DT_NVME))) {
			continue;
		}

		szPciePostfix = (char *)of_get_property(pU2Node, DT_PCIE_POSTFIX, NULL);
		if (!szPciePostfix) {
			continue;
		}
		if (strstr(syno_block_info, szPciePostfix)) {
			blRet = true;
			goto END;
		}
	}

END:
	return blRet;
}

/*
 * Get NVMe eunit disk index
 *
 * @param pdev            [IN] PCI device structure
 * @param syno_block_info [IN] synology block info
 *
 * Return 0      : cannot parse
 *        oethers: found and return disk index
 */
int syno_eunit_disk_index_get(struct pci_dev *pdev, char *syno_block_info)
{
	int iRet = 0;
	int disk_index;
	struct device_node *pDiskNode = NULL;
	struct device_node *pEunitNode = NULL;
	struct pci_dev *upstream;

	if (!syno_block_info || !pdev || !of_root) {
		goto END;
	}

	upstream = pci_upstream_bridge(pdev);
	if (!upstream) {
		goto END;
	}

	pEunitNode = syno_pci_dev_to_eunit_node(upstream);
	if (NULL == pEunitNode) {
		goto END;
	}

	for_each_child_of_node(pEunitNode, pDiskNode) {
		if (!pDiskNode->full_name) {
			continue;
		}
		if (strncmp(pDiskNode->full_name+strlen(pEunitNode->full_name)+1, DT_M2_CARD, strlen(DT_M2_CARD))) {
			continue;
		}
		if(!syno_eunit_disk_pciepath_cmp(syno_block_info, pDiskNode)) {
			continue;
		}
		if(1 == sscanf(pDiskNode->full_name+strlen(pEunitNode->full_name)+1,
					DT_M2_CARD"@%d", &disk_index)) {
			iRet = disk_index;
			goto END;
		}
	}
END:
	return iRet;
}

/*
 * Get NVMe eunit index
 *
 * @param pdev            [IN] PCI device structure
 * @param syno_block_info [IN] synology block info
 *
 * Return 0      : cannot parse
 *        oethers: found and return disk index
 */
int syno_eunit_index_get(struct pci_dev *pdev, char *syno_block_info)
{
	int iRet = 0, index;
	struct device_node *pDeviceNode = NULL;
	char *szPciePath = NULL;
	char *szCursor = NULL;

	if (!syno_block_info || !of_root) {
		goto END;
	}

	for_each_child_of_node(of_root, pDeviceNode) {
		if (!pDeviceNode->full_name) {
			continue;
		}
		if (0 != strncmp(pDeviceNode->full_name, "/"DT_PCIE_SLOT, strlen("/"DT_PCIE_SLOT))) {
			/* skip non-internal nvme device */
			continue;
		}
		if (!of_find_property(pDeviceNode, DT_PCIE_ROOT, NULL)) {
			continue;
		}
		szPciePath = (char *)of_get_property(pDeviceNode, DT_PCIE_ROOT, NULL);
		if (!szPciePath) {
			continue;
		}
		szCursor = syno_block_info;
		if (0 != strncmp(szCursor, "pciepath=", strlen("pciepath="))) {
			continue;
		}
		szCursor = szCursor + strlen("pciepath=");
		if (0 != strncmp(szCursor, szPciePath, strlen(szPciePath))) {
			continue;
		}
		szCursor = szCursor + strlen(szPciePath);
		if (0 == of_property_read_u32_index(pDeviceNode, DT_PCIE_EUNIT_PORT, 0, &index)) {
			iRet = index;
			goto END;
		}
	}
END:
	return iRet;
}
