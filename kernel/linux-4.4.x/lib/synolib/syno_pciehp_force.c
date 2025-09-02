#include <linux/synolib.h>
#include <linux/of.h>
#include <linux/pci.h>

int syno_pciehp_force_check(const char * name)
{
	int ret = 0;
	int i, num;
	struct device_node *pDeviceNode = NULL;
	const char *strings;

	if (NULL == name || NULL == of_root) {
		goto END;
	}

	for_each_child_of_node(of_root, pDeviceNode) {
		if (NULL == pDeviceNode->full_name) {
			continue;
		}
		if (0 != strncmp(pDeviceNode->full_name, "/"DT_PCIEHP_FORCE, strlen("/"DT_PCIEHP_FORCE))) {
			continue;
		}
		if (!of_find_property(pDeviceNode, DT_ROOT_LIST, NULL)) {
			continue;
		}

		num = of_property_count_strings(pDeviceNode, DT_ROOT_LIST);
		for (i = 0; i < num; ++i) {
			if (0 != of_property_read_string_index(pDeviceNode, DT_ROOT_LIST, i, &strings))
				continue;
			if (strnstr(name, strings, strlen(strings))) {
				pr_info("%s found and force pciehp", name);
				ret = 1;
				goto END;
			}
		}

	}

END:
        return ret;
}
EXPORT_SYMBOL(syno_pciehp_force_check);
