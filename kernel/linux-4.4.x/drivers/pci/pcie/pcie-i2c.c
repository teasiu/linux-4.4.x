#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/* Copyright (c) 2000-2021 Synology Inc. All rights reserved. */
#include <linux/pci.h>
#include <linux/pcieport_if.h>
#include <linux/platform_device.h>
#include <linux/synolib.h>

#define ASM2824_I2C_NUM_MAX 8
static struct platform_device *asm2824_device[ASM2824_I2C_NUM_MAX];

static int i2c_probe(struct pcie_device *dev)
{
	int busno, err = 0;
	struct asm2824_pdata data;

	busno = syno_pci_dev_to_i2c_bus(dev->port);
	if (busno < 0 || busno >= ASM2824_I2C_NUM_MAX) {
		err = -EINVAL;
		goto END;
	}
	asm2824_device[busno] = platform_device_alloc("asm2824-i2c", busno);
#ifdef MY_DEF_HERE
	syno_add_eunit_led_remap(dev->port);
#endif /* MY_DEF_HERE */

	memset(&data, 0, sizeof(data));
	data.pci_dev = dev->port;
	err = platform_device_add_data(asm2824_device[busno], &data, sizeof(data));
	if (err) {
		pr_err ("platform_device_add_data failed!");
		goto END;
	}

	err = platform_device_add(asm2824_device[busno]);
	if (err) {
		pr_err ("platform_device_add failed!\n");
		goto END;
	}

	return 0;
END:
	if (asm2824_device[busno]) {
		platform_device_put (asm2824_device[busno]);
	}
	return err;
}

static void i2c_remove(struct pcie_device *dev)
{
	int busno = syno_pci_dev_to_i2c_bus(dev->port);

	if (busno < 0 || busno >= ASM2824_I2C_NUM_MAX) {
		return;
	}
#ifdef MY_DEF_HERE
	syno_del_eunit_led_remap(dev->port);
#endif /* MY_DEF_HERE */

	platform_device_unregister(asm2824_device[busno]);
	asm2824_device[busno] = NULL;
}

static struct pcie_port_service_driver i2cdriver = {
	.name		= "i2c",
	.port_type	= PCIE_ANY_PORT,
	.service	= PCIE_PORT_SERVICE_I2C,
	.probe		= i2c_probe,
	.remove		= i2c_remove,
};

static int __init i2c_service_init(void)
{
	memset(asm2824_device, 0, sizeof(asm2824_device));
	return pcie_port_service_register(&i2cdriver);
}

static void __exit i2c_service_exit(void)
{
	pcie_port_service_unregister(&i2cdriver);
}

MODULE_DESCRIPTION("PCI Express I2C service driver");
MODULE_AUTHOR("Jason Peng <jasonpeng@synology.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

module_init(i2c_service_init);
module_exit(i2c_service_exit);
