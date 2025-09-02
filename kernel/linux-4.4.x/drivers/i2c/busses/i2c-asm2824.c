/* Copyright (c) 2000-2021 Synology Inc. All rights reserved. */
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/synolib.h>

typedef unsigned char BYTE;
typedef unsigned char* PBYTE;

#define ASM2824_I2C_CTRL_STS_REG	0xF8 //USP PCI offset F8h: Control/Status register
#define ASM2824_I2C_SLAVE_ADDR_REG	0xF9 //USP PCI offset F9h: Slave Address register
#define ASM2824_I2C_COMMAND_REG		0xFA //USP PCI offset FAh: Command Code register
#define ASM2824_I2C_DATA_REG		0xFB //USP PCI offset FBh: Data register

struct asm2824_priv {
	struct i2c_adapter adapter;
	struct pci_dev *pci_dev;
	struct mutex xfer_lock;
	int busno;
};

typedef union _I2C_CTRL_STS_ {
	struct {
		BYTE	Run : 1;	//bit[0]: run
		BYTE	RW : 1;		//bit[1]: read/write protocol
		BYTE	Rsvd : 2;	//bit[3:2]: reserved
		BYTE	Err : 4;	//bit[7:4]: error status
	};
	BYTE	AsByte;
} I2C_CTRL_STS;

//******************************************************************************
// Asm2824I2cReadByte
// 		Read Byte Protocol
// Input:	Bus	- ASM2824 USP PCI bus nu mber
//		Dev	- ASM2824 USP PCI device number
//		Fun	- ASM2824 USP PCI function number
//		Addr	- I2C slave address. Bit[6:0] is 7 bits slave address
//		Cmd	- I2C command code
//		pData	- Data buffer for read byte
// Output:	TRUE	- I2C operation success
//		FALSE	- Error occurred
//******************************************************************************
bool Asm2824I2cReadByte(struct asm2824_priv *priv, BYTE Addr, BYTE Cmd, PBYTE pData)
{
	int i, ret = -1;
	int retry = priv->adapter.timeout * 1000 / HZ / 10;
	I2C_CTRL_STS CtrlSts;
	struct pci_dev *dev = priv->pci_dev;

	//Write slave address
	pci_write_config_byte(dev, ASM2824_I2C_SLAVE_ADDR_REG, Addr);
	//Write command code
	pci_write_config_byte(dev, ASM2824_I2C_COMMAND_REG, Cmd);
	//Start I2C process
	CtrlSts.AsByte = 0; //Clear variable
	CtrlSts.RW = 0; //Read byte protocol
	CtrlSts.Run = 1; //Run
	pci_write_config_byte(dev, ASM2824_I2C_CTRL_STS_REG, CtrlSts.AsByte);

	for (i = 0; i < retry; ++i)
	{
		pci_read_config_byte(dev, ASM2824_I2C_CTRL_STS_REG, &CtrlSts.AsByte);
		if (CtrlSts.Err != 0) {
			ret = -EIO;
			goto END;
		}
		if (CtrlSts.Run == 0)
			break;
		msleep(10);
	}
	if (i == retry) {
		ret = -EIO;
		goto END;
	}
	//Read byte data
	pci_read_config_byte(dev, ASM2824_I2C_DATA_REG, pData);
	ret = 0;
END:
	return ret;
}

//*************************************************************************************************************
// Asm2824I2cWriteByte
//		Read Byte Protocol
// Input:	Bus	- ASM2824 USP PCI bus number
//		Dev	- ASM2824 USP PCI device number
//		Fun	- ASM2824 USP PCI function number
//		Addr	- I2C slave address. Bit[6:0] is 7 bits slave address
//		Cmd	- I2C command code
//		Data	- Byte data
// Output:	0	- I2C operation success
//		others	- Error occurred
//***
int Asm2824I2cWriteByte (struct asm2824_priv *priv, BYTE Addr, BYTE Cmd, BYTE Data)
{
	int i, ret = -1;
	int retry = priv->adapter.timeout * 1000 / HZ / 10;
	I2C_CTRL_STS CtrlSts;
	struct pci_dev *dev = priv->pci_dev;

	//Write slave address
	pci_write_config_byte(dev, ASM2824_I2C_SLAVE_ADDR_REG, Addr);
	//Write command code
	pci_write_config_byte(dev, ASM2824_I2C_COMMAND_REG, Cmd);
	//Write data
	pci_write_config_byte(dev, ASM2824_I2C_DATA_REG, Data);
	//Start I2C process
	CtrlSts.AsByte = 0; //Clear variable
	CtrlSts.RW = 1; //Write byte protocol
	CtrlSts.Run = 1; //Run
	pci_write_config_byte(dev, ASM2824_I2C_CTRL_STS_REG, CtrlSts.AsByte);
	for (i = 0; i < retry; ++i)
	{
		pci_read_config_byte(dev, ASM2824_I2C_CTRL_STS_REG, &CtrlSts.AsByte);
		if (CtrlSts.Err != 0) {
			ret = -EIO;
			goto END;
		}
		if (CtrlSts.Run == 0)
			break;
		msleep(10);
	}
	if (i == retry) {
		ret = -EIO;
		goto END;
	}
	ret = 0;
END:
	return ret;
}

/* Return negative errno on error. */
static s32 asm2824_access(struct i2c_adapter *adap, u16 addr,
		unsigned short flags, char read_write, u8 command,
		int size, union i2c_smbus_data *data)
{
	int ret = 0;
	BYTE Addr;
	struct asm2824_priv *priv = i2c_get_adapdata(adap);

	mutex_lock(&priv->xfer_lock);
	if (flags & I2C_CLIENT_PEC) {
		goto out;
	}

	if (size != I2C_SMBUS_BYTE_DATA) {
		goto out;
	}

	Addr = addr & 0x7f;
	if (read_write == I2C_SMBUS_WRITE) {
		ret = Asm2824I2cWriteByte(priv, Addr,
				command, data->byte);
	} else {
		ret = Asm2824I2cReadByte(priv, Addr,
				command, &data->byte);
	}

out:
	mutex_unlock(&priv->xfer_lock);
	return ret;
}

static u32 asm2824_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA;
}

static const struct i2c_algorithm asm2824_smbus_algorithm = {
	.smbus_xfer     = asm2824_access,
	.functionality  = asm2824_func,
};

static inline unsigned int asm2824_get_adapter_class(struct asm2824_priv *priv)
{
	return I2C_CLASS_HWMON;
}

static int asm2824_probe(struct platform_device *pdev)
{
	int err;
	struct asm2824_priv *priv;
	struct asm2824_pdata *pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(struct asm2824_priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	strlcpy(priv->adapter.name, "ASM2824 I2C adapter", sizeof(priv->adapter.name));
	i2c_set_adapdata(&priv->adapter, priv);
	priv->adapter.owner = THIS_MODULE;
	priv->adapter.class = asm2824_get_adapter_class(priv);
	priv->adapter.algo = &asm2824_smbus_algorithm;
	priv->adapter.dev.parent = &pdev->dev;
	priv->adapter.retries = 3;
	/* Default timeout in interrupt mode: 200 ms */
	priv->adapter.timeout = HZ / 5;
	priv->adapter.nr = pdev->id;
	priv->pci_dev = pdata->pci_dev;

	mutex_init(&priv->xfer_lock);

	err = i2c_add_numbered_adapter(&priv->adapter);
	if (err) {
		dev_err(&pdev->dev, "Failed to add SMBus adapter\n");
		return err;
	}
	platform_set_drvdata(pdev, priv);
	return 0;
}

static int asm2824_remove(struct platform_device *pdev)
{
	struct asm2824_priv *priv = platform_get_drvdata(pdev);

	if (priv) {
		i2c_del_adapter(&priv->adapter);
	}

	return 0;
}

static struct platform_driver asm2824_driver = {
	.driver = {
		.name = "asm2824-i2c",
	},
	.probe	= asm2824_probe,
	.remove	= asm2824_remove,
};

static int __init i2c_asm2824_init(void)
{
	return platform_driver_register(&asm2824_driver);
}

static void __exit i2c_asm2824_exit(void)
{
	platform_driver_unregister(&asm2824_driver);
}

MODULE_AUTHOR("Jason Peng <jasonpeng@synology.com>");
MODULE_DESCRIPTION("ASM2824 SMBus driver");
MODULE_LICENSE("GPL");

module_init(i2c_asm2824_init);
module_exit(i2c_asm2824_exit);
