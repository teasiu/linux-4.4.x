#!/bin/sh

SYNOINFO_DEF="/etc.defaults/synoinfo.conf"
# load usbhid for bromolow
SUPPORT_DUAL_HEAD=`get_key_value $SYNOINFO_DEF support_dual_head`
if [ "$SUPPORT_DUAL_HEAD" != "yes" ]; then
	modprobe usblp
	modprobe hid
	modprobe usbhid
	modprobe hid-generic
fi

# load synobios
modprobe synobios system_mode=1
/bin/mknod /dev/synobios c 201 0 2>/dev/null

# load syno_hddmon
SUPPORT_HDD_DYNAMIC_POWER=`get_key_value $SYNOINFO_DEF HddEnableDynamicPower`
if [ "$SUPPORT_HDD_DYNAMIC_POWER" = "yes" ]; then
	modprobe syno_hddmon
fi

# load syno_smbus_hddmon
SUPPORT_SMBUS_HDD_ENABLE=`get_key_value $SYNOINFO_DEF SMBusHddEnable`
if [ "$SUPPORT_SMBUS_HDD_ENABLE" = "yes" ]; then
	modprobe syno_smbus_hddmon
fi

# load intel aesni
SUPPORT_AESNI_INTEL=`get_key_value $SYNOINFO_DEF support_aesni_intel`
if [ "$SUPPORT_AESNI_INTEL" == "yes" ]; then
	modprobe ablk_helper
	modprobe gf128mul
	modprobe lrw
	modprobe glue_helper
	modprobe aesni-intel
fi

# load lp3943
SUPPORT_LP3943=`get_key_value $SYNOINFO_DEF support_leds_lp3943`
if [ "$SUPPORT_LP3943" == "yes" ]; then
	modprobe i2c-i801
	modprobe leds-lp3943
fi

CheckATMEGA1608Device()
{
	file=/sys/bus/i2c/devices/
	for f in $file*
	do
		name=`cat $f/name`
		if [ "$name" == "atmega1608" ]; then
			if [ ! -d $f/driver ]; then
				# if any atmega1608 not is probed by driver, return false
				return 0
			fi
		fi
	done
	return 1
}

# load atemga1608
SUPPORT_ATMEGA1608=`get_key_value $SYNOINFO_DEF support_leds_atmega1608`
if [ "$SUPPORT_ATMEGA1608" == "yes" ]; then
	modprobe leds-atmega1608
	for retry_count in `seq 1 3`
	do
		CheckATMEGA1608Device
		if [ $? -eq 0 ]; then
			echo Failed to probe Atmega1608 device, Retry: $retry_count...
			sleep 1
			modprobe --remove leds-atmega1608
			sleep 1
			modprobe leds-atmega1608
		else
			break
		fi
	done
fi

# load IPMI
SUPPORT_IPMI=`get_key_value $SYNOINFO_DEF support_ipmi`
if [ "$SUPPORT_IPMI" == "yes" ]; then
	modprobe ipmi_msghandler
	modprobe ipmi_devintf
	modprobe ipmi_si
fi

# load ACM
SUPPORT_ACM=`get_key_value $SYNOINFO_DEF support_acm`
if [ "$SUPPORT_ACM" == "yes" ]; then
	modprobe cdc-acm
	mknod /dev/ttyACM0 c 166 0
fi

# load usb-storage and uas
if [ -f "/sys/bus/usb/syno_all_usb_uas_enabled" ]; then
	SUPPORT_UASP=`get_key_value $SYNOINFO_DEF support_uasp`
	if [ "$SUPPORT_UASP" == "yes" ]; then
		echo 1 > /sys/bus/usb/syno_all_usb_uas_enabled
	else
		echo 0 > /sys/bus/usb/syno_all_usb_uas_enabled
	fi
fi

modprobe usb-storage
if [ -f "/sys/bus/usb/syno_all_usb_uas_enabled" ]; then
	modprobe uas
fi
