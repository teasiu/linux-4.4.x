#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * cdc-acm.c
 *
 * Copyright (c) 1999 Armin Fuerst	<fuerst@in.tum.de>
 * Copyright (c) 1999 Pavel Machek	<pavel@ucw.cz>
 * Copyright (c) 1999 Johannes Erdfelt	<johannes@erdfelt.com>
 * Copyright (c) 2000 Vojtech Pavlik	<vojtech@suse.cz>
 * Copyright (c) 2004 Oliver Neukum	<oliver@neukum.name>
 * Copyright (c) 2005 David Kubicek	<dave@awk.cz>
 * Copyright (c) 2011 Johan Hovold	<jhovold@gmail.com>
 *
 * USB Abstract Control Model driver for USB modems and ISDN adapters
 *
 * Sponsored by SuSE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#undef DEBUG
#undef VERBOSE_DEBUG

#ifdef MY_DEF_HERE
//TODO:check headers dependency, move the segment to below
#include <linux/libata.h>
#include <scsi/scsi_device.h>
#include <linux/synolib.h>
#include <linux/synobios.h>
#define SYNO_EUNIT_READY_RETRY 5
#define SYNO_EUNIT_ACM_WAITING_READY 100
#define SYNO_EUNIT_STATUS_REPORT_DELIM ','
#define SYNO_EUNIT_STATUS_BUFFER_SIZE 1024

struct acm_device_temp {
	char disk_name[DISK_NAME_LEN];
	char usb_path[SYNO_DTS_PROPERTY_CONTENT_LENGTH];
	struct list_head device_list;
};

static LIST_HEAD(acm_temp_device_list);

extern void syno_disk_not_ready_count_increase(void);
extern void syno_disk_not_ready_count_decrease(void);
#endif /* MY_DEF_HERE */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <linux/list.h>

#include "cdc-acm.h"


#define DRIVER_AUTHOR "Armin Fuerst, Pavel Machek, Johannes Erdfelt, Vojtech Pavlik, David Kubicek, Johan Hovold"
#define DRIVER_DESC "USB Abstract Control Model driver for USB modems and ISDN adapters"

static struct usb_driver acm_driver;
static struct tty_driver *acm_tty_driver;

static DEFINE_IDR(acm_minors);
static DEFINE_MUTEX(acm_minors_lock);

static void acm_tty_set_termios(struct tty_struct *tty,
				struct ktermios *termios_old);

#ifdef MY_DEF_HERE
static DEFINE_SPINLOCK(acm_list_lock);
struct syno_acm_list {
	struct acm* acm;
	struct list_head device_list;
};
static LIST_HEAD(syno_acm_list_head);

static bool syno_is_synology_acm(struct acm *acm)
{
	if (acm && 0 == strncmp(interface_to_usbdev(acm->control)->product, SYNO_EUNIT_NAME_HEAD, strlen(SYNO_EUNIT_NAME_HEAD))) {
		return true;
	}
	return false;
}
#endif /* MY_DEF_HERE */

/*
 * acm_minors accessors
 */

/*
 * Look up an ACM structure by minor. If found and not disconnected, increment
 * its refcount and return it with its mutex held.
 */
static struct acm *acm_get_by_minor(unsigned int minor)
{
	struct acm *acm;

	mutex_lock(&acm_minors_lock);
	acm = idr_find(&acm_minors, minor);
	if (acm) {
		mutex_lock(&acm->mutex);
		if (acm->disconnected) {
			mutex_unlock(&acm->mutex);
			acm = NULL;
		} else {
			tty_port_get(&acm->port);
			mutex_unlock(&acm->mutex);
		}
	}
	mutex_unlock(&acm_minors_lock);
	return acm;
}

/*
 * Try to find an available minor number and if found, associate it with 'acm'.
 */
static int acm_alloc_minor(struct acm *acm)
{
	int minor;

	mutex_lock(&acm_minors_lock);
	minor = idr_alloc(&acm_minors, acm, 0, ACM_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&acm_minors_lock);

	return minor;
}

/* Release the minor number associated with 'acm'.  */
static void acm_release_minor(struct acm *acm)
{
	mutex_lock(&acm_minors_lock);
	idr_remove(&acm_minors, acm->minor);
	mutex_unlock(&acm_minors_lock);
}

/*
 * Functions for ACM control messages.
 */

static int acm_ctrl_msg(struct acm *acm, int request, int value,
							void *buf, int len)
{
	int retval;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		return retval;

	retval = usb_control_msg(acm->dev, usb_sndctrlpipe(acm->dev, 0),
		request, USB_RT_ACM, value,
		acm->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);

	dev_dbg(&acm->control->dev,
			"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
			__func__, request, value, len, retval);

	usb_autopm_put_interface(acm->control);

	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */
static inline int acm_set_control(struct acm *acm, int control)
{
	if (acm->quirks & QUIRK_CONTROL_LINE_STATE)
		return -EOPNOTSUPP;

	return acm_ctrl_msg(acm, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			control, NULL, 0);
}

#define acm_set_line(acm, line) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line))
#define acm_send_break(acm, ms) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int acm_wb_alloc(struct acm *acm)
{
	int i, wbn;
	struct acm_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &acm->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW)
			return -1;
	}
}

static int acm_wb_is_avail(struct acm *acm)
{
	int i, n;
	unsigned long flags;

	n = ACM_NW;
	spin_lock_irqsave(&acm->write_lock, flags);
	for (i = 0; i < ACM_NW; i++)
		n -= acm->wb[i].use;
	spin_unlock_irqrestore(&acm->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold acm->write_lock
 */
static void acm_write_done(struct acm *acm, struct acm_wb *wb)
{
	wb->use = 0;
	acm->transmitting--;
	usb_autopm_put_interface_async(acm->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int acm_start_wb(struct acm *acm, struct acm_wb *wb)
{
	int rc;

	acm->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = acm->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&acm->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		acm_write_done(acm, wb);
	}
	return rc;
}

/*
 * attributes exported through sysfs
 */
static ssize_t show_caps
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	return sprintf(buf, "%d", acm->ctrl_caps);
}
static DEVICE_ATTR(bmCapabilities, S_IRUGO, show_caps, NULL);

static ssize_t show_country_codes
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	memcpy(buf, acm->country_codes, acm->country_code_size);
	return acm->country_code_size;
}

static DEVICE_ATTR(wCountryCodes, S_IRUGO, show_country_codes, NULL);

static ssize_t show_country_rel_date
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	return sprintf(buf, "%d", acm->country_rel_date);
}

static DEVICE_ATTR(iCountryCodeRelDate, S_IRUGO, show_country_rel_date, NULL);
/*
 * Interrupt handlers for various ACM device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void acm_ctrl_irq(struct urb *urb)
{
	struct acm *acm = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	unsigned char *data;
	int newctrl;
	int difference;
	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&acm->control->dev,
				"%s - urb shutting down with status: %d\n",
				__func__, status);
		return;
	default:
		dev_dbg(&acm->control->dev,
				"%s - nonzero urb status received: %d\n",
				__func__, status);
		goto exit;
	}

	usb_mark_last_busy(acm->dev);

	data = (unsigned char *)(dr + 1);
	switch (dr->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		dev_dbg(&acm->control->dev, "%s - network connection: %d\n",
							__func__, dr->wValue);
		break;

	case USB_CDC_NOTIFY_SERIAL_STATE:
		if (le16_to_cpu(dr->wLength) != 2) {
			dev_dbg(&acm->control->dev,
				"%s - malformed serial state\n", __func__);
			break;
		}

		newctrl = get_unaligned_le16(data);

		if (!acm->clocal && (acm->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
			dev_dbg(&acm->control->dev, "%s - calling hangup\n",
					__func__);
			tty_port_tty_hangup(&acm->port, false);
		}

		difference = acm->ctrlin ^ newctrl;
		spin_lock(&acm->read_lock);
		acm->ctrlin = newctrl;
		acm->oldcount = acm->iocount;

		if (difference & ACM_CTRL_DSR)
			acm->iocount.dsr++;
		if (difference & ACM_CTRL_DCD)
			acm->iocount.dcd++;
		if (newctrl & ACM_CTRL_BRK) {
			acm->iocount.brk++;
			tty_insert_flip_char(&acm->port, 0, TTY_BREAK);
		}
		if (newctrl & ACM_CTRL_RI)
			acm->iocount.rng++;
		if (newctrl & ACM_CTRL_FRAMING)
			acm->iocount.frame++;
		if (newctrl & ACM_CTRL_PARITY)
			acm->iocount.parity++;
		if (newctrl & ACM_CTRL_OVERRUN)
			acm->iocount.overrun++;
		spin_unlock(&acm->read_lock);

		if (newctrl & ACM_CTRL_BRK)
			tty_flip_buffer_push(&acm->port);

		if (difference)
			wake_up_all(&acm->wioctl);

		break;

	default:
		dev_dbg(&acm->control->dev,
			"%s - unknown notification %d received: index %d len %d\n",
			__func__,
			dr->bNotificationType, dr->wIndex, dr->wLength);

		break;
	}
exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM)
		dev_err(&acm->control->dev, "%s - usb_submit_urb failed: %d\n",
							__func__, retval);
}

static int acm_submit_read_urb(struct acm *acm, int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &acm->read_urbs_free))
		return 0;

	dev_vdbg(&acm->data->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(acm->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM && res != -ENODEV) {
			dev_err(&acm->data->dev,
					"%s - usb_submit_urb failed: %d\n",
					__func__, res);
		}
		set_bit(index, &acm->read_urbs_free);
		return res;
	}

	return 0;
}

static int acm_submit_read_urbs(struct acm *acm, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < acm->rx_buflimit; ++i) {
		res = acm_submit_read_urb(acm, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void acm_process_read_urb(struct acm *acm, struct urb *urb)
{
	unsigned long flags;

	if (!urb->actual_length)
		return;

	spin_lock_irqsave(&acm->read_lock, flags);
	tty_insert_flip_string(&acm->port, urb->transfer_buffer,
			urb->actual_length);
	spin_unlock_irqrestore(&acm->read_lock, flags);

	tty_flip_buffer_push(&acm->port);
}

#ifdef MY_DEF_HERE
static int parsing_key(char *key)
{
	if (NULL == key) {
		return EUNIT_STATUS_UNKNOWN;
	}

	if (0 == strcmp(DT_EUNIT_STATUS_EXPCTRL, key)) {
		return EUNIT_STATUS_EXPCTRL;
	} else if (0 == strcmp(DT_EUNIT_STATUS_FANPWM, key)) {
		return EUNIT_STATUS_FANPWM;
	} else if (0 == strcmp(DT_EUNIT_STATUS_FANSPEED, key)) {
		return EUNIT_STATUS_FANSPEED;
	} else if (0 == strcmp(DT_EUNIT_STATUS_HDDCTRL, key)) {
		return EUNIT_STATUS_HDDCTRL;
	} else if (0 == strcmp(DT_EUNIT_STATUS_DISKLED, key)) {
		return EUNIT_STATUS_DISKLED;
	} else if (0 == strcmp(DT_EUNIT_STATUS_7SEGLED, key)) {
		return EUNIT_STATUS_7SEGLED;
	} else if (0 == strcmp(DT_EUNIT_STATUS_EXPIDSET, key)) {
		return EUNIT_STATUS_EXPIDSET;
	} else if (0 == strcmp(DT_EUNIT_STATUS_EXPSNSET, key)) {
		return EUNIT_STATUS_EXPSNSET;
	} else if (0 == strcmp(DT_EUNIT_STATUS_UPVERSION, key)) {
		return EUNIT_STATUS_UPVERSION;
	} else if (0 == strcmp(DT_EUNIT_STATUS_HDDENABLE, key)) {
		return EUNIT_STATUS_HDDENABLE;
	} else if (0 == strcmp(DT_EUNIT_STATUS_HDDPRESENT, key)) {
		return EUNIT_STATUS_HDDPRESENT;
	} else if (0 == strcmp(DT_EUNIT_STATUS_MONTHERMAL, key)) {
		return EUNIT_STATUS_MONTHERMAL;
	} else if (0 == strcmp(DT_EUNIT_STATUS_MONCURRENT, key)) {
		return EUNIT_STATUS_MONCURRENT;
	} else if (0 == strcmp(DT_EUNIT_STATUS_MONVOLTAGE, key)) {
		return EUNIT_STATUS_MONVOLTAGE;
	}
	return EUNIT_STATUS_UNKNOWN;
}

static void parse_single_status(char *input, struct acm *acm)
{
	char *sepptr = input;

	if (NULL == input || NULL == acm) {
		return;
	}

	sepptr = strchr(input, ':');
	if (!sepptr) {
		return;
	}
	*sepptr = 0;
	sepptr++;
	snprintf(acm->cached_expstatus[parsing_key(input)], SYNO_DTS_PROPERTY_CONTENT_LENGTH, "%s", sepptr);
	return;
}

// must get acm->status_lock
static void parsing_input(char *input, struct acm *acm)
{
	char *status = input;
	char *commaptr = NULL;
	int input_len = 0;

	if (NULL == input || NULL == acm) {
		return;
	}
	input_len = strlen(input);

	while(commaptr <= input + input_len) {
		commaptr = strchr(status, SYNO_EUNIT_STATUS_REPORT_DELIM);
		if (commaptr) {
			*commaptr = 0;
		}
		parse_single_status(status, acm);
		if (!commaptr) {
			break;
		}
		status = commaptr + 1;
	}

	return;
}

static void syno_expstatus_parsing(struct acm *acm)
{
	char acm_buffer[SYNO_EUNIT_STATUS_BUFFER_SIZE] = {0};

	if (!acm || acm->disconnected) {
		return;
	}

	if (strchr(acm->acm_buffer, SYNO_EUNIT_STATUS_REPORT_DELIM)) {
		snprintf(acm_buffer, SYNO_EUNIT_STATUS_BUFFER_SIZE, "%s", acm->acm_buffer);
		memset(acm->acm_buffer, 0, SYNO_EUNIT_STATUS_BUFFER_SIZE);
		snprintf(acm->acm_buffer, SYNO_EUNIT_STATUS_BUFFER_SIZE, "%s", strrchr(acm_buffer, SYNO_EUNIT_STATUS_REPORT_DELIM)+1);
		*strrchr(acm_buffer, SYNO_EUNIT_STATUS_REPORT_DELIM) = '\0';
		parsing_input(acm_buffer, acm);
	}

	return;
}
#endif /* MY_DEF_HERE */

static void acm_read_bulk_callback(struct urb *urb)
{
	struct acm_rb *rb = urb->context;
	struct acm *acm = rb->instance;
	unsigned long flags;
	int status = urb->status;

	dev_vdbg(&acm->data->dev, "%s - urb %d, len %d\n", __func__,
					rb->index, urb->actual_length);

	if (!acm->dev) {
		set_bit(rb->index, &acm->read_urbs_free);
		dev_dbg(&acm->data->dev, "%s - disconnected\n", __func__);
		return;
	}

#ifdef MY_DEF_HERE
	if (acm->acm_buffer && urb->actual_length) {
		write_lock_irqsave(&acm->status_lock, flags);
		strncat(acm->acm_buffer, urb->transfer_buffer,
				(SYNO_EUNIT_STATUS_BUFFER_SIZE > strlen(acm->acm_buffer) + urb->actual_length)?
				urb->actual_length : SYNO_EUNIT_STATUS_BUFFER_SIZE - strlen(acm->acm_buffer) - 1);
		syno_expstatus_parsing(acm);
		write_unlock_irqrestore(&acm->status_lock, flags);
	}
#endif /* MY_DEF_HERE */

	if (status) {
		set_bit(rb->index, &acm->read_urbs_free);
		dev_dbg(&acm->data->dev, "%s - non-zero urb status: %d\n",
							__func__, status);
		if ((status != -ENOENT) || (urb->actual_length == 0))
			return;
	}

	usb_mark_last_busy(acm->dev);

	acm_process_read_urb(acm, urb);
	/*
	 * Unthrottle may run on another CPU which needs to see events
	 * in the same order. Submission has an implict barrier
	 */
	smp_mb__before_atomic();
	set_bit(rb->index, &acm->read_urbs_free);

	/* throttle device if requested by tty */
	spin_lock_irqsave(&acm->read_lock, flags);
	acm->throttled = acm->throttle_req;
	if (!acm->throttled) {
		spin_unlock_irqrestore(&acm->read_lock, flags);
		acm_submit_read_urb(acm, rb->index, GFP_ATOMIC);
	} else {
		spin_unlock_irqrestore(&acm->read_lock, flags);
	}
}

/* data interface wrote those outgoing bytes */
static void acm_write_bulk(struct urb *urb)
{
	struct acm_wb *wb = urb->context;
	struct acm *acm = wb->instance;
	unsigned long flags;
	int status = urb->status;

	if (status || (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&acm->data->dev, "%s - len %d/%d, status %d\n",
			__func__,
			urb->actual_length,
			urb->transfer_buffer_length,
			status);

	spin_lock_irqsave(&acm->write_lock, flags);
	acm_write_done(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	schedule_work(&acm->work);
}

static void acm_softint(struct work_struct *work)
{
	struct acm *acm = container_of(work, struct acm, work);

	dev_vdbg(&acm->data->dev, "%s\n", __func__);

	tty_port_tty_wakeup(&acm->port);
}

/*
 * TTY handlers
 */

static int acm_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct acm *acm;
	int retval;

	dev_dbg(tty->dev, "%s\n", __func__);

	acm = acm_get_by_minor(tty->index);
	if (!acm)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	/*
	 * Suppress initial echoing for some devices which might send data
	 * immediately after acm driver has been installed.
	 */
	if (acm->quirks & DISABLE_ECHO)
		tty->termios.c_lflag &= ~ECHO;

	tty->driver_data = acm;

	return 0;

error_init_termios:
	tty_port_put(&acm->port);
	return retval;
}

static int acm_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct acm *acm = tty->driver_data;

	dev_dbg(tty->dev, "%s\n", __func__);

	return tty_port_open(&acm->port, tty, filp);
}

static void acm_port_dtr_rts(struct tty_port *port, int raise)
{
	struct acm *acm = container_of(port, struct acm, port);
	int val;
	int res;

	if (raise)
		val = ACM_CTRL_DTR | ACM_CTRL_RTS;
	else
		val = 0;

	/* FIXME: add missing ctrlout locking throughout driver */
	acm->ctrlout = val;

	res = acm_set_control(acm, val);
	if (res && (acm->ctrl_caps & USB_CDC_CAP_LINE))
		/* This is broken in too many devices to spam the logs */
		dev_dbg(&acm->control->dev, "failed to set dtr/rts\n");
}

static int acm_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct acm *acm = container_of(port, struct acm, port);
	int retval = -ENODEV;
	int i;

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	mutex_lock(&acm->mutex);
	if (acm->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	acm->control->needs_remote_wakeup = 1;

	acm->ctrlurb->dev = acm->dev;
	retval = usb_submit_urb(acm->ctrlurb, GFP_KERNEL);
	if (retval) {
		dev_err(&acm->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	acm_tty_set_termios(tty, NULL);

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	spin_lock_irq(&acm->read_lock);
	acm->throttled = 0;
	acm->throttle_req = 0;
	spin_unlock_irq(&acm->read_lock);

	retval = acm_submit_read_urbs(acm, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;

	usb_autopm_put_interface(acm->control);

	mutex_unlock(&acm->mutex);

	return 0;

error_submit_read_urbs:
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(acm->control);
error_get_interface:
disconnected:
	mutex_unlock(&acm->mutex);

	return usb_translate_errors(retval);
}

static void acm_port_destruct(struct tty_port *port)
{
	struct acm *acm = container_of(port, struct acm, port);

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	acm_release_minor(acm);
	usb_put_intf(acm->control);
	kfree(acm->country_codes);
	kfree(acm);
}

static void acm_port_shutdown(struct tty_port *port)
{
	struct acm *acm = container_of(port, struct acm, port);
	struct urb *urb;
	struct acm_wb *wb;
	int i;

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	/*
	 * Need to grab write_lock to prevent race with resume, but no need to
	 * hold it due to the tty-port initialised flag.
	 */
	spin_lock_irq(&acm->write_lock);
	spin_unlock_irq(&acm->write_lock);

	usb_autopm_get_interface_no_resume(acm->control);
	acm->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(acm->control);

	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(acm->control);
	}

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
}

static void acm_tty_cleanup(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	dev_dbg(&acm->control->dev, "%s\n", __func__);
	tty_port_put(&acm->port);
}

static void acm_tty_hangup(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	dev_dbg(&acm->control->dev, "%s\n", __func__);
	tty_port_hangup(&acm->port);
}

static void acm_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct acm *acm = tty->driver_data;
	dev_dbg(&acm->control->dev, "%s\n", __func__);
	tty_port_close(&acm->port, tty, filp);
}

static int acm_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct acm *acm = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;
#ifdef MY_DEF_HERE
	int iWaitTime = 0;
#endif /* MY_DEF_HERE */

	if (!count)
		return 0;

	dev_vdbg(&acm->data->dev, "%s - count %d\n", __func__, count);

#ifdef MY_DEF_HERE
	if (syno_is_synology_acm(acm)) {
		for (iWaitTime = 0; iWaitTime < SYNO_EUNIT_READY_RETRY; iWaitTime++) {
			if (0 == acm->transmitting) {
				break;
			}
			msleep(SYNO_EUNIT_ACM_WAITING_READY);
		}
	}
#endif /* MY_DEF_HERE */

	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = acm_wb_alloc(acm);
	if (wbn < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;
	}
	wb = &acm->wb[wbn];

	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -ENODEV;
	}

	count = (count > acm->writesize) ? acm->writesize : count;
	dev_vdbg(&acm->data->dev, "%s - write %d\n", __func__, count);
	memcpy(wb->buf, buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(acm->control);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return stat;
	}

	if (acm->susp_count) {
		usb_anchor_urb(wb->urb, &acm->delayed);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return count;
	}

	stat = acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);

	if (stat < 0)
		return stat;
	return count;
}

static int acm_tty_write_room(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return acm_wb_is_avail(acm) ? acm->writesize : 0;
}

static int acm_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	/*
	 * if the device was unplugged then any remaining characters fell out
	 * of the connector ;)
	 */
	if (acm->disconnected)
		return 0;
	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	return (ACM_NW - acm_wb_is_avail(acm)) * acm->writesize;
}

static void acm_tty_throttle(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;

	spin_lock_irq(&acm->read_lock);
	acm->throttle_req = 1;
	spin_unlock_irq(&acm->read_lock);
}

static void acm_tty_unthrottle(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	unsigned int was_throttled;

	spin_lock_irq(&acm->read_lock);
	was_throttled = acm->throttled;
	acm->throttled = 0;
	acm->throttle_req = 0;
	spin_unlock_irq(&acm->read_lock);

	if (was_throttled)
		acm_submit_read_urbs(acm, GFP_KERNEL);
}

static int acm_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct acm *acm = tty->driver_data;
	int retval;

	retval = acm_send_break(acm, state ? 0xffff : 0);
	if (retval < 0)
		dev_dbg(&acm->control->dev, "%s - send break failed\n",
								__func__);
	return retval;
}

static int acm_tty_tiocmget(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;

	return (acm->ctrlout & ACM_CTRL_DTR ? TIOCM_DTR : 0) |
	       (acm->ctrlout & ACM_CTRL_RTS ? TIOCM_RTS : 0) |
	       (acm->ctrlin  & ACM_CTRL_DSR ? TIOCM_DSR : 0) |
	       (acm->ctrlin  & ACM_CTRL_RI  ? TIOCM_RI  : 0) |
	       (acm->ctrlin  & ACM_CTRL_DCD ? TIOCM_CD  : 0) |
	       TIOCM_CTS;
}

static int acm_tty_tiocmset(struct tty_struct *tty,
			    unsigned int set, unsigned int clear)
{
	struct acm *acm = tty->driver_data;
	unsigned int newctrl;

	newctrl = acm->ctrlout;
	set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
					(set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
	clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
					(clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

	newctrl = (newctrl & ~clear) | set;

	if (acm->ctrlout == newctrl)
		return 0;
	return acm_set_control(acm, acm->ctrlout = newctrl);
}

static int get_serial_info(struct acm *acm, struct serial_struct __user *info)
{
	struct serial_struct tmp;

	if (!info)
		return -EINVAL;

	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = ASYNC_LOW_LATENCY;
	tmp.xmit_fifo_size = acm->writesize;
	tmp.baud_base = le32_to_cpu(acm->line.dwDTERate);
	tmp.close_delay	= jiffies_to_msecs(acm->port.close_delay) / 10;
	tmp.closing_wait = acm->port.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
				ASYNC_CLOSING_WAIT_NONE :
				jiffies_to_msecs(acm->port.closing_wait) / 10;

	if (copy_to_user(info, &tmp, sizeof(tmp)))
		return -EFAULT;
	else
		return 0;
}

static int set_serial_info(struct acm *acm,
				struct serial_struct __user *newinfo)
{
	struct serial_struct new_serial;
	unsigned int closing_wait, close_delay;
	unsigned int old_closing_wait, old_close_delay;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	close_delay = msecs_to_jiffies(new_serial.close_delay * 10);
	closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
			ASYNC_CLOSING_WAIT_NONE :
			msecs_to_jiffies(new_serial.closing_wait * 10);

	/* we must redo the rounding here, so that the values match */
	old_close_delay	= jiffies_to_msecs(acm->port.close_delay) / 10;
	old_closing_wait = acm->port.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
				ASYNC_CLOSING_WAIT_NONE :
				jiffies_to_msecs(acm->port.closing_wait) / 10;

	mutex_lock(&acm->port.mutex);

	if (!capable(CAP_SYS_ADMIN)) {
		if ((new_serial.close_delay != old_close_delay) ||
	            (new_serial.closing_wait != old_closing_wait))
			retval = -EPERM;
	} else {
		acm->port.close_delay  = close_delay;
		acm->port.closing_wait = closing_wait;
	}

	mutex_unlock(&acm->port.mutex);
	return retval;
}

static int wait_serial_change(struct acm *acm, unsigned long arg)
{
	int rv = 0;
	DECLARE_WAITQUEUE(wait, current);
	struct async_icount old, new;

	do {
		spin_lock_irq(&acm->read_lock);
		old = acm->oldcount;
		new = acm->iocount;
		acm->oldcount = new;
		spin_unlock_irq(&acm->read_lock);

		if ((arg & TIOCM_DSR) &&
			old.dsr != new.dsr)
			break;
		if ((arg & TIOCM_CD)  &&
			old.dcd != new.dcd)
			break;
		if ((arg & TIOCM_RI) &&
			old.rng != new.rng)
			break;

		add_wait_queue(&acm->wioctl, &wait);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		remove_wait_queue(&acm->wioctl, &wait);
		if (acm->disconnected) {
			if (arg & TIOCM_CD)
				break;
			else
				rv = -ENODEV;
		} else {
			if (signal_pending(current))
				rv = -ERESTARTSYS;
		}
	} while (!rv);

	

	return rv;
}

static int get_serial_usage(struct acm *acm,
			    struct serial_icounter_struct __user *count)
{
	struct serial_icounter_struct icount;
	int rv = 0;

	memset(&icount, 0, sizeof(icount));
	icount.dsr = acm->iocount.dsr;
	icount.rng = acm->iocount.rng;
	icount.dcd = acm->iocount.dcd;
	icount.frame = acm->iocount.frame;
	icount.overrun = acm->iocount.overrun;
	icount.parity = acm->iocount.parity;
	icount.brk = acm->iocount.brk;

	if (copy_to_user(count, &icount, sizeof(icount)) > 0)
		rv = -EFAULT;

	return rv;
}

static int acm_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct acm *acm = tty->driver_data;
	int rv = -ENOIOCTLCMD;

	switch (cmd) {
	case TIOCGSERIAL: /* gets serial port data */
		rv = get_serial_info(acm, (struct serial_struct __user *) arg);
		break;
	case TIOCSSERIAL:
		rv = set_serial_info(acm, (struct serial_struct __user *) arg);
		break;
	case TIOCMIWAIT:
		rv = usb_autopm_get_interface(acm->control);
		if (rv < 0) {
			rv = -EIO;
			break;
		}
		rv = wait_serial_change(acm, arg);
		usb_autopm_put_interface(acm->control);
		break;
	case TIOCGICOUNT:
		rv = get_serial_usage(acm, (struct serial_icounter_struct __user *) arg);
		break;
	}

	return rv;
}

static void acm_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	struct acm *acm = tty->driver_data;
	struct ktermios *termios = &tty->termios;
	struct usb_cdc_line_coding newline;
	int newctrl = acm->ctrlout;

	newline.dwDTERate = cpu_to_le32(tty_get_baud_rate(tty));
	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 0;
	newline.bParityType = termios->c_cflag & PARENB ?
				(termios->c_cflag & PARODD ? 1 : 2) +
				(termios->c_cflag & CMSPAR ? 2 : 0) : 0;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		newline.bDataBits = 5;
		break;
	case CS6:
		newline.bDataBits = 6;
		break;
	case CS7:
		newline.bDataBits = 7;
		break;
	case CS8:
	default:
		newline.bDataBits = 8;
		break;
	}
	/* FIXME: Needs to clear unsupported bits in the termios */
	acm->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (C_BAUD(tty) == B0) {
		newline.dwDTERate = acm->line.dwDTERate;
		newctrl &= ~ACM_CTRL_DTR;
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		newctrl |=  ACM_CTRL_DTR;
	}

	if (newctrl != acm->ctrlout)
		acm_set_control(acm, acm->ctrlout = newctrl);

	if (memcmp(&acm->line, &newline, sizeof newline)) {
		memcpy(&acm->line, &newline, sizeof newline);
		dev_dbg(&acm->control->dev, "%s - set line: %d %d %d %d\n",
			__func__,
			le32_to_cpu(newline.dwDTERate),
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
		acm_set_line(acm, &acm->line);
	}
}

static const struct tty_port_operations acm_port_ops = {
	.dtr_rts = acm_port_dtr_rts,
	.shutdown = acm_port_shutdown,
	.activate = acm_port_activate,
	.destruct = acm_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void acm_write_buffers_free(struct acm *acm)
{
	int i;
	struct acm_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++)
		usb_free_coherent(usb_dev, acm->writesize, wb->buf, wb->dmah);
}

static void acm_read_buffers_free(struct acm *acm)
{
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);
	int i;

	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_coherent(usb_dev, acm->readsize,
			  acm->read_buffers[i].base, acm->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int acm_write_buffers_alloc(struct acm *acm)
{
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(acm->dev, acm->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(acm->dev, acm->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

#ifdef MY_DEF_HERE

int syno_acm_slotindex_get(int *slot_index, struct acm *acm) {
	int ret = -1;
	const char *control_string = NULL, *usb_port_string = NULL;
	struct device_node *device_node = NULL, *control_method = NULL, *usb_port = NULL;
	int eunit_index = 0;

	if (NULL == acm || NULL == slot_index) {
		goto END;
	}
	for_each_child_of_node(of_root, device_node) {
		if (!device_node->full_name) {
			continue;
		}

		if (strstr(device_node->full_name, DT_ESATA_SLOT)) {
			sscanf(device_node->full_name, "/"DT_ESATA_SLOT"@%d", &eunit_index);
		} else if (strstr(device_node->full_name, DT_CX4_SLOT)) {
			sscanf(device_node->full_name, "/"DT_CX4_SLOT"@%d", &eunit_index);
		} else {
			continue;
		}

		for_each_child_of_node(device_node, control_method) {
			if (!control_method->name || strcmp(DT_EUNIT_CONTROL_METHOD, control_method->name)) {
				continue;
			}
			if (0 > of_property_read_string(control_method, DT_EUNIT_CONTROL_TYPE, &control_string)) {
				continue;
			}
			if (0 != strcmp(DT_USB_TO_TTY, control_string)) {
				continue;
			}
			for_each_child_of_node(control_method, usb_port) {
				if (usb_port->name && 0 == strcmp(usb_port->name, DT_USB2)) {
					if (0 > of_property_read_string(usb_port, DT_USB_PORT, &usb_port_string)) {
						continue;
					}
					if (0 == strcmp(dev_name(&acm->dev->dev), usb_port_string)) {
						*slot_index = eunit_index;
						ret = 0;
					}
					//TODO: do early break
				}
			}
		}
	}

END:
	return ret;
}

int syno_usb_acm_container_index_get_by_diskname(int *slot_index, const char *disk_name)
{
	struct acm *acm;
	int ret = -1;
	struct syno_device_list *sdl = NULL, *tmp = NULL;
	struct syno_acm_list *sal = NULL, *sal_tmp = NULL;
	unsigned long flags = 0;

	spin_lock_irqsave(&acm_list_lock, flags);
	list_for_each_entry_safe(sal, sal_tmp, &syno_acm_list_head, device_list) {
		acm = sal->acm;
		list_for_each_entry_safe(sdl, tmp, &acm->syno_device_list, device_list) {
			if (0 == strcmp(sdl->disk_name, disk_name)) {
				if (0 <= syno_acm_slotindex_get(slot_index, acm)){
					ret = 0;
				}
			}
		}
	}
	spin_unlock_irqrestore(&acm_list_lock, flags);
	return ret;
}

static int syno_samd_tty_write(struct acm *acm, const char *command)
{
	char buffer[SYNO_EUNIT_STATUS_BUFFER_SIZE] = {0};
	//TODO: refine macro end char
	char end[2] = {0xd, 0x0};
	struct tty_struct tty;

	if (!acm || !command) {
		return -EINVAL;
	}
	memset(&tty, 0, sizeof(tty));
	tty.driver_data = acm;
	snprintf(buffer, SYNO_EUNIT_STATUS_BUFFER_SIZE, "%s%s", command, end);
	return acm_tty_write(&tty, buffer, strlen(buffer));
}

static ssize_t syno_eunit_info_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct syno_device_list *sdl = NULL, *tmp = NULL;
	char szTmp[SYNO_EUNIT_STATUS_BUFFER_SIZE] = {0};
	char *szTmp1 = NULL;
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);
	unsigned long flags = 0;

	if (acm->disconnected) {
		return 0;
	}

	szTmp1 = (char*) kzalloc(PAGE_SIZE, GFP_KERNEL);

	if (NULL == szTmp1) {
		printk(KERN_WARNING "%s kzalloc failed\n", __FUNCTION__);
		return 0;
	}

	spin_lock_irqsave(&acm_list_lock, flags);
	list_for_each_entry_safe(sdl, tmp, &acm->syno_device_list, device_list) {
		snprintf(szTmp1, BDEVNAME_SIZE, "/dev/%s,", sdl->disk_name);
		strncat(szTmp, szTmp1, BDEVNAME_SIZE);
	}
	spin_unlock_irqrestore(&acm_list_lock, flags);
	if (strlen(szTmp)) {
		szTmp[strlen(szTmp)-1] = '\0';
	}

	snprintf(szTmp1, PAGE_SIZE, "%s%s%s%s", EBOX_INFO_DEV_LIST_KEY, "=\"", szTmp, "\"\n");

	/* vendor id and device id */
	snprintf(szTmp,
			BDEVNAME_SIZE,
			"%s=%s0x%x%s", EBOX_INFO_VENDOR_KEY, "\"",
			usb_dev->descriptor.idVendor,
			"\"\n");
	strncat(szTmp1, szTmp, BDEVNAME_SIZE);
	snprintf(szTmp,
			BDEVNAME_SIZE,
			"%s=%s0x%x%s", EBOX_INFO_DEVICE_KEY, "\"",
			usb_dev->descriptor.idProduct,
			"\"\n");
	strncat(szTmp1, szTmp, BDEVNAME_SIZE);
	snprintf(szTmp, BDEVNAME_SIZE,
			EBOX_INFO_USB_PATH"=\"%s\"\n",
			dev_name(&acm->dev->dev));
	strncat(szTmp1, szTmp, BDEVNAME_SIZE);


	/* deepsleep support */
	snprintf(szTmp,
			BDEVNAME_SIZE,
			"%s=\"%s\"\n", EBOX_INFO_DEEP_SLEEP, "yes");
	strncat(szTmp1, szTmp, BDEVNAME_SIZE);

	if (NULL != acm->cached_expstatus[EUNIT_STATUS_EXPIDSET] && 0 < strlen(acm->cached_expstatus[EUNIT_STATUS_EXPIDSET])) {
		snprintf(szTmp,
			BDEVNAME_SIZE,
			"%s=\"%s\"\n%s=\"%d\"\n",
			EBOX_INFO_UNIQUE_KEY,
			acm->cached_expstatus[EUNIT_STATUS_EXPIDSET],
			EBOX_INFO_EMID_KEY,
			0);
		strncat(szTmp1, szTmp, BDEVNAME_SIZE);
	} else {
		snprintf(szTmp,
			BDEVNAME_SIZE,
			"%s=\"%s\"\n%s=\"%d\"\n",
			EBOX_INFO_UNIQUE_KEY,
			"Unknwon",
			EBOX_INFO_EMID_KEY,
			0);
		strncat(szTmp1, szTmp, BDEVNAME_SIZE);
	}

	/* put it together */
	snprintf(buf, PAGE_SIZE, "%s", szTmp1);
	kfree(szTmp1);

	return strlen(buf);
}

//TODO:refine to macro format, reference usbcore.c
static ssize_t syno_eunit_upversion_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_UPVERSION]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}

static ssize_t syno_eunit_hddenable_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_HDDENABLE]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}

static ssize_t syno_eunit_hddpresent_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}

static ssize_t syno_eunit_monthermal_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_MONTHERMAL]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}

static ssize_t syno_eunit_moncurrent_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_MONCURRENT]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}
static ssize_t syno_eunit_monvoltage_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_MONVOLTAGE]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}

static ssize_t syno_eunit_expctrl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_EXPCTRL]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}
static ssize_t syno_eunit_fanpwm_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_FANPWM]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}
static ssize_t syno_eunit_fanspeed_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_FANSPEED]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}
static ssize_t syno_eunit_hddctrl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_HDDCTRL]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}
static ssize_t syno_eunit_diskled_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_DISKLED]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}
static ssize_t syno_eunit_7segled_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_7SEGLED]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}


static ssize_t syno_eunit_expidset_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}


static ssize_t syno_eunit_expsnset_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	unsigned long flags = 0;
	int len = 0;
	read_lock_irqsave(&acm->status_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%s", acm->cached_expstatus[EUNIT_STATUS_EXPSNSET]);
	read_unlock_irqrestore(&acm->status_lock, flags);
	return len;
}

static ssize_t syno_eunit_write_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	struct acm *acm = usb_get_intfdata(to_usb_interface(dev));
	if (len > syno_samd_tty_write(acm, buf)) {
		return -1;
	}
	return len;
}

static DEVICE_ATTR(syno_eunit_info, S_IRUGO, syno_eunit_info_show, NULL);
static DEVICE_ATTR(syno_eunit_expctrl, S_IRUGO, syno_eunit_expctrl_show, NULL);
static DEVICE_ATTR(syno_eunit_fanpwm, S_IRUGO, syno_eunit_fanpwm_show, NULL);
static DEVICE_ATTR(syno_eunit_fanspeed, S_IRUGO, syno_eunit_fanspeed_show, NULL);
static DEVICE_ATTR(syno_eunit_hddctrl, S_IRUGO, syno_eunit_hddctrl_show, NULL);
static DEVICE_ATTR(syno_eunit_diskled, S_IRUGO, syno_eunit_diskled_show, NULL);
static DEVICE_ATTR(syno_eunit_7segled, S_IRUGO, syno_eunit_7segled_show, NULL);
static DEVICE_ATTR(syno_eunit_expidset, S_IRUGO, syno_eunit_expidset_show, NULL);
static DEVICE_ATTR(syno_eunit_expsnset, S_IRUGO, syno_eunit_expsnset_show, NULL);
static DEVICE_ATTR(syno_eunit_upversion, S_IRUGO, syno_eunit_upversion_show, NULL);
static DEVICE_ATTR(syno_eunit_hddenable, S_IRUGO, syno_eunit_hddenable_show, NULL);
static DEVICE_ATTR(syno_eunit_hddpresent, S_IRUGO, syno_eunit_hddpresent_show, NULL);
static DEVICE_ATTR(syno_eunit_monthermal, S_IRUGO, syno_eunit_monthermal_show, NULL);
static DEVICE_ATTR(syno_eunit_moncurrent, S_IRUGO, syno_eunit_moncurrent_show, NULL);
static DEVICE_ATTR(syno_eunit_monvoltage, S_IRUGO, syno_eunit_monvoltage_show, NULL);
static DEVICE_ATTR(syno_eunit_write, S_IWUSR, NULL, syno_eunit_write_store);

static void syno_remove_eunit_files(struct device *dev)
{
	device_remove_file(dev, &dev_attr_syno_eunit_expctrl);
	device_remove_file(dev, &dev_attr_syno_eunit_fanpwm);
	device_remove_file(dev, &dev_attr_syno_eunit_fanspeed);
	device_remove_file(dev, &dev_attr_syno_eunit_hddctrl);
	device_remove_file(dev, &dev_attr_syno_eunit_diskled);
	device_remove_file(dev, &dev_attr_syno_eunit_7segled);
	device_remove_file(dev, &dev_attr_syno_eunit_expidset);
	device_remove_file(dev, &dev_attr_syno_eunit_expsnset);
	device_remove_file(dev, &dev_attr_syno_eunit_info);
	device_remove_file(dev, &dev_attr_syno_eunit_upversion);
	device_remove_file(dev, &dev_attr_syno_eunit_hddenable);
	device_remove_file(dev, &dev_attr_syno_eunit_hddpresent);
	device_remove_file(dev, &dev_attr_syno_eunit_monthermal);
	device_remove_file(dev, &dev_attr_syno_eunit_moncurrent);
	device_remove_file(dev, &dev_attr_syno_eunit_monvoltage);
	device_remove_file(dev, &dev_attr_syno_eunit_write);
}

int syno_acm_get_usb_port(const char **usb_port_string, int eunit_slot) {
	int ret = -1;
	const char *control_string = NULL;
	struct device_node *device_node = NULL, *control_method = NULL, *usb_port = NULL;
	int eunit_index = 0;

	if (NULL == usb_port_string || 0 >= eunit_slot) {
		goto END;
	}
	for_each_child_of_node(of_root, device_node) {
		if (!device_node->full_name) {
			continue;
		}

		if (strstr(device_node->full_name, DT_ESATA_SLOT)) {
			sscanf(device_node->full_name, "/"DT_ESATA_SLOT"@%d", &eunit_index);
		} else if (strstr(device_node->full_name, DT_CX4_SLOT)) {
			sscanf(device_node->full_name, "/"DT_CX4_SLOT"@%d", &eunit_index);
		} else {
			continue;
		}

		if (eunit_index != eunit_slot) {
			continue;
		}

		for_each_child_of_node(device_node, control_method) {
			if (!control_method->name || strcmp(DT_EUNIT_CONTROL_METHOD, control_method->name)) {
				continue;
			}
			if (0 > of_property_read_string(control_method, DT_EUNIT_CONTROL_TYPE, &control_string)) {
				continue;
			}
			if (0 != strcmp(DT_USB_TO_TTY, control_string)) {
				continue;
			}
			for_each_child_of_node(control_method, usb_port) {
				if (usb_port->name && 0 == strcmp(usb_port->name, DT_USB2)) {
					if (0 > of_property_read_string(usb_port, DT_USB_PORT, usb_port_string)) {
						continue;
					}
					ret = 0;
					//TODO: do early break
				}
			}
		}
	}

END:
	return ret;
}
EXPORT_SYMBOL(syno_acm_get_usb_port);

static void syno_create_eunit_files(struct device *dev)
{
	int retval = 0;
	//TODO: receive return error code
	retval = device_create_file(dev, &dev_attr_syno_eunit_expctrl);
	retval = device_create_file(dev, &dev_attr_syno_eunit_fanpwm);
	retval = device_create_file(dev, &dev_attr_syno_eunit_fanspeed);
	retval = device_create_file(dev, &dev_attr_syno_eunit_hddctrl);
	retval = device_create_file(dev, &dev_attr_syno_eunit_diskled);
	retval = device_create_file(dev, &dev_attr_syno_eunit_7segled);
	retval = device_create_file(dev, &dev_attr_syno_eunit_expidset);
	retval = device_create_file(dev, &dev_attr_syno_eunit_expsnset);
	retval = device_create_file(dev, &dev_attr_syno_eunit_info);
	retval = device_create_file(dev, &dev_attr_syno_eunit_monthermal);
	retval = device_create_file(dev, &dev_attr_syno_eunit_monvoltage);
	retval = device_create_file(dev, &dev_attr_syno_eunit_moncurrent);
	retval = device_create_file(dev, &dev_attr_syno_eunit_upversion);
	retval = device_create_file(dev, &dev_attr_syno_eunit_hddenable);
	retval = device_create_file(dev, &dev_attr_syno_eunit_hddpresent);
	retval = device_create_file(dev, &dev_attr_syno_eunit_write);
}
void syno_acm_device_list_add(struct scsi_device *sdev, const char* device_name)
{
	const char *usb_port_string = NULL;
	struct acm *acm = NULL;
	struct syno_device_list *sdl = NULL;
	int slot_index = -1;
	struct ata_port *ap = NULL;
	struct acm_device_temp *adt = NULL;
	struct syno_acm_list *sal = NULL, *sal_tmp = NULL;
	unsigned long flags = 0;

	if (NULL == sdev || !device_name) {
		goto END;
	}

	ap = ata_shost_to_port(sdev->host);
	if (!ap) {
		goto END;
	}
	slot_index = syno_external_libata_index_get(ap);
	if (0 >= slot_index) {
		goto END;
	}

	if (0 > syno_acm_get_usb_port(&usb_port_string, slot_index)) {
		goto END;
	}

	spin_lock_irqsave(&acm_list_lock, flags);
	list_for_each_entry_safe(sal, sal_tmp, &syno_acm_list_head, device_list) {
		acm = sal->acm;
		if (!acm->disconnected && 0 == strcmp(usb_port_string, dev_name(&acm->dev->dev))) {
			sdl = kzalloc(sizeof(*sdl), GFP_ATOMIC);
			if (!sdl) {
				continue;
			}
			snprintf(sdl->disk_name, DISK_NAME_LEN, "%s", device_name);
			list_add(&sdl->device_list, &acm->syno_device_list);

			if (0 < acm->initial_disk_not_ready_check) {
				syno_disk_not_ready_count_decrease();
				acm->initial_disk_not_ready_check -= 1;
			}
		}
	}
	adt = kzalloc(sizeof(*adt), GFP_ATOMIC);
	if (!adt) {
		spin_unlock_irqrestore(&acm_list_lock, flags);
		goto END;
	}
	snprintf(adt->disk_name, DISK_NAME_LEN, "%s", device_name);
	snprintf(adt->usb_path, SYNO_DTS_PROPERTY_CONTENT_LENGTH, "%s", usb_port_string);
	list_add(&adt->device_list, &acm_temp_device_list);
	spin_unlock_irqrestore(&acm_list_lock, flags);

END:
	return;
}

void syno_acm_device_list_delete(const char* device_name)
{
	struct acm *acm;
	struct syno_device_list *sdl = NULL, *tmp = NULL;
	struct syno_acm_list *sal = NULL, *sal_tmp = NULL;
	unsigned long flags = 0;
	struct acm_device_temp *adt = NULL, *adt_tmp = NULL;

	spin_lock_irqsave(&acm_list_lock, flags);
	list_for_each_entry_safe(sal, sal_tmp, &syno_acm_list_head, device_list) {
		acm = sal->acm;
		list_for_each_entry_safe(sdl, tmp, &acm->syno_device_list, device_list) {
			if (0 == strcmp(sdl->disk_name, device_name)) {
				list_del(&sdl->device_list);
				kfree(sdl);
			}
		}
	}

	list_for_each_entry_safe(adt, adt_tmp, &acm_temp_device_list, device_list) {
		if (0 == strcmp(adt->disk_name, device_name)) {
			list_del(&adt->device_list);
			kfree(adt);
		}
	}

	spin_unlock_irqrestore(&acm_list_lock, flags);
	return;
}

void syno_acm_device_list_set(struct scsi_device *sdev, int add, const char* device_name)
{
	if (add)
		syno_acm_device_list_add(sdev, device_name);
	else
		syno_acm_device_list_delete(device_name);
}
EXPORT_SYMBOL(syno_acm_device_list_set);


static void syno_period_cache_update(struct work_struct *work)
{
	struct delayed_work *delayed_work = container_of(work, struct delayed_work, work);
	struct acm *acm = container_of(delayed_work, struct acm, cache_update_work);

	if (acm->disconnected)
		return;

	acm_submit_read_urbs(acm, GFP_KERNEL);
	syno_samd_tty_write(acm, DT_EUNIT_STATUS_EXPSTATUS);

	schedule_delayed_work(&acm->cache_update_work, 4000);
	return;
}
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
static struct acm *syno_acm_get_by_usbport(const char *usb_port)
{
	struct acm *acm = NULL;
	struct acm *acm_tmp = NULL;
	struct syno_acm_list *sal = NULL, *sal_tmp = NULL;
	unsigned long flags = 0;

	if (!usb_port) {
		return acm;
	}

	spin_lock_irqsave(&acm_list_lock, flags);
	list_for_each_entry_safe(sal, sal_tmp, &syno_acm_list_head, device_list) {
		acm_tmp = sal->acm;
		if (!acm_tmp->disconnected) {
			if (0 == strcmp(usb_port, dev_name(&acm_tmp->dev->dev))) {
				acm = acm_tmp;
			}
		}
		if (acm) {
			break;
		}
	}
	spin_unlock_irqrestore(&acm_list_lock, flags);

	return acm;
}

int syno_usb_acm_unique_get(const int slot_type, const int slot_index, char *unique, int unique_size)
{
	const char *usb_port = NULL;
	int ret = -1;
	struct acm *acm = NULL;

	if (EUNIT_DEVICE != slot_type || 0 >= slot_index || !unique || 0 >= unique_size) {
		goto END;
	}

	if (0 > syno_acm_get_usb_port(&usb_port, slot_index)) {
		goto END;
	}

	if (NULL == (acm = syno_acm_get_by_usbport(usb_port))) {
		goto END;
	}

	if (NULL == acm->cached_expstatus || NULL == acm->cached_expstatus[EUNIT_STATUS_EXPIDSET] ||
		strlen(acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]) >= unique_size) {
		goto END;
	}
	snprintf(unique, unique_size, "%s", acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]);

	ret = 0;

END:
	return ret;
}

static int syno_usb_get_hdd_count(const char *eunit_model_name) {
	int iCount = 0;
	struct device_node *device_node = NULL, *pmp_slot_node = NULL;

	if (!eunit_model_name) {
		return iCount;
	}

	for_each_child_of_node(of_root, device_node) {
		if (NULL == device_node->full_name || NULL == (strstr(device_node->full_name, eunit_model_name))) {
			continue;
		}
		for_each_child_of_node(device_node, pmp_slot_node) {
			if (pmp_slot_node->name && 0 == (strcmp(DT_PMP_SLOT, pmp_slot_node->name))) {
				iCount += 1;
			}
		}
	}

	return iCount;
}

int syno_usb_eunit_hdd_ctrl(const int slot_type, const int slot_index, int hdd_ctrl) {
	int ret = -1, i = 0, disk_count = 0;
	char hdd_cmd[SYNO_EUNIT_STATUS_BUFFER_SIZE] = {0};
	struct acm *acm = NULL;
	const char *usb_port = NULL;

	if (0 >= slot_type || 0 >= slot_index || (0 != hdd_ctrl && 1 != hdd_ctrl)) {
		goto END;
	}

	if (0 > syno_acm_get_usb_port(&usb_port, slot_index)) {
		goto END;
	}

	if (NULL == (acm = syno_acm_get_by_usbport(usb_port))) {
		goto END;
	}

	if (NULL == acm->cached_expstatus || NULL == acm->cached_expstatus[EUNIT_STATUS_EXPIDSET] ||
		0 >= (disk_count = syno_usb_get_hdd_count(acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]))) {
		goto END;
	}

	// set hdd to manual mode
	if (hdd_ctrl) {
		if (0 >= snprintf(hdd_cmd, SYNO_EUNIT_STATUS_BUFFER_SIZE, "%s:A/", DT_EUNIT_STATUS_HDDCTRL)) {
			goto END;
		}
	} else {
		if (0 >= snprintf(hdd_cmd, SYNO_EUNIT_STATUS_BUFFER_SIZE, "%s:M/", DT_EUNIT_STATUS_HDDCTRL)) {
			goto END;
		}
		for (i = 0; i < disk_count; i++) {
			if (0 >= snprintf(hdd_cmd + strlen(hdd_cmd) , SYNO_EUNIT_STATUS_BUFFER_SIZE - strlen(hdd_cmd), "%d/", hdd_ctrl)) {
				goto END;
			}
		}
	}
	syno_samd_tty_write(acm, hdd_cmd);
	ret = 0;
END:
	return ret;
}
EXPORT_SYMBOL(syno_usb_eunit_hdd_ctrl);

int syno_usb_eunit_deep_sleep_indicator(const int slot_type, const int slot_index, const int control) {
	int ret = -1, offset = 0;
	char indicator_cmd[SYNO_EUNIT_STATUS_BUFFER_SIZE] = {0};
	int i = 0;
	struct acm *acm = NULL;
	struct device_node *device_node = NULL, *deep_sleep_indicator = NULL;
	const char *command = NULL, *eunit_model_name = NULL, *usb_port = NULL;

	if (0 >= slot_type || 0 >= slot_index || (1 != control && 0 != control)) {
		goto END;
	}

	if (0 > syno_acm_get_usb_port(&usb_port, slot_index)) {
		goto END;
	}

	if (NULL == (acm = syno_acm_get_by_usbport(usb_port))) {
		goto END;
	}

	if (NULL == acm->cached_expstatus || NULL == acm->cached_expstatus[EUNIT_STATUS_EXPIDSET] ||
		NULL == (eunit_model_name = acm->cached_expstatus[EUNIT_STATUS_EXPIDSET])) {
		goto END;
	}

	for_each_child_of_node(of_root, device_node) {
		if (NULL == device_node->full_name || NULL == (strstr(device_node->full_name, eunit_model_name))) {
			continue;
		}
		for_each_child_of_node(device_node, deep_sleep_indicator) {
			if (!deep_sleep_indicator->name || strcmp(SZ_DTS_EBOX_I2C_DEEPSELLP_INDICATOR, deep_sleep_indicator->name)) {
				continue;
			}
			if (0 > of_property_read_string(deep_sleep_indicator, DT_EUNIT_COMMAND, &command)) {
				continue;
			}
			if (0 > of_property_read_u32(deep_sleep_indicator, SZ_DTS_EBOX_I2C_OFFSET, &offset)) {
				continue;
			}
			snprintf(indicator_cmd, SYNO_EUNIT_STATUS_BUFFER_SIZE, "%s:", command);
			for (i = 0; i < offset; i++) {
				strncat(indicator_cmd, "R/", SYNO_EUNIT_STATUS_BUFFER_SIZE - strlen(indicator_cmd) - 1);
			}
			snprintf(indicator_cmd + strlen(indicator_cmd), SYNO_EUNIT_STATUS_BUFFER_SIZE - strlen(indicator_cmd),
					"%d/", control);
		}
	}

	if (acm && strlen(indicator_cmd)) {
		syno_samd_tty_write(acm, indicator_cmd);
		ret = 0;
	}
END:
	return ret;
}
EXPORT_SYMBOL(syno_usb_eunit_deep_sleep_indicator);

static int syno_initial_not_ready_disk_count(struct acm *acm) {
	int iRet = -1;
	int disk_count = 0;
	int i = 0;

	if (NULL == acm || NULL == acm->cached_expstatus ||
		NULL == acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT] ||
		NULL == acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]) {
		goto END;
	}

	if (0 >= (disk_count = syno_usb_get_hdd_count(acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]))) {
		goto END;
	}
	/* hddpresent string format ex: 0/1/1/1/1/ */
	if ((2 * disk_count) != strlen(acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT])) {
		goto END;
	}

	for (i = 0; i < strlen(acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT]); i++) {
		if ('1' == acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT][i]) {
			syno_disk_not_ready_count_increase();
			acm->initial_disk_not_ready_check += 1;
		}
	}

	iRet = 0;
END:
	return iRet;
}

int syno_usb_eunit_disk_delay_waiting(const int slot_type, const int slot_index, const int disk_id, int spinup)
{
	struct device_node *of_eunit = NULL;
	int spinup_group_delay = 0, spinup_group = 0, group_num = 0, spinup_group_size = 0;
	int accum_waking_disks_available = 0, waken_disk_count = 0;
	const char *usb_port = NULL;
	struct acm *acm = NULL;
	int need_waiting = 0;
	unsigned long flags = 0;

	if (0 > slot_type || 0 >= slot_index || 0 > disk_id || 0 > spinup) {
		return -EINVAL;
	}

	if (0 > syno_acm_get_usb_port(&usb_port, slot_index)) {
		return -ENOENT;
	}

	if (NULL == (acm = syno_acm_get_by_usbport(usb_port))) {
		return -ENOENT;
	}

	if (NULL == (of_eunit = of_get_child_by_name(of_root, acm->cached_expstatus[EUNIT_STATUS_EXPIDSET]))) {
		return -ENODEV;
	}

	if (of_property_read_u32_index(of_eunit, DT_SYNO_SPINUP_GROUP_DELAY, 0, &spinup_group_delay)) {
		// do not support spinup_group_delay, no waiting need
		return need_waiting;
	}

	if (0 > (spinup_group_size = of_property_count_elems_of_size(of_eunit, DT_SYNO_SPINUP_GROUP, sizeof(u32)))) {
		// do not support spinup_group, no waiting need
		return need_waiting;
	}

	write_lock_irqsave(&acm->status_lock, flags);
	if (SPINUP_CHECK == spinup) {
		if (time_before(jiffies, acm->last_waking_time + spinup_group_delay*HZ) && 0 == (acm->waken_disks & (1 << disk_id))) {
			waken_disk_count = get_count_order(acm->waken_disks);
			for (group_num = 0; group_num < spinup_group_size && spinup == SPINUP_CHECK; group_num++) {
				of_property_read_u32_index(of_eunit, DT_SYNO_SPINUP_GROUP, group_num, &spinup_group);

				accum_waking_disks_available += spinup_group;
				if (accum_waking_disks_available == waken_disk_count) {
					// all previous waking up disks have occupy entire available waking up group
					// the previous group is full, this disk need to wait for delay
					spinup = SPINUP_DELAY;
				} else if (accum_waking_disks_available > waken_disk_count) {
					// this disk is in a waking up group, do not need delay
					spinup = SPINUP_NODELAY;
				}
			}
		} else {
			spinup = SPINUP_NODELAY;
		}
	}

	switch (spinup) {
	case SPINDOWN:
		acm->waken_disks &= ~(1 << disk_id);
		break;
	case SPINUP_CHECK:
		dev_info(&acm->control->dev, "waken disks status not checked, status should be SPINUP_NODELAY or SPINUP_DELAY\n");
		break;
	case SPINUP_NODELAY:
		acm->waken_disks |= (1 << disk_id);
		acm->last_waking_time = jiffies;
		break;
	case SPINUP_DELAY:
		need_waiting = 1;
		break;
	default:
		dev_info(&acm->control->dev, "%x is not a spinup opertion\n", spinup);
		break;
	}
	write_unlock_irqrestore(&acm->status_lock, flags);
	if (of_eunit) {
		of_node_put(of_eunit);
	}
	return need_waiting;
}

static int syno_get_command_offset(const char *eunit_unique, const char *dt_property, const int disk_slot) {
	struct device_node *of_eunit = NULL;
	struct device_node *pNode = NULL;
	int iOffset = -1;

	if (!eunit_unique || !dt_property || 0 >= disk_slot) {
		goto END;
	}

	if (NULL == (of_eunit = of_get_child_by_name(of_root, eunit_unique))) {
		goto END;
	}

	for_each_child_of_node(of_eunit, pNode) {
		if (!pNode->full_name || NULL == strstr(pNode->full_name, dt_property)) {
			continue;
		}
		if (0 == of_property_read_u32_index(pNode, DT_EUNIT_OFFSET, disk_slot - 1, &iOffset)) {
			break;
		}
	}

END:
	if (of_eunit) {
		of_node_put(of_eunit);
	}
	if (pNode) {
		of_node_put(pNode);
	}
	return iOffset;
}

int syno_usb_eunit_disk_is_wait_power_on(const int eunit_slot_type, const int eunit_slot_index, const int disk_slot_index)
{
	const char *usb_port = NULL;
	struct acm *acm = NULL;
	int iHddenableOffset = -1;
	int iHddpresentOffset = -1;
	int iNeedWait = -1;

	if (0 > eunit_slot_type || 0 >= eunit_slot_index || 0 >= disk_slot_index) {
		iNeedWait = -EINVAL;
		goto END;
	}

	if (0 > syno_acm_get_usb_port(&usb_port, eunit_slot_index)) {
		iNeedWait = -ENOENT;
		goto END;
	}

	if (NULL == (acm = syno_acm_get_by_usbport(usb_port))) {
		iNeedWait = -ENOENT;
		goto END;
	}

	if (0 > (iHddpresentOffset = syno_get_command_offset(acm->cached_expstatus[EUNIT_STATUS_EXPIDSET], DT_EUNIT_DISK_PRESENT, disk_slot_index))) {
		iNeedWait = -ENXIO;
		goto END;
	}
	if (0 > (iHddenableOffset = syno_get_command_offset(acm->cached_expstatus[EUNIT_STATUS_EXPIDSET], DT_EUNIT_DISK_POWER_ON, disk_slot_index))) {
		iNeedWait = -ENXIO;
		goto END;
	}

	if ((acm->cached_expstatus[EUNIT_STATUS_HDDENABLE] && (iHddenableOffset * 2) < strlen(acm->cached_expstatus[EUNIT_STATUS_HDDENABLE])) &&
		(acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT] && (iHddpresentOffset * 2) < strlen(acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT]))) {
		if (('1' == acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT][iHddpresentOffset * 2]) &
			('0' == acm->cached_expstatus[EUNIT_STATUS_HDDENABLE][iHddenableOffset * 2])) {
			iNeedWait = 1;
		} else {
			iNeedWait = 0;
		}
	}

END:
	return iNeedWait;
}
#endif /* MY_DEF_HERE */

static int acm_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct acm *acm;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	u8 ac_management_function = 0;
	u8 call_management_function = 0;
	int call_interface_num = -1;
	int data_interface_num = -1;
	unsigned long quirks;
	int num_rx_buf;
	int i;
	unsigned int elength = 0;
	int combined_interfaces = 0;
	struct device *tty_dev;
	int rv = -ENOMEM;
#ifdef MY_DEF_HERE
	struct syno_device_list *sdl = NULL;
	struct acm_device_temp *adt = NULL, *tmp = NULL;
	struct syno_acm_list *sal = NULL;
	unsigned long flags = 0;
#endif /* MY_DEF_HERE */

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;

	if (quirks == IGNORE_DEVICE)
		return -ENODEV;

	num_rx_buf = (quirks == SINGLE_RX_URB) ? 1 : ACM_NR;

	/* handle quirks deadly to normal probing*/
	if (quirks == NO_UNION_NORMAL) {
		data_interface = usb_ifnum_to_if(usb_dev, 1);
		control_interface = usb_ifnum_to_if(usb_dev, 0);
		/* we would crash */
		if (!data_interface || !control_interface)
			return -ENODEV;
		goto skip_normal_probe;
	}

	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	while (buflen > 0) {
		elength = buffer[0];
		if (!elength) {
			dev_err(&intf->dev, "skipping garbage byte\n");
			elength = 1;
			goto next_desc;
		}
		if (buffer[1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			if (elength < sizeof(struct usb_cdc_union_desc))
				goto next_desc;
			if (union_header) {
				dev_err(&intf->dev, "More than one "
					"union descriptor, skipping ...\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			if (elength < sizeof(struct usb_cdc_country_functional_desc))
				goto next_desc;
			cfd = (struct usb_cdc_country_functional_desc *)buffer;
			break;
		case USB_CDC_HEADER_TYPE: /* maybe check version */
			break; /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			if (elength < 4)
				goto next_desc;
			ac_management_function = buffer[3];
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			if (elength < 5)
				goto next_desc;
			call_management_function = buffer[3];
			call_interface_num = buffer[4];
			break;
		default:
			/*
			 * there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev, "Ignoring descriptor: "
					"type %02x, length %ud\n",
					buffer[2], elength);
			break;
		}
next_desc:
		buflen -= elength;
		buffer += elength;
	}

	if (!union_header) {
		if (call_interface_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			/* quirks for Droids MuIn LCD */
			if (quirks & NO_DATA_INTERFACE)
				data_interface = usb_ifnum_to_if(usb_dev, 0);
			else
				data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = call_interface_num));
			control_interface = intf;
		} else {
			if (intf->cur_altsetting->desc.bNumEndpoints != 3) {
				dev_dbg(&intf->dev,"No union descriptor, giving up\n");
				return -ENODEV;
			} else {
				dev_warn(&intf->dev,"No union descriptor, testing for castrated device\n");
				combined_interfaces = 1;
				control_interface = data_interface = intf;
				goto look_for_collapsed_interface;
			}
		}
	} else {
		control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
		data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
	}

	if (!control_interface || !data_interface) {
		dev_dbg(&intf->dev, "no interfaces\n");
		return -ENODEV;
	}

	if (data_interface_num != call_interface_num)
		dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

	if (control_interface == data_interface) {
		/* some broken devices designed for windows work this way */
		dev_warn(&intf->dev,"Control and data interfaces are not separated!\n");
		combined_interfaces = 1;
		/* a popular other OS doesn't use it */
		quirks |= NO_CAP_LINE;
		if (data_interface->cur_altsetting->desc.bNumEndpoints != 3) {
			dev_err(&intf->dev, "This needs exactly 3 endpoints\n");
			return -EINVAL;
		}
look_for_collapsed_interface:
		for (i = 0; i < 3; i++) {
			struct usb_endpoint_descriptor *ep;
			ep = &data_interface->cur_altsetting->endpoint[i].desc;

			if (usb_endpoint_is_int_in(ep))
				epctrl = ep;
			else if (usb_endpoint_is_bulk_out(ep))
				epwrite = ep;
			else if (usb_endpoint_is_bulk_in(ep))
				epread = ep;
			else
				return -EINVAL;
		}
		if (!epctrl || !epread || !epwrite)
			return -ENODEV;
		else
			goto made_compressed_probe;
	}

skip_normal_probe:

	/*workaround for switched interfaces */
	if (data_interface->cur_altsetting->desc.bInterfaceClass
						!= CDC_DATA_INTERFACE_TYPE) {
		if (control_interface->cur_altsetting->desc.bInterfaceClass
						== CDC_DATA_INTERFACE_TYPE) {
			dev_dbg(&intf->dev,
				"Your device has switched interfaces.\n");
			swap(control_interface, data_interface);
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (!combined_interfaces && intf != control_interface)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}


	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;


	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		swap(epread, epwrite);
	}
made_compressed_probe:
	dev_dbg(&intf->dev, "interfaces are valid\n");

	acm = kzalloc(sizeof(struct acm), GFP_KERNEL);
	if (acm == NULL)
		goto alloc_fail;

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) *
				(quirks == SINGLE_RX_URB ? 1 : 2);
	acm->combined_interfaces = combined_interfaces;
	acm->writesize = usb_endpoint_maxp(epwrite) * 20;
	acm->control = control_interface;
	acm->data = data_interface;

	usb_get_intf(acm->control); /* undone in destruct() */

	minor = acm_alloc_minor(acm);
	if (minor < 0) {
		rv = -ENODEV;
		goto alloc_fail1;
	}

	acm->minor = minor;
	acm->dev = usb_dev;
	acm->ctrl_caps = ac_management_function;
	if (quirks & NO_CAP_LINE)
		acm->ctrl_caps &= ~USB_CDC_CAP_LINE;
	acm->ctrlsize = ctrlsize;
	acm->readsize = readsize;
	acm->rx_buflimit = num_rx_buf;
	INIT_WORK(&acm->work, acm_softint);
	init_waitqueue_head(&acm->wioctl);
	spin_lock_init(&acm->write_lock);
	spin_lock_init(&acm->read_lock);
	mutex_init(&acm->mutex);
	acm->is_int_ep = usb_endpoint_xfer_int(epread);
	if (acm->is_int_ep)
		acm->bInterval = epread->bInterval;
	tty_port_init(&acm->port);
	acm->port.ops = &acm_port_ops;
	init_usb_anchor(&acm->delayed);
	acm->quirks = quirks;

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &acm->ctrl_dma);
	if (!buf)
		goto alloc_fail2;
	acm->ctrl_buffer = buf;

	if (acm_write_buffers_alloc(acm) < 0)
		goto alloc_fail4;

	acm->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!acm->ctrlurb)
		goto alloc_fail5;

	for (i = 0; i < num_rx_buf; i++) {
		struct acm_rb *rb = &(acm->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(acm->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail6;
		rb->index = i;
		rb->instance = acm;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail6;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (acm->is_int_ep) {
			usb_fill_int_urb(urb, acm->dev,
					 usb_rcvintpipe(usb_dev, epread->bEndpointAddress),
					 rb->base,
					 acm->readsize,
					 acm_read_bulk_callback, rb,
					 acm->bInterval);
		} else {
			usb_fill_bulk_urb(urb, acm->dev,
					  usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress),
					  rb->base,
					  acm->readsize,
					  acm_read_bulk_callback, rb);
		}

		acm->read_urbs[i] = urb;
		__set_bit(i, &acm->read_urbs_free);
	}
	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *snd = &(acm->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail7;

		if (usb_endpoint_xfer_int(epwrite))
			usb_fill_int_urb(snd->urb, usb_dev,
				usb_sndintpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, acm->writesize, acm_write_bulk, snd, epwrite->bInterval);
		else
			usb_fill_bulk_urb(snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, acm->writesize, acm_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		if (quirks & SEND_ZERO_PACKET)
			snd->urb->transfer_flags |= URB_ZERO_PACKET;
		snd->instance = acm;
	}

	usb_set_intfdata(intf, acm);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail7;

	if (cfd) { /* export the country data */
		acm->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!acm->country_codes)
			goto skip_countries;
		acm->country_code_size = cfd->bLength - 4;
		memcpy(acm->country_codes, (u8 *)&cfd->wCountyCode0,
							cfd->bLength - 4);
		acm->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(acm->country_codes);
			acm->country_codes = NULL;
			acm->country_code_size = 0;
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
						&dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(acm->country_codes);
			acm->country_codes = NULL;
			acm->country_code_size = 0;
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(acm->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 acm->ctrl_buffer, ctrlsize, acm_ctrl_irq, acm,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 16);
	acm->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	acm->ctrlurb->transfer_dma = acm->ctrl_dma;

	dev_info(&intf->dev, "ttyACM%d: USB ACM device\n", minor);

	acm->line.dwDTERate = cpu_to_le32(9600);
	acm->line.bDataBits = 8;
	acm_set_line(acm, &acm->line);

	usb_driver_claim_interface(&acm_driver, data_interface, acm);
	usb_set_intfdata(data_interface, acm);

#ifdef MY_DEF_HERE
	if (syno_is_synology_acm(acm)) {
		rwlock_init(&acm->status_lock);
		acm->initial_disk_not_ready_check = 0;

		spin_lock_irqsave(&acm_list_lock, flags);
		INIT_LIST_HEAD(&acm->syno_device_list);
		sal = kzalloc(sizeof(*sal), GFP_ATOMIC);
		if (!sal) {
			spin_unlock_irqrestore(&acm_list_lock, flags);
			goto alloc_fail8;
		}
		sal->acm = acm;
		list_add(&sal->device_list, &syno_acm_list_head);
		spin_unlock_irqrestore(&acm_list_lock, flags);

		acm->acm_buffer = kzalloc(sizeof(char)*SYNO_EUNIT_STATUS_BUFFER_SIZE, GFP_KERNEL);
		if (!acm->acm_buffer) {
			goto alloc_fail8;
		}
		acm->cached_expstatus = kzalloc(sizeof(char *) * EUNIT_STATUS_INDEX_END, GFP_KERNEL);
		if (!acm->cached_expstatus) {
			goto alloc_fail8;
		}

		for (i = 0; i < EUNIT_STATUS_INDEX_END; i++) {
			acm->cached_expstatus[i] = kzalloc(sizeof(char)*SYNO_DTS_PROPERTY_CONTENT_LENGTH, GFP_KERNEL);
			if (!acm->cached_expstatus[i]) {
				goto alloc_fail8;
			}
		}
		INIT_DELAYED_WORK(&acm->cache_update_work, syno_period_cache_update);

		acm_submit_read_urbs(acm, GFP_KERNEL);
		//TODO: macro
		syno_samd_tty_write(acm, DT_EUNIT_STATUS_EXPCTRL":1/");

		syno_period_cache_update(&acm->cache_update_work.work);

		syno_create_eunit_files(&intf->dev);

		/* Retry! wait for usb return hddpresent and parse usb string*/
		for (i = 0; i < SYNO_EUNIT_READY_RETRY; i++) {
			msleep(SYNO_EUNIT_ACM_WAITING_READY);
			if (0 != strlen(acm->cached_expstatus[EUNIT_STATUS_HDDPRESENT])) {
				break;
			}
		}

		syno_initial_not_ready_disk_count(acm);

		spin_lock_irqsave(&acm_list_lock, flags);
		list_for_each_entry_safe(adt, tmp, &acm_temp_device_list, device_list) {
			if (0 == strcmp(adt->usb_path, dev_name(&acm->dev->dev))) {
				sdl = kzalloc(sizeof(*sdl), GFP_ATOMIC);
				if (!sdl) {
					continue;
				}
				snprintf(sdl->disk_name, DISK_NAME_LEN, "%s", adt->disk_name);
				list_add(&sdl->device_list, &acm->syno_device_list);
			}
		}
		spin_unlock_irqrestore(&acm_list_lock, flags);
	}
#endif /* MY_DEF_HERE */

	tty_dev = tty_port_register_device(&acm->port, acm_tty_driver, minor,
			&control_interface->dev);
	if (IS_ERR(tty_dev)) {
		rv = PTR_ERR(tty_dev);
		goto alloc_fail8;
	}

	if (quirks & CLEAR_HALT_CONDITIONS) {
		usb_clear_halt(usb_dev, usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress));
		usb_clear_halt(usb_dev, usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress));
	}

	return 0;
alloc_fail8:
	if (!acm->combined_interfaces) {
		/* Clear driver data so that disconnect() returns early. */
		usb_set_intfdata(data_interface, NULL);
		usb_driver_release_interface(&acm_driver, data_interface);
	}

#ifdef MY_DEF_HERE
	if (syno_is_synology_acm(acm)) {
		cancel_delayed_work_sync(&acm->cache_update_work);
		syno_remove_eunit_files(&acm->control->dev);

		spin_lock_irqsave(&acm_list_lock, flags);
		if (sal) {
			list_del(&sal->device_list);
			kfree(sal);
		}
		spin_unlock_irqrestore(&acm_list_lock, flags);

		if (acm->acm_buffer) {
			kfree(acm->acm_buffer);
		}

		if (acm->cached_expstatus) {
			for (i = 0; i < EUNIT_STATUS_INDEX_END; i++) {
				if (acm->cached_expstatus[i]) {
					kfree(acm->cached_expstatus[i]);
				}
			}
			kfree(acm->cached_expstatus);
		}
	}
#endif /* MY_DEF_HERE */

	if (acm->country_codes) {
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
		kfree(acm->country_codes);
	}
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
alloc_fail7:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(acm->read_urbs[i]);
	acm_read_buffers_free(acm);
	usb_free_urb(acm->ctrlurb);
alloc_fail5:
	acm_write_buffers_free(acm);
alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
alloc_fail2:
	acm_release_minor(acm);
alloc_fail1:
	kfree(acm);
alloc_fail:
	return rv;
}

static void stop_data_traffic(struct acm *acm)
{
	int i;

	dev_dbg(&acm->control->dev, "%s\n", __func__);

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);

	cancel_work_sync(&acm->work);
}

static void acm_disconnect(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	int i;
#ifdef MY_DEF_HERE
	struct syno_acm_list *sal = NULL, *sal_tmp = NULL;
	unsigned long flags = 0;
#endif /* MY_DEF_HERE */

	dev_dbg(&intf->dev, "%s\n", __func__);

	/* sibling interface is already cleaning up */
	if (!acm)
		return;

	mutex_lock(&acm->mutex);
	acm->disconnected = true;

#ifdef MY_DEF_HERE
	if (syno_is_synology_acm(acm)) {
		spin_lock_irqsave(&acm_list_lock, flags);
		list_for_each_entry_safe(sal, sal_tmp, &syno_acm_list_head, device_list) {
			if (sal->acm == acm) {
				list_del(&sal->device_list);
				kfree(sal);
			}
		}
		spin_unlock_irqrestore(&acm_list_lock, flags);

		if (acm->acm_buffer) {
			kfree(acm->acm_buffer);
		}
		cancel_delayed_work_sync(&acm->cache_update_work);
		syno_remove_eunit_files(&acm->control->dev);
		for (i = 0; i < EUNIT_STATUS_INDEX_END; i++) {
			if (acm->cached_expstatus[i]) {
				kfree(acm->cached_expstatus[i]);
			}
		}
		if (acm->cached_expstatus) {
			kfree(acm->cached_expstatus);
		}
		//TODO: macro
		if (SYSTEM_POWER_OFF == system_state) {
			syno_samd_tty_write(acm, DT_EUNIT_STATUS_EXPCTRL":0/");
			for (i = 0; i < SYNO_EUNIT_READY_RETRY; i++) {
				if (0 == acm->transmitting) {
					break;
				}
				msleep(SYNO_EUNIT_ACM_WAITING_READY);
			}
		}
	}
#endif /* MY_DEF_HERE */

	if (acm->country_codes) {
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	wake_up_all(&acm->wioctl);
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
	usb_set_intfdata(acm->control, NULL);
	usb_set_intfdata(acm->data, NULL);
	mutex_unlock(&acm->mutex);

	tty = tty_port_tty_get(&acm->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}

	stop_data_traffic(acm);

	tty_unregister_device(acm_tty_driver, acm->minor);

	usb_free_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_urb(acm->read_urbs[i]);
	acm_write_buffers_free(acm);
	usb_free_coherent(usb_dev, acm->ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
	acm_read_buffers_free(acm);

	if (!acm->combined_interfaces)
		usb_driver_release_interface(&acm_driver, intf == acm->control ?
					acm->data : acm->control);

	tty_port_put(&acm->port);
}

#ifdef CONFIG_PM
static int acm_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct acm *acm = usb_get_intfdata(intf);
	int cnt;

	spin_lock_irq(&acm->write_lock);
	if (PMSG_IS_AUTO(message)) {
		if (acm->transmitting) {
			spin_unlock_irq(&acm->write_lock);
			return -EBUSY;
		}
	}
	cnt = acm->susp_count++;
	spin_unlock_irq(&acm->write_lock);

	if (cnt)
		return 0;

	stop_data_traffic(acm);

	return 0;
}

static int acm_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct urb *urb;
	int rv = 0;

	spin_lock_irq(&acm->write_lock);

	if (--acm->susp_count)
		goto out;

	if (test_bit(ASYNCB_INITIALIZED, &acm->port.flags)) {
		rv = usb_submit_urb(acm->ctrlurb, GFP_ATOMIC);

		for (;;) {
			urb = usb_get_from_anchor(&acm->delayed);
			if (!urb)
				break;

			acm_start_wb(acm, urb->context);
		}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto out;

		rv = acm_submit_read_urbs(acm, GFP_ATOMIC);
	}
out:
	spin_unlock_irq(&acm->write_lock);

	return rv;
}

static int acm_reset_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);

	if (test_bit(ASYNCB_INITIALIZED, &acm->port.flags))
		tty_port_tty_hangup(&acm->port, false);

	return acm_resume(intf);
}

#endif /* CONFIG_PM */

#define NOKIA_PCSUITE_ACM_INFO(x) \
		USB_DEVICE_AND_INTERFACE_INFO(0x0421, x, \
		USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM, \
		USB_CDC_ACM_PROTO_VENDOR)

#define SAMSUNG_PCSUITE_ACM_INFO(x) \
		USB_DEVICE_AND_INTERFACE_INFO(0x04e7, x, \
		USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM, \
		USB_CDC_ACM_PROTO_VENDOR)

/*
 * USB driver structure.
 */

static const struct usb_device_id acm_ids[] = {
	/* quirky and broken devices */
	{ USB_DEVICE(0x0424, 0x274e), /* Microchip Technology, Inc. (formerly SMSC) */
	  .driver_info = DISABLE_ECHO, }, /* DISABLE ECHO in termios flag */
	{ USB_DEVICE(0x076d, 0x0006), /* Denso Cradle CU-321 */
	.driver_info = NO_UNION_NORMAL, },/* has no union descriptor */
	{ USB_DEVICE(0x17ef, 0x7000), /* Lenovo USB modem */
	.driver_info = NO_UNION_NORMAL, },/* has no union descriptor */
	{ USB_DEVICE(0x0870, 0x0001), /* Metricom GS Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x045b, 0x023c),	/* Renesas USB Download mode */
	.driver_info = DISABLE_ECHO,	/* Don't echo banner */
	},
	{ USB_DEVICE(0x045b, 0x0248),	/* Renesas USB Download mode */
	.driver_info = DISABLE_ECHO,	/* Don't echo banner */
	},
	{ USB_DEVICE(0x045b, 0x024D),	/* Renesas USB Download mode */
	.driver_info = DISABLE_ECHO,	/* Don't echo banner */
	},
	{ USB_DEVICE(0x0e8d, 0x0003), /* FIREFLY, MediaTek Inc; andrey.arapov@gmail.com */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0e8d, 0x2000), /* MediaTek Inc Preloader */
	.driver_info = DISABLE_ECHO, /* DISABLE ECHO in termios flag */
	},
	{ USB_DEVICE(0x0e8d, 0x3329), /* MediaTek Inc GPS */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0482, 0x0203), /* KYOCERA AH-K3001V */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x079b, 0x000f), /* BT On-Air USB MODEM */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0ace, 0x1602), /* ZyDAS 56K USB MODEM */
	.driver_info = SINGLE_RX_URB,
	},
	{ USB_DEVICE(0x0ace, 0x1608), /* ZyDAS 56K USB MODEM */
	.driver_info = SINGLE_RX_URB, /* firmware bug */
	},
	{ USB_DEVICE(0x0ace, 0x1611), /* ZyDAS 56K USB MODEM - new version */
	.driver_info = SINGLE_RX_URB, /* firmware bug */
	},
	{ USB_DEVICE(0x11ca, 0x0201), /* VeriFone Mx870 Gadget Serial */
	.driver_info = SINGLE_RX_URB,
	},
	{ USB_DEVICE(0x1965, 0x0018), /* Uniden UBC125XLT */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x22b8, 0x7000), /* Motorola Q Phone */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0803, 0x3095), /* Zoom Telephonics Model 3095F USB MODEM */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1321), /* Conexant USB MODEM CX93010 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1324), /* Conexant USB MODEM RD02-D400 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1328), /* Shiro / Aztech USB MODEM UM-3100 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1349), /* Hiro (Conexant) USB MODEM H50228 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x20df, 0x0001), /* Simtec Electronics Entropy Key */
	.driver_info = QUIRK_CONTROL_LINE_STATE, },
	{ USB_DEVICE(0x2184, 0x001c) },	/* GW Instek AFG-2225 */
	{ USB_DEVICE(0x2184, 0x0036) },	/* GW Instek AFG-125 */
	{ USB_DEVICE(0x22b8, 0x6425), /* Motorola MOTOMAGX phones */
	},
	/* Motorola H24 HSPA module: */
	{ USB_DEVICE(0x22b8, 0x2d91) }, /* modem                                */
	{ USB_DEVICE(0x22b8, 0x2d92),   /* modem           + diagnostics        */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},
	{ USB_DEVICE(0x22b8, 0x2d93),   /* modem + AT port                      */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},
	{ USB_DEVICE(0x22b8, 0x2d95),   /* modem + AT port + diagnostics        */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},
	{ USB_DEVICE(0x22b8, 0x2d96),   /* modem                         + NMEA */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},
	{ USB_DEVICE(0x22b8, 0x2d97),   /* modem           + diagnostics + NMEA */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},
	{ USB_DEVICE(0x22b8, 0x2d99),   /* modem + AT port               + NMEA */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},
	{ USB_DEVICE(0x22b8, 0x2d9a),   /* modem + AT port + diagnostics + NMEA */
	.driver_info = NO_UNION_NORMAL, /* handle only modem interface          */
	},

	{ USB_DEVICE(0x0572, 0x1329), /* Hummingbird huc56s (Conexant) */
	.driver_info = NO_UNION_NORMAL, /* union descriptor misplaced on
					   data interface instead of
					   communications interface.
					   Maybe we should define a new
					   quirk for this. */
	},
	{ USB_DEVICE(0x0572, 0x1340), /* Conexant CX93010-2x UCMxx */
	.driver_info = NO_UNION_NORMAL,
	},
	{ USB_DEVICE(0x05f9, 0x4002), /* PSC Scanning, Magellan 800i */
	.driver_info = NO_UNION_NORMAL,
	},
	{ USB_DEVICE(0x1bbb, 0x0003), /* Alcatel OT-I650 */
	.driver_info = NO_UNION_NORMAL, /* reports zero length descriptor */
	},
	{ USB_DEVICE(0x1576, 0x03b1), /* Maretron USB100 */
	.driver_info = NO_UNION_NORMAL, /* reports zero length descriptor */
	},
	{ USB_DEVICE(0xfff0, 0x0100), /* DATECS FP-2000 */
	.driver_info = NO_UNION_NORMAL, /* reports zero length descriptor */
	},
	{ USB_DEVICE(0x09d8, 0x0320), /* Elatec GmbH TWN3 */
	.driver_info = NO_UNION_NORMAL, /* has misplaced union descriptor */
	},
	{ USB_DEVICE(0x0ca6, 0xa050), /* Castles VEGA3000 */
	.driver_info = NO_UNION_NORMAL, /* reports zero length descriptor */
	},

	{ USB_DEVICE(0x2912, 0x0001), /* ATOL FPrint */
	.driver_info = CLEAR_HALT_CONDITIONS,
	},

	/* Nokia S60 phones expose two ACM channels. The first is
	 * a modem and is picked up by the standard AT-command
	 * information below. The second is 'vendor-specific' but
	 * is treated as a serial device at the S60 end, so we want
	 * to expose it on Linux too. */
	{ NOKIA_PCSUITE_ACM_INFO(0x042D), }, /* Nokia 3250 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04D8), }, /* Nokia 5500 Sport */
	{ NOKIA_PCSUITE_ACM_INFO(0x04C9), }, /* Nokia E50 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0419), }, /* Nokia E60 */
	{ NOKIA_PCSUITE_ACM_INFO(0x044D), }, /* Nokia E61 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0001), }, /* Nokia E61i */
	{ NOKIA_PCSUITE_ACM_INFO(0x0475), }, /* Nokia E62 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0508), }, /* Nokia E65 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0418), }, /* Nokia E70 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0425), }, /* Nokia N71 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0486), }, /* Nokia N73 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04DF), }, /* Nokia N75 */
	{ NOKIA_PCSUITE_ACM_INFO(0x000e), }, /* Nokia N77 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0445), }, /* Nokia N80 */
	{ NOKIA_PCSUITE_ACM_INFO(0x042F), }, /* Nokia N91 & N91 8GB */
	{ NOKIA_PCSUITE_ACM_INFO(0x048E), }, /* Nokia N92 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0420), }, /* Nokia N93 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04E6), }, /* Nokia N93i  */
	{ NOKIA_PCSUITE_ACM_INFO(0x04B2), }, /* Nokia 5700 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0134), }, /* Nokia 6110 Navigator (China) */
	{ NOKIA_PCSUITE_ACM_INFO(0x046E), }, /* Nokia 6110 Navigator */
	{ NOKIA_PCSUITE_ACM_INFO(0x002f), }, /* Nokia 6120 classic &  */
	{ NOKIA_PCSUITE_ACM_INFO(0x0088), }, /* Nokia 6121 classic */
	{ NOKIA_PCSUITE_ACM_INFO(0x00fc), }, /* Nokia 6124 classic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0042), }, /* Nokia E51 */
	{ NOKIA_PCSUITE_ACM_INFO(0x00b0), }, /* Nokia E66 */
	{ NOKIA_PCSUITE_ACM_INFO(0x00ab), }, /* Nokia E71 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0481), }, /* Nokia N76 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0007), }, /* Nokia N81 & N81 8GB */
	{ NOKIA_PCSUITE_ACM_INFO(0x0071), }, /* Nokia N82 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04F0), }, /* Nokia N95 & N95-3 NAM */
	{ NOKIA_PCSUITE_ACM_INFO(0x0070), }, /* Nokia N95 8GB  */
	{ NOKIA_PCSUITE_ACM_INFO(0x00e9), }, /* Nokia 5320 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0099), }, /* Nokia 6210 Navigator, RM-367 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0128), }, /* Nokia 6210 Navigator, RM-419 */
	{ NOKIA_PCSUITE_ACM_INFO(0x008f), }, /* Nokia 6220 Classic */
	{ NOKIA_PCSUITE_ACM_INFO(0x00a0), }, /* Nokia 6650 */
	{ NOKIA_PCSUITE_ACM_INFO(0x007b), }, /* Nokia N78 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0094), }, /* Nokia N85 */
	{ NOKIA_PCSUITE_ACM_INFO(0x003a), }, /* Nokia N96 & N96-3  */
	{ NOKIA_PCSUITE_ACM_INFO(0x00e9), }, /* Nokia 5320 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0108), }, /* Nokia 5320 XpressMusic 2G */
	{ NOKIA_PCSUITE_ACM_INFO(0x01f5), }, /* Nokia N97, RM-505 */
	{ NOKIA_PCSUITE_ACM_INFO(0x02e3), }, /* Nokia 5230, RM-588 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0178), }, /* Nokia E63 */
	{ NOKIA_PCSUITE_ACM_INFO(0x010e), }, /* Nokia E75 */
	{ NOKIA_PCSUITE_ACM_INFO(0x02d9), }, /* Nokia 6760 Slide */
	{ NOKIA_PCSUITE_ACM_INFO(0x01d0), }, /* Nokia E52 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0223), }, /* Nokia E72 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0275), }, /* Nokia X6 */
	{ NOKIA_PCSUITE_ACM_INFO(0x026c), }, /* Nokia N97 Mini */
	{ NOKIA_PCSUITE_ACM_INFO(0x0154), }, /* Nokia 5800 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x04ce), }, /* Nokia E90 */
	{ NOKIA_PCSUITE_ACM_INFO(0x01d4), }, /* Nokia E55 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0302), }, /* Nokia N8 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0335), }, /* Nokia E7 */
	{ NOKIA_PCSUITE_ACM_INFO(0x03cd), }, /* Nokia C7 */
	{ SAMSUNG_PCSUITE_ACM_INFO(0x6651), }, /* Samsung GTi8510 (INNOV8) */

	/* Support for Owen devices */
	{ USB_DEVICE(0x03eb, 0x0030), }, /* Owen SI30 */

	/* NOTE: non-Nokia COMM/ACM/0xff is likely MSFT RNDIS... NOT a modem! */

	/* Support for Droids MuIn LCD */
	{ USB_DEVICE(0x04d8, 0x000b),
	.driver_info = NO_DATA_INTERFACE,
	},

#if IS_ENABLED(CONFIG_INPUT_IMS_PCU)
	{ USB_DEVICE(0x04d8, 0x0082),	/* Application mode */
	.driver_info = IGNORE_DEVICE,
	},
	{ USB_DEVICE(0x04d8, 0x0083),	/* Bootloader mode */
	.driver_info = IGNORE_DEVICE,
	},

	{ USB_DEVICE(0x04d8, 0xf58b),
	.driver_info = IGNORE_DEVICE,
	},
#endif

	/*Samsung phone in firmware update mode */
	{ USB_DEVICE(0x04e8, 0x685d),
	.driver_info = IGNORE_DEVICE,
	},

	/* Exclude Infineon Flash Loader utility */
	{ USB_DEVICE(0x058b, 0x0041),
	.driver_info = IGNORE_DEVICE,
	},

	/* Exclude ETAS ES58x */
	{ USB_DEVICE(0x108c, 0x0159), /* ES581.4 */
	.driver_info = IGNORE_DEVICE,
	},
	{ USB_DEVICE(0x108c, 0x0168), /* ES582.1 */
	.driver_info = IGNORE_DEVICE,
	},
	{ USB_DEVICE(0x108c, 0x0169), /* ES584.1 */
	.driver_info = IGNORE_DEVICE,
	},

	{ USB_DEVICE(0x1bc7, 0x0021), /* Telit 3G ACM only composition */
	.driver_info = SEND_ZERO_PACKET,
	},
	{ USB_DEVICE(0x1bc7, 0x0023), /* Telit 3G ACM + ECM composition */
	.driver_info = SEND_ZERO_PACKET,
	},

	/* Exclude Goodix Fingerprint Reader */
	{ USB_DEVICE(0x27c6, 0x5395),
	.driver_info = IGNORE_DEVICE,
	},

	/* Exclude Heimann Sensor GmbH USB appset demo */
	{ USB_DEVICE(0x32a7, 0x0000),
	.driver_info = IGNORE_DEVICE,
	},

#ifdef MY_DEF_HERE
	{ USB_DEVICE(0x4d8, 0xa),
	.driver_info = DISABLE_ECHO, /* DISABLE ECHO in termios flag */
	},
#endif /* MY_DEF_HERE */

	/* control interfaces without any protocol set */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_PROTO_NONE) },

	/* control interfaces with various AT-command sets */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_V25TER) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101_WAKE) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_GSM) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_3G) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_CDMA) },

	{ USB_DEVICE(0x1519, 0x0452), /* Intel 7260 modem */
	.driver_info = SEND_ZERO_PACKET,
	},

	{ }
};

MODULE_DEVICE_TABLE(usb, acm_ids);

static struct usb_driver acm_driver = {
	.name =		"cdc_acm",
	.probe =	acm_probe,
	.disconnect =	acm_disconnect,
#ifdef CONFIG_PM
	.suspend =	acm_suspend,
	.resume =	acm_resume,
	.reset_resume =	acm_reset_resume,
#endif
	.id_table =	acm_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
	.disable_hub_initiated_lpm = 1,
};

/*
 * TTY driver structures.
 */

static const struct tty_operations acm_ops = {
	.install =		acm_tty_install,
	.open =			acm_tty_open,
	.close =		acm_tty_close,
	.cleanup =		acm_tty_cleanup,
	.hangup =		acm_tty_hangup,
	.write =		acm_tty_write,
	.write_room =		acm_tty_write_room,
	.ioctl =		acm_tty_ioctl,
	.throttle =		acm_tty_throttle,
	.unthrottle =		acm_tty_unthrottle,
	.chars_in_buffer =	acm_tty_chars_in_buffer,
	.break_ctl =		acm_tty_break_ctl,
	.set_termios =		acm_tty_set_termios,
	.tiocmget =		acm_tty_tiocmget,
	.tiocmset =		acm_tty_tiocmset,
};

/*
 * Init / exit.
 */

static int __init acm_init(void)
{
	int retval;
	acm_tty_driver = alloc_tty_driver(ACM_TTY_MINORS);
	if (!acm_tty_driver)
		return -ENOMEM;
	acm_tty_driver->driver_name = "acm",
	acm_tty_driver->name = "ttyACM",
	acm_tty_driver->major = ACM_TTY_MAJOR,
	acm_tty_driver->minor_start = 0,
	acm_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	acm_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	acm_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	acm_tty_driver->init_termios = tty_std_termios;
	acm_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
								HUPCL | CLOCAL;
	tty_set_operations(acm_tty_driver, &acm_ops);

	retval = tty_register_driver(acm_tty_driver);
	if (retval) {
		put_tty_driver(acm_tty_driver);
		return retval;
	}

	retval = usb_register(&acm_driver);
	if (retval) {
		tty_unregister_driver(acm_tty_driver);
		put_tty_driver(acm_tty_driver);
		return retval;
	}

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");

	return 0;
}

static void __exit acm_exit(void)
{
	usb_deregister(&acm_driver);
	tty_unregister_driver(acm_tty_driver);
	put_tty_driver(acm_tty_driver);
	idr_destroy(&acm_minors);
}

module_init(acm_init);
module_exit(acm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ACM_TTY_MAJOR);
