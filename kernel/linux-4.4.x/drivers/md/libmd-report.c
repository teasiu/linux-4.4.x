#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
// Copyright (c) 2000-2014 Synology Inc. All rights reserved.
#ifdef MY_ABC_HERE
#include <linux/bio.h>
#include <linux/synobios.h>
#include <linux/synolib.h>

#include <linux/raid/libmd-report.h>
#include "md.h"
#include "dm.h"

int (*funcSYNOSendRaidEvent)(unsigned int, unsigned int, unsigned int, unsigned long long) = NULL;

void SynoReportFaultyDevice(int md_minor, struct block_device *bdev)
{
	int index  = SynoDiskGetDeviceIndex(bdev);

	if (funcSYNOSendRaidEvent) {
		funcSYNOSendRaidEvent(MD_FAULTY_DEVICE, md_minor, index, 0);
	}
}
EXPORT_SYMBOL(SynoReportFaultyDevice);

void SynoReportBadSector(sector_t sector, unsigned long rw,
						 int md_minor, struct block_device *bdev, const char *szFuncName)
{
	char b[BDEVNAME_SIZE];
	int index = SynoDiskGetDeviceIndex(bdev);

	bdevname(bdev,b);

	printk("%s error, md%d, %s index [%d], sector %llu [%s]\n",
				   rw ? "write" : "read", md_minor, b, index, (unsigned long long)sector, szFuncName);

	if (funcSYNOSendRaidEvent) {
		funcSYNOSendRaidEvent(
			(rw == WRITE) ? MD_SECTOR_WRITE_ERROR : MD_SECTOR_READ_ERROR,
			md_minor, index, sector);
	}
}

EXPORT_SYMBOL(SynoReportBadSector);

void SynoReportCorrectBadSector(sector_t sector, int md_minor,
								struct block_device *bdev, const char *szFuncName)
{
	char b[BDEVNAME_SIZE];
	int index = SynoDiskGetDeviceIndex(bdev);

	bdevname(bdev,b);

	printk("read error corrected, md%d, %s index [%d], sector %llu [%s]\n",
				   md_minor, b, index, (unsigned long long)sector, szFuncName);

	if (funcSYNOSendRaidEvent) {
		funcSYNOSendRaidEvent(MD_SECTOR_REWRITE_OK, md_minor,
							  index, sector);
	}
}
EXPORT_SYMBOL(SynoReportCorrectBadSector);
EXPORT_SYMBOL(funcSYNOSendRaidEvent);

#ifdef MY_ABC_HERE
int (*funcSYNOSendAutoRemapRaidEvent)(unsigned int, unsigned long long, unsigned int) = NULL;
void SynoAutoRemapReport(struct mddev *mddev, sector_t sector, struct block_device *bdev)
{
	int index = SynoDiskGetDeviceIndex(bdev);

	if (NULL == funcSYNOSendAutoRemapRaidEvent) {
		printk("Can't reference to function 'SYNOSendAutoRemapRaidEvent'\n");
	} else {
		printk("report md[%d] auto-remapped sector:[%llu]\n",
			mddev->md_minor, (unsigned long long)sector);
		funcSYNOSendAutoRemapRaidEvent(mddev->md_minor, sector, (unsigned int)index);
	}
}

EXPORT_SYMBOL(SynoAutoRemapReport);
EXPORT_SYMBOL(funcSYNOSendAutoRemapRaidEvent);
#endif /* MY_ABC_HERE */
#endif /* MY_ABC_HERE */
