/*
 * Copyright (C) 2020 Synology Inc.  All rights reserved.
 */

#ifndef _LINUX_SYNO_CACHE_PROTECTION_UTIL_H
#define _LINUX_SYNO_CACHE_PROTECTION_UTIL_H

#define TITLE "syno_cache_protection: "
#define syno_cache_protection_err(fmt, args...) \
	printk(KERN_ERR TITLE"%s [%d]: "fmt"\n", __FUNCTION__, __LINE__, ##args);
#define syno_cache_protection_warn(fmt, args...) \
	printk(KERN_WARNING TITLE"%s [%d]: "fmt"\n", __FUNCTION__, __LINE__, ##args);

#endif /* _LINUX_SYNO_CACHE_PROTECTION_UTIL_H */

