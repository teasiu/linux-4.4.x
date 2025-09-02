/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#ifndef _LINUX_SYNO_CACHE_PROTECTION_INTERNAL_H
#define _LINUX_SYNO_CACHE_PROTECTION_INTERNAL_H

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/kobject.h>
#include <linux/syno_cache_protection.h>

struct syno_cache_protection_instance {
	struct list_head active_uuids;
	struct list_head passive_uuids;
	struct list_head fs_types;
	spinlock_t active_uuid_lock;
	spinlock_t passive_uuid_lock;
	spinlock_t fs_type_lock;
	const struct syno_cache_protection_connection_operations *c_op;
	const struct syno_cache_protection_space_allocate_operations *s_op;
	spinlock_t reclaim_lock;
	bool local_metadata_reclaim, local_data_reclaim; /* used for active node */
	bool remote_metadata_reclaim, remote_data_reclaim; /* used for passive node */
};

extern struct syno_cache_protection_instance *instance;

#endif /* _LINUX_SYNO_CACHE_PROTECTION_INTERNAL_H */

