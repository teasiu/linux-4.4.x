// Copyright (c) 2000-2019 Synology Inc. All rights reserved.

#ifndef _LINUX_SYNO_CACHE_PROTECTION_H
#define _LINUX_SYNO_CACHE_PROTECTION_H

#include <linux/types.h>
#include <linux/pagemap.h>

#define SYNO_CACHE_PROTECTION_UUID_SIZE_MAX 40

#define SYNO_CACHE_PROTECTION_METADATA_SIZE 256

#define SYNO_CACHE_PROTECTION_DATA_SIZE (PAGE_CACHE_SIZE)
#define SYNO_CACHE_PROTECTION_DATA_SHIFT (PAGE_CACHE_SHIFT)

#define SYNO_CACHE_PROTECTION_REQUEST_MAX_DATA_PAGES 16

#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MAX 6
#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_WAIT 0
#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_NOWAIT 1
#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_HIGH_PRIORITY_1 2
#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_HIGH_PRIORITY_2 3
#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_HIGH_PRIORITY_3 4
#define SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE 5

enum SYNO_CACHE_PROTECTION_FS_TYPE {
	SYNO_CACHE_PROTECTION_FS_BTRFS = 0,
	SYNO_CACHE_PROTECTION_FS_ALL
};

enum SYNO_CACHE_PROTECTION_ROLE {
	SYNO_CACHE_PROTECTION_ROLE_ACTIVE = 0,
	SYNO_CACHE_PROTECTION_ROLE_PASSIVE
};

enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE {
	SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA = 0,
	SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM,
	SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER,
	SYNO_CACHE_PROTECTION_SPACE_POOL_DATA,
	SYNO_CACHE_PROTECTION_SPACE_POOL_MAX
};

struct syno_cache_protection_connection_operations {
	void* (*get_req)(size_t size, size_t channel);
	void (*put_req)(void *req);
	int (*write_req)(void *req, size_t len, const void *data);
	int (*read_req)(void *req, size_t len, void *data);
	int (*send_req)(void *req, bool wait);
	int (*status)(char* buf);
};

struct syno_cache_protection_space_allocate_operations {
	void* (*alloc)(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, gfp_t gfp_mask, bool reserved);
	void (*free)(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, void *data);
	int (*reserve)(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count, gfp_t gfp_mask);
	void (*reserve_free)(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count);
	int (*status)(char* buf);
	int (*enable)(void);
	int (*disable)(void);
	bool (*enabled)(void);
	void (*link_event)(bool link_is_up);
};

struct syno_cache_protection_fs_type {
	struct module *owner;
	size_t id;
	struct list_head list;
	struct syno_cache_protection_fs *(*alloc_passive_instance)(size_t uuid_len, u8 *uuid);
	void (*free_instance)(struct syno_cache_protection_fs *fs);
	bool enabled;
};

struct syno_cache_protection_fs {
	struct syno_cache_protection_fs_type *fs_type;
	enum SYNO_CACHE_PROTECTION_ROLE role;
	size_t uuid_len;
	u8 uuid[SYNO_CACHE_PROTECTION_UUID_SIZE_MAX]; /* FS specific uuid */
	void *private;
	int (*reclaim)(void *private, bool metadata);
	int (*do_command)(void *private, void *req);
	bool enabled;
	u64 count;
	atomic_t refs;
	struct list_head list;
	struct list_head unbind_list;
	spinlock_t lock;
	wait_queue_head_t wait;
	bool module_ref;
};

extern int syno_cache_protection_send_ctl_ping(void);
extern int syno_cache_protection_send_ctl_space_enable(void);
extern struct syno_cache_protection_fs *syno_cache_protection_alloc_fs_instance(void);
extern void syno_cache_protection_fs_get(struct syno_cache_protection_fs *fs);
extern void syno_cache_protection_fs_put(struct syno_cache_protection_fs *fs);
extern int syno_cache_protection_add(struct syno_cache_protection_fs *fs);
extern void syno_cache_protection_remove(struct syno_cache_protection_fs *fs);
extern int syno_cache_protection_register_fs(struct syno_cache_protection_fs_type *fs_type);
extern void syno_cache_protection_unregister_fs(struct syno_cache_protection_fs_type *fs_type);
extern int syno_cache_protection_clear_passive_instance_with_fs(enum SYNO_CACHE_PROTECTION_ROLE role, size_t fs_type, size_t uuid_len, u8 *uuid);
extern int syno_cache_protection_alloc_passive_instance_with_fs(enum SYNO_CACHE_PROTECTION_ROLE role, size_t fs_type, size_t uuid_len, u8 *uuid);
extern int syno_cache_protection_clear_all(void);
extern void syno_cache_protection_all_reclaim(bool metadata);
extern void syno_cache_protection_send_reclaim(bool metadata);
extern void syno_cache_protection_send_reclaim_end(bool metadata);
extern bool syno_cache_protection_check_local_reclaim(bool metadata);
extern int syno_cache_protection_do_request(void *req);
extern void* syno_cache_protection_get_request(struct syno_cache_protection_fs *fs, size_t len, size_t channel);
extern int syno_cache_protection_write_request(void *req, size_t len, const void *data);
extern int syno_cache_protection_read_request(void *req, size_t len, void *data);
extern int syno_cache_protection_send_request(void *req, bool wait);
extern void syno_cache_protection_put_request(void *req);
extern void* syno_cache_protection_space_alloc(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, gfp_t gfp_mask, bool reserved);
extern void syno_cache_protection_space_free(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, void *data);
extern int syno_cache_protection_space_reserve(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count, gfp_t gfp_mask);
extern void syno_cache_protection_space_reserve_free(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count);
extern struct syno_cache_protection_fs* syno_cache_protection_get_passive_instance(size_t fs_type, size_t uuid_len, const u8 *uuid);
extern void syno_cache_protection_passive_remove(struct syno_cache_protection_fs *fs);
extern void syno_cache_protection_connection_link_event(bool link_is_up);

extern int __init syno_cache_protection_ntb_connection_init(void);
extern void syno_cache_protection_ntb_connection_exit(void);
extern const struct syno_cache_protection_connection_operations* syno_cache_protection_ntb_connection_get_connections(void);

extern int __init syno_cache_protection_mem_pool_init(void);
extern void syno_cache_protection_mem_pool_exit(void);
extern const struct syno_cache_protection_space_allocate_operations* syno_cache_protection_mem_pool_get_space_allocator(void);

extern int __init syno_cache_protection_init_sysfs(void);
extern void syno_cache_protection_exit_sysfs(void);

#endif /* _LINUX_SYNO_CACHE_PROTECTION_H */
