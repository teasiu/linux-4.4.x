/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#ifndef __BTRFS_SYNO_CACHE_PROTECTION_PASSIVE_MODEL_
#define __BTRFS_SYNO_CACHE_PROTECTION_PASSIVE_MODEL_

#include <linux/types.h>
#include <linux/rbtree.h>
#include <linux/spinlock_types.h>
#include <linux/time.h>
#include <linux/syno_cache_protection.h>
#include "syno-cache-protection-btrfs.h"

/*
 * Passive memory layout and model
 */

#define SYNO_CACHE_PROTECTION_NAME_INLINE_LEN 64
#define SYNO_CACHE_PROTECTION_COMMAND_EXTRA_BUFFER 4
#define SYNO_CACHE_PROTECTION_VIRTUAL_BUFFER_MAX_PAGES 256

enum SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_TYPE {
	SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_METADATA = 0,
	SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_DATA,
	SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_VIRTUAL_BUFFER,
};

struct syno_cache_protection_passive_btrfs_page {
	struct rb_node page_node;
	u64 pg_offset;
	atomic_t refs;
	void *value;
	/* for reclaim */
	struct syno_cache_protection_passive_btrfs_instance *instance;
	struct syno_cache_protection_passive_btrfs_inode *inode;
	struct list_head lru_list;
	atomic64_t version;
};

struct syno_cache_protection_passive_btrfs_inode {
	struct rb_node inode_node;
	u64 subvolid;
	u64 inum;
	struct rb_root page_tree;
	spinlock_t lock;
	atomic_t refs;
	u64 i_size;
};

struct syno_cache_protection_passive_btrfs_virtual_buffer {
	enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type;
	u32 count;
	u64 size;
	void *pages[SYNO_CACHE_PROTECTION_VIRTUAL_BUFFER_MAX_PAGES];
};

struct syno_cache_protection_passive_btrfs_buffer {
	enum SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_TYPE type;
	void *data;
};

struct syno_cache_protection_passive_btrfs_buffers {
	size_t count;
	struct syno_cache_protection_passive_btrfs_buffer buffer[SYNO_CACHE_PROTECTION_COMMAND_EXTRA_BUFFER];
};

struct syno_cache_protection_passive_btrfs_metadata_command {
	struct list_head list;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	u64 transid;
	struct syno_cache_protection_passive_btrfs_buffers extra_buffers;
};

struct syno_cache_protection_passive_btrfs_ordered_extent {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 inum;
	u64 file_offset;
	u64 start;
	u64 len;
	u64 disk_len;
	u64 truncated_len;
	u64 flags;
	u32 compress_type;
	u64 i_size;
	u32 total_csums;
	u32 total_csum_size;
	struct syno_cache_protection_passive_btrfs_virtual_buffer *csums;
};

struct syno_cache_protection_passive_btrfs_inline_extent {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 inum;
	u64 inline_len;
	void *inline_data;
};

struct syno_cache_protection_passive_btrfs_create {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 dir;
	u64 inum;
	u64 generation;
	u64 mode;
	u64 rdev;
	u64 name_len;
	u64 symname_len;
	unsigned char *name;
	struct syno_cache_protection_passive_btrfs_virtual_buffer *symname;
	unsigned char iname[SYNO_CACHE_PROTECTION_NAME_INLINE_LEN];
};

struct syno_cache_protection_passive_btrfs_inode_operation {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 inum;
	u64 flags;
	u64 mode;
	u32 uid;
	u32 gid;
	struct timespec times[2];
	u64 offset;
	u64 length;
};

struct syno_cache_protection_passive_btrfs_rename {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 old_dir;
	u64 new_dir;
	u64 old_name_len;
	unsigned char *old_name;
	u64 new_name_len;
	unsigned char *new_name;
};

struct syno_cache_protection_passive_btrfs_clone {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 src_subvolid;
	u64 src_inum;
	u64 src_offset;
	u64 len;
	u64 dst_subvolid;
	u64 dst_inum;
	u64 dst_offset;
};

struct syno_cache_protection_passive_btrfs_xattr {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 inum;
	u32 name_size;
	u32 value_size;
	unsigned char *name;
	unsigned char iname[SYNO_CACHE_PROTECTION_NAME_INLINE_LEN];
	struct syno_cache_protection_passive_btrfs_virtual_buffer *value;
	u32 flags;
};

struct syno_cache_protection_passive_btrfs_subvol_operation {
	struct syno_cache_protection_passive_btrfs_metadata_command node;
	u64 subvolid;
	u64 inum;
	u64 uid;
	struct btrfs_ioctl_qgroup_create_args qgroup_ca;
	struct btrfs_ioctl_qgroup_assign_args qgroup_aa;
	struct btrfs_ioctl_qgroup_limit_args qgroup_la;
	struct btrfs_ioctl_usrquota_limit_args usrquota_la;
};

struct syno_cache_protection_passive_btrfs_instance {
	size_t uuid_len;
	u8 uuid[SYNO_CACHE_PROTECTION_UUID_SIZE_MAX]; /* FS specific uuid */
	struct syno_cache_protection_fs *cache_protection_fs;
	spinlock_t lock;
	struct rb_root inode_tree;
	struct list_head metadata_command_head;
	atomic64_t last_transid;
	u64 old_generation;
	/* page reclaim */
	struct list_head lru_page_head;
	atomic64_t reclaim_version;
	struct work_struct lru_page_reclaim_work;
};

struct syno_cache_protection_passive_btrfs_instance* syno_cache_protection_passive_btrfs_instance_alloc(struct syno_cache_protection_fs *fs);
void syno_cache_protection_passive_btrfs_instance_free(struct syno_cache_protection_passive_btrfs_instance *passive_instance);
struct syno_cache_protection_passive_btrfs_inode* syno_cache_protection_passive_btrfs_get_or_alloc_inode(
						struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						u64 subvolid, u64 inum, bool create, bool reserved);
struct syno_cache_protection_passive_btrfs_ordered_extent* syno_cache_protection_passive_btrfs_ordered_extent_alloc(u64 transid, u64 subvolid, u64 inum, u64 file_offset,
		u64 start, u64 len, u64 disk_len, u64 truncated_len, u64 flags, u32 compress_type, u64 i_size, u32 total_csums, u32 total_csum_size, bool reserved);
struct syno_cache_protection_passive_btrfs_inline_extent* syno_cache_protection_passive_btrfs_inline_extent_alloc(u64 transid, u64 subvolid, u64 inum,
		u64 inline_len, bool reserved);
void syno_cache_protection_passive_btrfs_metadata_command_free(struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command);
void syno_cache_protection_passive_btrfs_inode_free(struct syno_cache_protection_passive_btrfs_instance *passive_instance, struct syno_cache_protection_passive_btrfs_inode *inode);
struct syno_cache_protection_passive_btrfs_page* syno_cache_protection_passive_btrfs_get_or_alloc_page(
						struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_passive_btrfs_inode *inode,
						u64 pg_offset, bool reserved, bool *new_alloc);
struct syno_cache_protection_passive_btrfs_page *syno_cache_protection_passive_btrfs_page_tree_search(struct rb_root *root,
					  u64 pg_offset);
struct syno_cache_protection_passive_btrfs_page *syno_cache_protection_passive_btrfs_page_tree_search_with_range(struct rb_root *root,
					  u64 pg_start, u64 pg_end);
void syno_cache_protection_passive_btrfs_page_free(struct syno_cache_protection_passive_btrfs_page *page);
struct syno_cache_protection_passive_btrfs_create* syno_cache_protection_passive_btrfs_create_alloc(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
		u64 subvolid, u64 dir, u64 inum, u64 generation, u64 mode, u64 rdev, u64 name_len, void *name, bool reserved);
struct syno_cache_protection_passive_btrfs_inode_operation* syno_cache_protection_passive_btrfs_inode_operation_alloc(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
		u64 subvolid, u64 inum, u64 flags, u64 mode, u32 uid, u32 gid, struct btrfs_timespec *times, u64 offset, u64 length, bool reserved);

struct syno_cache_protection_passive_btrfs_rename*
syno_cache_protection_passive_btrfs_rename_alloc(
	u64 transid, u64 subvolid, u64 old_dir, u64 new_dir,
	u64 old_name_len, void *old_name, u64 new_name_len, void *new_name,
	bool reserved);

struct syno_cache_protection_passive_btrfs_clone*
syno_cache_protection_passive_btrfs_clone_alloc(
	u64 transid, u64 src_subvolid, u64 src_inum, u64 src_offset, u64 len,
	u64 dst_subvolid, u64 dst_inum, u64 dst_offset, bool reserved);

struct syno_cache_protection_passive_btrfs_xattr*
	syno_cache_protection_passive_btrfs_xattr_alloc(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
	u64 subvolid, u64 inum, u32 name_size, u32 value_size, void *name, u32 flags, bool reserved);

struct syno_cache_protection_passive_btrfs_subvol_operation*
syno_cache_protection_passive_btrfs_subvol_operation_alloc(
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
	u64 subvolid, u64 inum, u64 create, u64 qgroupid, u64 assign,
	u64 src, u64 dst, u64 uid, struct btrfs_qgroup_limit_item qgroup_limit,
	struct btrfs_usrquota_limit_item usrquota_limit, bool reserved);

int syno_cache_protection_passive_btrfs_buffer_insert(struct syno_cache_protection_passive_btrfs_buffers *buffers, enum SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_TYPE type, void *data);
struct syno_cache_protection_passive_btrfs_virtual_buffer* syno_cache_protection_passive_btrfs_virtual_buffer_alloc(u64 len, bool reserved, enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type);
int syno_cache_protection_passive_btrfs_virtual_buffer_write(struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer, u64 pos, u64 len, const char *srcv);
int syno_cache_protection_passive_btrfs_virtual_buffer_read(struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer, u64 pos, u64 len, void *dstv);
void syno_cache_protection_passive_btrfs_virtual_buffer_free(struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer);
int syno_cache_protection_passive_btrfs_virtual_buffer_fill_from_request(void *req, struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer, size_t pos, size_t len);

#endif /* __BTRFS_SYNO_CACHE_PROTECTION_PASSIVE_MODEL_ */
