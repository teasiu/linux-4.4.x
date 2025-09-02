/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#ifndef __BTRFS_SYNO_CACHE_PROTECTION_COMMAND_
#define __BTRFS_SYNO_CACHE_PROTECTION_COMMAND_

#include <linux/syno_cache_protection.h>
#include "../ctree.h"

/* for write/reserve */
#define SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_WAIT SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_WAIT

/* for metadata operation */
#define SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_NOWAIT

/* for reserve free */
#define SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_RESERVE_FREE SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_HIGH_PRIORITY_1

/* for extent ordered/inline */
#define SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_EXTENT SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_HIGH_PRIORITY_2

/* for commit, reclaim */
#define SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_HIGH SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_HIGH_PRIORITY_3

struct syno_cache_protection_btrfs_command_operations {
	bool wait;
	size_t channel;
	bool skip_check_enabled;
	size_t (*size)(void *data);
	int (*send)(void *data, void *req);
	int (*receive)(void *private, void *req, bool reserved);
	int (*reserve)(void *reserve_parm, void *data);
};

struct syno_cache_protection_timespec {
	__le64 sec;
	__le32 nsec;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_header {
	__le32 command;
	__le32 reserved;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_checkpoint_end {
	__le64 transid;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_write {
	__le64 subvolid;
	__le64 inum;
	__le32 num_pages;
	__le64 page_index;
	__le64 i_size;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_ordered_extent_csum {
	__le64 bytenr;
	__le32 len;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_ordered_extent {
	__le32 err;
	__le64 transid;
	__le64 subvolid;
	__le64 inum;
	__le64 file_offset;
	__le64 start;
	__le64 len;
	__le64 disk_len;
	__le64 truncated_len;
	__le64 flags;
	__le32 compress_type;
	__le32 bl_update_isize;
	__le64 i_size;
	__le32 total_csums;
	__le32 total_csum_size;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_inline_extent {
	__le32 err;
	__le64 transid;
	__le64 subvolid;
	__le64 inum;
	__le64 inline_len;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_create {
	__le32 type;
	__le64 transid;
	__le64 subvolid;
	__le64 dir;
	__le64 inum;
	__le64 generation;
	__le64 mode;
	__le64 rdev;
	__le64 nlink;
	__le64 name_len;
	__le64 symname_len;
	__u8 name[BTRFS_NAME_LEN];
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_inode_operation {
	__le32 type;
	__le64 transid;
	__le64 subvolid;
	__le64 inum;
	__le64 flags;
	__le64 mode;
	__le32 uid;
	__le32 gid;
	struct btrfs_timespec times[2];
	__le64 offset;
	__le64 length;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_rename {
	__le64 transid;
	__le64 subvolid;
	__le64 old_dir;
	__le64 new_dir;
	__le32 old_name_len;
	__u8 old_name[BTRFS_NAME_LEN];
	__le32 new_name_len;
	__u8 new_name[BTRFS_NAME_LEN];
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_clone {
	__le64 transid;
	__le64 src_subvolid;
	__le64 src_inum;
	__le64 src_offset;
	__le64 len;
	__le64 dst_subvolid;
	__le64 dst_inum;
	__le64 dst_offset;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_xattr {
	__le32 type;
	__le64 transid;
	__le64 subvolid;
	__le64 inum;
	__le32 name_size;
	__le32 value_size;
	__u8 name[XATTR_NAME_MAX];
	__le32 flags;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_subvol_operation {
	__le32 type;
	__le64 transid;
	__le64 subvolid;
	__le64 inum;
	__le64 create;
	__le64 qgroupid;
	__le64 assign;
	__le64 src;
	__le64 dst;
	__le64 uid;
	struct btrfs_qgroup_limit_item qgroup_limit;
	struct btrfs_usrquota_limit_item usrquota_limit;
} __attribute__ ((__packed__));

struct syno_cache_protection_stream_btrfs_command_space_reserve {
	__le32 count[SYNO_CACHE_PROTECTION_SPACE_POOL_MAX];
} __attribute__ ((__packed__));

const struct syno_cache_protection_btrfs_command_operations *syno_cache_protection_btrfs_get_command_ops(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command);

#endif /* __BTRFS_SYNO_CACHE_PROTECTION_COMMAND_ */
