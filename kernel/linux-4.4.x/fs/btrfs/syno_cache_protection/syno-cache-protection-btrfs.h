/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#ifndef __BTRFS_SYNO_CACHE_PROTECTION_
#define __BTRFS_SYNO_CACHE_PROTECTION_

#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/mm_types.h>
#include <linux/syno_cache_protection.h>

#define BTRFS_LEAF_SIZE 16384

struct btrfs_fs_info;
struct btrfs_ordered_extent;
struct syno_cache_protection_passive_btrfs_instance;

enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND {
	/* sender */
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHECKPOINT_END = 0,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DATA_RECLAIM = 1,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_WRITE = 2,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT = 3,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INLINE_EXTENT = 4,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SPACE_RESERVE = 5,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SPACE_RESERVE_FREE = 6,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKFILE = 7,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKNOD = 8,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKDIR = 9,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_LINK = 10,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SYMLINK = 11,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_FLAGS = 12,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RMDIR = 13,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_UNLINK = 14,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_UTIME = 15,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHMODE = 16,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHOWN = 17,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RENAME = 18,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_TRUNCATE = 19,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE = 20,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CLONE = 21,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR = 22,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_REMOVEXATTR = 23,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DEFAULT_SUBVOL = 24,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SUBVOL_DELETE = 25,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_CREATE = 26,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_ASSIGN = 27,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_LIMIT = 28,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_LIMIT = 29,
	SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_CLEAN = 30,
};

struct syno_cache_protection_parameter_command_generic {
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	void *parm;
};

struct syno_cache_protection_parameter_command_checkpoint_end {
	u64 transid;
};
struct syno_cache_protection_parameter_command_write {
	struct inode *inode;
	size_t num_pages;
	struct page **pages;
};
struct syno_cache_protection_parameter_command_ordered_extent {
	u32 err;
	u64 transid;
	struct btrfs_ordered_extent *ordered_extent;
	u32 total_csums;
	u32 total_csum_size;
	bool bl_update_isize;
};
struct syno_cache_protection_parameter_command_inline_extent {
	u32 err;
	u64 transid;
	struct inode *inode;
	u64 inline_len;
};
struct syno_cache_protection_parameter_command_space_reserve {
	size_t count[SYNO_CACHE_PROTECTION_SPACE_POOL_MAX];
};
struct syno_cache_protection_parameter_command_create {
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	u64 transid;
	struct inode *dir;
	struct inode *inode;
	struct dentry *dentry;
	u64 symname_len;
	const char *symname;
};
struct syno_cache_protection_parameter_command_inode_operation {
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	u64 transid;
	struct inode *inode;
	u64 flags;
	u64 offset;
	u64 length;
};
struct syno_cache_protection_parameter_command_rename {
	u64 transid;
	struct inode *old_dir;
	struct inode *new_dir;
	struct dentry *old_dentry;
	struct dentry *new_dentry;
};
struct syno_cache_protection_parameter_command_clone {
	u64 transid;
	struct inode *src_inode;
	u64 src_offset;
	u64 len;
	struct inode *dst_inode;
	u64 dst_offset;
};
struct syno_cache_protection_parameter_command_xattr {
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	u64 transid;
	struct inode *inode;
	size_t name_size;
	size_t value_size;
	const char *name;
	const char *value;
	int flags;
};
struct syno_cache_protection_parameter_command_subvol_operation {
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	u64 transid;
	u64 uid;
	struct inode *inode;
	struct btrfs_ioctl_qgroup_create_args *qgroup_ca;
	struct btrfs_ioctl_qgroup_assign_args *qgroup_aa;
	struct btrfs_ioctl_qgroup_limit_args *qgroup_la;
	struct btrfs_ioctl_usrquota_limit_args *usrquota_la;
};

struct syno_cache_protection_command_request {
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
 	struct btrfs_fs_info *fs_info;
	const struct syno_cache_protection_btrfs_command_operations *ops;
	void *req;
	struct syno_cache_protection_parameter_command_space_reserve reserve_parm;
	bool reserved;
};

struct syno_cache_protection_replay_args {
	const char *mount_path;
	size_t mount_path_len;
	u64 root_subvolid;
	bool verbose;
};

int __init btrfs_syno_cache_protection_init(void);
void btrfs_syno_cache_protection_exit(void);
int btrfs_syno_cache_protection_active_enable(struct btrfs_fs_info *fs_info);
int btrfs_syno_cache_protection_active_disable(struct btrfs_fs_info *fs_info);
void btrfs_syno_cache_protection_free_command(void* command);
void* btrfs_syno_cache_protection_alloc_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, struct btrfs_fs_info *fs_info, void *data);
int btrfs_syno_cache_protection_write_and_send_command(void *command, void *data);
int btrfs_syno_cache_protection_exec_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, struct btrfs_fs_info *fs_info, void *data);
void btrfs_init_syno_cache_protection_async_checkpoint_work(struct work_struct *work);
void btrfs_init_syno_cache_protection_async_flush_work(struct work_struct *work);
void btrfs_init_syno_cache_protection_async_data_reclaim_work(struct work_struct *work);
void btrfs_init_syno_cache_protection_auto_disable_work(struct work_struct *work);
int btrfs_syno_cache_protection_passive_replay(struct btrfs_fs_info *fs_info, struct syno_cache_protection_replay_args *replay_args);
int btrfs_syno_cache_protection_passive_replay_prepare(struct btrfs_fs_info *fs_info);
void btrfs_syno_cache_protection_passive_replay_release(struct btrfs_fs_info *fs_info);
int syno_cache_protection_recover(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance, struct syno_cache_protection_replay_args *replay_args);
bool syno_cache_protection_is_enabled(struct btrfs_fs_info *fs_info);
void syno_cache_protection_set_disable_and_error(struct btrfs_fs_info *fs_info, int err);

#endif /* __BTRFS_SYNO_CACHE_PROTECTION_ */
