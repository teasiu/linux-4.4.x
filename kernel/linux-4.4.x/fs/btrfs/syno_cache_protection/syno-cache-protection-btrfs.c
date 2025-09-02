/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/bug.h>
#include <linux/mm.h>
#include <linux/workqueue.h>
#include <linux/spinlock_types.h>
#include <linux/file.h>
#include "../ctree.h"
#include "../disk-io.h"
#include "../transaction.h"
#include "../volumes.h"
#include <linux/syno_cache_protection.h>
#include "syno-cache-protection-btrfs.h"
#include "syno-cache-protection-btrfs-command.h"
#include "syno-cache-protection-btrfs-passive-model.h"

static void __syno_cache_protection_set_disable_and_status(struct btrfs_fs_info *fs_info, bool *enabled, enum btrfs_syno_cache_protection_state_enum *status, int *err)
{
	if (!fs_info)
		return;

	spin_lock(&fs_info->syno_cache_protection_lock);
	if (enabled && fs_info->cache_protection_fs)
		fs_info->cache_protection_fs->enabled = *enabled;
	if (status && fs_info->syno_cache_protection_status != *status) {
		fs_info->syno_cache_protection_status = *status;
		if (err)
			fs_info->syno_cache_protection_error_code = *err;
	}
	spin_unlock(&fs_info->syno_cache_protection_lock);
}

static void syno_cache_protection_set_status(struct btrfs_fs_info *fs_info, enum btrfs_syno_cache_protection_state_enum status)
{
	int err = 0;
	__syno_cache_protection_set_disable_and_status(fs_info, NULL, &status, &err);
}

void syno_cache_protection_set_disable_and_error(struct btrfs_fs_info *fs_info, int err)
{
	bool enabled = false;
	enum btrfs_syno_cache_protection_state_enum status = SYNO_CACHE_PROTECTION_STATE_ERROR;
	__syno_cache_protection_set_disable_and_status(fs_info, &enabled, &status, &err);
	queue_work(system_unbound_wq, &fs_info->syno_cache_protection_auto_disable_work);
}

static void* __btrfs_syno_cache_protection_alloc_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command,
				struct btrfs_fs_info *fs_info, void *data, struct syno_cache_protection_command_request *parent);
static int syno_cache_protection_reserve_exec_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, struct syno_cache_protection_command_request *parent)
{
	int ret;
	void *req = NULL;

	req = __btrfs_syno_cache_protection_alloc_command(command, parent->fs_info, &parent->reserve_parm, parent);
	if (!req) {
		ret = 0;
		goto out;
	}
	if (IS_ERR(req)) {
		ret = PTR_ERR(req);
		req = NULL;
		goto out;
	}

	ret = btrfs_syno_cache_protection_write_and_send_command(req, &parent->reserve_parm);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_remote_reserve(struct syno_cache_protection_command_request *command, void *data)
{
	int ret;
	const struct syno_cache_protection_btrfs_command_operations *ops = NULL;

	if (!command || !command->fs_info || !command->ops) {
		ret = -EINVAL;
		goto out;
	}

	ops = command->ops;

	if (!ops->reserve) {
		ret = 0;
		goto out;
	}

	ret = ops->reserve(&command->reserve_parm, data);
	if (ret < 0)
		goto out;
	else if (ret == 1) {
		ret = 0;
		goto out;
	}

	ret = syno_cache_protection_reserve_exec_command(SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SPACE_RESERVE, command);
	if (ret)
		goto out;
	command->reserved = true;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_remote_reserve_free(struct syno_cache_protection_command_request *command)
{
	int ret;

	if (!command || !command->fs_info || !command->ops) {
		ret = -EINVAL;
		goto out;
	}

	if (!command->reserved) {
		ret = 0;
		goto out;
	}

	ret = syno_cache_protection_reserve_exec_command(SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SPACE_RESERVE_FREE, command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

void btrfs_syno_cache_protection_free_command(void* command)
{
	struct syno_cache_protection_command_request *command_request = (struct syno_cache_protection_command_request*)command;
	struct btrfs_fs_info *fs_info;
	bool need_wake_up = false;

	if (!command_request)
		return;

	if (command_request->req)
		syno_cache_protection_put_request(command_request->req);

	if (command_request->reserved)
		btrfs_syno_cache_protection_command_remote_reserve_free(command_request);

	fs_info = command_request->fs_info;
	kfree(command_request);

	if (fs_info) {
		spin_lock(&fs_info->syno_cache_protection_lock);
		fs_info->syno_cache_protection_nr--;
		if (0 == fs_info->syno_cache_protection_nr)
			need_wake_up = true;
		spin_unlock(&fs_info->syno_cache_protection_lock);
		if (need_wake_up && waitqueue_active(&fs_info->syno_cache_protection_wait))
			wake_up(&fs_info->syno_cache_protection_wait);
	}
}

static void* __btrfs_syno_cache_protection_alloc_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, struct btrfs_fs_info *fs_info, void *data, struct syno_cache_protection_command_request *parent)
{
	int err = 0;
	struct syno_cache_protection_stream_btrfs_command_header command_header;
	size_t channel = 0;
	void *req = NULL;
	size_t len = 0;
	const struct syno_cache_protection_btrfs_command_operations *ops = NULL;
	struct syno_cache_protection_command_request *command_request = NULL;

	if (!fs_info) {
		err = -EINVAL;
		goto out;
	}

	ops = syno_cache_protection_btrfs_get_command_ops(command);

	if (!ops) {
		err = -EINVAL;
		goto out;
	}

	command_request = kzalloc(sizeof(*command_request), GFP_NOFS);
	if (!command_request) {
		err = -ENOMEM;
		goto out;
	}
	command_request->command = command;
	command_request->ops = ops;
	spin_lock(&fs_info->syno_cache_protection_lock);
	if ((!fs_info->cache_protection_fs) ||
		(!ops->skip_check_enabled && !fs_info->cache_protection_fs->enabled)) {
		spin_unlock(&fs_info->syno_cache_protection_lock);
		btrfs_syno_cache_protection_free_command(command_request);
		command_request = NULL;
		goto end;
	}
	fs_info->syno_cache_protection_nr++;
	command_request->fs_info = fs_info;
	spin_unlock(&fs_info->syno_cache_protection_lock);

	if (ops->reserve) {
		err = btrfs_syno_cache_protection_command_remote_reserve(command_request, data);
		if (err)
			goto out;
	}

	channel = ops->channel;
	if (parent && parent->ops &&
		(parent->command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT ||
		 parent->command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INLINE_EXTENT))
		channel = parent->ops->channel;
	if (ops->size)
		len += ops->size(data);

	req = syno_cache_protection_get_request(fs_info->cache_protection_fs, sizeof(command_header) + len, channel);
	if (IS_ERR(req)) {
		err = PTR_ERR(req);
		req = NULL;
		goto out;
	}
	command_request->req = req;

	memset(&command_header, 0, sizeof(command_header));
	command_header.command = cpu_to_le32(command);
	command_header.reserved = cpu_to_le32(command_request->reserved ? 1 : 0);
	err = syno_cache_protection_write_request(req, sizeof(command_header), &command_header);
	if (err)
		goto out;

end:
	return command_request;

out:
	if (fs_info)
		syno_cache_protection_set_disable_and_error(fs_info, err);
	if (command_request)
		btrfs_syno_cache_protection_free_command(command_request);
	return ERR_PTR(err);
}

void* btrfs_syno_cache_protection_alloc_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, struct btrfs_fs_info *fs_info, void *data)
{
	return __btrfs_syno_cache_protection_alloc_command(command, fs_info, data, NULL);
}

int btrfs_syno_cache_protection_write_and_send_command(void *command, void *data)
{
	int ret;
	struct syno_cache_protection_command_request *command_request = (struct syno_cache_protection_command_request*)command;
	const struct syno_cache_protection_btrfs_command_operations *ops = NULL;
	bool wait;

	if (!command_request || !command_request->fs_info || !command_request->ops || !command_request->req) {
		ret = -EINVAL;
		goto out;
	}

	ops = command_request->ops;

	if (ops->send) {
		ret = ops->send(data, command_request->req);
		if (ret)
			goto out;
	}

	wait = ops->wait;
	ret = syno_cache_protection_send_request(command_request->req, wait);
	if (ret)
		goto out;
	command_request->req = NULL;

	ret = 0;
out:
	if (ret && command_request && command_request->fs_info)
		syno_cache_protection_set_disable_and_error(command_request->fs_info, ret);
	btrfs_syno_cache_protection_free_command(command_request);
	return ret;
}

int btrfs_syno_cache_protection_exec_command(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, struct btrfs_fs_info *fs_info, void *data)
{
	int ret;
	void *req = NULL;

	req = btrfs_syno_cache_protection_alloc_command(command, fs_info, data);
	if (!req) {
		ret = 0;
		goto out;
	}
	if (IS_ERR(req)) {
		ret = PTR_ERR(req);
		req = NULL;
		goto out;
	}

	ret = btrfs_syno_cache_protection_write_and_send_command(req, data);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_do_command(void *private, void *req)
{
	int ret;
	struct syno_cache_protection_stream_btrfs_command_header command_header;
	const struct syno_cache_protection_btrfs_command_operations *ops = NULL;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command;
	bool reserved = false;

	ret = syno_cache_protection_read_request(req, sizeof(command_header), &command_header);
	if (ret)
		goto out;

	command = (enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND) le32_to_cpu(command_header.command);
	if (le32_to_cpu(command_header.reserved))
		reserved = true;

	ops = syno_cache_protection_btrfs_get_command_ops(command);

	if (!ops || !ops->receive) {
		ret = -EINVAL;
		goto out;
	}

	ret = ops->receive(private, req, reserved);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_active_reclaim(void *private, bool metadata)
{
	struct btrfs_fs_info *fs_info = (struct btrfs_fs_info *)private;

	if (metadata) {
		queue_work(system_unbound_wq, &fs_info->syno_cache_protection_async_checkpoint_work);
	} else {
		queue_work(system_unbound_wq, &fs_info->syno_cache_protection_async_flush_work);
		queue_work(system_unbound_wq, &fs_info->syno_cache_protection_async_data_reclaim_work);
	}

	return 0;
}

static struct syno_cache_protection_fs_type syno_cache_protection_btrfs_type;

int btrfs_syno_cache_protection_active_enable(struct btrfs_fs_info *fs_info)
{
	int ret;
	struct syno_cache_protection_fs *fs = NULL;
	struct btrfs_trans_handle *trans;

	if (fs_info->cache_protection_fs) {
		ret = 0;
		goto out;
	}

	if (fs_info->syno_cache_protection_recovering) {
		ret = -EBUSY;
		goto out;
	}

	syno_cache_protection_set_status(fs_info, SYNO_CACHE_PROTECTION_STATE_ENABLING);

	ret = syno_cache_protection_send_ctl_ping();
	if (ret)
		goto out;

	ret = syno_cache_protection_send_ctl_space_enable();
	if (ret)
		goto out;

	ret = syno_cache_protection_clear_passive_instance_with_fs(SYNO_CACHE_PROTECTION_ROLE_ACTIVE, SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
	if (ret) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection send clear passive instance command with fsid %pU err %d for enable", fs_info->fs_devices->fsid, ret);
		goto out;
	}

	ret = syno_cache_protection_alloc_passive_instance_with_fs(SYNO_CACHE_PROTECTION_ROLE_ACTIVE, SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
	if (ret) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection send alloc passive instance command with fsid %pU err %d for enable", fs_info->fs_devices->fsid, ret);
		goto out;
	}

	fs = syno_cache_protection_alloc_fs_instance();
	if (!fs) {
		ret = -ENOMEM;
		goto out;
	}

	BUILD_BUG_ON(BTRFS_FSID_SIZE > SYNO_CACHE_PROTECTION_UUID_SIZE_MAX);
	fs->fs_type = &syno_cache_protection_btrfs_type;
	fs->role = SYNO_CACHE_PROTECTION_ROLE_ACTIVE;
	fs->uuid_len = BTRFS_FSID_SIZE;
	memcpy(fs->uuid, fs_info->fs_devices->fsid, fs->uuid_len);
	fs->private = fs_info;
	fs->reclaim = btrfs_syno_cache_protection_active_reclaim;
	fs->do_command = btrfs_syno_cache_protection_do_command;
	fs->enabled = true;

	ret = syno_cache_protection_add(fs);
	if (ret)
		goto out;

	syno_cache_protection_fs_get(fs);
	spin_lock(&fs_info->syno_cache_protection_lock);
	fs_info->cache_protection_fs = fs;
	spin_unlock(&fs_info->syno_cache_protection_lock);

	trans = btrfs_join_transaction(fs_info->tree_root);
	if (!IS_ERR(trans))
		btrfs_commit_transaction(trans, fs_info->tree_root);

	syno_cache_protection_set_status(fs_info, SYNO_CACHE_PROTECTION_STATE_ENABLED);

	if (syno_cache_protection_check_local_reclaim(true)) {
		queue_work(system_unbound_wq, &fs_info->syno_cache_protection_async_checkpoint_work);
	}
	if (syno_cache_protection_check_local_reclaim(false)) {
		queue_work(system_unbound_wq, &fs_info->syno_cache_protection_async_flush_work);
		queue_work(system_unbound_wq, &fs_info->syno_cache_protection_async_data_reclaim_work);
	}

	ret = 0;
out:
	syno_cache_protection_fs_put(fs);
	if (ret) {
		syno_cache_protection_clear_passive_instance_with_fs(SYNO_CACHE_PROTECTION_ROLE_ACTIVE, SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
		syno_cache_protection_set_status(fs_info, SYNO_CACHE_PROTECTION_STATE_NONE);
	}
	return ret;
}

int __btrfs_syno_cache_protection_active_disable(struct btrfs_fs_info *fs_info, bool update_status)
{
	int err;
	struct syno_cache_protection_fs *fs = NULL;

	if (!fs_info)
		goto out;
	spin_lock(&fs_info->syno_cache_protection_lock);
	if (!fs_info->cache_protection_fs) {
		spin_unlock(&fs_info->syno_cache_protection_lock);
		goto out;
	}
	fs_info->cache_protection_fs->enabled = false;
	if (update_status)
		fs_info->syno_cache_protection_status = SYNO_CACHE_PROTECTION_STATE_DISABLING;
	fs = fs_info->cache_protection_fs;
	spin_unlock(&fs_info->syno_cache_protection_lock);

again:
	spin_lock(&fs_info->syno_cache_protection_lock);
	if (fs_info->syno_cache_protection_nr > 0) {
		spin_unlock(&fs_info->syno_cache_protection_lock);
		wait_event(fs_info->syno_cache_protection_wait, 0 == fs_info->syno_cache_protection_nr);
		goto again;
	}
	fs_info->cache_protection_fs = NULL;
	spin_unlock(&fs_info->syno_cache_protection_lock);

	err = syno_cache_protection_clear_passive_instance_with_fs(SYNO_CACHE_PROTECTION_ROLE_ACTIVE, fs->fs_type->id, fs->uuid_len, fs->uuid);
	if (err)
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection send clear passive instance command with fsid %pU err %d for disable", fs_info->fs_devices->fsid, err);

	syno_cache_protection_remove(fs);
out:
	syno_cache_protection_fs_put(fs);
	if (update_status)
		syno_cache_protection_set_status(fs_info, SYNO_CACHE_PROTECTION_STATE_NONE);
	return 0;
}

int btrfs_syno_cache_protection_active_disable(struct btrfs_fs_info *fs_info)
{
	return __btrfs_syno_cache_protection_active_disable(fs_info, true);
}

static void btrfs_syno_cache_protection_free_instance(struct syno_cache_protection_fs *fs)
{
	if (!fs)
		return;
	if (fs->role == SYNO_CACHE_PROTECTION_ROLE_PASSIVE)
		syno_cache_protection_passive_btrfs_instance_free((struct syno_cache_protection_passive_btrfs_instance *)fs->private);
}

static struct syno_cache_protection_fs *btrfs_syno_cache_protection_alloc_passive_instance(size_t uuid_len, u8 *uuid)
{
	struct syno_cache_protection_fs *fs = NULL;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = NULL;
	int err;

	fs = syno_cache_protection_alloc_fs_instance();
	if (!fs) {
		err = -ENOMEM;
		goto out;
	}

	fs->fs_type = &syno_cache_protection_btrfs_type;
	fs->role = SYNO_CACHE_PROTECTION_ROLE_PASSIVE;
	fs->uuid_len = uuid_len;
	memcpy(fs->uuid, uuid, uuid_len);
	fs->private = NULL;
	fs->do_command = btrfs_syno_cache_protection_do_command;
	fs->enabled = true;

	passive_instance = syno_cache_protection_passive_btrfs_instance_alloc(fs);
	if (IS_ERR(passive_instance)) {
		err = PTR_ERR(passive_instance);
		passive_instance = NULL;
		goto out;
	}

	fs->private = passive_instance;

	return fs;

out:
	syno_cache_protection_fs_put(fs);
	return ERR_PTR(err);
}

static struct syno_cache_protection_fs_type syno_cache_protection_btrfs_type = {
	.owner = THIS_MODULE,
	.id = SYNO_CACHE_PROTECTION_FS_BTRFS,
	.list = LIST_HEAD_INIT(syno_cache_protection_btrfs_type.list),
	.alloc_passive_instance = btrfs_syno_cache_protection_alloc_passive_instance,
	.free_instance = btrfs_syno_cache_protection_free_instance,
};

int __init btrfs_syno_cache_protection_init(void)
{
	return syno_cache_protection_register_fs(&syno_cache_protection_btrfs_type);
}

void btrfs_syno_cache_protection_exit(void)
{
	syno_cache_protection_unregister_fs(&syno_cache_protection_btrfs_type);
}

static void __btrfs_syno_cache_protection_async_checkpoint_work(struct work_struct *work)
{
	struct btrfs_fs_info *fs_info;
	struct btrfs_trans_handle *trans;
	struct btrfs_root *root;
	signed long timeout;
	unsigned long expire;

	fs_info = container_of(work, struct btrfs_fs_info, syno_cache_protection_async_checkpoint_work);
	root = fs_info->tree_root;

again:
	if (!syno_cache_protection_is_enabled(fs_info))
		goto out;

	expire = jiffies + msecs_to_jiffies(1000);

	trans = btrfs_attach_transaction(root);
	if (IS_ERR(trans))
		goto check;

	btrfs_commit_transaction(trans, root);
check:
	if (syno_cache_protection_check_local_reclaim(true)) {
		timeout = expire - jiffies;
		if (timeout > 0)
			schedule_timeout_interruptible(1 + timeout);
		goto again;
	}
out:
	return;
}
void btrfs_init_syno_cache_protection_async_checkpoint_work(struct work_struct *work)
{
	INIT_WORK(work, __btrfs_syno_cache_protection_async_checkpoint_work);
}

static int syno_cache_protection_start_delalloc_inodes(struct btrfs_root *root)
{
	struct btrfs_inode *binode;
	struct inode *inode;
	struct list_head splice;
	struct address_space *mapping;
	struct extent_io_tree *tree;
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_NONE,
		.nr_to_write = LONG_MAX,
		.range_start = 0,
		.range_end = LLONG_MAX,
	};

	INIT_LIST_HEAD(&splice);

	spin_lock(&root->delalloc_lock);
	list_splice_init(&root->syno_delalloc_inodes, &splice);
	while (!list_empty(&splice)) {
		list_rotate_left(&splice);
		binode = list_entry(splice.next, struct btrfs_inode,
				    syno_delalloc_inodes);

		list_move_tail(&binode->syno_delalloc_inodes,
			       &root->syno_delalloc_inodes);
		inode = igrab(&binode->vfs_inode);
		if (!inode) {
			cond_resched_lock(&root->delalloc_lock);
			continue;
		}
		spin_unlock(&root->delalloc_lock);

		mapping = inode->i_mapping;
		tree = &BTRFS_I(inode)->io_tree;
		syno_cache_protection_extent_writepages(tree, mapping, btrfs_get_extent, &wbc);
		iput(inode);

		cond_resched();
		spin_lock(&root->delalloc_lock);
	}
	spin_unlock(&root->delalloc_lock);
	return 0;
}

static int syno_cache_protection_start_delalloc_roots(struct btrfs_fs_info *fs_info)
{
	struct btrfs_root *root;
	struct list_head splice;
	int ret;

	if (test_bit(BTRFS_FS_STATE_ERROR, &fs_info->fs_state)) {
		ret = -EROFS;
		goto out;
	}

	INIT_LIST_HEAD(&splice);

	spin_lock(&fs_info->delalloc_root_lock);
	list_splice_init(&fs_info->syno_delalloc_roots, &splice);
	while (!list_empty(&splice)) {
		root = list_first_entry(&splice, struct btrfs_root,
					syno_delalloc_root);
		root = btrfs_grab_fs_root(root);
		BUG_ON(!root);
		list_move_tail(&root->syno_delalloc_root,
			       &fs_info->syno_delalloc_roots);
		spin_unlock(&fs_info->delalloc_root_lock);

		syno_cache_protection_start_delalloc_inodes(root);
		btrfs_put_fs_root(root);
		spin_lock(&fs_info->delalloc_root_lock);
	}
	spin_unlock(&fs_info->delalloc_root_lock);

	ret = 0;
out:
	return ret;
}

static void __btrfs_syno_cache_protection_async_flush_work(struct work_struct *work)
{
	struct btrfs_fs_info *fs_info;
	signed long timeout;
	unsigned long expire;

	fs_info = container_of(work, struct btrfs_fs_info, syno_cache_protection_async_flush_work);

again:
	if (!syno_cache_protection_is_enabled(fs_info))
		goto out;

	expire = jiffies + msecs_to_jiffies(100);

	/* start flush all dirty data, not wait page lock */
	syno_cache_protection_start_delalloc_roots(fs_info);

	if (syno_cache_protection_check_local_reclaim(false)) {
		timeout = expire - jiffies;
		if (timeout > 0)
			schedule_timeout_interruptible(1 + timeout);
		goto again;
	}
out:
	return;
}
void btrfs_init_syno_cache_protection_async_flush_work(struct work_struct *work)
{
	INIT_WORK(work, __btrfs_syno_cache_protection_async_flush_work);
}

static void __btrfs_syno_cache_protection_async_data_reclaim_work(struct work_struct *work)
{
	int err;
	struct btrfs_fs_info *fs_info;
	signed long timeout;
	unsigned long expire;

	fs_info = container_of(work, struct btrfs_fs_info, syno_cache_protection_async_data_reclaim_work);

again:
	if (!syno_cache_protection_is_enabled(fs_info))
		goto out;

	expire = jiffies + msecs_to_jiffies(1000);

	err = btrfs_syno_cache_protection_exec_command(SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DATA_RECLAIM, fs_info, NULL);
	if (err) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection send data reclaim command with fsid %pU err %d", fs_info->fs_devices->fsid, err);
		syno_cache_protection_set_disable_and_error(fs_info, err);
		goto out;
	}

	/* start flush all dirty data */
	btrfs_start_delalloc_roots(fs_info, 0, -1);
	/* wait all data & metadata finished */
	btrfs_wait_ordered_roots(fs_info, -1, 0, (u64)-1);

	err = btrfs_syno_cache_protection_exec_command(SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DATA_RECLAIM, fs_info, NULL);
	if (err) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection send data reclaim command with fsid %pU err %d", fs_info->fs_devices->fsid, err);
		syno_cache_protection_set_disable_and_error(fs_info, err);
		goto out;
	}

	if (syno_cache_protection_check_local_reclaim(false)) {
		timeout = expire - jiffies;
		if (timeout > 0)
			schedule_timeout_interruptible(1 + timeout);
		goto again;
	}
out:
	return;
}
void btrfs_init_syno_cache_protection_async_data_reclaim_work(struct work_struct *work)
{
	INIT_WORK(work, __btrfs_syno_cache_protection_async_data_reclaim_work);
}

static void __btrfs_syno_cache_protection_auto_disable_work(struct work_struct *work)
{
	struct btrfs_fs_info *fs_info;

	fs_info = container_of(work, struct btrfs_fs_info, syno_cache_protection_auto_disable_work);

	mutex_lock(&fs_info->syno_cache_protection_ioctl_lock);
	__btrfs_syno_cache_protection_active_disable(fs_info, false);
	mutex_unlock(&fs_info->syno_cache_protection_ioctl_lock);
}
void btrfs_init_syno_cache_protection_auto_disable_work(struct work_struct *work)
{
	INIT_WORK(work, __btrfs_syno_cache_protection_auto_disable_work);
}

int btrfs_syno_cache_protection_passive_replay(struct btrfs_fs_info *fs_info, struct syno_cache_protection_replay_args *replay_args)
{
	int ret;
	struct syno_cache_protection_fs *cache_protection_fs = NULL;

	if (!fs_info) {
		ret = -EINVAL;
		goto out;
	}

	cache_protection_fs = syno_cache_protection_get_passive_instance(SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
	if (!cache_protection_fs)
		goto success;
	spin_lock(&cache_protection_fs->lock);
	cache_protection_fs->enabled = false;
	spin_unlock(&cache_protection_fs->lock);

	ret = syno_cache_protection_recover(fs_info, (struct syno_cache_protection_passive_btrfs_instance *)cache_protection_fs->private, replay_args);
	if (ret) {
		btrfs_warn(fs_info, "Failed to syno cache protection recover with fsid %pU err %d", fs_info->fs_devices->fsid, ret);
		goto out;
	}

success:
	ret = 0;
out:
	syno_cache_protection_fs_put(cache_protection_fs);
	return ret;
}

bool syno_cache_protection_is_enabled(struct btrfs_fs_info *fs_info)
{
	bool ret;

	if (!fs_info) {
		ret = false;
		goto out;
	}

	spin_lock(&fs_info->syno_cache_protection_lock);
	if (!fs_info->cache_protection_fs)
		ret = false;
	else
		ret = true;
	spin_unlock(&fs_info->syno_cache_protection_lock);
out:
	return ret;
}

