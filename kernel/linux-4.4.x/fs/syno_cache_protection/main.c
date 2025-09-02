/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/syno_cache_protection.h>
#include "internal.h"
#include "util.h"

// Driver defines:
#define DRIVER_NAME			"syno_cache_protection"
#define DRIVER_DESCRIPTION	"SYNO Cache Protection"
#define DRIVER_LICENSE		"GPL"
#define DRIVER_VERSION		"1.0"
#define DRIVER_AUTHOR		"Robbie Ko <robbieko@synology.com>"

MODULE_LICENSE(DRIVER_LICENSE);
MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);

enum SYNO_CACHE_PROTECTION_TYPE {
	SYNO_CACHE_PROTECTION_TYPE_CTL = 0,
	SYNO_CACHE_PROTECTION_TYPE_FS
};

enum SYNO_CACHE_PROTECTION_CTL_CMD {
	SYNO_CACHE_PROTECTION_CTL_PING = 0,
	SYNO_CACHE_PROTECTION_CTL_SPACE_ENABLE,
	SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_METADATA,
	SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_METADATA_END,
	SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_DATA,
	SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_DATA_END,
	SYNO_CACHE_PROTECTION_CTL_CLEAR_PASSIVE_INSTANCE,
	SYNO_CACHE_PROTECTION_CTL_ALLOC_PASSIVE_INSTANCE,
};

struct syno_cache_protection_instance *instance = NULL;
struct syno_cache_protection_instance **syno_cache_protection_instance_ptr = &instance;
EXPORT_SYMBOL(syno_cache_protection_instance_ptr);

struct syno_cache_protection_stream_header {
	__le32 type;
	union {
		/* for ctl */
		struct {
			__le32 cmd;
			/* for passive instance clear */
			__le32 fs_type;
			__le32 uuid_len;
			u8 uuid[SYNO_CACHE_PROTECTION_UUID_SIZE_MAX];
		} ctl_parm;
		/* for fs */
		struct {
			__le32 fs_type;
			__le32 role;
			__le32 uuid_len;
			u8 uuid[SYNO_CACHE_PROTECTION_UUID_SIZE_MAX];
		} fs_parm;
	};
} __attribute__ ((__packed__));

static noinline struct syno_cache_protection_fs *_find_fs(struct list_head *head, size_t fs_type, size_t uuid_len, const u8 *uuid)
{
	struct syno_cache_protection_fs *cache_protection_fs;

	list_for_each_entry(cache_protection_fs, head, list) {
		if (fs_type == cache_protection_fs->fs_type->id &&
			uuid_len == cache_protection_fs->uuid_len &&
			memcmp(uuid, cache_protection_fs->uuid, uuid_len) == 0)
			return cache_protection_fs;
	}
	return NULL;
}

static struct syno_cache_protection_fs *_find_active_fs(size_t fs_type, size_t uuid_len, const u8 *uuid)
{
	return _find_fs(&instance->active_uuids, fs_type, uuid_len, uuid);
}
static struct syno_cache_protection_fs *find_active_fs(struct syno_cache_protection_fs *fs)
{
	return _find_fs(&instance->active_uuids, fs->fs_type->id, fs->uuid_len, fs->uuid);
}
static struct syno_cache_protection_fs *_find_passive_fs(size_t fs_type, size_t uuid_len, const u8 *uuid)
{
	return _find_fs(&instance->passive_uuids, fs_type, uuid_len, uuid);
}
static struct syno_cache_protection_fs *find_passive_fs(struct syno_cache_protection_fs *fs)
{
	return _find_fs(&instance->passive_uuids, fs->fs_type->id, fs->uuid_len, fs->uuid);
}

static noinline struct syno_cache_protection_fs_type *_find_fs_type(size_t fs_type, bool check)
{
	struct syno_cache_protection_fs_type *cache_protection_fs_type;

	list_for_each_entry(cache_protection_fs_type, &instance->fs_types, list) {
		if (fs_type == cache_protection_fs_type->id && (!check || cache_protection_fs_type->enabled))
			return cache_protection_fs_type;
	}
	return NULL;
}

static noinline struct syno_cache_protection_fs_type *find_fs_type(size_t fs_type)
{
	struct syno_cache_protection_fs_type *cache_protection_fs_type;

	spin_lock(&instance->fs_type_lock);
	cache_protection_fs_type = _find_fs_type(fs_type, true);
	spin_unlock(&instance->fs_type_lock);

	return cache_protection_fs_type;
}

int syno_cache_protection_register_fs(struct syno_cache_protection_fs_type *fs_type)
{
	int ret;
	struct syno_cache_protection_fs_type *cache_protection_fs_type;

	spin_lock(&instance->fs_type_lock);
	if (!list_empty(&fs_type->list)) {
		ret = -EBUSY;
		goto out;
	}
	cache_protection_fs_type = _find_fs_type(fs_type->id, false);
	if (cache_protection_fs_type) {
		ret = -EEXIST;
		goto out;
	}
	fs_type->enabled = true;
	list_add_tail(&fs_type->list, &instance->fs_types);

	ret = 0;
out:
	spin_unlock(&instance->fs_type_lock);
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_register_fs);

static void syno_cache_protection_unbind_passive_instance(size_t type);
void syno_cache_protection_unregister_fs(struct syno_cache_protection_fs_type *fs_type)
{
	fs_type->enabled = false;
	syno_cache_protection_unbind_passive_instance(fs_type->id);
	spin_lock(&instance->fs_type_lock);
	list_del_init(&fs_type->list);
	spin_unlock(&instance->fs_type_lock);
}
EXPORT_SYMBOL(syno_cache_protection_unregister_fs);

static void syno_cache_protection_clear_passive_instance(size_t fs_type, size_t uuid_len, u8 *uuid)
{
	struct syno_cache_protection_fs *fs, *tmp;
	struct list_head free_list;

	if (!instance)
		return;

	INIT_LIST_HEAD(&free_list);

	spin_lock(&instance->passive_uuid_lock);
	list_for_each_entry_safe(fs, tmp, &instance->passive_uuids, list) {
		if ((fs_type == SYNO_CACHE_PROTECTION_FS_ALL) ||
			(fs_type == fs->fs_type->id &&
			uuid_len == fs->uuid_len &&
			0 == memcmp(uuid, fs->uuid, uuid_len))) {
			spin_lock(&fs->lock);
			fs->enabled = false;
			spin_unlock(&fs->lock);
			list_move_tail(&fs->list, &free_list);
		}
	}
	spin_unlock(&instance->passive_uuid_lock);

	list_for_each_entry_safe(fs, tmp, &free_list, list) {
again:
		spin_lock(&fs->lock);
		if (fs->count > 0) {
			spin_unlock(&fs->lock);
			wait_event(fs->wait, 0 == fs->count);
			goto again;
		}
		spin_unlock(&fs->lock);
		list_del_init(&fs->list);
		syno_cache_protection_fs_put(fs);
	}
}


static void syno_cache_protection_fs_inc(struct syno_cache_protection_fs *fs)
{
	spin_lock(&fs->lock);
	fs->count++;
	spin_unlock(&fs->lock);
}

static void syno_cache_protection_fs_dec(struct syno_cache_protection_fs *fs)
{
	bool need_wake_up = false;
	spin_lock(&fs->lock);
	fs->count--;
	if (0 == fs->count)
		need_wake_up = true;
	spin_unlock(&fs->lock);

	if (need_wake_up && waitqueue_active(&fs->wait))
		wake_up(&fs->wait);
}

void syno_cache_protection_fs_get(struct syno_cache_protection_fs *fs)
{
	atomic_inc(&fs->refs);
}
EXPORT_SYMBOL(syno_cache_protection_fs_get);

void syno_cache_protection_fs_put(struct syno_cache_protection_fs *fs)
{
	if (!fs)
		return;

	if (atomic_dec_and_test(&fs->refs)) {
		WARN_ON_ONCE(!list_empty(&fs->list));
		fs->fs_type->free_instance(fs);
		if (fs->module_ref)
			module_put(fs->fs_type->owner);
		kfree(fs);
	}
}
EXPORT_SYMBOL(syno_cache_protection_fs_put);

static void syno_cache_protection_init_fs(struct syno_cache_protection_fs *fs)
{
	memset(fs, 0, sizeof(*fs));
	INIT_LIST_HEAD(&fs->list);
	INIT_LIST_HEAD(&fs->unbind_list);
	spin_lock_init(&fs->lock);
	init_waitqueue_head(&fs->wait);
	atomic_set(&fs->refs, 0);
}

struct syno_cache_protection_fs *syno_cache_protection_alloc_fs_instance(void)
{
	struct syno_cache_protection_fs *fs = NULL;

	fs = kzalloc(sizeof(*fs), GFP_NOFS);
	if (!fs)
		goto out;
	syno_cache_protection_init_fs(fs);
	syno_cache_protection_fs_get(fs);
out:
	return fs;
}
EXPORT_SYMBOL(syno_cache_protection_alloc_fs_instance);

int syno_cache_protection_add(struct syno_cache_protection_fs *fs)
{
	int ret;
	struct syno_cache_protection_fs *cache_protection_fs;

	spin_lock(&instance->active_uuid_lock);
	if (!list_empty(&fs->list)) {
		ret = -EBUSY;
		goto out;
	}
	cache_protection_fs = find_active_fs(fs);
	if (cache_protection_fs) {
		ret = -EEXIST;
		goto out;
	}
	list_add_tail(&fs->list, &instance->active_uuids);
	syno_cache_protection_fs_get(fs);

	ret = 0;
out:
	spin_unlock(&instance->active_uuid_lock);
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_add);

void syno_cache_protection_remove(struct syno_cache_protection_fs *fs)
{
	if (list_empty(&fs->list))
		return;

	spin_lock(&instance->active_uuid_lock);
	list_del_init(&fs->list);
	spin_unlock(&instance->active_uuid_lock);
	syno_cache_protection_fs_put(fs);
}
EXPORT_SYMBOL(syno_cache_protection_remove);

static void syno_cache_protection_unbind_passive_instance(size_t type)
{
	struct syno_cache_protection_fs *fs, *tmp;
	struct list_head unbind_list;

	if (!instance)
		return;

	INIT_LIST_HEAD(&unbind_list);

	spin_lock(&instance->passive_uuid_lock);
	list_for_each_entry(fs, &instance->passive_uuids, list) {
		if (type == SYNO_CACHE_PROTECTION_FS_ALL || type == fs->fs_type->id) {
			syno_cache_protection_fs_get(fs);
			spin_lock(&fs->lock);
			fs->enabled = false;
			spin_unlock(&fs->lock);
			list_move_tail(&fs->unbind_list, &unbind_list);
		}
	}
	spin_unlock(&instance->passive_uuid_lock);

	list_for_each_entry_safe(fs, tmp, &unbind_list, unbind_list) {
again:
		spin_lock(&fs->lock);
		if (fs->count > 0) {
			spin_unlock(&fs->lock);
			wait_event(fs->wait, 0 == fs->count);
			goto again;
		}
		spin_unlock(&fs->lock);
		list_del_init(&fs->unbind_list);
		fs->reclaim = NULL;
		fs->do_command = NULL;
		syno_cache_protection_fs_put(fs);
	}
}

void* syno_cache_protection_get_request(struct syno_cache_protection_fs *fs, size_t len, size_t channel)
{
	int err;
	void *req = NULL;
	struct syno_cache_protection_stream_header header;

	req = instance->c_op->get_req(sizeof(struct syno_cache_protection_stream_header) + len, channel);
	if (IS_ERR(req)) {
		err = PTR_ERR(req);
		req = NULL;
		goto out;
	}

	memset(&header, 0, sizeof(header));
	header.type = cpu_to_le32(SYNO_CACHE_PROTECTION_TYPE_FS);
	header.fs_parm.fs_type = cpu_to_le32(fs->fs_type->id);
	header.fs_parm.role = cpu_to_le32(fs->role);
	header.fs_parm.uuid_len = cpu_to_le32(fs->uuid_len);
	memcpy(header.fs_parm.uuid, fs->uuid, fs->uuid_len);

	err = instance->c_op->write_req(req, sizeof(header), &header);
	if (err)
		goto out;

	return req;

out:
	if (req)
		instance->c_op->put_req(req);
	return ERR_PTR(err);
}
EXPORT_SYMBOL(syno_cache_protection_get_request);

int syno_cache_protection_write_request(void *req, size_t len, const void *data)
{
	return instance->c_op->write_req(req, len, data);
}
EXPORT_SYMBOL(syno_cache_protection_write_request);

int syno_cache_protection_read_request(void *req, size_t len, void *data)
{
	return instance->c_op->read_req(req, len, data);
}
EXPORT_SYMBOL(syno_cache_protection_read_request);

int syno_cache_protection_send_request(void *req, bool wait)
{
	return instance->c_op->send_req(req, wait);
}
EXPORT_SYMBOL(syno_cache_protection_send_request);

void syno_cache_protection_put_request(void *req)
{
	instance->c_op->put_req(req);
}
EXPORT_SYMBOL(syno_cache_protection_put_request);

static int get_fs_instance_with_activate(struct syno_cache_protection_stream_header *header, struct syno_cache_protection_fs **fs)
{
	int ret;
	struct syno_cache_protection_fs *cache_protection_fs;

	spin_lock(&instance->active_uuid_lock);
	cache_protection_fs = _find_active_fs(le32_to_cpu(header->fs_parm.fs_type),
											le32_to_cpu(header->fs_parm.uuid_len),
											header->fs_parm.uuid);
	if (!cache_protection_fs) {
		ret = -ENOENT;
		goto out;
	}
	spin_lock(&cache_protection_fs->lock);
	if (cache_protection_fs->enabled) {
		*fs = cache_protection_fs;
		syno_cache_protection_fs_get(*fs);
	}
	spin_unlock(&cache_protection_fs->lock);
	ret = 0;
out:
	spin_unlock(&instance->active_uuid_lock);
	return ret;
}

static int get_fs_instance_with_passive(struct syno_cache_protection_stream_header *header, struct syno_cache_protection_fs **fs)
{
	int ret;
	struct syno_cache_protection_fs *cache_protection_fs;

	spin_lock(&instance->passive_uuid_lock);
	cache_protection_fs = _find_passive_fs(le32_to_cpu(header->fs_parm.fs_type),
											le32_to_cpu(header->fs_parm.uuid_len),
											header->fs_parm.uuid);
	if (!cache_protection_fs) {
		ret = -ENOENT;
		goto out;
	}
	spin_lock(&cache_protection_fs->lock);
	if (cache_protection_fs->enabled) {
		*fs = cache_protection_fs;
		syno_cache_protection_fs_get(*fs);
	}
	spin_unlock(&cache_protection_fs->lock);
	ret = 0;
out:
	spin_unlock(&instance->passive_uuid_lock);
	return ret;
}

static int get_fs_instance(struct syno_cache_protection_stream_header *header, struct syno_cache_protection_fs **fs)
{
	enum SYNO_CACHE_PROTECTION_ROLE role;

	role = (enum SYNO_CACHE_PROTECTION_ROLE) le32_to_cpu(header->fs_parm.role);
	if (role == SYNO_CACHE_PROTECTION_ROLE_ACTIVE) {
		return get_fs_instance_with_passive(header, fs);
	} else {
		return get_fs_instance_with_activate(header, fs);
	}
}

static int syno_cache_protection_do_fs_request(void *req, struct syno_cache_protection_stream_header *header)
{
	int ret;
	struct syno_cache_protection_fs *fs = NULL;

	if (!req || !header) {
		ret = -EINVAL;
		goto out;
	}

	ret = get_fs_instance(header, &fs);
	if (ret)
		goto out;

	if (!fs || !fs->enabled || !fs->do_command) {
		ret = -ECONNRESET;
		goto out;
	}

	syno_cache_protection_fs_inc(fs);
	ret = fs->do_command(fs->private, req);
	syno_cache_protection_fs_dec(fs);

	if (ret)
		goto out;
	ret = 0;
out:
	syno_cache_protection_fs_put(fs);
	return ret;
}

void* syno_cache_protection_space_alloc(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, gfp_t gfp_mask, bool reserved)
{
	return instance->s_op->alloc(pool_type, gfp_mask, reserved);
}
EXPORT_SYMBOL(syno_cache_protection_space_alloc);

void syno_cache_protection_space_free(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, void *data)
{
	instance->s_op->free(pool_type, data);
}
EXPORT_SYMBOL(syno_cache_protection_space_free);

int syno_cache_protection_space_reserve(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count, gfp_t gfp_mask)
{
	return instance->s_op->reserve(pool_type, count, gfp_mask);
}
EXPORT_SYMBOL(syno_cache_protection_space_reserve);

void syno_cache_protection_space_reserve_free(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count)
{
	return instance->s_op->reserve_free(pool_type, count);
}
EXPORT_SYMBOL(syno_cache_protection_space_reserve_free);

struct syno_cache_protection_fs* syno_cache_protection_get_passive_instance(size_t fs_type, size_t uuid_len, const u8 *uuid)
{
	struct syno_cache_protection_fs *cache_protection_fs;

	spin_lock(&instance->passive_uuid_lock);
	cache_protection_fs = _find_passive_fs(fs_type, uuid_len, uuid);
	if (cache_protection_fs)
		syno_cache_protection_fs_get(cache_protection_fs);
	spin_unlock(&instance->passive_uuid_lock);

	return cache_protection_fs;
}
EXPORT_SYMBOL(syno_cache_protection_get_passive_instance);

static int _syno_cache_protection_send_ctl_request(enum SYNO_CACHE_PROTECTION_CTL_CMD cmd, bool wait, size_t channel, struct syno_cache_protection_stream_header *external_header)
{
	int ret;
	void *req = NULL;
	struct syno_cache_protection_stream_header internal_header;
	struct syno_cache_protection_stream_header *header;

	if (!instance) {
		ret = -EINVAL;
		goto out;
	}

	req = instance->c_op->get_req(sizeof(struct syno_cache_protection_stream_header), channel);
	if (IS_ERR(req)) {
		ret = PTR_ERR(req);
		req = NULL;
		goto out;
	}

	if (external_header) {
		header = external_header;
	} else {
		header = &internal_header;
		memset(header, 0, sizeof(*header));
	}

	header->type = cpu_to_le32(SYNO_CACHE_PROTECTION_TYPE_CTL);
	header->ctl_parm.cmd = cpu_to_le32(cmd);

	ret = instance->c_op->write_req(req, sizeof(*header), header);
	if (ret)
		goto out;

	ret = instance->c_op->send_req(req, wait);
	if (ret)
		goto out;
	req = NULL;

	ret = 0;
out:
	if (req)
		instance->c_op->put_req(req);
	return ret;
}

static int syno_cache_protection_send_ctl_request(enum SYNO_CACHE_PROTECTION_CTL_CMD cmd, bool wait, size_t channel)
{
	return _syno_cache_protection_send_ctl_request(cmd, wait, channel, NULL);
}

int syno_cache_protection_send_ctl_ping(void)
{
	return syno_cache_protection_send_ctl_request(SYNO_CACHE_PROTECTION_CTL_PING, true, SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE);
}
EXPORT_SYMBOL(syno_cache_protection_send_ctl_ping);

int syno_cache_protection_send_ctl_space_enable(void)
{
	return syno_cache_protection_send_ctl_request(SYNO_CACHE_PROTECTION_CTL_SPACE_ENABLE, true, SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE);
}
EXPORT_SYMBOL(syno_cache_protection_send_ctl_space_enable);

void syno_cache_protection_all_reclaim(bool metadata)
{
	struct syno_cache_protection_fs *fs;

	spin_lock(&instance->active_uuid_lock);
	list_for_each_entry(fs, &instance->active_uuids, list) {
		if (fs->reclaim)
			fs->reclaim(fs->private, metadata);
	}
	spin_unlock(&instance->active_uuid_lock);
}
EXPORT_SYMBOL(syno_cache_protection_all_reclaim);

void syno_cache_protection_send_reclaim(bool metadata)
{
	int err;
	bool is_need_send = false;
	enum SYNO_CACHE_PROTECTION_CTL_CMD cmd;
	unsigned long flags;

	if (!instance)
		return;

	spin_lock_irqsave(&instance->reclaim_lock, flags);
	if (metadata && !instance->remote_metadata_reclaim) {
		instance->remote_metadata_reclaim = true;
		is_need_send = true;
	} else if (!metadata && !instance->remote_data_reclaim) {
		instance->remote_data_reclaim = true;
		is_need_send = true;
	}
	spin_unlock_irqrestore(&instance->reclaim_lock, flags);

	if (!is_need_send)
		return;

	if (metadata)
		cmd = SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_METADATA;
	else
		cmd = SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_DATA;

	err = syno_cache_protection_send_ctl_request(cmd, true, SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE);
	if (err) {
		syno_cache_protection_warn("Failed to send reclaim command [%d] err %d", (int)cmd, err);
		spin_lock_irqsave(&instance->reclaim_lock, flags);
		if (metadata)
			instance->remote_metadata_reclaim = false;
		else /* data */
			instance->remote_data_reclaim = false;
		spin_unlock_irqrestore(&instance->reclaim_lock, flags);
	}
}
EXPORT_SYMBOL(syno_cache_protection_send_reclaim);

void syno_cache_protection_send_reclaim_end(bool metadata)
{
	int err;
	bool is_need_send = false;
	enum SYNO_CACHE_PROTECTION_CTL_CMD cmd;
	unsigned long flags;

	if (!instance)
		return;

	spin_lock_irqsave(&instance->reclaim_lock, flags);
	if (metadata && instance->remote_metadata_reclaim) {
		instance->remote_metadata_reclaim = false;
		is_need_send = true;
	} else if (!metadata && instance->remote_data_reclaim) {
		instance->remote_data_reclaim = false;
		is_need_send = true;
	}
	spin_unlock_irqrestore(&instance->reclaim_lock, flags);

	if (!is_need_send)
		return;

	if (metadata)
		cmd = SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_METADATA_END;
	else
		cmd = SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_DATA_END;

	err = syno_cache_protection_send_ctl_request(cmd, true, SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE);
	if (err) {
		syno_cache_protection_warn("Failed to send reclaim end command [%d] err %d", (int)cmd, err);
		spin_lock_irqsave(&instance->reclaim_lock, flags);
		if (metadata)
			instance->remote_metadata_reclaim = true;
		else /* data */
			instance->remote_data_reclaim = true;
		spin_unlock_irqrestore(&instance->reclaim_lock, flags);
	}
}
EXPORT_SYMBOL(syno_cache_protection_send_reclaim_end);

int syno_cache_protection_clear_all(void)
{
	syno_cache_protection_clear_passive_instance(SYNO_CACHE_PROTECTION_FS_ALL, 0, NULL);
	return 0;
}
EXPORT_SYMBOL(syno_cache_protection_clear_all);

int syno_cache_protection_clear_passive_instance_with_fs(enum SYNO_CACHE_PROTECTION_ROLE role, size_t fs_type, size_t uuid_len, u8 *uuid)
{
	int ret = 0;
	struct syno_cache_protection_stream_header header;

	if (role == SYNO_CACHE_PROTECTION_ROLE_ACTIVE) {
		memset(&header, 0, sizeof(header));
		header.ctl_parm.fs_type = cpu_to_le32(fs_type);
		header.ctl_parm.uuid_len = cpu_to_le32(uuid_len);
		memcpy(header.ctl_parm.uuid, uuid, uuid_len);
		ret = _syno_cache_protection_send_ctl_request(SYNO_CACHE_PROTECTION_CTL_CLEAR_PASSIVE_INSTANCE, true, SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE, &header);
	} else {
		syno_cache_protection_clear_passive_instance(fs_type, uuid_len, uuid);
	}
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_clear_passive_instance_with_fs);

static void syno_cache_protection_do_ctl_clear_passive_instance(void *req, struct syno_cache_protection_stream_header *header)
{
	size_t fs_type, uuid_len;
	u8 *uuid;

	fs_type = le32_to_cpu(header->ctl_parm.fs_type);
	uuid_len = le32_to_cpu(header->ctl_parm.uuid_len);
	uuid = header->ctl_parm.uuid;

	syno_cache_protection_clear_passive_instance(fs_type, uuid_len, uuid);
}

static int syno_cache_protection_alloc_passive_instance(size_t type, size_t uuid_len, u8 *uuid)
{
	int ret;
	struct syno_cache_protection_fs *tmp_fs, *passive_fs = NULL;
	struct syno_cache_protection_fs_type *fs_type;

	fs_type = find_fs_type(type);
	if (!fs_type) {
		ret = -ENOENT;
		goto out;
	}
	passive_fs = fs_type->alloc_passive_instance(uuid_len, uuid);
	if (IS_ERR(passive_fs)) {
		ret = PTR_ERR(passive_fs);
		passive_fs = NULL;
		goto out;
	}

	if (!try_module_get(passive_fs->fs_type->owner)) {
		ret = -EBUSY;
		goto out;
	}
	passive_fs->module_ref = true;

	spin_lock(&instance->passive_uuid_lock);
	tmp_fs = find_passive_fs(passive_fs);
	if (tmp_fs) {
		spin_unlock(&instance->passive_uuid_lock);
		ret = -EEXIST;
		goto out;
	}
	list_add_tail(&passive_fs->list, &instance->passive_uuids);
	syno_cache_protection_fs_get(passive_fs);
	spin_unlock(&instance->passive_uuid_lock);

	ret = 0;
out:
	syno_cache_protection_fs_put(passive_fs);
	return ret;
}

static int syno_cache_protection_do_ctl_alloc_passive_instance(void *req, struct syno_cache_protection_stream_header *header)
{
	size_t fs_type, uuid_len;
	u8 *uuid;

	fs_type = le32_to_cpu(header->ctl_parm.fs_type);
	uuid_len = le32_to_cpu(header->ctl_parm.uuid_len);
	uuid = header->ctl_parm.uuid;

	return syno_cache_protection_alloc_passive_instance(fs_type, uuid_len, uuid);
}

int syno_cache_protection_alloc_passive_instance_with_fs(enum SYNO_CACHE_PROTECTION_ROLE role, size_t fs_type, size_t uuid_len, u8 *uuid)
{
	int ret = 0;
	struct syno_cache_protection_stream_header header;

	if (role == SYNO_CACHE_PROTECTION_ROLE_ACTIVE) {
		memset(&header, 0, sizeof(header));
		header.ctl_parm.fs_type = cpu_to_le32(fs_type);
		header.ctl_parm.uuid_len = cpu_to_le32(uuid_len);
		memcpy(header.ctl_parm.uuid, uuid, uuid_len);
		ret = _syno_cache_protection_send_ctl_request(SYNO_CACHE_PROTECTION_CTL_ALLOC_PASSIVE_INSTANCE, true, SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MANAGE, &header);
	} else {
		ret = syno_cache_protection_alloc_passive_instance(fs_type, uuid_len, uuid);
	}
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_alloc_passive_instance_with_fs);

static int syno_cache_protection_do_ctl_request(void *req, struct syno_cache_protection_stream_header *header)
{
	int ret;
	enum SYNO_CACHE_PROTECTION_CTL_CMD cmd;
	unsigned long flags;

	if (!req || !header || !instance) {
		ret = -EINVAL;
		goto out;
	}

	cmd = (enum SYNO_CACHE_PROTECTION_CTL_CMD)le32_to_cpu(header->ctl_parm.cmd);
	switch (cmd) {
		case SYNO_CACHE_PROTECTION_CTL_PING:
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_CTL_SPACE_ENABLE:
			ret = instance->s_op->enable();
			break;
		case SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_METADATA:
			spin_lock_irqsave(&instance->reclaim_lock, flags);
			instance->local_metadata_reclaim = true;
			spin_unlock_irqrestore(&instance->reclaim_lock, flags);
			syno_cache_protection_all_reclaim(true);
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_METADATA_END:
			spin_lock_irqsave(&instance->reclaim_lock, flags);
			instance->local_metadata_reclaim = false;
			spin_unlock_irqrestore(&instance->reclaim_lock, flags);
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_DATA:
			spin_lock_irqsave(&instance->reclaim_lock, flags);
			instance->local_data_reclaim = true;
			spin_unlock_irqrestore(&instance->reclaim_lock, flags);
			syno_cache_protection_all_reclaim(false);
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_CTL_SPACE_RECLAIM_DATA_END:
			spin_lock_irqsave(&instance->reclaim_lock, flags);
			instance->local_data_reclaim = false;
			spin_unlock_irqrestore(&instance->reclaim_lock, flags);
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_CTL_CLEAR_PASSIVE_INSTANCE:
			syno_cache_protection_do_ctl_clear_passive_instance(req, header);
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_CTL_ALLOC_PASSIVE_INSTANCE:
			ret = syno_cache_protection_do_ctl_alloc_passive_instance(req, header);
			break;
		default:
			syno_cache_protection_err("bug in unknown ctl cmd %d", (int)cmd);
			ret = -EINVAL;
			break;
	}
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

int syno_cache_protection_do_request(void *req)
{
	int ret;
	struct syno_cache_protection_stream_header header;
	enum SYNO_CACHE_PROTECTION_TYPE type;

	if (!req || !instance) {
		ret = -EINVAL;
		goto out;
	}

	ret = instance->c_op->read_req(req, sizeof(header), &header);
	if (ret)
		goto out;

	type = (enum SYNO_CACHE_PROTECTION_TYPE)le32_to_cpu(header.type);

	if (type == SYNO_CACHE_PROTECTION_TYPE_CTL) {
		ret = syno_cache_protection_do_ctl_request(req, &header);
	} else if (type == SYNO_CACHE_PROTECTION_TYPE_FS) {
		ret = syno_cache_protection_do_fs_request(req, &header);
	} else {
		syno_cache_protection_err("bug in unknown request type %d", (int)type);
		BUG();
	}
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_do_request);

void syno_cache_protection_connection_link_event(bool link_is_up)
{
	unsigned long flags;

	if (!instance) {
		const struct syno_cache_protection_space_allocate_operations *s_op = NULL;

		s_op = syno_cache_protection_mem_pool_get_space_allocator();
		if (s_op && s_op->link_event)
			s_op->link_event(link_is_up);
		goto out;
	}

	spin_lock_irqsave(&instance->reclaim_lock, flags);
	instance->local_metadata_reclaim = false;
	instance->local_data_reclaim = false;
	instance->remote_metadata_reclaim = false;
	instance->remote_data_reclaim = false;
	spin_unlock_irqrestore(&instance->reclaim_lock, flags);
	if (instance->s_op->link_event)
		instance->s_op->link_event(link_is_up);
out:
	return;
}
EXPORT_SYMBOL(syno_cache_protection_connection_link_event);

bool syno_cache_protection_check_local_reclaim(bool metadata)
{
	bool reclaim;
	unsigned long flags;

	spin_lock_irqsave(&instance->reclaim_lock, flags);
	if (metadata)
		reclaim = instance->local_metadata_reclaim;
	else
		reclaim = instance->local_data_reclaim;
	spin_unlock_irqrestore(&instance->reclaim_lock, flags);
	return reclaim;
}
EXPORT_SYMBOL(syno_cache_protection_check_local_reclaim);

static void exit_syno_cache_protection(void)
{
	// all active instance protected by module dependency
	if (instance) {
		syno_cache_protection_clear_passive_instance(SYNO_CACHE_PROTECTION_FS_ALL, 0, NULL);
	}
	kfree(instance);
	instance = NULL;
	syno_cache_protection_exit_sysfs();
	syno_cache_protection_ntb_connection_exit();
	syno_cache_protection_mem_pool_exit();
}

static int __init init_syno_cache_protection(void)
{
	int ret;

	ret = syno_cache_protection_mem_pool_init();
	if (ret)
		goto out;

	ret = syno_cache_protection_ntb_connection_init();
	if (ret)
		goto out;

	ret = syno_cache_protection_init_sysfs();
	if (ret)
		goto out;

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance) {
		ret = -ENOMEM;
		goto out;
	}
	INIT_LIST_HEAD(&instance->active_uuids);
	INIT_LIST_HEAD(&instance->passive_uuids);
	INIT_LIST_HEAD(&instance->fs_types);
	spin_lock_init(&instance->active_uuid_lock);
	spin_lock_init(&instance->passive_uuid_lock);
	spin_lock_init(&instance->fs_type_lock);
	spin_lock_init(&instance->reclaim_lock);

	instance->c_op = syno_cache_protection_ntb_connection_get_connections();
	if (!instance->c_op) {
		ret = -ENOMEM;
		goto out;
	}

	instance->s_op = syno_cache_protection_mem_pool_get_space_allocator();
	if (!instance->s_op) {
		ret = -ENOMEM;
		goto out;
	}

	ret = 0;
out:
	if (ret)
		exit_syno_cache_protection();
	return ret;
}

late_initcall(init_syno_cache_protection);
module_exit(exit_syno_cache_protection);

