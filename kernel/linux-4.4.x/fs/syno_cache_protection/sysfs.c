/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/syno_cache_protection.h>
#include "internal.h"

static struct kset *syno_cache_protection_kset = NULL;

#define __INIT_KOBJ_ATTR(_name, _mode, _show, _store)			\
{									\
	.attr	= { .name = __stringify(_name), .mode = _mode },	\
	.show	= _show,						\
	.store	= _store,						\
}

#define SYNO_CACHE_PROTECTION_ATTR_RW(_name, _show, _store)			\
	static struct kobj_attribute syno_cache_protection_attr_##_name =		\
			__INIT_KOBJ_ATTR(_name, 0644, _show, _store)

#define SYNO_CACHE_PROTECTION_ATTR_WRONLY(_name, _store)					\
	static struct kobj_attribute syno_cache_protection_attr_##_name =		\
			__INIT_KOBJ_ATTR(_name, 0644, NULL, _store)

#define SYNO_CACHE_PROTECTION_ATTR_RDONLY(_name, _show)					\
	static struct kobj_attribute syno_cache_protection_attr_##_name =		\
			__INIT_KOBJ_ATTR(_name, 0444, _show, NULL)
#define SYNO_CACHE_PROTECTION_ATTR(_name, _show) SYNO_CACHE_PROTECTION_ATTR_RDONLY(_name, _show)

#define SYNO_CACHE_PROTECTION_ATTR_PTR(_name)    (&syno_cache_protection_attr_##_name.attr)

static ssize_t syno_cache_protection_pool_status_show(struct kobject *kobj,
				struct kobj_attribute *a, char *buf)
{


	int len = 0;

	if (!instance || !instance->s_op || !instance->s_op->status) {
		len = -EINVAL;
		goto out;
	}

	len = instance->s_op->status(buf);

	len += snprintf(buf + len, PAGE_SIZE - len, "Reclaim Status:\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "  Local : Metadata:%d, Data:%d\n", instance->local_metadata_reclaim ? 1 : 0, instance->local_data_reclaim ? 1 : 0);
	len += snprintf(buf + len, PAGE_SIZE - len, "  Remote: Metadata:%d, Data:%d\n", instance->remote_metadata_reclaim ? 1 : 0, instance->remote_data_reclaim ? 1 : 0);
out:
	return len;
}
SYNO_CACHE_PROTECTION_ATTR(pool_status, syno_cache_protection_pool_status_show);

static ssize_t syno_cache_protection_pool_enable_show(struct kobject *kobj,
				struct kobj_attribute *a, char *buf)
{
	if (!instance || !instance->s_op || !instance->s_op->enabled)
		return -EINVAL;
	return snprintf(buf, PAGE_SIZE, "%u\n", instance->s_op->enabled() ? 1 : 0);
}

static ssize_t syno_cache_protection_pool_enable_store(struct kobject *kobj,
				 struct kobj_attribute *a,
				 const char *buf, size_t len)
{
	int err;
	u8 val;

	if (!instance || !instance->s_op || !instance->s_op->enable || !instance->s_op->disable) {
		len = -EINVAL;
		goto out;
	}

	if (len > 2) {
		len = -EINVAL;
		goto out;
	}

	err = kstrtou8(skip_spaces(buf), 0, &val);
	if (err) {
		len = err;
		goto out;
	}

	if (val > 0)
		err = instance->s_op->enable();
	else
		err = instance->s_op->disable();

	if (err) {
		len = err;
		goto out;
	}

out:
	return len;
}
SYNO_CACHE_PROTECTION_ATTR_RW(pool_enable, syno_cache_protection_pool_enable_show, syno_cache_protection_pool_enable_store);

static ssize_t syno_cache_protection_connection_status_show(struct kobject *kobj,
				struct kobj_attribute *a, char *buf)
{
	int len = 0;

	if (!instance || !instance->c_op || !instance->c_op->status) {
		len = -EINVAL;
		goto out;
	}

	len = instance->c_op->status(buf);
out:
	return len;
}
SYNO_CACHE_PROTECTION_ATTR(connection_status, syno_cache_protection_connection_status_show);

static ssize_t syno_cache_protection_release_all(struct kobject *kobj,
				 struct kobj_attribute *a,
				 const char *buf, size_t len)
{
	int err;
	u8 val;

	if (!instance || len > 2) {
		len = -EINVAL;
		goto out;
	}

	err = kstrtou8(skip_spaces(buf), 0, &val);
	if (err) {
		len = err;
		goto out;
	}

	if (1 == val)
		err = syno_cache_protection_clear_all();

	if (err) {
		len = err;
		goto out;
	}

out:
	return len;
}
SYNO_CACHE_PROTECTION_ATTR_WRONLY(release_all, syno_cache_protection_release_all);

static ssize_t syno_cache_protection_reclaim_all(struct kobject *kobj,
				 struct kobj_attribute *a,
				 const char *buf, size_t len)
{
	int err;
	u8 val;

	if (!instance) {
		len = -EINVAL;
		goto out;
	}

	err = kstrtou8(skip_spaces(buf), 0, &val);
	if (err) {
		len = err;
		goto out;
	}

	if (val & 1)
		syno_cache_protection_all_reclaim(true);
	if (val & 2)
		syno_cache_protection_all_reclaim(false);

	if (err) {
		len = err;
		goto out;
	}

out:
	return len;
}
SYNO_CACHE_PROTECTION_ATTR_WRONLY(reclaim_all, syno_cache_protection_reclaim_all);

static const struct attribute *syno_cache_protection_attrs[] = {
	SYNO_CACHE_PROTECTION_ATTR_PTR(pool_status),
	SYNO_CACHE_PROTECTION_ATTR_PTR(pool_enable),
	SYNO_CACHE_PROTECTION_ATTR_PTR(connection_status),
	SYNO_CACHE_PROTECTION_ATTR_PTR(release_all),
	SYNO_CACHE_PROTECTION_ATTR_PTR(reclaim_all),
	NULL,
};

void syno_cache_protection_exit_sysfs(void)
{
	if (!syno_cache_protection_kset)
		return;

	sysfs_remove_files(&syno_cache_protection_kset->kobj, syno_cache_protection_attrs);
	kset_unregister(syno_cache_protection_kset);
	syno_cache_protection_kset = NULL;
}

int __init syno_cache_protection_init_sysfs(void)
{
	int ret;

	syno_cache_protection_kset = kset_create_and_add("syno-cache-protection", NULL, fs_kobj);
	if (!syno_cache_protection_kset) {
		ret = -ENOMEM;
		goto out;
	}

	ret = sysfs_create_files(&syno_cache_protection_kset->kobj, syno_cache_protection_attrs);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}
