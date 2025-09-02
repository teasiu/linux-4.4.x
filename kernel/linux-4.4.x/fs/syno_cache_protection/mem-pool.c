/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/module.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/syno_cache_protection.h>

#define SYNO_CACHE_PROTECTION_METADATA_NR 524288 /* 128 MB */
#define SYNO_CACHE_PROTECTION_CHECKSUM_NR 32768 /* 128 MB */

static unsigned int mempool_reclaim_ratio_high = 50;
static unsigned int mempool_reclaim_ratio_low = 30;
static unsigned int mempool_data_ratio = 20;
static unsigned int mempool_machine_min_total_pages = 1048576;
static unsigned int mempool_max_pages = 2097152;
static unsigned int mempool_min_pages = 262144;
static unsigned int mempool_reserve_pages = 524288;

module_param(mempool_reclaim_ratio_high, uint, S_IRUGO|S_IWUSR);
module_param(mempool_reclaim_ratio_low, uint, S_IRUGO|S_IWUSR);
module_param(mempool_data_ratio, uint, S_IRUGO|S_IWUSR);
module_param(mempool_machine_min_total_pages, uint, S_IRUGO|S_IWUSR);
module_param(mempool_max_pages, uint, S_IRUGO|S_IWUSR);
module_param(mempool_min_pages, uint, S_IRUGO|S_IWUSR);
module_param(mempool_reserve_pages, uint, S_IRUGO|S_IWUSR);

struct _entry {
	struct list_head list;
};

struct _pool {
	spinlock_t lock;
	size_t max, nr, background_high_nr, background_low_nr, reserve_nr;
	wait_queue_head_t wait, queued;
	struct list_head head;
	bool allocation;
};

struct syno_cache_protection_mem_pool_instance {
	const struct syno_cache_protection_space_allocate_operations *s_op;
	struct _pool *syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_MAX];
	struct mutex ctl_mutex;
	bool enabled;
	atomic_t link_is_up;
	struct work_struct reclaim_work;
};

static struct syno_cache_protection_mem_pool_instance *instance = NULL;
struct syno_cache_protection_mem_pool_instance **syno_cache_protection_mem_pool_instance_ptr = &instance;
EXPORT_SYMBOL(syno_cache_protection_mem_pool_instance_ptr);

static void pool_destroy(struct _pool* pool)
{
	struct _entry *entry;

	if (!pool)
		return;

	spin_lock(&pool->lock);
	while (!list_empty(&pool->head)) {
		entry = list_first_entry(&pool->head, struct _entry, list);
		list_del_init(&entry->list);
		kfree(entry);
		cond_resched_lock(&pool->lock);
	}
	spin_unlock(&pool->lock);
	kfree(pool);
}

static size_t mempool_calc_data_nr(void)
{
	size_t data_nr;

	data_nr = (totalram_pages * mempool_data_ratio) / 100;

	if (data_nr < mempool_min_pages)
		data_nr = mempool_min_pages;

	if (data_nr > mempool_max_pages)
		data_nr = mempool_max_pages;

	if (data_nr > (totalram_pages - mempool_reserve_pages))
		data_nr = totalram_pages - mempool_reserve_pages;

	return data_nr;
}

static struct _pool* pool_alloc(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type)
{
	int err;
	struct _pool *pool = NULL;
	size_t nr, size, i;
	struct _entry *entry;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool) {
		err = -ENOMEM;
		goto out;
	}
	spin_lock_init(&pool->lock);
	INIT_LIST_HEAD(&pool->head);
	init_waitqueue_head(&pool->wait);
	init_waitqueue_head(&pool->queued);

	if (pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA) {
		nr = SYNO_CACHE_PROTECTION_METADATA_NR;
		size = SYNO_CACHE_PROTECTION_METADATA_SIZE;
	} else if (pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM) {
		nr = SYNO_CACHE_PROTECTION_CHECKSUM_NR;
		size = SYNO_CACHE_PROTECTION_DATA_SIZE;
	} else if (pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER) {
		nr = mempool_calc_data_nr();
		size = SYNO_CACHE_PROTECTION_METADATA_SIZE;
	} else if (pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_DATA) {
		nr = mempool_calc_data_nr();
		size = SYNO_CACHE_PROTECTION_DATA_SIZE;
	} else {
		err = -EINVAL;
		goto out;
	}

	for (i = 0; i < nr; i++) {
		entry = kmalloc(size, GFP_KERNEL);
		if (!entry) {
			err = -ENOMEM;
			goto out;
		}
		INIT_LIST_HEAD(&entry->list);
		list_add_tail(&entry->list, &pool->head);
	}

	pool->max = nr;
	pool->background_high_nr = (nr * mempool_reclaim_ratio_high) / 100;
	pool->background_low_nr = (nr * mempool_reclaim_ratio_low) / 100;
	pool->nr = 0;
	pool->reserve_nr = 0;

	return pool;

out:
	pool_destroy(pool);
	return ERR_PTR(err);
}

static void syno_cache_protection_mem_pool_reclaim_work(struct work_struct *work)
{
	signed long timeout;
	unsigned long expire;

again:
	if (!instance)
		goto out;

	expire = jiffies + msecs_to_jiffies(1000);

	mutex_lock(&instance->ctl_mutex);

	if (!instance->enabled || atomic_read(&instance->link_is_up) == 0) {
		syno_cache_protection_send_reclaim_end(true);
		syno_cache_protection_send_reclaim_end(false);
		mutex_unlock(&instance->ctl_mutex);
		goto out;
	}

	if ((instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->nr > instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->background_high_nr) ||
		(instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM]->nr > instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM]->background_high_nr)) {
		syno_cache_protection_send_reclaim(true);
	} else if ((instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->nr < instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->background_low_nr) &&
				(instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM]->nr < instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM]->background_low_nr)) {
		syno_cache_protection_send_reclaim_end(true);
	}

	if ((instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->nr > instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->background_high_nr) ||
		(instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER]->nr > instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER]->background_high_nr) ||
		(instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA]->nr > instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA]->background_high_nr)) {
		syno_cache_protection_send_reclaim(false);
	} else if ((instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->nr < instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA]->background_low_nr) &&
				(instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER]->nr < instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER]->background_low_nr) &&
				(instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA]->nr < instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA]->background_low_nr)) {
		syno_cache_protection_send_reclaim_end(false);
	}

	mutex_unlock(&instance->ctl_mutex);

	timeout = expire - jiffies;
	if (timeout > 0)
		schedule_timeout_interruptible(1 + timeout);
	goto again;

out:
	return;
}

static void __syno_cache_protection_mem_pool_free(struct _pool *pool, void *data)
{
	struct _entry *entry;

	WARN_ON_ONCE(!spin_is_locked(&pool->lock));

	if (!data)
		goto out;

	entry = data;
	INIT_LIST_HEAD(&entry->list);
	list_add_tail(&entry->list, &pool->head);

out:
	return;
}

static void syno_cache_protection_mem_pool_free(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, void *data)
{
	struct _pool *pool;

	if (!instance || !instance->enabled || !data)
		return;
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		return;
	}

	pool = instance->syno_cache_protection_pool[pool_type];
	WARN_ON_ONCE(!pool->nr);
	spin_lock(&pool->lock);
	__syno_cache_protection_mem_pool_free(pool, data);
	pool->nr--;
	spin_unlock(&pool->lock);
	if (waitqueue_active(&pool->wait))
		wake_up(&pool->wait);
}

static void* __syno_cache_protection_mem_pool_alloc(struct _pool *pool)
{
	struct _entry *entry = NULL;

	WARN_ON_ONCE(!spin_is_locked(&pool->lock));

	if (WARN_ON_ONCE(list_empty(&pool->head)))
		goto out;

	entry = list_first_entry(&pool->head, struct _entry, list);
	list_del_init(&entry->list);

out:
	return entry;
}

static void __syno_cache_protection_mem_pool_reclaim(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type)
{
	struct _pool *pool;

	if (!instance || !instance->enabled || atomic_read(&instance->link_is_up) == 0)
		return;
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		return;
	}

	pool = instance->syno_cache_protection_pool[pool_type];
	if (pool->nr < pool->background_high_nr)
		return;

	if (pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA ||
		pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER ||
		pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_DATA)
		syno_cache_protection_send_reclaim(false);
	if (pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA ||
		pool_type == SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM)
		syno_cache_protection_send_reclaim(true);
}

static void syno_cache_protection_mem_pool_reserve_free(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count)
{
	struct _pool *pool;

	if (!instance || !instance->enabled)
		return;
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		return;
	}
	if (!count)
		return;

	pool = instance->syno_cache_protection_pool[pool_type];
	spin_lock(&pool->lock);
	if (pool->reserve_nr < count) {
		count = pool->reserve_nr;
		WARN_ON_ONCE(1);
	}
	pool->reserve_nr -= count;
	spin_unlock(&pool->lock);
	if (waitqueue_active(&pool->wait))
		wake_up(&pool->wait);
}

static int syno_cache_protection_mem_pool_reserve(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, size_t count, gfp_t gfp_mask)
{
	int ret;
	struct _pool *pool;
	bool queued = false;
	wait_queue_t wait;

	if (!instance || !instance->enabled) {
		ret = -ENOSPC;
		goto out;
	}
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		goto out;
	}
	if (!count) {
		ret = 0;
		goto out;
	}

	pool = instance->syno_cache_protection_pool[pool_type];
again:
	spin_lock(&pool->lock);
	if (pool->allocation && !queued) {
		spin_unlock(&pool->lock);
		if (!(gfp_mask & __GFP_DIRECT_RECLAIM) || (atomic_read(&instance->link_is_up) == 0)) {
			ret = -ENOSPC;
			goto out;
		}
		queued = true;
		init_wait(&wait);
		prepare_to_wait_exclusive(&pool->queued, &wait, TASK_UNINTERRUPTIBLE);
		schedule();
		finish_wait(&pool->queued, &wait);
		goto again;
	}
	if (pool->nr + pool->reserve_nr + count > pool->max) {
		if (!(gfp_mask & __GFP_DIRECT_RECLAIM) || (atomic_read(&instance->link_is_up) == 0)) {
			ret = -ENOSPC;
			goto out_wake_up;
		}
		queued = true;
		pool->allocation = true;
		spin_unlock(&pool->lock);
		__syno_cache_protection_mem_pool_reclaim(pool_type);
		wait_event(pool->wait, (pool->nr + pool->reserve_nr + count <= pool->max) || (atomic_read(&instance->link_is_up) == 0));
		goto again;
	}
	pool->reserve_nr += count;
	ret = 0;

out_wake_up:
	if (!waitqueue_active(&pool->queued))
		pool->allocation = false;
	else
		wake_up(&pool->queued);
	spin_unlock(&pool->lock);
	if (gfp_mask & __GFP_DIRECT_RECLAIM)
		__syno_cache_protection_mem_pool_reclaim(pool_type);
out:
	return ret;
}

static void* syno_cache_protection_mem_pool_alloc_with_throttle(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, gfp_t gfp_mask)
{
	void *data = NULL;
	struct _pool *pool;
	int count = 1;
	bool queued = false;
	wait_queue_t wait;

	if (!instance || !instance->enabled)
		goto out;
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		goto out;
	}

	pool = instance->syno_cache_protection_pool[pool_type];
again:
	spin_lock(&pool->lock);
	if (pool->allocation && !queued) {
		spin_unlock(&pool->lock);
		if (!(gfp_mask & __GFP_DIRECT_RECLAIM) || (atomic_read(&instance->link_is_up) == 0))
			goto out;
		queued = true;
		init_wait(&wait);
		prepare_to_wait_exclusive(&pool->queued, &wait, TASK_UNINTERRUPTIBLE);
		schedule();
		finish_wait(&pool->queued, &wait);
		goto again;
	}
	if (pool->nr + pool->reserve_nr + count > pool->max) {
		if (!(gfp_mask & __GFP_DIRECT_RECLAIM) || (atomic_read(&instance->link_is_up) == 0))
			goto out_wake_up;
		queued = true;
		pool->allocation = true;
		spin_unlock(&pool->lock);
		__syno_cache_protection_mem_pool_reclaim(pool_type);
		wait_event(pool->wait, (pool->nr + pool->reserve_nr + count <= pool->max) || (atomic_read(&instance->link_is_up) == 0));
		goto again;
	}
	pool->nr += count;
	data = __syno_cache_protection_mem_pool_alloc(pool);

out_wake_up:
	if (!waitqueue_active(&pool->queued))
		pool->allocation = false;
	else
		wake_up(&pool->queued);
	spin_unlock(&pool->lock);
	if (gfp_mask & __GFP_DIRECT_RECLAIM)
		__syno_cache_protection_mem_pool_reclaim(pool_type);
out:
	return data;
}

static void* syno_cache_protection_mem_pool_alloc_with_reserved(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type)
{
	void *data = NULL;
	struct _pool *pool;

	if (!instance || !instance->enabled)
		goto out;
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		goto out;
	}

	pool = instance->syno_cache_protection_pool[pool_type];
	WARN_ON_ONCE(pool->nr >= pool->max);
	spin_lock(&pool->lock);
	pool->nr++;
	data = __syno_cache_protection_mem_pool_alloc(pool);
	spin_unlock(&pool->lock);
	WARN_ON_ONCE(!data);
out:
	return data;
}

static void* syno_cache_protection_mem_pool_alloc(enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type, gfp_t gfp_mask, bool reserved)
{
	if (!instance || !instance->enabled)
		return NULL;
	if (pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		WARN_ON_ONCE(1);
		return NULL;
	}

	if (reserved)
		return syno_cache_protection_mem_pool_alloc_with_reserved(pool_type);
	else
		return syno_cache_protection_mem_pool_alloc_with_throttle(pool_type, gfp_mask);
}

static int __syno_cache_protection_mem_pool_disable(bool force)
{
	int ret;
	size_t i;

	if (!instance) {
		ret = 0;
		goto out;
	}

	mutex_lock(&instance->ctl_mutex);

	for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++) {
		if (instance->syno_cache_protection_pool[i] && (instance->syno_cache_protection_pool[i]->nr > 0)) {
			if (!force) {
				ret = -EBUSY;
				goto out_unlock;
			} else {
				WARN_ON_ONCE(1);
			}
		}
	}

	instance->enabled = false;
	for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++) {
		pool_destroy(instance->syno_cache_protection_pool[i]);
		instance->syno_cache_protection_pool[i] = NULL;
	}

	ret = 0;
out_unlock:
	mutex_unlock(&instance->ctl_mutex);
out:
	return ret;
}

static int syno_cache_protection_mem_pool_disable(void)
{
	return __syno_cache_protection_mem_pool_disable(false);
}

static int syno_cache_protection_mem_pool_enable(void)
{
	int ret;
	size_t i;
	struct _pool *pool;

	if (!instance)
		return -EINVAL;

	mutex_lock(&instance->ctl_mutex);

	if (instance->enabled)
		goto success;

	if (totalram_pages < mempool_machine_min_total_pages) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++) {
		pool = pool_alloc((enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE)i);
		if (IS_ERR(pool)) {
			ret = PTR_ERR(pool);
			goto out;
		}
		instance->syno_cache_protection_pool[i] = pool;
	}
	if (instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA]->max !=
		instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER]->max) {
		ret = -EINVAL;
		goto out;
	}

	instance->enabled = true;
	queue_work(system_unbound_wq, &instance->reclaim_work);

success:
	ret = 0;
out:
	mutex_unlock(&instance->ctl_mutex);
	if (ret)
		__syno_cache_protection_mem_pool_disable(true);
	return ret;
}

static bool syno_cache_protection_mem_pool_enabled(void)
{
	if (instance)
		return instance->enabled;
	return false;
}

static void syno_cache_protection_mem_pool_link_event(bool link_is_up)
{
	int i;

	if (!instance)
		return;

	atomic_set(&instance->link_is_up, link_is_up ? 1 : 0);

	if (!instance->enabled)
		return;

	if (atomic_read(&instance->link_is_up) == 0) {
		for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++) {
			wake_up(&instance->syno_cache_protection_pool[i]->wait);
		}
	} else {
		if (!work_busy(&instance->reclaim_work))
			queue_work(system_unbound_wq, &instance->reclaim_work);
	}
}

static int syno_cache_protection_mem_pool_status(char* buf)
{
	int len = 0;
	struct _pool *pool;

	if (!instance)
		goto out;

	len += snprintf(buf + len, PAGE_SIZE - len, "Enabled:%d\n", instance->enabled);
	len += snprintf(buf + len, PAGE_SIZE - len, "Link:%d\n", atomic_read(&instance->link_is_up));

	if (!instance->enabled)
		goto out;

	pool = instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA];
	len += snprintf(buf + len, PAGE_SIZE - len, "metadata: %zd / %zd (reserve %zd)\n", pool->nr, pool->max, pool->reserve_nr);
	pool = instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM];
	len += snprintf(buf + len, PAGE_SIZE - len, "checksum: %zd / %zd (reserve %zd)\n", pool->nr, pool->max, pool->reserve_nr);
	pool = instance->syno_cache_protection_pool[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA];
	len += snprintf(buf + len, PAGE_SIZE - len, "data: %zd / %zd (reserve %zd)\n", pool->nr, pool->max, pool->reserve_nr);

out:
	return len;
}

const struct syno_cache_protection_space_allocate_operations* syno_cache_protection_mem_pool_get_space_allocator(void)
{
	if (instance)
		return instance->s_op;
	return NULL;
}
EXPORT_SYMBOL(syno_cache_protection_mem_pool_get_space_allocator);

static const struct syno_cache_protection_space_allocate_operations mem_pool_ops = {
	.alloc = syno_cache_protection_mem_pool_alloc,
	.free = syno_cache_protection_mem_pool_free,
	.reserve = syno_cache_protection_mem_pool_reserve,
	.reserve_free = syno_cache_protection_mem_pool_reserve_free,
	.status = syno_cache_protection_mem_pool_status,
	.enable = syno_cache_protection_mem_pool_enable,
	.disable = syno_cache_protection_mem_pool_disable,
	.enabled = syno_cache_protection_mem_pool_enabled,
	.link_event = syno_cache_protection_mem_pool_link_event,
};

void syno_cache_protection_mem_pool_exit(void)
{
	if (!instance)
		return;

	__syno_cache_protection_mem_pool_disable(true);
	cancel_work_sync(&instance->reclaim_work);
	kfree(instance);
	instance = NULL;
}
EXPORT_SYMBOL(syno_cache_protection_mem_pool_exit);

int __init syno_cache_protection_mem_pool_init(void)
{
	int ret;

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance) {
		ret = -ENOMEM;
		goto out;
	}
	instance->s_op = &mem_pool_ops;
	instance->enabled = false;
	atomic_set(&instance->link_is_up, 0);
	mutex_init(&instance->ctl_mutex);
	INIT_WORK(&instance->reclaim_work, syno_cache_protection_mem_pool_reclaim_work);

	ret = 0;
out:
	if (ret)
		syno_cache_protection_mem_pool_exit();
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_mem_pool_init);
