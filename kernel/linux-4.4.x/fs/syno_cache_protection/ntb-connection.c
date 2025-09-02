/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/mempool.h>
#include <linux/mempool.h>
#include <linux/ntb.h>
#include <linux/ntb_transport.h>
#include <linux/syno_cache_protection.h>
#include "util.h"

#define SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX SYNO_CACHE_PROTECTION_CONNECTION_CHANNEL_MAX
#define SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_REQUEST_MAX 128

#define SYNO_CACHE_PROTECTION_NTB_ENTRY_MAX_BUFFER 128

#define SYNO_CACHE_PROTECTION_NTB_POOL_SIZE (PAGE_CACHE_SIZE)
#define SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE 1024
/*
 * Metadata : 4 * 256 = 1024
 * Data : 3072
 */
#define SYNO_CACHE_PROTECTION_NTB_POOL_MIN_NR  1024
#define SYNO_CACHE_PROTECTION_NTB_POOL_NR  4096

#define SYNO_CACHE_PROTECTION_NTB_CONNECTION_REQUEST_MAX (SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX * SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_REQUEST_MAX)

#define SYNO_CACHE_PROTECTION_NTB_CONNECTION_RX_BUFFER_MAX 256

#define SYNO_CACHE_PROTECTION_NTB_MAGIC 0x4D5F50434F4E5953ULL /* ascii SYNOCP_M, no null */

struct syno_cache_protection_ntb_stream_buffer {
	__le32 offset;
	__le32 len;
	__le64 addr;
} __attribute__ ((__packed__));

struct syno_cache_protection_ntb_stream_entry_payload {
	__le32 buffer_count;
	struct syno_cache_protection_ntb_stream_buffer buffers[SYNO_CACHE_PROTECTION_NTB_ENTRY_MAX_BUFFER];
} __attribute__ ((__packed__));

struct syno_cache_protection_ntb_stream_entry_header {
	__le64 magic;
	__le32 response;
	__le32 channel;
	__le32 err;
	__le32 data_len;
	__le64 offset;
} __attribute__ ((__packed__));

struct syno_cache_protection_ntb_buffer {
	u32 offset;
	u32 len;
	u64 addr;
};

struct syno_cache_protection_ntb_entry {
	struct list_head list;
	struct rb_node entry_node;
	unsigned long long offset;
	struct completion completion;
	bool wait;
	size_t channel;
	size_t init_size, size, data_len;
	int err;
	size_t vec_index, vec_done;
	size_t buffer_count;
	struct syno_cache_protection_ntb_buffer buffers[SYNO_CACHE_PROTECTION_NTB_ENTRY_MAX_BUFFER];
	struct list_head extra_head;
	struct syno_cache_protection_ntb_stream_entry_payload payload;
};

struct syno_cache_protection_ntb_rx_work {
	struct list_head list;
	struct work_struct normal_work;
	struct syno_cache_protection_ntb_entry entry;
	struct syno_cache_protection_ntb_stream_entry_header header;
	struct syno_cache_protection_ntb_stream_entry_payload payload;
};

struct syno_cache_protection_ntb_connection_pool {
	struct list_head head;
	wait_queue_head_t wait;
	wait_queue_head_t queued[SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX];
	bool allocation[SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX];
	spinlock_t lock;
	size_t nr;
};

struct syno_cache_protection_ntb_connection_instance {
	const struct syno_cache_protection_connection_operations *c_op;
	struct device *client_dev;
	bool ntb_initialized;
	struct ntb_transport_raw_block *raw_block;
	unsigned long long ntb_total_size, ntb_used_size;

	struct ntb_transport_qp *qp;
	bool ntb_qp_link_is_up;
	unsigned int mtu;
	struct syno_cache_protection_ntb_connection_pool rx_work_pool;
	u64 rx_buffer_count;
	void **rx_buffers;
	wait_queue_head_t tx_wait_queue;
	struct work_struct link_down_work;

	struct workqueue_struct *channel_workqueue[SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX];
	atomic_t channel_nr[SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX];
	wait_queue_head_t channel_queue_wait[SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX];
	struct syno_cache_protection_ntb_connection_pool free_pool;
	atomic_t response_pending_nr;
	bool enable;
	spinlock_t request_tree_lock;
	struct rb_root request_tree;
};

static void ntb_connection_put_req(void *req);

static struct syno_cache_protection_ntb_connection_instance *instance = NULL;
struct syno_cache_protection_ntb_connection_instance **syno_cache_protection_ntb_connection_instance_ptr = &instance;
EXPORT_SYMBOL(syno_cache_protection_ntb_connection_instance_ptr);

static void syno_cache_protection_ntb_receiver_do_work(struct work_struct *normal_work);

static void syno_cache_protection_ntb_connection_pool_init(struct syno_cache_protection_ntb_connection_pool *pool)
{
	int i;

	memset(pool, 0, sizeof(*pool));
	INIT_LIST_HEAD(&pool->head);
	init_waitqueue_head(&pool->wait);
	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX; i++)
		init_waitqueue_head(&pool->queued[i]);
	spin_lock_init(&pool->lock);
}

static struct syno_cache_protection_ntb_entry *__syno_cache_protection_ntb_connection_request_tree_search(struct rb_root *root, unsigned long long offset)
{
	struct rb_node *n = root->rb_node;
	struct syno_cache_protection_ntb_entry *entry;

	while (n) {
		entry = rb_entry(n, struct syno_cache_protection_ntb_entry, entry_node);

		if (offset < entry->offset)
			n = n->rb_left;
		else if (offset > entry->offset)
			n = n->rb_right;
		else
			return entry;
	}

	return NULL;
}

static int __syno_cache_protection_ntb_connection_request_tree_insert(struct rb_root *root, struct syno_cache_protection_ntb_entry *new)
{
	struct rb_node **p = &root->rb_node;
	struct rb_node *parent = NULL;
	struct syno_cache_protection_ntb_entry *entry = NULL;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct syno_cache_protection_ntb_entry, entry_node);

		if (new->offset < entry->offset)
			p = &(*p)->rb_left;
		else if (new->offset > entry->offset)
			p = &(*p)->rb_right;
		else
			return -EEXIST;
	}

	rb_link_node(&new->entry_node, parent, p);
	rb_insert_color(&new->entry_node, root);
	return 0;
}

static struct syno_cache_protection_ntb_entry *syno_cache_protection_ntb_connection_request_tree_get_and_remove(unsigned long long offset)
{
	struct syno_cache_protection_ntb_entry *entry = NULL;
	unsigned long flags;

	if (!instance)
		goto out;

	spin_lock_irqsave(&instance->request_tree_lock, flags);
	entry = __syno_cache_protection_ntb_connection_request_tree_search(&instance->request_tree, offset);
	if (entry) {
		rb_erase(&entry->entry_node, &instance->request_tree);
		RB_CLEAR_NODE(&entry->entry_node);
	}
	spin_unlock_irqrestore(&instance->request_tree_lock, flags);
out:
	return entry;
}

static int syno_cache_protection_ntb_connection_request_tree_insert(struct syno_cache_protection_ntb_entry *entry)
{
	int ret;
	unsigned long flags;

	if (!instance || !instance->enable) {
		ret = -EINVAL;
		goto out;
	}

	WARN_ON(!RB_EMPTY_NODE(&entry->entry_node));

	spin_lock_irqsave(&instance->request_tree_lock, flags);
	ret = __syno_cache_protection_ntb_connection_request_tree_insert(&instance->request_tree, entry);
	spin_unlock_irqrestore(&instance->request_tree_lock, flags);
out:
	return ret;
}

static void syno_cache_protection_ntb_connection_request_tree_remove(struct syno_cache_protection_ntb_entry *entry)
{
	unsigned long flags;

	if (!instance)
		return;

	if (RB_EMPTY_NODE(&entry->entry_node))
		return;

	spin_lock_irqsave(&instance->request_tree_lock, flags);
	if (!RB_EMPTY_NODE(&entry->entry_node)) {
		rb_erase(&entry->entry_node, &instance->request_tree);
		RB_CLEAR_NODE(&entry->entry_node);
	}
	spin_unlock_irqrestore(&instance->request_tree_lock, flags);
}

/*
 * NTB and driver implementation
 */

static int __ntb_start_xmit(void *data, size_t len, bool can_wait, bool polling)
{
	int ret;
	DEFINE_WAIT(wait);
	bool accounting = false;

	if (!instance || !instance->ntb_initialized) {
		ret = -EINVAL;
		goto out;
	}

again:
	if (!instance->ntb_qp_link_is_up) {
		ret = -ENOTCONN;
		goto out;
	}

	ret = ntb_transport_tx_enqueue(instance->qp, NULL, data, len);
	if ((ret == -EBUSY || ret == -EAGAIN) && can_wait && instance->ntb_qp_link_is_up) {
		if (!accounting && polling) {
			atomic_inc(&instance->response_pending_nr);
			accounting = true;
		}
		prepare_to_wait(&instance->tx_wait_queue, &wait, TASK_UNINTERRUPTIBLE);
		if (polling)
			schedule_timeout_uninterruptible(msecs_to_jiffies(5));
		else if (instance->ntb_qp_link_is_up)
			schedule();
		finish_wait(&instance->tx_wait_queue, &wait);
		goto again;
	} else if (ret) {
		syno_cache_protection_err("Failed to ntb_transport_tx_enqueue with err:%d", ret);
		goto out;
	}
	if (waitqueue_active(&instance->tx_wait_queue))
		wake_up(&instance->tx_wait_queue);

	ret = 0;
out:
	if (accounting)
		atomic_dec(&instance->response_pending_nr);
	return ret;
}

static int ntb_start_xmit(void *data, size_t len)
{
	return __ntb_start_xmit(data, len, true, false);
}

static int ntb_start_xmit_with_response(void *data, size_t len)
{
	return __ntb_start_xmit(data, len, true, true);
}

static int ntb_start_xmit_with_response_nowait(void *data, size_t len)
{
	return __ntb_start_xmit(data, len, false, true);
}

static void ntb_rx_handler(struct ntb_transport_qp *qp, void *qp_data, void *data, int len)
{
	int ret;
	struct syno_cache_protection_ntb_stream_entry_header *header = data;
	struct syno_cache_protection_ntb_entry *entry;
	struct syno_cache_protection_ntb_stream_entry_header response;
	struct syno_cache_protection_ntb_rx_work *work;
	size_t channel;
	bool success = false;
	unsigned long flags;

	if (!instance || !instance->ntb_initialized) {
		return;
	}

	if (len < sizeof(*header)) {
		syno_cache_protection_err("Failed to invalid header with len too small [%d]", len);
		goto enqueue_again;
	}

	if (le64_to_cpu(header->magic) != SYNO_CACHE_PROTECTION_NTB_MAGIC) {
		syno_cache_protection_err("Failed to invalid header with magic [0x%llx]", le64_to_cpu(header->magic));
		goto enqueue_again;
	}

	if (le32_to_cpu(header->response)) {
		entry = syno_cache_protection_ntb_connection_request_tree_get_and_remove(le64_to_cpu(header->offset));
		if (entry) {
			if (entry->wait) {
				entry->err = le32_to_cpu(header->err);
				complete(&entry->completion);
			} else {
				ntb_connection_put_req(entry);
			}
		} else {
			syno_cache_protection_err("Failed to entry not found with offset[%llu]", le64_to_cpu(header->offset));
		}
		if (waitqueue_active(&instance->tx_wait_queue))
			wake_up(&instance->tx_wait_queue);
	} else {
		if (!instance->enable) {
			ret = -EINVAL;
			goto response;
		}

		spin_lock_irqsave(&instance->rx_work_pool.lock, flags);
		if (!list_empty(&instance->rx_work_pool.head)) {
			work = list_first_entry(&instance->rx_work_pool.head, struct syno_cache_protection_ntb_rx_work, list);
			list_del_init(&work->list);
		} else {
			work = NULL;
		}
		spin_unlock_irqrestore(&instance->rx_work_pool.lock, flags);

		if (!work) {
			ret = -EOVERFLOW;
			syno_cache_protection_err("Failed to rx work overflow");
			WARN_ON_ONCE(1);
			goto response;
		}

		channel = le32_to_cpu(header->channel);
		if (channel >= SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX) {
			syno_cache_protection_err("Failed to ERR channel [%zd]", channel);
			channel = 0;
		}
		memcpy(&work->header, header, sizeof(*header));
		INIT_WORK(&work->normal_work, syno_cache_protection_ntb_receiver_do_work);
		queue_work(instance->channel_workqueue[channel],  &work->normal_work);
		success = true;

response:
		if (!success) {
			memset(&response, 0, sizeof(response));
			response.magic = cpu_to_le64(SYNO_CACHE_PROTECTION_NTB_MAGIC);
			response.response = cpu_to_le32(1);
			response.err = cpu_to_le32(ret);
			response.offset = header->offset;
			ret = ntb_start_xmit_with_response_nowait(&response, sizeof(response));
			if (ret)
				syno_cache_protection_err("Failed to response ntb_start_xmit_nowait with offset:%llu", le64_to_cpu(response.offset));
		}
	}

enqueue_again:
	memset(header, 0, sizeof(*header));
	ret = ntb_transport_rx_enqueue(qp, data, data, instance->mtu);
	if (ret)
		syno_cache_protection_err("Failed to ntb_transport_rx_enqueue");
}

static void ntb_connection_clear_all_pending_entries(void)
{
	struct rb_node *node;
	struct syno_cache_protection_ntb_entry *entry;
	unsigned long flags;

	if (!instance)
		return;

	spin_lock_irqsave(&instance->request_tree_lock, flags);
	while (!RB_EMPTY_ROOT(&instance->request_tree)) {
		node = rb_first(&instance->request_tree);
		entry = rb_entry(node, struct syno_cache_protection_ntb_entry, entry_node);
		rb_erase(&entry->entry_node, &instance->request_tree);
		RB_CLEAR_NODE(&entry->entry_node);
		if (entry->wait) {
			entry->err = -ENOTCONN;
			complete(&entry->completion);
		} else {
			ntb_connection_put_req(entry);
		}
		if (need_resched()) {
			spin_unlock_irqrestore(&instance->request_tree_lock, flags);
			cond_resched();
			spin_lock_irqsave(&instance->request_tree_lock, flags);
		}
	}
	spin_unlock_irqrestore(&instance->request_tree_lock, flags);
}

static void ntb_connection_entry_wake_up_all(struct work_struct *work)
{
	ntb_connection_clear_all_pending_entries();
	if (waitqueue_active(&instance->tx_wait_queue))
		wake_up(&instance->tx_wait_queue);
}

static void ntb_event_handler(void *data, int link_is_up)
{
	if (!instance)
		return;

	if (link_is_up) {
		instance->ntb_qp_link_is_up = true;
	} else {
		instance->ntb_qp_link_is_up = false;
		if (!work_busy(&instance->link_down_work))
			queue_work(system_unbound_wq, &instance->link_down_work);
	}
	syno_cache_protection_connection_link_event(instance->ntb_qp_link_is_up);
}

static const struct ntb_queue_handlers ntb_handlers = {
	.rx_handler = ntb_rx_handler,
	.event_handler = ntb_event_handler,
};

static int copy_to_remote(unsigned long long offset, const void *src, size_t len)
{
	int ret;
	char __iomem *dst;

	if (!instance || !instance->ntb_initialized) {
		ret = -EINVAL;
		goto out;
	}

	if (!instance->raw_block->link_is_up) {
		ret = -ENOTCONN;
		goto out;
	}

	dst = (char __iomem *)(instance->raw_block->tx_buff + offset);

	memcpy_toio(dst, src, len);

	ret = 0;
out:
	return ret;
}

static int copy_from_local(void *dst, unsigned long long offset, size_t len)
{
	int ret;
	void *src;

	if (!instance || !instance->ntb_initialized) {
		ret = -EINVAL;
		goto out;
	}

	src = (void *)(instance->raw_block->rx_buff + offset);

	memcpy(dst, src, len);

	ret = 0;
out:
	return ret;
}

#if 0
static int copy_from_remote(void *dst , unsigned long long offset, size_t len)
{
	int ret;
	char __iomem *src;

	if (!instance || !instance->ntb_initialized) {
		ret = -EINVAL;
		goto out;
	}

	if (!instance->raw_block->link_is_up) {
		ret = -ENOTCONN;
		goto out;
	}

	src = (char __iomem *)(instance->raw_block->tx_buff + offset);

	memcpy_fromio(dst, src, n);

	ret = 0;
out:
	return ret;
}

static int copy_to_local(unsigned long long offset, const void *src, size_t len)
{
	int ret;
	void *dst;

	if (!instance || !instance->ntb_initialized) {
		ret = -EINVAL;
		goto out;
	}

	dst = (void *)(instance->raw_block->rx_buff + offset);

	memcpy(dst, src, len);

	ret = 0;
out:
	return ret;
}
#endif

static void ntb_connection_remove(struct device *client_dev)
{
	unsigned len;
	size_t i;
	void *rx_buffer;

	if (!instance)
		return;

	instance->ntb_initialized = false;

	if (instance->raw_block) {
		ntb_transport_free_block(instance->raw_block);
		instance->raw_block = NULL;
	}

	ntb_transport_link_down(instance->qp);

	while ((rx_buffer = ntb_transport_rx_remove(instance->qp, &len))) {}

	for (i = 0; i < instance->rx_buffer_count; i++) {
		kfree(instance->rx_buffers[i]);
		instance->rx_buffers[i] = NULL;
	}
	kfree(instance->rx_buffers);
	instance->rx_buffers = NULL;

	ntb_transport_free_queue(instance->qp);
	instance->qp = NULL;
	instance->client_dev = NULL;
}

static int ntb_connection_probe(struct device *client_dev)
{
	int ret;
	struct ntb_dev *ntb;
	size_t i;

	if (!instance) {
		ret = -ENODEV;
		goto out;
	}

	if (instance->ntb_initialized) {
		ret = -EEXIST;
		goto out;
	}

	instance->client_dev = client_dev;
	ntb = dev_ntb(client_dev->parent);

	instance->qp = ntb_transport_create_queue_by_idx(instance, client_dev, NTB_QP_ID_CACHE_PROTECTION, &ntb_handlers);
	if (!instance->qp) {
		ret = -EIO;
		goto out;
	}
	instance->mtu = ntb_transport_max_size(instance->qp);

	init_waitqueue_head(&instance->tx_wait_queue);

	/* Add some empty rx bufs */
	instance->rx_buffer_count = SYNO_CACHE_PROTECTION_NTB_CONNECTION_RX_BUFFER_MAX;
	instance->rx_buffers = kzalloc(instance->rx_buffer_count * sizeof(void *), GFP_KERNEL);
	if (!instance->rx_buffers) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < instance->rx_buffer_count; i++) {
		instance->rx_buffers[i] = kzalloc(instance->mtu, GFP_KERNEL);
		if (!instance->rx_buffers[i]) {
			ret = -ENOMEM;
			goto out;
		}

		ret = ntb_transport_rx_enqueue(instance->qp, instance->rx_buffers[i], instance->rx_buffers[i], instance->mtu);
		if (ret == -ENOMEM && i > 0) {
			// rx full
			ret = 0;
			break;
		}
		if (ret) {
			syno_cache_protection_err("Failed to ntb_transport_rx_enqueue");
			goto out;
		}
	}

	ntb_transport_link_up(instance->qp);

	// Install event handler
	instance->raw_block = ntb_transport_create_block(client_dev, NTB_RAW_BLOCK_ID_CACHE_PROTECTION, NULL);
	if (!instance->raw_block) {
		syno_cache_protection_err("Failed to ntb_transport_create_block");
		goto out;
	}
	instance->ntb_total_size = instance->raw_block->size;
	instance->ntb_used_size = 0;

	instance->ntb_initialized = true;

	ret = 0;
out:
	if (ret)
		ntb_connection_remove(client_dev);
	return ret;
}

static struct ntb_transport_client ntb_connection_client = {
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.probe = ntb_connection_probe,
	.remove = ntb_connection_remove,
};

static void ntb_connection_put_req(void *req)
{
	struct syno_cache_protection_ntb_entry *entry = (struct syno_cache_protection_ntb_entry*)req;
	struct syno_cache_protection_ntb_entry *tmp;
	size_t channel;
	struct syno_cache_protection_ntb_connection_pool *pool = NULL;
	unsigned long flags;

	if (!instance)
		return;

	if (WARN_ON_ONCE(!RB_EMPTY_NODE(&entry->entry_node))) {
		spin_lock_irqsave(&instance->request_tree_lock, flags);
		if (!RB_EMPTY_NODE(&entry->entry_node)) {
			rb_erase(&entry->entry_node, &instance->request_tree);
			RB_CLEAR_NODE(&entry->entry_node);
		}
		spin_unlock_irqrestore(&instance->request_tree_lock, flags);
	}

	channel = entry->channel;

	if (channel >= SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX) {
		syno_cache_protection_err("Failed to ERR channel [%zd]", channel);
		channel = 0;
	}

	pool = &instance->free_pool;
	spin_lock_irqsave(&pool->lock, flags);
	while (!list_empty(&entry->extra_head)) {
		tmp = list_first_entry(&entry->extra_head, struct syno_cache_protection_ntb_entry, list);
		list_move_tail(&tmp->list, &pool->head);
		pool->nr++;
	}
	list_move_tail(&entry->list, &pool->head);
	pool->nr++;
	spin_unlock_irqrestore(&pool->lock, flags);
	if (waitqueue_active(&pool->wait))
		wake_up(&pool->wait);

	atomic_dec(&instance->channel_nr[channel]);
	if (waitqueue_active(&instance->channel_queue_wait[channel]))
		wake_up(&instance->channel_queue_wait[channel]);
}

static void* ntb_connection_get_req(size_t size, size_t channel)
{
	struct syno_cache_protection_ntb_entry *entry = NULL, *tmp;
	struct syno_cache_protection_ntb_connection_pool *pool;
	wait_queue_t wait;
	int err;
	unsigned long flags;
	size_t i, total_nr, extra_nr, remain_size, index;
	bool queued = false;
	int channel_nr;

	if (!instance || !instance->enable) {
		err = -EINVAL;
		goto out;
	}

	if (channel >= SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX) {
		err = -EINVAL;
		goto out;
	}

	total_nr = 1;
	extra_nr = 0;
	if (size > SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE) {
		remain_size = size - SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE;
		extra_nr = div64_u64(remain_size + SYNO_CACHE_PROTECTION_NTB_POOL_SIZE - 1, SYNO_CACHE_PROTECTION_NTB_POOL_SIZE);
		total_nr += extra_nr;
		if (total_nr > SYNO_CACHE_PROTECTION_NTB_ENTRY_MAX_BUFFER) {
			err = -EOVERFLOW;
			goto out;
		}
	}

	channel_nr = atomic_inc_return(&instance->channel_nr[channel]);
	if (channel_nr > SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_REQUEST_MAX) {
		init_wait(&wait);
		prepare_to_wait_exclusive(&instance->channel_queue_wait[channel], &wait, TASK_UNINTERRUPTIBLE);
		schedule();
		finish_wait(&instance->channel_queue_wait[channel], &wait);
	}

	pool = &instance->free_pool;
alloc_entry:
	spin_lock_irqsave(&pool->lock, flags);
	if (pool->allocation[channel] && !queued) {
		spin_unlock_irqrestore(&pool->lock, flags);
		queued = true;
		init_wait(&wait);
		prepare_to_wait_exclusive(&pool->queued[channel], &wait, TASK_UNINTERRUPTIBLE);
		schedule();
		finish_wait(&pool->queued[channel], &wait);
		goto alloc_entry;
	}
	if (pool->nr < total_nr) {
		queued = true;
		pool->allocation[channel] = true;
		spin_unlock_irqrestore(&pool->lock, flags);
		wait_event(pool->wait, pool->nr >= total_nr);
		goto alloc_entry;
	}
	pool->nr -= total_nr;

	entry = list_first_entry(&pool->head, struct syno_cache_protection_ntb_entry, list);
	list_del_init(&entry->list);

	for (i = 0; i < extra_nr; i++) {
		tmp = list_first_entry(&pool->head, struct syno_cache_protection_ntb_entry, list);
		list_move_tail(&tmp->list, &entry->extra_head);
	}

	if (!waitqueue_active(&pool->queued[channel]))
		pool->allocation[channel] = false;
	else
		wake_up(&pool->queued[channel]);
	spin_unlock_irqrestore(&pool->lock, flags);

	entry->data_len = 0;
	entry->channel = channel;
	entry->err = 0;
	entry->wait = false;
	entry->size = SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE;
	init_completion(&entry->completion);
	entry->vec_index = 0;
	entry->vec_done = 0;
	index = 0;
	BUILD_BUG_ON(sizeof(struct syno_cache_protection_ntb_stream_entry_payload) + SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE >= SYNO_CACHE_PROTECTION_NTB_POOL_SIZE);
	entry->buffers[index].offset = SYNO_CACHE_PROTECTION_NTB_POOL_SIZE - SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE;
	entry->buffers[index].len = SYNO_CACHE_PROTECTION_NTB_POOL_INLINE_SIZE;
	entry->buffers[index].addr = entry->offset;
	index++;
	list_for_each_entry(tmp, &entry->extra_head, list) {
		entry->buffers[index].offset = 0;
		entry->buffers[index].len = SYNO_CACHE_PROTECTION_NTB_POOL_SIZE;
		entry->buffers[index].addr = tmp->offset;
		entry->size += tmp->init_size;
		index++;
	}
	entry->buffer_count = index;
	return entry;

out:
	return ERR_PTR(err);
}

static void syno_cache_protection_ntb_receiver_do_work(struct work_struct *normal_work)
{
	struct syno_cache_protection_ntb_rx_work *work;
	struct syno_cache_protection_ntb_stream_entry_header response;
	int err;
	unsigned long flags;
	struct syno_cache_protection_ntb_entry *entry = NULL;
	struct syno_cache_protection_ntb_stream_entry_payload *payload = NULL;
	size_t i, uppersize;

	if (!instance || !instance->ntb_initialized)
		return;

	work = container_of(normal_work, struct syno_cache_protection_ntb_rx_work, normal_work);

	if (!instance->enable) {
		err = -ECONNRESET;
		goto out;
	}

	payload = &work->payload;
	uppersize = offsetof(struct syno_cache_protection_ntb_stream_entry_payload, buffers);
	err = copy_from_local(payload, le64_to_cpu(work->header.offset), uppersize);
	if (err)
		goto out;
	err = copy_from_local(((char*)payload) + uppersize, le64_to_cpu(work->header.offset) + uppersize,
						sizeof(struct syno_cache_protection_ntb_stream_buffer) * le32_to_cpu(payload->buffer_count));
	if (err)
		goto out;

	entry = &work->entry;
	entry->data_len = 0;
	entry->size = le32_to_cpu(work->header.data_len);
	entry->vec_index = 0;
	entry->vec_done = 0;
	entry->buffer_count = le32_to_cpu(payload->buffer_count);
	for (i = 0; i < entry->buffer_count; i++) {
		entry->buffers[i].offset = le32_to_cpu(payload->buffers[i].offset);
		entry->buffers[i].len = le32_to_cpu(payload->buffers[i].len);
		entry->buffers[i].addr = le64_to_cpu(payload->buffers[i].addr);
	}
	err = syno_cache_protection_do_request(entry);

out:
	memset(&response, 0, sizeof(response));
	response.magic = cpu_to_le64(SYNO_CACHE_PROTECTION_NTB_MAGIC);
	response.response = cpu_to_le32(1);
	response.err = cpu_to_le32(err);
	response.offset = work->header.offset;
	err = ntb_start_xmit_with_response(&response, sizeof(response));
	if (err)
		syno_cache_protection_err("Failed to response ntb_start_xmit with offset:%llu err:%d", le64_to_cpu(work->header.offset), err);

	spin_lock_irqsave(&instance->rx_work_pool.lock, flags);
	list_move_tail(&work->list, &instance->rx_work_pool.head);
	spin_unlock_irqrestore(&instance->rx_work_pool.lock, flags);
}

static int ntb_connection_write_req(void *req, size_t len, const void *data)
{
	int ret;
	struct syno_cache_protection_ntb_entry *entry = (struct syno_cache_protection_ntb_entry*)req;
	char *src = (char *)data;
	size_t cur, remain_len = len;

	if (!instance || !instance->enable) {
		ret = -EINVAL;
		goto out;
	}

	if (!entry || (len && !data) || (entry->data_len + len > entry->size)) {
		ret = -EINVAL;
		syno_cache_protection_err("Failed to ntb connection write req with entry[exist %d size %zd data_len %zd] len %zd", entry ? 1 : 0, entry ? entry->size : 0, entry ? entry->data_len : 0, len);
		goto out;
	}

	while (remain_len > 0) {
		if (entry->vec_index >= entry->buffer_count) {
			ret = -EOVERFLOW;
			goto out;
		}

		cur = min(remain_len, entry->buffers[entry->vec_index].len - entry->vec_done);

		ret = copy_to_remote(entry->buffers[entry->vec_index].addr + entry->buffers[entry->vec_index].offset + entry->vec_done, src, cur);
		if (ret)
			goto out;

		entry->vec_done += cur;
		if (entry->buffers[entry->vec_index].len == entry->vec_done) {
			entry->vec_done = 0;
			entry->vec_index++;
		}

		src += cur;
		remain_len -= cur;
	}
	entry->data_len += len;

	ret = 0;
out:
	return ret;
}

static int ntb_connection_read_req(void *req, size_t len, void *data)
{
	int ret;
	struct syno_cache_protection_ntb_entry *entry = (struct syno_cache_protection_ntb_entry*)req;
	char *dst = (char *)data;
	size_t cur, remain_len = len;

	if (!instance || !instance->enable) {
		ret = -EINVAL;
		goto out;
	}

	if (!entry || (len && !data) || (entry->data_len + len > entry->size)) {
		ret = -EINVAL;
		syno_cache_protection_err("Failed to ntb connection read req with entry[exist %d size %zd data_len %zd] len %zd", entry ? 1 : 0, entry ? entry->size : 0, entry ? entry->data_len : 0, len);
		goto out;
	}

	while (remain_len > 0) {
		if (entry->vec_index >= entry->buffer_count) {
			ret = -EOVERFLOW;
			goto out;
		}

		cur = min(remain_len, entry->buffers[entry->vec_index].len - entry->vec_done);

		ret = copy_from_local(dst, entry->buffers[entry->vec_index].addr + entry->buffers[entry->vec_index].offset + entry->vec_done, cur);
		if (ret)
			goto out;

		entry->vec_done += cur;
		if (entry->buffers[entry->vec_index].len == entry->vec_done) {
			entry->vec_done = 0;
			entry->vec_index++;
		}

		dst += cur;
		remain_len -= cur;
	}
	entry->data_len += len;

	ret = 0;
out:
	return ret;
}

static int ntb_connection_send_req(void *req, bool wait)
{
	int ret = 0;
	struct syno_cache_protection_ntb_entry *entry = (struct syno_cache_protection_ntb_entry*)req;
	struct syno_cache_protection_ntb_stream_entry_header header;
	struct syno_cache_protection_ntb_stream_entry_payload *payload = NULL;
	size_t i, payload_size;

	if (!instance || !instance->enable) {
		ret = -EINVAL;
		goto out;
	}

	if (!entry) {
		ret = -EINVAL;
		goto out;
	}

	payload = &entry->payload;
	payload->buffer_count = cpu_to_le32(entry->buffer_count);
	for (i = 0; i < entry->buffer_count; i++) {
		payload->buffers[i].offset = cpu_to_le32(entry->buffers[i].offset);
		payload->buffers[i].len = cpu_to_le32(entry->buffers[i].len);
		payload->buffers[i].addr = cpu_to_le64(entry->buffers[i].addr);
	}
	payload_size = offsetof(struct syno_cache_protection_ntb_stream_entry_payload, buffers) + sizeof(struct syno_cache_protection_ntb_stream_buffer) * entry->buffer_count;
	ret = copy_to_remote(entry->offset, payload, payload_size);
	if (ret)
		goto out;

	entry->wait = wait;
	ret = syno_cache_protection_ntb_connection_request_tree_insert(entry);
	if (ret)
		goto out;

	memset(&header, 0, sizeof(header));
	header.magic = cpu_to_le64(SYNO_CACHE_PROTECTION_NTB_MAGIC);
	header.response = cpu_to_le32(0);
	header.channel = cpu_to_le32(entry->channel);
	header.data_len = cpu_to_le32(entry->data_len);
	header.offset = cpu_to_le64(entry->offset);
	ret = ntb_start_xmit(&header, sizeof(header));
	if (ret) {
		syno_cache_protection_ntb_connection_request_tree_remove(entry);
		if (ret != -ENOTCONN)
			syno_cache_protection_err("Failed to ntb_start_xmit with offset:%llu, err:%d", entry->offset, ret);
		goto out;
	}

	if (wait) {
		wait_for_completion(&entry->completion);
		ret = entry->err;
		if (!ret)
			ntb_connection_put_req(entry);
	}

out:
	return ret;
}

const struct syno_cache_protection_connection_operations* syno_cache_protection_ntb_connection_get_connections(void)
{
	if (!instance || !instance->enable)
		return NULL;
	return instance->c_op;
}
EXPORT_SYMBOL(syno_cache_protection_ntb_connection_get_connections);

static int ntb_connection_status(char* buf)
{
	int len = 0;
	size_t i;

	if (!instance)
		goto out;

	len += snprintf(buf + len, PAGE_SIZE - len, "ntb_qp_link_is_up:%d\n", instance->ntb_qp_link_is_up ? 1 : 0);
	if (instance->raw_block)
		len += snprintf(buf + len, PAGE_SIZE - len, "ntb_raw_block_link_is_up:%d\n", instance->raw_block->link_is_up ? 1 : 0);
	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "channel_nr[%zd]:%d\n", i, atomic_read(&instance->channel_nr[i]));
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "response_pending_nr:%d\n", atomic_read(&instance->response_pending_nr));

out:
	return len;
}

static const struct syno_cache_protection_connection_operations ntb_connection_ops = {
	.get_req = ntb_connection_get_req,
	.put_req = ntb_connection_put_req,
	.write_req = ntb_connection_write_req,
	.read_req = ntb_connection_read_req,
	.send_req = ntb_connection_send_req,
	.status = ntb_connection_status,
};

static struct syno_cache_protection_ntb_rx_work *connection_pool_work_list_rm(struct syno_cache_protection_ntb_connection_pool *pool)
{
	struct syno_cache_protection_ntb_rx_work *work = NULL;
	unsigned long flags;

	if (!pool)
		goto out;

	spin_lock_irqsave(&pool->lock, flags);
	if (list_empty(&pool->head)) {
		spin_unlock_irqrestore(&pool->lock, flags);
		goto out;
	}
	work = list_first_entry(&pool->head, struct syno_cache_protection_ntb_rx_work, list);
	list_del_init(&work->list);
	spin_unlock_irqrestore(&pool->lock, flags);

out:
	return work;
}

static struct syno_cache_protection_ntb_entry *connection_pool_list_rm(struct syno_cache_protection_ntb_connection_pool *pool)
{
	struct syno_cache_protection_ntb_entry *entry = NULL;
	unsigned long flags;

	if (!pool)
		goto out;

	spin_lock_irqsave(&pool->lock, flags);
	if (list_empty(&pool->head)) {
		spin_unlock_irqrestore(&pool->lock, flags);
		goto out;
	}
	entry = list_first_entry(&pool->head, struct syno_cache_protection_ntb_entry, list);
	list_del(&entry->list);
	spin_unlock_irqrestore(&pool->lock, flags);

out:
	return entry;
}

void syno_cache_protection_ntb_connection_exit(void)
{
	size_t i;
	struct syno_cache_protection_ntb_entry *entry;
	struct syno_cache_protection_ntb_rx_work *work;

	if (!instance)
		return;

	instance->enable = false;

	cancel_work_sync(&instance->link_down_work);

	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX; i++) {
		if (instance->channel_workqueue[i])
			drain_workqueue(instance->channel_workqueue[i]);
	}

	while (NULL != (work = connection_pool_work_list_rm(&instance->rx_work_pool)))
		kfree(work);

	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX; i++) {
		if (instance->channel_workqueue[i])
			destroy_workqueue(instance->channel_workqueue[i]);
	}

	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX; i++) {
		if (atomic_read(&instance->channel_nr[i]) > 0) {
			wait_event(instance->channel_queue_wait[i], atomic_read(&instance->channel_nr[i]) == 0);
		}
	}

	while (NULL != (entry = connection_pool_list_rm(&instance->free_pool)))
		kfree(entry);

	ntb_transport_unregister_client(&ntb_connection_client);
	ntb_transport_unregister_client_dev(KBUILD_MODNAME);

	kfree(instance);
	instance = NULL;
}
EXPORT_SYMBOL(syno_cache_protection_ntb_connection_exit);

int __init syno_cache_protection_ntb_connection_init(void)
{
	int ret;
	size_t i;
	struct syno_cache_protection_ntb_entry *entry;
	struct syno_cache_protection_ntb_rx_work *work;
	unsigned long flags;

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance) {
		ret = -ENOMEM;
		goto out;
	}
	instance->c_op = &ntb_connection_ops;
	instance->request_tree = RB_ROOT;
	spin_lock_init(&instance->request_tree_lock);

	INIT_WORK(&instance->link_down_work, ntb_connection_entry_wake_up_all);

	syno_cache_protection_ntb_connection_pool_init(&instance->free_pool);
	syno_cache_protection_ntb_connection_pool_init(&instance->rx_work_pool);

	ret = ntb_transport_register_client_dev(KBUILD_MODNAME);
	if (ret)
		goto out;

	ret = ntb_transport_register_client(&ntb_connection_client);
	if (ret)
		goto out;

	if (!instance->ntb_initialized) {
		ret = -EIO;
		goto out;
	}

	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_POOL_NR; i++) {
		if (instance->ntb_used_size + SYNO_CACHE_PROTECTION_NTB_POOL_SIZE > instance->ntb_total_size) {
			if (i < SYNO_CACHE_PROTECTION_NTB_POOL_MIN_NR) {
				ret = -EOVERFLOW;
				goto out;
			}
			break;
		}
		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if (!entry)
			goto out;
		INIT_LIST_HEAD(&entry->list);
		INIT_LIST_HEAD(&entry->extra_head);
		RB_CLEAR_NODE(&entry->entry_node);
		entry->offset = instance->ntb_used_size;
		entry->init_size = SYNO_CACHE_PROTECTION_NTB_POOL_SIZE;
		instance->ntb_used_size += entry->init_size;
		spin_lock_irqsave(&instance->free_pool.lock, flags);
		list_add_tail(&entry->list, &instance->free_pool.head);
		instance->free_pool.nr++;
		spin_unlock_irqrestore(&instance->free_pool.lock, flags);
	}

	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_CHANNEL_MAX; i++) {
		instance->channel_workqueue[i] = alloc_workqueue("syno-%zd-ntb-channel", WQ_UNBOUND, 0, i);
		if (!instance->channel_workqueue[i]) {
			ret = -ENOMEM;
			goto out;
		}
		init_waitqueue_head(&instance->channel_queue_wait[i]);
		atomic_set(&instance->channel_nr[i], 0);
	}

	for (i = 0; i < SYNO_CACHE_PROTECTION_NTB_CONNECTION_REQUEST_MAX; i++) {
		work = kzalloc(sizeof(*work), GFP_KERNEL);
		if (!work)
			goto out;
		INIT_LIST_HEAD(&work->list);
		INIT_WORK(&work->normal_work, syno_cache_protection_ntb_receiver_do_work);
		spin_lock_irqsave(&instance->rx_work_pool.lock, flags);
		list_add_tail(&work->list, &instance->rx_work_pool.head);
		spin_unlock_irqrestore(&instance->rx_work_pool.lock, flags);
	}

	atomic_set(&instance->response_pending_nr, 0);

	instance->enable = true;

	ret = 0;
out:
	if (ret)
		syno_cache_protection_ntb_connection_exit();
	return ret;
}
EXPORT_SYMBOL(syno_cache_protection_ntb_connection_init);

