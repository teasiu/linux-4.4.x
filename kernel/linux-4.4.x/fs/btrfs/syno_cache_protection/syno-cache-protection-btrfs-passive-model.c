/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/bug.h>
#include "../ctree.h"
#include "syno-cache-protection-btrfs-passive-model.h"
#include "syno-cache-protection-btrfs.h"

int syno_cache_protection_passive_btrfs_buffer_insert(struct syno_cache_protection_passive_btrfs_buffers *buffers, enum SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_TYPE type, void *data)
{
	if (!buffers)
		return -EINVAL;
	if (buffers->count >= SYNO_CACHE_PROTECTION_COMMAND_EXTRA_BUFFER)
		return -EOVERFLOW;
	buffers->buffer[buffers->count].type = type;
	buffers->buffer[buffers->count].data = data;
	buffers->count++;
	return 0;
}

struct syno_cache_protection_passive_btrfs_virtual_buffer* syno_cache_protection_passive_btrfs_virtual_buffer_alloc(u64 len, bool reserved, enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type)
{
	int err;
	size_t i, j, count;
	u64 target_len;
	struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer = NULL;

	target_len = round_up(len, SYNO_CACHE_PROTECTION_DATA_SIZE);
	count = target_len / SYNO_CACHE_PROTECTION_DATA_SIZE;

	if ((pool_type != SYNO_CACHE_PROTECTION_SPACE_POOL_DATA && pool_type != SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM) ||
		pool_type >= SYNO_CACHE_PROTECTION_SPACE_POOL_MAX) {
		err = -EINVAL;
		goto out;
	}

	if (count > SYNO_CACHE_PROTECTION_VIRTUAL_BUFFER_MAX_PAGES) {
		err = -EOVERFLOW;
		goto out;
	}

	BUILD_BUG_ON(sizeof(*virtual_buffer) > SYNO_CACHE_PROTECTION_DATA_SIZE);
	virtual_buffer = syno_cache_protection_space_alloc(pool_type, GFP_NOFS, reserved);
	if (!virtual_buffer) {
		err = -ENOSPC;
		goto out;
	}

	memset(virtual_buffer, 0, sizeof(*virtual_buffer));
	virtual_buffer->pool_type = pool_type;
	for (i = 0; i < count; i++) {
		virtual_buffer->pages[i] = syno_cache_protection_space_alloc(pool_type, GFP_NOFS, reserved);
		if (!virtual_buffer->pages[i]) {
			err = -ENOSPC;
			goto out;
		}
	}
	virtual_buffer->count = count;
	virtual_buffer->size = count * SYNO_CACHE_PROTECTION_DATA_SIZE;

	return virtual_buffer;

out:
	if (virtual_buffer) {
		for (j = 0; j < i; j++) {
			syno_cache_protection_space_free(pool_type, virtual_buffer->pages[j]);
			virtual_buffer->pages[j] = NULL;
		}
		syno_cache_protection_space_free(pool_type, virtual_buffer);
	}
	return ERR_PTR(err);
}

int syno_cache_protection_passive_btrfs_virtual_buffer_write(struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer, u64 pos, u64 len, const char *srcv)
{
	int ret;
	char *page;
	size_t cur;
	size_t offset;
	char *src = (char *)srcv;
	unsigned long index = pos >> SYNO_CACHE_PROTECTION_DATA_SHIFT;

	if (pos + len > virtual_buffer->size) {
		ret = -EOVERFLOW;
		goto out;
	}

	offset = (pos) & (SYNO_CACHE_PROTECTION_DATA_SIZE - 1);
	while (len > 0) {
		page = virtual_buffer->pages[index];
		cur = min(len, (u64)SYNO_CACHE_PROTECTION_DATA_SIZE - offset);
		memcpy(((char*)page) + offset, src, cur);
		src += cur;
		len -= cur;
		offset = 0;
		index++;
	}

	ret = 0;
out:
	return ret;
}

int syno_cache_protection_passive_btrfs_virtual_buffer_read(struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer, u64 pos, u64 len, void *dstv)
{
	int ret;
	size_t cur;
	size_t offset;
	struct page *page;
	char *dst = (char *)dstv;
	unsigned long index = pos >> SYNO_CACHE_PROTECTION_DATA_SHIFT;

	if (pos + len > virtual_buffer->size) {
		ret = -EOVERFLOW;
		goto out;
	}

	offset = (pos) & (SYNO_CACHE_PROTECTION_DATA_SIZE - 1);
	while (len > 0) {
		page = virtual_buffer->pages[index];
		cur = min(len, (u64)SYNO_CACHE_PROTECTION_DATA_SIZE - offset);
		memcpy(dst, ((char*)page) + offset, cur);
		dst += cur;
		len -= cur;
		offset = 0;
		index++;
	}

	ret = 0;
out:
	return ret;
}

void syno_cache_protection_passive_btrfs_virtual_buffer_free(struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer)
{
	size_t i;
	enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE pool_type;

	if (!virtual_buffer)
		return;

	pool_type = virtual_buffer->pool_type;
	for (i = 0; i < virtual_buffer->count; i++) {
		syno_cache_protection_space_free(pool_type, virtual_buffer->pages[i]);
		virtual_buffer->pages[i] = NULL;
	}
	syno_cache_protection_space_free(pool_type, virtual_buffer);
}

int syno_cache_protection_passive_btrfs_virtual_buffer_fill_from_request(void *req, struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer, size_t pos, size_t len)
{
	int ret;
	char szBuf[256];
	size_t cbBuf, remain_len, tmp_len, tmp_pos;

	cbBuf = sizeof(szBuf);
	remain_len = len;
	tmp_pos = pos;
	while (remain_len) {
		tmp_len = min(cbBuf, remain_len);
		ret = syno_cache_protection_read_request(req, tmp_len, szBuf);
		if (ret)
			goto out;
		ret = syno_cache_protection_passive_btrfs_virtual_buffer_write(virtual_buffer, tmp_pos, tmp_len, szBuf);
		if (ret)
			goto out;
		tmp_pos += tmp_len;
		remain_len -= tmp_len;
	}

	ret = 0;
out:
	return ret;
}

static inline int page_in_tree(const struct syno_cache_protection_passive_btrfs_page *page)
{
	return !RB_EMPTY_NODE(&page->page_node);
}

void syno_cache_protection_passive_btrfs_page_free(struct syno_cache_protection_passive_btrfs_page *page)
{
	if (!page)
		return;

	WARN_ON(atomic_read(&page->refs) == 0);
	if (atomic_dec_and_test(&page->refs)) {
		WARN_ON_ONCE(page_in_tree(page));
		/* remove page from lru */
		if (!list_empty(&page->lru_list) && page->instance) {
			spin_lock(&page->instance->lock);
			list_del_init(&page->lru_list);
			spin_unlock(&page->instance->lock);
		}
		syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_DATA, page->value);
		syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER, page);
	}
}

struct syno_cache_protection_passive_btrfs_page *syno_cache_protection_passive_btrfs_page_tree_search(struct rb_root *root,
					  u64 pg_offset)
{
	struct rb_node *n;
	struct syno_cache_protection_passive_btrfs_page *page;

	if (!root)
		return NULL;

	n = root->rb_node;

	while (n) {
		page = rb_entry(n, struct syno_cache_protection_passive_btrfs_page, page_node);

		if (pg_offset < page->pg_offset)
			n = n->rb_left;
		else if (pg_offset > page->pg_offset)
			n = n->rb_right;
		else
			return page;
	}

	return NULL;
}

struct syno_cache_protection_passive_btrfs_page *syno_cache_protection_passive_btrfs_page_tree_search_with_range(struct rb_root *root,
					  u64 pg_start, u64 pg_end)
{
	struct rb_node *n;
	struct rb_node *prev = NULL;
	struct syno_cache_protection_passive_btrfs_page *page;
	struct syno_cache_protection_passive_btrfs_page *prev_page = NULL;

	if (!root)
		return NULL;

	if (pg_start > pg_end)
		return NULL;

	n = root->rb_node;

	while (n) {
		prev = n;
		page = rb_entry(n, struct syno_cache_protection_passive_btrfs_page, page_node);
		prev_page = page;

		if (pg_start < page->pg_offset)
			n = n->rb_left;
		else if (pg_start > page->pg_offset)
			n = n->rb_right;
		else
			return page;
	}

	while (prev && prev_page->pg_offset < pg_start) {
		prev = rb_next(prev);
		prev_page = rb_entry(prev, struct syno_cache_protection_passive_btrfs_page, page_node);
	}
	if (prev && prev_page->pg_offset > pg_end)
		prev_page = NULL;

	return prev_page;
}

static struct syno_cache_protection_passive_btrfs_page* syno_cache_protection_passive_btrfs_page_alloc(u64 pg_offset, bool reserved)
{
	void *value = NULL;
	struct syno_cache_protection_passive_btrfs_page *page = NULL;

	value = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_DATA, GFP_NOFS, reserved);
	if (!value)
		goto out;

	BUILD_BUG_ON(sizeof(*page) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	page = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER, GFP_NOFS, reserved);
	if (!page)
		goto out;
	memset(page, 0, sizeof(*page));
	RB_CLEAR_NODE(&page->page_node);
	page->pg_offset = pg_offset;
	atomic_set(&page->refs, 1);
	page->value = value;
	/* for page lru */
	INIT_LIST_HEAD(&page->lru_list);
	atomic64_set(&page->version, 0);

out:
	if (!page)
		syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_DATA, value);
	return page;
}

static int syno_cache_protection_passive_btrfs_page_tree_insert(struct rb_root *root, struct syno_cache_protection_passive_btrfs_page *page)
{
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct syno_cache_protection_passive_btrfs_page *entry;

	if (!root || !page)
		return -EINVAL;

	p = &root->rb_node;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct syno_cache_protection_passive_btrfs_page, page_node);

		if (page->pg_offset < entry->pg_offset)
			p = &(*p)->rb_left;
		else if (page->pg_offset > entry->pg_offset)
			p = &(*p)->rb_right;
		else
			return -EEXIST;
	}

	rb_link_node(&page->page_node, parent, p);
	rb_insert_color(&page->page_node, root);
	return 0;
}

struct syno_cache_protection_passive_btrfs_page* syno_cache_protection_passive_btrfs_get_or_alloc_page(
						struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_passive_btrfs_inode *inode,
						u64 pg_offset, bool reserved, bool *new_alloc)
{
	struct syno_cache_protection_passive_btrfs_page *page;
	int err;

	if (!inode)
		return NULL;

	*new_alloc = false;
	spin_lock(&inode->lock);
again:
	page = syno_cache_protection_passive_btrfs_page_tree_search(&inode->page_tree, pg_offset);
	if (page)
		goto found;
	spin_unlock(&inode->lock);

	page = syno_cache_protection_passive_btrfs_page_alloc(pg_offset, reserved);
	if (!page)
		goto out;
	/* for page lru */
	page->instance = passive_instance;
	page->inode = inode;

	spin_lock(&inode->lock);
	err = syno_cache_protection_passive_btrfs_page_tree_insert(&inode->page_tree, page);
	if (err) {
		spin_unlock(&inode->lock);
		syno_cache_protection_passive_btrfs_page_free(page);
		page = NULL;
		if (need_resched())
			cond_resched();
		spin_lock(&inode->lock);
		if (err == -EEXIST) {
			goto again;
		} else {
			spin_unlock(&inode->lock);
			BUG();
			goto out;
		}
	}
	*new_alloc = true;

found:
	atomic_inc(&page->refs);
	spin_unlock(&inode->lock);

	/* add page to lru */
	spin_lock(&passive_instance->lock);
	atomic64_set(&page->version, atomic64_read(&passive_instance->reclaim_version) + 1);
	list_move_tail(&page->lru_list, &passive_instance->lru_page_head);
	spin_unlock(&passive_instance->lock);

out:
	return page;
}

/*
 * compare two keys in a memcmp fashion
 */
int syno_cache_protection_passive_btrfs_inode_comp(struct syno_cache_protection_passive_btrfs_inode *i1, struct syno_cache_protection_passive_btrfs_inode *i2)
{
	if (i1->subvolid > i2->subvolid)
		return 1;
	if (i1->subvolid < i2->subvolid)
		return -1;
	if (i1->inum > i2->inum)
		return 1;
	if (i1->inum < i2->inum)
		return -1;
	return 0;
}

static struct syno_cache_protection_passive_btrfs_inode *syno_cache_protection_passive_btrfs_inode_tree_search(struct rb_root *root,
					  u64 subvolid, u64 inum)
{
	struct rb_node *n;
	struct syno_cache_protection_passive_btrfs_inode *inode;
	struct syno_cache_protection_passive_btrfs_inode comp;
	int cmp;

	if (!root)
		return NULL;

	n = root->rb_node;
	comp.subvolid = subvolid;
	comp.inum = inum;

	while (n) {
		inode = rb_entry(n, struct syno_cache_protection_passive_btrfs_inode, inode_node);

		cmp = syno_cache_protection_passive_btrfs_inode_comp(&comp, inode);
		if (cmp < 0)
			n = n->rb_left;
		else if (cmp > 0)
			n = n->rb_right;
		else
			return inode;
	}

	return NULL;
}

static struct syno_cache_protection_passive_btrfs_inode* syno_cache_protection_passive_btrfs_inode_alloc(u64 subvolid, u64 inum, bool reserved)
{
	struct syno_cache_protection_passive_btrfs_inode *inode = NULL;

	BUILD_BUG_ON(sizeof(*inode) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	inode = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!inode)
		goto out;
	memset(inode, 0, sizeof(*inode));
	RB_CLEAR_NODE(&inode->inode_node);
	inode->subvolid = subvolid;
	inode->inum = inum;
	inode->page_tree = RB_ROOT;
	spin_lock_init(&inode->lock);
	atomic_set(&inode->refs, 1);

out:
	return inode;
}

static int syno_cache_protection_passive_btrfs_inode_tree_insert(struct rb_root *root, struct syno_cache_protection_passive_btrfs_inode *inode)
{
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct syno_cache_protection_passive_btrfs_inode *entry;
	int cmp;

	if (!root || !inode)
		return -EINVAL;

	p = &root->rb_node;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct syno_cache_protection_passive_btrfs_inode, inode_node);

		cmp = syno_cache_protection_passive_btrfs_inode_comp(inode, entry);
		if (cmp < 0)
			p = &(*p)->rb_left;
		else if (cmp > 0)
			p = &(*p)->rb_right;
		else
			return -EEXIST;
	}

	rb_link_node(&inode->inode_node, parent, p);
	rb_insert_color(&inode->inode_node, root);
	return 0;
}

struct syno_cache_protection_passive_btrfs_inode* syno_cache_protection_passive_btrfs_get_or_alloc_inode(
						struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						u64 subvolid, u64 inum, bool create, bool reserved)
{
	struct syno_cache_protection_passive_btrfs_inode *inode;
	int err;

	if (!passive_instance)
		return NULL;

	spin_lock(&passive_instance->lock);
again:
	inode = syno_cache_protection_passive_btrfs_inode_tree_search(&passive_instance->inode_tree, subvolid, inum);
	if (inode)
		goto found;

	if (!create)
		goto out_lock;
	spin_unlock(&passive_instance->lock);

	inode = syno_cache_protection_passive_btrfs_inode_alloc(subvolid, inum, reserved);
	if (!inode)
		goto out;

	spin_lock(&passive_instance->lock);
	err = syno_cache_protection_passive_btrfs_inode_tree_insert(&passive_instance->inode_tree, inode);
	if (err) {
		spin_unlock(&passive_instance->lock);
		syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
		inode = NULL;
		if (need_resched())
			cond_resched();
		spin_lock(&passive_instance->lock);
		if (err == -EEXIST) {
			goto again;
		} else {
			spin_unlock(&passive_instance->lock);
			BUG();
			goto out;
		}
	}

found:
	atomic_inc(&inode->refs);
out_lock:
	spin_unlock(&passive_instance->lock);
out:
	return inode;
}

static inline int inode_in_tree(const struct syno_cache_protection_passive_btrfs_inode *inode)
{
	return !RB_EMPTY_NODE(&inode->inode_node);
}

void syno_cache_protection_passive_btrfs_inode_free(struct syno_cache_protection_passive_btrfs_instance *passive_instance, struct syno_cache_protection_passive_btrfs_inode *inode)
{
	struct rb_node *node;
	struct syno_cache_protection_passive_btrfs_page *page;

	if (!inode)
		return;

	if (atomic_read(&inode->refs) == 2 && RB_EMPTY_ROOT(&inode->page_tree) && inode_in_tree(inode)) {
		spin_lock(&passive_instance->lock);
		spin_lock(&inode->lock);
		if (atomic_read(&inode->refs) == 2 && RB_EMPTY_ROOT(&inode->page_tree) && inode_in_tree(inode)) {
			rb_erase(&inode->inode_node, &passive_instance->inode_tree);
			RB_CLEAR_NODE(&inode->inode_node);
			atomic_dec(&inode->refs);
		}
		spin_unlock(&inode->lock);
		spin_unlock(&passive_instance->lock);
	}

	WARN_ON(atomic_read(&inode->refs) == 0);
	if (atomic_dec_and_test(&inode->refs)) {
		WARN_ON_ONCE(inode_in_tree(inode));
		spin_lock(&inode->lock);
		while (!RB_EMPTY_ROOT(&inode->page_tree)) {
			node = rb_first(&inode->page_tree);
			page = rb_entry(node, struct syno_cache_protection_passive_btrfs_page, page_node);
			rb_erase(node, &inode->page_tree);
			RB_CLEAR_NODE(node);
			spin_unlock(&inode->lock);
			syno_cache_protection_passive_btrfs_page_free(page);
			if (need_resched())
				cond_resched();
			spin_lock(&inode->lock);
		}
		spin_unlock(&inode->lock);
		syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, inode);
	}
}

void syno_cache_protection_passive_btrfs_metadata_command_free(struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command)
{
	size_t i;

	for (i = 0; i < metadata_command->extra_buffers.count; i++) {
		switch (metadata_command->extra_buffers.buffer[i].type) {
			case SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_METADATA:
				syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, metadata_command->extra_buffers.buffer[i].data);
				break;
			case SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_DATA:
				syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_DATA, metadata_command->extra_buffers.buffer[i].data);
				break;
			case SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_VIRTUAL_BUFFER:
				syno_cache_protection_passive_btrfs_virtual_buffer_free((struct syno_cache_protection_passive_btrfs_virtual_buffer*)metadata_command->extra_buffers.buffer[i].data);
				break;
			default:
				BUG();
		}
	}
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, metadata_command);
}

struct syno_cache_protection_passive_btrfs_ordered_extent* syno_cache_protection_passive_btrfs_ordered_extent_alloc(u64 transid, u64 subvolid, u64 inum, u64 file_offset,
		u64 start, u64 len, u64 disk_len, u64 truncated_len, u64 flags, u32 compress_type, u64 i_size, u32 total_csums, u32 total_csum_size, bool reserved)
{
	struct syno_cache_protection_passive_btrfs_ordered_extent *ordered_extent = NULL;

	BUILD_BUG_ON(sizeof(*ordered_extent) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	ordered_extent = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!ordered_extent)
		goto out;
	memset(ordered_extent, 0, sizeof(*ordered_extent));
	INIT_LIST_HEAD(&ordered_extent->node.list);
	ordered_extent->node.command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT;
	ordered_extent->node.transid = transid;
	ordered_extent->subvolid = subvolid;
	ordered_extent->inum = inum;
	ordered_extent->file_offset = file_offset;
	ordered_extent->start = start;
	ordered_extent->len = len;
	ordered_extent->disk_len = disk_len;
	ordered_extent->truncated_len = truncated_len;
	ordered_extent->flags = flags;
	ordered_extent->compress_type = compress_type;
	ordered_extent->i_size = i_size;
	ordered_extent->total_csums = total_csums;
	ordered_extent->total_csum_size = total_csum_size;

out:
	return ordered_extent;
}

struct syno_cache_protection_passive_btrfs_inline_extent* syno_cache_protection_passive_btrfs_inline_extent_alloc(u64 transid, u64 subvolid, u64 inum,
		u64 inline_len, bool reserved)
{
	struct syno_cache_protection_passive_btrfs_inline_extent *inline_extent = NULL;

	BUILD_BUG_ON(sizeof(*inline_extent) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	inline_extent = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!inline_extent)
		goto out;
	memset(inline_extent, 0, sizeof(*inline_extent));
	INIT_LIST_HEAD(&inline_extent->node.list);
	inline_extent->node.command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INLINE_EXTENT;
	inline_extent->node.transid = transid;
	inline_extent->subvolid = subvolid;
	inline_extent->inum = inum;
	inline_extent->inline_len = inline_len;

out:
	return inline_extent;
}


struct syno_cache_protection_passive_btrfs_create* syno_cache_protection_passive_btrfs_create_alloc(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
		u64 subvolid, u64 dir, u64 inum, u64 generation, u64 mode, u64 rdev, u64 name_len, void *name, bool reserved)
{
	int err;
	void *external_name = NULL;
	struct syno_cache_protection_passive_btrfs_create *create = NULL;

	if (name_len > BTRFS_NAME_LEN) {
		err = -ENAMETOOLONG;
		goto out;
	}

	BUILD_BUG_ON(sizeof(*create) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	BUILD_BUG_ON(BTRFS_NAME_LEN >= SYNO_CACHE_PROTECTION_METADATA_SIZE);
	create = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!create) {
		err = -ENOSPC;
		goto out;
	}
	memset(create, 0, sizeof(*create));
	INIT_LIST_HEAD(&create->node.list);
	create->node.command = command;
	create->node.transid = transid;
	create->subvolid = subvolid;
	create->dir = dir;
	create->inum = inum;
	create->generation = generation;
	create->mode = mode;
	create->rdev = rdev;
	create->name_len = name_len;
	create->iname[SYNO_CACHE_PROTECTION_NAME_INLINE_LEN - 1] = '\0';

	if (name_len > SYNO_CACHE_PROTECTION_NAME_INLINE_LEN - 1) {
		external_name = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
		if (!external_name) {
			err = -ENOSPC;
			goto out;
		}
		create->name = external_name;
		err = syno_cache_protection_passive_btrfs_buffer_insert(&create->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_METADATA, external_name);
		if (err)
			goto out;
	} else {
		create->name = create->iname;
	}
	memcpy(create->name, name, name_len);
	create->name[name_len] = '\0';

	return create;

out:
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, external_name);
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, create);
	return ERR_PTR(err);
}

struct syno_cache_protection_passive_btrfs_inode_operation* syno_cache_protection_passive_btrfs_inode_operation_alloc(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
		u64 subvolid, u64 inum, u64 flags, u64 mode, u32 uid, u32 gid, struct btrfs_timespec *times, u64 offset, u64 length, bool reserved)
{
	int err;
	struct syno_cache_protection_passive_btrfs_inode_operation *inode_operation = NULL;

	BUILD_BUG_ON(sizeof(*inode_operation) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	inode_operation = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!inode_operation) {
		err = -ENOSPC;
		goto out;
	}
	memset(inode_operation, 0, sizeof(*inode_operation));
	INIT_LIST_HEAD(&inode_operation->node.list);
	inode_operation->node.command = command;
	inode_operation->node.transid = transid;
	inode_operation->subvolid = subvolid;
	inode_operation->inum = inum;
	inode_operation->flags = flags;
	inode_operation->mode = mode;
	inode_operation->uid = uid;
	inode_operation->gid = gid;

	inode_operation->times[0].tv_sec = le64_to_cpu(times[0].sec);
	inode_operation->times[0].tv_nsec = le32_to_cpu(times[0].nsec);
	inode_operation->times[1].tv_sec = le64_to_cpu(times[1].sec);
	inode_operation->times[1].tv_nsec = le32_to_cpu(times[1].nsec);

	inode_operation->offset = offset;
	inode_operation->length = length;

	return inode_operation;

out:
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, inode_operation);
	return ERR_PTR(err);
}

struct syno_cache_protection_passive_btrfs_rename*
syno_cache_protection_passive_btrfs_rename_alloc(
	u64 transid, u64 subvolid, u64 old_dir, u64 new_dir,
	u64 old_name_len, void *old_name, u64 new_name_len, void *new_name,
	bool reserved)
{
	int err;
	struct syno_cache_protection_passive_btrfs_rename *rename = NULL;
	void *old_external_name = NULL;
	void *new_external_name = NULL;

	if (old_name_len > BTRFS_NAME_LEN || new_name_len > BTRFS_NAME_LEN) {
		err = -ENAMETOOLONG;
		goto out;
	}

	BUILD_BUG_ON(sizeof(*rename) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	BUILD_BUG_ON(BTRFS_NAME_LEN >= SYNO_CACHE_PROTECTION_METADATA_SIZE);
	rename = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!rename) {
		err = -ENOSPC;
		goto out;
	}
	memset(rename, 0, sizeof(*rename));
	INIT_LIST_HEAD(&rename->node.list);
	rename->node.command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RENAME;
	rename->node.transid = transid;
	rename->subvolid = subvolid;
	rename->old_dir = old_dir;
	rename->new_dir = new_dir;

	rename->old_name_len = old_name_len;
	old_external_name = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!old_external_name) {
		err = -ENOSPC;
		goto out;
	}
	rename->old_name = old_external_name;
	err = syno_cache_protection_passive_btrfs_buffer_insert(&rename->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_METADATA, old_external_name);
	if (err)
		goto out;
	memcpy(rename->old_name, old_name, old_name_len);
	rename->old_name[old_name_len] = '\0';

	rename->new_name_len = new_name_len;
	new_external_name = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!new_external_name) {
		err = -ENOSPC;
		goto out;
	}
	rename->new_name = new_external_name;
	err = syno_cache_protection_passive_btrfs_buffer_insert(&rename->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_METADATA, new_external_name);
	if (err)
		goto out;
	memcpy(rename->new_name, new_name, new_name_len);
	rename->new_name[new_name_len] = '\0';

	return rename;
out:
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, old_external_name);
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, new_external_name);
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, rename);
	return ERR_PTR(err);
}

struct syno_cache_protection_passive_btrfs_clone*
syno_cache_protection_passive_btrfs_clone_alloc(
	u64 transid, u64 src_subvolid, u64 src_inum, u64 src_offset, u64 len,
	u64 dst_subvolid, u64 dst_inum, u64 dst_offset, bool reserved)
{
	int err;
	struct syno_cache_protection_passive_btrfs_clone *clone = NULL;

	BUILD_BUG_ON(sizeof(*clone) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	clone = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!clone) {
		err = -ENOSPC;
		goto out;
	}
	memset(clone, 0, sizeof(*clone));
	INIT_LIST_HEAD(&clone->node.list);
	clone->node.command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CLONE;
	clone->node.transid = transid;
	clone->src_subvolid = src_subvolid;
	clone->src_inum = src_inum;
	clone->src_offset = src_offset;
	clone->len = len;
	clone->dst_subvolid = dst_subvolid;
	clone->dst_inum = dst_inum;
	clone->dst_offset = dst_offset;

	return clone;
out:
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, clone);
	return ERR_PTR(err);
}

struct syno_cache_protection_passive_btrfs_xattr* syno_cache_protection_passive_btrfs_xattr_alloc(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command,
		u64 transid, u64 subvolid, u64 inum, u32 name_size, u32 value_size, void *name, u32 flags, bool reserved)
{
	int err;
	void *external_name = NULL;
	struct syno_cache_protection_passive_btrfs_xattr *xattr = NULL;

	if (name_size > BTRFS_NAME_LEN) {
		err = -ENAMETOOLONG;
		goto out;
	}

	BUILD_BUG_ON(sizeof(*xattr) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	BUILD_BUG_ON(BTRFS_NAME_LEN >= SYNO_CACHE_PROTECTION_METADATA_SIZE);
	xattr = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!xattr) {
		err = -ENOSPC;
		goto out;
	}
	memset(xattr, 0, sizeof(*xattr));
	INIT_LIST_HEAD(&xattr->node.list);
	xattr->node.command = command;
	xattr->node.transid = transid;
	xattr->subvolid = subvolid;
	xattr->inum = inum;
	xattr->name_size = name_size;
	xattr->value_size = value_size;
	xattr->flags = flags;
	xattr->iname[SYNO_CACHE_PROTECTION_NAME_INLINE_LEN - 1] = 0;

	if (name_size > SYNO_CACHE_PROTECTION_NAME_INLINE_LEN - 1) {
		external_name = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
		if (!external_name) {
			err = -ENOSPC;
			goto out;
		}
		xattr->name = external_name;
		err = syno_cache_protection_passive_btrfs_buffer_insert(&xattr->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_METADATA, external_name);
		if (err)
			goto out;
	} else {
		xattr->name = xattr->iname;
	}
	memcpy(xattr->name, name, name_size);
	xattr->name[name_size] = '\0';

	return xattr;

out:
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, xattr);
	return ERR_PTR(err);
}

struct syno_cache_protection_passive_btrfs_subvol_operation*
syno_cache_protection_passive_btrfs_subvol_operation_alloc(
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command, u64 transid,
	u64 subvolid, u64 inum, u64 create, u64 qgroupid, u64 assign,
	u64 src, u64 dst, u64 uid, struct btrfs_qgroup_limit_item qgroup_limit,
	struct btrfs_usrquota_limit_item usrquota_limit, bool reserved)
{
	int err;
	struct syno_cache_protection_passive_btrfs_subvol_operation *subvol_operation = NULL;

	BUILD_BUG_ON(sizeof(*subvol_operation) > SYNO_CACHE_PROTECTION_METADATA_SIZE);
	subvol_operation = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, GFP_NOFS, reserved);
	if (!subvol_operation) {
		err = -ENOSPC;
		goto out;
	}
	memset(subvol_operation, 0, sizeof(*subvol_operation));
	INIT_LIST_HEAD(&subvol_operation->node.list);
	subvol_operation->node.command = command;
	subvol_operation->node.transid = transid;
	subvol_operation->subvolid = subvolid;
	subvol_operation->inum = inum;
	subvol_operation->uid = uid;

	if (SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_CREATE == command) {
		subvol_operation->qgroup_ca.create = create;
		subvol_operation->qgroup_ca.qgroupid = qgroupid;
	}
	if (SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_ASSIGN == command) {
		subvol_operation->qgroup_aa.assign = assign;
		subvol_operation->qgroup_aa.src = src;
		subvol_operation->qgroup_aa.dst = dst;
	}
	if (SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_LIMIT == command) {
		subvol_operation->qgroup_la.qgroupid = qgroupid;
		subvol_operation->qgroup_la.lim.flags = le64_to_cpu(qgroup_limit.flags);
		subvol_operation->qgroup_la.lim.max_rfer = le64_to_cpu(qgroup_limit.max_rfer);
		subvol_operation->qgroup_la.lim.max_excl = le64_to_cpu(qgroup_limit.max_excl);
		subvol_operation->qgroup_la.lim.rsv_rfer = le64_to_cpu(qgroup_limit.rsv_rfer);
		subvol_operation->qgroup_la.lim.rsv_excl = le64_to_cpu(qgroup_limit.rsv_excl);
	}
	if (SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_LIMIT == command) {
		subvol_operation->usrquota_la.uid = uid;
		subvol_operation->usrquota_la.rfer_soft = le64_to_cpu(usrquota_limit.rfer_soft);
		subvol_operation->usrquota_la.rfer_hard = le64_to_cpu(usrquota_limit.rfer_hard);
	}

	return subvol_operation;

out:
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA, subvol_operation);
	return ERR_PTR(err);
}

void syno_cache_protection_passive_btrfs_instance_free(struct syno_cache_protection_passive_btrfs_instance *passive_instance)
{
	struct rb_node *node;
	struct syno_cache_protection_passive_btrfs_inode *inode;
	struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command;

	if (!passive_instance)
		return;

	cancel_work_sync(&passive_instance->lru_page_reclaim_work);
	spin_lock(&passive_instance->lock);
	while (!RB_EMPTY_ROOT(&passive_instance->inode_tree)) {
		node = rb_first(&passive_instance->inode_tree);
		inode = rb_entry(node, struct syno_cache_protection_passive_btrfs_inode, inode_node);

		rb_erase(node, &passive_instance->inode_tree);
		RB_CLEAR_NODE(node);
		spin_unlock(&passive_instance->lock);
		syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
		if (need_resched())
			cond_resched();
		spin_lock(&passive_instance->lock);
	}
	while (!list_empty(&passive_instance->metadata_command_head)) {
		metadata_command = list_first_entry(&passive_instance->metadata_command_head, struct syno_cache_protection_passive_btrfs_metadata_command, list);
		list_del(&metadata_command->list);
		spin_unlock(&passive_instance->lock);
		syno_cache_protection_passive_btrfs_metadata_command_free(metadata_command);
		if (need_resched())
			cond_resched();
		spin_lock(&passive_instance->lock);
	}
	spin_unlock(&passive_instance->lock);
	syno_cache_protection_space_free(SYNO_CACHE_PROTECTION_SPACE_POOL_DATA, passive_instance);
}

/* for lru page reclaim */
static void btrfs_syno_cache_protection_lru_page_reclaim_work(struct work_struct *work)
{
	struct syno_cache_protection_passive_btrfs_instance *passive_instance;
	struct syno_cache_protection_passive_btrfs_inode *inode;
	struct syno_cache_protection_passive_btrfs_page *page;

	passive_instance = container_of(work, struct syno_cache_protection_passive_btrfs_instance, lru_page_reclaim_work);

	spin_lock(&passive_instance->lock);
	while (!list_empty(&passive_instance->lru_page_head)) {
		page = list_first_entry(&passive_instance->lru_page_head, struct syno_cache_protection_passive_btrfs_page, lru_list);
		if (atomic64_read(&page->version) >= (atomic64_read(&passive_instance->reclaim_version) & ~1ULL))
			break;
		inode = page->inode;
		if (!inode) {
			WARN_ON_ONCE(1);
			list_del_init(&page->lru_list);
			continue;
		}
		spin_lock(&inode->lock);
		if (!page_in_tree(page)) {
			spin_unlock(&inode->lock);
			list_del_init(&page->lru_list);
			continue;
		}
		if (atomic_read(&page->refs) > 1) {
			spin_unlock(&inode->lock);
			atomic64_set(&page->version, atomic64_read(&passive_instance->reclaim_version) + 1);
			list_move_tail(&page->lru_list, &passive_instance->lru_page_head);
			continue;
		}
		atomic_inc(&inode->refs);
		rb_erase(&page->page_node, &inode->page_tree);
		RB_CLEAR_NODE(&page->page_node);
		list_del_init(&page->lru_list);
		spin_unlock(&inode->lock);
		spin_unlock(&passive_instance->lock);
		syno_cache_protection_passive_btrfs_page_free(page);
		syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
		if (need_resched())
			cond_resched();
		spin_lock(&passive_instance->lock);
	}
	spin_unlock(&passive_instance->lock);

	return;
}

struct syno_cache_protection_passive_btrfs_instance* syno_cache_protection_passive_btrfs_instance_alloc(struct syno_cache_protection_fs *fs)
{
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = NULL;
	int err;

	BUILD_BUG_ON(sizeof(*passive_instance) > SYNO_CACHE_PROTECTION_DATA_SIZE);
	passive_instance = syno_cache_protection_space_alloc(SYNO_CACHE_PROTECTION_SPACE_POOL_DATA, GFP_NOFS, false);
	if (!passive_instance) {
		err = -ENOSPC;
		goto out;
	}
	memset(passive_instance, 0, sizeof(*passive_instance));
	passive_instance->cache_protection_fs = fs;
	passive_instance->uuid_len = fs->uuid_len;
	memcpy(passive_instance->uuid, fs->uuid, fs->uuid_len);
	spin_lock_init(&passive_instance->lock);
	passive_instance->inode_tree = RB_ROOT;
	INIT_LIST_HEAD(&passive_instance->metadata_command_head);
	atomic64_set(&passive_instance->last_transid, 0);
	/* for page lru */
	INIT_LIST_HEAD(&passive_instance->lru_page_head);
	atomic64_set(&passive_instance->reclaim_version, 0);
	INIT_WORK(&passive_instance->lru_page_reclaim_work, btrfs_syno_cache_protection_lru_page_reclaim_work);

	return passive_instance;

out:
	return ERR_PTR(err);
}

