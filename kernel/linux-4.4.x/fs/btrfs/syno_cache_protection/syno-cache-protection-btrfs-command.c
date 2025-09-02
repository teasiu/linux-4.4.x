/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/time.h>
#include <linux/falloc.h>
#include "../ctree.h"
#include "../btrfs_inode.h"
#include "../ordered-data.h"
#include "syno-cache-protection-btrfs.h"
#include "syno-cache-protection-btrfs-command.h"
#include "syno-cache-protection-btrfs-passive-model.h"

static int syno_cache_protection_command_generic_reserve(void *reserve_parm, size_t metadata, size_t data)
{
	int ret;
	struct syno_cache_protection_parameter_command_space_reserve *reserve = (struct syno_cache_protection_parameter_command_space_reserve*)reserve_parm;

	if (!reserve) {
		ret = -EINVAL;
		goto out;
	}

	memset(reserve, 0, sizeof(*reserve));
	reserve->count[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA] = metadata;
	reserve->count[SYNO_CACHE_PROTECTION_SPACE_POOL_DATAHEADER] = data;
	reserve->count[SYNO_CACHE_PROTECTION_SPACE_POOL_DATA] = data;

	ret = 0;
out:
	return ret;
}

static size_t btrfs_syno_cache_protection_command_checkpoint_end_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_checkpoint_end);
}

static int btrfs_syno_cache_protection_command_checkpoint_end_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_checkpoint_end *parm = (struct syno_cache_protection_parameter_command_checkpoint_end*)data;
	struct syno_cache_protection_stream_btrfs_command_checkpoint_end command_checkpoint_end;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	memset(&command_checkpoint_end, 0, sizeof(command_checkpoint_end));
	command_checkpoint_end.transid = cpu_to_le64(parm->transid);

	ret = syno_cache_protection_write_request(req, sizeof(command_checkpoint_end), &command_checkpoint_end);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_checkpoint_end_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_checkpoint_end command_checkpoint_end;
	struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command;
	u64 transid;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command_checkpoint_end), &command_checkpoint_end);
	if (ret)
		goto out;

	transid = le64_to_cpu(command_checkpoint_end.transid);
	if (atomic64_read(&passive_instance->last_transid) < transid)
		atomic64_set(&passive_instance->last_transid, transid);

	spin_lock(&passive_instance->lock);
	while (!list_empty(&passive_instance->metadata_command_head)) {
		metadata_command = list_first_entry(&passive_instance->metadata_command_head, struct syno_cache_protection_passive_btrfs_metadata_command, list);
		if (metadata_command->transid > transid)
			break;
		list_del(&metadata_command->list);
		spin_unlock(&passive_instance->lock);
		syno_cache_protection_passive_btrfs_metadata_command_free(metadata_command);
		spin_lock(&passive_instance->lock);
	}
	spin_unlock(&passive_instance->lock);

	ret = 0;
out:
	return ret;
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_checkpoint_end_ops = {
	.wait = false,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_HIGH,
	.skip_check_enabled = true,
	.size = btrfs_syno_cache_protection_command_checkpoint_end_size,
	.send = btrfs_syno_cache_protection_command_checkpoint_end_send,
	.receive = btrfs_syno_cache_protection_command_checkpoint_end_receive,
};

static int btrfs_syno_cache_protection_command_data_reclaim_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	atomic64_inc(&passive_instance->reclaim_version);
	if ((0 == atomic64_read(&passive_instance->reclaim_version) % 2) && !work_busy(&passive_instance->lru_page_reclaim_work))
		queue_work(system_unbound_wq, &passive_instance->lru_page_reclaim_work);

	ret = 0;
out:
	return ret;
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_data_reclaim_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_HIGH,
	.skip_check_enabled = true,
	.receive = btrfs_syno_cache_protection_command_data_reclaim_receive,
};

static size_t btrfs_syno_cache_protection_command_write_size(void *data)
{
	struct syno_cache_protection_parameter_command_write *parm = (struct syno_cache_protection_parameter_command_write*)data;

	BUG_ON(!data);

	return sizeof(struct syno_cache_protection_stream_btrfs_command_write) + parm->num_pages * SYNO_CACHE_PROTECTION_DATA_SIZE;
}

static int btrfs_syno_cache_protection_command_write_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_write *parm = (struct syno_cache_protection_parameter_command_write*)data;
	struct syno_cache_protection_stream_btrfs_command_write command_write;
	struct inode *inode;
	struct btrfs_root *root;
	size_t i, num_pages;
	u64 pg_index;
	struct page *page;
	char *addr;

	if (!req || !parm || parm->num_pages <= 0) {
		ret = -EINVAL;
		goto out;
	}

	inode = parm->inode;
	root = BTRFS_I(inode)->root;
	num_pages = parm->num_pages;
	pg_index = page_index(parm->pages[0]);

	memset(&command_write, 0, sizeof(command_write));
	command_write.subvolid = cpu_to_le64(root->objectid);
	command_write.inum = cpu_to_le64(btrfs_ino(inode));
	command_write.num_pages = cpu_to_le32(num_pages);
	command_write.page_index = cpu_to_le64(pg_index);
	command_write.i_size = cpu_to_le64(i_size_read(inode));

	ret = syno_cache_protection_write_request(req, sizeof(command_write), &command_write);
	if (ret)
		goto out;

	for (i = 0; i < num_pages; i++) {
		page = parm->pages[i];
		addr = kmap(page);
		ret = syno_cache_protection_write_request(req, SYNO_CACHE_PROTECTION_DATA_SIZE, addr);
		kunmap(page);
		if (ret)
			goto out;
	}

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_write_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_write command_write;
	struct syno_cache_protection_passive_btrfs_inode *inode = NULL;
	struct syno_cache_protection_passive_btrfs_page *page = NULL;
	size_t i, num_pages;
	u64 pg_index;
	bool new_alloc;
	u64 new_i_size;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command_write), &command_write);
	if (ret)
		goto out;

	inode = syno_cache_protection_passive_btrfs_get_or_alloc_inode(passive_instance, le64_to_cpu(command_write.subvolid), le64_to_cpu(command_write.inum), true, reserved);
	if (!inode) {
		ret = -ENOSPC;
		goto out;
	}

	num_pages = le32_to_cpu(command_write.num_pages);
	pg_index = le64_to_cpu(command_write.page_index);

	for (i = 0; i < num_pages; i++) {
		page = syno_cache_protection_passive_btrfs_get_or_alloc_page(passive_instance, inode, pg_index + i, reserved, &new_alloc);
		if (!page) {
			ret = -ENOSPC;
			goto out;
		}
		ret = syno_cache_protection_read_request(req, SYNO_CACHE_PROTECTION_DATA_SIZE, page->value);
		if (ret) {
			if (new_alloc) {
				spin_lock(&inode->lock);
				rb_erase(&page->page_node, &inode->page_tree);
				RB_CLEAR_NODE(&page->page_node);
				spin_unlock(&inode->lock);
				syno_cache_protection_passive_btrfs_page_free(page);
			}
			syno_cache_protection_passive_btrfs_page_free(page);
			goto out;
		}
		syno_cache_protection_passive_btrfs_page_free(page);
	}

	new_i_size = le64_to_cpu(command_write.i_size);
	if (new_i_size > inode->i_size)
		inode->i_size = new_i_size;

	ret = 0;
out:
	syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
	return ret;
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_write_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_WAIT,
	.size = btrfs_syno_cache_protection_command_write_size,
	.send = btrfs_syno_cache_protection_command_write_send,
	.receive = btrfs_syno_cache_protection_command_write_receive,
};

static size_t syno_cache_protection_btrfs_command_ordered_extent_size(void *data)
{
	struct syno_cache_protection_parameter_command_ordered_extent *parm = (struct syno_cache_protection_parameter_command_ordered_extent*)data;

	BUG_ON(!data);

	return sizeof(struct syno_cache_protection_stream_btrfs_command_ordered_extent) + parm->total_csum_size;
}

static int syno_cache_protection_btrfs_command_ordered_extent_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_ordered_extent *parm = (struct syno_cache_protection_parameter_command_ordered_extent*)data;
	struct syno_cache_protection_stream_btrfs_command_ordered_extent command_ordered_extent;
	struct btrfs_ordered_extent *ordered_extent;
	struct inode *inode;
	struct btrfs_root *root;
	size_t command_size;
	u64 transid;
	struct syno_cache_protection_stream_btrfs_command_ordered_extent_csum command_ordered_extent_csum;
	struct btrfs_ordered_sum *sum;
	size_t i, csum_count, csum_data_size;
	__le32 csum_data;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	transid = parm->transid;
	ordered_extent = parm->ordered_extent;
	inode = ordered_extent->inode;
	root = BTRFS_I(inode)->root;

	command_size = sizeof(command_ordered_extent);
	memset(&command_ordered_extent, 0, command_size);
	command_ordered_extent.err = cpu_to_le32(parm->err);
	command_ordered_extent.transid = cpu_to_le64(transid);
	command_ordered_extent.subvolid = cpu_to_le64(root->objectid);
	command_ordered_extent.inum = cpu_to_le64(btrfs_ino(inode));
	command_ordered_extent.file_offset = cpu_to_le64(ordered_extent->file_offset);
	command_ordered_extent.start = cpu_to_le64(ordered_extent->start);
	command_ordered_extent.len = cpu_to_le64(ordered_extent->len);
	command_ordered_extent.disk_len = cpu_to_le64(ordered_extent->disk_len);
	command_ordered_extent.truncated_len = cpu_to_le64(ordered_extent->truncated_len);
	command_ordered_extent.flags = cpu_to_le64(ordered_extent->flags);
	command_ordered_extent.compress_type = cpu_to_le32(ordered_extent->compress_type);
	command_ordered_extent.bl_update_isize = cpu_to_le32(parm->bl_update_isize ? 1 : 0);
	command_ordered_extent.i_size = cpu_to_le64(i_size_read(inode));
	command_ordered_extent.total_csums = cpu_to_le32(parm->total_csums);
	command_ordered_extent.total_csum_size = cpu_to_le32(parm->total_csum_size);

	ret = syno_cache_protection_write_request(req, command_size, &command_ordered_extent);
	if (ret)
		goto out;

	if (parm->total_csum_size > 0) {
		command_size = sizeof(command_ordered_extent_csum);
		csum_data_size = sizeof(csum_data);
		list_for_each_entry(sum, &parm->ordered_extent->list, list) {
			csum_count = (int)DIV_ROUND_UP(sum->len, root->fs_info->csum_root->sectorsize);

			command_ordered_extent_csum.bytenr = cpu_to_le64(sum->bytenr);
			command_ordered_extent_csum.len = cpu_to_le32(sum->len);
			ret = syno_cache_protection_write_request(req, command_size, &command_ordered_extent_csum);
			if (ret)
				goto out;

			for (i = 0; i < csum_count; i++) {
				csum_data = cpu_to_le32(sum->sums[i]);
				ret = syno_cache_protection_write_request(req, csum_data_size, &csum_data);
				if (ret)
					goto out;
			}
		}
	}

	ret = 0;
out:
	return ret;
}

static void btrfs_syno_cache_protection_command_inode_drop_range(struct syno_cache_protection_passive_btrfs_instance *passive_instance, u64 subvolid, u64 inum, u64 start, u64 end)
{
	struct syno_cache_protection_passive_btrfs_inode *inode = NULL;
	struct syno_cache_protection_passive_btrfs_page *page;
	u64 pg_start, pg_end;
	size_t offset;

	if (start > end)
		goto out;

	pg_start = start >> SYNO_CACHE_PROTECTION_DATA_SHIFT;
	pg_end = end >> SYNO_CACHE_PROTECTION_DATA_SHIFT;

	offset = start & (SYNO_CACHE_PROTECTION_DATA_SIZE - 1);
	if (offset)
		pg_start++;
	offset = end & (SYNO_CACHE_PROTECTION_DATA_SIZE - 1);
	if (offset != (SYNO_CACHE_PROTECTION_DATA_SIZE - 1)) {
		if (0 == pg_end)
			goto out;
		else
			pg_end--;
	}

	inode = syno_cache_protection_passive_btrfs_get_or_alloc_inode(passive_instance, subvolid, inum, false, false);
	if (!inode)
		goto out;

	spin_lock(&inode->lock);
	while (1) {
		page = syno_cache_protection_passive_btrfs_page_tree_search_with_range(&inode->page_tree, pg_start, pg_end);
		if (!page)
			break;
		rb_erase(&page->page_node, &inode->page_tree);
		RB_CLEAR_NODE(&page->page_node);
		spin_unlock(&inode->lock);
		syno_cache_protection_passive_btrfs_page_free(page);
		if (need_resched())
			cond_resched();
		spin_lock(&inode->lock);
	}
	spin_unlock(&inode->lock);

out:
	syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
	return;
}

static int syno_cache_protection_btrfs_command_ordered_extent_receive(void *private, void *req, bool reserved)
{
	int ret, err;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_ordered_extent command_ordered_extent;
	unsigned long flags;
	struct syno_cache_protection_passive_btrfs_ordered_extent *ordered_extent = NULL;
	u64 transid;
	struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer = NULL;
	size_t total_csum_size;
	bool bl_update_isize;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command_ordered_extent), &command_ordered_extent);
	if (ret)
		goto out;

	err = le32_to_cpu(command_ordered_extent.err);
	flags = le64_to_cpu(command_ordered_extent.flags);
	transid = le64_to_cpu(command_ordered_extent.transid);
	bl_update_isize = le32_to_cpu(command_ordered_extent.bl_update_isize) ? true : false;

	if (!err && transid > atomic64_read(&passive_instance->last_transid) && (!test_bit(BTRFS_ORDERED_NOCOW, &flags) || bl_update_isize)) {
		ordered_extent = syno_cache_protection_passive_btrfs_ordered_extent_alloc(transid, le64_to_cpu(command_ordered_extent.subvolid),
							le64_to_cpu(command_ordered_extent.inum), le64_to_cpu(command_ordered_extent.file_offset), le64_to_cpu(command_ordered_extent.start),
							le64_to_cpu(command_ordered_extent.len), le64_to_cpu(command_ordered_extent.disk_len),
							le64_to_cpu(command_ordered_extent.truncated_len), le64_to_cpu(command_ordered_extent.flags), le32_to_cpu(command_ordered_extent.compress_type),
							le64_to_cpu(command_ordered_extent.i_size), le32_to_cpu(command_ordered_extent.total_csums), le32_to_cpu(command_ordered_extent.total_csum_size), reserved);
		if (!ordered_extent) {
			ret = -ENOSPC;
			goto out;
		}

		total_csum_size = ordered_extent->total_csum_size;
		if (total_csum_size > 0) {
			virtual_buffer = syno_cache_protection_passive_btrfs_virtual_buffer_alloc(total_csum_size, reserved, SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM);
			if (IS_ERR(virtual_buffer)) {
				ret = PTR_ERR(virtual_buffer);
				virtual_buffer = NULL;
				goto out;
			}
			ret = syno_cache_protection_passive_btrfs_virtual_buffer_fill_from_request(req, virtual_buffer, 0, total_csum_size);
			if (ret)
				goto out;

			ret = syno_cache_protection_passive_btrfs_buffer_insert(&ordered_extent->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_VIRTUAL_BUFFER, virtual_buffer);
			if (ret)
				goto out;
			ordered_extent->csums = virtual_buffer;
		}

		spin_lock(&passive_instance->lock);
		list_add_tail(&ordered_extent->node.list, &passive_instance->metadata_command_head);
		spin_unlock(&passive_instance->lock);
	}

	btrfs_syno_cache_protection_command_inode_drop_range(passive_instance, le64_to_cpu(command_ordered_extent.subvolid), le64_to_cpu(command_ordered_extent.inum),
														le64_to_cpu(command_ordered_extent.file_offset), le64_to_cpu(command_ordered_extent.file_offset) + le64_to_cpu(command_ordered_extent.len) - 1);

	ret = 0;
out:
	if (ret) {
		syno_cache_protection_passive_btrfs_virtual_buffer_free(virtual_buffer);
		if (ordered_extent)
			syno_cache_protection_passive_btrfs_metadata_command_free(&ordered_extent->node);
	}
	return ret;
}

static int btrfs_syno_cache_protection_command_ordered_extent_reserve(void *reserve_parm, void *data)
{
	int ret;
	struct syno_cache_protection_parameter_command_space_reserve *reserve = (struct syno_cache_protection_parameter_command_space_reserve*)reserve_parm;
	struct syno_cache_protection_parameter_command_ordered_extent *parm = (struct syno_cache_protection_parameter_command_ordered_extent*)data;
	size_t reserve_checksum_blocks;

	if (!reserve) {
		ret = -EINVAL;
		goto out;
	}

	if (test_bit(BTRFS_ORDERED_NOCOW, &parm->ordered_extent->flags) && !parm->bl_update_isize) {
		ret = 1;
		goto out;
	}

	reserve_checksum_blocks = DIV_ROUND_UP(parm->total_csum_size, SYNO_CACHE_PROTECTION_DATA_SIZE);
	if (reserve_checksum_blocks)
		reserve_checksum_blocks++;
	/*
	 * Metadata:
	 *     1 : ordered extent command
	 * Checksum:
	 *     1 : virtual buffer header
	 *     n : round_up(csum size / SYNO_CACHE_PROTECTION_DATA_SIZE)
	 */

	memset(reserve, 0, sizeof(*reserve));
	reserve->count[SYNO_CACHE_PROTECTION_SPACE_POOL_METADATA] = 1;
	reserve->count[SYNO_CACHE_PROTECTION_SPACE_POOL_CHECKSUM] = reserve_checksum_blocks;

	ret = 0;
out:
	return ret;
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_ordered_extent = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_EXTENT,
	.skip_check_enabled = true,
	.size = syno_cache_protection_btrfs_command_ordered_extent_size,
	.send = syno_cache_protection_btrfs_command_ordered_extent_send,
	.receive = syno_cache_protection_btrfs_command_ordered_extent_receive,
	.reserve = btrfs_syno_cache_protection_command_ordered_extent_reserve,
};

static size_t syno_cache_protection_btrfs_command_inline_extent_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_inline_extent);
}

static int syno_cache_protection_btrfs_command_inline_extent_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_inline_extent *parm = (struct syno_cache_protection_parameter_command_inline_extent*)data;
	struct syno_cache_protection_stream_btrfs_command_inline_extent command;
	struct inode *inode;
	struct btrfs_root *root;
	size_t command_size;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	inode = parm->inode;
	root = BTRFS_I(parm->inode)->root;

	command_size = sizeof(command);
	memset(&command, 0, command_size);
	command.err = cpu_to_le32(parm->err);
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(root->objectid);
	command.inum = cpu_to_le64(btrfs_ino(inode));
	command.inline_len = cpu_to_le64(parm->inline_len);

	ret = syno_cache_protection_write_request(req, command_size, &command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int syno_cache_protection_btrfs_command_inline_extent_receive(void *private, void *req, bool reserved)
{
	int ret, err;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_inline_extent command;
	struct syno_cache_protection_passive_btrfs_inline_extent *inline_extent = NULL;
	u64 transid;
	struct syno_cache_protection_passive_btrfs_inode *inode = NULL;
	struct syno_cache_protection_passive_btrfs_page *page = NULL;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	err = le32_to_cpu(command.err);
	transid = le64_to_cpu(command.transid);

	if (!err && transid > atomic64_read(&passive_instance->last_transid)) {
		inline_extent = syno_cache_protection_passive_btrfs_inline_extent_alloc(transid, le64_to_cpu(command.subvolid),  le64_to_cpu(command.inum), le64_to_cpu(command.inline_len), reserved);
		if (!inline_extent) {
			ret = -ENOSPC;
			goto out;
		}
	}

	inode = syno_cache_protection_passive_btrfs_get_or_alloc_inode(passive_instance, le64_to_cpu(command.subvolid), le64_to_cpu(command.inum), false, false);
	if (!inode) {
		ret = 0;
		goto out;
	}

	spin_lock(&inode->lock);
	page = syno_cache_protection_passive_btrfs_page_tree_search(&inode->page_tree, 0);
	if (page) {
		rb_erase(&page->page_node, &inode->page_tree);
		RB_CLEAR_NODE(&page->page_node);
		if (inline_extent) {
			inline_extent->inline_data = page->value;
			page->value = NULL;
		}
	}
	spin_unlock(&inode->lock);

	if (inline_extent && inline_extent->inline_data) {
		ret = syno_cache_protection_passive_btrfs_buffer_insert(&inline_extent->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_DATA, inline_extent->inline_data);
		if (ret)
			goto out;
		spin_lock(&passive_instance->lock);
		list_add_tail(&inline_extent->node.list, &passive_instance->metadata_command_head);
		spin_unlock(&passive_instance->lock);
	}

	ret = 0;
out:
	syno_cache_protection_passive_btrfs_page_free(page);
	syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
	if (inline_extent && (ret || !inline_extent->inline_data))
		syno_cache_protection_passive_btrfs_metadata_command_free(&inline_extent->node);
	return ret;
}

static int btrfs_syno_cache_protection_command_inline_extent_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *     1 : inline extent command
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 1, 0);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_inline_extent = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_EXTENT,
	.skip_check_enabled = true,
	.size = syno_cache_protection_btrfs_command_inline_extent_size,
	.send = syno_cache_protection_btrfs_command_inline_extent_send,
	.receive = syno_cache_protection_btrfs_command_inline_extent_receive,
	.reserve = btrfs_syno_cache_protection_command_inline_extent_reserve,
};

static size_t btrfs_syno_cache_protection_command_space_reserve_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_space_reserve);
}

static int btrfs_syno_cache_protection_command_space_reserve_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_space_reserve *parm = (struct syno_cache_protection_parameter_command_space_reserve*)data;
	struct syno_cache_protection_stream_btrfs_command_space_reserve command_space_reserve;
	size_t i;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	memset(&command_space_reserve, 0, sizeof(command_space_reserve));
	for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++)
		command_space_reserve.count[i] = cpu_to_le32(parm->count[i]);

	ret = syno_cache_protection_write_request(req, sizeof(command_space_reserve), &command_space_reserve);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_space_reserve_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_stream_btrfs_command_space_reserve command_space_reserve;
	size_t i = 0, j = 0, count;

	if (!req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command_space_reserve), &command_space_reserve);
	if (ret)
		goto out;

	for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++) {
		count = le32_to_cpu(command_space_reserve.count[i]);
		ret = syno_cache_protection_space_reserve((enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE)i, count, GFP_NOFS);
		if (ret)
			goto out;
	}

	ret = 0;
out:
	if (ret) {
		for (j = 0; j < i; j++) {
			count = le32_to_cpu(command_space_reserve.count[j]);
			syno_cache_protection_space_reserve_free((enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE)j, count);
		}
	}
	return ret;
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_space_reserve_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_WAIT,
	.skip_check_enabled = true,
	.size = btrfs_syno_cache_protection_command_space_reserve_size,
	.send = btrfs_syno_cache_protection_command_space_reserve_send,
	.receive = btrfs_syno_cache_protection_command_space_reserve_receive,
};

static int btrfs_syno_cache_protection_command_space_reserve_free_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_stream_btrfs_command_space_reserve command_space_reserve;
	size_t i, count;

	ret = syno_cache_protection_read_request(req, sizeof(command_space_reserve), &command_space_reserve);
	if (ret)
		goto out;

	for (i = 0; i < SYNO_CACHE_PROTECTION_SPACE_POOL_MAX; i++) {
		count = le32_to_cpu(command_space_reserve.count[i]);
		syno_cache_protection_space_reserve_free((enum SYNO_CACHE_PROTECTION_SPACE_POOL_TYPE)i, count);
	}

	ret = 0;
out:
	return ret;
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_space_reserve_free_ops = {
	.wait = false,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_RESERVE_FREE,
	.skip_check_enabled = true,
	.size = btrfs_syno_cache_protection_command_space_reserve_size,
	.send = btrfs_syno_cache_protection_command_space_reserve_send,
	.receive = btrfs_syno_cache_protection_command_space_reserve_free_receive,
};

static size_t btrfs_syno_cache_protection_command_create_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_create);
}

static int btrfs_syno_cache_protection_command_create_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_create *parm = (struct syno_cache_protection_parameter_command_create*)data;
	struct syno_cache_protection_stream_btrfs_command_create command;
	struct btrfs_root *root;
	umode_t mode;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	if (parm->dentry->d_name.len > BTRFS_NAME_LEN) {
		ret = -ENAMETOOLONG;
		goto out;
	}

	root = BTRFS_I(parm->dir)->root;
	mode = parm->inode->i_mode;

	memset(&command, 0, sizeof(command));
	command.type = cpu_to_le32((int)parm->command);
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(root->objectid);
	command.dir = cpu_to_le64(btrfs_ino(parm->dir));
	command.inum = cpu_to_le64(btrfs_ino(parm->inode));
	command.generation = cpu_to_le64(BTRFS_I(parm->inode)->generation);
	command.mode = cpu_to_le64(mode);
	command.nlink = cpu_to_le64(parm->inode->i_nlink);
	command.name_len = cpu_to_le64(parm->dentry->d_name.len);
	memcpy(command.name, parm->dentry->d_name.name, command.name_len);
	if (S_ISCHR(mode) || S_ISBLK(mode) || S_ISFIFO(mode) || S_ISSOCK(mode))
		command.rdev = cpu_to_le64(new_encode_dev(parm->inode->i_rdev));

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_create_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_create command;
	struct syno_cache_protection_passive_btrfs_create *create = NULL;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND type;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	type = (enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND)le32_to_cpu(command.type);

	create = syno_cache_protection_passive_btrfs_create_alloc(type, le64_to_cpu(command.transid), le64_to_cpu(command.subvolid),
				le64_to_cpu(command.dir), le64_to_cpu(command.inum), le64_to_cpu(command.generation), le64_to_cpu(command.mode),
				le64_to_cpu(command.rdev), le64_to_cpu(command.name_len), command.name, reserved);
	if (IS_ERR(create)) {
		ret = PTR_ERR(create);
		create = NULL;
		goto out;
	}

	spin_lock(&passive_instance->lock);
	list_add_tail(&create->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	if (create->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_UNLINK && (0 == le64_to_cpu(command.nlink)))
		btrfs_syno_cache_protection_command_inode_drop_range(passive_instance, create->subvolid, create->inum, 0, -1);

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_create_reserve(void *reserve_parm, void *data)
{
	/*
	 * 1 : mkfile/mknod/mkdir/link/rmdir/unlink command
	 * 1 : file name
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 2, 0);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_create_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_create_size,
	.send = btrfs_syno_cache_protection_command_create_send,
	.receive = btrfs_syno_cache_protection_command_create_receive,
	.reserve = btrfs_syno_cache_protection_command_create_reserve,
};

static size_t btrfs_syno_cache_protection_command_symlink_size(void *data)
{
	struct syno_cache_protection_parameter_command_create *parm = (struct syno_cache_protection_parameter_command_create*)data;

	BUG_ON(!data);

	return sizeof(struct syno_cache_protection_stream_btrfs_command_create) + parm->symname_len;
}

static int btrfs_syno_cache_protection_command_symlink_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_create *parm = (struct syno_cache_protection_parameter_command_create*)data;
	struct syno_cache_protection_stream_btrfs_command_create command;
	struct btrfs_root *root;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	if (parm->dentry->d_name.len > BTRFS_NAME_LEN || parm->symname_len >= BTRFS_LEAF_SIZE) {
		ret = -ENAMETOOLONG;
		goto out;
	}

	root = BTRFS_I(parm->dir)->root;

	memset(&command, 0, sizeof(command));
	command.type = cpu_to_le32((int)parm->command);
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(root->objectid);
	command.dir = cpu_to_le64(btrfs_ino(parm->dir));
	command.inum = cpu_to_le64(btrfs_ino(parm->inode));
	command.generation = cpu_to_le64(BTRFS_I(parm->inode)->generation);
	command.mode = cpu_to_le64(parm->inode->i_mode);
	command.name_len = cpu_to_le64(parm->dentry->d_name.len);
	memcpy(command.name, parm->dentry->d_name.name, command.name_len);
	command.symname_len = cpu_to_le64(parm->symname_len);

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = syno_cache_protection_write_request(req, parm->symname_len, parm->symname);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_symlink_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_create command;
	struct syno_cache_protection_passive_btrfs_create *create = NULL;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND type;
	struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer = NULL;
	u64 symname_len;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	type = (enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND)le32_to_cpu(command.type);

	create = syno_cache_protection_passive_btrfs_create_alloc(type, le64_to_cpu(command.transid), le64_to_cpu(command.subvolid),
				le64_to_cpu(command.dir), le64_to_cpu(command.inum), le64_to_cpu(command.generation), le64_to_cpu(command.mode),
				le64_to_cpu(command.rdev), le64_to_cpu(command.name_len), command.name, reserved);
	if (IS_ERR(create)) {
		ret = PTR_ERR(create);
		create = NULL;
		goto out;
	}

	symname_len = le64_to_cpu(command.symname_len);
	virtual_buffer = syno_cache_protection_passive_btrfs_virtual_buffer_alloc(symname_len, reserved, SYNO_CACHE_PROTECTION_SPACE_POOL_DATA);
	if (IS_ERR(virtual_buffer)) {
		ret = PTR_ERR(virtual_buffer);
		virtual_buffer = NULL;
		goto out;
	}
	ret = syno_cache_protection_passive_btrfs_virtual_buffer_fill_from_request(req, virtual_buffer, 0, symname_len);
	if (ret)
		goto out;
	ret = syno_cache_protection_passive_btrfs_buffer_insert(&create->node.extra_buffers, SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_VIRTUAL_BUFFER, virtual_buffer);
	if (ret)
		goto out;
	create->symname_len = symname_len;
	create->symname = virtual_buffer;

	spin_lock(&passive_instance->lock);
	list_add_tail(&create->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	ret = 0;
out:
	if (ret) {
		syno_cache_protection_passive_btrfs_virtual_buffer_free(virtual_buffer);
		if (create)
			syno_cache_protection_passive_btrfs_metadata_command_free(&create->node);
	}
	return ret;
}

static int btrfs_syno_cache_protection_command_symlink_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *   1 : mkfile command
	 *   1 : file name
	 * Data:
	 *   1 : virtual buffer
	 *   4 : symname
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 2, 5);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_symlink_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_symlink_size,
	.send = btrfs_syno_cache_protection_command_symlink_send,
	.receive = btrfs_syno_cache_protection_command_symlink_receive,
	.reserve = btrfs_syno_cache_protection_command_symlink_reserve,
};

static void btrfs_syno_cache_protection_command_inode_truncate(struct syno_cache_protection_passive_btrfs_instance *passive_instance, u64 subvolid, u64 inum, u64 newsize)
{
	struct syno_cache_protection_passive_btrfs_inode *inode = NULL;

	inode = syno_cache_protection_passive_btrfs_get_or_alloc_inode(passive_instance, subvolid, inum, false, false);
	if (!inode)
		goto out;

	spin_lock(&inode->lock);
	inode->i_size = newsize;
	spin_unlock(&inode->lock);

	btrfs_syno_cache_protection_command_inode_drop_range(passive_instance, subvolid, inum, newsize, -1);

out:
	syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
	return;
}

static void btrfs_syno_cache_protection_command_inode_fallocate(struct syno_cache_protection_passive_btrfs_instance *passive_instance, u64 subvolid, u64 inum,
																u64 mode, u64 offset, u64 len)
{
	struct syno_cache_protection_passive_btrfs_inode *inode = NULL;

	if (!(mode & FALLOC_FL_PUNCH_HOLE))
		goto out;

	inode = syno_cache_protection_passive_btrfs_get_or_alloc_inode(passive_instance, subvolid, inum, false, false);
	if (!inode)
		goto out;

	btrfs_syno_cache_protection_command_inode_drop_range(passive_instance, subvolid, inum, offset, offset + len - 1);

out:
	syno_cache_protection_passive_btrfs_inode_free(passive_instance, inode);
	return;
}

static size_t btrfs_syno_cache_protection_command_inode_operation_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_inode_operation);
}

static int btrfs_syno_cache_protection_command_inode_operation_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_inode_operation *parm = (struct syno_cache_protection_parameter_command_inode_operation*)data;
	struct syno_cache_protection_stream_btrfs_command_inode_operation command;
	struct btrfs_root *root;
	struct inode *inode;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	root = BTRFS_I(parm->inode)->root;
	inode = parm->inode;

	memset(&command, 0, sizeof(command));
	command.type = cpu_to_le32((int)parm->command);
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(root->objectid);
	command.inum = cpu_to_le64(btrfs_ino(parm->inode));
	command.flags = cpu_to_le64(parm->flags);
	command.mode = cpu_to_le64(inode->i_mode);
	command.uid = cpu_to_le32(inode->i_uid.val);
	command.gid = cpu_to_le32(inode->i_gid.val);
	command.times[0].sec  = cpu_to_le64(inode->i_atime.tv_sec);
	command.times[0].nsec = cpu_to_le32(inode->i_atime.tv_nsec);
	command.times[1].sec  = cpu_to_le64(inode->i_mtime.tv_sec);
	command.times[1].nsec = cpu_to_le32(inode->i_mtime.tv_nsec);
	command.offset = cpu_to_le64(parm->offset);
	command.length = cpu_to_le64(parm->length);

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_inode_operation_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_inode_operation command;
	struct syno_cache_protection_passive_btrfs_inode_operation *inode_operation = NULL;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND type;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	type = (enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND)le32_to_cpu(command.type);

	inode_operation = syno_cache_protection_passive_btrfs_inode_operation_alloc(type, le64_to_cpu(command.transid), le64_to_cpu(command.subvolid),
				le64_to_cpu(command.inum), le64_to_cpu(command.flags), le64_to_cpu(command.mode),
				le32_to_cpu(command.uid), le32_to_cpu(command.gid), command.times, le64_to_cpu(command.offset), le64_to_cpu(command.length), reserved);
	if (IS_ERR(inode_operation)) {
		ret = PTR_ERR(inode_operation);
		inode_operation = NULL;
		goto out;
	}

	spin_lock(&passive_instance->lock);
	list_add_tail(&inode_operation->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	if (inode_operation->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_TRUNCATE)
		btrfs_syno_cache_protection_command_inode_truncate(passive_instance, inode_operation->subvolid, inode_operation->inum, inode_operation->length);
	if (inode_operation->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE)
		btrfs_syno_cache_protection_command_inode_fallocate(passive_instance, inode_operation->subvolid, inode_operation->inum,
															inode_operation->flags, inode_operation->offset, inode_operation->length);

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_inode_operation_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *   1 : command
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 1, 0);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_inode_operation_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_inode_operation_size,
	.send = btrfs_syno_cache_protection_command_inode_operation_send,
	.receive = btrfs_syno_cache_protection_command_inode_operation_receive,
	.reserve = btrfs_syno_cache_protection_command_inode_operation_reserve,
};

static size_t btrfs_syno_cache_protection_command_rename_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_rename);
}

static int btrfs_syno_cache_protection_command_rename_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_rename *parm = (struct syno_cache_protection_parameter_command_rename*)data;
	struct syno_cache_protection_stream_btrfs_command_rename command;
	struct btrfs_root *root;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	root = BTRFS_I(parm->old_dir)->root;

	memset(&command, 0, sizeof(command));
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(root->objectid);
	command.old_dir = cpu_to_le64(btrfs_ino(parm->old_dir));
	command.new_dir = cpu_to_le64(btrfs_ino(parm->new_dir));
	command.old_name_len = parm->old_dentry->d_name.len;
	memcpy(command.old_name, parm->old_dentry->d_name.name, command.old_name_len);
	command.new_name_len = parm->new_dentry->d_name.len;
	memcpy(command.new_name, parm->new_dentry->d_name.name, command.new_name_len);

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_rename_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_rename command;
	struct syno_cache_protection_passive_btrfs_rename *rename = NULL;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	rename = syno_cache_protection_passive_btrfs_rename_alloc(
				le64_to_cpu(command.transid), le64_to_cpu(command.subvolid),
				le64_to_cpu(command.old_dir), le64_to_cpu(command.new_dir),
				le64_to_cpu(command.old_name_len), command.old_name,
				le64_to_cpu(command.new_name_len), command.new_name, reserved);
	if (IS_ERR(rename)) {
		ret = PTR_ERR(rename);
		rename = NULL;
		goto out;
	}

	spin_lock(&passive_instance->lock);
	list_add_tail(&rename->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_rename_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *   1 : command
	 *   1 : old name
	 *   1 : new name
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 3, 0);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_rename_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_rename_size,
	.send = btrfs_syno_cache_protection_command_rename_send,
	.receive = btrfs_syno_cache_protection_command_rename_receive,
	.reserve = btrfs_syno_cache_protection_command_rename_reserve,
};

static size_t btrfs_syno_cache_protection_command_clone_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_clone);
}

static int btrfs_syno_cache_protection_command_clone_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_clone *parm = (struct syno_cache_protection_parameter_command_clone*)data;
	struct syno_cache_protection_stream_btrfs_command_clone command;
	struct btrfs_root *src_root, *dst_root;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	src_root = BTRFS_I(parm->src_inode)->root;
	dst_root = BTRFS_I(parm->dst_inode)->root;

	memset(&command, 0, sizeof(command));
	command.transid = cpu_to_le64(parm->transid);
	command.src_subvolid = cpu_to_le64(src_root->objectid);
	command.src_inum = cpu_to_le64(btrfs_ino(parm->src_inode));
	command.src_offset = cpu_to_le64(parm->src_offset);
	command.len = cpu_to_le64(parm->len);
	command.dst_subvolid = cpu_to_le64(dst_root->objectid);
	command.dst_inum = cpu_to_le64(btrfs_ino(parm->dst_inode));
	command.dst_offset = cpu_to_le64(parm->dst_offset);

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_clone_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_clone command;
	struct syno_cache_protection_passive_btrfs_clone *clone = NULL;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	clone = syno_cache_protection_passive_btrfs_clone_alloc(le64_to_cpu(command.transid), le64_to_cpu(command.src_subvolid), le64_to_cpu(command.src_inum), le64_to_cpu(command.src_offset),
														le64_to_cpu(command.len), le64_to_cpu(command.dst_subvolid), le64_to_cpu(command.dst_inum), le64_to_cpu(command.dst_offset), reserved);
	if (IS_ERR(clone)) {
		ret = PTR_ERR(clone);
		clone = NULL;
		goto out;
	}

	spin_lock(&passive_instance->lock);
	list_add_tail(&clone->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_clone_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *   1 : command
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 1, 0);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_clone_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_clone_size,
	.send = btrfs_syno_cache_protection_command_clone_send,
	.receive = btrfs_syno_cache_protection_command_clone_receive,
	.reserve = btrfs_syno_cache_protection_command_clone_reserve,
};

static size_t btrfs_syno_cache_protection_command_xattr_size(void *data)
{
	struct syno_cache_protection_parameter_command_xattr *parm =
		(struct syno_cache_protection_parameter_command_xattr*)data;

	BUG_ON(!data);

	return sizeof(struct syno_cache_protection_stream_btrfs_command_xattr) + parm->value_size;
}

static int btrfs_syno_cache_protection_command_xattr_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_xattr *parm =
		(struct syno_cache_protection_parameter_command_xattr*)data;
	struct syno_cache_protection_stream_btrfs_command_xattr command;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	if (parm->name_size > BTRFS_NAME_LEN || parm->value_size >= BTRFS_LEAF_SIZE) {
		ret = -ENAMETOOLONG;
		goto out;
	}

	memset(&command, 0, sizeof(command));
	command.type = cpu_to_le32((int)parm->command);
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(BTRFS_I(parm->inode)->root->objectid);
	command.inum = cpu_to_le64(btrfs_ino(parm->inode));
	command.name_size = cpu_to_le32(parm->name_size);
	command.value_size = cpu_to_le32(parm->value_size);
	memcpy(command.name, parm->name, command.name_size);
	command.flags = cpu_to_le32(parm->flags);

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = syno_cache_protection_write_request(req, parm->value_size, parm->value);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_xattr_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance =
		(struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_xattr command;
	struct syno_cache_protection_passive_btrfs_xattr *xattr = NULL;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND type;
	struct syno_cache_protection_passive_btrfs_virtual_buffer *virtual_buffer = NULL;
	u32 value_size;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	type = (enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND)le32_to_cpu(command.type);

	xattr = syno_cache_protection_passive_btrfs_xattr_alloc(type, le64_to_cpu(command.transid), le64_to_cpu(command.subvolid),
		le64_to_cpu(command.inum), le32_to_cpu(command.name_size), le32_to_cpu(command.value_size), command.name,
		le32_to_cpu(command.flags), reserved);
	if (IS_ERR(xattr)) {
		ret = PTR_ERR(xattr);
		xattr = NULL;
		goto out;
	}

	value_size = le32_to_cpu(command.value_size);
	virtual_buffer = syno_cache_protection_passive_btrfs_virtual_buffer_alloc(value_size, reserved, SYNO_CACHE_PROTECTION_SPACE_POOL_DATA);
	if (IS_ERR(virtual_buffer)) {
		ret = PTR_ERR(virtual_buffer);
		virtual_buffer = NULL;
		goto out;
	}
	ret = syno_cache_protection_passive_btrfs_virtual_buffer_fill_from_request(req, virtual_buffer, 0, value_size);
	if (ret)
		goto out;
	ret = syno_cache_protection_passive_btrfs_buffer_insert(&xattr->node.extra_buffers,
		SYNO_CACHE_PROTECTION_PASSIVE_BTRFS_EXTRA_BUFFER_VIRTUAL_BUFFER, virtual_buffer);
	if (ret)
		goto out;
	xattr->value_size = value_size;
	xattr->value = virtual_buffer;

	spin_lock(&passive_instance->lock);
	list_add_tail(&xattr->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	ret = 0;
out:
	if (ret) {
		syno_cache_protection_passive_btrfs_virtual_buffer_free(virtual_buffer);
		if (xattr)
			syno_cache_protection_passive_btrfs_metadata_command_free(&xattr->node);
	}
	return ret;
}

static int btrfs_syno_cache_protection_command_xattr_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *   1 : mkfile command
	 *   1 : file name
	 * Data:
	 *   1 : virtual buffer
	 *   4 : xattr value
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 2, 5);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_xattr_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_xattr_size,
	.send = btrfs_syno_cache_protection_command_xattr_send,
	.receive = btrfs_syno_cache_protection_command_xattr_receive,
	.reserve = btrfs_syno_cache_protection_command_xattr_reserve,
};

static size_t btrfs_syno_cache_protection_command_subvol_operation_size(void *data)
{
	return sizeof(struct syno_cache_protection_stream_btrfs_command_subvol_operation);
}

static int btrfs_syno_cache_protection_command_subvol_operation_send(void *data, void *req)
{
	int ret;
	struct syno_cache_protection_parameter_command_subvol_operation *parm = (struct syno_cache_protection_parameter_command_subvol_operation*)data;
	struct syno_cache_protection_stream_btrfs_command_subvol_operation command;
	struct btrfs_root *root;

	if (!data || !req) {
		ret = -EINVAL;
		goto out;
	}

	root = BTRFS_I(parm->inode)->root;

	memset(&command, 0, sizeof(command));
	command.type = cpu_to_le32((int)parm->command);
	command.transid = cpu_to_le64(parm->transid);
	command.subvolid = cpu_to_le64(root->objectid);
	command.inum = cpu_to_le64(btrfs_ino(parm->inode));
	command.uid = cpu_to_le64(parm->uid);

	if (parm->qgroup_ca) {
		command.create = cpu_to_le64(parm->qgroup_ca->create);
		command.qgroupid = cpu_to_le64(parm->qgroup_ca->qgroupid);
	}
	if (parm->qgroup_aa) {
		command.assign = cpu_to_le64(parm->qgroup_aa->assign);
		command.src = cpu_to_le64(parm->qgroup_aa->src);
		command.dst = cpu_to_le64(parm->qgroup_aa->dst);
	}
	if (parm->qgroup_la) {
		command.qgroupid = cpu_to_le64(parm->qgroup_la->qgroupid);
		command.qgroup_limit.flags = cpu_to_le64(parm->qgroup_la->lim.flags);
		command.qgroup_limit.max_rfer = cpu_to_le64(parm->qgroup_la->lim.max_rfer);
		command.qgroup_limit.max_excl = cpu_to_le64(parm->qgroup_la->lim.max_excl);
		command.qgroup_limit.rsv_rfer = cpu_to_le64(parm->qgroup_la->lim.rsv_rfer);
		command.qgroup_limit.rsv_excl = cpu_to_le64(parm->qgroup_la->lim.rsv_excl);
	}
	if (parm->usrquota_la) {
		command.usrquota_limit.rfer_soft = cpu_to_le64(parm->usrquota_la->rfer_soft);
		command.usrquota_limit.rfer_hard = cpu_to_le64(parm->usrquota_la->rfer_hard);
	}

	ret = syno_cache_protection_write_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_subvol_operation_receive(void *private, void *req, bool reserved)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)private;
	struct syno_cache_protection_stream_btrfs_command_subvol_operation command;
	struct syno_cache_protection_passive_btrfs_subvol_operation *subvol_operation = NULL;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND type;

	if (!private || !req) {
		ret = -EINVAL;
		goto out;
	}

	ret = syno_cache_protection_read_request(req, sizeof(command), &command);
	if (ret)
		goto out;

	type = (enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND)le32_to_cpu(command.type);

	subvol_operation = syno_cache_protection_passive_btrfs_subvol_operation_alloc(type, le64_to_cpu(command.transid), le64_to_cpu(command.subvolid),
				le64_to_cpu(command.inum), le64_to_cpu(command.create), le64_to_cpu(command.qgroupid), le64_to_cpu(command.assign),
				le64_to_cpu(command.src), le64_to_cpu(command.dst), le64_to_cpu(command.uid), command.qgroup_limit, command.usrquota_limit, reserved);
	if (IS_ERR(subvol_operation)) {
		ret = PTR_ERR(subvol_operation);
		subvol_operation = NULL;
		goto out;
	}

	spin_lock(&passive_instance->lock);
	list_add_tail(&subvol_operation->node.list, &passive_instance->metadata_command_head);
	spin_unlock(&passive_instance->lock);

	ret = 0;
out:
	return ret;
}

static int btrfs_syno_cache_protection_command_subvol_operation_reserve(void *reserve_parm, void *data)
{
	/*
	 * Metadata:
	 *   1 : command
	 */
	return syno_cache_protection_command_generic_reserve(reserve_parm, 1, 0);
}

static const struct syno_cache_protection_btrfs_command_operations syno_cache_protection_btrfs_command_subvol_operation_ops = {
	.wait = true,
	.channel = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHANNEL_NOWAIT,
	.size = btrfs_syno_cache_protection_command_subvol_operation_size,
	.send = btrfs_syno_cache_protection_command_subvol_operation_send,
	.receive = btrfs_syno_cache_protection_command_subvol_operation_receive,
	.reserve = btrfs_syno_cache_protection_command_subvol_operation_reserve,
};

const struct syno_cache_protection_btrfs_command_operations *syno_cache_protection_btrfs_get_command_ops(enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command)
{
	const struct syno_cache_protection_btrfs_command_operations *ops = NULL;
	switch (command) {
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CHECKPOINT_END:
			ops = &syno_cache_protection_btrfs_command_checkpoint_end_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DATA_RECLAIM:
			ops = &syno_cache_protection_btrfs_command_data_reclaim_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_WRITE:
			ops = &syno_cache_protection_btrfs_command_write_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT:
			ops = &syno_cache_protection_btrfs_command_ordered_extent;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INLINE_EXTENT:
			ops = &syno_cache_protection_btrfs_command_inline_extent;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SPACE_RESERVE:
			ops = &syno_cache_protection_btrfs_command_space_reserve_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SPACE_RESERVE_FREE:
			ops = &syno_cache_protection_btrfs_command_space_reserve_free_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKFILE:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKNOD:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKDIR:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_LINK:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RMDIR:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_UNLINK:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SUBVOL_DELETE:
			ops = &syno_cache_protection_btrfs_command_create_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SYMLINK:
			ops = &syno_cache_protection_btrfs_command_symlink_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_FLAGS:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_UTIME:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHMODE:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHOWN:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_TRUNCATE:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DEFAULT_SUBVOL:
			ops = &syno_cache_protection_btrfs_command_inode_operation_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RENAME:
			ops = &syno_cache_protection_btrfs_command_rename_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CLONE:
			ops = &syno_cache_protection_btrfs_command_clone_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_REMOVEXATTR:
			ops = &syno_cache_protection_btrfs_command_xattr_ops;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_CREATE:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_ASSIGN:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_LIMIT:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_LIMIT:
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_CLEAN:
			ops = &syno_cache_protection_btrfs_command_subvol_operation_ops;
			break;
		default:
			pr_err("BTRFS: Cache Protection bug in unknown btrfs cmd %d\n", (int)command);
			break;
	}
	return ops;
}

