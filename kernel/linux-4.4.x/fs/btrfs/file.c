#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Copyright (C) 2007 Oracle.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License v2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 021110-1307, USA.
 */

#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/backing-dev.h>
#include <linux/mpage.h>
#include <linux/falloc.h>
#include <linux/swap.h>
#include <linux/writeback.h>
#include <linux/statfs.h>
#include <linux/compat.h>
#include <linux/slab.h>
#include <linux/btrfs.h>
#include <linux/uio.h>
#if defined(MY_DEF_HERE)
#ifdef CONFIG_SPLICE_FROM_SOCKET
#include <net/sock.h>
#endif
#endif /* MY_DEF_HERE */
#include "ctree.h"
#include "disk-io.h"
#include "transaction.h"
#include "btrfs_inode.h"
#include "print-tree.h"
#include "tree-log.h"
#include "locking.h"
#include "volumes.h"
#include "qgroup.h"
#include "compression.h"
#ifdef MY_ABC_HERE
#include "backref.h"
#endif /* MY_ABC_HERE */

static struct kmem_cache *btrfs_inode_defrag_cachep;
/*
 * when auto defrag is enabled we
 * queue up these defrag structs to remember which
 * inodes need defragging passes
 */
#ifdef MY_ABC_HERE
#define BTRFS_INODE_SYNO_DEFRAG_DELAY_GEN 5
#endif /* MY_ABC_HERE */
struct inode_defrag {
	struct rb_node rb_node;
	/* objectid */
	u64 ino;
	/*
	 * transid where the defrag was added, we search for
	 * extents newer than this
	 */
	u64 transid;

	/* root objectid */
	u64 root;

	/* last offset we were able to defrag */
	u64 last_offset;

#ifdef MY_ABC_HERE
	u64 end_offset;

	/* do normal or syno defrag */
	int defrag_type;

	struct list_head list;
#endif /* MY_ABC_HERE */

	/* if we've wrapped around back to zero once already */
	int cycled;
};

static int __compare_inode_defrag(struct inode_defrag *defrag1,
				  struct inode_defrag *defrag2)
{
	if (defrag1->root > defrag2->root)
		return 1;
	else if (defrag1->root < defrag2->root)
		return -1;
	else if (defrag1->ino > defrag2->ino)
		return 1;
	else if (defrag1->ino < defrag2->ino)
		return -1;
#ifdef MY_ABC_HERE
	else if (defrag1->defrag_type == defrag2->defrag_type &&
		     defrag1->defrag_type == BTRFS_INODE_DEFRAG_SYNO) {
		if (defrag1->last_offset > defrag2->end_offset)
			return 1;
		if (defrag1->end_offset < defrag2->last_offset)
			return -1;
		return 0;
	}
#endif /* MY_ABC_HERE */
	else
		return 0;
}

/* pop a record for an inode into the defrag tree.  The lock
 * must be held already
 *
 * If you're inserting a record for an older transid than an
 * existing record, the transid already in the tree is lowered
 *
 * If an existing record is found the defrag item you
 * pass in is freed
 */
static int __btrfs_add_inode_defrag(struct inode *inode,
				    struct inode_defrag *defrag)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct inode_defrag *entry;
	struct rb_node **p;
	struct rb_node *parent = NULL;
	int ret;

	p = &root->fs_info->defrag_inodes.rb_node;
	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct inode_defrag, rb_node);

		ret = __compare_inode_defrag(defrag, entry);
		if (ret < 0)
			p = &parent->rb_left;
		else if (ret > 0)
			p = &parent->rb_right;
		else {
			/* if we're reinserting an entry for
			 * an old defrag run, make sure to
			 * lower the transid of our existing record
			 */
#ifdef MY_ABC_HERE
			if (defrag->defrag_type != entry->defrag_type)
			/*
			 * We do not allow different defrag mode on same inode at the same time.
			 * Merge of different type is not allowed, either.
			 */
				return -EEXIST;
			if (defrag->defrag_type == BTRFS_INODE_DEFRAG_SYNO) {
				if (defrag->last_offset < entry->last_offset)
					entry->last_offset = defrag->last_offset;
				if (defrag->end_offset > entry->end_offset)
					entry->end_offset = defrag->end_offset;
				entry->transid = defrag->transid;
				list_move_tail(&entry->list, &root->fs_info->defrag_inodes_list[0]);
				return -EEXIST;
			}
#endif /* MY_ABC_HERE */
			if (defrag->transid < entry->transid)
				entry->transid = defrag->transid;
			if (defrag->last_offset > entry->last_offset)
				entry->last_offset = defrag->last_offset;
			return -EEXIST;
		}
	}
#ifdef MY_ABC_HERE
	if (defrag->defrag_type == BTRFS_INODE_DEFRAG_SYNO) {
		list_add_tail(&defrag->list, &root->fs_info->defrag_inodes_list[0]);
		root->fs_info->reclaim_space_entry_count++;
	} else {
		list_add(&defrag->list, &root->fs_info->defrag_inodes_list[1]);
		set_bit(BTRFS_INODE_IN_DEFRAG, &BTRFS_I(inode)->runtime_flags);
	}
#else
	set_bit(BTRFS_INODE_IN_DEFRAG, &BTRFS_I(inode)->runtime_flags);
#endif /* MY_ABC_HERE */
	rb_link_node(&defrag->rb_node, parent, p);
	rb_insert_color(&defrag->rb_node, &root->fs_info->defrag_inodes);
	return 0;
}

#ifdef MY_ABC_HERE
static inline int __need_auto_defrag(struct btrfs_root *root,
			   int defrag_type)
{
	if (btrfs_fs_closing(root->fs_info))
		return 0;

	if ((btrfs_test_opt(root, AUTO_DEFRAG) &&
		  (defrag_type & BTRFS_INODE_DEFRAG_NORMAL)) ||
		(btrfs_test_opt(root, AUTO_RECLAIM_SPACE) &&
		  (defrag_type & BTRFS_INODE_DEFRAG_SYNO)))
		return 1;

	return 0;
}
#else
static inline int __need_auto_defrag(struct btrfs_root *root)
{
	if (!btrfs_test_opt(root, AUTO_DEFRAG))
		return 0;

	if (btrfs_fs_closing(root->fs_info))
		return 0;

	return 1;
}
#endif /* MY_ABC_HERE */

/*
 * insert a defrag record for this inode if auto defrag is
 * enabled
 */
#ifdef MY_ABC_HERE
int btrfs_add_inode_defrag(struct btrfs_trans_handle *trans,
			   struct inode *inode, u64 start, u64 end,
			   int defrag_type)
#else
int btrfs_add_inode_defrag(struct btrfs_trans_handle *trans,
			   struct inode *inode)
#endif /* MY_ABC_HERE */
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct inode_defrag *defrag;
	u64 transid;
	int ret;

#ifdef MY_DEF_HERE
	if (root->fs_info->syno_cache_protection_recovering)
		return 0;
#endif /* MY_DEF_HERE */

#ifdef MY_ABC_HERE
	if (!__need_auto_defrag(root, defrag_type))
		return 0;

	if (defrag_type != BTRFS_INODE_DEFRAG_SYNO)
#else
	if (!__need_auto_defrag(root))
		return 0;
#endif /* MY_ABC_HERE */
	if (test_bit(BTRFS_INODE_IN_DEFRAG, &BTRFS_I(inode)->runtime_flags))
		return 0;

#ifdef MY_ABC_HERE
	/*
	 * Every 4K page can contains 40 entries(96 bytes/entry),
	 * 16MB should be able to accommodate 40*(16MB/4K) = 163840 entries.
	 */
	if (defrag_type == BTRFS_INODE_DEFRAG_SYNO &&
	    root->fs_info->reclaim_space_entry_count >= 163840) {
		return 0;
	}
#endif /* MY_ABC_HERE */
	if (trans)
		transid = trans->transid;
	else
		transid = BTRFS_I(inode)->root->last_trans;

	defrag = kmem_cache_zalloc(btrfs_inode_defrag_cachep, GFP_NOFS);
	if (!defrag)
		return -ENOMEM;

	defrag->ino = btrfs_ino(inode);
	defrag->transid = transid;
	defrag->root = root->root_key.objectid;
#ifdef MY_ABC_HERE
	defrag->last_offset = start;
	defrag->end_offset = end;
	defrag->defrag_type = defrag_type;
#endif /* MY_ABC_HERE */

	spin_lock(&root->fs_info->defrag_inodes_lock);
	if (!test_bit(BTRFS_INODE_IN_DEFRAG, &BTRFS_I(inode)->runtime_flags)) {
		/*
		 * If we set IN_DEFRAG flag and evict the inode from memory,
		 * and then re-read this inode, this new inode doesn't have
		 * IN_DEFRAG flag. At the case, we may find the existed defrag.
		 */
		ret = __btrfs_add_inode_defrag(inode, defrag);
		if (ret)
			kmem_cache_free(btrfs_inode_defrag_cachep, defrag);
	} else {
		kmem_cache_free(btrfs_inode_defrag_cachep, defrag);
	}
	spin_unlock(&root->fs_info->defrag_inodes_lock);
	return 0;
}

/*
 * Requeue the defrag object. If there is a defrag object that points to
 * the same inode in the tree, we will merge them together (by
 * __btrfs_add_inode_defrag()) and free the one that we want to requeue.
 */
static void btrfs_requeue_inode_defrag(struct inode *inode,
				       struct inode_defrag *defrag)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	int ret;

#ifdef MY_ABC_HERE
	if (!__need_auto_defrag(root, defrag->defrag_type))
#else
	if (!__need_auto_defrag(root))
#endif /* MY_ABC_HERE */
		goto out;

	/*
	 * Here we don't check the IN_DEFRAG flag, because we need merge
	 * them together.
	 */
	spin_lock(&root->fs_info->defrag_inodes_lock);
	ret = __btrfs_add_inode_defrag(inode, defrag);
	spin_unlock(&root->fs_info->defrag_inodes_lock);
	if (ret)
		goto out;
	return;
out:
	kmem_cache_free(btrfs_inode_defrag_cachep, defrag);
}

/*
 * pick the defragable inode that we want, if it doesn't exist, we will get
 * the next one.
 */
static struct inode_defrag *
#ifdef MY_ABC_HERE
btrfs_pick_defrag_inode(struct btrfs_fs_info *fs_info, struct list_head *cur_list)
#else
btrfs_pick_defrag_inode(struct btrfs_fs_info *fs_info, u64 root, u64 ino)
#endif /* MY_ABC_HERE */
{
	struct inode_defrag *entry = NULL;
#ifdef MY_ABC_HERE
	spin_lock(&fs_info->defrag_inodes_lock);
	entry = list_first_entry_or_null(cur_list, struct inode_defrag, list);
	if (!entry)
		goto out;
	if (entry->defrag_type == BTRFS_INODE_DEFRAG_NORMAL) {
		goto out;
	}
	if (entry->transid + BTRFS_INODE_SYNO_DEFRAG_DELAY_GEN >= fs_info->generation)
		entry = NULL;

out:
	if (entry) {
		rb_erase(&entry->rb_node, &fs_info->defrag_inodes);
		list_del(&entry->list);
		if (entry->defrag_type == BTRFS_INODE_DEFRAG_SYNO) {
			fs_info->reclaim_space_entry_count--;
		}
	}
	spin_unlock(&fs_info->defrag_inodes_lock);
	return entry;
#else
	struct inode_defrag tmp;
	struct rb_node *p;
	struct rb_node *parent = NULL;
	int ret;

	tmp.ino = ino;
	tmp.root = root;

	spin_lock(&fs_info->defrag_inodes_lock);
	p = fs_info->defrag_inodes.rb_node;
	while (p) {
		parent = p;
		entry = rb_entry(parent, struct inode_defrag, rb_node);

		ret = __compare_inode_defrag(&tmp, entry);
		if (ret < 0)
			p = parent->rb_left;
		else if (ret > 0)
			p = parent->rb_right;
		else
			goto out;
	}

	if (parent && __compare_inode_defrag(&tmp, entry) > 0) {
		parent = rb_next(parent);
		if (parent)
			entry = rb_entry(parent, struct inode_defrag, rb_node);
		else
			entry = NULL;
	}
out:
	if (entry)
		rb_erase(parent, &fs_info->defrag_inodes);
	spin_unlock(&fs_info->defrag_inodes_lock);
	return entry;
#endif /* MY_ABC_HERE */
}

void btrfs_cleanup_defrag_inodes(struct btrfs_fs_info *fs_info)
{
	struct inode_defrag *defrag;
	struct rb_node *node;

	spin_lock(&fs_info->defrag_inodes_lock);
	node = rb_first(&fs_info->defrag_inodes);
	while (node) {
		rb_erase(node, &fs_info->defrag_inodes);
		defrag = rb_entry(node, struct inode_defrag, rb_node);
#ifdef MY_ABC_HERE
		list_del(&defrag->list);
		if (defrag->defrag_type == BTRFS_INODE_DEFRAG_SYNO)
			fs_info->reclaim_space_entry_count--;
#endif /* MY_ABC_HERE */
		kmem_cache_free(btrfs_inode_defrag_cachep, defrag);

		cond_resched_lock(&fs_info->defrag_inodes_lock);

		node = rb_first(&fs_info->defrag_inodes);
	}
	spin_unlock(&fs_info->defrag_inodes_lock);
}

#define BTRFS_DEFRAG_BATCH	1024

static int __btrfs_run_defrag_inode(struct btrfs_fs_info *fs_info,
				    struct inode_defrag *defrag)
{
	struct btrfs_root *inode_root;
	struct inode *inode;
	struct btrfs_key key;
	struct btrfs_ioctl_defrag_range_args range;
	int num_defrag;
	int index;
	int ret;
#ifdef MY_ABC_HERE
	unsigned long max_to_defrag = BTRFS_DEFRAG_BATCH;
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	int srcu_unlock = 1;
#endif /* MY_ABC_HERE */

	/* get the inode */
	key.objectid = defrag->root;
	key.type = BTRFS_ROOT_ITEM_KEY;
	key.offset = (u64)-1;

	index = srcu_read_lock(&fs_info->subvol_srcu);

	inode_root = btrfs_read_fs_root_no_name(fs_info, &key);
	if (IS_ERR(inode_root)) {
		ret = PTR_ERR(inode_root);
		goto cleanup;
	}
#ifdef MY_ABC_HERE
	btrfs_hold_fs_root(inode_root);
	srcu_read_unlock(&fs_info->subvol_srcu, index);
	srcu_unlock = 0;
#endif /* MY_ABC_HERE */

	key.objectid = defrag->ino;
	key.type = BTRFS_INODE_ITEM_KEY;
	key.offset = 0;
	inode = btrfs_iget(fs_info->sb, &key, inode_root, NULL);
#ifdef MY_ABC_HERE
	btrfs_release_fs_root(inode_root);
#endif /* MY_ABC_HERE */
	if (IS_ERR(inode)) {
		ret = PTR_ERR(inode);
		goto cleanup;
	}
#ifdef MY_ABC_HERE
#else
	srcu_read_unlock(&fs_info->subvol_srcu, index);
#endif /* MY_ABC_HERE */

	/* do a chunk of defrag */
	clear_bit(BTRFS_INODE_IN_DEFRAG, &BTRFS_I(inode)->runtime_flags);
	memset(&range, 0, sizeof(range));
	range.len = (u64)-1;
	range.start = defrag->last_offset;
#ifdef MY_ABC_HERE
	if (defrag->defrag_type == BTRFS_INODE_DEFRAG_SYNO) {
		range.flags |= BTRFS_DEFRAG_RANGE_SYNO_DEFRAG;
		range.len = defrag->end_offset - defrag->last_offset;
		defrag->transid = 0;
		max_to_defrag = 0;
	}

	sb_start_write(fs_info->sb);
	num_defrag = btrfs_defrag_file(inode, NULL, &range, defrag->transid,
				       max_to_defrag);
	sb_end_write(fs_info->sb);
#else

	sb_start_write(fs_info->sb);
	num_defrag = btrfs_defrag_file(inode, NULL, &range, defrag->transid,
				       BTRFS_DEFRAG_BATCH);
	sb_end_write(fs_info->sb);
#endif /* MY_ABC_HERE */
	/*
	 * if we filled the whole defrag batch, there
	 * must be more work to do.  Queue this defrag
	 * again
	 */
#ifdef MY_ABC_HERE
	if (defrag->defrag_type == BTRFS_INODE_DEFRAG_SYNO) {
		kmem_cache_free(btrfs_inode_defrag_cachep, defrag);
	} else
#endif /* MY_ABC_HERE */
	if (num_defrag == BTRFS_DEFRAG_BATCH) {
		defrag->last_offset = range.start;
		btrfs_requeue_inode_defrag(inode, defrag);
	} else if (defrag->last_offset && !defrag->cycled) {
		/*
		 * we didn't fill our defrag batch, but
		 * we didn't start at zero.  Make sure we loop
		 * around to the start of the file.
		 */
		defrag->last_offset = 0;
		defrag->cycled = 1;
		btrfs_requeue_inode_defrag(inode, defrag);
	} else {
		kmem_cache_free(btrfs_inode_defrag_cachep, defrag);
	}

	iput(inode);
	return 0;
cleanup:
#ifdef MY_ABC_HERE
	if (srcu_unlock)
#endif /* MY_ABC_HERE */
	srcu_read_unlock(&fs_info->subvol_srcu, index);
	kmem_cache_free(btrfs_inode_defrag_cachep, defrag);
	return ret;
}

/*
 * run through the list of inodes in the FS that need
 * defragging
 */
int btrfs_run_defrag_inodes(struct btrfs_fs_info *fs_info)
{
	struct inode_defrag *defrag;
#ifdef MY_ABC_HERE
	int list_idx = 0;
	struct list_head *cur_list = &fs_info->defrag_inodes_list[list_idx];
	int last_null = 0;
#else
	u64 first_ino = 0;
	u64 root_objectid = 0;
#endif /* MY_ABC_HERE */

	atomic_inc(&fs_info->defrag_running);
	while (1) {
		/* Pause the auto defragger. */
		if (test_bit(BTRFS_FS_STATE_REMOUNTING,
			     &fs_info->fs_state))
			break;

#ifdef MY_ABC_HERE
		if (!__need_auto_defrag(fs_info->tree_root,
			    BTRFS_INODE_DEFRAG_NORMAL | BTRFS_INODE_DEFRAG_SYNO))
			break;

		/* find an inode to defrag */
		defrag = btrfs_pick_defrag_inode(fs_info, cur_list);
		list_idx = (list_idx + 1) % 2;
		cur_list = &fs_info->defrag_inodes_list[list_idx];
		if (!defrag) {
			if (last_null)
				break;
			last_null = 1;
			continue;
		}
		last_null = 0;
#else
		if (!__need_auto_defrag(fs_info->tree_root))
			break;

		/* find an inode to defrag */
		defrag = btrfs_pick_defrag_inode(fs_info, root_objectid,
						 first_ino);
		if (!defrag) {
			if (root_objectid || first_ino) {
				root_objectid = 0;
				first_ino = 0;
				continue;
			} else {
				break;
			}
		}

		first_ino = defrag->ino + 1;
		root_objectid = defrag->root;
#endif /* MY_ABC_HERE */

		__btrfs_run_defrag_inode(fs_info, defrag);
	}
	atomic_dec(&fs_info->defrag_running);

	/*
	 * during unmount, we use the transaction_wait queue to
	 * wait for the defragger to stop
	 */
	wake_up(&fs_info->transaction_wait);
	return 0;
}

/* simple helper to fault in pages and copy.  This should go away
 * and be replaced with calls into generic code.
 */
static noinline int btrfs_copy_from_user(loff_t pos, size_t write_bytes,
					 struct page **prepared_pages,
					 struct iov_iter *i)
{
	size_t copied = 0;
	size_t total_copied = 0;
	int pg = 0;
	int offset = pos & (PAGE_CACHE_SIZE - 1);

	while (write_bytes > 0) {
		size_t count = min_t(size_t,
				     PAGE_CACHE_SIZE - offset, write_bytes);
		struct page *page = prepared_pages[pg];
		/*
		 * Copy data from userspace to the current page
		 */
		copied = iov_iter_copy_from_user_atomic(page, i, offset, count);

		/* Flush processor's dcache for this page */
		flush_dcache_page(page);

		/*
		 * if we get a partial write, we can end up with
		 * partially up to date pages.  These add
		 * a lot of complexity, so make sure they don't
		 * happen by forcing this copy to be retried.
		 *
		 * The rest of the btrfs_file_write code will fall
		 * back to page at a time copies after we return 0.
		 */
		if (!PageUptodate(page) && copied < count)
			copied = 0;

		iov_iter_advance(i, copied);
		write_bytes -= copied;
		total_copied += copied;

		/* Return to btrfs_file_write_iter to fault page */
		if (unlikely(copied == 0))
			break;

		if (copied < PAGE_CACHE_SIZE - offset) {
			offset += copied;
		} else {
			pg++;
			offset = 0;
		}
	}
	return total_copied;
}

/*
 * unlocks pages after btrfs_file_write is done with them
 */
static void btrfs_drop_pages(struct page **pages, size_t num_pages)
{
	size_t i;
	for (i = 0; i < num_pages; i++) {
		/* page checked is some magic around finding pages that
		 * have been modified without going through btrfs_set_page_dirty
		 * clear it here. There should be no need to mark the pages
		 * accessed as prepare_pages should have marked them accessed
		 * in prepare_pages via find_or_create_page()
		 */
		ClearPageChecked(pages[i]);
		unlock_page(pages[i]);
		page_cache_release(pages[i]);
	}
}

/*
 * after copy_from_user, pages need to be dirtied and we need to make
 * sure holes are created between the current EOF and the start of
 * any next extents (if required).
 *
 * this also makes the decision about creating an inline extent vs
 * doing real data extents, marking pages dirty and delalloc as required.
 */
int btrfs_dirty_pages(struct btrfs_root *root, struct inode *inode,
			     struct page **pages, size_t num_pages,
			     loff_t pos, size_t write_bytes,
			     struct extent_state **cached)
{
	int err = 0;
	int i;
	u64 num_bytes;
	u64 start_pos;
	u64 end_of_last_block;
	u64 end_pos = pos + write_bytes;
	loff_t isize = i_size_read(inode);
	unsigned int extra_bits = 0;

	start_pos = pos & ~((u64)root->sectorsize - 1);
	num_bytes = ALIGN(write_bytes + pos - start_pos, root->sectorsize);

	end_of_last_block = start_pos + num_bytes - 1;

	/*
	 * The pages may have already been dirty, clear out old accounting so
	 * we can set things up properly
	 */
	clear_extent_bit(&BTRFS_I(inode)->io_tree, start_pos, end_of_last_block,
			 EXTENT_DIRTY | EXTENT_DELALLOC |
			 EXTENT_DO_ACCOUNTING | EXTENT_DEFRAG, 0, 0, cached, GFP_NOFS);

	err = btrfs_set_extent_delalloc(inode, start_pos, end_of_last_block,
					extra_bits, cached);
	if (err)
		return err;

	for (i = 0; i < num_pages; i++) {
		struct page *p = pages[i];
		SetPageUptodate(p);
		ClearPageChecked(p);
		set_page_dirty(p);
	}

	/*
	 * we've only changed i_size in ram, and we haven't updated
	 * the disk i_size.  There is no need to log the inode
	 * at this time.
	 */
	if (end_pos > isize)
		i_size_write(inode, end_pos);
	return 0;
}

/*
 * this drops all the extents in the cache that intersect the range
 * [start, end].  Existing extents are split as required.
 */
void btrfs_drop_extent_cache(struct inode *inode, u64 start, u64 end,
			     int skip_pinned)
{
	struct extent_map *em;
	struct extent_map *split = NULL;
	struct extent_map *split2 = NULL;
	struct extent_map_tree *em_tree = &BTRFS_I(inode)->extent_tree;
	u64 len = end - start + 1;
	u64 gen;
	int ret;
	int testend = 1;
	unsigned long flags;
	int compressed = 0;
	bool modified;

	WARN_ON(end < start);
	if (end == (u64)-1) {
		len = (u64)-1;
		testend = 0;
	}
	while (1) {
		int no_splits = 0;

		modified = false;
		if (!split)
			split = alloc_extent_map();
		if (!split2)
			split2 = alloc_extent_map();
		if (!split || !split2)
			no_splits = 1;

		write_lock(&em_tree->lock);
		em = lookup_extent_mapping(em_tree, start, len);
		if (!em) {
			write_unlock(&em_tree->lock);
			break;
		}
		flags = em->flags;
		gen = em->generation;
		if (skip_pinned && test_bit(EXTENT_FLAG_PINNED, &em->flags)) {
			if (testend && em->start + em->len >= start + len) {
				free_extent_map(em);
				write_unlock(&em_tree->lock);
				break;
			}
			start = em->start + em->len;
			if (testend)
				len = start + len - (em->start + em->len);
			free_extent_map(em);
			write_unlock(&em_tree->lock);
			continue;
		}
		compressed = test_bit(EXTENT_FLAG_COMPRESSED, &em->flags);
		clear_bit(EXTENT_FLAG_PINNED, &em->flags);
		clear_bit(EXTENT_FLAG_LOGGING, &flags);
		modified = !list_empty(&em->list);
		if (no_splits)
			goto next;

		if (em->start < start) {
			split->start = em->start;
			split->len = start - em->start;

			if (em->block_start < EXTENT_MAP_LAST_BYTE) {
				split->orig_start = em->orig_start;
				split->block_start = em->block_start;

				if (compressed)
					split->block_len = em->block_len;
				else
					split->block_len = split->len;
				split->orig_block_len = max(split->block_len,
						em->orig_block_len);
				split->ram_bytes = em->ram_bytes;
			} else {
				split->orig_start = split->start;
				split->block_len = 0;
				split->block_start = em->block_start;
				split->orig_block_len = 0;
				split->ram_bytes = split->len;
			}

			split->generation = gen;
			split->bdev = em->bdev;
			split->flags = flags;
			split->compress_type = em->compress_type;
			replace_extent_mapping(em_tree, em, split, modified);
			free_extent_map(split);
			split = split2;
			split2 = NULL;
		}
		if (testend && em->start + em->len > start + len) {
			u64 diff = start + len - em->start;

			split->start = start + len;
			split->len = em->start + em->len - (start + len);
			split->bdev = em->bdev;
			split->flags = flags;
			split->compress_type = em->compress_type;
			split->generation = gen;

			if (em->block_start < EXTENT_MAP_LAST_BYTE) {
				split->orig_block_len = max(em->block_len,
						    em->orig_block_len);

				split->ram_bytes = em->ram_bytes;
				if (compressed) {
					split->block_len = em->block_len;
					split->block_start = em->block_start;
					split->orig_start = em->orig_start;
				} else {
					split->block_len = split->len;
					split->block_start = em->block_start
						+ diff;
					split->orig_start = em->orig_start;
				}
			} else {
				split->ram_bytes = split->len;
				split->orig_start = split->start;
				split->block_len = 0;
				split->block_start = em->block_start;
				split->orig_block_len = 0;
			}

			if (extent_map_in_tree(em)) {
				replace_extent_mapping(em_tree, em, split,
						       modified);
			} else {
				ret = add_extent_mapping(em_tree, split,
							 modified);
				ASSERT(ret == 0); /* Logic error */
			}
			free_extent_map(split);
			split = NULL;
		}
next:
		if (extent_map_in_tree(em))
			remove_extent_mapping(em_tree, em);
		write_unlock(&em_tree->lock);

		/* once for us */
		free_extent_map(em);
		/* once for the tree*/
		free_extent_map(em);
	}
	if (split)
		free_extent_map(split);
	if (split2)
		free_extent_map(split2);
}

/*
 * this is very complex, but the basic idea is to drop all extents
 * in the range start - end.  hint_block is filled in with a block number
 * that would be a good hint to the block allocator for this file.
 *
 * If an extent intersects the range but is not entirely inside the range
 * it is either truncated or split.  Anything entirely inside the range
 * is deleted from the tree.
 *
 * Note: the VFS' inode number of bytes is not updated, it's up to the caller
 * to deal with that. We set the field 'bytes_found' of the arguments structure
 * with the number of allocated bytes found in the target range, so that the
 * caller can update the inode's number of bytes in an atomic way when
 * replacing extents in a range to avoid races with stat(2).
 */
int btrfs_drop_extents(struct btrfs_trans_handle *trans,
			struct btrfs_root *root, struct inode *inode,
			struct btrfs_drop_extents_args *args)
{
	struct extent_buffer *leaf;
	struct btrfs_file_extent_item *fi;
	struct btrfs_key key;
	struct btrfs_key new_key;
	u64 ino = btrfs_ino(inode);
	u64 search_start = args->start;
	u64 disk_bytenr = 0;
	u64 num_bytes = 0;
#ifdef MY_ABC_HERE
	u64 ram_bytes = 0;
#endif /* MY_ABC_HERE */
	u64 extent_offset = 0;
	u64 extent_end = 0;
	u64 last_end = args->start;
	int del_nr = 0;
	int del_slot = 0;
	int extent_type;
	int recow;
	int ret;
	int modify_tree = -1;
	int update_refs;
	int found = 0;
	int leafs_visited = 0;
#ifdef MY_ABC_HERE
	u64 relative_offset;
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	bool should_throttle = false;
#endif /* MY_ABC_HERE */
	struct btrfs_path *path = args->path;

	args->bytes_found = 0;
	args->extent_inserted = false;

	/* Must always have a path if ->replace_extent is true */
	ASSERT(!(args->replace_extent && !args->path));

	if (!path) {
		path = btrfs_alloc_path();
		if (!path) {
			ret = -ENOMEM;
			goto out;
		}
	}

	if (args->drop_cache)
		btrfs_drop_extent_cache(inode, args->start, args->end - 1, 0);

	if (args->start >= BTRFS_I(inode)->disk_i_size && !args->replace_extent)
		modify_tree = 0;

	update_refs = (test_bit(BTRFS_ROOT_REF_COWS, &root->state) ||
		       root == root->fs_info->tree_root);
	while (1) {
		recow = 0;
		ret = btrfs_lookup_file_extent(trans, root, path, ino,
					       search_start, modify_tree);
		if (ret < 0)
			break;
		if (ret > 0 && path->slots[0] > 0 && search_start == args->start) {
			leaf = path->nodes[0];
			btrfs_item_key_to_cpu(leaf, &key, path->slots[0] - 1);
			if (key.objectid == ino &&
			    key.type == BTRFS_EXTENT_DATA_KEY)
				path->slots[0]--;
		}
		ret = 0;
		leafs_visited++;
next_slot:
		leaf = path->nodes[0];
		if (path->slots[0] >= btrfs_header_nritems(leaf)) {
			BUG_ON(del_nr > 0);
			ret = btrfs_next_leaf(root, path);
			if (ret < 0)
				break;
			if (ret > 0) {
				ret = 0;
				break;
			}
			leafs_visited++;
			leaf = path->nodes[0];
			recow = 1;
		}

		btrfs_item_key_to_cpu(leaf, &key, path->slots[0]);

		if (key.objectid > ino)
			break;
		if (WARN_ON_ONCE(key.objectid < ino) ||
		    key.type < BTRFS_EXTENT_DATA_KEY) {
			ASSERT(del_nr == 0);
			path->slots[0]++;
			goto next_slot;
		}
		if (key.type > BTRFS_EXTENT_DATA_KEY || key.offset >= args->end)
			break;

		fi = btrfs_item_ptr(leaf, path->slots[0],
				    struct btrfs_file_extent_item);
		extent_type = btrfs_file_extent_type(leaf, fi);

		if (extent_type == BTRFS_FILE_EXTENT_REG ||
		    extent_type == BTRFS_FILE_EXTENT_PREALLOC) {
			disk_bytenr = btrfs_file_extent_disk_bytenr(leaf, fi);
			num_bytes = btrfs_file_extent_disk_num_bytes(leaf, fi);
#ifdef MY_ABC_HERE
			ram_bytes = btrfs_file_extent_ram_bytes(leaf, fi);
#endif /* MY_ABC_HERE */
			extent_offset = btrfs_file_extent_offset(leaf, fi);
			extent_end = key.offset +
				btrfs_file_extent_num_bytes(leaf, fi);
		} else if (extent_type == BTRFS_FILE_EXTENT_INLINE) {
			extent_end = key.offset +
				btrfs_file_extent_inline_len(leaf,
						     path->slots[0], fi);
		} else {
			/* can't happen */
			BUG();
		}

		/*
		 * Don't skip extent items representing 0 byte lengths. They
		 * used to be created (bug) if while punching holes we hit
		 * -ENOSPC condition. So if we find one here, just ensure we
		 * delete it, otherwise we would insert a new file extent item
		 * with the same key (offset) as that 0 bytes length file
		 * extent item in the call to setup_items_for_insert() later
		 * in this function.
		 */
		if (extent_end == key.offset && extent_end >= search_start) {
			last_end = extent_end;
			goto delete_extent_item;
		}

		if (extent_end <= search_start) {
			path->slots[0]++;
			goto next_slot;
		}

		found = 1;
		search_start = max(key.offset, args->start);
		if (recow || !modify_tree) {
			modify_tree = -1;
			btrfs_release_path(path);
			continue;
		}

		/*
		 *     | - range to drop - |
		 *  | -------- extent -------- |
		 */
		if (args->start > key.offset && args->end < extent_end) {
			BUG_ON(del_nr > 0);
			if (extent_type == BTRFS_FILE_EXTENT_INLINE) {
				ret = -EOPNOTSUPP;
				break;
			}

			memcpy(&new_key, &key, sizeof(new_key));
			new_key.offset = args->start;
			ret = btrfs_duplicate_item(trans, root, path,
						   &new_key);
			if (ret == -EAGAIN) {
				btrfs_release_path(path);
				continue;
			}
			if (ret < 0)
				break;

			leaf = path->nodes[0];
			fi = btrfs_item_ptr(leaf, path->slots[0] - 1,
					    struct btrfs_file_extent_item);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							args->start - key.offset);

			fi = btrfs_item_ptr(leaf, path->slots[0],
					    struct btrfs_file_extent_item);

			extent_offset += args->start - key.offset;
			btrfs_set_file_extent_offset(leaf, fi, extent_offset);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							extent_end - args->start);
			btrfs_mark_buffer_dirty(leaf);

			if (update_refs && disk_bytenr > 0) {
				ret = btrfs_inc_extent_ref(trans, root,
						disk_bytenr, num_bytes, 0,
						root->root_key.objectid,
						new_key.objectid,
						args->start - extent_offset
#ifdef MY_ABC_HERE
						,1, 0
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
						,btrfs_syno_usage_ref_check(root, new_key.objectid, args->end)
#endif /* MY_ABC_HERE */
						);
				BUG_ON(ret); /* -ENOMEM */
			}
			key.offset = args->start;
		}
		/*
		 * From here on out we will have actually dropped something, so
		 * last_end can be updated.
		 */
		last_end = extent_end;

		/*
		 *  | ---- range to drop ----- |
		 *      | -------- extent -------- |
		 */
		if (args->start <= key.offset && args->end < extent_end) {
			if (extent_type == BTRFS_FILE_EXTENT_INLINE) {
				ret = -EOPNOTSUPP;
				break;
			}

#ifdef MY_ABC_HERE
			relative_offset = key.offset - extent_offset;
			if (relative_offset >= LLONG_MAX)
				relative_offset = 0;
			if (args->first_punch_pos && relative_offset < *(args->first_punch_pos))
				*(args->first_punch_pos) = relative_offset;
			if (args->last_punch_pos &&
			    relative_offset + num_bytes > *(args->last_punch_pos))
				*(args->last_punch_pos) = relative_offset + num_bytes;
			if (args->partial_punch)
				*(args->partial_punch) = 1;
#endif /* MY_ABC_HERE */

			memcpy(&new_key, &key, sizeof(new_key));
			new_key.offset = args->end;
			btrfs_set_item_key_safe(root->fs_info, path, &new_key);

			extent_offset += args->end - key.offset;
			btrfs_set_file_extent_offset(leaf, fi, extent_offset);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							extent_end - args->end);
			btrfs_mark_buffer_dirty(leaf);
			if (update_refs && disk_bytenr > 0)
				args->bytes_found += args->end - key.offset;
			break;
		}

		search_start = extent_end;
		/*
		 *       | ---- range to drop ----- |
		 *  | -------- extent -------- |
		 */
		if (args->start > key.offset && args->end >= extent_end) {
			BUG_ON(del_nr > 0);
			if (extent_type == BTRFS_FILE_EXTENT_INLINE) {
				ret = -EOPNOTSUPP;
				break;
			}

			btrfs_set_file_extent_num_bytes(leaf, fi,
							args->start - key.offset);
			btrfs_mark_buffer_dirty(leaf);
#ifdef MY_ABC_HERE
			relative_offset = key.offset - extent_offset;
			if (relative_offset >= LLONG_MAX)
				relative_offset = 0;
			if (args->first_punch_pos && relative_offset < *(args->first_punch_pos))
				*(args->first_punch_pos) = relative_offset;
			if (args->last_punch_pos &&
			    relative_offset + num_bytes > *(args->last_punch_pos))
				*(args->last_punch_pos) = relative_offset + num_bytes;
			if (args->partial_punch)
				*(args->partial_punch) = 1;
#endif /* MY_ABC_HERE */
			if (update_refs && disk_bytenr > 0)
				args->bytes_found += extent_end - args->start;
			if (args->end == extent_end)
				break;

			path->slots[0]++;
			goto next_slot;
		}

		/*
		 *  | ---- range to drop ----- |
		 *    | ------ extent ------ |
		 */
		if (args->start <= key.offset && args->end >= extent_end) {
delete_extent_item:
			if (del_nr == 0) {
				del_slot = path->slots[0];
				del_nr = 1;
			} else {
				BUG_ON(del_slot + del_nr != path->slots[0]);
				del_nr++;
			}

			if (update_refs &&
			    extent_type == BTRFS_FILE_EXTENT_INLINE) {
				args->bytes_found += extent_end - key.offset;
				extent_end = ALIGN(extent_end,
						   root->sectorsize);
			} else if (update_refs && disk_bytenr > 0) {
#ifdef MY_ABC_HERE
				ret = btrfs_free_extent_uid(trans, root,
						disk_bytenr, num_bytes, 0,
						root->root_key.objectid,
						key.objectid, key.offset -
						extent_offset, 0, ram_bytes,
						inode, i_uid_read(inode)
#ifdef MY_ABC_HERE
						,btrfs_syno_usage_ref_check(root, key.objectid, key.offset)
#endif /* MY_ABC_HERE */
						);
#else
				ret = btrfs_free_extent(trans, root,
						disk_bytenr, num_bytes, 0,
						root->root_key.objectid,
						key.objectid, key.offset - extent_offset
#ifdef MY_ABC_HERE
						,0, ram_bytes
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
						,btrfs_syno_usage_ref_check(root, key.objectid, key.offset)
#endif /* MY_ABC_HERE */
						);
#endif /* MY_ABC_HERE */
				BUG_ON(ret); /* -ENOMEM */
#ifdef MY_ABC_HERE
				relative_offset = key.offset - extent_offset;
				if (args->partial_punch &&
				    extent_end - key.offset != num_bytes) {
					*(args->partial_punch) = 1;
					if (relative_offset >= LLONG_MAX)
						relative_offset = 0;
					if (args->first_punch_pos &&
					    relative_offset < *(args->first_punch_pos))
						*(args->first_punch_pos) = relative_offset;
					if (args->last_punch_pos &&
					    relative_offset + num_bytes > *(args->last_punch_pos))
						*(args->last_punch_pos) = relative_offset + num_bytes;
				}
#endif /* MY_ABC_HERE */
				args->bytes_found += extent_end - key.offset;
#ifdef MY_ABC_HERE
				if (trans->check_throttle && btrfs_should_throttle_delayed_refs(trans, root) == 1) {
					should_throttle = true;
				}
#endif /* MY_ABC_HERE */
			}

			if (args->end == extent_end)
				break;

#ifdef MY_ABC_HERE
			if (!should_throttle && path->slots[0] + 1 < btrfs_header_nritems(leaf)) {
#else
			if (path->slots[0] + 1 < btrfs_header_nritems(leaf)) {
#endif /* MY_ABC_HERE */
				path->slots[0]++;
				goto next_slot;
			}

			ret = btrfs_del_items(trans, root, path, del_slot,
					      del_nr);
			if (ret) {
				btrfs_abort_transaction(trans, root, ret);
				break;
			}

			del_nr = 0;
			del_slot = 0;

			btrfs_release_path(path);
#ifdef MY_ABC_HERE
			if (should_throttle) {
				ret = -EAGAIN;
				break;
			}
#endif /* MY_ABC_HERE */
			continue;
		}

		BUG_ON(1);
	}

	if (!ret && del_nr > 0) {
		/*
		 * Set path->slots[0] to first slot, so that after the delete
		 * if items are move off from our leaf to its immediate left or
		 * right neighbor leafs, we end up with a correct and adjusted
		 * path->slots[0] for our insertion (if args->replace_extent).
		 */
		path->slots[0] = del_slot;
		ret = btrfs_del_items(trans, root, path, del_slot, del_nr);
		if (ret)
			btrfs_abort_transaction(trans, root, ret);
	}

	leaf = path->nodes[0];
	/*
	 * If btrfs_del_items() was called, it might have deleted a leaf, in
	 * which case it unlocked our path, so check path->locks[0] matches a
	 * write lock.
	 */
	if (!ret && args->replace_extent && leafs_visited == 1 &&
	    (path->locks[0] == BTRFS_WRITE_LOCK_BLOCKING ||
	     path->locks[0] == BTRFS_WRITE_LOCK) &&
	    btrfs_leaf_free_space(root, leaf) >=
	    sizeof(struct btrfs_item) + args->extent_item_size) {

		key.objectid = ino;
		key.type = BTRFS_EXTENT_DATA_KEY;
		key.offset = args->start;
		if (!del_nr && path->slots[0] < btrfs_header_nritems(leaf)) {
			struct btrfs_key slot_key;

			btrfs_item_key_to_cpu(leaf, &slot_key, path->slots[0]);
			if (btrfs_comp_cpu_keys(&key, &slot_key) > 0)
				path->slots[0]++;
		}
		setup_items_for_insert(root, path, &key,
				       &args->extent_item_size,
				       args->extent_item_size,
				       sizeof(struct btrfs_item) +
				       args->extent_item_size, 1);
		args->extent_inserted = true;
	}

	if (!args->path)
		btrfs_free_path(path);
	else if (!args->extent_inserted)
		btrfs_release_path(path);

out:
	args->drop_end = found ? min(args->end, last_end) : args->end;
	return ret;
}

static int extent_mergeable(struct extent_buffer *leaf, int slot,
			    u64 objectid, u64 bytenr, u64 orig_offset,
			    u64 *start, u64 *end)
{
	struct btrfs_file_extent_item *fi;
	struct btrfs_key key;
	u64 extent_end;

	if (slot < 0 || slot >= btrfs_header_nritems(leaf))
		return 0;

	btrfs_item_key_to_cpu(leaf, &key, slot);
	if (key.objectid != objectid || key.type != BTRFS_EXTENT_DATA_KEY)
		return 0;

	fi = btrfs_item_ptr(leaf, slot, struct btrfs_file_extent_item);
	if (btrfs_file_extent_type(leaf, fi) != BTRFS_FILE_EXTENT_REG ||
	    btrfs_file_extent_disk_bytenr(leaf, fi) != bytenr ||
	    btrfs_file_extent_offset(leaf, fi) != key.offset - orig_offset ||
	    btrfs_file_extent_compression(leaf, fi) ||
	    btrfs_file_extent_encryption(leaf, fi) ||
	    btrfs_file_extent_other_encoding(leaf, fi))
		return 0;

	extent_end = key.offset + btrfs_file_extent_num_bytes(leaf, fi);
	if ((*start && *start != key.offset) || (*end && *end != extent_end))
		return 0;

	*start = key.offset;
	*end = extent_end;
	return 1;
}

/*
 * Mark extent in the range start - end as written.
 *
 * This changes extent type from 'pre-allocated' to 'regular'. If only
 * part of extent is marked as written, the extent will be split into
 * two or three.
 */
int btrfs_mark_extent_written(struct btrfs_trans_handle *trans,
			      struct inode *inode, u64 start, u64 end)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct extent_buffer *leaf;
	struct btrfs_path *path;
	struct btrfs_file_extent_item *fi;
	struct btrfs_key key;
	struct btrfs_key new_key;
	u64 bytenr;
	u64 num_bytes;
#ifdef MY_ABC_HERE
	u64 ram_bytes;
#endif /* MY_ABC_HERE */
	u64 extent_end;
	u64 orig_offset;
	u64 other_start;
	u64 other_end;
	u64 split;
	int del_nr = 0;
	int del_slot = 0;
	int recow;
	int ret = 0;
	u64 ino = btrfs_ino(inode);
#ifdef MY_ABC_HERE
	struct btrfs_key syno_usage_key;
#endif /* MY_ABC_HERE */

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;
again:
	recow = 0;
	split = start;
	key.objectid = ino;
	key.type = BTRFS_EXTENT_DATA_KEY;
	key.offset = split;

	ret = btrfs_search_slot(trans, root, &key, path, -1, 1);
	if (ret < 0)
		goto out;
	if (ret > 0 && path->slots[0] > 0)
		path->slots[0]--;

	leaf = path->nodes[0];
	btrfs_item_key_to_cpu(leaf, &key, path->slots[0]);
	BUG_ON(key.objectid != ino || key.type != BTRFS_EXTENT_DATA_KEY);
	fi = btrfs_item_ptr(leaf, path->slots[0],
			    struct btrfs_file_extent_item);
	BUG_ON(btrfs_file_extent_type(leaf, fi) !=
	       BTRFS_FILE_EXTENT_PREALLOC);
	extent_end = key.offset + btrfs_file_extent_num_bytes(leaf, fi);
	BUG_ON(key.offset > start || extent_end < end);

	bytenr = btrfs_file_extent_disk_bytenr(leaf, fi);
	num_bytes = btrfs_file_extent_disk_num_bytes(leaf, fi);
#ifdef MY_ABC_HERE
	ram_bytes = btrfs_file_extent_ram_bytes(leaf, fi);
#endif /* MY_ABC_HERE */
	orig_offset = key.offset - btrfs_file_extent_offset(leaf, fi);
	memcpy(&new_key, &key, sizeof(new_key));

	if (start == key.offset && end < extent_end) {
		other_start = 0;
		other_end = start;
		if (extent_mergeable(leaf, path->slots[0] - 1,
				     ino, bytenr, orig_offset,
				     &other_start, &other_end)) {
			new_key.offset = end;
			btrfs_set_item_key_safe(root->fs_info, path, &new_key);
			fi = btrfs_item_ptr(leaf, path->slots[0],
					    struct btrfs_file_extent_item);
			btrfs_set_file_extent_generation(leaf, fi,
							 trans->transid);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							extent_end - end);
#ifdef MY_DEF_HERE
			btrfs_set_file_extent_syno_flag(leaf, fi, 0);
#endif /* MY_DEF_HERE */
			btrfs_set_file_extent_offset(leaf, fi,
						     end - orig_offset);
			fi = btrfs_item_ptr(leaf, path->slots[0] - 1,
					    struct btrfs_file_extent_item);
			btrfs_set_file_extent_generation(leaf, fi,
							 trans->transid);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							end - other_start);
#ifdef MY_DEF_HERE
			btrfs_set_file_extent_syno_flag(leaf, fi, 0);
#endif /* MY_DEF_HERE */
			btrfs_mark_buffer_dirty(leaf);
			goto out;
		}
	}

	if (start > key.offset && end == extent_end) {
		other_start = end;
		other_end = 0;
		if (extent_mergeable(leaf, path->slots[0] + 1,
				     ino, bytenr, orig_offset,
				     &other_start, &other_end)) {
			fi = btrfs_item_ptr(leaf, path->slots[0],
					    struct btrfs_file_extent_item);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							start - key.offset);
			btrfs_set_file_extent_generation(leaf, fi,
							 trans->transid);
#ifdef MY_DEF_HERE
			btrfs_set_file_extent_syno_flag(leaf, fi, 0);
#endif /* MY_DEF_HERE */
			path->slots[0]++;
			new_key.offset = start;
			btrfs_set_item_key_safe(root->fs_info, path, &new_key);

			fi = btrfs_item_ptr(leaf, path->slots[0],
					    struct btrfs_file_extent_item);
			btrfs_set_file_extent_generation(leaf, fi,
							 trans->transid);
			btrfs_set_file_extent_num_bytes(leaf, fi,
							other_end - start);
			btrfs_set_file_extent_offset(leaf, fi,
						     start - orig_offset);
#ifdef MY_DEF_HERE
			btrfs_set_file_extent_syno_flag(leaf, fi, 0);
#endif /* MY_DEF_HERE */
			btrfs_mark_buffer_dirty(leaf);
			goto out;
		}
	}

	while (start > key.offset || end < extent_end) {
		if (key.offset == start)
			split = end;

		new_key.offset = split;
		ret = btrfs_duplicate_item(trans, root, path, &new_key);
		if (ret == -EAGAIN) {
			btrfs_release_path(path);
			goto again;
		}
		if (ret < 0) {
			btrfs_abort_transaction(trans, root, ret);
			goto out;
		}

		leaf = path->nodes[0];
		fi = btrfs_item_ptr(leaf, path->slots[0] - 1,
				    struct btrfs_file_extent_item);
		btrfs_set_file_extent_generation(leaf, fi, trans->transid);
		btrfs_set_file_extent_num_bytes(leaf, fi,
						split - key.offset);

		fi = btrfs_item_ptr(leaf, path->slots[0],
				    struct btrfs_file_extent_item);

		btrfs_set_file_extent_generation(leaf, fi, trans->transid);
		btrfs_set_file_extent_offset(leaf, fi, split - orig_offset);
		btrfs_set_file_extent_num_bytes(leaf, fi,
						extent_end - split);
		btrfs_mark_buffer_dirty(leaf);

		ret = btrfs_inc_extent_ref(trans, root, bytenr, num_bytes, 0,
					   root->root_key.objectid,
					   ino, orig_offset
#ifdef MY_ABC_HERE
					   ,1, 0
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
					   ,btrfs_syno_usage_ref_check(root, new_key.objectid, new_key.offset)
#endif /* MY_ABC_HERE */
					   );
		BUG_ON(ret); /* -ENOMEM */

		if (split == start) {
			key.offset = start;
		} else {
			BUG_ON(start != key.offset);
			path->slots[0]--;
			extent_end = end;
		}
		recow = 1;
	}

	other_start = end;
	other_end = 0;
	if (extent_mergeable(leaf, path->slots[0] + 1,
			     ino, bytenr, orig_offset,
			     &other_start, &other_end)) {
		if (recow) {
			btrfs_release_path(path);
			goto again;
		}
		extent_end = other_end;
		del_slot = path->slots[0] + 1;
		del_nr++;
#ifdef MY_ABC_HERE
		btrfs_item_key_to_cpu(leaf, &syno_usage_key, path->slots[0] + 1);
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
		ret = btrfs_free_extent_uid(trans, root, bytenr, num_bytes,
					0, root->root_key.objectid, ino, orig_offset,
					0, ram_bytes,
					inode, i_uid_read(inode)
#ifdef MY_ABC_HERE
					,btrfs_syno_usage_ref_check(root, syno_usage_key.objectid, syno_usage_key.offset)
#endif /* MY_ABC_HERE */
					);
#else
		ret = btrfs_free_extent(trans, root, bytenr, num_bytes,
					0, root->root_key.objectid,
					ino, orig_offset
#ifdef MY_ABC_HERE
					,0, ram_bytes
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
					,btrfs_syno_usage_ref_check(root, syno_usage_key.objectid, syno_usage_key.offset)
#endif /* MY_ABC_HERE */
					);
#endif /* MY_ABC_HERE */
		BUG_ON(ret); /* -ENOMEM */
	}
	other_start = 0;
	other_end = start;
	if (extent_mergeable(leaf, path->slots[0] - 1,
			     ino, bytenr, orig_offset,
			     &other_start, &other_end)) {
		if (recow) {
			btrfs_release_path(path);
			goto again;
		}
		key.offset = other_start;
		del_slot = path->slots[0];
		del_nr++;
#ifdef MY_ABC_HERE
		btrfs_item_key_to_cpu(leaf, &syno_usage_key, path->slots[0]);
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
		ret = btrfs_free_extent_uid(trans, root, bytenr, num_bytes,
					0, root->root_key.objectid,
					ino, orig_offset, 0, ram_bytes,
					inode, i_uid_read(inode)
#ifdef MY_ABC_HERE
					,btrfs_syno_usage_ref_check(root, syno_usage_key.objectid, syno_usage_key.offset)
#endif /* MY_ABC_HERE */
					);
#else
		ret = btrfs_free_extent(trans, root, bytenr, num_bytes,
					0, root->root_key.objectid,
					ino, orig_offset
#ifdef MY_ABC_HERE
					,0, ram_bytes
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
					,btrfs_syno_usage_ref_check(root, syno_usage_key.objectid, syno_usage_key.offset)
#endif /* MY_ABC_HERE */
					);
#endif /* MY_ABC_HERE */
		BUG_ON(ret); /* -ENOMEM */
	}
	if (del_nr == 0) {
		fi = btrfs_item_ptr(leaf, path->slots[0],
			   struct btrfs_file_extent_item);
		btrfs_set_file_extent_type(leaf, fi,
					   BTRFS_FILE_EXTENT_REG);
		btrfs_set_file_extent_generation(leaf, fi, trans->transid);
		btrfs_mark_buffer_dirty(leaf);
	} else {
		fi = btrfs_item_ptr(leaf, del_slot - 1,
			   struct btrfs_file_extent_item);
		btrfs_set_file_extent_type(leaf, fi,
					   BTRFS_FILE_EXTENT_REG);
		btrfs_set_file_extent_generation(leaf, fi, trans->transid);
		btrfs_set_file_extent_num_bytes(leaf, fi,
						extent_end - key.offset);
#ifdef MY_DEF_HERE
		btrfs_set_file_extent_syno_flag(leaf, fi, 0);
#endif /* MY_DEF_HERE */
		btrfs_mark_buffer_dirty(leaf);

		ret = btrfs_del_items(trans, root, path, del_slot, del_nr);
		if (ret < 0) {
			btrfs_abort_transaction(trans, root, ret);
			goto out;
		}
	}
out:
	btrfs_free_path(path);
	return ret;
}

/*
 * on error we return an unlocked page and the error value
 * on success we return a locked page and 0
 */
static int prepare_uptodate_page(struct inode *inode,
				 struct page *page, u64 pos,
				 bool force_uptodate)
{
	int ret = 0;

	if (((pos & (PAGE_CACHE_SIZE - 1)) || force_uptodate) &&
	    !PageUptodate(page)) {
		ret = btrfs_readpage(NULL, page);
		if (ret)
			return ret;
		lock_page(page);
		if (!PageUptodate(page)) {
			unlock_page(page);
			return -EIO;
		}
		if (page->mapping != inode->i_mapping) {
			unlock_page(page);
			return -EAGAIN;
		}
	}
	return 0;
}

/*
 * this just gets pages into the page cache and locks them down.
 */
#ifdef MY_ABC_HERE
static noinline int prepare_pages(struct inode *inode, struct page **pages,
				  size_t num_pages, loff_t pos,
				  size_t write_bytes, bool force_uptodate)
{
	int i;
	unsigned long index = pos >> PAGE_CACHE_SHIFT;
	gfp_t mask = btrfs_alloc_write_mask(inode->i_mapping);
	int err = 0;
	int faili;

restart:
	for (i = 0; i < num_pages; i++) {
again:
		pages[i] = find_or_create_page(inode->i_mapping, index + i,
					       mask | __GFP_WRITE);
		if (!pages[i]) {
			faili = i - 1;
			err = -ENOMEM;
			goto fail;
		}

		if (i == 0) {
			err = prepare_uptodate_page(inode, pages[i], pos,
						    force_uptodate);
			if (!err && i == num_pages - 1)
				err = prepare_uptodate_page(inode, pages[i],
							pos + write_bytes, false);
			if (err) {
				page_cache_release(pages[i]);
				if (err == -EAGAIN) {
					err = 0;
					goto again;
				}
				faili = i - 1;
				goto fail;
			}
		} else if (i == num_pages - 1) {
			if (((pos + write_bytes) & (PAGE_CACHE_SIZE - 1)) &&
				!PageUptodate(pages[i])) {

				faili = i - 1;
				while (faili >= 0) {
					unlock_page(pages[faili]);
					page_cache_release(pages[faili]);
					faili--;
				}

				err = prepare_uptodate_page(inode, pages[i],
						    pos + write_bytes, false);
				if (err) {
					page_cache_release(pages[i]);
					if (err == -EAGAIN)
						goto restart;
					goto out;
				}

				unlock_page(pages[i]);
				page_cache_release(pages[i]);
				goto restart;
			}
		}

		wait_on_page_writeback(pages[i]);
	}

	return 0;
fail:
	while (faili >= 0) {
		unlock_page(pages[faili]);
		page_cache_release(pages[faili]);
		faili--;
	}
out:
	return err;
}
#else /* MY_ABC_HERE */
static noinline int prepare_pages(struct inode *inode, struct page **pages,
				  size_t num_pages, loff_t pos,
				  size_t write_bytes, bool force_uptodate)
{
	int i;
	unsigned long index = pos >> PAGE_CACHE_SHIFT;
	gfp_t mask = btrfs_alloc_write_mask(inode->i_mapping);
	int err = 0;
	int faili;

	for (i = 0; i < num_pages; i++) {
again:
		pages[i] = find_or_create_page(inode->i_mapping, index + i,
					       mask | __GFP_WRITE);
		if (!pages[i]) {
			faili = i - 1;
			err = -ENOMEM;
			goto fail;
		}

		if (i == 0)
			err = prepare_uptodate_page(inode, pages[i], pos,
						    force_uptodate);
		if (!err && i == num_pages - 1)
			err = prepare_uptodate_page(inode, pages[i],
						    pos + write_bytes, false);
		if (err) {
			page_cache_release(pages[i]);
			if (err == -EAGAIN) {
				err = 0;
				goto again;
			}
			faili = i - 1;
			goto fail;
		}
		wait_on_page_writeback(pages[i]);
	}

	return 0;
fail:
	while (faili >= 0) {
		unlock_page(pages[faili]);
		page_cache_release(pages[faili]);
		faili--;
	}
	return err;

}
#endif /* MY_ABC_HERE */

/*
 * This function locks the extent and properly waits for data=ordered extents
 * to finish before allowing the pages to be modified if need.
 *
 * The return value:
 * 1 - the extent is locked
 * 0 - the extent is not locked, and everything is OK
 * -EAGAIN - need re-prepare the pages
 * the other < 0 number - Something wrong happens
 */
static noinline int
lock_and_cleanup_extent_if_need(struct inode *inode, struct page **pages,
				size_t num_pages, loff_t pos,
				u64 *lockstart, u64 *lockend,
				struct extent_state **cached_state)
{
	u64 start_pos;
	u64 last_pos;
	int i;
	int ret = 0;

	start_pos = pos & ~((u64)PAGE_CACHE_SIZE - 1);
	last_pos = start_pos + ((u64)num_pages << PAGE_CACHE_SHIFT) - 1;

	if (start_pos < inode->i_size) {
		struct btrfs_ordered_extent *ordered;
		lock_extent_bits(&BTRFS_I(inode)->io_tree,
				 start_pos, last_pos, cached_state);
		ordered = btrfs_lookup_ordered_range(inode, start_pos,
						     last_pos - start_pos + 1);
		if (ordered &&
		    ordered->file_offset + ordered->len > start_pos &&
		    ordered->file_offset <= last_pos) {
			unlock_extent_cached(&BTRFS_I(inode)->io_tree,
					     start_pos, last_pos,
					     cached_state, GFP_NOFS);
			for (i = 0; i < num_pages; i++) {
				unlock_page(pages[i]);
				page_cache_release(pages[i]);
			}
			btrfs_start_ordered_extent(inode, ordered, 1);
			btrfs_put_ordered_extent(ordered);
			return -EAGAIN;
		}
		if (ordered)
			btrfs_put_ordered_extent(ordered);

		*lockstart = start_pos;
		*lockend = last_pos;
		ret = 1;
	}

	/*
	 * It's possible the pages are dirty right now, but we don't want
	 * to clean them yet because copy_from_user may catch a page fault
	 * and we might have to fall back to one page at a time.  If that
	 * happens, we'll unlock these pages and we'd have a window where
	 * reclaim could sneak in and drop the once-dirty page on the floor
	 * without writing it.
	 *
	 * We have the pages locked and the extent range locked, so there's
	 * no way someone can start IO on any dirty pages in this range.
	 *
	 * We'll call btrfs_dirty_pages() later on, and that will flip around
	 * delalloc bits and dirty the pages as required.
	 */
	for (i = 0; i < num_pages; i++) {
		set_page_extent_mapped(pages[i]);
		WARN_ON(!PageLocked(pages[i]));
	}

	return ret;
}

static noinline int check_can_nocow(struct inode *inode, loff_t pos,
				    size_t *write_bytes)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_ordered_extent *ordered;
	u64 lockstart, lockend;
	u64 num_bytes;
	int ret;

	ret = btrfs_start_write_no_snapshoting(root);
	if (!ret)
		return -ENOSPC;

	lockstart = round_down(pos, root->sectorsize);
	lockend = round_up(pos + *write_bytes, root->sectorsize) - 1;

	while (1) {
		lock_extent(&BTRFS_I(inode)->io_tree, lockstart, lockend);
		ordered = btrfs_lookup_ordered_range(inode, lockstart,
						     lockend - lockstart + 1);
		if (!ordered) {
			break;
		}
		unlock_extent(&BTRFS_I(inode)->io_tree, lockstart, lockend);
		btrfs_start_ordered_extent(inode, ordered, 1);
		btrfs_put_ordered_extent(ordered);
	}

	num_bytes = lockend - lockstart + 1;
	ret = can_nocow_extent(inode, lockstart, &num_bytes, NULL, NULL, NULL, false);
	if (ret <= 0) {
		ret = 0;
		btrfs_end_write_no_snapshoting(root);
	} else {
		*write_bytes = min_t(size_t, *write_bytes ,
				     num_bytes - pos + lockstart);
	}

	unlock_extent(&BTRFS_I(inode)->io_tree, lockstart, lockend);

	return ret;
}

#ifdef MY_ABC_HERE
void syno_ordered_extent_throttle(struct btrfs_fs_info *fs_info)
{
	DEFINE_WAIT(wait);

	if (!fs_info)
		return;

	if (fs_info->syno_max_ordered_queue_size && atomic64_read(&fs_info->syno_ordered_extent_nr) > fs_info->syno_max_ordered_queue_size) {
		prepare_to_wait_exclusive(&fs_info->syno_ordered_queue_wait, &wait, TASK_UNINTERRUPTIBLE);
		if (atomic64_read(&fs_info->syno_ordered_extent_nr) > fs_info->syno_max_ordered_queue_size) {
			schedule();
		}
		finish_wait(&fs_info->syno_ordered_queue_wait, &wait);
	}
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
#include <linux/tcp.h>
#include <net/tcp.h>

static void update_time_for_write(struct inode *inode);

#ifdef MY_ABC_HERE
extern int ecryptfs_encrypt_page_export(struct page *enc_page,
				   void *crypt_stat_ptr,
				   struct page *page);

static int generic_write_init_and_checks(struct file* filp, loff_t pos, size_t nbytes)
{
	struct iovec iov = { .iov_base = NULL, .iov_len = nbytes };
	struct kiocb kiocb;
	struct iov_iter iter;

	init_sync_kiocb(&kiocb, filp);
	kiocb.ki_pos = pos;
	iov_iter_init(&iter, WRITE, &iov, 1, nbytes);

	return generic_write_checks(&kiocb, &iter);
}

static noinline int __btrfs_ecryptfs_zero_copy(struct file *file, loff_t pos, int num_page,
      struct page **pages, void *crypt_stat)
{
	struct inode *inode = file_inode(file);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct page *dst_pages[MAX_PAGES_PER_RECVFILE + 1];
	struct extent_state *cached_state = NULL;
	u64 release_bytes = 0;
	u64 lockstart;
	u64 lockend;
	u64 start_pos = 0;
	u64 end_pos;
	int ret = 0, i;
	size_t count = num_page*PAGE_CACHE_SIZE;
	bool need_unlock;
#ifdef MY_DEF_HERE
	/* for syno cache protection */
	int err;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND syno_cache_protection_command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_WRITE;
	struct syno_cache_protection_parameter_command_write syno_cache_protection_parm;
	void *syno_cache_protection_req = NULL;
	bool bl_nlink = false;
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
	if (inode->i_nlink > 0)
		bl_nlink = true;
#endif /* MY_DEF_HERE */

#ifdef MY_ABC_HERE
	syno_ordered_extent_throttle(root->fs_info);
#endif /* MY_ABC_HERE */

	file_start_write(file);

	BUG_ON(num_page > MAX_PAGES_PER_RECVFILE + 1);
	mutex_lock(&inode->i_mutex);
	ret = generic_write_init_and_checks(file, pos, count);
	if (ret <= 0) {
		goto out;
	}

	/*
	 * We can write back this queue in page reclaim
	 */
	current->backing_dev_info = inode_to_bdi(inode);
	ret = file_remove_privs(file);
	if (ret)
		goto out;

	update_time_for_write(inode);
	/* copied from btrfs_file_write_iter */
	start_pos = round_down(pos, root->sectorsize);
	if (start_pos > i_size_read(inode)) {
		end_pos = round_up(pos + count, root->sectorsize);
		ret = btrfs_cont_expand(inode, i_size_read(inode), end_pos);
		if (ret)
			goto out;
	}

	ret = btrfs_check_data_free_space(inode, pos, count);
	if (ret)
		goto out;

	ret = btrfs_delalloc_reserve_metadata(inode, count);
	if (ret) {
		btrfs_free_reserved_data_space(inode, pos, count);
		goto out;
	}

	release_bytes = count;
	need_unlock = false;

#ifdef MY_DEF_HERE
	if (syno_cache_protection_is_enabled(root->fs_info) && bl_nlink) {
		memset(&syno_cache_protection_parm, 0, sizeof(syno_cache_protection_parm));
		syno_cache_protection_parm.num_pages = num_page;
		syno_cache_protection_req = btrfs_syno_cache_protection_alloc_command(syno_cache_protection_command, root->fs_info, &syno_cache_protection_parm);
		if (IS_ERR(syno_cache_protection_req)) {
			err = PTR_ERR(syno_cache_protection_req);
			syno_cache_protection_req = NULL;
			btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection alloc write command with num_pages [%d] err %d", num_page, err);
		}
	}
#endif /* MY_DEF_HERE */

again:
	/*
	 * This is going to setup the pages array with the number of
	 * pages we want, so we don't really need to worry about the
	 * contents of pages from loop to loop
	 */
	ret = prepare_pages(inode, dst_pages, num_page, pos, count, false);
	if (ret)
		goto out;

	ret = lock_and_cleanup_extent_if_need(inode, dst_pages, num_page,
					      pos, &lockstart, &lockend,
					      &cached_state);
	if (ret < 0) {
		if (ret == -EAGAIN)
			goto again;
		goto out;
	} else if (ret > 0) {
		need_unlock = true;
		ret = 0;
	}
	for (i = 0;i < num_page;i++) {
		ret = ecryptfs_encrypt_page_export(dst_pages[i], crypt_stat, pages[i]);
		if (ret) {
			if (need_unlock)
				unlock_extent_cached(&BTRFS_I(inode)->io_tree,
						     lockstart, lockend, &cached_state,
						     GFP_NOFS);
			btrfs_drop_pages(dst_pages, num_page);
			goto out;
		}
		cond_resched();
	}
	ret = btrfs_dirty_pages(root, inode, dst_pages,
				num_page, pos, count,
				NULL);

#ifdef MY_DEF_HERE
	if (!ret && num_page > 0 && syno_cache_protection_is_enabled(root->fs_info) && syno_cache_protection_req) {
		syno_cache_protection_parm.inode = inode;
		syno_cache_protection_parm.num_pages = num_page;
		syno_cache_protection_parm.pages = dst_pages;
		err = btrfs_syno_cache_protection_write_and_send_command(syno_cache_protection_req, &syno_cache_protection_parm);
		if (err) {
			btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection send write command with root %llu inode %llu pg_index %lu fsid %pU err %d", root->objectid, btrfs_ino(inode),
						page_index(dst_pages[0]), root->fs_info->fs_devices->fsid, err);
		}
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */

	if (need_unlock)
		unlock_extent_cached(&BTRFS_I(inode)->io_tree,
				     lockstart, lockend, &cached_state,
				     GFP_NOFS);
	if (ret) {
		btrfs_drop_pages(dst_pages, num_page);
		goto out;
	}

	release_bytes = 0;
	btrfs_drop_pages(dst_pages, num_page);

	balance_dirty_pages_ratelimited(inode->i_mapping);
#ifdef MY_ABC_HERE
	/*
	 * DSM#134799
	 * In order to align the previous enc seq write performance
	 * we skip multiple writeback
	 */
	// syno_writeback_balance_dirty_pages(root->fs_info);
#endif /* MY_ABC_HERE */
	if (num_page < (root->nodesize >> PAGE_CACHE_SHIFT) + 1)
		btrfs_btree_balance_dirty(root);

out:
#ifdef MY_DEF_HERE
	if (syno_cache_protection_req) {
		btrfs_syno_cache_protection_free_command(syno_cache_protection_req);
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */
	if (release_bytes) {
		btrfs_delalloc_release_space(inode, pos, release_bytes);
	}
	file_end_write(file);
	current->backing_dev_info = NULL;
	mutex_unlock(&inode->i_mutex);

	return ret;
}

static noinline int btrfs_ecryptfs_zero_copy(struct file *file, loff_t pos, int num_page,
      struct page **pages, void *crypt_stat)
{
	ssize_t ret;
#ifdef MY_DEF_HERE
	int nrptrs = min((int)SYNO_CACHE_PROTECTION_REQUEST_MAX_DATA_PAGES, (int)MAX_PAGES_PER_RECVFILE);
#else
	int nrptrs = MAX_PAGES_PER_RECVFILE;
#endif /* MY_DEF_HERE */
	int tmp_num_pages, remain_pages = num_page;
	loff_t offset = pos;
	struct page **tmp_pages = pages;

	do {
		tmp_num_pages = min(remain_pages, nrptrs);
		ret = __btrfs_ecryptfs_zero_copy(file, offset, tmp_num_pages, tmp_pages, crypt_stat);
		if (ret)
			break;
		remain_pages -= tmp_num_pages;
		offset += tmp_num_pages * PAGE_CACHE_SIZE;
		tmp_pages += tmp_num_pages;
	} while (remain_pages > 0);
	return ret;
}
#endif /* MY_ABC_HERE */

static noinline ssize_t btrfs_do_recvfile(struct file *file, struct socket *sock,
					loff_t pos, size_t count, size_t * rbytes, size_t * wbytes)
{
	struct inode *inode = file_inode(file);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct msghdr   msg;
	struct page *pages[MAX_PAGES_PER_RECVFILE];
	struct extent_state *cached_state = NULL;
	struct kvec iov[MAX_PAGES_PER_RECVFILE];
	u64 release_bytes = 0;
	u64 lockstart;
	u64 lockend;
	u64 start_pos = 0;
	u64 end_pos;
	size_t num_written = 0;
	ssize_t ret = 0;
	int recv_meg_ret = 0;
	int i;
	long rcvtimeo;
	bool only_release_metadata = false;
	bool need_unlock;
	size_t offset = pos & (PAGE_CACHE_SIZE - 1);
	size_t write_bytes = min(count,
				 MAX_PAGES_PER_RECVFILE * (size_t)PAGE_CACHE_SIZE -
				 offset);
	size_t num_pages = (write_bytes + offset +
				PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	size_t reserve_bytes;
	size_t dirty_pages;
	size_t copied = 0;
#ifdef MY_DEF_HERE
	/* for syno cache protection */
	int err;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND syno_cache_protection_command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_WRITE;
	struct syno_cache_protection_parameter_command_write syno_cache_protection_parm;
	void *syno_cache_protection_req = NULL;
	bool bl_nlink = false;
#endif /* MY_DEF_HERE */

#ifdef MY_DEF_HERE
	if (inode->i_nlink > 0)
		bl_nlink = true;
#endif /* MY_DEF_HERE */

#ifdef MY_ABC_HERE
	syno_ordered_extent_throttle(root->fs_info);
#endif /* MY_ABC_HERE */

	/* copied from btrfs_file_write_iter */
	start_pos = round_down(pos, root->sectorsize);
	if (round_down(pos, root->sectorsize) > i_size_read(inode)) {
		end_pos = round_up(pos + count, root->sectorsize);
		ret = btrfs_cont_expand(inode, i_size_read(inode), end_pos);
		if (ret)
			return ret;
	}
	memset(&msg, 0, sizeof(msg));
	msg.msg_flags = MSG_KERNSPACE;
	rcvtimeo = sock->sk->sk_rcvtimeo;
	sock->sk->sk_rcvtimeo = 64 * HZ;

	reserve_bytes = num_pages << PAGE_CACHE_SHIFT;

	ret = btrfs_check_data_free_space(inode, pos, write_bytes);
	if (ret < 0) {
		if ((BTRFS_I(inode)->flags & (BTRFS_INODE_NODATACOW |
					      BTRFS_INODE_PREALLOC)) &&
		    check_can_nocow(inode, pos, &write_bytes) > 0) {
			/*
			 * For nodata cow case, no need to reserve
			 * data space.
			 */
			only_release_metadata = true;
			/*
			 * our prealloc extent may be smaller than
			 * write_bytes, so scale down.
			 */
			num_pages = DIV_ROUND_UP(write_bytes + offset,
						 PAGE_CACHE_SIZE);
			reserve_bytes = num_pages << PAGE_CACHE_SHIFT;
		} else {
			goto err;
		}
	}

	ret = btrfs_delalloc_reserve_metadata(inode, reserve_bytes);
	if (ret) {
		if (!only_release_metadata)
			btrfs_free_reserved_data_space(inode, pos, write_bytes);
		else
			btrfs_end_write_no_snapshoting(root);
		goto err;
	}

	release_bytes = reserve_bytes;
	need_unlock = false;

#ifdef MY_DEF_HERE
	if (syno_cache_protection_is_enabled(root->fs_info) && bl_nlink) {
		memset(&syno_cache_protection_parm, 0, sizeof(syno_cache_protection_parm));
		syno_cache_protection_parm.num_pages = num_pages;
		syno_cache_protection_req = btrfs_syno_cache_protection_alloc_command(syno_cache_protection_command, root->fs_info, &syno_cache_protection_parm);
		if (IS_ERR(syno_cache_protection_req)) {
			err = PTR_ERR(syno_cache_protection_req);
			syno_cache_protection_req = NULL;
			btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection alloc write command with num_pages [%zd] err %d", num_pages, err);
		}
	}
#endif /* MY_DEF_HERE */

again:
	/*
	 * This is going to setup the pages array with the number of
	 * pages we want, so we don't really need to worry about the
	 * contents of pages from loop to loop
	 */
	ret = prepare_pages(inode, pages, num_pages,
				pos, write_bytes,
				false);
	if (ret)
		goto err;

	ret = lock_and_cleanup_extent_if_need(inode, pages, num_pages,
						  pos, &lockstart, &lockend,
						  &cached_state);
	if (ret < 0) {
		if (ret == -EAGAIN)
			goto again;
		goto err;
	} else if (ret > 0) {
		need_unlock = true;
		ret = 0;
	}

	iov[0].iov_base = kmap(pages[0]) + offset;
	iov[0].iov_len = PAGE_CACHE_SIZE - offset;
	for (i = 1; i < num_pages; i++) {
		iov[i].iov_base = kmap(pages[i]);
		iov[i].iov_len = PAGE_CACHE_SIZE;
	}
	if (0 != ((write_bytes + offset) & (PAGE_CACHE_SIZE - 1)) && 1 < num_pages) {
		iov[num_pages-1].iov_len = (write_bytes + offset) & (PAGE_CACHE_SIZE - 1);
	}
	recv_meg_ret = kernel_recvmsg(
			sock, &msg, &iov[0], num_pages, write_bytes,
			MSG_WAITALL | MSG_NOCATCHSIGNAL);
	for (i = 0; i < num_pages; i++) {
		kunmap(pages[i]);
	}
	if (0 > recv_meg_ret) {
		if (need_unlock)
			unlock_extent_cached(&BTRFS_I(inode)->io_tree,
						 lockstart, lockend, &cached_state,
						 GFP_NOFS);
		btrfs_drop_pages(pages, num_pages);
		ret = recv_meg_ret;
		goto err;
	}
	copied = (size_t) recv_meg_ret;
	*rbytes = copied;
	if (write_bytes > recv_meg_ret) {
		recv_meg_ret = -EPIPE;
	}
	dirty_pages = (copied + offset + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	/*
	 * If we had a short copy we need to release the excess delaloc
	 * bytes we reserved.  We need to increment outstanding_extents
	 * because btrfs_delalloc_release_space will decrement it, but
	 * we still have an outstanding extent for the chunk we actually
	 * managed to copy.
	 */
	if (num_pages > dirty_pages) {
		release_bytes = (num_pages - dirty_pages) <<
			PAGE_CACHE_SHIFT;
		if (copied > 0) {
			spin_lock(&BTRFS_I(inode)->lock);
			BTRFS_I(inode)->outstanding_extents++;
			spin_unlock(&BTRFS_I(inode)->lock);
		}
		if (only_release_metadata) {
			btrfs_delalloc_release_metadata(inode,
							release_bytes);
		} else {
			u64 __pos;

			__pos = round_down(pos, root->sectorsize) +
				(dirty_pages << PAGE_CACHE_SHIFT);
			btrfs_delalloc_release_space(inode, __pos,
							 release_bytes);
		}
	}

	release_bytes = dirty_pages << PAGE_CACHE_SHIFT;
	if (copied > 0)
		ret = btrfs_dirty_pages(root, inode, pages,
					dirty_pages, pos, copied,
					NULL);

#ifdef MY_DEF_HERE
	if (!ret && copied > 0 && dirty_pages > 0 && syno_cache_protection_is_enabled(root->fs_info) && syno_cache_protection_req) {
		syno_cache_protection_parm.inode = inode;
		syno_cache_protection_parm.num_pages = dirty_pages;
		syno_cache_protection_parm.pages = pages;
		err = btrfs_syno_cache_protection_write_and_send_command(syno_cache_protection_req, &syno_cache_protection_parm);
		if (err) {
			btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection send write command with root %llu inode %llu pg_index %lu fsid %pU err %d", root->objectid, btrfs_ino(inode),
						page_index(pages[0]), root->fs_info->fs_devices->fsid, err);
		}
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */

	if (need_unlock)
		unlock_extent_cached(&BTRFS_I(inode)->io_tree,
					 lockstart, lockend, &cached_state,
					 GFP_NOFS);
	if (ret) {
		btrfs_drop_pages(pages, num_pages);
		goto err;
	}

	release_bytes = 0;
	if (only_release_metadata)
		btrfs_end_write_no_snapshoting(root);

	if (only_release_metadata && copied > 0) {
		u64 lockstart = round_down(pos, root->sectorsize);
		u64 lockend = lockstart +
			(dirty_pages << PAGE_CACHE_SHIFT) - 1;

		set_extent_bit(&BTRFS_I(inode)->io_tree, lockstart,
				   lockend, EXTENT_NORESERVE, NULL,
				   NULL, GFP_NOFS);
		only_release_metadata = false;
	}

	btrfs_drop_pages(pages, num_pages);

	cond_resched();

	balance_dirty_pages_ratelimited(inode->i_mapping);
#ifdef MY_ABC_HERE
	syno_writeback_balance_dirty_pages(root->fs_info);
#endif /* MY_ABC_HERE */
	if (dirty_pages < (root->nodesize >> PAGE_CACHE_SHIFT) + 1)
		btrfs_btree_balance_dirty(root);

	num_written = copied;
	if (recv_meg_ret == -EPIPE) {
		ret = -EPIPE;
		goto err;
	}

err:
	sock->sk->sk_rcvtimeo = rcvtimeo;

#ifdef MY_DEF_HERE
	if (syno_cache_protection_req) {
		btrfs_syno_cache_protection_free_command(syno_cache_protection_req);
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */

	if (release_bytes) {
		if (only_release_metadata) {
			btrfs_end_write_no_snapshoting(root);
			btrfs_delalloc_release_metadata(inode, release_bytes);
		} else {
			btrfs_delalloc_release_space(inode, pos, release_bytes);
		}
	}
	*wbytes = num_written;

	return ret ? ret : num_written;
}

static noinline ssize_t btrfs_recvfile(int fd, struct file *file, struct socket *sock,
					loff_t pos, size_t count, size_t *received, size_t *written)
{
	ssize_t ret;

	update_time_for_write(file_inode(file));
	do {
		size_t bytes_received = 0;
		size_t bytes_written = 0;

#ifdef MY_DEF_HERE
		int nrptrs = min((int)SYNO_CACHE_PROTECTION_REQUEST_MAX_DATA_PAGES, (int)MAX_PAGES_PER_RECVFILE);
#else
		int nrptrs = MAX_PAGES_PER_RECVFILE;
#endif /* MY_DEF_HERE */
		size_t offset = pos & (PAGE_CACHE_SIZE - 1);
		size_t max_bytes = nrptrs * (size_t)PAGE_CACHE_SIZE - offset;

		ret = btrfs_do_recvfile(file, sock, pos, (count > max_bytes) ?
			   max_bytes : count, &bytes_received, &bytes_written);
		*received += bytes_received;
		*written += bytes_written;
		if (ret <= 0)
			break;
		count -= bytes_written;
		pos += bytes_written;
	} while (count > 0);
	return ret < 0 ? ret : *written;
}
#endif /* MY_ABC_HERE */

static noinline ssize_t __btrfs_buffered_write(struct file *file,
					       struct iov_iter *i,
					       loff_t pos)
{
	struct inode *inode = file_inode(file);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct page **pages = NULL;
	struct extent_state *cached_state = NULL;
	u64 release_bytes = 0;
	u64 lockstart;
	u64 lockend;
	size_t num_written = 0;
	int nrptrs;
	int ret = 0;
	bool only_release_metadata = false;
	bool force_page_uptodate = false;
	bool need_unlock;
#ifdef MY_ABC_HERE
	bool relock = false;
#endif /* MY_ABC_HERE */
#ifdef MY_DEF_HERE
	int err;
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND syno_cache_protection_command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_WRITE;
	struct syno_cache_protection_parameter_command_write syno_cache_protection_parm;
	void *syno_cache_protection_req = NULL;
	bool bl_nlink = false;
#endif /* MY_DEF_HERE */

	nrptrs = min(DIV_ROUND_UP(iov_iter_count(i), PAGE_CACHE_SIZE),
			PAGE_CACHE_SIZE / (sizeof(struct page *)));
	nrptrs = min(nrptrs, current->nr_dirtied_pause - current->nr_dirtied);
	nrptrs = max(nrptrs, 8);
#ifdef MY_DEF_HERE
	nrptrs = min(nrptrs, SYNO_CACHE_PROTECTION_REQUEST_MAX_DATA_PAGES);
#endif /* MY_DEF_HERE */
	pages = kmalloc_array(nrptrs, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

#ifdef MY_DEF_HERE
	if (inode->i_nlink > 0)
		bl_nlink = true;
#endif /* MY_DEF_HERE */

#ifdef MY_ABC_HERE
	inode_dio_begin(inode);
	smp_mb__after_atomic();

	/*
	 * If the write is beyond the EOF, we need update
	 * the isize, but it is protected by i_mutex. So we can
	 * not unlock the i_mutex at this case.
	 */
	if (pos + iov_iter_count(i) <= i_size_read(inode)) {
		inode_unlock(inode);
		relock = true;
	}
	if (relock)
		down_read(&BTRFS_I(inode)->dio_sem);
#endif /* MY_ABC_HERE */

	while (iov_iter_count(i) > 0) {
		size_t offset = pos & (PAGE_CACHE_SIZE - 1);
		size_t write_bytes = min(iov_iter_count(i),
					 nrptrs * (size_t)PAGE_CACHE_SIZE -
					 offset);
		size_t num_pages = DIV_ROUND_UP(write_bytes + offset,
						PAGE_CACHE_SIZE);
		size_t reserve_bytes;
		size_t dirty_pages;
		size_t copied;
#ifdef MY_ABC_HERE
		bool reservation_with_no_commit;
#endif /* MY_ABC_HERE */

		WARN_ON(num_pages > nrptrs);

		/*
		 * Fault pages before locking them in prepare_pages
		 * to avoid recursive lock
		 */
		if (unlikely(iov_iter_fault_in_readable(i, write_bytes))) {
			ret = -EFAULT;
			break;
		}

		reserve_bytes = num_pages << PAGE_CACHE_SHIFT;
		only_release_metadata = false;
#ifdef MY_ABC_HERE
		reservation_with_no_commit = false;
		if (BTRFS_I(inode)->flags & (BTRFS_INODE_NODATACOW | BTRFS_INODE_PREALLOC))
			reservation_with_no_commit = true;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
		/* If necessary, ecryptfs_zero_copy and syno_recvfile can also add this logic. */
reserve_again:
		if (reservation_with_no_commit)
			ret = btrfs_check_data_free_space_with_no_commit(inode, pos, write_bytes);
		else
#endif /* MY_ABC_HERE */
			ret = btrfs_check_data_free_space(inode, pos, write_bytes);
		if (ret < 0) {
			if ((BTRFS_I(inode)->flags & (BTRFS_INODE_NODATACOW |
						      BTRFS_INODE_PREALLOC)) &&
			    check_can_nocow(inode, pos, &write_bytes) > 0) {
				/*
				 * For nodata cow case, no need to reserve
				 * data space.
				 */
				only_release_metadata = true;
				/*
				 * our prealloc extent may be smaller than
				 * write_bytes, so scale down.
				 */
				num_pages = DIV_ROUND_UP(write_bytes + offset,
							 PAGE_CACHE_SIZE);
				reserve_bytes = num_pages << PAGE_CACHE_SHIFT;
			} else {
#ifdef MY_ABC_HERE
				if (reservation_with_no_commit) {
					reservation_with_no_commit = false;
					goto reserve_again;
				}
#endif /* MY_ABC_HERE */
				break;
			}
		}

		ret = btrfs_delalloc_reserve_metadata(inode, reserve_bytes);
		if (ret) {
			if (!only_release_metadata)
				btrfs_free_reserved_data_space(inode, pos,
							       write_bytes);
			else
				btrfs_end_write_no_snapshoting(root);
			break;
		}

		release_bytes = reserve_bytes;
		need_unlock = false;

#ifdef MY_DEF_HERE
		if (syno_cache_protection_is_enabled(root->fs_info) && bl_nlink) {
			memset(&syno_cache_protection_parm, 0, sizeof(syno_cache_protection_parm));
			syno_cache_protection_parm.num_pages = num_pages;
			syno_cache_protection_req = btrfs_syno_cache_protection_alloc_command(syno_cache_protection_command, root->fs_info, &syno_cache_protection_parm);
			if (IS_ERR(syno_cache_protection_req)) {
				err = PTR_ERR(syno_cache_protection_req);
				syno_cache_protection_req = NULL;
				btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection alloc write command with num_pages [%zd] err %d", num_pages, err);
			}
		}
#endif /* MY_DEF_HERE */
again:
		/*
		 * This is going to setup the pages array with the number of
		 * pages we want, so we don't really need to worry about the
		 * contents of pages from loop to loop
		 */
		ret = prepare_pages(inode, pages, num_pages,
				    pos, write_bytes,
				    force_page_uptodate);
		if (ret)
			break;

		ret = lock_and_cleanup_extent_if_need(inode, pages, num_pages,
						      pos, &lockstart, &lockend,
						      &cached_state);
		if (ret < 0) {
			if (ret == -EAGAIN)
				goto again;
			break;
		} else if (ret > 0) {
			need_unlock = true;
			ret = 0;
		}

		copied = btrfs_copy_from_user(pos, write_bytes, pages, i);

		/*
		 * if we have trouble faulting in the pages, fall
		 * back to one page at a time
		 */
		if (copied < write_bytes)
			nrptrs = 1;

		if (copied == 0) {
			force_page_uptodate = true;
			dirty_pages = 0;
		} else {
			force_page_uptodate = false;
			dirty_pages = DIV_ROUND_UP(copied + offset,
						   PAGE_CACHE_SIZE);
		}

		/*
		 * If we had a short copy we need to release the excess delaloc
		 * bytes we reserved.  We need to increment outstanding_extents
		 * because btrfs_delalloc_release_space will decrement it, but
		 * we still have an outstanding extent for the chunk we actually
		 * managed to copy.
		 */
		if (num_pages > dirty_pages) {
			release_bytes = (num_pages - dirty_pages) <<
				PAGE_CACHE_SHIFT;
			if (copied > 0) {
				spin_lock(&BTRFS_I(inode)->lock);
				BTRFS_I(inode)->outstanding_extents++;
				spin_unlock(&BTRFS_I(inode)->lock);
			}
			if (only_release_metadata) {
				btrfs_delalloc_release_metadata(inode,
								release_bytes);
			} else {
				u64 __pos;

				__pos = round_down(pos, root->sectorsize) +
					(dirty_pages << PAGE_CACHE_SHIFT);
				btrfs_delalloc_release_space(inode, __pos,
							     release_bytes);
			}
		}

		release_bytes = dirty_pages << PAGE_CACHE_SHIFT;

		if (copied > 0)
			ret = btrfs_dirty_pages(root, inode, pages,
						dirty_pages, pos, copied,
						NULL);

#ifdef MY_DEF_HERE
		if (!ret && copied > 0 && dirty_pages > 0 && syno_cache_protection_is_enabled(root->fs_info) && syno_cache_protection_req) {
			syno_cache_protection_parm.inode = inode;
			syno_cache_protection_parm.num_pages = dirty_pages;
			syno_cache_protection_parm.pages = pages;
			err = btrfs_syno_cache_protection_write_and_send_command(syno_cache_protection_req, &syno_cache_protection_parm);
			if (err) {
				btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection send write command with root %llu inode %llu pg_index %lu fsid %pU err %d", root->objectid, btrfs_ino(inode),
							page_index(pages[0]), root->fs_info->fs_devices->fsid, err);
			}
			syno_cache_protection_req = NULL;
		}

		if (syno_cache_protection_req) {
			btrfs_syno_cache_protection_free_command(syno_cache_protection_req);
			syno_cache_protection_req = NULL;
		}
#endif /* MY_DEF_HERE */

		if (need_unlock)
			unlock_extent_cached(&BTRFS_I(inode)->io_tree,
					     lockstart, lockend, &cached_state,
					     GFP_NOFS);
		if (ret) {
			btrfs_drop_pages(pages, num_pages);
			break;
		}

		release_bytes = 0;
		if (only_release_metadata)
			btrfs_end_write_no_snapshoting(root);

		if (only_release_metadata && copied > 0) {
			lockstart = round_down(pos, root->sectorsize);
			lockend = lockstart +
				(dirty_pages << PAGE_CACHE_SHIFT) - 1;

			set_extent_bit(&BTRFS_I(inode)->io_tree, lockstart,
				       lockend, EXTENT_NORESERVE, NULL,
				       NULL, GFP_NOFS);
		}

		btrfs_drop_pages(pages, num_pages);

		cond_resched();

		balance_dirty_pages_ratelimited(inode->i_mapping);
#ifdef MY_ABC_HERE
		syno_writeback_balance_dirty_pages(root->fs_info);
#endif /* MY_ABC_HERE */
		if (dirty_pages < (root->nodesize >> PAGE_CACHE_SHIFT) + 1)
			btrfs_btree_balance_dirty(root);

		pos += copied;
		num_written += copied;
	}

	kfree(pages);

#ifdef MY_DEF_HERE
	if (syno_cache_protection_req) {
		btrfs_syno_cache_protection_free_command(syno_cache_protection_req);
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */

	if (release_bytes) {
		if (only_release_metadata) {
			btrfs_end_write_no_snapshoting(root);
			btrfs_delalloc_release_metadata(inode, release_bytes);
		} else {
			btrfs_delalloc_release_space(inode,
						round_down(pos, root->sectorsize),
						release_bytes);
		}
	}

#ifdef MY_ABC_HERE
	if (relock)
		up_read(&BTRFS_I(inode)->dio_sem);
	inode_dio_end(inode);
	if (relock)
		inode_lock(inode);
#endif /* MY_ABC_HERE */

	return num_written ? num_written : ret;
}

static ssize_t __btrfs_direct_write(struct kiocb *iocb,
				    struct iov_iter *from,
				    loff_t pos)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file_inode(file);
	ssize_t written;
	ssize_t written_buffered;
	loff_t endbyte;
	int err;

	written = generic_file_direct_write(iocb, from, pos);

	if (written < 0 || !iov_iter_count(from))
		return written;

	pos += written;
	written_buffered = __btrfs_buffered_write(file, from, pos);
	if (written_buffered < 0) {
		err = written_buffered;
		goto out;
	}
	/*
	 * Ensure all data is persisted. We want the next direct IO read to be
	 * able to read what was just written.
	 */
	endbyte = pos + written_buffered - 1;
	err = btrfs_fdatawrite_range(inode, pos, endbyte);
	if (err)
		goto out;
	err = filemap_fdatawait_range(inode->i_mapping, pos, endbyte);
	if (err)
		goto out;
	written += written_buffered;
	iocb->ki_pos = pos + written_buffered;
	invalidate_mapping_pages(file->f_mapping, pos >> PAGE_CACHE_SHIFT,
				 endbyte >> PAGE_CACHE_SHIFT);
out:
	return written ? written : err;
}

static void update_time_for_write(struct inode *inode)
{
	struct timespec now;

	if (IS_NOCMTIME(inode))
		return;

	now = current_fs_time(inode->i_sb);
	if (!timespec_equal(&inode->i_mtime, &now))
		inode->i_mtime = now;

	if (!timespec_equal(&inode->i_ctime, &now))
		inode->i_ctime = now;

	if (IS_I_VERSION(inode))
		inode_inc_iversion(inode);

#ifdef MY_ABC_HERE
	if (unlikely(block_dump))
		block_dump___btrfs_update_inode(inode);
#endif /* MY_ABC_HERE */
}

static ssize_t btrfs_file_write_iter(struct kiocb *iocb,
				    struct iov_iter *from)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file_inode(file);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	u64 start_pos;
	u64 end_pos;
	ssize_t num_written = 0;
	bool sync = (file->f_flags & O_DSYNC) || IS_SYNC(file->f_mapping->host);
	ssize_t err;
	loff_t pos;
	size_t count;

#ifdef MY_ABC_HERE
	syno_ordered_extent_throttle(root->fs_info);
#endif /* MY_ABC_HERE */

	inode_lock(inode);
	err = generic_write_checks(iocb, from);
	if (err <= 0) {
		inode_unlock(inode);
		return err;
	}

	current->backing_dev_info = inode_to_bdi(inode);
	err = file_remove_privs(file);
	if (err) {
		inode_unlock(inode);
		goto out;
	}

	/*
	 * If BTRFS flips readonly due to some impossible error
	 * (fs_info->fs_state now has BTRFS_SUPER_FLAG_ERROR),
	 * although we have opened a file as writable, we have
	 * to stop this write operation to ensure FS consistency.
	 */
	if (test_bit(BTRFS_FS_STATE_ERROR, &root->fs_info->fs_state)) {
		inode_unlock(inode);
		err = -EROFS;
		goto out;
	}

	/*
	 * We reserve space for updating the inode when we reserve space for the
	 * extent we are going to write, so we will enospc out there.  We don't
	 * need to start yet another transaction to update the inode as we will
	 * update the inode when we finish writing whatever data we write.
	 */
	update_time_for_write(inode);

	pos = iocb->ki_pos;
	count = iov_iter_count(from);
	start_pos = round_down(pos, root->sectorsize);
	if (start_pos > i_size_read(inode)) {
		/* Expand hole size to cover write data, preventing empty gap */
		end_pos = round_up(pos + count, root->sectorsize);
		err = btrfs_cont_expand(inode, i_size_read(inode), end_pos);
		if (err) {
			inode_unlock(inode);
			goto out;
		}
	}

	if (sync)
		atomic_inc(&BTRFS_I(inode)->sync_writers);

	if (iocb->ki_flags & IOCB_DIRECT) {
		num_written = __btrfs_direct_write(iocb, from, pos);
	} else {
		num_written = __btrfs_buffered_write(file, from, pos);
		if (num_written > 0)
			iocb->ki_pos = pos + num_written;
	}

	inode_unlock(inode);

	/*
	 * We also have to set last_sub_trans to the current log transid,
	 * otherwise subsequent syncs to a file that's been synced in this
	 * transaction will appear to have already occurred.
	 */
	spin_lock(&BTRFS_I(inode)->lock);
	BTRFS_I(inode)->last_sub_trans = root->log_transid;
	spin_unlock(&BTRFS_I(inode)->lock);
	if (num_written > 0) {
		err = generic_write_sync(file, pos, num_written);
		if (err < 0)
			num_written = err;
	}

	if (sync)
		atomic_dec(&BTRFS_I(inode)->sync_writers);
out:
	current->backing_dev_info = NULL;
	return num_written ? num_written : err;
}

int btrfs_release_file(struct inode *inode, struct file *filp)
{
	struct btrfs_file_private *private = filp->private_data;

	if (private && private->trans)
		btrfs_ioctl_trans_end(filp);
	if (private && private->filldir_buf)
		kfree(private->filldir_buf);
	kfree(private);
	filp->private_data = NULL;

	/*
	 * ordered_data_close is set by settattr when we are about to truncate
	 * a file from a non-zero size to a zero size.  This tries to
	 * flush down new bytes that may have been written if the
	 * application were using truncate to replace a file in place.
	 */
	if (test_and_clear_bit(BTRFS_INODE_ORDERED_DATA_CLOSE,
			       &BTRFS_I(inode)->runtime_flags))
			filemap_flush(inode->i_mapping);
	return 0;
}

static int start_ordered_ops(struct inode *inode, loff_t start, loff_t end)
{
	int ret;
	struct blk_plug plug;

	/*
	 * This is only called in fsync, which would do synchronous writes, so
	 * a plug can merge adjacent IOs as much as possible.  Esp. in case of
	 * multiple disks using raid profile, a large IO can be split to
	 * several segments of stripe length (currently 64K).
	 */
	blk_start_plug(&plug);
	atomic_inc(&BTRFS_I(inode)->sync_writers);
	ret = btrfs_fdatawrite_range(inode, start, end);
	atomic_dec(&BTRFS_I(inode)->sync_writers);
	blk_finish_plug(&plug);

	return ret;
}

/*
 * fsync call for both files and directories.  This logs the inode into
 * the tree log instead of forcing full commits whenever possible.
 *
 * It needs to call filemap_fdatawait so that all ordered extent updates are
 * in the metadata btree are up to date for copying to the log.
 *
 * It drops the inode mutex before doing the tree log commit.  This is an
 * important optimization for directories because holding the mutex prevents
 * new operations on the dir while we write to disk.
 */
int btrfs_sync_file(struct file *file, loff_t start, loff_t end, int datasync)
{
	struct dentry *dentry = file_dentry(file);
	struct inode *inode = d_inode(dentry);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_trans_handle *trans;
	struct btrfs_log_ctx ctx;
	int ret = 0;

#ifdef MY_ABC_HERE
	atomic64_inc(&root->fs_info->fsync_cnt);
#endif /* MY_ABC_HERE */
	trace_btrfs_sync_file(file, datasync);

	/*
	 * We write the dirty pages in the range and wait until they complete
	 * out of the ->i_mutex. If so, we can flush the dirty pages by
	 * multi-task, and make the performance up.  See
	 * btrfs_wait_ordered_range for an explanation of the ASYNC check.
	 */
	ret = start_ordered_ops(inode, start, end);
	if (ret)
		return ret;

	inode_lock(inode);

	/*
	 * We take the dio_sem here because the tree log stuff can race with
	 * lockless dio writes and get an extent map logged for an extent we
	 * never waited on.  We need it this high up for lockdep reasons.
	 */
	down_write(&BTRFS_I(inode)->dio_sem);

	atomic_inc(&root->log_batch);

	/*
	 * If the inode needs a full sync, make sure we use a full range to
	 * avoid log tree corruption, due to hole detection racing with ordered
	 * extent completion for adjacent ranges, and assertion failures during
	 * hole detection. Do this while holding the inode lock, to avoid races
	 * with other tasks.
	 */
	if (test_bit(BTRFS_INODE_NEEDS_FULL_SYNC,
		     &BTRFS_I(inode)->runtime_flags)) {
		start = 0;
		end = LLONG_MAX;
	}

	/*
	 * Before we acquired the inode's lock, someone may have dirtied more
	 * pages in the target range. We need to make sure that writeback for
	 * any such pages does not start while we are logging the inode, because
	 * if it does, any of the following might happen when we are not doing a
	 * full inode sync:
	 *
	 * 1) We log an extent after its writeback finishes but before its
	 *    checksums are added to the csum tree, leading to -EIO errors
	 *    when attempting to read the extent after a log replay.
	 *
	 * 2) We can end up logging an extent before its writeback finishes.
	 *    Therefore after the log replay we will have a file extent item
	 *    pointing to an unwritten extent (and no data checksums as well).
	 *
	 * So trigger writeback for any eventual new dirty pages and then we
	 * wait for all ordered extents to complete below.
	 */
	ret = start_ordered_ops(inode, start, end);
	if (ret) {
		up_write(&BTRFS_I(inode)->dio_sem);
		inode_unlock(inode);
		goto out;
	}

	/*
	 * We have to do this here to avoid the priority inversion of waiting on
	 * IO of a lower priority task while holding a transaction open.
	 *
	 * Also, the range length can be represented by u64, we have to do the
	 * typecasts to avoid signed overflow if it's [0, LLONG_MAX].
	 */
	ret = btrfs_wait_ordered_range(inode, start, (u64)end - (u64)start + 1);
	if (ret) {
		up_write(&BTRFS_I(inode)->dio_sem);
		inode_unlock(inode);
		goto out;
	}
	atomic_inc(&root->log_batch);

	smp_mb();
	if (btrfs_inode_in_log(inode, root->fs_info->generation) ||
	    BTRFS_I(inode)->last_trans <= root->fs_info->last_trans_committed) {
		/*
		 * We've had everything committed since the last time we were
		 * modified so clear this flag in case it was set for whatever
		 * reason, it's no longer relevant.
		 */
		clear_bit(BTRFS_INODE_NEEDS_FULL_SYNC,
			  &BTRFS_I(inode)->runtime_flags);
		/*
		 * An ordered extent might have started before and completed
		 * already with io errors, in which case the inode was not
		 * updated and we end up here. So check the inode's mapping
		 * flags for any errors that might have happened while doing
		 * writeback of file data.
		 */
		ret = filemap_check_errors(inode->i_mapping);
		up_write(&BTRFS_I(inode)->dio_sem);
		inode_unlock(inode);
		goto out;
	}

	/*
	 * ok we haven't committed the transaction yet, lets do a commit
	 */
	if (file->private_data)
		btrfs_ioctl_trans_end(file);

	/*
	 * We use start here because we will need to wait on the IO to complete
	 * in btrfs_sync_log, which could require joining a transaction (for
	 * example checking cross references in the nocow path).  If we use join
	 * here we could get into a situation where we're waiting on IO to
	 * happen that is blocked on a transaction trying to commit.  With start
	 * we inc the extwriter counter, so we wait for all extwriters to exit
	 * before we start blocking join'ers.  This comment is to keep somebody
	 * from thinking they are super smart and changing this to
	 * btrfs_join_transaction *cough*Josef*cough*.
	 */
	trans = btrfs_start_transaction(root, 0);
	if (IS_ERR(trans)) {
		ret = PTR_ERR(trans);
		up_write(&BTRFS_I(inode)->dio_sem);
		inode_unlock(inode);
		goto out;
	}
	trans->sync = true;

	btrfs_init_log_ctx(&ctx);

	ret = btrfs_log_dentry_safe(trans, root, dentry, start, end, &ctx);
	if (ret < 0) {
		/* Fallthrough and commit/free transaction. */
		ret = 1;
	}

	/* we've logged all the items and now have a consistent
	 * version of the file in the log.  It is possible that
	 * someone will come in and modify the file, but that's
	 * fine because the log is consistent on disk, and we
	 * have references to all of the file's extents
	 *
	 * It is possible that someone will come in and log the
	 * file again, but that will end up using the synchronization
	 * inside btrfs_sync_log to keep things safe.
	 */
	up_write(&BTRFS_I(inode)->dio_sem);
	inode_unlock(inode);

	if (ret != BTRFS_NO_LOG_SYNC) {
		if (!ret) {
			ret = btrfs_sync_log(trans, root, &ctx);
			if (!ret) {
				ret = btrfs_end_transaction(trans, root);
				goto out;
			}
		}
#ifdef MY_ABC_HERE
		atomic64_inc(&root->fs_info->fsync_full_commit_cnt);
#endif /* MY_ABC_HERE */
		ret = btrfs_commit_transaction(trans, root);
	} else {
		ret = btrfs_end_transaction(trans, root);
	}
out:
	return ret > 0 ? -EIO : ret;
}

static const struct vm_operations_struct btrfs_file_vm_ops = {
	.fault		= filemap_fault,
	.map_pages	= filemap_map_pages,
	.page_mkwrite	= btrfs_page_mkwrite,
};

static int btrfs_file_mmap(struct file	*filp, struct vm_area_struct *vma)
{
	struct address_space *mapping = filp->f_mapping;

	if (!mapping->a_ops->readpage)
		return -ENOEXEC;

	file_accessed(filp);
	vma->vm_ops = &btrfs_file_vm_ops;

	return 0;
}

static int hole_mergeable(struct inode *inode, struct extent_buffer *leaf,
			  int slot, u64 start, u64 end)
{
	struct btrfs_file_extent_item *fi;
	struct btrfs_key key;

	if (slot < 0 || slot >= btrfs_header_nritems(leaf))
		return 0;

	btrfs_item_key_to_cpu(leaf, &key, slot);
	if (key.objectid != btrfs_ino(inode) ||
	    key.type != BTRFS_EXTENT_DATA_KEY)
		return 0;

	fi = btrfs_item_ptr(leaf, slot, struct btrfs_file_extent_item);

	if (btrfs_file_extent_type(leaf, fi) != BTRFS_FILE_EXTENT_REG)
		return 0;

	if (btrfs_file_extent_disk_bytenr(leaf, fi))
		return 0;

	if (key.offset == end)
		return 1;
	if (key.offset + btrfs_file_extent_num_bytes(leaf, fi) == start)
		return 1;
	return 0;
}

static int fill_holes(struct btrfs_trans_handle *trans, struct inode *inode,
		      struct btrfs_path *path, u64 offset, u64 end)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct extent_buffer *leaf;
	struct btrfs_file_extent_item *fi;
	struct extent_map *hole_em;
	struct extent_map_tree *em_tree = &BTRFS_I(inode)->extent_tree;
	struct btrfs_key key;
	int ret;
#ifdef MY_ABC_HERE
	int modify_slot = -1;
	int del_slot = -1;
	int update_offset = 0;
	u64 num_bytes = 0;
#endif /* MY_ABC_HERE */

	if (btrfs_fs_incompat(root->fs_info, NO_HOLES))
		goto out;

	key.objectid = btrfs_ino(inode);
	key.type = BTRFS_EXTENT_DATA_KEY;
	key.offset = offset;

#ifdef MY_ABC_HERE
	ret = btrfs_search_slot(trans, root, &key, path, -1, 1);
#else
	ret = btrfs_search_slot(trans, root, &key, path, 0, 1);
#endif /* MY_ABC_HERE */
	if (ret <= 0) {
		/*
		 * We should have dropped this offset, so if we find it then
		 * something has gone horribly wrong.
		 */
		if (ret == 0)
			ret = -EINVAL;
		return ret;
	}

	leaf = path->nodes[0];
#ifdef MY_ABC_HERE
	if (hole_mergeable(inode, leaf, path->slots[0] - 1, offset, end)) {
		fi = btrfs_item_ptr(leaf, path->slots[0] - 1,
				    struct btrfs_file_extent_item);
		num_bytes = btrfs_file_extent_num_bytes(leaf, fi) + end - offset;
		modify_slot = path->slots[0] - 1;
	}

	if (hole_mergeable(inode, leaf, path->slots[0], offset, end)) {
		fi = btrfs_item_ptr(leaf, path->slots[0],
				    struct btrfs_file_extent_item);
		if (modify_slot != -1) {
			num_bytes += btrfs_file_extent_num_bytes(leaf, fi);
			del_slot = path->slots[0];
		} else {
			num_bytes = btrfs_file_extent_num_bytes(leaf, fi) + end - offset;
			modify_slot = path->slots[0];
			update_offset = 1;
		}
	}
	if (modify_slot >= 0)
		goto out;
#else
	if (hole_mergeable(inode, leaf, path->slots[0]-1, offset, end)) {
		u64 num_bytes;

		path->slots[0]--;
		fi = btrfs_item_ptr(leaf, path->slots[0],
				    struct btrfs_file_extent_item);
		num_bytes = btrfs_file_extent_num_bytes(leaf, fi) +
			end - offset;
		btrfs_set_file_extent_num_bytes(leaf, fi, num_bytes);
		btrfs_set_file_extent_ram_bytes(leaf, fi, num_bytes);
		btrfs_set_file_extent_offset(leaf, fi, 0);
		btrfs_mark_buffer_dirty(leaf);
		goto out;
	}

	if (hole_mergeable(inode, leaf, path->slots[0], offset, end)) {
		u64 num_bytes;

		key.offset = offset;
		btrfs_set_item_key_safe(root->fs_info, path, &key);
		fi = btrfs_item_ptr(leaf, path->slots[0],
				    struct btrfs_file_extent_item);
		num_bytes = btrfs_file_extent_num_bytes(leaf, fi) + end -
			offset;
		btrfs_set_file_extent_num_bytes(leaf, fi, num_bytes);
		btrfs_set_file_extent_ram_bytes(leaf, fi, num_bytes);
		btrfs_set_file_extent_offset(leaf, fi, 0);
		btrfs_mark_buffer_dirty(leaf);
		goto out;
	}
#endif /* MY_ABC_HERE */
	btrfs_release_path(path);

	ret = btrfs_insert_file_extent(trans, root, btrfs_ino(inode), offset,
				       0, 0, end - offset, 0, end - offset,
				       0, 0, 0);
	if (ret)
		return ret;

out:
#ifdef MY_ABC_HERE
	if (modify_slot >= 0) {
		fi = btrfs_item_ptr(leaf, modify_slot, struct btrfs_file_extent_item);

		btrfs_set_file_extent_num_bytes(leaf, fi, num_bytes);
		btrfs_set_file_extent_ram_bytes(leaf, fi, num_bytes);
		if (update_offset) {
			key.offset = offset;
			btrfs_set_item_key_safe(root->fs_info, path, &key);
		}
		btrfs_set_file_extent_offset(leaf, fi, 0);
#ifdef MY_DEF_HERE
		btrfs_set_file_extent_generation(leaf, fi, trans->transid);
#endif /* MY_DEF_HERE */
		btrfs_mark_buffer_dirty(leaf);
		if (del_slot >= 0) {
			ret = btrfs_del_items(trans, root, path, del_slot, 1);
			if (ret) {
				btrfs_abort_transaction(trans, root, ret);
				btrfs_release_path(path);
				return ret;
			}
		}
	}

#endif /* MY_ABC_HERE */
	btrfs_release_path(path);

	hole_em = alloc_extent_map();
	if (!hole_em) {
		btrfs_drop_extent_cache(inode, offset, end - 1, 0);
		set_bit(BTRFS_INODE_NEEDS_FULL_SYNC,
			&BTRFS_I(inode)->runtime_flags);
	} else {
		hole_em->start = offset;
		hole_em->len = end - offset;
		hole_em->ram_bytes = hole_em->len;
		hole_em->orig_start = offset;

		hole_em->block_start = EXTENT_MAP_HOLE;
		hole_em->block_len = 0;
		hole_em->orig_block_len = 0;
		hole_em->bdev = root->fs_info->fs_devices->latest_bdev;
		hole_em->compress_type = BTRFS_COMPRESS_NONE;
		hole_em->generation = trans->transid;

		do {
			btrfs_drop_extent_cache(inode, offset, end - 1, 0);
			write_lock(&em_tree->lock);
			ret = add_extent_mapping(em_tree, hole_em, 1);
			write_unlock(&em_tree->lock);
		} while (ret == -EEXIST);
		free_extent_map(hole_em);
		if (ret)
			set_bit(BTRFS_INODE_NEEDS_FULL_SYNC,
				&BTRFS_I(inode)->runtime_flags);
	}

	return 0;
}

/*
 * Find a hole extent on given inode and change start/len to the end of hole
 * extent.(hole/vacuum extent whose em->start <= start &&
 *	   em->start + em->len > start)
 * When a hole extent is found, return 1 and modify start/len.
 */
static int find_first_non_hole(struct inode *inode, u64 *start, u64 *len)
{
	struct extent_map *em;
	int ret = 0;

	em = btrfs_get_extent(inode, NULL, 0, *start, *len, 0);
	if (IS_ERR_OR_NULL(em)) {
		if (!em)
			ret = -ENOMEM;
		else
			ret = PTR_ERR(em);
		return ret;
	}

	/* Hole or vacuum extent(only exists in no-hole mode) */
	if (em->block_start == EXTENT_MAP_HOLE) {
		ret = 1;
		*len = em->start + em->len > *start + *len ?
		       0 : *start + *len - em->start - em->len;
		*start = em->start + em->len;
	}
	free_extent_map(em);
	return ret;
}

static int btrfs_insert_clone_extent(struct btrfs_trans_handle *trans,
				     struct inode *inode,
				     struct btrfs_path *path,
				     struct btrfs_clone_extent_info *clone_info,
				     const u64 clone_len,
				     const u64 bytes_to_drop)
{
	struct btrfs_fs_info *fs_info = btrfs_sb(inode->i_sb);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_file_extent_item *extent;
	struct extent_buffer *leaf;
	struct btrfs_key key;
	int slot;
	u64 ref_offset;
	int ret;
#ifdef MY_ABC_HERE
	int syno_usage;
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	int skip_quota = 0;
#endif /* MY_ABC_HERE */

	if (clone_len == 0)
		return 0;

	if (clone_info->disk_offset == 0 &&
	    btrfs_fs_incompat(fs_info, NO_HOLES)) {
#ifdef MY_ABC_HERE
		down_read(&root->rescan_lock);
		btrfs_update_inode_bytes(BTRFS_I(inode), 0, bytes_to_drop);
		btrfs_qgroup_syno_accounting(BTRFS_I(inode),
						0, bytes_to_drop, UPDATE_QUOTA);
		btrfs_usrquota_syno_accounting(BTRFS_I(inode),
						0, bytes_to_drop, UPDATE_QUOTA);
		up_read(&root->rescan_lock);
#else
		btrfs_update_inode_bytes(BTRFS_I(inode), 0, bytes_to_drop);
#endif /* MY_ABC_HERE */
		return 0;
	}

	key.objectid = btrfs_ino(inode);
	key.type = BTRFS_EXTENT_DATA_KEY;
	key.offset = clone_info->file_offset;
	ret = btrfs_insert_empty_item(trans, root, path, &key,
				      clone_info->item_size);
	if (ret)
		return ret;
	leaf = path->nodes[0];
	slot = path->slots[0];
	write_extent_buffer(leaf, clone_info->extent_buf,
			    btrfs_item_ptr_offset(leaf, slot),
			    clone_info->item_size);
	extent = btrfs_item_ptr(leaf, slot, struct btrfs_file_extent_item);
#ifdef MY_DEF_HERE
	btrfs_set_file_extent_syno_flag(leaf, extent, 0);
#endif /* MY_DEF_HERE */
	btrfs_set_file_extent_offset(leaf, extent, clone_info->data_offset);
	btrfs_set_file_extent_num_bytes(leaf, extent, clone_len);

#ifdef MY_ABC_HERE
	syno_usage = btrfs_syno_usage_ref_check(root, btrfs_ino(inode), clone_info->file_offset);
#endif /* MY_ABC_HERE */

	btrfs_mark_buffer_dirty(leaf);
	btrfs_release_path(path);

	/* If it's a hole, nothing more needs to be done. */
	if (clone_info->disk_offset == 0) {
#ifdef MY_ABC_HERE
		down_read(&root->rescan_lock);
		btrfs_update_inode_bytes(BTRFS_I(inode), 0, bytes_to_drop);
		btrfs_qgroup_syno_accounting(BTRFS_I(inode),
						0, bytes_to_drop, UPDATE_QUOTA);
		btrfs_usrquota_syno_accounting(BTRFS_I(inode),
						0, bytes_to_drop, UPDATE_QUOTA);
		up_read(&root->rescan_lock);
#else
		btrfs_update_inode_bytes(BTRFS_I(inode), 0, bytes_to_drop);
#endif /* MY_ABC_HERE */
		return 0;
	}

#ifdef MY_ABC_HERE
	down_read(&root->rescan_lock);
	btrfs_update_inode_bytes(BTRFS_I(inode), clone_len, bytes_to_drop);
	btrfs_qgroup_syno_accounting(BTRFS_I(inode),
				clone_len, bytes_to_drop, UPDATE_QUOTA);
	btrfs_usrquota_syno_accounting(BTRFS_I(inode),
				clone_len, bytes_to_drop, UPDATE_QUOTA);
	up_read(&root->rescan_lock);
#else
	btrfs_update_inode_bytes(BTRFS_I(inode), clone_len, bytes_to_drop);
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if (fs_info->syno_quota_v1_enabled &&
		clone_info->quota_enabled &&
		!clone_info->same_inode &&
		ulist_add_lru_adjust(clone_info->disko_ulist, clone_info->disk_offset, 0, GFP_NOFS)) {
		skip_quota = 0;
		if (clone_info->check_backref) {
			skip_quota = check_root_inode_ref(trans, fs_info,
				clone_info->disk_offset, 0, root->objectid, btrfs_ino(inode),
				(u64)-1, 0);
			if (skip_quota < 0) {
				ret = skip_quota;
				goto out;
			}
		}
		if (!skip_quota)
			clone_info->accounting_reserve = true;
		if (clone_info->disko_ulist->nnodes > ULIST_NODES_MAX) {
			clone_info->check_backref = true;
			ulist_remove_first(clone_info->disko_ulist);
		}
	} else {
		skip_quota = 1;
	}
#endif /* MY_ABC_HERE */

	ref_offset = clone_info->file_offset - clone_info->data_offset;
#ifdef MY_ABC_HERE
	ret = btrfs_inc_extent_ref_uid(trans, root,
			clone_info->disk_offset, clone_info->disk_len, 0,
			root->root_key.objectid,
			btrfs_ino(inode),
			ref_offset,
			skip_quota, skip_quota ? 0 : clone_info->ram_bytes,
			inode, i_uid_read(inode)
#ifdef MY_ABC_HERE
			,syno_usage
#endif /* MY_ABC_HERE */
			);
#else
	ret = btrfs_inc_extent_ref(trans, root,
			clone_info->disk_offset, clone_info->disk_len, 0,
			root->root_key.objectid,
			btrfs_ino(inode),
			ref_offset,
#ifdef MY_ABC_HERE
			skip_quota, skip_quota ? 0 : clone_info->ram_bytes,
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
			,syno_usage
#endif /* MY_ABC_HERE */
			);
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if (!ret && fs_info->syno_usrquota_v1_enabled && clone_info->set_clone_range_flag) {
		ret = btrfs_set_disk_extent_flags(trans, root, clone_info->disk_offset, clone_info->disk_len, BTRFS_EXTENT_FLAG_HAS_CLONE_RANGE, 0, 1);
	}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
out:
#endif /* MY_ABC_HERE */
	return ret;
}

/*
 * The respective range must have been previously locked, as well as the inode.
 * The end offset is inclusive (last byte of the range).
 * @clone_info is NULL for fallocate's hole punching and non-NULL for extent
 * cloning.
 * When cloning, we don't want to end up in a state where we dropped extents
 * without inserting a new one, so we must abort the transaction to avoid a
 * corruption.
 */
int btrfs_punch_hole_range(struct inode *inode, struct btrfs_path *path,
			   const u64 start, const u64 end,
			   struct btrfs_clone_extent_info *clone_info,
			   struct btrfs_trans_handle **trans_out
#ifdef MY_ABC_HERE
			   , struct btrfs_punch_hole_args *args
#endif /* MY_ABC_HERE */
			   )
{
	struct btrfs_drop_extents_args drop_args = { 0 };
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_fs_info *fs_info = btrfs_sb(inode->i_sb);
	u64 min_size = btrfs_calc_trans_metadata_size(root, 1);
	u64 ino_size = round_up(inode->i_size, fs_info->sectorsize);
	struct btrfs_trans_handle *trans = NULL;
	struct btrfs_block_rsv *rsv;
	unsigned int rsv_count;
	u64 cur_offset;
	u64 drop_end;
	u64 len = end - start;
	int ret = 0;
#ifdef MY_ABC_HERE
	/*
	 * first_punch_pos records the relative file offset of first punch position.
	 * last_punch_pos records the possible end offset of last punch position
	 * for example, if we punch at file offset 32768
	 * where this FILE_EXTENT_DATA points to extent offset 8192 of EXTENT_ITEM
	 * whose size is 1048576(1MB)
	 * first_punch_pos = (32768 - 8192);
	 * last_punch_pos = (32768 - 8192 + 1048576)
	 *
	 * partial_punch records if there's any punch that results in some part
	 * of EXTENT_ITEM being left, i.e. does this punch not remove the
	 * entire EXTENT_ITEM.
	 */
	u64 first_punch_pos = start;
	u64 last_punch_pos = end + 1;
	int partial_punch = 0;
#endif /* MY_ABC_HERE */

	if (end <= start)
		return -EINVAL;

	rsv = btrfs_alloc_block_rsv(root, BTRFS_BLOCK_RSV_TEMP);
	if (!rsv) {
		ret = -ENOMEM;
		goto out;
	}
	rsv->size = btrfs_calc_trans_metadata_size(root, 1);
	rsv->failfast = 1;

	/*
	 * 1 - update the inode
	 * 1 - removing the extents in the range
	 * 1 - adding the hole extent if no_holes isn't set or if we are cloning
	 *     an extent
	 */
	if (!btrfs_fs_incompat(fs_info, NO_HOLES) || clone_info)
		rsv_count = 3;
	else
		rsv_count = 2;

	trans = btrfs_start_transaction(root, rsv_count);
	if (IS_ERR(trans)) {
		ret = PTR_ERR(trans);
		trans = NULL;
		goto out_free;
	}

	ret = btrfs_block_rsv_migrate(&fs_info->trans_block_rsv, rsv,
				      min_size);
	BUG_ON(ret);
	trans->block_rsv = rsv;

	cur_offset = start;
	drop_args.path = path;
	drop_args.end = end + 1;
	drop_args.drop_cache = true;
#ifdef MY_ABC_HERE
	drop_args.first_punch_pos = &first_punch_pos;
	drop_args.last_punch_pos = &last_punch_pos;
	drop_args.partial_punch = &partial_punch;
#endif /* MY_ABC_HERE */
	while (cur_offset < end) {
		drop_args.start = cur_offset;
#ifdef MY_ABC_HERE
		trans->check_throttle = true;
#endif /* MY_ABC_HERE */
		ret = btrfs_drop_extents(trans, root, inode, &drop_args);
		/* If we are punching a hole decrement the inode's byte count */
		if (!clone_info)
#ifdef MY_ABC_HERE
		{
			down_read(&root->rescan_lock);
			btrfs_update_inode_bytes(BTRFS_I(inode), 0, drop_args.bytes_found);
			btrfs_qgroup_syno_accounting(BTRFS_I(inode),
					0, drop_args.bytes_found, UPDATE_QUOTA);
			btrfs_usrquota_syno_accounting(BTRFS_I(inode),
					0, drop_args.bytes_found, UPDATE_QUOTA);
			up_read(&root->rescan_lock);
		}
#else
			btrfs_update_inode_bytes(BTRFS_I(inode), 0, drop_args.bytes_found);
#endif /* MY_ABC_HERE */
		drop_end = drop_args.drop_end;
		if (ret != -ENOSPC
#ifdef MY_ABC_HERE
			&& ret != -EAGAIN
#endif /* MY_ABC_HERE */
			) {
			/*
			 * When cloning we want to avoid transaction aborts when
			 * nothing was done and we are attempting to clone parts
			 * of inline extents, in such cases -EOPNOTSUPP is
			 * returned by __btrfs_drop_extents() without having
			 * changed anything in the file.
			 */
			if (clone_info && ret && ret != -EOPNOTSUPP)
				btrfs_abort_transaction(trans, root, ret);
			break;
		}

		trans->block_rsv = &fs_info->trans_block_rsv;

		if (!clone_info && cur_offset < drop_end &&
		    cur_offset < ino_size) {
			ret = fill_holes(trans, inode, path,
					cur_offset, drop_end);
			if (ret) {
				/*
				 * If we failed then we didn't insert our hole
				 * entries for the area we dropped, so now the
				 * fs is corrupted, so we must abort the
				 * transaction.
				 */
				btrfs_abort_transaction(trans, root, ret);
				break;
			}
		}

		if (clone_info && drop_end > clone_info->file_offset) {
			u64 clone_len = drop_end - clone_info->file_offset;

			ret = btrfs_insert_clone_extent(trans, inode, path,
							clone_info, clone_len,
							drop_args.bytes_found);
			if (ret) {
				btrfs_abort_transaction(trans, root, ret);
				break;
			}
			clone_info->data_len -= clone_len;
			clone_info->data_offset += clone_len;
			clone_info->file_offset += clone_len;
		}

#ifdef MY_ABC_HERE
		if (!clone_info && args && args->non_blocking) {
			args->need_restart = true;
			args->next_offset = drop_end;
			ret = 0;
			goto out_trans;
		}
#endif /* MY_ABC_HERE */

		ret = btrfs_update_inode(trans, root, inode);
		if (ret)
			break;

		btrfs_end_transaction(trans, root);
		btrfs_btree_balance_dirty(root);

		trans = btrfs_start_transaction(root, rsv_count);
		if (IS_ERR(trans)) {
			ret = PTR_ERR(trans);
			trans = NULL;
			break;
		}

		ret = btrfs_block_rsv_migrate(&fs_info->trans_block_rsv,
					      rsv, min_size);
		BUG_ON(ret);	/* shouldn't happen */
		trans->block_rsv = rsv;

		cur_offset = drop_end;
#ifdef MY_ABC_HERE
		len = end - cur_offset;
#endif  /* MY_ABC_HERE */
		if (!clone_info && len) {
			ret = find_first_non_hole(inode, &cur_offset, &len);
			if (unlikely(ret < 0))
				break;
			if (ret && !len) {
				ret = 0;
				break;
			}
		}
	}

	/*
	 * If we were cloning, force the next fsync to be a full one since we
	 * we replaced (or just dropped in the case of cloning holes when
	 * NO_HOLES is enabled) extents and extent maps.
	 * This is for the sake of simplicity, and cloning into files larger
	 * than 16Mb would force the full fsync any way (when
	 * try_release_extent_mapping() is invoked during page cache truncation.
	 */
	if (clone_info)
		set_bit(BTRFS_INODE_NEEDS_FULL_SYNC,
			&BTRFS_I(inode)->runtime_flags);

	if (ret)
		goto out_trans;

	trans->block_rsv = &fs_info->trans_block_rsv;
	/*
	 * If we are using the NO_HOLES feature we might have had already an
	 * hole that overlaps a part of the region [lockstart, lockend] and
	 * ends at (or beyond) lockend. Since we have no file extent items to
	 * represent holes, drop_end can be less than lockend and so we must
	 * make sure we have an extent map representing the existing hole (the
	 * call to __btrfs_drop_extents() might have dropped the existing extent
	 * map representing the existing hole), otherwise the fast fsync path
	 * will not record the existence of the hole region
	 * [existing_hole_start, lockend].
	 */
	if (drop_end <= end)
		drop_end = end + 1;
	/*
	 * Don't insert file hole extent item if it's for a range beyond eof
	 * (because it's useless) or if it represents a 0 bytes range (when
	 * cur_offset == drop_end).
	 */
	if (!clone_info && cur_offset < ino_size && cur_offset < drop_end) {
		ret = fill_holes(trans, inode, path,
				cur_offset, drop_end);
		if (ret) {
			/* Same comment as above. */
			btrfs_abort_transaction(trans, root, ret);
			goto out_trans;
		}
	}
	if (clone_info) {
		ret = btrfs_insert_clone_extent(trans, inode, path, clone_info,
						clone_info->data_len,
						drop_args.bytes_found);
		if (ret) {
			btrfs_abort_transaction(trans, root, ret);
			goto out_trans;
		}
	}

out_trans:
	if (!trans)
		goto out_free;

	trans->block_rsv = &fs_info->trans_block_rsv;
	if (ret)
		btrfs_end_transaction(trans, root);
	else
		*trans_out = trans;
out_free:
	btrfs_free_block_rsv(root, rsv);
out:
#ifdef MY_ABC_HERE
	if (!ret && !clone_info && partial_punch) {
		btrfs_add_inode_defrag(NULL, inode, first_punch_pos,
				    last_punch_pos, BTRFS_INODE_DEFRAG_SYNO);
	}
#endif /* MY_ABC_HERE */
	return ret;
}

static int btrfs_punch_hole(struct inode *inode, loff_t offset, loff_t len
#ifdef MY_ABC_HERE
				, struct btrfs_punch_hole_args *args
#endif /* MY_ABC_HERE */
				)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct extent_state *cached_state = NULL;
	struct btrfs_path *path;
	struct btrfs_trans_handle *trans = NULL;
	u64 lockstart;
	u64 lockend;
	u64 tail_start;
	u64 tail_len;
	u64 orig_start = offset;
	int ret = 0;
	bool same_page;
	u64 ino_size;
	bool truncated_page = false;
	bool updated_inode = false;
#ifdef MY_DEF_HERE
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE;
	struct syno_cache_protection_parameter_command_inode_operation syno_cache_protection_parm;
	void *syno_cache_protection_req = NULL;
	int temp_err;
	struct btrfs_fs_info *fs_info = root->fs_info;
	loff_t temp_offset = offset, temp_len = len;
#endif /* MY_DEF_HERE */
#ifdef MY_ABC_HERE
	unsigned long delayed_ref_updates = 0;
#endif /* MY_ABC_HERE */

	ret = btrfs_wait_ordered_range(inode, offset, len);
	if (ret)
		return ret;

#ifdef MY_ABC_HERE
	if (args && args->non_blocking)
		btrfs_throttle(root);
#endif /* MY_ABC_HERE */

	inode_lock(inode);
	ino_size = round_up(inode->i_size, PAGE_CACHE_SIZE);
	ret = find_first_non_hole(inode, &offset, &len);
	if (ret < 0)
		goto out_only_mutex;
	if (ret && !len) {
		/* Already in a large hole */
		ret = 0;
		goto out_only_mutex;
	}

	lockstart = round_up(offset, BTRFS_I(inode)->root->sectorsize);
	lockend = round_down(offset + len,
			     BTRFS_I(inode)->root->sectorsize) - 1;
	same_page = ((offset >> PAGE_CACHE_SHIFT) ==
		    ((offset + len - 1) >> PAGE_CACHE_SHIFT));
	/*
	 * We needn't truncate any page which is beyond the end of the file
	 * because we are sure there is no data there.
	 */
	/*
	 * Only do this if we are in the same page and we aren't doing the
	 * entire page.
	 */
	if (same_page && len < PAGE_CACHE_SIZE) {
		if (offset < ino_size) {
			truncated_page = true;
			ret = btrfs_truncate_page(inode, offset, len, 0);
		} else {
			ret = 0;
		}
		goto out_only_mutex;
	}

	/* zero back part of the first page */
	if (offset < ino_size) {
		truncated_page = true;
		ret = btrfs_truncate_page(inode, offset, 0, 0);
		if (ret) {
			inode_unlock(inode);
			return ret;
		}
	}

	/* Check the aligned pages after the first unaligned page,
	 * if offset != orig_start, which means the first unaligned page
	 * including several following pages are already in holes,
	 * the extra check can be skipped */
	if (offset == orig_start) {
		/* after truncate page, check hole again */
		len = offset + len - lockstart;
		offset = lockstart;
		ret = find_first_non_hole(inode, &offset, &len);
		if (ret < 0)
			goto out_only_mutex;
		if (ret && !len) {
			ret = 0;
			goto out_only_mutex;
		}
		lockstart = offset;
	}

	/* Check the tail unaligned part is in a hole */
	tail_start = lockend + 1;
	tail_len = offset + len - tail_start;
	if (tail_len) {
		ret = find_first_non_hole(inode, &tail_start, &tail_len);
		if (unlikely(ret < 0))
			goto out_only_mutex;
		if (!ret) {
			/* zero the front end of the last page */
			if (tail_start + tail_len < ino_size) {
				truncated_page = true;
				ret = btrfs_truncate_page(inode,
						tail_start + tail_len, 0, 1);
				if (ret)
					goto out_only_mutex;
			}
		}
	}

	if (lockend < lockstart) {
		ret = 0;
		goto out_only_mutex;
	}

	while (1) {
		struct btrfs_ordered_extent *ordered;

		truncate_pagecache_range(inode, lockstart, lockend);

		lock_extent_bits(&BTRFS_I(inode)->io_tree, lockstart, lockend,
				 &cached_state);
		ordered = btrfs_lookup_first_ordered_extent(inode, lockend);

		/*
		 * We need to make sure we have no ordered extents in this range
		 * and nobody raced in and read a page in this range, if we did
		 * we need to try again.
		 */
		if ((!ordered ||
		    (ordered->file_offset + ordered->len <= lockstart ||
		     ordered->file_offset > lockend)) &&
		     !btrfs_page_exists_in_range(inode, lockstart, lockend)) {
			if (ordered)
				btrfs_put_ordered_extent(ordered);
			break;
		}
		if (ordered)
			btrfs_put_ordered_extent(ordered);
		unlock_extent_cached(&BTRFS_I(inode)->io_tree, lockstart,
				     lockend, &cached_state, GFP_NOFS);
		ret = btrfs_wait_ordered_range(inode, lockstart,
					       lockend - lockstart + 1);
		if (ret) {
			inode_unlock(inode);
			return ret;
		}
	}

#ifdef MY_DEF_HERE
	if (syno_cache_protection_is_enabled(fs_info) && inode->i_nlink > 0) {
		syno_cache_protection_req = btrfs_syno_cache_protection_alloc_command(command, fs_info, NULL);
		if (IS_ERR(syno_cache_protection_req)) {
			temp_err = PTR_ERR(syno_cache_protection_req);
			syno_cache_protection_req = NULL;
			btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection alloc command [%d] err %d", (int)command, temp_err);
		}
	}
#endif /* MY_DEF_HERE */

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	ret = btrfs_punch_hole_range(inode, path, lockstart, lockend, NULL,
				     &trans
#ifdef MY_ABC_HERE
				     , args
#endif /* MY_ABC_HERE */
				     );
	btrfs_free_path(path);
	if (ret)
		goto out;

	ASSERT(trans != NULL);
	inode_inc_iversion(inode);
	inode->i_mtime = inode->i_ctime = current_fs_time(inode->i_sb);
	ret = btrfs_update_inode(trans, root, inode);
#ifdef MY_ABC_HERE
	/* skip delayed-refs throttle in end_transaction */
	trans->sync = true;
	delayed_ref_updates = trans->delayed_ref_updates;
#endif /* MY_ABC_HERE */
	updated_inode = true;
#ifdef MY_DEF_HERE
	if (!ret && syno_cache_protection_is_enabled(fs_info) && syno_cache_protection_req) {
		memset(&syno_cache_protection_parm, 0, sizeof(syno_cache_protection_parm));
		syno_cache_protection_parm.command = command;
		syno_cache_protection_parm.transid = trans->transid;
		syno_cache_protection_parm.inode = inode;
		syno_cache_protection_parm.flags = FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE;
		syno_cache_protection_parm.offset = temp_offset;
		syno_cache_protection_parm.length = temp_len;
		temp_err = btrfs_syno_cache_protection_write_and_send_command(syno_cache_protection_req, &syno_cache_protection_parm);
		if (temp_err) {
			btrfs_warn(fs_info, "Failed to SYNO Cache Protection send command [%d] err %d", (int)command, temp_err);
		}
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */
	btrfs_end_transaction(trans, root);
	btrfs_btree_balance_dirty(root);
out:
	unlock_extent_cached(&BTRFS_I(inode)->io_tree, lockstart, lockend,
			     &cached_state, GFP_NOFS);
out_only_mutex:
	if (!updated_inode && truncated_page && !ret) {
		/*
		 * If we only end up zeroing part of a page, we still need to
		 * update the inode item, so that all the time fields are
		 * updated as well as the necessary btrfs inode in memory fields
		 * for detecting, at fsync time, if the inode isn't yet in the
		 * log tree or it's there but not up to date.
		 */
		trans = btrfs_start_transaction(root, 1);
		if (IS_ERR(trans)) {
			ret = PTR_ERR(trans);
		} else {
			int ret2;

			ret = btrfs_update_inode(trans, root, inode);
			ret2 = btrfs_end_transaction(trans, root);
			if (!ret)
				ret = ret2;
		}
	}
	inode_unlock(inode);
#ifdef MY_ABC_HERE
	if (!ret && updated_inode && 0 != delayed_ref_updates)
		btrfs_throttle_delayed_refs(root, delayed_ref_updates);
#endif /* MY_ABC_HERE */
#ifdef MY_DEF_HERE
	if (syno_cache_protection_req) {
		btrfs_syno_cache_protection_free_command(syno_cache_protection_req);
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */
	return ret;
}

/* Helper structure to record which range is already reserved */
struct falloc_range {
	struct list_head list;
	u64 start;
	u64 len;
};

/*
 * Helper function to add falloc range
 *
 * Caller should have locked the larger range of extent containing
 * [start, len)
 */
static int add_falloc_range(struct list_head *head, u64 start, u64 len)
{
	struct falloc_range *prev = NULL;
	struct falloc_range *range = NULL;

	if (list_empty(head))
		goto insert;

	/*
	 * As fallocate iterate by bytenr order, we only need to check
	 * the last range.
	 */
	prev = list_entry(head->prev, struct falloc_range, list);
	if (prev->start + prev->len == start) {
		prev->len += len;
		return 0;
	}
insert:
	range = kmalloc(sizeof(*range), GFP_KERNEL);
	if (!range)
		return -ENOMEM;
	range->start = start;
	range->len = len;
	list_add_tail(&range->list, head);
	return 0;
}

static long btrfs_fallocate(struct file *file, int mode,
			    loff_t offset, loff_t len)
{
	struct inode *inode = file_inode(file);
	struct extent_state *cached_state = NULL;
	struct falloc_range *range;
	struct falloc_range *tmp;
	struct list_head reserve_list;
	u64 cur_offset;
	u64 last_byte;
	u64 alloc_start;
	u64 alloc_end;
	u64 alloc_hint = 0;
	u64 locked_end;
	u64 actual_end = 0;
	struct extent_map *em;
	int blocksize = BTRFS_I(inode)->root->sectorsize;
	int ret;
#ifdef MY_DEF_HERE
	enum SYNO_CACHE_PROTECTION_BTRFS_COMMAND command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE;
	struct syno_cache_protection_parameter_command_inode_operation syno_cache_protection_parm;
	void *syno_cache_protection_req = NULL;
	int temp_err = 0;
	struct btrfs_fs_info *fs_info = BTRFS_I(inode)->root->fs_info;
#endif /* MY_DEF_HERE */

	alloc_start = round_down(offset, blocksize);
	alloc_end = round_up(offset + len, blocksize);
	cur_offset = alloc_start;

	/* Make sure we aren't being give some crap mode */
#ifdef CONFIG_SYNO_BTRFS_FALLOCATE_MARK_WRITTEN
	if (mode & ~(FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE |
		         FALLOC_FL_MARK_WRITTEN))
		return -EOPNOTSUPP;
	if (mode & FALLOC_FL_MARK_WRITTEN) {
		if ((mode & FALLOC_FL_KEEP_SIZE) ||
		    !(BTRFS_I(inode)->flags & BTRFS_INODE_NODATASUM))
			return -EINVAL;
	}
#else
	if (mode & ~(FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE))
		return -EOPNOTSUPP;
#endif /* CONFIG_SYNO_BTRFS_FALLOCATE_MARK_WRITTEN */

	if (mode & FALLOC_FL_PUNCH_HOLE)
		return btrfs_punch_hole(inode, offset, len
#ifdef MY_ABC_HERE
								, NULL
#endif /* MY_ABC_HERE */
								);

	/*
	 * Only trigger disk allocation, don't trigger qgroup reserve
	 *
	 * For qgroup space, it will be checked later.
	 */
	ret = btrfs_alloc_data_chunk_ondemand(inode, alloc_end - alloc_start);
	if (ret < 0)
		return ret;

	inode_lock(inode);

	if (!(mode & FALLOC_FL_KEEP_SIZE) && offset + len > inode->i_size) {
		ret = inode_newsize_ok(inode, offset + len);
		if (ret)
			goto out;
	}

	/*
	 * TODO: Move these two operations after we have checked
	 * accurate reserved space, or fallocate can still fail but
	 * with page truncated or size expanded.
	 *
	 * But that's a minor problem and won't do much harm BTW.
	 */
	if (alloc_start > inode->i_size) {
		ret = btrfs_cont_expand(inode, i_size_read(inode),
					alloc_start);
		if (ret)
			goto out;
	} else if (offset + len > inode->i_size) {
		/*
		 * If we are fallocating from the end of the file onward we
		 * need to zero out the end of the page if i_size lands in the
		 * middle of a page.
		 */
		ret = btrfs_truncate_page(inode, inode->i_size, 0, 0);
		if (ret)
			goto out;
	}

	/*
	 * wait for ordered IO before we have any locks.  We'll loop again
	 * below with the locks held.
	 */
	ret = btrfs_wait_ordered_range(inode, alloc_start,
				       alloc_end - alloc_start);
	if (ret)
		goto out;

	locked_end = alloc_end - 1;
	while (1) {
		struct btrfs_ordered_extent *ordered;

		/* the extent lock is ordered inside the running
		 * transaction
		 */
		lock_extent_bits(&BTRFS_I(inode)->io_tree, alloc_start,
				 locked_end, &cached_state);
		ordered = btrfs_lookup_first_ordered_extent(inode,
							    alloc_end - 1);
		if (ordered &&
		    ordered->file_offset + ordered->len > alloc_start &&
		    ordered->file_offset < alloc_end) {
			btrfs_put_ordered_extent(ordered);
			unlock_extent_cached(&BTRFS_I(inode)->io_tree,
					     alloc_start, locked_end,
					     &cached_state, GFP_KERNEL);
			/*
			 * we can't wait on the range with the transaction
			 * running or with the extent lock held
			 */
			ret = btrfs_wait_ordered_range(inode, alloc_start,
						       alloc_end - alloc_start);
			if (ret)
				goto out;
		} else {
			if (ordered)
				btrfs_put_ordered_extent(ordered);
			break;
		}
	}

#ifdef MY_DEF_HERE
	if (syno_cache_protection_is_enabled(fs_info) && inode->i_nlink > 0) {
		syno_cache_protection_req = btrfs_syno_cache_protection_alloc_command(command, fs_info, NULL);
		if (IS_ERR(syno_cache_protection_req)) {
			temp_err = PTR_ERR(syno_cache_protection_req);
			syno_cache_protection_req = NULL;
			btrfs_warn(fs_info, "Failed to SYNO Cache Protection alloc command [%d] err %d", (int)command, temp_err);
		}
	}
#endif /* MY_DEF_HERE */

	/* First, check if we exceed the qgroup limit */
	INIT_LIST_HEAD(&reserve_list);
	while (1) {
		em = btrfs_get_extent(inode, NULL, 0, cur_offset,
				      alloc_end - cur_offset, 0);
		if (IS_ERR_OR_NULL(em)) {
			if (!em)
				ret = -ENOMEM;
			else
				ret = PTR_ERR(em);
			break;
		}
		last_byte = min(extent_map_end(em), alloc_end);
		actual_end = min_t(u64, extent_map_end(em), offset + len);
		last_byte = ALIGN(last_byte, blocksize);
		if (em->block_start == EXTENT_MAP_HOLE ||
		    (cur_offset >= inode->i_size &&
		     !test_bit(EXTENT_FLAG_PREALLOC, &em->flags))) {
			ret = add_falloc_range(&reserve_list, cur_offset,
					       last_byte - cur_offset);
			if (ret < 0) {
				free_extent_map(em);
				break;
			}
			ret = btrfs_qgroup_reserve_data(inode, cur_offset,
					last_byte - cur_offset);
			if (ret < 0) {
#ifdef MY_ABC_HERE
				cur_offset = last_byte;
#endif /* MY_ABC_HERE */
				free_extent_map(em);
				break;
			}
		} else {
			/*
			 * Do not need to reserve unwritten extent for this
			 * range, free reserved data space first, otherwise
			 * it'll result in false ENOSPC error.
			 */
			btrfs_free_reserved_data_space(inode, cur_offset,
				last_byte - cur_offset);
		}
		free_extent_map(em);
		cur_offset = last_byte;
		if (cur_offset >= alloc_end)
			break;
	}

	/*
	 * If ret is still 0, means we're OK to fallocate.
	 * Or just cleanup the list and exit.
	 */
	list_for_each_entry_safe(range, tmp, &reserve_list, list) {
		if (!ret)
			ret = btrfs_prealloc_file_range(inode, mode,
					range->start,
					range->len, i_blocksize(inode),
					offset + len, &alloc_hint);
		else
			btrfs_free_reserved_data_space(inode, range->start,
						       range->len);
		list_del(&range->list);
		kfree(range);
	}
	if (ret < 0)
		goto out_unlock;

	if (actual_end > inode->i_size &&
	    !(mode & FALLOC_FL_KEEP_SIZE)) {
		struct btrfs_trans_handle *trans;
		struct btrfs_root *root = BTRFS_I(inode)->root;

		/*
		 * We didn't need to allocate any more space, but we
		 * still extended the size of the file so we need to
		 * update i_size and the inode item.
		 */
		trans = btrfs_start_transaction(root, 1);
		if (IS_ERR(trans)) {
			ret = PTR_ERR(trans);
		} else {
			inode->i_ctime = current_fs_time(inode->i_sb);
			i_size_write(inode, actual_end);
			btrfs_ordered_update_i_size(inode, actual_end, NULL);
			ret = btrfs_update_inode(trans, root, inode);
			if (ret)
				btrfs_end_transaction(trans, root);
			else
				ret = btrfs_end_transaction(trans, root);
		}
	}
#ifdef MY_DEF_HERE
	if (!ret && syno_cache_protection_is_enabled(fs_info) && syno_cache_protection_req) {
		struct btrfs_trans_handle *trans;
		struct btrfs_root *root = BTRFS_I(inode)->root;

		trans = btrfs_join_transaction(root);
		if (IS_ERR(trans)) {
			temp_err = PTR_ERR(trans);
			trans = NULL;
		}
		if (trans) {
			memset(&syno_cache_protection_parm, 0, sizeof(syno_cache_protection_parm));
			syno_cache_protection_parm.command = command;
			syno_cache_protection_parm.transid = trans->transid;
			syno_cache_protection_parm.inode = inode;
			syno_cache_protection_parm.flags = mode;
			syno_cache_protection_parm.offset = offset;
			syno_cache_protection_parm.length = len;
			temp_err = btrfs_syno_cache_protection_write_and_send_command(syno_cache_protection_req, &syno_cache_protection_parm);
			syno_cache_protection_req = NULL;
			btrfs_end_transaction(trans, root);
		}
		if (temp_err) {
			btrfs_warn(fs_info, "Failed to SYNO Cache Protection send command [%d] err %d", (int)command, temp_err);
		}
	}
#endif /* MY_DEF_HERE */
out_unlock:
	unlock_extent_cached(&BTRFS_I(inode)->io_tree, alloc_start, locked_end,
			     &cached_state, GFP_KERNEL);
out:
	inode_unlock(inode);
	/* Let go of our reservation. */
	if (ret != 0)
		btrfs_free_reserved_data_space(inode, alloc_start,
				       alloc_end - cur_offset);
#ifdef MY_DEF_HERE
	if (syno_cache_protection_req) {
		btrfs_syno_cache_protection_free_command(syno_cache_protection_req);
		syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */
	return ret;
}

#ifdef MY_ABC_HERE
static long btrfs_non_blocking_punch_hole(struct file *file, loff_t offset, loff_t len)
{
	int ret;
	struct inode *inode = file_inode(file);
	loff_t cur_offset = offset;
	loff_t end = offset + len;
	struct btrfs_punch_hole_args args;

	memset(&args, 0, sizeof(args));
	args.non_blocking = true;
	do {
		args.need_restart = false;
		args.next_offset = 0;
		len = end - cur_offset;
		ret = btrfs_punch_hole(inode, cur_offset, len, &args);
		if (ret)
			break;
		if (args.need_restart)
			cur_offset = args.next_offset;
		else
			cur_offset += len;
	} while (cur_offset < end);

	return ret;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
int btrfs_fallocate_check_punch(struct file *file, loff_t offset, loff_t len, loff_t *extent_align_offset, loff_t *extent_len)
{
	struct inode *inode = file_inode(file);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct extent_buffer *leaf;
	struct btrfs_path *path = NULL;
	u64 ino = btrfs_ino(inode);
	struct btrfs_file_extent_item *fi;
	struct btrfs_key found_key;
	u64 end = offset + len;
	u64 real_extent_offset = offset;
	u64 real_extent_len = len;
	int extent_type;
	u64 disk_bytenr;
	int ret;

	if (!extent_align_offset || !extent_len) {
		ret = -EINVAL;
		goto out;
	}

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	ret = btrfs_lookup_file_extent(NULL, root, path, ino, offset, 0);
	if (ret < 0)
		goto out;

	if (ret > 0 && path->slots[0] > 0) {
		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &found_key,
				      path->slots[0] - 1);
		if (found_key.objectid == ino &&
		    found_key.type == BTRFS_EXTENT_DATA_KEY)
			path->slots[0]--;
	}

next_slot:
	leaf = path->nodes[0];
	if (path->slots[0] >= btrfs_header_nritems(leaf)) {
		ret = btrfs_next_leaf(root, path);
		if (ret < 0)
			goto out;
		if (ret > 0)
			goto success;
		leaf = path->nodes[0];
	}

	btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);

	if (found_key.objectid != ino ||
		found_key.type != BTRFS_EXTENT_DATA_KEY ||
		found_key.offset > end)
		goto success;

	fi = btrfs_item_ptr(leaf, path->slots[0],
			    struct btrfs_file_extent_item);
	extent_type = btrfs_file_extent_type(leaf, fi);

	if (extent_type == BTRFS_FILE_EXTENT_REG ||
	    extent_type == BTRFS_FILE_EXTENT_PREALLOC) {
		disk_bytenr = btrfs_file_extent_disk_bytenr(leaf, fi);
		if (disk_bytenr == 0) {
			path->slots[0]++;
			goto next_slot;
		}
		real_extent_offset = found_key.offset;
		real_extent_len = btrfs_file_extent_num_bytes(leaf, fi); 
	}
success:
	*extent_align_offset = real_extent_offset;
	*extent_len = real_extent_len;
	ret = 0;
out:
	btrfs_free_path(path);
	return ret;
}
#endif /* MY_ABC_HERE */

static int find_desired_extent(struct inode *inode, loff_t *offset, int whence)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct extent_map *em = NULL;
	struct extent_state *cached_state = NULL;
	u64 lockstart;
	u64 lockend;
	u64 start;
	u64 len;
	int ret = 0;

	if (inode->i_size == 0)
		return -ENXIO;

	/*
	 * *offset can be negative, in this case we start finding DATA/HOLE from
	 * the very start of the file.
	 */
	start = max_t(loff_t, 0, *offset);

	lockstart = round_down(start, root->sectorsize);
	lockend = round_up(i_size_read(inode), root->sectorsize);
	if (lockend <= lockstart)
		lockend = lockstart + root->sectorsize;
	lockend--;
	len = lockend - lockstart + 1;

	lock_extent_bits(&BTRFS_I(inode)->io_tree, lockstart, lockend,
			 &cached_state);

	while (start < inode->i_size) {
		em = btrfs_get_extent_fiemap(inode, NULL, 0, start, len, 0);
		if (IS_ERR(em)) {
			ret = PTR_ERR(em);
			em = NULL;
			break;
		}

		if (whence == SEEK_HOLE &&
		    (em->block_start == EXTENT_MAP_HOLE ||
		     test_bit(EXTENT_FLAG_PREALLOC, &em->flags)))
			break;
		else if (whence == SEEK_DATA &&
			   (em->block_start != EXTENT_MAP_HOLE &&
			    !test_bit(EXTENT_FLAG_PREALLOC, &em->flags)))
			break;

		start = em->start + em->len;
		free_extent_map(em);
		em = NULL;
		cond_resched();
	}
	free_extent_map(em);
	if (!ret) {
		if (whence == SEEK_DATA && start >= inode->i_size)
			ret = -ENXIO;
		else
			*offset = min_t(loff_t, start, inode->i_size);
	}
	unlock_extent_cached(&BTRFS_I(inode)->io_tree, lockstart, lockend,
			     &cached_state, GFP_NOFS);
	return ret;
}

static loff_t btrfs_file_llseek(struct file *file, loff_t offset, int whence)
{
	struct inode *inode = file->f_mapping->host;
	int ret;

	inode_lock(inode);
	switch (whence) {
	case SEEK_END:
	case SEEK_CUR:
		offset = generic_file_llseek(file, offset, whence);
		goto out;
	case SEEK_DATA:
	case SEEK_HOLE:
		if (offset >= i_size_read(inode)) {
			inode_unlock(inode);
			return -ENXIO;
		}

		ret = find_desired_extent(inode, &offset, whence);
		if (ret) {
			inode_unlock(inode);
			return ret;
		}
	}

	offset = vfs_setpos(file, offset, inode->i_sb->s_maxbytes);
out:
	inode_unlock(inode);
	return offset;
}

#if defined(MY_DEF_HERE)
#ifdef CONFIG_SPLICE_FROM_SOCKET
static ssize_t btrfs_splice_from_socket(struct file *file, struct socket *sock,
					loff_t __user *ppos, size_t write_bytes)
{
	struct inode *inode = file_inode(file);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct page **pages = NULL;
	struct extent_state *cached_state = NULL;
	u64 release_bytes = 0;
	u64 lockstart;
	u64 lockend;
	struct kvec *iov = NULL;
	struct msghdr msg = { 0 };
	size_t offset, offset_tmp, remaining;
	size_t num_pages, dirty_pages;
	size_t copied = 0;
	size_t reserve_bytes;
	u64 start_pos;
	ssize_t err = 0;
	int ret = 0;
	bool only_release_metadata = false;
	bool need_unlock;
	loff_t pos;
	int i;
	bool sync = (file->f_flags & O_DSYNC) || IS_SYNC(file->f_mapping->host);
	struct kiocb iocb;
	struct iov_iter iter;

	init_sync_kiocb(&iocb, file);

	if (unlikely(iocb.ki_flags & IOCB_DIRECT))
		return -EINVAL;

	if (copy_from_user(&iocb.ki_pos, ppos, sizeof(loff_t)))
		return -EFAULT;

	/* limit the splice to 128K */
	write_bytes = min_t(size_t, write_bytes, SZ_128K);

	/* minimal init of iter, used by write_check only */
	iov_iter_init(&iter, WRITE, NULL, 0, write_bytes);

	file_start_write(file);

	mutex_lock(&inode->i_mutex);
	err = generic_write_checks(&iocb, &iter);
	if (err <= 0) {
		pr_debug("%s: generic_write_checks err, write_bytes %zu\n",
			 __func__, write_bytes);
		mutex_unlock(&inode->i_mutex);
		goto out;
	}
	pos = iocb.ki_pos;
	write_bytes = iov_iter_count(&iter);

	if (write_bytes == 0) {
		mutex_unlock(&inode->i_mutex);
		goto out;
	}

	current->backing_dev_info = inode_to_bdi(inode);
	err = file_remove_privs(file);
	if (err) {
		pr_debug("%s: file_remove_privs, err %zu\n", __func__, err);
		mutex_unlock(&inode->i_mutex);
		goto out;
	}

	/*
	 * If BTRFS flips readonly due to some impossible error
	 * (fs_info->fs_state now has BTRFS_SUPER_FLAG_ERROR),
	 * although we have opened a file as writable, we have
	 * to stop this write operation to ensure FS consistency.
	 */
	if (test_bit(BTRFS_FS_STATE_ERROR, &root->fs_info->fs_state)) {
		mutex_unlock(&inode->i_mutex);
		err = -EROFS;
		goto out;
	}

	/*
	 * We reserve space for updating the inode when we reserve space for the
	 * extent we are going to write, so we will enospc out there.  We don't
	 * need to start yet another transaction to update the inode as we will
	 * update the inode when we finish writing whatever data we write.
	 */
	update_time_for_write(inode);

	start_pos = round_down(pos, root->sectorsize);
	if (start_pos > i_size_read(inode)) {
		u64 end_pos;
		/* Expand hole size to cover write data, preventing empty gap */
		end_pos = round_up(pos + write_bytes, root->sectorsize);
		err = btrfs_cont_expand(inode, i_size_read(inode), end_pos);
		if (err) {
			mutex_unlock(&inode->i_mutex);
			goto out;
		}
	}

	if (sync)
		atomic_inc(&BTRFS_I(inode)->sync_writers);

	offset = pos & (PAGE_CACHE_SIZE - 1);
	num_pages = DIV_ROUND_UP(write_bytes + offset,
				 PAGE_CACHE_SIZE);

	pages = kmalloc_array(num_pages, sizeof(*pages), GFP_NOFS);
	if (!pages) {
		ret = -ENOMEM;
		goto out_free;
	}

	iov = kmalloc_array(num_pages, sizeof(*iov), GFP_NOFS);
	if (!iov) {
		ret = -ENOMEM;
		goto out_free;
	}

	reserve_bytes = num_pages << PAGE_CACHE_SHIFT;

	if (BTRFS_I(inode)->flags & (BTRFS_INODE_NODATACOW |
				     BTRFS_INODE_PREALLOC)) {
		ret = check_can_nocow(inode, pos, &write_bytes);
		if (ret < 0)
			goto out_free;
		if (ret > 0) {
			/*
			 * For nodata cow case, no need to reserve
			 * data space.
			 */
			only_release_metadata = true;
			/*
			 * our prealloc extent may be smaller than
			 * write_bytes, so scale down.
			 */
			num_pages = DIV_ROUND_UP(write_bytes + offset,
						 PAGE_CACHE_SIZE);
			reserve_bytes = num_pages << PAGE_CACHE_SHIFT;
			goto reserve_metadata;
		}
	}
	ret = btrfs_check_data_free_space(inode, pos, write_bytes);
	if (ret < 0)
		goto out_free;

reserve_metadata:
	ret = btrfs_delalloc_reserve_metadata(inode, reserve_bytes);
	if (ret) {
		if (!only_release_metadata)
			btrfs_free_reserved_data_space(inode, pos,
						       write_bytes);
		else
			btrfs_end_write_no_snapshoting(root);
		goto out_free;
	}

	release_bytes = reserve_bytes;
	need_unlock = false;
again:
	/*
	 * This is going to setup the pages array with the number of
	 * pages we want, so we don't really need to worry about the
	 * contents of pages from loop to loop
	 */
	ret = prepare_pages(inode, pages, num_pages,
			    pos, write_bytes,
			    false);

	if (ret)
		goto out_free;

	ret = lock_and_cleanup_extent_if_need(inode, pages, num_pages,
					      pos, &lockstart, &lockend,
					      &cached_state);
	if (ret < 0) {
		if (ret == -EAGAIN)
			goto again;
		goto out_free;
	} else if (ret > 0) {
		need_unlock = true;
		ret = 0;
	}

	remaining = write_bytes;
	offset_tmp = offset;
	for (i = 0; i < num_pages; i++) {
		unsigned int bytes = min_t(unsigned int,
					   PAGE_CACHE_SIZE - offset_tmp,
					   remaining);

		iov[i].iov_base = kmap(pages[i]) + offset_tmp;
		iov[i].iov_len = bytes;
		offset_tmp = 0;
		remaining -= bytes;
	}

	/* receive the data from socket now */
	copied = kernel_recvmsg(sock, &msg, iov, num_pages, write_bytes,
				MSG_WAITALL);

	for (i = 0; i < num_pages; i++)
		kunmap(pages[i]);

	if (copied == 0)
		dirty_pages = 0;
	else
		dirty_pages = DIV_ROUND_UP(copied + offset,
					   PAGE_CACHE_SIZE);

	/*
	 * If we had a short copy we need to release the excess delaloc
	 * bytes we reserved.  We need to increment outstanding_extents
	 * because btrfs_delalloc_release_space will decrement it, but
	 * we still have an outstanding extent for the chunk we actually
	 * managed to copy.
	 */
	if (num_pages > dirty_pages) {
		release_bytes = (num_pages - dirty_pages) <<
			PAGE_CACHE_SHIFT;
		if (copied > 0) {
			spin_lock(&BTRFS_I(inode)->lock);
			BTRFS_I(inode)->outstanding_extents++;
			spin_unlock(&BTRFS_I(inode)->lock);
		}
		if (only_release_metadata) {
			btrfs_delalloc_release_metadata(inode,
							release_bytes);
		} else {
			u64 __pos;

			__pos = round_down(pos, root->sectorsize) +
				(dirty_pages << PAGE_CACHE_SHIFT);
			btrfs_delalloc_release_space(inode, __pos,
						     release_bytes);
		}
	}

	release_bytes = dirty_pages << PAGE_CACHE_SHIFT;

	if (copied > 0)
		ret = btrfs_dirty_pages(root, inode, pages,
					dirty_pages, pos, copied,
					NULL);
	if (need_unlock)
		unlock_extent_cached(&BTRFS_I(inode)->io_tree,
				     lockstart, lockend, &cached_state,
				     GFP_NOFS);
	if (ret) {
		btrfs_drop_pages(pages, num_pages);
		copied = 0;
		goto out_free;
	}

	release_bytes = 0;
	if (only_release_metadata)
		btrfs_end_write_no_snapshoting(root);

	if (only_release_metadata && copied > 0) {
		lockstart = round_down(pos, root->sectorsize);
		lockend = lockstart +
			(dirty_pages << PAGE_CACHE_SHIFT) - 1;

		set_extent_bit(&BTRFS_I(inode)->io_tree, lockstart,
			       lockend, EXTENT_NORESERVE, NULL,
			       NULL, GFP_NOFS);
		only_release_metadata = false;
	}

	btrfs_drop_pages(pages, num_pages);

	cond_resched();

	balance_dirty_pages_ratelimited(inode->i_mapping);
	if (dirty_pages < (root->nodesize >> PAGE_CACHE_SHIFT) + 1)
		btrfs_btree_balance_dirty(root);

out_free:
	kfree(iov);
	kfree(pages);

	if (release_bytes) {
		if (only_release_metadata) {
			btrfs_end_write_no_snapshoting(root);
			btrfs_delalloc_release_metadata(inode, release_bytes);
		} else {
			btrfs_delalloc_release_space(inode, pos,
						     release_bytes);
		}
	}

	mutex_unlock(&inode->i_mutex);

	/*
	 * We also have to set last_sub_trans to the current log transid,
	 * otherwise subsequent syncs to a file that's been synced in this
	 * transaction will appear to have already occurred.
	 */
	spin_lock(&BTRFS_I(inode)->lock);
	BTRFS_I(inode)->last_sub_trans = root->log_transid;
	spin_unlock(&BTRFS_I(inode)->lock);
	if (copied > 0)
		ret = generic_write_sync(file, pos, copied);

	if (sync)
		atomic_dec(&BTRFS_I(inode)->sync_writers);

	if (!ret) {
		pos += copied;
		if (copy_to_user(ppos, &pos, sizeof(*ppos)))
			ret = -EFAULT;
	}
out:
	current->backing_dev_info = NULL;

	file_end_write(file);

	return ret ? ret : copied;
}
#endif
#endif /* MY_DEF_HERE */

const struct file_operations btrfs_file_operations = {
	.llseek		= btrfs_file_llseek,
	.read_iter      = generic_file_read_iter,
	.splice_read	= generic_file_splice_read,
#if defined(MY_DEF_HERE)
#ifdef CONFIG_SPLICE_FROM_SOCKET
	.splice_from_socket = btrfs_splice_from_socket,
#endif
#endif /* MY_DEF_HERE */
	.write_iter	= btrfs_file_write_iter,
	.mmap		= btrfs_file_mmap,
	.open		= generic_file_open,
	.release	= btrfs_release_file,
	.fsync		= btrfs_sync_file,
	.fallocate	= btrfs_fallocate,
#ifdef MY_ABC_HERE
	.fallocate_check_punch = btrfs_fallocate_check_punch,
#endif /* MY_ABC_HERE */
	.unlocked_ioctl	= btrfs_ioctl,
#ifdef MY_ABC_HERE
	.syno_recvfile = btrfs_recvfile,
#ifdef MY_ABC_HERE
	.ecryptfs_zero_copy = btrfs_ecryptfs_zero_copy,
#endif /* MY_ABC_HERE */
#endif /* MY_ABC_HERE */
#ifdef CONFIG_COMPAT
	.compat_ioctl	= btrfs_compat_ioctl,
#endif
	.clone_file_range = btrfs_clone_file_range,
#ifdef MY_ABC_HERE
	.clone_check_compr = btrfs_clone_check_compr,
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	.non_blocking_punch_hole = btrfs_non_blocking_punch_hole,
#endif
#ifdef MY_ABC_HERE
	.quota_query = btrfs_quota_query,
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	.syno_space_usage = btrfs_syno_space_usage,
#endif /* MY_ABC_HERE */
};

void btrfs_auto_defrag_exit(void)
{
	kmem_cache_destroy(btrfs_inode_defrag_cachep);
}

int __init btrfs_auto_defrag_init(void)
{
	btrfs_inode_defrag_cachep = kmem_cache_create("btrfs_inode_defrag",
					sizeof(struct inode_defrag), 0,
					SLAB_RECLAIM_ACCOUNT | SLAB_MEM_SPREAD,
					NULL);
	if (!btrfs_inode_defrag_cachep)
		return -ENOMEM;

	return 0;
}

int btrfs_fdatawrite_range(struct inode *inode, loff_t start, loff_t end)
{
	int ret;

	/*
	 * So with compression we will find and lock a dirty page and clear the
	 * first one as dirty, setup an async extent, and immediately return
	 * with the entire range locked but with nobody actually marked with
	 * writeback.  So we can't just filemap_write_and_wait_range() and
	 * expect it to work since it will just kick off a thread to do the
	 * actual work.  So we need to call filemap_fdatawrite_range _again_
	 * since it will wait on the page lock, which won't be unlocked until
	 * after the pages have been marked as writeback and so we're good to go
	 * from there.  We have to do this otherwise we'll miss the ordered
	 * extents and that results in badness.  Please Josef, do not think you
	 * know better and pull this out at some point in the future, it is
	 * right and you are wrong.
	 */
	ret = filemap_fdatawrite_range(inode->i_mapping, start, end);
	if (!ret && test_bit(BTRFS_INODE_HAS_ASYNC_EXTENT,
			     &BTRFS_I(inode)->runtime_flags))
		ret = filemap_fdatawrite_range(inode->i_mapping, start, end);

	return ret;
}
