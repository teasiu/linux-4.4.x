#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Copyright (C) 2011 STRATO.  All rights reserved.
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

#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/btrfs.h>
#ifdef MY_ABC_HERE
#include <net/netlink.h>
#include <net/genetlink.h>
#endif /* MY_ABC_HERE */

#include "ctree.h"
#include "transaction.h"
#include "disk-io.h"
#include "locking.h"
#include "ulist.h"
#include "backref.h"
#include "extent_io.h"
#include "qgroup.h"


/* TODO XXX FIXME
 *  - subvol delete -> delete when ref goes to 0? delete limits also?
 *  - reorganize keys
 *  - compressed
 *  - sync
 *  - copy also limits on subvol creation
 *  - limit
 *  - caches fuer ulists
 *  - performance benchmarks
 *  - check all ioctl parameters
 */

/*
 * one struct for each qgroup, organized in fs_info->qgroup_tree.
 */
struct btrfs_qgroup {
	u64 qgroupid;

	/*
	 * state
	 */
	u64 rfer;	/* referenced */
	u64 rfer_cmpr;	/* referenced compressed */
	u64 excl;	/* exclusive */
	u64 excl_cmpr;	/* exclusive compressed */

	/*
	 * limits
	 */
	u64 lim_flags;	/* which limits are set */
	u64 max_rfer;
	u64 max_excl;
#ifdef MY_ABC_HERE
	union {
		u64 soft_rfer;
		u64 rsv_rfer;
	};
	union {
		u64 soft_excl;
		u64 rsv_excl;
	};
#else
	u64 rsv_rfer;
	u64 rsv_excl;
#endif /* MY_ABC_HERE */

	/*
	 * reservation tracking
	 */
	u64 reserved;

	/*
	 * lists
	 */
	struct list_head groups;  /* groups this group is member of */
	struct list_head members; /* groups that are members of this group */
	struct list_head dirty;   /* dirty groups */
	struct rb_node node;	  /* tree of qgroups */

	/*
	 * temp variables for accounting operations
	 * Refer to qgroup_shared_accounting() for details.
	 */
	u64 old_refcnt;
	u64 new_refcnt;
#ifdef MY_ABC_HERE
	bool need_rescan;
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	int last_sent;
#endif /* MY_ABC_HERE */
};

#ifdef MY_ABC_HERE
enum {
	SENT_UNDER = -1,
	SENT_NONE = 0,
	SENT_OVER = 1,
};

u64 qgroup_soft_limit = 0;

static const struct genl_multicast_group qgroup_mcgrps[] = {
	{ .name = "events", },
};

/* Netlink family structure for quota */
static struct genl_family btrfs_qgroup_genl_family = {
	.module = THIS_MODULE,
	.hdrsize = 0,
	.name = "BTRFS_QUOTA",
	.version = 1,
	.maxattr = QGROUP_NL_A_MAX,
	.mcgrps = qgroup_mcgrps,
	.n_mcgrps = ARRAY_SIZE(qgroup_mcgrps),
};

static void prepare_netlink_notification(struct btrfs_qgroup *qg,
	u64 *soft_qgroup_subvol_id, u64 *soft_qgroup_limit, u64 *soft_qgroup_used,
	bool *over_limit)
{
	u64 soft_limit;

	if (!(qg->lim_flags & BTRFS_QGROUP_LIMIT_MAX_RFER) || !qgroup_soft_limit)
		return;

	soft_limit = div_u64(qg->max_rfer * qgroup_soft_limit, 100);
	// Should we send QGROUP_NL_C_OVER_LIMIT?
	if (qg->last_sent != SENT_OVER && qg->rfer > soft_limit) {
		qg->last_sent = SENT_OVER;
		*over_limit = true;
		goto notify;
	}

	// Should we send QGROUP_NL_C_UNDER_LIMIT?
	if (qg->last_sent != SENT_UNDER) {
		if (soft_limit <= SZ_1M * 100)
			return;
		if (qg->rfer >= div_u64(qg->max_rfer * (qgroup_soft_limit - 1), 100))
			return;
		if (qg->rfer >= soft_limit - (SZ_1M * 100))
			return;

		qg->last_sent = SENT_UNDER;
		*over_limit = false;
		goto notify;
	}

	return;
notify:
	*soft_qgroup_subvol_id = qg->qgroupid;
	*soft_qgroup_limit = qg->max_rfer;
	*soft_qgroup_used = qg->rfer;
}

static void send_netlink_notification(struct btrfs_fs_info *fs_info, u64 qgroupid,
		u64 quota_limit, u64 quota_used, int type)
{
	static atomic_t seq = ATOMIC_INIT(0);
	struct sk_buff *skb;
	void *msg_head;
	int ret;
	int msg_size = nla_total_size(BTRFS_FSID_SIZE) + (3 * nla_total_size(sizeof(u64)));

	/* We have to allocate using GFP_NOFS as we are called from a
	 * filesystem performing write and thus further recursion into
	 * the fs to free some data could cause deadlocks. */
	skb = genlmsg_new(msg_size, GFP_NOFS);
	if (!skb) {
		btrfs_warn(fs_info, "Not enough memory to send qgroup warning.\n");
		return;
	}
	msg_head = genlmsg_put(skb, 0, atomic_add_return(1, &seq),
			&btrfs_qgroup_genl_family, 0, type);
	if (!msg_head) {
		btrfs_warn(fs_info, "Cannot store netlink header in qgroup warning.\n");
		goto err_out;
	}
	ret = nla_put(skb, QGROUP_NL_A_FSID, BTRFS_FSID_SIZE, fs_info->super_copy->fsid);
	if (ret)
		goto attr_err_out;
	ret = nla_put_u64(skb, QGROUP_NL_A_SUBVOL_ID, qgroupid);
	if (ret)
		goto attr_err_out;
	ret = nla_put_u64(skb, QGROUP_NL_A_QUOTA_LIMIT, quota_limit);
	if (ret)
		goto attr_err_out;
	ret = nla_put_u64(skb, QGROUP_NL_A_QUOTA_USED, quota_used);
	if (ret)
		goto attr_err_out;
	genlmsg_end(skb, msg_head);

	genlmsg_multicast(&btrfs_qgroup_genl_family, skb, 0, 0, GFP_NOFS);
	return;

attr_err_out:
	btrfs_warn(fs_info, "Not enough space to compose qgroup netlink message!\n");
err_out:
	kfree_skb(skb);
}

int __init qgroup_netlink_init(void)
{
	if (genl_register_family(&btrfs_qgroup_genl_family) != 0)
		printk(KERN_ERR
		       "Failed to create btrfs qgroup netlink interface.\n");
	return 0;
};

void qgroup_netlink_exit(void)
{
	genl_unregister_family(&btrfs_qgroup_genl_family);
};
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
#else
static void btrfs_qgroup_update_old_refcnt(struct btrfs_qgroup *qg, u64 seq,
					   int mod)
{
	if (qg->old_refcnt < seq)
		qg->old_refcnt = seq;
	qg->old_refcnt += mod;
}

static void btrfs_qgroup_update_new_refcnt(struct btrfs_qgroup *qg, u64 seq,
					   int mod)
{
	if (qg->new_refcnt < seq)
		qg->new_refcnt = seq;
	qg->new_refcnt += mod;
}

static inline u64 btrfs_qgroup_get_old_refcnt(struct btrfs_qgroup *qg, u64 seq)
{
	if (qg->old_refcnt < seq)
		return 0;
	return qg->old_refcnt - seq;
}

static inline u64 btrfs_qgroup_get_new_refcnt(struct btrfs_qgroup *qg, u64 seq)
{
	if (qg->new_refcnt < seq)
		return 0;
	return qg->new_refcnt - seq;
}
#endif /* MY_ABC_HERE */

/*
 * glue structure to represent the relations between qgroups.
 */
struct btrfs_qgroup_list {
	struct list_head next_group;
	struct list_head next_member;
	struct btrfs_qgroup *group;
	struct btrfs_qgroup *member;
};

#define ptr_to_u64(x) ((u64)(uintptr_t)x)
#define u64_to_ptr(x) ((struct btrfs_qgroup *)(uintptr_t)x)

#ifdef MY_ABC_HERE
static int
syno_quota_rescan_init(struct btrfs_fs_info *fs_info, u64 progress_objectid,
		   int init_flags);
#else
static int
qgroup_rescan_init(struct btrfs_fs_info *fs_info, u64 progress_objectid,
		   int init_flags);
static void qgroup_rescan_zero_tracking(struct btrfs_fs_info *fs_info);
#endif /* MY_ABC_HERE */

/* must be called with qgroup_ioctl_lock held */
static struct btrfs_qgroup *find_qgroup_rb(struct btrfs_fs_info *fs_info,
					   u64 qgroupid)
{
	struct rb_node *n = fs_info->qgroup_tree.rb_node;
	struct btrfs_qgroup *qgroup;

	while (n) {
		qgroup = rb_entry(n, struct btrfs_qgroup, node);
		if (qgroup->qgroupid < qgroupid)
			n = n->rb_left;
		else if (qgroup->qgroupid > qgroupid)
			n = n->rb_right;
		else
			return qgroup;
	}
	return NULL;
}

/* must be called with qgroup_lock held */
static struct btrfs_qgroup *add_qgroup_rb(struct btrfs_fs_info *fs_info,
					  u64 qgroupid)
{
	struct rb_node **p = &fs_info->qgroup_tree.rb_node;
	struct rb_node *parent = NULL;
	struct btrfs_qgroup *qgroup;

	while (*p) {
		parent = *p;
		qgroup = rb_entry(parent, struct btrfs_qgroup, node);

		if (qgroup->qgroupid < qgroupid)
			p = &(*p)->rb_left;
		else if (qgroup->qgroupid > qgroupid)
			p = &(*p)->rb_right;
		else
			return qgroup;
	}

	qgroup = kzalloc(sizeof(*qgroup), GFP_ATOMIC);
	if (!qgroup)
		return ERR_PTR(-ENOMEM);

	qgroup->qgroupid = qgroupid;
	INIT_LIST_HEAD(&qgroup->groups);
	INIT_LIST_HEAD(&qgroup->members);
	INIT_LIST_HEAD(&qgroup->dirty);

	rb_link_node(&qgroup->node, parent, p);
	rb_insert_color(&qgroup->node, &fs_info->qgroup_tree);

	return qgroup;
}

static void __del_qgroup_rb(struct btrfs_qgroup *qgroup)
{
	struct btrfs_qgroup_list *list;

	list_del(&qgroup->dirty);
	while (!list_empty(&qgroup->groups)) {
		list = list_first_entry(&qgroup->groups,
					struct btrfs_qgroup_list, next_group);
		list_del(&list->next_group);
		list_del(&list->next_member);
		kfree(list);
	}

	while (!list_empty(&qgroup->members)) {
		list = list_first_entry(&qgroup->members,
					struct btrfs_qgroup_list, next_member);
		list_del(&list->next_group);
		list_del(&list->next_member);
		kfree(list);
	}
	kfree(qgroup);
}

/* must be called with qgroup_lock held */
static int del_qgroup_rb(struct btrfs_fs_info *fs_info, u64 qgroupid)
{
	struct btrfs_qgroup *qgroup = find_qgroup_rb(fs_info, qgroupid);

	if (!qgroup)
		return -ENOENT;

	rb_erase(&qgroup->node, &fs_info->qgroup_tree);
	__del_qgroup_rb(qgroup);
	return 0;
}

/* must be called with qgroup_lock held */
static int add_relation_rb(struct btrfs_fs_info *fs_info,
			   u64 memberid, u64 parentid)
{
	struct btrfs_qgroup *member;
	struct btrfs_qgroup *parent;
	struct btrfs_qgroup_list *list;

	member = find_qgroup_rb(fs_info, memberid);
	parent = find_qgroup_rb(fs_info, parentid);
	if (!member || !parent)
		return -ENOENT;

	list = kzalloc(sizeof(*list), GFP_ATOMIC);
	if (!list)
		return -ENOMEM;

	list->group = parent;
	list->member = member;
	list_add_tail(&list->next_group, &member->groups);
	list_add_tail(&list->next_member, &parent->members);

	return 0;
}

/* must be called with qgroup_lock held */
static int del_relation_rb(struct btrfs_fs_info *fs_info,
			   u64 memberid, u64 parentid)
{
	struct btrfs_qgroup *member;
	struct btrfs_qgroup *parent;
	struct btrfs_qgroup_list *list;

	member = find_qgroup_rb(fs_info, memberid);
	parent = find_qgroup_rb(fs_info, parentid);
	if (!member || !parent)
		return -ENOENT;

	list_for_each_entry(list, &member->groups, next_group) {
		if (list->group == parent) {
			list_del(&list->next_group);
			list_del(&list->next_member);
			kfree(list);
			return 0;
		}
	}
	return -ENOENT;
}

#ifdef CONFIG_BTRFS_FS_RUN_SANITY_TESTS
int btrfs_verify_qgroup_counts(struct btrfs_fs_info *fs_info, u64 qgroupid,
			       u64 rfer, u64 excl)
{
	struct btrfs_qgroup *qgroup;

	qgroup = find_qgroup_rb(fs_info, qgroupid);
	if (!qgroup)
		return -EINVAL;
	if (qgroup->rfer != rfer || qgroup->excl != excl)
		return -EINVAL;
	return 0;
}
#endif

#ifdef MY_ABC_HERE
static void update_syno_quota_rescan_progress(struct btrfs_root *quota_root,
        struct syno_quota_rescan_ctx *ctx, u64 subvol_id,
        enum syno_quota_rescan_progress_update_type type);
#endif /* MY_ABC_HERE */

/*
 * The full config is read in one go, only called from open_ctree()
 * It doesn't use any locking, as at this point we're still single-threaded
 */
int btrfs_read_qgroup_config(struct btrfs_fs_info *fs_info)
{
	struct btrfs_key key;
	struct btrfs_key found_key;
	struct btrfs_root *quota_root = fs_info->quota_root;
	struct btrfs_path *path = NULL;
	struct extent_buffer *l;
	int slot;
	int ret = 0;
	u64 flags = 0;
	u64 rescan_progress = 0;
#ifdef MY_ABC_HERE
	u64 subvol_id;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled &&
			!fs_info->syno_quota_v2_enabled)
		return 0;
#else
	if (!fs_info->quota_enabled)
		return 0;
#endif /* MY_ABC_HERE */

	fs_info->qgroup_ulist = ulist_alloc(GFP_NOFS);
	if (!fs_info->qgroup_ulist) {
		ret = -ENOMEM;
		goto out;
	}

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	/* default this to quota off, in case no status key is found */
	fs_info->qgroup_flags = 0;

	/*
	 * pass 1: read status, all qgroup infos and limits
	 */
	key.objectid = 0;
	key.type = 0;
	key.offset = 0;
	ret = btrfs_search_slot_for_read(quota_root, &key, path, 1, 1);
	if (ret)
		goto out;

	while (1) {
		struct btrfs_qgroup *qgroup;

		slot = path->slots[0];
		l = path->nodes[0];
		btrfs_item_key_to_cpu(l, &found_key, slot);

		if (found_key.type == BTRFS_QGROUP_STATUS_KEY) {
			struct btrfs_qgroup_status_item *ptr;

			ptr = btrfs_item_ptr(l, slot,
					     struct btrfs_qgroup_status_item);

#ifdef MY_ABC_HERE
			if ((fs_info->syno_quota_v1_enabled &&
					btrfs_qgroup_status_version(l, ptr) !=
					BTRFS_QGROUP_STATUS_VERSION) ||
				(fs_info->syno_quota_v2_enabled &&
					btrfs_qgroup_status_version(l, ptr) !=
					BTRFS_QGROUP_V2_STATUS_VERSION)) {
#else
			if (btrfs_qgroup_status_version(l, ptr) !=
			    BTRFS_QGROUP_STATUS_VERSION) {
#endif /* MY_ABC_HERE */
				btrfs_err(fs_info,
				 "old qgroup version, quota disabled");
				goto out;
			}
			if (btrfs_qgroup_status_generation(l, ptr) !=
			    fs_info->generation) {
				flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
				btrfs_err(fs_info,
					"qgroup generation mismatch, "
					"marked as inconsistent");
			}
			fs_info->qgroup_flags = btrfs_qgroup_status_flags(l,
									  ptr);
			rescan_progress = btrfs_qgroup_status_rescan(l, ptr);
			goto next1;
		}

		if (found_key.type != BTRFS_QGROUP_INFO_KEY &&
		    found_key.type != BTRFS_QGROUP_LIMIT_KEY)
			goto next1;

		qgroup = find_qgroup_rb(fs_info, found_key.offset);
		if ((qgroup && found_key.type == BTRFS_QGROUP_INFO_KEY) ||
		    (!qgroup && found_key.type == BTRFS_QGROUP_LIMIT_KEY)) {
			btrfs_err(fs_info, "inconsistent qgroup config");
			flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
		}
		if (!qgroup) {
			qgroup = add_qgroup_rb(fs_info, found_key.offset);
			if (IS_ERR(qgroup)) {
				ret = PTR_ERR(qgroup);
				goto out;
			}
		}
		switch (found_key.type) {
		case BTRFS_QGROUP_INFO_KEY: {
			struct btrfs_qgroup_info_item *ptr;

			ptr = btrfs_item_ptr(l, slot,
					     struct btrfs_qgroup_info_item);
			qgroup->rfer = btrfs_qgroup_info_rfer(l, ptr);
			qgroup->rfer_cmpr = btrfs_qgroup_info_rfer_cmpr(l, ptr);
			qgroup->excl = btrfs_qgroup_info_excl(l, ptr);
			qgroup->excl_cmpr = btrfs_qgroup_info_excl_cmpr(l, ptr);
			/* generation currently unused */
			break;
		}
		case BTRFS_QGROUP_LIMIT_KEY: {
			struct btrfs_qgroup_limit_item *ptr;

			ptr = btrfs_item_ptr(l, slot,
					     struct btrfs_qgroup_limit_item);
			qgroup->lim_flags = btrfs_qgroup_limit_flags(l, ptr);
			qgroup->max_rfer = btrfs_qgroup_limit_max_rfer(l, ptr);
			qgroup->max_excl = btrfs_qgroup_limit_max_excl(l, ptr);
			qgroup->rsv_rfer = btrfs_qgroup_limit_rsv_rfer(l, ptr);
			qgroup->rsv_excl = btrfs_qgroup_limit_rsv_excl(l, ptr);
			break;
		}
		}
next1:
		ret = btrfs_next_item(quota_root, path);
		if (ret < 0)
			goto out;
		if (ret)
			break;
	}
	btrfs_release_path(path);

#ifdef MY_ABC_HERE
	// Setup rescan before pass 2, since we may goto out in pass 2 and miss the rescan setup.
	subvol_id = rescan_progress;
	if (!subvol_id || !fs_info->syno_quota_v2_enabled) // No need to rescan.
		goto pass2;

	fs_info->syno_quota_rescan_ctx = kzalloc(sizeof(struct syno_quota_rescan_ctx), GFP_KERNEL);
	if (!fs_info->syno_quota_rescan_ctx) {
		btrfs_warn(fs_info, "Failed to alloc syno_quota_rescan_ctx");
		ret = -ENOMEM;
		goto out;
	}

	fs_info->syno_quota_rescan_subvol_ulist = ulist_alloc(GFP_KERNEL);
	if (!fs_info->syno_quota_rescan_subvol_ulist) {
		btrfs_warn(fs_info, "Failed to alloc syno_quota_rescan_subvol_ulist");
		ret = -ENOMEM;
		goto out;
	}

	// Read rescan subvol list.
	while (subvol_id) {
		struct btrfs_syno_quota_rescan_item *ptr;
		struct syno_quota_rescan_ctx *ctx = fs_info->syno_quota_rescan_ctx;
		u64 flags;
		u64 ino;

		key.objectid = 0;
		key.type = BTRFS_SYNO_QUOTA_RESCAN_KEY;
		key.offset = subvol_id;

		ret = btrfs_search_slot(NULL, quota_root, &key, path, 0, 0);
		if (ret) {
			btrfs_warn(fs_info,
				"Failed to read syno quota rescan item, root = %llu", subvol_id);
			if (ret > 0)
				ret = -ENOENT;
			break;
		}

		slot = path->slots[0];
		l = path->nodes[0];
		ptr = btrfs_item_ptr(l, slot, struct btrfs_syno_quota_rescan_item);

		flags = btrfs_syno_quota_rescan_flags(l, ptr);
		if (!(flags & (SYNO_QUOTA_RESCAN_QUEUED | SYNO_QUOTA_RESCAN_DOING))) {
			update_syno_quota_rescan_progress(fs_info->quota_root, ctx,
				subvol_id, SYNO_QUOTA_PROGRESS_ADD_FINISHED);
			subvol_id = btrfs_syno_quota_rescan_next_root(l, ptr);
			btrfs_release_path(path);
			continue;
		}

		ino = btrfs_syno_quota_rescan_inode(l, ptr);
		ret = ulist_add(fs_info->syno_quota_rescan_subvol_ulist, subvol_id, ino, GFP_KERNEL);
		if (ret != 1) {
			if (ret == 0) {
				btrfs_warn(fs_info, "Syno quota rescan detect duplicate items, the list is broken.");
				ret = -EEXIST;
			} else {
				btrfs_warn(fs_info, "Syno quota rescan encounter -ENOMEM.");
				ret = -ENOMEM;
			}
			break;
		}
		update_syno_quota_rescan_progress(fs_info->quota_root, ctx,
			subvol_id, SYNO_QUOTA_PROGRESS_ADD_NEW);

		subvol_id = btrfs_syno_quota_rescan_next_root(l, ptr);
		btrfs_release_path(path);
	}
	btrfs_release_path(path);
pass2:
#endif /* MY_ABC_HERE */

	/*
	 * pass 2: read all qgroup relations
	 */
	key.objectid = 0;
	key.type = BTRFS_QGROUP_RELATION_KEY;
	key.offset = 0;
	ret = btrfs_search_slot_for_read(quota_root, &key, path, 1, 0);
	if (ret)
		goto out;
	while (1) {
		slot = path->slots[0];
		l = path->nodes[0];
		btrfs_item_key_to_cpu(l, &found_key, slot);

		if (found_key.type != BTRFS_QGROUP_RELATION_KEY)
			goto next2;

		if (found_key.objectid > found_key.offset) {
			/* parent <- member, not needed to build config */
			/* FIXME should we omit the key completely? */
			goto next2;
		}

		ret = add_relation_rb(fs_info, found_key.objectid,
				      found_key.offset);
		if (ret == -ENOENT) {
			btrfs_warn(fs_info,
				"orphan qgroup relation 0x%llx->0x%llx",
				found_key.objectid, found_key.offset);
			ret = 0;	/* ignore the error */
		}
		if (ret)
			goto out;
next2:
		ret = btrfs_next_item(quota_root, path);
		if (ret < 0)
			goto out;
		if (ret)
			break;
	}
out:
	btrfs_free_path(path);
	fs_info->qgroup_flags |= flags;
#ifdef MY_ABC_HERE
	if (!(fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_ON)) {
		fs_info->syno_quota_v1_enabled = false;
		fs_info->syno_quota_v2_enabled = false;
		fs_info->pending_quota_state = 0;
	} else {
		if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN &&
				ret >= 0 && fs_info->syno_quota_v2_enabled)
			ret = syno_quota_rescan_init(fs_info, rescan_progress, 0);
	}
#else
	if (!(fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_ON)) {
		fs_info->quota_enabled = 0;
		fs_info->pending_quota_state = 0;
	} else if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN &&
		   ret >= 0) {
		ret = qgroup_rescan_init(fs_info, rescan_progress, 0);
	}
#endif /* MY_ABC_HERE */

	if (ret < 0) {
		ulist_free(fs_info->qgroup_ulist);
		fs_info->qgroup_ulist = NULL;
#ifdef MY_ABC_HERE
		ulist_free(fs_info->syno_quota_rescan_subvol_ulist);
		fs_info->syno_quota_rescan_subvol_ulist = NULL;
		kfree(fs_info->syno_quota_rescan_ctx);
		fs_info->syno_quota_rescan_ctx = NULL;
#endif /* MY_ABC_HERE */
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_RESCAN;
	}

	return ret < 0 ? ret : 0;
}

static u64 btrfs_qgroup_subvolid(u64 qgroupid)
{
	return (qgroupid & ((1ULL << BTRFS_QGROUP_LEVEL_SHIFT) - 1));
}

/*
 * Called in close_ctree() when quota is still enabled.  This verifies we don't
 * leak some reserved space.
 *
 * Return false if no reserved space is left.
 * Return true if some reserved space is leaked.
 */
bool btrfs_check_quota_leak(struct btrfs_fs_info *fs_info)
{
	struct rb_node *node;
	bool ret = false;

#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v2_enabled)
#else
	if (!fs_info->quota_enabled)
#endif /* MY_ABC_HERE */
		return ret;
	/*
	 * Since we're unmounting, there is no race and no need to grab qgroup
	 * lock.  And here we don't go post-order to provide a more user
	 * friendly sorted result.
	 */
	for (node = rb_first(&fs_info->qgroup_tree); node; node = rb_next(node)) {
		struct btrfs_qgroup *qgroup;

		qgroup = rb_entry(node, struct btrfs_qgroup, node);
		if (qgroup->reserved) {
			ret = true;
			btrfs_warn(fs_info,
				"qgroup %llu/%llu has unreleased space, rsv %llu",
				   btrfs_qgroup_level(qgroup->qgroupid),
				   btrfs_qgroup_subvolid(qgroup->qgroupid),
				   qgroup->reserved);
		}
	}
	return ret;
}

/*
 * This is called from close_ctree() or open_ctree() or btrfs_quota_disable(),
 * first two are in single-threaded paths.And for the third one, we have set
 * quota_root to be null with qgroup_lock held before, so it is safe to clean
 * up the in-memory structures without qgroup_lock held.
 */
void btrfs_free_qgroup_config(struct btrfs_fs_info *fs_info)
{
	struct rb_node *n;
	struct btrfs_qgroup *qgroup;

	while ((n = rb_first(&fs_info->qgroup_tree))) {
		qgroup = rb_entry(n, struct btrfs_qgroup, node);
		rb_erase(n, &fs_info->qgroup_tree);
		__del_qgroup_rb(qgroup);
	}
	/*
	 * we call btrfs_free_qgroup_config() when umounting
	 * filesystem and disabling quota, so we set qgroup_ulist
	 * to be null here to avoid double free.
	 */
	ulist_free(fs_info->qgroup_ulist);
	fs_info->qgroup_ulist = NULL;
#ifdef MY_ABC_HERE
	ulist_free(fs_info->syno_quota_rescan_subvol_ulist);
	fs_info->syno_quota_rescan_subvol_ulist = NULL;
	kfree(fs_info->syno_quota_rescan_ctx);
	fs_info->syno_quota_rescan_ctx = NULL;
#endif /* MY_ABC_HERE */
}

static int add_qgroup_relation_item(struct btrfs_trans_handle *trans,
				    struct btrfs_root *quota_root,
				    u64 src, u64 dst)
{
	int ret;
	struct btrfs_path *path;
	struct btrfs_key key;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	key.objectid = src;
	key.type = BTRFS_QGROUP_RELATION_KEY;
	key.offset = dst;

	ret = btrfs_insert_empty_item(trans, quota_root, path, &key, 0);

	btrfs_mark_buffer_dirty(path->nodes[0]);

	btrfs_free_path(path);
	return ret;
}

static int del_qgroup_relation_item(struct btrfs_trans_handle *trans,
				    struct btrfs_root *quota_root,
				    u64 src, u64 dst)
{
	int ret;
	struct btrfs_path *path;
	struct btrfs_key key;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	key.objectid = src;
	key.type = BTRFS_QGROUP_RELATION_KEY;
	key.offset = dst;

	ret = btrfs_search_slot(trans, quota_root, &key, path, -1, 1);
	if (ret < 0)
		goto out;

	if (ret > 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = btrfs_del_item(trans, quota_root, path);
out:
	btrfs_free_path(path);
	return ret;
}

static int add_qgroup_item(struct btrfs_trans_handle *trans,
			   struct btrfs_root *quota_root, u64 qgroupid)
{
	int ret;
	struct btrfs_path *path;
	struct btrfs_qgroup_info_item *qgroup_info;
	struct btrfs_qgroup_limit_item *qgroup_limit;
	struct extent_buffer *leaf;
	struct btrfs_key key;

	if (btrfs_test_is_dummy_root(quota_root))
		return 0;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	key.objectid = 0;
	key.type = BTRFS_QGROUP_INFO_KEY;
	key.offset = qgroupid;

	/*
	 * Avoid a transaction abort by catching -EEXIST here. In that
	 * case, we proceed by re-initializing the existing structure
	 * on disk.
	 */

	ret = btrfs_insert_empty_item(trans, quota_root, path, &key,
				      sizeof(*qgroup_info));
	if (ret && ret != -EEXIST)
		goto out;

	leaf = path->nodes[0];
	qgroup_info = btrfs_item_ptr(leaf, path->slots[0],
				 struct btrfs_qgroup_info_item);
	btrfs_set_qgroup_info_generation(leaf, qgroup_info, trans->transid);
	btrfs_set_qgroup_info_rfer(leaf, qgroup_info, 0);
	btrfs_set_qgroup_info_rfer_cmpr(leaf, qgroup_info, 0);
	btrfs_set_qgroup_info_excl(leaf, qgroup_info, 0);
	btrfs_set_qgroup_info_excl_cmpr(leaf, qgroup_info, 0);

	btrfs_mark_buffer_dirty(leaf);

	btrfs_release_path(path);

	key.type = BTRFS_QGROUP_LIMIT_KEY;
	ret = btrfs_insert_empty_item(trans, quota_root, path, &key,
				      sizeof(*qgroup_limit));
	if (ret && ret != -EEXIST)
		goto out;

	leaf = path->nodes[0];
	qgroup_limit = btrfs_item_ptr(leaf, path->slots[0],
				  struct btrfs_qgroup_limit_item);
	btrfs_set_qgroup_limit_flags(leaf, qgroup_limit, 0);
	btrfs_set_qgroup_limit_max_rfer(leaf, qgroup_limit, 0);
	btrfs_set_qgroup_limit_max_excl(leaf, qgroup_limit, 0);
	btrfs_set_qgroup_limit_rsv_rfer(leaf, qgroup_limit, 0);
	btrfs_set_qgroup_limit_rsv_excl(leaf, qgroup_limit, 0);

	btrfs_mark_buffer_dirty(leaf);

	ret = 0;
out:
	btrfs_free_path(path);
	return ret;
}

static int del_qgroup_item(struct btrfs_trans_handle *trans,
			   struct btrfs_root *quota_root, u64 qgroupid)
{
	int ret;
	struct btrfs_path *path;
	struct btrfs_key key;
#ifdef MY_ABC_HERE
	struct btrfs_fs_info *fs_info = quota_root->fs_info;
#endif /* MY_ABC_HERE */

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	key.objectid = 0;
	key.type = BTRFS_QGROUP_INFO_KEY;
	key.offset = qgroupid;
	ret = btrfs_search_slot(trans, quota_root, &key, path, -1, 1);
	if (ret < 0)
		goto out;

	if (ret > 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = btrfs_del_item(trans, quota_root, path);
	if (ret)
		goto out;

	btrfs_release_path(path);

	key.type = BTRFS_QGROUP_LIMIT_KEY;
	ret = btrfs_search_slot(trans, quota_root, &key, path, -1, 1);
	if (ret < 0)
		goto out;

	if (ret > 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = btrfs_del_item(trans, quota_root, path);

#ifdef MY_ABC_HERE
	btrfs_release_path(path);

	// Remove rescan item.
	mutex_lock(&fs_info->qgroup_rescan_lock);
	key.type = BTRFS_SYNO_QUOTA_RESCAN_KEY;
	ret = btrfs_search_slot(trans, quota_root, &key, path, -1, 1);
	if (ret) {
		if (ret > 0)
			ret = 0; // This subvol may be from quota 1.0 or vanilla kernel.
		goto unlock_rescan_lock;
	}

	ret = btrfs_del_item(trans, quota_root, path);

unlock_rescan_lock:
	mutex_unlock(&fs_info->qgroup_rescan_lock);
#endif /* MY_ABC_HERE */

out:
	btrfs_free_path(path);
	return ret;
}

static int update_qgroup_limit_item(struct btrfs_trans_handle *trans,
				    struct btrfs_root *root,
				    struct btrfs_qgroup *qgroup)
{
	struct btrfs_path *path;
	struct btrfs_key key;
	struct extent_buffer *l;
	struct btrfs_qgroup_limit_item *qgroup_limit;
	int ret;
	int slot;

	key.objectid = 0;
	key.type = BTRFS_QGROUP_LIMIT_KEY;
	key.offset = qgroup->qgroupid;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	ret = btrfs_search_slot(trans, root, &key, path, 0, 1);
	if (ret > 0)
		ret = -ENOENT;

	if (ret)
		goto out;

	l = path->nodes[0];
	slot = path->slots[0];
	qgroup_limit = btrfs_item_ptr(l, slot, struct btrfs_qgroup_limit_item);
	btrfs_set_qgroup_limit_flags(l, qgroup_limit, qgroup->lim_flags);
	btrfs_set_qgroup_limit_max_rfer(l, qgroup_limit, qgroup->max_rfer);
	btrfs_set_qgroup_limit_max_excl(l, qgroup_limit, qgroup->max_excl);
	btrfs_set_qgroup_limit_rsv_rfer(l, qgroup_limit, qgroup->rsv_rfer);
	btrfs_set_qgroup_limit_rsv_excl(l, qgroup_limit, qgroup->rsv_excl);

	btrfs_mark_buffer_dirty(l);

out:
	btrfs_free_path(path);
	return ret;
}

static int update_qgroup_info_item(struct btrfs_trans_handle *trans,
				   struct btrfs_root *root,
				   struct btrfs_qgroup *qgroup)
{
	struct btrfs_path *path;
	struct btrfs_key key;
	struct extent_buffer *l;
	struct btrfs_qgroup_info_item *qgroup_info;
	int ret;
	int slot;

	if (btrfs_test_is_dummy_root(root))
		return 0;

	key.objectid = 0;
	key.type = BTRFS_QGROUP_INFO_KEY;
	key.offset = qgroup->qgroupid;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	ret = btrfs_search_slot(trans, root, &key, path, 0, 1);
	if (ret > 0)
		ret = -ENOENT;

	if (ret)
		goto out;

	l = path->nodes[0];
	slot = path->slots[0];
	qgroup_info = btrfs_item_ptr(l, slot, struct btrfs_qgroup_info_item);
	btrfs_set_qgroup_info_generation(l, qgroup_info, trans->transid);
	btrfs_set_qgroup_info_rfer(l, qgroup_info, qgroup->rfer);
	btrfs_set_qgroup_info_rfer_cmpr(l, qgroup_info, qgroup->rfer_cmpr);
	btrfs_set_qgroup_info_excl(l, qgroup_info, qgroup->excl);
	btrfs_set_qgroup_info_excl_cmpr(l, qgroup_info, qgroup->excl_cmpr);

	btrfs_mark_buffer_dirty(l);

out:
	btrfs_free_path(path);
	return ret;
}

static int update_qgroup_status_item(struct btrfs_trans_handle *trans,
				     struct btrfs_fs_info *fs_info,
				    struct btrfs_root *root)
{
	struct btrfs_path *path;
	struct btrfs_key key;
	struct extent_buffer *l;
	struct btrfs_qgroup_status_item *ptr;
	int ret;
	int slot;

	key.objectid = 0;
	key.type = BTRFS_QGROUP_STATUS_KEY;
	key.offset = 0;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	ret = btrfs_search_slot(trans, root, &key, path, 0, 1);
	if (ret > 0)
		ret = -ENOENT;

	if (ret)
		goto out;

	l = path->nodes[0];
	slot = path->slots[0];
	ptr = btrfs_item_ptr(l, slot, struct btrfs_qgroup_status_item);
	btrfs_set_qgroup_status_flags(l, ptr, fs_info->qgroup_flags);
	btrfs_set_qgroup_status_generation(l, ptr, trans->transid);
	btrfs_set_qgroup_status_rescan(l, ptr,
				fs_info->qgroup_rescan_progress.objectid);

	btrfs_mark_buffer_dirty(l);

out:
	btrfs_free_path(path);
	return ret;
}

#ifdef MY_ABC_HERE
int btrfs_read_syno_quota_rescan_item(struct btrfs_root *quota_root, u64 subvol_id,
		struct btrfs_syno_quota_rescan_item *rescan_item)
{
	int ret;
	struct btrfs_path *path;
	struct btrfs_key key;

	if (unlikely(!quota_root))
		return -EINVAL;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	key.objectid = 0;
	key.type = BTRFS_SYNO_QUOTA_RESCAN_KEY;
	key.offset = subvol_id;

	ret = btrfs_search_slot(NULL, quota_root, &key, path, 0, 0);
	if (ret) {
		if (ret > 0)
			ret = -ENOENT;
		goto out;
	}

	read_extent_buffer(path->nodes[0], rescan_item,
		btrfs_item_ptr_offset(path->nodes[0], path->slots[0]), sizeof(*rescan_item));

	ret = 0;
out:
	btrfs_free_path(path);
	return ret;
}

// Will read tree size from btree rescan item, so rescan item must be updated before us.
static void update_syno_quota_rescan_progress(struct btrfs_root *quota_root,
	struct syno_quota_rescan_ctx *ctx, u64 subvol_id,
	enum syno_quota_rescan_progress_update_type type)
{
	struct btrfs_syno_quota_rescan_item rescan_item;
	int ret;

	ret = btrfs_read_syno_quota_rescan_item(quota_root, subvol_id, &rescan_item);
	if (ret) {
		btrfs_warn(quota_root->fs_info, "Failed to read syno quota rescan item, id = %llu, ret = %d",
			subvol_id, ret);
		return;
	}

	switch (type) {
	case SYNO_QUOTA_PROGRESS_REMOVE_SCANNING: // Same as SYNO_QUOTA_PROGRESS_FINISH_ONE.
		ctx->subvol_id = 0;
		ctx->total_finished_size += rescan_item.tree_size;
		memset(ctx->current_path, 0, sizeof(ctx->current_path));
		break;
	case SYNO_QUOTA_PROGRESS_REMOVE_QUEUED:
		if (ctx->total_size >= rescan_item.tree_size)
			ctx->total_size -= rescan_item.tree_size;
		else
			WARN_ON(1);
		break;
	case SYNO_QUOTA_PROGRESS_REMOVE_FINISHED:
		break;
	case SYNO_QUOTA_PROGRESS_ADD_NEW:
		ctx->total_size += rescan_item.tree_size;
		break;
	case SYNO_QUOTA_PROGRESS_ADD_FINISHED:
		ctx->total_size += rescan_item.tree_size;
		ctx->total_finished_size += rescan_item.tree_size;
		break;
	case SYNO_QUOTA_PROGRESS_FINISH_ONE: // Same as SYNO_QUOTA_PROGRESS_REMOVE_SCANNING.
		ctx->subvol_id = 0;
		ctx->total_finished_size += rescan_item.tree_size;
		memset(ctx->current_path, 0, sizeof(ctx->current_path));
		break;
	case SYNO_QUOTA_PROGRESS_FINISH_ALL:
		WARN_ON(ctx->total_finished_size + rescan_item.tree_size != ctx->total_size);

		// Reset progress.
		memset(ctx, 0, sizeof(*ctx));
		break;
	default:
		WARN_ON(1);
	}
}

// Cannot used on new empty rescan item.
static void update_syno_quota_rescan_flags(struct extent_buffer *leaf,
			struct btrfs_syno_quota_rescan_item *rescan_item, u64 flags)
{
	u64 orig_flags = btrfs_syno_quota_rescan_flags(leaf, rescan_item);

	switch (flags) {
	case SYNO_QUOTA_RESCAN_DONE: {
		// If error was found after we ran the scan, we should trigger another new scan to fix.
		orig_flags &= (SYNO_QUOTA_RESCAN_ERR | SYNO_QUOTA_RESCAN_NEED);
		btrfs_set_syno_quota_rescan_flags(leaf, rescan_item,
							orig_flags | SYNO_QUOTA_RESCAN_DONE);
		break;
	}
	case (SYNO_QUOTA_RESCAN_DONE | SYNO_QUOTA_RESCAN_ERR): {
		orig_flags &= SYNO_QUOTA_RESCAN_NEED;
		btrfs_set_syno_quota_rescan_flags(leaf, rescan_item,
			orig_flags | SYNO_QUOTA_RESCAN_DONE | SYNO_QUOTA_RESCAN_ERR);
		break;
	}
	case SYNO_QUOTA_RESCAN_QUEUED: {
		if (orig_flags & SYNO_QUOTA_RESCAN_DOING) {
			WARN_ON(1);
			break;
		}
		// Start a new scan, clear all errors.
		btrfs_set_syno_quota_rescan_flags(leaf, rescan_item, SYNO_QUOTA_RESCAN_QUEUED);
		break;
	}
	case SYNO_QUOTA_RESCAN_DOING: {
		if (orig_flags & SYNO_QUOTA_RESCAN_DONE) {
			WARN_ON(1);
			break;
		}
		orig_flags &= ~SYNO_QUOTA_RESCAN_QUEUED;
		btrfs_set_syno_quota_rescan_flags(leaf, rescan_item,
							orig_flags | SYNO_QUOTA_RESCAN_DOING);
		break;
	}
	case SYNO_QUOTA_RESCAN_ERR: {
		if (orig_flags & SYNO_QUOTA_RESCAN_DONE) {
			WARN_ON(1);
			break;
		}
		btrfs_set_syno_quota_rescan_flags(leaf, rescan_item,
							orig_flags | SYNO_QUOTA_RESCAN_ERR);
		break;
	}
	case SYNO_QUOTA_RESCAN_NEED: {
		btrfs_set_syno_quota_rescan_flags(leaf, rescan_item,
							orig_flags | SYNO_QUOTA_RESCAN_NEED);
		break;
	}
	default:
		WARN_ON(1);
	}
}

int btrfs_add_update_syno_quota_rescan_item(struct btrfs_trans_handle *trans,
		struct btrfs_root *quota_root, u64 subvol_id,
		struct syno_quota_rescan_item_updater *updater)
{
	int ret;
	struct btrfs_path *path;
	struct btrfs_syno_quota_rescan_item *rescan_item;
	struct extent_buffer *leaf;
	struct btrfs_key key;

	if (unlikely(!quota_root))
		return -EINVAL;

	if (!quota_root->fs_info->syno_quota_v2_enabled && !updater->enable)
		return -EINVAL;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	key.objectid = 0;
	key.type = BTRFS_SYNO_QUOTA_RESCAN_KEY;
	key.offset = subvol_id;

	ret = btrfs_insert_empty_item(trans, quota_root, path, &key,
				      sizeof(*rescan_item));
	if (ret && ret != -EEXIST)
		goto out;

	// Insert a new item, SYNO_QUOTA_RESCAN_ITEM_SKIP is not allowed.
	if (ret != -EEXIST && syno_quota_rescan_item_check(updater)) {
		WARN_ON(1);
		ret = -EINVAL;
		goto out;
	}

	leaf = path->nodes[0];
	rescan_item = btrfs_item_ptr(leaf, path->slots[0],
				 struct btrfs_syno_quota_rescan_item);

	if (updater->flags != SYNO_QUOTA_RESCAN_ITEM_SKIP) {
		if (ret != -EEXIST)
			btrfs_set_syno_quota_rescan_flags(leaf, rescan_item, updater->flags);
		else
			update_syno_quota_rescan_flags(leaf, rescan_item, updater->flags);
	}
	if (updater->version != SYNO_QUOTA_RESCAN_ITEM_SKIP)
		btrfs_set_syno_quota_rescan_version(leaf, rescan_item, updater->version);
	btrfs_set_syno_quota_rescan_generation(leaf, rescan_item, trans->transid);
	if (updater->rescan_inode != SYNO_QUOTA_RESCAN_ITEM_SKIP)
		btrfs_set_syno_quota_rescan_inode(leaf, rescan_item, updater->rescan_inode);
	if (updater->end_inode != SYNO_QUOTA_RESCAN_ITEM_SKIP)
		btrfs_set_syno_quota_rescan_end_inode(leaf, rescan_item, updater->end_inode);
	if (updater->tree_size != SYNO_QUOTA_RESCAN_ITEM_SKIP)
		btrfs_set_syno_quota_rescan_tree_size(leaf, rescan_item, updater->tree_size);
	if (updater->next_root != SYNO_QUOTA_RESCAN_ITEM_SKIP)
		btrfs_set_syno_quota_rescan_next_root(leaf, rescan_item, updater->next_root);

	btrfs_mark_buffer_dirty(leaf);
	ret = 0;
out:
	btrfs_free_path(path);
	return ret;
}

/*
 * Detach all rescan items in fs_info->qgroup_rescan_progress.objectid list, so we don't
 * account these subvols into rescan progress in btrfs_read_qgroup_config() next time.
 * Used when we are done all the rescan work.
 */
static void remove_syno_quota_rescan_list(struct btrfs_trans_handle *trans, struct btrfs_root *quota_root)
{
	struct btrfs_fs_info *fs_info = quota_root->fs_info;
	struct btrfs_path *path = NULL;
	struct btrfs_key key;
	struct extent_buffer *l;
	struct btrfs_syno_quota_rescan_item *ptr;
	u64 subvol_id = fs_info->qgroup_rescan_progress.objectid;
	int slot;
	int ret = 0;

	path = btrfs_alloc_path();
	if (!path) {
		btrfs_warn(fs_info,
			"Failed to alloc path in remove_syno_quota_rescan_items()");
		return;
	}

	while (subvol_id) {
		key.objectid = 0;
		key.type = BTRFS_SYNO_QUOTA_RESCAN_KEY;
		key.offset = subvol_id;

		ret = btrfs_search_slot(trans, quota_root, &key, path, 0, 1);
		if (ret) {
			btrfs_warn(fs_info,
				"Failed to read syno quota rescan item, root = %llu", subvol_id);
			WARN_ON(ret > 0);
			break;
		}

		slot = path->slots[0];
		l = path->nodes[0];
		ptr = btrfs_item_ptr(l, slot, struct btrfs_syno_quota_rescan_item);

		subvol_id = btrfs_syno_quota_rescan_next_root(l, ptr);
		btrfs_set_syno_quota_rescan_next_root(l, ptr, 0);

		btrfs_release_path(path);
	}

	fs_info->qgroup_rescan_progress.objectid = 0;

	btrfs_free_path(path);
}
#endif /* MY_ABC_HERE */

/*
 * called with qgroup_lock held
 */
static int btrfs_clean_quota_tree(struct btrfs_trans_handle *trans,
				  struct btrfs_root *root)
{
	struct btrfs_path *path;
	struct btrfs_key key;
	struct extent_buffer *leaf = NULL;
	int ret;
	int nr = 0;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	path->leave_spinning = 1;

	key.objectid = 0;
	key.offset = 0;
	key.type = 0;

	while (1) {
		ret = btrfs_search_slot(trans, root, &key, path, -1, 1);
		if (ret < 0)
			goto out;
		leaf = path->nodes[0];
		nr = btrfs_header_nritems(leaf);
		if (!nr)
			break;
		/*
		 * delete the leaf one by one
		 * since the whole tree is going
		 * to be deleted.
		 */
		path->slots[0] = 0;
		ret = btrfs_del_items(trans, root, path, 0, nr);
		if (ret)
			goto out;

		btrfs_release_path(path);
	}
	ret = 0;
out:
#ifdef MY_ABC_HERE
/*
 * We already clear fs_info->pending_quota_state in btrfs_quota_disable().
 * Remove this so btrfs_quota_remove_v1() can reuse btrfs_clean_quota_tree().
 */
#else
	root->fs_info->pending_quota_state = 0;
#endif /* MY_ABC_HERE */
	btrfs_free_path(path);
	return ret;
}

#ifdef MY_ABC_HERE
int btrfs_quota_enable(struct btrfs_fs_info *fs_info, u64 cmd)
#else
int btrfs_quota_enable(struct btrfs_fs_info *fs_info)
#endif /* MY_ABC_HERE */
{
	struct btrfs_root *quota_root;
	struct btrfs_root *tree_root = fs_info->tree_root;
	struct btrfs_path *path = NULL;
	struct btrfs_qgroup_status_item *ptr;
	struct extent_buffer *leaf;
	struct btrfs_key key;
	struct btrfs_key found_key;
	struct btrfs_qgroup *qgroup = NULL;
	struct btrfs_trans_handle *trans = NULL;
	struct ulist *ulist = NULL;
	int ret = 0;
	int slot;

#ifdef MY_ABC_HERE
	// Default using v2 quota.
	if (cmd == BTRFS_QUOTA_CTL_ENABLE)
		cmd = BTRFS_QUOTA_V2_CTL_ENABLE;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if (btrfs_test_opt(tree_root, NO_QUOTA_TREE)) {
		btrfs_info(fs_info, "Can't enable quota with mount_opt no_quota_tree");
		return -EINVAL;
	}
#endif /* MY_ABC_HERE */

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (fs_info->quota_root) {
#ifdef MY_ABC_HERE
#else
		fs_info->pending_quota_state = PENDING_QUOTA_STATE_V1;
#endif /* MY_ABC_HERE */
		goto out;
	}

	ulist = ulist_alloc(GFP_NOFS);
	if (!ulist) {
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * Unlock qgroup_ioctl_lock before starting the transaction. This is to
	 * avoid lock acquisition inversion problems (reported by lockdep) between
	 * qgroup_ioctl_lock and the vfs freeze semaphores, acquired when we
	 * start a transaction.
	 * After we started the transaction lock qgroup_ioctl_lock again and
	 * check if someone else created the quota root in the meanwhile. If so,
	 * just return success and release the transaction handle.
	 *
	 * Also we don't need to worry about someone else calling
	 * btrfs_sysfs_add_qgroups() after we unlock and getting an error because
	 * that function returns 0 (success) when the sysfs entries already exist.
	 */
	mutex_unlock(&fs_info->qgroup_ioctl_lock);

	/*
	 * 1 for quota root item
	 * 1 for BTRFS_QGROUP_STATUS item
	 *
	 * Yet we also need 2*n items for a QGROUP_INFO/QGROUP_LIMIT items
	 * per subvolume. However those are not currently reserved since it
	 * would be a lot of overkill.
	 */
	trans = btrfs_start_transaction(tree_root, 2);

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (IS_ERR(trans)) {
		ret = PTR_ERR(trans);
		trans = NULL;
		goto out;
	}

	if (fs_info->quota_root)
		goto out;

	fs_info->qgroup_ulist = ulist;
	ulist = NULL;

	/*
	 * initially create the quota tree
	 */
#ifdef MY_ABC_HERE
	if (cmd == BTRFS_QUOTA_V2_CTL_ENABLE)
		quota_root = btrfs_create_tree(trans, fs_info,
				       BTRFS_SYNO_QUOTA_V2_TREE_OBJECTID);
	else
#endif /* MY_ABC_HERE */
	quota_root = btrfs_create_tree(trans, fs_info,
				       BTRFS_QUOTA_TREE_OBJECTID);
	if (IS_ERR(quota_root)) {
		ret =  PTR_ERR(quota_root);
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out;
	}

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out_free_root;
	}

	key.objectid = 0;
	key.type = BTRFS_QGROUP_STATUS_KEY;
	key.offset = 0;

	ret = btrfs_insert_empty_item(trans, quota_root, path, &key,
				      sizeof(*ptr));
	if (ret) {
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out_free_path;
	}

	leaf = path->nodes[0];
	ptr = btrfs_item_ptr(leaf, path->slots[0],
				 struct btrfs_qgroup_status_item);
	btrfs_set_qgroup_status_generation(leaf, ptr, trans->transid);
#ifdef MY_ABC_HERE
	if (cmd == BTRFS_QUOTA_V2_CTL_ENABLE)
		btrfs_set_qgroup_status_version(leaf, ptr, BTRFS_QGROUP_V2_STATUS_VERSION);
	else
#endif /* MY_ABC_HERE */
	btrfs_set_qgroup_status_version(leaf, ptr, BTRFS_QGROUP_STATUS_VERSION);
	fs_info->qgroup_flags = BTRFS_QGROUP_STATUS_FLAG_ON |
				BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
	btrfs_set_qgroup_status_flags(leaf, ptr, fs_info->qgroup_flags);
	btrfs_set_qgroup_status_rescan(leaf, ptr, 0);

	btrfs_mark_buffer_dirty(leaf);

	key.objectid = 0;
	key.type = BTRFS_ROOT_REF_KEY;
	key.offset = 0;

	btrfs_release_path(path);
	ret = btrfs_search_slot_for_read(tree_root, &key, path, 1, 0);
	if (ret > 0)
		goto out_add_root;
	if (ret < 0) {
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out_free_path;
	}

	while (1) {
		slot = path->slots[0];
		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &found_key, slot);

		if (found_key.type == BTRFS_ROOT_REF_KEY) {
			ret = add_qgroup_item(trans, quota_root,
					      found_key.offset);
			if (ret) {
				btrfs_abort_transaction(trans, tree_root, ret);
				goto out_free_path;
			}

			qgroup = add_qgroup_rb(fs_info, found_key.offset);
			if (IS_ERR(qgroup)) {
				ret = PTR_ERR(qgroup);
				btrfs_abort_transaction(trans, tree_root, ret);
				goto out_free_path;
			}
#ifdef MY_ABC_HERE
			if (cmd == BTRFS_QUOTA_V2_CTL_ENABLE) {
				struct syno_quota_rescan_item_updater updater;
				struct btrfs_root *fs_root;

				syno_quota_rescan_item_init(&updater);
				updater.flags = SYNO_QUOTA_RESCAN_DONE | SYNO_QUOTA_RESCAN_NEED;
				updater.version = 0; // So root->invalid_quota will be set.
				updater.rescan_inode = 0;
				updater.end_inode = (u64)-1;
				updater.tree_size = 0;
				updater.next_root = 0;
				updater.enable = true;

				ret = btrfs_add_update_syno_quota_rescan_item(trans, quota_root,
					found_key.offset, &updater);
				if (ret)
					btrfs_warn(fs_info,
						"Failed to create syno quota rescan item for root %llu, ret = %d",
						found_key.offset, ret);

				/*
				 * If fs root is not in memory, we set invalid_quota in
				 * btrfs_read_syno_quota_for_root().
				 */
				fs_root = btrfs_lookup_fs_root(fs_info, found_key.offset);
				if (fs_root) {
					fs_root->invalid_quota = true;
					fs_root->rescan_inode = 0;
				}

				ret = 0;
			}
#endif /* MY_ABC_HERE */
		}
		ret = btrfs_next_item(tree_root, path);
		if (ret < 0) {
			btrfs_abort_transaction(trans, tree_root, ret);
			goto out_free_path;
		}
		if (ret)
			break;
	}

out_add_root:
	btrfs_release_path(path);
	ret = add_qgroup_item(trans, quota_root, BTRFS_FS_TREE_OBJECTID);
	if (ret) {
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out_free_path;
	}

	qgroup = add_qgroup_rb(fs_info, BTRFS_FS_TREE_OBJECTID);
	if (IS_ERR(qgroup)) {
		ret = PTR_ERR(qgroup);
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out_free_path;
	}

#ifdef MY_ABC_HERE
	if (!ret && cmd == BTRFS_QUOTA_V2_CTL_ENABLE) {
		struct syno_quota_rescan_item_updater updater;

		syno_quota_rescan_item_init(&updater);
		updater.flags = SYNO_QUOTA_RESCAN_DONE;
		updater.version = BTRFS_QGROUP_V2_STATUS_VERSION;
		updater.rescan_inode = (u64)-1;
		updater.end_inode = (u64)-1;
		updater.tree_size = 0;
		updater.next_root = 0;
		updater.enable = true;

		ret = btrfs_add_update_syno_quota_rescan_item(trans, quota_root,
			BTRFS_FS_TREE_OBJECTID, &updater);
		if (ret)
			btrfs_warn(fs_info,
				"Failed to create syno quota rescan item for root 5, ret = %d", ret);
		ret = 0; // No need to abort transaction, it is not that critical.
	}
#endif /* MY_ABC_HERE */

	spin_lock(&fs_info->qgroup_lock);
	fs_info->quota_root = quota_root;
#ifdef MY_ABC_HERE
	if (cmd == BTRFS_QUOTA_V2_CTL_ENABLE)
		fs_info->pending_quota_state = PENDING_QUOTA_STATE_V2;
	else
		fs_info->pending_quota_state = PENDING_QUOTA_STATE_V1;
#else
	fs_info->pending_quota_state = 1;
#endif /* MY_ABC_HERE */
	spin_unlock(&fs_info->qgroup_lock);

	ret = btrfs_commit_transaction(trans, tree_root);
	trans = NULL;
	if (ret)
		goto out_free_path;

out_free_path:
	btrfs_free_path(path);
out_free_root:
	if (ret) {
		free_extent_buffer(quota_root->node);
		free_extent_buffer(quota_root->commit_root);
		kfree(quota_root);
	}
out:
	if (ret) {
		ulist_free(fs_info->qgroup_ulist);
		fs_info->qgroup_ulist = NULL;
	}
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	if (ret && trans)
		btrfs_end_transaction(trans, tree_root);
	else if (trans)
		ret = btrfs_end_transaction(trans, tree_root);
	ulist_free(ulist);
	return ret;
}

int btrfs_quota_disable(struct btrfs_fs_info *fs_info)
{
	struct btrfs_root *tree_root = fs_info->tree_root;
	struct btrfs_root *quota_root;
	struct btrfs_trans_handle *trans = NULL;
	int ret = 0;

#ifdef MY_ABC_HERE
	/*
	 * Protected by fs_info->subvol_sem, so user quota will not do enable
	 * before we finish qgroup disable.
	 */
	if (fs_info->syno_usrquota_v1_enabled || fs_info->syno_usrquota_v2_enabled) {
		btrfs_warn(fs_info,
			"Should disable user quota before disable qgroup.");
		return -EINVAL;
	}
#endif /* MY_ABC_HERE */

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (!fs_info->quota_root)
		goto out;
	mutex_unlock(&fs_info->qgroup_ioctl_lock);

	/*
	 * 1 For the root item
	 *
	 * We should also reserve enough items for the quota tree deletion in
	 * btrfs_clean_quota_tree but this is not done.
	 *
	 * Also, we must always start a transaction without holding the mutex
	 * qgroup_ioctl_lock, see btrfs_quota_enable().
	 */
	trans = btrfs_start_transaction(fs_info->tree_root, 1);

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (IS_ERR(trans)) {
		ret = PTR_ERR(trans);
		trans = NULL;
		goto out;
	}

	if (!fs_info->quota_root)
		goto out;

#ifdef MY_ABC_HERE
	fs_info->need_clear_reserve = true;
	smp_wmb();
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	fs_info->syno_quota_v1_enabled = false;
	fs_info->syno_quota_v2_enabled = false;
#else
	fs_info->quota_enabled = 0;
#endif /* MY_ABC_HERE */
	fs_info->pending_quota_state = 0;
	btrfs_qgroup_wait_for_completion(fs_info, false);
	spin_lock(&fs_info->qgroup_lock);
	quota_root = fs_info->quota_root;
	fs_info->quota_root = NULL;
	fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_ON;
	spin_unlock(&fs_info->qgroup_lock);

	btrfs_free_qgroup_config(fs_info);

	ret = btrfs_clean_quota_tree(trans, quota_root);
	if (ret) {
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out;
	}

	ret = btrfs_del_root(trans, tree_root, &quota_root->root_key);
	if (ret) {
		btrfs_abort_transaction(trans, tree_root, ret);
		goto out;
	}

	list_del(&quota_root->dirty_list);

	btrfs_tree_lock(quota_root->node);
	clean_tree_block(trans, tree_root->fs_info, quota_root->node);
	btrfs_tree_unlock(quota_root->node);
	btrfs_free_tree_block(trans, quota_root, quota_root->node, 0, 1);

	free_extent_buffer(quota_root->node);
	free_extent_buffer(quota_root->commit_root);
	kfree(quota_root);

#ifdef MY_ABC_HERE
	btrfs_start_delalloc_roots(fs_info, 1, -1);
	btrfs_wait_ordered_roots(fs_info, -1, 0, (u64)-1);
	fs_info->need_clear_reserve = false;
#endif /* MY_ABC_HERE */

out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	if (ret && trans)
		btrfs_end_transaction(trans, tree_root);
	else if (trans)
		ret = btrfs_end_transaction(trans, tree_root);

	return ret;
}

#ifdef MY_ABC_HERE
int btrfs_quota_unload(struct btrfs_fs_info *fs_info)
{
	struct btrfs_root *quota_root;
	struct btrfs_trans_handle *trans;

#ifdef MY_ABC_HERE
	/*
	 * Protected by fs_info->subvol_sem, so user quota will not do enable
	 * before we finish qgroup disable.
	 */
	if (fs_info->syno_usrquota_v1_enabled || fs_info->syno_usrquota_v2_enabled) {
		btrfs_warn(fs_info,
			"Should disable user quota before disable qgroup.");
		return -EINVAL;
	}
#endif /* MY_ABC_HERE */

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (!fs_info->quota_root) {
		mutex_unlock(&fs_info->qgroup_ioctl_lock);
		return 0;
	}
#ifdef MY_ABC_HERE
	fs_info->need_clear_reserve = true;
	smp_wmb();
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	fs_info->syno_quota_v1_enabled = false;
	fs_info->syno_quota_v2_enabled = false;
#else
	fs_info->quota_enabled = 0;
#endif /* MY_ABC_HERE */
	fs_info->pending_quota_state = 0;
	btrfs_qgroup_wait_for_completion(fs_info, false);
	spin_lock(&fs_info->qgroup_lock);
	quota_root = fs_info->quota_root;
	fs_info->quota_root = NULL;
	fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_ON;
	spin_unlock(&fs_info->qgroup_lock);

	btrfs_free_qgroup_config(fs_info);

	mutex_unlock(&fs_info->qgroup_ioctl_lock);

#ifdef MY_ABC_HERE
	btrfs_start_delalloc_roots(fs_info, 1, -1);
	btrfs_wait_ordered_roots(fs_info, -1, 0, (u64)-1);
	fs_info->need_clear_reserve = false;
#endif /* MY_ABC_HERE */

	trans = btrfs_join_transaction(fs_info->tree_root);
	if (IS_ERR(trans))
		return PTR_ERR(trans);
	btrfs_commit_transaction(trans, fs_info->tree_root);

	list_del(&quota_root->dirty_list);
	free_extent_buffer(quota_root->node);
	free_extent_buffer(quota_root->commit_root);
	kfree(quota_root);

	return 0;
}

int btrfs_quota_remove_v1(struct btrfs_fs_info *fs_info)
{
	struct btrfs_root *tree_root = fs_info->tree_root;
	struct btrfs_root *root;
	struct btrfs_trans_handle *trans = NULL;
	struct btrfs_path *path = NULL;
	struct btrfs_key location;
	int ret = 0;
	int nr;

	// Ensure quota v1 tree is not in use.
	if (fs_info->syno_quota_v1_enabled ||
			(fs_info->quota_root &&
			fs_info->quota_root->root_key.objectid == BTRFS_QUOTA_TREE_OBJECTID))
		return -EBUSY;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	// Read old qgroup root.
	location.objectid = BTRFS_QUOTA_TREE_OBJECTID;
	location.type = BTRFS_ROOT_ITEM_KEY;
	location.offset = 0;

	root = btrfs_read_tree_root(tree_root, &location);
	if (IS_ERR(root)) {
		ret = PTR_ERR(root);
		goto out;
	}
	set_bit(BTRFS_ROOT_TRACK_DIRTY, &root->state);

	location.objectid = 0;
	location.offset = 0;
	location.type = 0;

	while (1) {
		trans = btrfs_start_transaction(tree_root, 1);
		if (IS_ERR(trans)) {
			ret = PTR_ERR(trans);
			trans = NULL;
			goto free_root;
		}

		ret = btrfs_search_slot(trans, root, &location, path, -1, 1);
		if (ret < 0)
			goto free_root;
		nr = btrfs_header_nritems(path->nodes[0]);
		if (!nr)
			break;
		path->slots[0] = 0;
		ret = btrfs_del_items(trans, root, path, 0, nr);
		if (ret)
			goto free_root;

		btrfs_release_path(path);
		btrfs_end_transaction_throttle(trans, tree_root);
		trans = NULL;
		cond_resched();
	}
	btrfs_release_path(path);

	// Remove root item from root tree.
	ret = btrfs_del_root(trans, tree_root, &root->root_key);

free_root:
	btrfs_release_path(path);
	list_del(&root->dirty_list);
	btrfs_tree_lock(root->node);
	clean_tree_block(trans, fs_info, root->node);
	btrfs_tree_unlock(root->node);
	btrfs_free_tree_block(trans, root, root->node, 0, 1);

	free_extent_buffer(root->node);
	free_extent_buffer(root->commit_root);
#ifdef MY_ABC_HERE
	btrfs_free_root_eb_monitor(root);
#endif /* MY_ABC_HERE */
	kfree(root);

	if (trans) {
		if (!ret)
			ret = btrfs_commit_transaction(trans, tree_root);
		else
			btrfs_end_transaction(trans, root);
	}
out:
	btrfs_free_path(path);
	return ret;
}
#endif /* MY_ABC_HERE */

static void qgroup_dirty(struct btrfs_fs_info *fs_info,
			 struct btrfs_qgroup *qgroup)
{
	if (list_empty(&qgroup->dirty))
		list_add(&qgroup->dirty, &fs_info->dirty_qgroups);
}

/*
 * The easy accounting, if we are adding/removing the only ref for an extent
 * then this qgroup and all of the parent qgroups get their reference and
 * exclusive counts adjusted.
 *
 * Caller should hold fs_info->qgroup_lock.
 */
static int __qgroup_excl_accounting(struct btrfs_fs_info *fs_info,
				    struct ulist *tmp, u64 ref_root,
				    u64 num_bytes, int sign)
{
	struct btrfs_qgroup *qgroup;
	struct btrfs_qgroup_list *glist;
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	int ret = 0;

	qgroup = find_qgroup_rb(fs_info, ref_root);
	if (!qgroup)
		goto out;

#ifdef MY_ABC_HERE
	if (unlikely(sign < 0 && qgroup->rfer < num_bytes)) {
		qgroup->rfer = 0;
		qgroup->rfer_cmpr = 0;
	} else {
		qgroup->rfer += sign * num_bytes;
		qgroup->rfer_cmpr += sign * num_bytes;
	}
#else
	qgroup->rfer += sign * num_bytes;
	qgroup->rfer_cmpr += sign * num_bytes;
#endif /* MY_ABC_HERE */

	WARN_ON(sign < 0 && qgroup->excl < num_bytes);
	qgroup->excl += sign * num_bytes;
	qgroup->excl_cmpr += sign * num_bytes;
#ifdef MY_ABC_HERE
	if (sign > 0) {
		if (qgroup->reserved >= num_bytes)
			qgroup->reserved -= num_bytes;
		else
			qgroup->reserved = 0;
	}
#else
	if (sign > 0)
		qgroup->reserved -= num_bytes;
#endif /* MY_ABC_HERE */

	qgroup_dirty(fs_info, qgroup);

	/* Get all of the parent groups that contain this qgroup */
	list_for_each_entry(glist, &qgroup->groups, next_group) {
		ret = ulist_add(tmp, glist->group->qgroupid,
				ptr_to_u64(glist->group), GFP_ATOMIC);
		if (ret < 0)
			goto out;
	}

	/* Iterate all of the parents and adjust their reference counts */
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(tmp, &uiter))) {
		qgroup = u64_to_ptr(unode->aux);
#ifdef MY_ABC_HERE
		if (unlikely(sign < 0 && qgroup->rfer < num_bytes)) {
			qgroup->rfer = 0;
			qgroup->rfer_cmpr = 0;
		} else {
			qgroup->rfer += sign * num_bytes;
			qgroup->rfer_cmpr += sign * num_bytes;
		}
#else
		qgroup->rfer += sign * num_bytes;
		qgroup->rfer_cmpr += sign * num_bytes;
#endif /* MY_ABC_HERE */
		WARN_ON(sign < 0 && qgroup->excl < num_bytes);
		qgroup->excl += sign * num_bytes;
#ifdef MY_ABC_HERE
		if (sign > 0) {
			if (qgroup->reserved >= num_bytes)
				qgroup->reserved -= num_bytes;
			else
				qgroup->reserved = 0;
		}
#else
		if (sign > 0)
			qgroup->reserved -= num_bytes;
#endif /* MY_ABC_HERE */
		qgroup->excl_cmpr += sign * num_bytes;
		qgroup_dirty(fs_info, qgroup);

		/* Add any parents of the parents */
		list_for_each_entry(glist, &qgroup->groups, next_group) {
			ret = ulist_add(tmp, glist->group->qgroupid,
					ptr_to_u64(glist->group), GFP_ATOMIC);
			if (ret < 0)
				goto out;
		}
	}
	ret = 0;
out:
	return ret;
}


/*
 * Quick path for updating qgroup with only excl refs.
 *
 * In that case, just update all parent will be enough.
 * Or we needs to do a full rescan.
 * Caller should also hold fs_info->qgroup_lock.
 *
 * Return 0 for quick update, return >0 for need to full rescan
 * and mark INCONSISTENT flag.
 * Return < 0 for other error.
 */
static int quick_update_accounting(struct btrfs_fs_info *fs_info,
				   struct ulist *tmp, u64 src, u64 dst,
				   int sign)
{
	struct btrfs_qgroup *qgroup;
	int ret = 1;
	int err = 0;

#ifdef MY_ABC_HERE
	if (fs_info->syno_quota_v2_enabled)
		return 0;
#endif /* MY_ABC_HERE */

	qgroup = find_qgroup_rb(fs_info, src);
	if (!qgroup)
		goto out;
	if (qgroup->excl == qgroup->rfer) {
		ret = 0;
		err = __qgroup_excl_accounting(fs_info, tmp, dst,
					       qgroup->excl, sign);
		if (err < 0) {
			ret = err;
			goto out;
		}
	}
out:
	if (ret)
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
	return ret;
}

int btrfs_add_qgroup_relation(struct btrfs_trans_handle *trans,
			      struct btrfs_fs_info *fs_info, u64 src, u64 dst)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *parent;
	struct btrfs_qgroup *member;
	struct btrfs_qgroup_list *list;
	struct ulist *tmp;
	int ret = 0;

	/* Check the level of src and dst first */
	if (btrfs_qgroup_level(src) >= btrfs_qgroup_level(dst))
		return -EINVAL;

	tmp = ulist_alloc(GFP_NOFS);
	if (!tmp)
		return -ENOMEM;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	quota_root = fs_info->quota_root;
	if (!quota_root) {
#ifdef MY_ABC_HERE
		ret = -ESRCH;
#else
		ret = -EINVAL;
#endif /* MY_ABC_HERE */
		goto out;
	}
	member = find_qgroup_rb(fs_info, src);
	parent = find_qgroup_rb(fs_info, dst);
	if (!member || !parent) {
		ret = -EINVAL;
		goto out;
	}

	/* check if such qgroup relation exist firstly */
	list_for_each_entry(list, &member->groups, next_group) {
		if (list->group == parent) {
			ret = -EEXIST;
			goto out;
		}
	}

	ret = add_qgroup_relation_item(trans, quota_root, src, dst);
	if (ret)
		goto out;

	ret = add_qgroup_relation_item(trans, quota_root, dst, src);
	if (ret) {
		del_qgroup_relation_item(trans, quota_root, src, dst);
		goto out;
	}

	spin_lock(&fs_info->qgroup_lock);
	ret = add_relation_rb(quota_root->fs_info, src, dst);
	if (ret < 0) {
		spin_unlock(&fs_info->qgroup_lock);
		goto out;
	}
	ret = quick_update_accounting(fs_info, tmp, src, dst, 1);
	spin_unlock(&fs_info->qgroup_lock);
out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	ulist_free(tmp);
	return ret;
}

int __del_qgroup_relation(struct btrfs_trans_handle *trans,
			      struct btrfs_fs_info *fs_info, u64 src, u64 dst)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *parent;
	struct btrfs_qgroup *member;
	struct btrfs_qgroup_list *list;
	struct ulist *tmp;
	int ret = 0;
	int err;

	tmp = ulist_alloc(GFP_NOFS);
	if (!tmp)
		return -ENOMEM;

	quota_root = fs_info->quota_root;
	if (!quota_root) {
#ifdef MY_ABC_HERE
		ret = -ESRCH;
#else
		ret = -EINVAL;
#endif /* MY_ABC_HERE */
		goto out;
	}

	member = find_qgroup_rb(fs_info, src);
	parent = find_qgroup_rb(fs_info, dst);
	if (!member || !parent) {
		ret = -EINVAL;
		goto out;
	}

	/* check if such qgroup relation exist firstly */
	list_for_each_entry(list, &member->groups, next_group) {
		if (list->group == parent)
			goto exist;
	}
	ret = -ENOENT;
	goto out;
exist:
	ret = del_qgroup_relation_item(trans, quota_root, src, dst);
	err = del_qgroup_relation_item(trans, quota_root, dst, src);
	if (err && !ret)
		ret = err;

	spin_lock(&fs_info->qgroup_lock);
	del_relation_rb(fs_info, src, dst);
	ret = quick_update_accounting(fs_info, tmp, src, dst, -1);
	spin_unlock(&fs_info->qgroup_lock);
out:
	ulist_free(tmp);
	return ret;
}

int btrfs_del_qgroup_relation(struct btrfs_trans_handle *trans,
			      struct btrfs_fs_info *fs_info, u64 src, u64 dst)
{
	int ret = 0;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	ret = __del_qgroup_relation(trans, fs_info, src, dst);
	mutex_unlock(&fs_info->qgroup_ioctl_lock);

	return ret;
}

int btrfs_create_qgroup(struct btrfs_trans_handle *trans,
			struct btrfs_fs_info *fs_info, u64 qgroupid)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *qgroup;
	int ret = 0;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	quota_root = fs_info->quota_root;
	if (!quota_root) {
#ifdef MY_ABC_HERE
		ret = -ESRCH;
#else
		ret = -EINVAL;
#endif /* MY_ABC_HERE */
		goto out;
	}
	qgroup = find_qgroup_rb(fs_info, qgroupid);
	if (qgroup) {
		ret = -EEXIST;
		goto out;
	}

	ret = add_qgroup_item(trans, quota_root, qgroupid);
	if (ret)
		goto out;

	spin_lock(&fs_info->qgroup_lock);
	qgroup = add_qgroup_rb(fs_info, qgroupid);
	spin_unlock(&fs_info->qgroup_lock);

	if (IS_ERR(qgroup))
		ret = PTR_ERR(qgroup);
out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}

#ifdef MY_ABC_HERE
/*
 * struct btrfs_ioctl_qgroup_query_args should be initialized to zero
 */
int btrfs_qgroup_query(struct btrfs_root *root,
			struct btrfs_ioctl_qgroup_query_args *qqa)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_qgroup *qgroup;
	u64 qgroupid = root->root_key.objectid;
	int ret = 0;

#ifdef MY_ABC_HERE
	if (unlikely(root->invalid_quota))
		return -ESRCH;
#endif /* MY_ABC_HERE */

	mutex_lock(&fs_info->qgroup_ioctl_lock);
#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled &&
			!fs_info->syno_quota_v2_enabled) {
		ret = -ESRCH;
		goto unlock;
	}
#else
	if (!fs_info->quota_enabled) {
		ret = -ESRCH;
		goto unlock;
	}
#endif /* MY_ABC_HERE */

	memset(qqa, 0, sizeof(*qqa));
	qgroup = find_qgroup_rb(fs_info, qgroupid);
	if (!qgroup)
		goto unlock;

	qqa->rfer = qgroup->rfer;
	qqa->rfer_cmpr = qgroup->rfer_cmpr;
	qqa->excl = qgroup->excl;
	qqa->excl_cmpr = qgroup->excl_cmpr;

	if (qgroup->lim_flags & BTRFS_QGROUP_LIMIT_MAX_RFER)
		qqa->max_rfer = qgroup->max_rfer;
	if (qgroup->lim_flags & BTRFS_QGROUP_LIMIT_MAX_EXCL)
		qqa->max_excl = qgroup->max_excl;
	if (qgroup->lim_flags & BTRFS_QGROUP_LIMIT_RSV_RFER)
		qqa->rsv_rfer = qgroup->rsv_rfer;
	if (qgroup->lim_flags & BTRFS_QGROUP_LIMIT_RSV_EXCL)
		qqa->rsv_excl = qgroup->rsv_excl;
	qqa->reserved = qgroup->reserved;
unlock:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
// We will remove rescan item later in del_qgroup_item().
void btrfs_remove_queued_syno_rescan(struct btrfs_trans_handle *trans,
					struct btrfs_fs_info *fs_info, u64 subvol_id)
{
	struct syno_quota_rescan_ctx *ctx = fs_info->syno_quota_rescan_ctx;
	struct ulist *ulist = fs_info->syno_quota_rescan_subvol_ulist;
	struct ulist_node *node;

	if (!fs_info->quota_root || !ctx || !ulist)
		return;

	mutex_lock(&fs_info->qgroup_rescan_lock);
	node = ulist_search(ulist, subvol_id);
	if (node) {
		struct ulist_node *prev_node = NULL;
		struct ulist_node *next_node = NULL;
		u64 prev_subvol_id = 0;
		u64 next_subvol_id = 0;
		int ret;

		if (node->list.prev != &ulist->nodes) {
			prev_node = list_entry(node->list.prev, struct ulist_node, list);
			prev_subvol_id = prev_node->val;
		}

		if (node->list.next != &ulist->nodes) {
			next_node = list_entry(node->list.next, struct ulist_node, list);
			next_subvol_id = next_node->val;
		}

		if (prev_subvol_id) {
			struct syno_quota_rescan_item_updater updater;

			syno_quota_rescan_item_init(&updater);
			updater.next_root = (next_subvol_id)? next_subvol_id : 0;
			ret = btrfs_add_update_syno_quota_rescan_item(trans, fs_info->quota_root,
				prev_subvol_id, &updater);
			if (ret)
				btrfs_warn(fs_info,
					"Failed to update syno quota rescan item, id = %llu, ret = %d.",
					prev_subvol_id, ret);
		}

		if (fs_info->qgroup_rescan_progress.objectid == subvol_id)
			fs_info->qgroup_rescan_progress.objectid = next_subvol_id;
		ulist_del(ulist, subvol_id, node->aux);

		// Update progress info.
		if (!fs_info->qgroup_rescan_progress.objectid) { // No next subvol. All rescan are done.
			update_syno_quota_rescan_progress(fs_info->quota_root, ctx,
				subvol_id, SYNO_QUOTA_PROGRESS_FINISH_ALL);
			remove_syno_quota_rescan_list(trans, fs_info->quota_root);
		} else if (ctx->subvol_id == subvol_id) // We are removing the current scanning subvol.
			update_syno_quota_rescan_progress(fs_info->quota_root, ctx,
				subvol_id, SYNO_QUOTA_PROGRESS_FINISH_ONE);
		else // We are removing a queued subvol.
			update_syno_quota_rescan_progress(fs_info->quota_root, ctx,
				subvol_id, SYNO_QUOTA_PROGRESS_REMOVE_QUEUED);
	}
	mutex_unlock(&fs_info->qgroup_rescan_lock);
	return;
}
#endif /* MY_ABC_HERE */

int btrfs_remove_qgroup(struct btrfs_trans_handle *trans,
			struct btrfs_fs_info *fs_info, u64 qgroupid)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *qgroup;
	struct btrfs_qgroup_list *list;
	int ret = 0;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	quota_root = fs_info->quota_root;
	if (!quota_root) {
#ifdef MY_ABC_HERE
		ret = -ESRCH;
#else
		ret = -EINVAL;
#endif /* MY_ABC_HERE */
		goto out;
	}

	qgroup = find_qgroup_rb(fs_info, qgroupid);
	if (!qgroup) {
		ret = -ENOENT;
		goto out;
	} else {
		/* check if there are no children of this qgroup */
		if (!list_empty(&qgroup->members)) {
			ret = -EBUSY;
			goto out;
		}
	}

#ifdef MY_ABC_HERE
	btrfs_remove_queued_syno_rescan(trans, fs_info, qgroupid);
#endif /* MY_ABC_HERE */

	ret = del_qgroup_item(trans, quota_root, qgroupid);

	while (!list_empty(&qgroup->groups)) {
		list = list_first_entry(&qgroup->groups,
					struct btrfs_qgroup_list, next_group);
		ret = __del_qgroup_relation(trans, fs_info,
					   qgroupid,
					   list->group->qgroupid);
		if (ret)
			goto out;
	}

	spin_lock(&fs_info->qgroup_lock);
	del_qgroup_rb(quota_root->fs_info, qgroupid);
	spin_unlock(&fs_info->qgroup_lock);
out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}

int btrfs_limit_qgroup(struct btrfs_trans_handle *trans,
		       struct btrfs_fs_info *fs_info, u64 qgroupid,
		       struct btrfs_qgroup_limit *limit)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *qgroup;
	int ret = 0;
#ifdef MY_ABC_HERE
	struct btrfs_root *root = trans->root;
	bool has_limit = false;
#endif /* MY_ABC_HERE */
	/* Sometimes we would want to clear the limit on this qgroup.
	 * To meet this requirement, we treat the -1 as a special value
	 * which tell kernel to clear the limit on this qgroup.
	 */
#ifdef MY_ABC_HERE
#else
	const u64 CLEAR_VALUE = -1;
#endif /* MY_ABC_HERE */

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	quota_root = fs_info->quota_root;
	if (!quota_root) {
#ifdef MY_ABC_HERE
		ret = -ESRCH;
#else
		ret = -EINVAL;
#endif /* MY_ABC_HERE */
		goto out;
	}

#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled &&
		!fs_info->syno_quota_v2_enabled) {
		ret = -ESRCH;
		goto out;
	}
#endif /* MY_ABC_HERE */

	qgroup = find_qgroup_rb(fs_info, qgroupid);
	if (!qgroup) {
		ret = -ENOENT;
		goto out;
	}

	spin_lock(&fs_info->qgroup_lock);

#ifdef MY_ABC_HERE
	qgroup->lim_flags = limit->flags;
	qgroup->max_rfer = limit->max_rfer;
	qgroup->max_excl = limit->max_excl;
	qgroup->rsv_rfer = limit->rsv_rfer;
	qgroup->rsv_excl = limit->rsv_excl;
#else
	if (limit->flags & BTRFS_QGROUP_LIMIT_MAX_RFER) {
		if (limit->max_rfer == CLEAR_VALUE) {
			qgroup->lim_flags &= ~BTRFS_QGROUP_LIMIT_MAX_RFER;
			limit->flags &= ~BTRFS_QGROUP_LIMIT_MAX_RFER;
			qgroup->max_rfer = 0;
		} else {
			qgroup->max_rfer = limit->max_rfer;
		}
	}
	if (limit->flags & BTRFS_QGROUP_LIMIT_MAX_EXCL) {
		if (limit->max_excl == CLEAR_VALUE) {
			qgroup->lim_flags &= ~BTRFS_QGROUP_LIMIT_MAX_EXCL;
			limit->flags &= ~BTRFS_QGROUP_LIMIT_MAX_EXCL;
			qgroup->max_excl = 0;
		} else {
			qgroup->max_excl = limit->max_excl;
		}
	}
	if (limit->flags & BTRFS_QGROUP_LIMIT_RSV_RFER) {
		if (limit->rsv_rfer == CLEAR_VALUE) {
			qgroup->lim_flags &= ~BTRFS_QGROUP_LIMIT_RSV_RFER;
			limit->flags &= ~BTRFS_QGROUP_LIMIT_RSV_RFER;
			qgroup->rsv_rfer = 0;
		} else {
			qgroup->rsv_rfer = limit->rsv_rfer;
		}
	}
	if (limit->flags & BTRFS_QGROUP_LIMIT_RSV_EXCL) {
		if (limit->rsv_excl == CLEAR_VALUE) {
			qgroup->lim_flags &= ~BTRFS_QGROUP_LIMIT_RSV_EXCL;
			limit->flags &= ~BTRFS_QGROUP_LIMIT_RSV_EXCL;
			qgroup->rsv_excl = 0;
		} else {
			qgroup->rsv_excl = limit->rsv_excl;
		}
	}
	qgroup->lim_flags |= limit->flags;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if ((qgroup->lim_flags & BTRFS_QGROUP_LIMIT_MAX_RFER && qgroup->max_rfer) ||
	    (qgroup->lim_flags & BTRFS_QGROUP_LIMIT_MAX_EXCL && qgroup->max_excl))
		has_limit = true;
	btrfs_root_set_has_quota_limit(root, has_limit);
#endif /* MY_ABC_HERE */
	spin_unlock(&fs_info->qgroup_lock);

	ret = update_qgroup_limit_item(trans, quota_root, qgroup);
	if (ret) {
#ifdef MY_ABC_HERE
#else
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
#endif /* MY_ABC_HERE */
		btrfs_info(fs_info, "unable to update quota limit for %llu",
		       qgroupid);
	}

out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}

#if defined(MY_ABC_HERE) || defined(MY_ABC_HERE)
int btrfs_insert_quota_record(struct btrfs_trans_handle *trans,
				  struct btrfs_delayed_ref_node *node)
{
	struct btrfs_delayed_data_ref *ref;
	struct btrfs_transaction *cur_trans = trans->transaction;
	struct btrfs_quota_account_rec *rec;

	rec = kmalloc(sizeof(*rec), GFP_NOFS);
	if (!rec)
		return -ENOMEM;

	ref = btrfs_delayed_node_to_data_ref(node);

	rec->ref_root = ref->root;
	rec->bytenr = node->bytenr;
	rec->num_bytes = node->num_bytes;
#ifdef MY_ABC_HERE
	rec->ram_bytes = ref->ram_bytes;
#endif /* MY_ABC_HERE */
	rec->op_type = (node->action == BTRFS_ADD_DELAYED_REF);
#ifdef MY_ABC_HERE
	rec->objectid = ref->objectid;
	rec->uid = ref->uid;
	rec->inode = syno_usrquota_inode_get(ref->inode);
#endif /* MY_ABC_HERE */

	spin_lock(&cur_trans->quota_account_lock);
	list_add_tail(&rec->list, &cur_trans->quota_account_list);
	spin_unlock(&cur_trans->quota_account_lock);
	return 0;
}

int qgroup_update_rfer(struct btrfs_trans_handle *trans,
					 struct btrfs_fs_info *fs_info,
					 struct btrfs_quota_account_rec *rec)
{
	int ret = 0;
	struct ulist *tmp;
	struct ulist_iterator uiter;
	struct ulist_node *unode;
	struct btrfs_qgroup *qgroup;
	struct btrfs_qgroup_list *glist;
	int sign = rec->op_type ? 1 : -1;
#ifdef MY_ABC_HERE
	u64 soft_qgroup_subvol_id = 0;
	u64 soft_qgroup_limit = 0;
	u64 soft_qgroup_used = 0;
	bool over_limit;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled)
		return 0;

	WARN_ON_ONCE(!fs_info->quota_root);
#else
	if (!fs_info->quota_enabled)
		return 0;

	BUG_ON(!fs_info->quota_root);
#endif /* MY_ABC_HERE */

	mutex_lock(&fs_info->qgroup_rescan_lock);
	if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN) {
		if (fs_info->qgroup_rescan_progress.objectid <= rec->bytenr) {
			mutex_unlock(&fs_info->qgroup_rescan_lock);
			return 0;
		}
	}
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	ASSERT(is_fstree(rec->ref_root));

	tmp = ulist_alloc(GFP_NOFS);
	if (!tmp)
		return -ENOMEM;

	spin_lock(&fs_info->qgroup_lock);
	if (!fs_info->quota_root)
		goto out;
	qgroup = find_qgroup_rb(fs_info, rec->ref_root);
	if (!qgroup)
		goto out;

#ifdef MY_ABC_HERE
	if (unlikely(sign < 0 && qgroup->rfer < rec->num_bytes))
		qgroup->rfer = 0;
	else {
		qgroup->rfer += sign * rec->num_bytes;
	}
	if (unlikely(sign < 0 && qgroup->rfer_cmpr < rec->ram_bytes))
		qgroup->rfer_cmpr = 0;
	else
		qgroup->rfer_cmpr += sign * rec->ram_bytes;
	if (sign > 0) {
		if (qgroup->reserved >= rec->ram_bytes)
			qgroup->reserved -= rec->ram_bytes;
		else
			qgroup->reserved = 0;
	}
#else
	if (unlikely(sign < 0 && qgroup->rfer < rec->num_bytes)) {
		qgroup->rfer = 0;
		qgroup->rfer_cmpr = 0;
	} else {
		qgroup->rfer += sign * rec->num_bytes;
		qgroup->rfer_cmpr += sign * rec->num_bytes;
	}
	if (sign > 0) {
		if (qgroup->reserved >= rec->num_bytes)
			qgroup->reserved -= rec->num_bytes;
		else
			qgroup->reserved = 0;
	}
#endif /* MY_ABC_HERE */

	qgroup_dirty(fs_info, qgroup);

	/* Get all of the parent groups that contain this qgroup */
	list_for_each_entry(glist, &qgroup->groups, next_group) {
		ret = ulist_add(tmp, glist->group->qgroupid,
				ptr_to_u64(glist->group), GFP_ATOMIC);
		if (ret < 0)
			goto out;
	}

	/* Iterate all of the parents and adjust their reference counts */
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(tmp, &uiter))) {
		qgroup = u64_to_ptr(unode->aux);

#ifdef MY_ABC_HERE
		if (unlikely(sign < 0 && qgroup->rfer < rec->num_bytes))
			qgroup->rfer = 0;
		else
			qgroup->rfer += sign * rec->num_bytes;
		if (unlikely(sign < 0 && qgroup->rfer_cmpr < rec->ram_bytes))
			qgroup->rfer_cmpr = 0;
		else
			qgroup->rfer_cmpr += sign * rec->ram_bytes;
		if (sign > 0) {
			if (qgroup->reserved >= rec->ram_bytes)
				qgroup->reserved -= rec->ram_bytes;
			else
				qgroup->reserved = 0;
		}
#else
		if (unlikely(sign < 0 && qgroup->rfer < rec->num_bytes)) {
			qgroup->rfer = 0;
			qgroup->rfer_cmpr = 0;
		} else {
			qgroup->rfer += sign * rec->num_bytes;
			qgroup->rfer_cmpr += sign * rec->num_bytes;
		}
		if (sign > 0) {
			if (qgroup->reserved >= rec->num_bytes)
				qgroup->reserved -= rec->num_bytes;
			else
				qgroup->reserved = 0;
		}
#endif /* MY_ABC_HERE */

		qgroup_dirty(fs_info, qgroup);
#ifdef MY_ABC_HERE
		if (!soft_qgroup_subvol_id)
			prepare_netlink_notification(qgroup, &soft_qgroup_subvol_id,
				&soft_qgroup_limit, &soft_qgroup_used, &over_limit);
#endif /* MY_ABC_HERE */

		/* Add any parents of the parents */
		list_for_each_entry(glist, &qgroup->groups, next_group) {
			ret = ulist_add(tmp, glist->group->qgroupid,
					ptr_to_u64(glist->group), GFP_ATOMIC);
			if (ret < 0)
				goto out;
		}
	}
	ret = 0;
out:
	ulist_free(tmp);
	spin_unlock(&fs_info->qgroup_lock);
#ifdef MY_ABC_HERE
	if (soft_qgroup_subvol_id)
		send_netlink_notification(fs_info, soft_qgroup_subvol_id,
			soft_qgroup_limit, soft_qgroup_used,
			(over_limit)? QGROUP_NL_C_OVER_LIMIT : QGROUP_NL_C_UNDER_LIMIT);
#endif /* MY_ABC_HERE */

	return ret;
}

int btrfs_quota_accounting(struct btrfs_trans_handle *trans,
				    struct btrfs_fs_info *fs_info)
{
	struct btrfs_quota_account_rec *rec;
	struct btrfs_transaction *cur_trans = trans->transaction;
	int ret = 0;

#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled) {
		WARN_ON(!list_empty(&cur_trans->quota_account_list));
		return 0;
	}
#endif /* MY_ABC_HERE */

	while (1) {
		spin_lock(&cur_trans->quota_account_lock);
		if (list_empty(&cur_trans->quota_account_list)) {
			spin_unlock(&cur_trans->quota_account_lock);
			break;
		}
		rec = list_first_entry(&cur_trans->quota_account_list,
					struct btrfs_quota_account_rec, list);
		list_del_init(&rec->list);
		spin_unlock(&cur_trans->quota_account_lock);
		if (!trans->aborted) {
			ret = qgroup_update_rfer(trans, fs_info, rec);
			if (ret)
				printk(KERN_WARNING"quota update failed root[%llu]\n",
						rec->ref_root);
		}
#ifdef MY_ABC_HERE
		if (!trans->aborted) {
			ret = btrfs_usrquota_account_ref(trans, fs_info, rec);
			if (ret)
				printk(KERN_WARNING"usrquota update failed root[%llu], uid[%u]\n",
						rec->ref_root, rec->uid);
		}
		syno_usrquota_inode_put(rec->inode);
#endif /* MY_ABC_HERE */
		kfree(rec);
	}

	/* The only error we could have is ENOMEM, quota update failed should
	 * not result in readonly volume.
	 */
	return 0;
}

int btrfs_qgroup_account_extent(struct btrfs_trans_handle *trans,
					 struct btrfs_fs_info *fs_info,
					 u64 bytenr, u64 num_bytes,
					 struct ulist *old_roots, struct ulist *new_roots)
{
	return 0;
}

int btrfs_qgroup_prepare_account_extents(struct btrfs_trans_handle *trans,
					 struct btrfs_fs_info *fs_info)
{
	return 0;
}
#else
int btrfs_qgroup_prepare_account_extents(struct btrfs_trans_handle *trans,
					 struct btrfs_fs_info *fs_info)
{
	struct btrfs_qgroup_extent_record *record;
	struct btrfs_delayed_ref_root *delayed_refs;
	struct rb_node *node;
	u64 qgroup_to_skip;
	int ret = 0;

	delayed_refs = &trans->transaction->delayed_refs;
	qgroup_to_skip = delayed_refs->qgroup_to_skip;

	/*
	 * No need to do lock, since this function will only be called in
	 * btrfs_commit_transaction().
	 */
	node = rb_first(&delayed_refs->dirty_extent_root);
	while (node) {
		record = rb_entry(node, struct btrfs_qgroup_extent_record,
				  node);
		ret = btrfs_find_all_roots(NULL, fs_info, record->bytenr, 0,
					   &record->old_roots);
		if (ret < 0)
			break;
		if (qgroup_to_skip)
			ulist_del(record->old_roots, qgroup_to_skip, 0);
		node = rb_next(node);
	}
	return ret;
}

struct btrfs_qgroup_extent_record *
btrfs_qgroup_insert_dirty_extent(struct btrfs_fs_info *fs_info,
				 struct btrfs_delayed_ref_root *delayed_refs,
				 struct btrfs_qgroup_extent_record *record)
{
	struct rb_node **p = &delayed_refs->dirty_extent_root.rb_node;
	struct rb_node *parent_node = NULL;
	struct btrfs_qgroup_extent_record *entry;
	u64 bytenr = record->bytenr;

	assert_spin_locked(&delayed_refs->lock);
	trace_btrfs_qgroup_insert_dirty_extent(fs_info, record);

	while (*p) {
		parent_node = *p;
		entry = rb_entry(parent_node, struct btrfs_qgroup_extent_record,
				 node);
		if (bytenr < entry->bytenr)
			p = &(*p)->rb_left;
		else if (bytenr > entry->bytenr)
			p = &(*p)->rb_right;
		else
			return entry;
	}

	rb_link_node(&record->node, parent_node, p);
	rb_insert_color(&record->node, &delayed_refs->dirty_extent_root);
	return NULL;
}

#define UPDATE_NEW	0
#define UPDATE_OLD	1
/*
 * Walk all of the roots that points to the bytenr and adjust their refcnts.
 */
static int qgroup_update_refcnt(struct btrfs_fs_info *fs_info,
				struct ulist *roots, struct ulist *tmp,
				struct ulist *qgroups, u64 seq, int update_old)
{
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	struct ulist_node *tmp_unode;
	struct ulist_iterator tmp_uiter;
	struct btrfs_qgroup *qg;
	int ret = 0;

	if (!roots)
		return 0;
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(roots, &uiter))) {
		qg = find_qgroup_rb(fs_info, unode->val);
		if (!qg)
			continue;

		ulist_reinit(tmp);
		ret = ulist_add(qgroups, qg->qgroupid, ptr_to_u64(qg),
				GFP_ATOMIC);
		if (ret < 0)
			return ret;
		ret = ulist_add(tmp, qg->qgroupid, ptr_to_u64(qg), GFP_ATOMIC);
		if (ret < 0)
			return ret;
		ULIST_ITER_INIT(&tmp_uiter);
		while ((tmp_unode = ulist_next(tmp, &tmp_uiter))) {
			struct btrfs_qgroup_list *glist;

			qg = u64_to_ptr(tmp_unode->aux);
			if (update_old)
				btrfs_qgroup_update_old_refcnt(qg, seq, 1);
			else
				btrfs_qgroup_update_new_refcnt(qg, seq, 1);
			list_for_each_entry(glist, &qg->groups, next_group) {
				ret = ulist_add(qgroups, glist->group->qgroupid,
						ptr_to_u64(glist->group),
						GFP_ATOMIC);
				if (ret < 0)
					return ret;
				ret = ulist_add(tmp, glist->group->qgroupid,
						ptr_to_u64(glist->group),
						GFP_ATOMIC);
				if (ret < 0)
					return ret;
			}
		}
	}
	return 0;
}

/*
 * Update qgroup rfer/excl counters.
 * Rfer update is easy, codes can explain themselves.
 *
 * Excl update is tricky, the update is split into 2 part.
 * Part 1: Possible exclusive <-> sharing detect:
 *	|	A	|	!A	|
 *  -------------------------------------
 *  B	|	*	|	-	|
 *  -------------------------------------
 *  !B	|	+	|	**	|
 *  -------------------------------------
 *
 * Conditions:
 * A:	cur_old_roots < nr_old_roots	(not exclusive before)
 * !A:	cur_old_roots == nr_old_roots	(possible exclusive before)
 * B:	cur_new_roots < nr_new_roots	(not exclusive now)
 * !B:	cur_new_roots == nr_new_roots	(possible exclusive now)
 *
 * Results:
 * +: Possible sharing -> exclusive	-: Possible exclusive -> sharing
 * *: Definitely not changed.		**: Possible unchanged.
 *
 * For !A and !B condition, the exception is cur_old/new_roots == 0 case.
 *
 * To make the logic clear, we first use condition A and B to split
 * combination into 4 results.
 *
 * Then, for result "+" and "-", check old/new_roots == 0 case, as in them
 * only on variant maybe 0.
 *
 * Lastly, check result **, since there are 2 variants maybe 0, split them
 * again(2x2).
 * But this time we don't need to consider other things, the codes and logic
 * is easy to understand now.
 */
static int qgroup_update_counters(struct btrfs_fs_info *fs_info,
				  struct ulist *qgroups,
				  u64 nr_old_roots,
				  u64 nr_new_roots,
				  u64 num_bytes, u64 seq)
{
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	struct btrfs_qgroup *qg;
	u64 cur_new_count, cur_old_count;

	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(qgroups, &uiter))) {
		bool dirty = false;

		qg = u64_to_ptr(unode->aux);
		cur_old_count = btrfs_qgroup_get_old_refcnt(qg, seq);
		cur_new_count = btrfs_qgroup_get_new_refcnt(qg, seq);

		trace_qgroup_update_counters(fs_info, qg->qgroupid,
					     cur_old_count, cur_new_count);

		/* Rfer update part */
		if (cur_old_count == 0 && cur_new_count > 0) {
			qg->rfer += num_bytes;
			qg->rfer_cmpr += num_bytes;
			dirty = true;
		}
		if (cur_old_count > 0 && cur_new_count == 0) {
			qg->rfer -= num_bytes;
			qg->rfer_cmpr -= num_bytes;
			dirty = true;
		}

		/* Excl update part */
		/* Exclusive/none -> shared case */
		if (cur_old_count == nr_old_roots &&
		    cur_new_count < nr_new_roots) {
			/* Exclusive -> shared */
			if (cur_old_count != 0) {
				qg->excl -= num_bytes;
				qg->excl_cmpr -= num_bytes;
				dirty = true;
			}
		}

		/* Shared -> exclusive/none case */
		if (cur_old_count < nr_old_roots &&
		    cur_new_count == nr_new_roots) {
			/* Shared->exclusive */
			if (cur_new_count != 0) {
				qg->excl += num_bytes;
				qg->excl_cmpr += num_bytes;
				dirty = true;
			}
		}

		/* Exclusive/none -> exclusive/none case */
		if (cur_old_count == nr_old_roots &&
		    cur_new_count == nr_new_roots) {
			if (cur_old_count == 0) {
				/* None -> exclusive/none */

				if (cur_new_count != 0) {
					/* None -> exclusive */
					qg->excl += num_bytes;
					qg->excl_cmpr += num_bytes;
					dirty = true;
				}
				/* None -> none, nothing changed */
			} else {
				/* Exclusive -> exclusive/none */

				if (cur_new_count == 0) {
					/* Exclusive -> none */
					qg->excl -= num_bytes;
					qg->excl_cmpr -= num_bytes;
					dirty = true;
				}
				/* Exclusive -> exclusive, nothing changed */
			}
		}

		if (dirty)
			qgroup_dirty(fs_info, qg);
	}
	return 0;
}

int
btrfs_qgroup_account_extent(struct btrfs_trans_handle *trans,
			    struct btrfs_fs_info *fs_info,
			    u64 bytenr, u64 num_bytes,
			    struct ulist *old_roots, struct ulist *new_roots)
{
	struct ulist *qgroups = NULL;
	struct ulist *tmp = NULL;
	u64 seq;
	u64 nr_new_roots = 0;
	u64 nr_old_roots = 0;
	int ret = 0;

	if (new_roots)
		nr_new_roots = new_roots->nnodes;
	if (old_roots)
		nr_old_roots = old_roots->nnodes;

#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled)
		goto out_free;
#else
	if (!fs_info->quota_enabled)
		goto out_free;
#endif /* MY_ABC_HERE */
	BUG_ON(!fs_info->quota_root);

	trace_btrfs_qgroup_account_extent(fs_info, bytenr, num_bytes,
					  nr_old_roots, nr_new_roots);

	qgroups = ulist_alloc(GFP_NOFS);
	if (!qgroups) {
		ret = -ENOMEM;
		goto out_free;
	}
	tmp = ulist_alloc(GFP_NOFS);
	if (!tmp) {
		ret = -ENOMEM;
		goto out_free;
	}

	mutex_lock(&fs_info->qgroup_rescan_lock);
	if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN) {
		if (fs_info->qgroup_rescan_progress.objectid <= bytenr) {
			mutex_unlock(&fs_info->qgroup_rescan_lock);
			ret = 0;
			goto out_free;
		}
	}
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	spin_lock(&fs_info->qgroup_lock);
	seq = fs_info->qgroup_seq;

	/* Update old refcnts using old_roots */
	ret = qgroup_update_refcnt(fs_info, old_roots, tmp, qgroups, seq,
				   UPDATE_OLD);
	if (ret < 0)
		goto out;

	/* Update new refcnts using new_roots */
	ret = qgroup_update_refcnt(fs_info, new_roots, tmp, qgroups, seq,
				   UPDATE_NEW);
	if (ret < 0)
		goto out;

	qgroup_update_counters(fs_info, qgroups, nr_old_roots, nr_new_roots,
			       num_bytes, seq);

	/*
	 * Bump qgroup_seq to avoid seq overlap
	 */
	fs_info->qgroup_seq += max(nr_old_roots, nr_new_roots) + 1;
out:
	spin_unlock(&fs_info->qgroup_lock);
out_free:
	ulist_free(tmp);
	ulist_free(qgroups);
	ulist_free(old_roots);
	ulist_free(new_roots);
	return ret;
}

int btrfs_qgroup_account_extents(struct btrfs_trans_handle *trans,
				 struct btrfs_fs_info *fs_info)
{
	struct btrfs_qgroup_extent_record *record;
	struct btrfs_delayed_ref_root *delayed_refs;
	struct ulist *new_roots = NULL;
	struct rb_node *node;
	u64 qgroup_to_skip;
	int ret = 0;

	delayed_refs = &trans->transaction->delayed_refs;
	qgroup_to_skip = delayed_refs->qgroup_to_skip;
	while ((node = rb_first(&delayed_refs->dirty_extent_root))) {
		record = rb_entry(node, struct btrfs_qgroup_extent_record,
				  node);

		trace_btrfs_qgroup_account_extents(fs_info, record);

		if (!ret) {
			/*
			 * Use (u64)-1 as time_seq to do special search, which
			 * doesn't lock tree or delayed_refs and search current
			 * root. It's safe inside commit_transaction().
			 */
			ret = btrfs_find_all_roots(trans, fs_info,
					record->bytenr, (u64)-1, &new_roots);
			if (ret < 0)
				goto cleanup;
			if (qgroup_to_skip)
				ulist_del(new_roots, qgroup_to_skip, 0);
			ret = btrfs_qgroup_account_extent(trans, fs_info,
					record->bytenr, record->num_bytes,
					record->old_roots, new_roots);
			record->old_roots = NULL;
			new_roots = NULL;
		}
cleanup:
		ulist_free(record->old_roots);
		ulist_free(new_roots);
		new_roots = NULL;
		rb_erase(node, &delayed_refs->dirty_extent_root);
		kfree(record);

	}
	return ret;
}
#endif /* MY_ABC_HERE && MY_ABC_HERE)*/

/*
 * called from commit_transaction. Writes all changed qgroups to disk.
 */
int btrfs_run_qgroups(struct btrfs_trans_handle *trans,
		      struct btrfs_fs_info *fs_info)
{
	struct btrfs_root *quota_root = fs_info->quota_root;
	int ret = 0;
#ifdef MY_ABC_HERE
#else
	int start_rescan_worker = 0;
#endif /* MY_ABC_HERE */

	if (!quota_root)
		goto out;

#ifdef MY_ABC_HERE
#else
	if (!fs_info->quota_enabled && fs_info->pending_quota_state)
		start_rescan_worker = 1;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if (fs_info->pending_quota_state == PENDING_QUOTA_STATE_V2)
		fs_info->syno_quota_v2_enabled = true;
	else
		fs_info->syno_quota_v2_enabled = false;

	if (fs_info->pending_quota_state == PENDING_QUOTA_STATE_V1)
		fs_info->syno_quota_v1_enabled = true;
	else
		fs_info->syno_quota_v1_enabled = false;
#else
	fs_info->quota_enabled = fs_info->pending_quota_state;
#endif /* MY_ABC_HERE */

	spin_lock(&fs_info->qgroup_lock);
	while (!list_empty(&fs_info->dirty_qgroups)) {
		struct btrfs_qgroup *qgroup;
		qgroup = list_first_entry(&fs_info->dirty_qgroups,
					  struct btrfs_qgroup, dirty);
		list_del_init(&qgroup->dirty);
		spin_unlock(&fs_info->qgroup_lock);
		ret = update_qgroup_info_item(trans, quota_root, qgroup);
#ifdef MY_ABC_HERE
		if ((ret || qgroup->need_rescan) &&
				fs_info->syno_quota_v2_enabled) {
			struct syno_quota_rescan_item_updater updater;

			syno_quota_rescan_item_init(&updater);
			updater.flags = SYNO_QUOTA_RESCAN_NEED;
			btrfs_add_update_syno_quota_rescan_item(trans,
				fs_info->quota_root,
				qgroup->qgroupid & ((1ULL << BTRFS_QGROUP_LEVEL_SHIFT) - 1),
				&updater);
			qgroup->need_rescan = false;
		}
#else
		if (ret)
			fs_info->qgroup_flags |=
					BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
#endif /* MY_ABC_HERE */
		ret = update_qgroup_limit_item(trans, quota_root, qgroup);
		if (ret)
			fs_info->qgroup_flags |=
					BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
		spin_lock(&fs_info->qgroup_lock);
	}
#ifdef MY_ABC_HERE
	if (fs_info->syno_quota_v1_enabled || fs_info->syno_quota_v2_enabled)
#else
	if (fs_info->quota_enabled)
#endif /* MY_ABC_HERE */
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_ON;
	else
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_ON;
	spin_unlock(&fs_info->qgroup_lock);

	ret = update_qgroup_status_item(trans, fs_info, quota_root);
	if (ret)
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;

#ifdef MY_ABC_HERE
#else
	if (!ret && start_rescan_worker) {
		ret = qgroup_rescan_init(fs_info, 0, 1);
		if (!ret) {
			qgroup_rescan_zero_tracking(fs_info);
			btrfs_queue_work(fs_info->qgroup_rescan_workers,
					 &fs_info->qgroup_rescan_work);
		}
		ret = 0;
	}
#endif /* MY_ABC_HERE */

out:

	return ret;
}

/*
 * Copy the accounting information between qgroups. This is necessary
 * when a snapshot or a subvolume is created. Throwing an error will
 * cause a transaction abort so we take extra care here to only error
 * when a readonly fs is a reasonable outcome.
 */
int btrfs_qgroup_inherit(struct btrfs_trans_handle *trans,
			 struct btrfs_fs_info *fs_info, u64 srcid, u64 objectid,
			 struct btrfs_qgroup_inherit *inherit)
{
	int ret = 0;
	int i;
	u64 *i_qgroups;
	struct btrfs_root *quota_root = fs_info->quota_root;
	struct btrfs_qgroup *srcgroup;
	struct btrfs_qgroup *dstgroup;
	u32 level_size = 0;
	u64 nums;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
#ifdef MY_ABC_HERE
	if (!fs_info->syno_quota_v1_enabled &&
		!fs_info->syno_quota_v2_enabled)
			goto out;
#else
	if (!fs_info->quota_enabled)
		goto out;
#endif /* MY_ABC_HERE */

	if (!quota_root) {
		ret = -EINVAL;
		goto out;
	}

	if (inherit) {
		i_qgroups = (u64 *)(inherit + 1);
		nums = inherit->num_qgroups + 2 * inherit->num_ref_copies +
		       2 * inherit->num_excl_copies;
		for (i = 0; i < nums; ++i) {
			srcgroup = find_qgroup_rb(fs_info, *i_qgroups);

			/*
			 * Zero out invalid groups so we can ignore
			 * them later.
			 */
			if (!srcgroup ||
			    ((srcgroup->qgroupid >> 48) <= (objectid >> 48)))
				*i_qgroups = 0ULL;

			++i_qgroups;
		}
	}

	/*
	 * create a tracking group for the subvol itself
	 */
	ret = add_qgroup_item(trans, quota_root, objectid);
	if (ret)
		goto out;

	if (srcid) {
		struct btrfs_root *srcroot;
		struct btrfs_key srckey;

		srckey.objectid = srcid;
		srckey.type = BTRFS_ROOT_ITEM_KEY;
		srckey.offset = (u64)-1;
		srcroot = btrfs_read_fs_root_no_name(fs_info, &srckey);
		if (IS_ERR(srcroot)) {
			ret = PTR_ERR(srcroot);
			goto out;
		}

		rcu_read_lock();
#ifdef MY_ABC_HERE
		// In syno quota v1/v2, we don't count metadata quota.
		level_size = 0;
#else
		level_size = srcroot->nodesize;
#endif /* MY_ABC_HERE */
		rcu_read_unlock();
	}

	/*
	 * add qgroup to all inherited groups
	 */
	if (inherit) {
		i_qgroups = (u64 *)(inherit + 1);
		for (i = 0; i < inherit->num_qgroups; ++i, ++i_qgroups) {
			if (*i_qgroups == 0)
				continue;
			ret = add_qgroup_relation_item(trans, quota_root,
						       objectid, *i_qgroups);
			if (ret && ret != -EEXIST)
				goto out;
			ret = add_qgroup_relation_item(trans, quota_root,
						       *i_qgroups, objectid);
			if (ret && ret != -EEXIST)
				goto out;
		}
		ret = 0;
	}


	spin_lock(&fs_info->qgroup_lock);

	dstgroup = add_qgroup_rb(fs_info, objectid);
	if (IS_ERR(dstgroup)) {
		ret = PTR_ERR(dstgroup);
		goto unlock;
	}

	if (inherit && inherit->flags & BTRFS_QGROUP_INHERIT_SET_LIMITS) {
		dstgroup->lim_flags = inherit->lim.flags;
		dstgroup->max_rfer = inherit->lim.max_rfer;
		dstgroup->max_excl = inherit->lim.max_excl;
		dstgroup->rsv_rfer = inherit->lim.rsv_rfer;
		dstgroup->rsv_excl = inherit->lim.rsv_excl;

		ret = update_qgroup_limit_item(trans, quota_root, dstgroup);
		if (ret) {
#ifdef MY_ABC_HERE
#else
			fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
#endif /* MY_ABC_HERE */
			btrfs_info(fs_info, "unable to update quota limit for %llu",
			       dstgroup->qgroupid);
			goto unlock;
		}
	}

	if (srcid) {
		srcgroup = find_qgroup_rb(fs_info, srcid);
		if (!srcgroup)
			goto unlock;

		/*
		 * We call inherit after we clone the root in order to make sure
		 * our counts don't go crazy, so at this point the only
		 * difference between the two roots should be the root node.
		 */
		dstgroup->rfer = srcgroup->rfer;
		dstgroup->rfer_cmpr = srcgroup->rfer_cmpr;
		dstgroup->excl = level_size;
		dstgroup->excl_cmpr = level_size;
		srcgroup->excl = level_size;
		srcgroup->excl_cmpr = level_size;

		/* inherit the limit info */
		dstgroup->lim_flags = srcgroup->lim_flags;
		dstgroup->max_rfer = srcgroup->max_rfer;
		dstgroup->max_excl = srcgroup->max_excl;
		dstgroup->rsv_rfer = srcgroup->rsv_rfer;
		dstgroup->rsv_excl = srcgroup->rsv_excl;

		qgroup_dirty(fs_info, dstgroup);
		qgroup_dirty(fs_info, srcgroup);
	}

	if (!inherit)
		goto unlock;

	i_qgroups = (u64 *)(inherit + 1);
	for (i = 0; i < inherit->num_qgroups; ++i) {
		if (*i_qgroups) {
			ret = add_relation_rb(quota_root->fs_info, objectid,
					      *i_qgroups);
			if (ret)
				goto unlock;
		}
		++i_qgroups;
	}

	for (i = 0; i <  inherit->num_ref_copies; ++i, i_qgroups += 2) {
		struct btrfs_qgroup *src;
		struct btrfs_qgroup *dst;

		if (!i_qgroups[0] || !i_qgroups[1])
			continue;

		src = find_qgroup_rb(fs_info, i_qgroups[0]);
		dst = find_qgroup_rb(fs_info, i_qgroups[1]);

		if (!src || !dst) {
			ret = -EINVAL;
			goto unlock;
		}

		dst->rfer = src->rfer - level_size;
		dst->rfer_cmpr = src->rfer_cmpr - level_size;
	}
	for (i = 0; i <  inherit->num_excl_copies; ++i, i_qgroups += 2) {
		struct btrfs_qgroup *src;
		struct btrfs_qgroup *dst;

		if (!i_qgroups[0] || !i_qgroups[1])
			continue;

		src = find_qgroup_rb(fs_info, i_qgroups[0]);
		dst = find_qgroup_rb(fs_info, i_qgroups[1]);

		if (!src || !dst) {
			ret = -EINVAL;
			goto unlock;
		}

		dst->excl = src->excl + level_size;
		dst->excl_cmpr = src->excl_cmpr + level_size;
	}

unlock:
	spin_unlock(&fs_info->qgroup_lock);
out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}

#ifdef MY_ABC_HERE
/*
 * Return 1 if we don't reserve qgroup, but it's not an EDQUOT error.
 * Caller is allowed to write.
 */
#endif /* MY_ABC_HERE */
static int qgroup_reserve(struct btrfs_root *root, u64 num_bytes)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *qgroup;
	struct btrfs_fs_info *fs_info = root->fs_info;
	u64 ref_root = root->root_key.objectid;
	int ret = 0;
	struct ulist_node *unode;
	struct ulist_iterator uiter;

	if (!is_fstree(ref_root))
		return 0;

	if (num_bytes == 0)
		return 0;

	spin_lock(&fs_info->qgroup_lock);
	quota_root = fs_info->quota_root;
#ifdef MY_ABC_HERE
	if (!quota_root) {
		ret = 1;
		goto out;
	}
#else
	if (!quota_root)
		goto out;
#endif /* MY_ABC_HERE */

	qgroup = find_qgroup_rb(fs_info, ref_root);
#ifdef MY_ABC_HERE
	if (!qgroup) {
		ret = 1;
		goto out;
	}
#else
	if (!qgroup)
		goto out;
#endif /* MY_ABC_HERE */

	/*
	 * in a first step, we check all affected qgroups if any limits would
	 * be exceeded
	 */
	ulist_reinit(fs_info->qgroup_ulist);
	ret = ulist_add(fs_info->qgroup_ulist, qgroup->qgroupid,
			(uintptr_t)qgroup, GFP_ATOMIC);
	if (ret < 0)
		goto out;
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(fs_info->qgroup_ulist, &uiter))) {
		struct btrfs_qgroup *qg;
		struct btrfs_qgroup_list *glist;

		qg = u64_to_ptr(unode->aux);

		if ((qg->lim_flags & BTRFS_QGROUP_LIMIT_MAX_RFER) &&
		    qg->reserved + (s64)qg->rfer + num_bytes >
		    qg->max_rfer
#ifdef MY_ABC_HERE
		    && !root->invalid_quota
#endif /* MY_ABC_HERE */
		) {
			ret = -EDQUOT;
			goto out;
		}

		if ((qg->lim_flags & BTRFS_QGROUP_LIMIT_MAX_EXCL) &&
		    qg->reserved + (s64)qg->excl + num_bytes >
		    qg->max_excl) {
			ret = -EDQUOT;
			goto out;
		}

		list_for_each_entry(glist, &qg->groups, next_group) {
			ret = ulist_add(fs_info->qgroup_ulist,
					glist->group->qgroupid,
					(uintptr_t)glist->group, GFP_ATOMIC);
			if (ret < 0)
				goto out;
		}
	}
	ret = 0;
	/*
	 * no limits exceeded, now record the reservation into all qgroups
	 */
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(fs_info->qgroup_ulist, &uiter))) {
		struct btrfs_qgroup *qg;

		qg = u64_to_ptr(unode->aux);

		qg->reserved += num_bytes;
	}

out:
	spin_unlock(&fs_info->qgroup_lock);
	return ret;
}

void btrfs_qgroup_free_refroot(struct btrfs_fs_info *fs_info,
			       u64 ref_root, u64 num_bytes)
{
	struct btrfs_root *quota_root;
	struct btrfs_qgroup *qgroup;
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	int ret = 0;

	if (!is_fstree(ref_root))
		return;

	if (num_bytes == 0)
		return;

	spin_lock(&fs_info->qgroup_lock);

	quota_root = fs_info->quota_root;
	if (!quota_root)
		goto out;

	qgroup = find_qgroup_rb(fs_info, ref_root);
	if (!qgroup)
		goto out;

	ulist_reinit(fs_info->qgroup_ulist);
	ret = ulist_add(fs_info->qgroup_ulist, qgroup->qgroupid,
			(uintptr_t)qgroup, GFP_ATOMIC);
	if (ret < 0)
		goto out;
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(fs_info->qgroup_ulist, &uiter))) {
		struct btrfs_qgroup *qg;
		struct btrfs_qgroup_list *glist;

		qg = u64_to_ptr(unode->aux);

#ifdef MY_ABC_HERE
		if (qg->reserved >= num_bytes)
			qg->reserved -= num_bytes;
		else {
#ifdef MY_ABC_HERE
			if (fs_info->syno_quota_v2_enabled)
				WARN_ONCE(1, "qgroup %llu reserved space underflow, "
					"have %llu to free %llu", qgroup->qgroupid,
					qg->reserved, num_bytes);
#endif /* MY_ABC_HERE */
			qg->reserved = 0;
		}
#else
		qg->reserved -= num_bytes;
#endif /* MY_ABC_HERE */

		list_for_each_entry(glist, &qg->groups, next_group) {
			ret = ulist_add(fs_info->qgroup_ulist,
					glist->group->qgroupid,
					(uintptr_t)glist->group, GFP_ATOMIC);
			if (ret < 0)
				goto out;
		}
	}

out:
	spin_unlock(&fs_info->qgroup_lock);
}

static inline void qgroup_free(struct btrfs_root *root, u64 num_bytes)
{
	return btrfs_qgroup_free_refroot(root->fs_info, root->objectid,
					 num_bytes);
}

#ifdef MY_ABC_HERE
/*
 * Copied from btrfs_qgroup_free_refroot()
 * Use after inode_add_bytes() / inode_sub_bytes(), so we are always in a transaction
 * and our accounting will be committed in btrfs_run_qgroups().
 */
int btrfs_qgroup_syno_accounting(struct btrfs_inode *b_inode,
		u64 add_bytes, u64 del_bytes, enum syno_quota_account_type type)
{
	struct btrfs_qgroup *qgroup;
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	struct btrfs_root *root = b_inode->root;
	struct btrfs_fs_info *fs_info = root->fs_info;
	u64 ref_root = root->root_key.objectid;
	u64 ino = b_inode->location.objectid;
	int ret = 0;
#ifdef MY_ABC_HERE
	u64 soft_qgroup_subvol_id = 0;
	u64 soft_qgroup_limit = 0;
	u64 soft_qgroup_used = 0;
	bool over_limit;
#endif /* MY_ABC_HERE */

	if (!is_fstree(ref_root))
		return -EINVAL;

	if (add_bytes == del_bytes && type != UPDATE_QUOTA_FREE_RESERVED)
		return 0;

	if (!fs_info->syno_quota_v2_enabled)
		return 0;

	spin_lock(&fs_info->qgroup_lock);

	if (!fs_info->quota_root)
		goto out;

	qgroup = find_qgroup_rb(fs_info, ref_root);
	if (!qgroup)
		goto out;

	add_bytes = round_up(add_bytes, fs_info->sectorsize);
	del_bytes = round_up(del_bytes, fs_info->sectorsize);

	ulist_reinit(fs_info->qgroup_ulist);
	ret = ulist_add(fs_info->qgroup_ulist, qgroup->qgroupid,
			(uintptr_t)qgroup, GFP_ATOMIC);
	if (ret < 0) {
		qgroup->need_rescan = true;
		goto out;
	}
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(fs_info->qgroup_ulist, &uiter))) {
		struct btrfs_qgroup *qg;
		struct btrfs_qgroup_list *glist;

		qg = u64_to_ptr(unode->aux);

		switch (type) {
		case ADD_QUOTA_RESCAN:
			qg->rfer += add_bytes;
#ifdef MY_ABC_HERE
			if (!soft_qgroup_subvol_id)
				prepare_netlink_notification(qg, &soft_qgroup_subvol_id,
					&soft_qgroup_limit, &soft_qgroup_used, &over_limit);
#endif /* MY_ABC_HERE */
			break;
		case UPDATE_QUOTA_FREE_RESERVED:
			if (qg->reserved >= add_bytes)
				qg->reserved -= add_bytes;
			else {
				WARN_ONCE(1, "qgroup %llu reserved space underflow, "
					"have %llu to free %llu", qgroup->qgroupid,
					qg->reserved, add_bytes);
				qg->reserved = 0;
			}
			/* fall through */
		case UPDATE_QUOTA:
			if (btrfs_quota_rescan_check(root, ino)) {
				qg->rfer += add_bytes;

				if (qg->rfer < del_bytes) {
					if (!root->invalid_quota)
						WARN_ONCE(1, "qgroup %llu ref underflow, have "
							"%llu to free %llu", qgroup->qgroupid, qg->rfer, del_bytes);
					qg->rfer = 0;
					qg->need_rescan = true;
				} else
					qg->rfer -= del_bytes;

#ifdef MY_ABC_HERE
				if (!soft_qgroup_subvol_id)
					prepare_netlink_notification(qg, &soft_qgroup_subvol_id,
						&soft_qgroup_limit, &soft_qgroup_used, &over_limit);
#endif /* MY_ABC_HERE */
			}
			break;
		}

		qgroup_dirty(fs_info, qg);
		list_for_each_entry(glist, &qg->groups, next_group) {
			ret = ulist_add(fs_info->qgroup_ulist,
					glist->group->qgroupid,
					(uintptr_t)glist->group, GFP_ATOMIC);
			if (ret < 0)
				goto out;
		}
	}
	ret = 0;

out:
	spin_unlock(&fs_info->qgroup_lock);
#ifdef MY_ABC_HERE
	if (soft_qgroup_subvol_id && (add_bytes != del_bytes))
		send_netlink_notification(fs_info, soft_qgroup_subvol_id,
			soft_qgroup_limit, soft_qgroup_used,
			(over_limit)? QGROUP_NL_C_OVER_LIMIT : QGROUP_NL_C_UNDER_LIMIT);
#endif /* MY_ABC_HERE */
	return ret;
}

/*
 * Similar to btrfs_qgroup_syno_accounting(), but used only in rescan, where
 * we don't have in-memory inode.
 */
static int btrfs_qgroup_syno_accounting_rescan(struct btrfs_root *root, u64 num_bytes)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_qgroup *qgroup;
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	u64 subvol_id = root->root_key.objectid;
	int ret = 0;

	if (num_bytes == 0)
		return 0;

	if (!fs_info->syno_quota_v2_enabled)
		return 0;

	spin_lock(&fs_info->qgroup_lock);

	if (!fs_info->quota_root)
		goto out;

	num_bytes = round_up(num_bytes, fs_info->sectorsize);
	qgroup = find_qgroup_rb(fs_info, subvol_id);
	if (!qgroup)
		goto out;

	ulist_reinit(fs_info->qgroup_ulist);
	ret = ulist_add(fs_info->qgroup_ulist, qgroup->qgroupid,
			(uintptr_t)qgroup, GFP_ATOMIC);
	if (ret < 0)
		goto out;
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(fs_info->qgroup_ulist, &uiter))) {
		struct btrfs_qgroup *qg;
		struct btrfs_qgroup_list *glist;

		qg = u64_to_ptr(unode->aux);

		qg->rfer += num_bytes;
		qgroup_dirty(fs_info, qg);

		list_for_each_entry(glist, &qg->groups, next_group) {
			ret = ulist_add(fs_info->qgroup_ulist,
					glist->group->qgroupid,
					(uintptr_t)glist->group, GFP_ATOMIC);
			if (ret < 0)
				goto out;
		}
	}
	ret = 0;

out:
	spin_unlock(&fs_info->qgroup_lock);
	return ret;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
void assert_qgroups_uptodate(struct btrfs_trans_handle *trans)
{
	struct btrfs_transaction *cur_trans = trans->transaction;

#ifdef MY_ABC_HERE
	if (!trans->root->fs_info->syno_quota_v1_enabled)
		return;
#endif /* MY_ABC_HERE */

	if (list_empty(&cur_trans->quota_account_list) && !trans->delayed_ref_elem.seq)
		return;
	btrfs_err(trans->root->fs_info,
		"qgroups not uptodate in trans handle %p:  list is%s empty, "
		"seq is %#x.%x",
		trans, list_empty(&cur_trans->quota_account_list) ? "" : " not",
		(u32)(trans->delayed_ref_elem.seq >> 32),
		(u32)trans->delayed_ref_elem.seq);
	BUG();
}
#else
void assert_qgroups_uptodate(struct btrfs_trans_handle *trans)
{
	if (list_empty(&trans->qgroup_ref_list) && !trans->delayed_ref_elem.seq)
		return;
	btrfs_err(trans->root->fs_info,
		"qgroups not uptodate in trans handle %p:  list is%s empty, "
		"seq is %#x.%x",
		trans, list_empty(&trans->qgroup_ref_list) ? "" : " not",
		(u32)(trans->delayed_ref_elem.seq >> 32),
		(u32)trans->delayed_ref_elem.seq);
	BUG();
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
// Progress path can be ctx->current_path or ctx->end_path.
static void syno_quota_update_progress_path(int progress_path[BTRFS_MAX_LEVEL][2],
		struct btrfs_path *path)
{
	int i;

	for (i = 1; i < BTRFS_MAX_LEVEL; i++) {
		if (path->nodes[i]) {
			progress_path[i][0] = btrfs_header_nritems(path->nodes[i]);
			progress_path[i][1] = path->slots[i];
		} else {
			progress_path[i][0] = 0;
			progress_path[i][1] = 0;
		}
	}
}

/*
 * Return 0 when more leafs are to be scanned.
 * Return 1 when done.
 * Never return -1 since we can try next subvol if error occurs.
 */
static int syno_quota_rescan_leaf(struct btrfs_trans_handle *trans,
			struct btrfs_fs_info *fs_info, struct btrfs_path *path)
{
	struct btrfs_root *quota_root = fs_info->quota_root;
	struct syno_quota_rescan_ctx *ctx = fs_info->syno_quota_rescan_ctx;
	struct ulist *ulist = fs_info->syno_quota_rescan_subvol_ulist;
	struct ulist_node *node;
	struct btrfs_root *root;
	struct extent_buffer *leaf;
	struct btrfs_key key, found_key;
	struct inode *inode;
	struct btrfs_inode_item *inode_item;
	struct syno_quota_rescan_item_updater updater;
	u64 subvol_id;
	u64 ino;
	u64 max_objectid = 0;
	u64 num_bytes;
	u64 uid;
	int srcu_index;
	int nritems;
	int ret = 0;
	int err = 0;

	if (unlikely(!ctx || !ulist)) {
		WARN_ON(1);
		return 1;
	}

	mutex_lock(&fs_info->qgroup_rescan_lock);
	if (list_empty(&ulist->nodes)) {
		mutex_unlock(&fs_info->qgroup_rescan_lock);
		return 1;
	}
	node = list_entry(ulist->nodes.next, struct ulist_node, list);
	subvol_id = node->val;
	ino = node->aux;
	ino++;
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	key.objectid = subvol_id;
	key.type = BTRFS_ROOT_ITEM_KEY;
	key.offset = (u64)-1;

	srcu_index = srcu_read_lock(&fs_info->subvol_srcu);
	root = btrfs_read_fs_root_no_name(fs_info, &key);
	if (IS_ERR(root)) {
		srcu_read_unlock(&fs_info->subvol_srcu, srcu_index);
		ret = PTR_ERR(root);
		btrfs_err(fs_info, "Failed to call btrfs_get_fs_root() for root %llu, ret = %d", subvol_id, ret);
		goto error_clean;
	}
	btrfs_hold_fs_root(root);
	srcu_read_unlock(&fs_info->subvol_srcu, srcu_index);

	if (unlikely(btrfs_root_dead(root))) {
		btrfs_release_fs_root(root);
		goto error_clean;
	}

	// We have changed to another root, reset progress info.
	if (ctx->subvol_id != subvol_id) {
		struct btrfs_syno_quota_rescan_item rescan_item;

		ret = btrfs_read_syno_quota_rescan_item(fs_info->quota_root, subvol_id, &rescan_item);
		if (ret)
			goto error_clean;

		ctx->subvol_id = subvol_id;
		ctx->subvol_size = rescan_item.tree_size;
		ctx->subvol_progress = 0;
		memset(ctx->current_path, 0, sizeof(ctx->current_path));
	}

	key.objectid = ino;
	key.type = BTRFS_INODE_ITEM_KEY;
	key.offset = 0;

search_again:
	ret = btrfs_search_slot_for_read(root, &key, path, 1, 0);
	if (ret < 0) {
		root->invalid_quota = false;
		btrfs_release_fs_root(root);
		btrfs_err(fs_info, "btrfs_search_slot() failed in root %llu, ret = %d", subvol_id, ret);
		goto error_clean;
	} else if (ret > 0)
		goto out;

	leaf = path->nodes[0];
	nritems = btrfs_header_nritems(path->nodes[0]);
	if (nritems) {
		btrfs_item_key_to_cpu(leaf, &found_key, nritems - 1);
		max_objectid = found_key.objectid;
		syno_quota_update_progress_path(ctx->current_path, path);
	}

next_slot:
	if (path->slots[0] >= nritems)
		goto out;
	btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);
	if (found_key.type != BTRFS_INODE_ITEM_KEY || found_key.offset != 0) {
		if (found_key.objectid == max_objectid)
			goto out;

		path->slots[0]++;
		goto next_slot;
	}

	ino = found_key.objectid;
	if (ino > root->rescan_end_inode)
		goto out;

	down_write(&root->rescan_lock);
	if (!btrfs_test_inode_nowait(fs_info->sb, ino, root)) {
		inode_item = btrfs_item_ptr(leaf, path->slots[0], struct btrfs_inode_item);
		num_bytes = btrfs_inode_nbytes(leaf, inode_item);
		uid = btrfs_inode_uid(leaf, inode_item);

		ret = btrfs_qgroup_syno_accounting_rescan(root, num_bytes);
		if (ret) {
			btrfs_warn(fs_info, "Failed in btrfs_qgroup_syno_accounting_rescan(), "
				"subvol_id = %llu, ino = %llu, ret = %d", subvol_id, ino, ret);
			err = ret;
		}

		ret = btrfs_usrquota_syno_accounting_rescan(root, uid, num_bytes);
		if (ret) {
			btrfs_warn(fs_info, "Failed in btrfs_usrquota_syno_accounting_rescan(), "
				"subvol_id = %llu, ino = %llu, uid = %llu, ret = %d",
				subvol_id, ino, uid, ret);
			err = ret;
		}

		root->rescan_inode = ino;
		up_write(&root->rescan_lock);

		path->slots[0]++;
		goto next_slot;
	}

	up_write(&root->rescan_lock);
	btrfs_release_path(path);
	inode = btrfs_iget(fs_info->sb, &found_key, root, NULL);
	if (IS_ERR(inode)) {
		ret = PTR_ERR(inode);
		if (ret != -ENOENT) {
			btrfs_warn(fs_info, "Failed to call btrfs_iget(), "
				"subvol_id = %llu, ino = %llu, ret = %d", subvol_id, ino, ret);
			err = ret;
		}
	} else {
		down_write(&root->rescan_lock);
		num_bytes = inode_get_bytes(inode);

		ret = btrfs_qgroup_syno_accounting(BTRFS_I(inode), num_bytes, 0, ADD_QUOTA_RESCAN);
		if (ret) {
			btrfs_warn(fs_info, "Failed in btrfs_qgroup_syno_accounting(), "
				"subvol_id = %llu, ino = %llu, ret = %d", subvol_id, ino, ret);
			err = ret;
		}

		ret = btrfs_usrquota_syno_accounting(BTRFS_I(inode), num_bytes, 0, ADD_QUOTA_RESCAN);
		if (ret) {
			btrfs_warn(fs_info, "Failed in btrfs_usrquota_syno_accounting(), "
				"subvol_id = %llu, ino = %llu, uid = %llu, ret = %d",
				subvol_id, ino, uid, ret);
			err = ret;
		}

		root->rescan_inode = ino;
		up_write(&root->rescan_lock);
		btrfs_add_delayed_iput(inode);
	}

	if (ino < max_objectid) {
		key.objectid = ino + 1;
		goto search_again;
	}

out:
	if (ino < max_objectid)
		ino = max_objectid;
	btrfs_release_path(path);

	// Mark err but continue the rescan, or we'll see very strange (perhaps zero) quota usage.
	if (unlikely(err)) {
		syno_quota_rescan_item_init(&updater);
		updater.flags = SYNO_QUOTA_RESCAN_ERR;
		btrfs_add_update_syno_quota_rescan_item(trans, quota_root, subvol_id, &updater);
	}
	ret = 0;

	mutex_lock(&fs_info->qgroup_rescan_lock);
	node = ulist_search(ulist, subvol_id);
	if (node) {
		if (ino > root->rescan_end_inode) { // This subvol is done. Switch to next subvol.
			root->rescan_inode = (u64)-1;
			root->rescan_end_inode = (u64)-1;

			ulist_del(ulist, subvol_id, node->aux);
			syno_quota_rescan_item_init(&updater);
			updater.rescan_inode = (u64)-1;
			updater.end_inode = (u64)-1;
			updater.flags = SYNO_QUOTA_RESCAN_DONE;
			ret = btrfs_add_update_syno_quota_rescan_item(trans, quota_root, subvol_id, &updater);
			if (ret)
				btrfs_warn(fs_info, "Failed to update syno quota rescan item, ret = %d", ret);

			if (!list_empty(&ulist->nodes)) {
				update_syno_quota_rescan_progress(quota_root, ctx,
					subvol_id, SYNO_QUOTA_PROGRESS_FINISH_ONE);
				ret = 0;
			} else {
				update_syno_quota_rescan_progress(quota_root, ctx,
					subvol_id, SYNO_QUOTA_PROGRESS_FINISH_ALL);
				remove_syno_quota_rescan_list(trans, quota_root);
				ret = 1;
			}
		} else {
			WARN_ON(ino < node->aux);
			node->aux = (ino > node->aux)? ino : node->aux;
			syno_quota_rescan_item_init(&updater);
			updater.rescan_inode = node->aux;
			updater.flags = SYNO_QUOTA_RESCAN_DOING;
			ret = btrfs_add_update_syno_quota_rescan_item(trans, quota_root, subvol_id, &updater);
			if (ret)
				btrfs_warn(fs_info, "Failed to update syno quota rescan item, ret = %d", ret);
			ret = 0;
		}
	}
	mutex_unlock(&fs_info->qgroup_rescan_lock);
	btrfs_release_fs_root(root);

	return ret;

error_clean:
	mutex_lock(&fs_info->qgroup_rescan_lock);
	if (ret < 0) {
		syno_quota_rescan_item_init(&updater);
		updater.rescan_inode = (u64)-1;
		updater.end_inode = (u64)-1;
		updater.flags = SYNO_QUOTA_RESCAN_ERR | SYNO_QUOTA_RESCAN_DONE;
		btrfs_add_update_syno_quota_rescan_item(trans, quota_root, subvol_id, &updater);
	}
	node = ulist_search(ulist, subvol_id);
	if (node) {
		ulist_del(ulist, subvol_id, node->aux);
		if (!list_empty(&ulist->nodes)) {
			update_syno_quota_rescan_progress(quota_root, ctx,
				subvol_id, SYNO_QUOTA_PROGRESS_REMOVE_SCANNING);
			ret = 0; // Scan next subvol.
		} else {
			update_syno_quota_rescan_progress(quota_root, ctx,
				subvol_id, SYNO_QUOTA_PROGRESS_FINISH_ALL);
			remove_syno_quota_rescan_list(trans, quota_root);
			ret = 1;
		}
	} else
		ret = 0; // Scan next subvol.
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	return ret;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
#else
/*
 * Check if the leaf is the last leaf. Which means all node pointers
 * are at their last position.
 */
static bool is_last_leaf(struct btrfs_path *path)
{
	int i;

	for (i = 1; i < BTRFS_MAX_LEVEL && path->nodes[i]; i++) {
		if (path->slots[i] != btrfs_header_nritems(path->nodes[i]) - 1)
			return false;
	}
	return true;
}

/*
 * returns < 0 on error, 0 when more leafs are to be scanned.
 * returns 1 when done.
 */
static int
qgroup_rescan_leaf(struct btrfs_fs_info *fs_info, struct btrfs_path *path,
		   struct btrfs_trans_handle *trans)
{
	struct btrfs_key found;
	struct extent_buffer *scratch_leaf = NULL;
	struct ulist *roots = NULL;
	struct seq_list tree_mod_seq_elem = SEQ_LIST_INIT(tree_mod_seq_elem);
	u64 num_bytes;
	bool done;
	int slot;
	int ret;

	mutex_lock(&fs_info->qgroup_rescan_lock);
	ret = btrfs_search_slot_for_read(fs_info->extent_root,
					 &fs_info->qgroup_rescan_progress,
					 path, 1, 0);

	pr_debug("current progress key (%llu %u %llu), search_slot ret %d\n",
		 fs_info->qgroup_rescan_progress.objectid,
		 fs_info->qgroup_rescan_progress.type,
		 fs_info->qgroup_rescan_progress.offset, ret);

	if (ret) {
		/*
		 * The rescan is about to end, we will not be scanning any
		 * further blocks. We cannot unset the RESCAN flag here, because
		 * we want to commit the transaction if everything went well.
		 * To make the live accounting work in this phase, we set our
		 * scan progress pointer such that every real extent objectid
		 * will be smaller.
		 */
		fs_info->qgroup_rescan_progress.objectid = (u64)-1;
		btrfs_release_path(path);
		mutex_unlock(&fs_info->qgroup_rescan_lock);
		return ret;
	}
	done = is_last_leaf(path);

	btrfs_item_key_to_cpu(path->nodes[0], &found,
			      btrfs_header_nritems(path->nodes[0]) - 1);
	fs_info->qgroup_rescan_progress.objectid = found.objectid + 1;

	btrfs_get_tree_mod_seq(fs_info, &tree_mod_seq_elem);
	scratch_leaf = btrfs_clone_extent_buffer(path->nodes[0]);
	if (!scratch_leaf) {
		ret = -ENOMEM;
		mutex_unlock(&fs_info->qgroup_rescan_lock);
		goto out;
	}
	extent_buffer_get(scratch_leaf);
	btrfs_tree_read_lock(scratch_leaf);
	btrfs_set_lock_blocking_rw(scratch_leaf, BTRFS_READ_LOCK);
	slot = path->slots[0];
	btrfs_release_path(path);
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	for (; slot < btrfs_header_nritems(scratch_leaf); ++slot) {
		btrfs_item_key_to_cpu(scratch_leaf, &found, slot);
		if (found.type != BTRFS_EXTENT_ITEM_KEY &&
		    found.type != BTRFS_METADATA_ITEM_KEY)
			continue;
		if (found.type == BTRFS_METADATA_ITEM_KEY)
			num_bytes = fs_info->extent_root->nodesize;
		else
			num_bytes = found.offset;

		ret = btrfs_find_all_roots(NULL, fs_info, found.objectid, 0,
					   &roots);
		if (ret < 0)
			goto out;
		/* For rescan, just pass old_roots as NULL */
		ret = btrfs_qgroup_account_extent(trans, fs_info,
				found.objectid, num_bytes, NULL, roots);
		if (ret < 0)
			goto out;
	}
out:
	if (scratch_leaf) {
		btrfs_tree_read_unlock_blocking(scratch_leaf);
		free_extent_buffer(scratch_leaf);
	}
	btrfs_put_tree_mod_seq(fs_info, &tree_mod_seq_elem);

	if (done && !ret) {
		ret = 1;
		fs_info->qgroup_rescan_progress.objectid = (u64)-1;
	}
	return ret;
}

static void btrfs_qgroup_rescan_worker(struct btrfs_work *work)
{
	struct btrfs_fs_info *fs_info = container_of(work, struct btrfs_fs_info,
						     qgroup_rescan_work);
	struct btrfs_path *path;
	struct btrfs_trans_handle *trans = NULL;
	int err = -ENOMEM;
	int ret = 0;

	path = btrfs_alloc_path();
	if (!path)
		goto out;

	err = 0;
	while (!err && !btrfs_fs_closing(fs_info)) {
		trans = btrfs_start_transaction(fs_info->fs_root, 0);
		if (IS_ERR(trans)) {
			err = PTR_ERR(trans);
			break;
		}
		if (!fs_info->quota_enabled) {
			err = -EINTR;
		} else {
			err = qgroup_rescan_leaf(fs_info, path, trans);
		}
		if (err > 0)
			btrfs_commit_transaction(trans, fs_info->fs_root);
		else
			btrfs_end_transaction(trans, fs_info->fs_root);
	}

out:
	btrfs_free_path(path);

	mutex_lock(&fs_info->qgroup_rescan_lock);
	if (err > 0 &&
	    fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT) {
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
	} else if (err < 0) {
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;
	}
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	/*
	 * only update status, since the previous part has already updated the
	 * qgroup info.
	 */
	trans = btrfs_start_transaction(fs_info->quota_root, 1);
	if (IS_ERR(trans)) {
		err = PTR_ERR(trans);
		trans = NULL;
		btrfs_err(fs_info,
			  "fail to start transaction for status update: %d\n",
			  err);
	}

	mutex_lock(&fs_info->qgroup_rescan_lock);
	if (!btrfs_fs_closing(fs_info))
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_RESCAN;
	if (trans) {
		ret = update_qgroup_status_item(trans, fs_info, fs_info->quota_root);
		if (ret < 0) {
			err = ret;
			btrfs_err(fs_info, "fail to update qgroup status: %d",
				  err);
		}
	}
	fs_info->qgroup_rescan_running = false;
	complete_all(&fs_info->qgroup_rescan_completion);
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	if (!trans)
		return;

	btrfs_end_transaction(trans, fs_info->quota_root);

	if (btrfs_fs_closing(fs_info)) {
		btrfs_info(fs_info, "qgroup scan paused");
	} else if (err >= 0) {
		btrfs_info(fs_info, "qgroup scan completed%s",
			err > 0 ? " (inconsistency flag cleared)" : "");
	} else {
		btrfs_err(fs_info, "qgroup scan failed with %d", err);
	}
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
static bool rescan_should_stop(struct btrfs_fs_info *fs_info)
{
	return btrfs_fs_closing(fs_info) ||
		test_bit(BTRFS_FS_STATE_REMOUNTING, &fs_info->fs_state) ||
		fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_PAUSE;
}

// Copied from btrfs_qgroup_rescan_worker.
static void btrfs_syno_quota_rescan_worker(struct btrfs_work *work)
{
	struct btrfs_fs_info *fs_info = container_of(work, struct btrfs_fs_info,
						     qgroup_rescan_work);
	struct btrfs_path *path;
	struct btrfs_trans_handle *trans = NULL;
	int err;
	int ret = 0;
	bool stopped = false;

again:
	path = btrfs_alloc_path();
	if (!path) {
		err = -ENOMEM;
		goto out;
	}
	path->reada = READA_FORWARD_ALWAYS;

	err = 0;
	while (!err && !(stopped = rescan_should_stop(fs_info))) {
		trans = btrfs_start_transaction(fs_info->fs_root, 0);
		if (IS_ERR(trans)) {
			err = PTR_ERR(trans);
			break;
		}

		if (!fs_info->syno_quota_v2_enabled)
			err = -EINTR;
		else
			err = syno_quota_rescan_leaf(trans, fs_info, path);

		if (err > 0)
			btrfs_commit_transaction(trans, fs_info->fs_root);
		else
			btrfs_end_transaction(trans, fs_info->fs_root);
	}

out:
	btrfs_free_path(path);
	if (err < 0)
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT;

	/*
	 * only update status, since the previous part has already updated the
	 * qgroup info.
	 */
	trans = btrfs_start_transaction(fs_info->quota_root, 1);
	if (IS_ERR(trans)) {
		err = PTR_ERR(trans);
		trans = NULL;
		btrfs_err(fs_info,
			  "fail to start transaction for status update: %d",
			  err);
	}

	mutex_lock(&fs_info->qgroup_rescan_lock);
	// In case another rescan join in after we left syno_quota_rescan_leaf().
	if (err >= 0 && !stopped && fs_info->syno_quota_rescan_subvol_ulist &&
			!list_empty(&fs_info->syno_quota_rescan_subvol_ulist->nodes)) {
		mutex_unlock(&fs_info->qgroup_rescan_lock);
		if (trans)
			btrfs_end_transaction(trans, fs_info->quota_root);
		goto again;
	}
	if (!stopped)
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_RESCAN;
	if (trans) {
		ret = update_qgroup_status_item(trans, fs_info, fs_info->quota_root);
		if (ret < 0) {
			err = ret;
			btrfs_err(fs_info, "fail to update qgroup status: %d",
				  err);
		}
	}
	fs_info->qgroup_rescan_running = false;
	complete_all(&fs_info->qgroup_rescan_completion);
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	if (!trans)
		return;

	btrfs_end_transaction(trans, fs_info->quota_root);

	if (stopped)
		btrfs_info(fs_info, "qgroup scan paused");
	else if (err >= 0)
		// Now we clear inconsistent flag by ioctl.
		btrfs_info(fs_info, "qgroup scan completed");
	else
		btrfs_err(fs_info, "qgroup scan failed with %d", err);
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
#else
/*
 * Checks that (a) no rescan is running and (b) quota is enabled. Allocates all
 * memory required for the rescan context.
 */
static int
qgroup_rescan_init(struct btrfs_fs_info *fs_info, u64 progress_objectid,
		   int init_flags)
{
	int ret = 0;

	if (!init_flags &&
	    (!(fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN) ||
	     !(fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_ON))) {
		ret = -EINVAL;
		goto err;
	}

	mutex_lock(&fs_info->qgroup_rescan_lock);
	spin_lock(&fs_info->qgroup_lock);

	if (init_flags) {
		if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN)
			ret = -EINPROGRESS;
		else if (!(fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_ON))
			ret = -EINVAL;

		if (ret) {
			spin_unlock(&fs_info->qgroup_lock);
			mutex_unlock(&fs_info->qgroup_rescan_lock);
			goto err;
		}
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_RESCAN;
	}

	memset(&fs_info->qgroup_rescan_progress, 0,
		sizeof(fs_info->qgroup_rescan_progress));
	fs_info->qgroup_rescan_progress.objectid = progress_objectid;
	init_completion(&fs_info->qgroup_rescan_completion);
	fs_info->qgroup_rescan_running = true;

	spin_unlock(&fs_info->qgroup_lock);
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	memset(&fs_info->qgroup_rescan_work, 0,
	       sizeof(fs_info->qgroup_rescan_work));
	btrfs_init_work(&fs_info->qgroup_rescan_work,
			btrfs_qgroup_rescan_helper,
			btrfs_qgroup_rescan_worker, NULL, NULL);

	if (ret) {
err:
		btrfs_info(fs_info, "qgroup_rescan_init failed with %d", ret);
		return ret;
	}

	return 0;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
// Copied from qgroup_rescan_init.
static int syno_quota_rescan_init(struct btrfs_fs_info *fs_info, u64 progress_objectid,
		   int init_flags)
{
	int ret = 0;

	if (!init_flags) {
		/* we're resuming qgroup rescan at mount time */
		if (!(fs_info->qgroup_flags &
		      BTRFS_QGROUP_STATUS_FLAG_RESCAN)) {
			btrfs_warn(fs_info,
			"qgroup rescan init failed, qgroup rescan is not queued");
			ret = -EINVAL;
		} else if (!(fs_info->qgroup_flags &
			     BTRFS_QGROUP_STATUS_FLAG_ON)) {
			btrfs_warn(fs_info,
			"qgroup rescan init failed, qgroup is not enabled");
			ret = -EINVAL;
		}

		if (ret)
			return ret;
	}

	mutex_lock(&fs_info->qgroup_rescan_lock);

	if (init_flags) {
		if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN)
			ret = -EINPROGRESS;
		else if (!(fs_info->qgroup_flags &
			     BTRFS_QGROUP_STATUS_FLAG_ON)) {
			btrfs_warn(fs_info,
				"qgroup rescan init failed, qgroup is not enabled");
			ret = -ESRCH;
		} else if (!fs_info->syno_quota_v2_enabled)
			ret = -ESRCH;

		if (!ret && !fs_info->syno_quota_rescan_ctx) {
			fs_info->syno_quota_rescan_ctx =
				kzalloc(sizeof(struct syno_quota_rescan_ctx), GFP_KERNEL);
			if (!fs_info->syno_quota_rescan_ctx)
				ret = -ENOMEM;
		}

		if (!ret && !fs_info->syno_quota_rescan_subvol_ulist) {
			fs_info->syno_quota_rescan_subvol_ulist = ulist_alloc(GFP_KERNEL);
			if (!fs_info->syno_quota_rescan_subvol_ulist) {
				kfree(fs_info->syno_quota_rescan_ctx);
				fs_info->syno_quota_rescan_ctx = NULL;
				ret = -ENOMEM;
			}
		}

		if (ret) {
			mutex_unlock(&fs_info->qgroup_rescan_lock);
			return ret;
		}
		fs_info->qgroup_flags |= BTRFS_QGROUP_STATUS_FLAG_RESCAN;
	}

	memset(&fs_info->qgroup_rescan_progress, 0,
		sizeof(fs_info->qgroup_rescan_progress));
	fs_info->qgroup_rescan_progress.objectid = progress_objectid;
	init_completion(&fs_info->qgroup_rescan_completion);

	fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_PAUSE;
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	btrfs_init_work(&fs_info->qgroup_rescan_work,
			btrfs_qgroup_rescan_helper,
			btrfs_syno_quota_rescan_worker, NULL, NULL);
	return 0;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
#else
static void
qgroup_rescan_zero_tracking(struct btrfs_fs_info *fs_info)
{
	struct rb_node *n;
	struct btrfs_qgroup *qgroup;

	spin_lock(&fs_info->qgroup_lock);
	/* clear all current qgroup tracking information */
	for (n = rb_first(&fs_info->qgroup_tree); n; n = rb_next(n)) {
		qgroup = rb_entry(n, struct btrfs_qgroup, node);
		qgroup->rfer = 0;
		qgroup->rfer_cmpr = 0;
		qgroup->excl = 0;
		qgroup->excl_cmpr = 0;
		qgroup_dirty(fs_info, qgroup);
	}
	spin_unlock(&fs_info->qgroup_lock);
}

int
btrfs_qgroup_rescan(struct btrfs_fs_info *fs_info)
{
	int ret = 0;
	struct btrfs_trans_handle *trans;

	ret = qgroup_rescan_init(fs_info, 0, 1);
	if (ret)
		return ret;

	/*
	 * We have set the rescan_progress to 0, which means no more
	 * delayed refs will be accounted by btrfs_qgroup_account_ref.
	 * However, btrfs_qgroup_account_ref may be right after its call
	 * to btrfs_find_all_roots, in which case it would still do the
	 * accounting.
	 * To solve this, we're committing the transaction, which will
	 * ensure we run all delayed refs and only after that, we are
	 * going to clear all tracking information for a clean start.
	 */

	trans = btrfs_join_transaction(fs_info->fs_root);
	if (IS_ERR(trans)) {
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_RESCAN;
		return PTR_ERR(trans);
	}
	ret = btrfs_commit_transaction(trans, fs_info->fs_root);
	if (ret) {
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_RESCAN;
		return ret;
	}

	qgroup_rescan_zero_tracking(fs_info);

	btrfs_queue_work(fs_info->qgroup_rescan_workers,
			 &fs_info->qgroup_rescan_work);

	return 0;
}
#endif /* MY_ABC_HERE */

int btrfs_qgroup_wait_for_completion(struct btrfs_fs_info *fs_info,
				     bool interruptible)
{
	int running;
	int ret = 0;

	mutex_lock(&fs_info->qgroup_rescan_lock);
	spin_lock(&fs_info->qgroup_lock);
	running = fs_info->qgroup_rescan_running;
	spin_unlock(&fs_info->qgroup_lock);
	mutex_unlock(&fs_info->qgroup_rescan_lock);

	if (!running)
		return 0;

	if (interruptible)
		ret = wait_for_completion_interruptible(
					&fs_info->qgroup_rescan_completion);
	else
		wait_for_completion(&fs_info->qgroup_rescan_completion);

	return ret;
}

/*
 * this is only called from open_ctree where we're still single threaded, thus
 * locking is omitted here.
 */
void
btrfs_qgroup_rescan_resume(struct btrfs_fs_info *fs_info)
{
	if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN) {
#ifdef MY_ABC_HERE
		fs_info->qgroup_rescan_running = true;
		fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_PAUSE;
#endif /* MY_ABC_HERE */
		btrfs_queue_work(fs_info->qgroup_rescan_workers,
				 &fs_info->qgroup_rescan_work);
	}
}

#ifdef MY_ABC_HERE
int btrfs_reset_qgroup_status(struct btrfs_trans_handle *trans, struct btrfs_fs_info *fs_info)
{
	struct btrfs_root *quota_root = fs_info->quota_root;
	struct btrfs_path *path = NULL;
	struct btrfs_qgroup_status_item *ptr;
	struct extent_buffer *leaf;
	struct btrfs_key key;
	int ret;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (!fs_info->quota_root) {
		ret = -ENOENT;
		goto out;
	}

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	key.objectid = 0;
	key.type = BTRFS_QGROUP_STATUS_KEY;
	key.offset = 0;

	ret = btrfs_search_slot(trans, quota_root, &key, path, 0, 1);
	if (ret)
		goto out;

	leaf = path->nodes[0];
	ptr = btrfs_item_ptr(leaf, path->slots[0],
				 struct btrfs_qgroup_status_item);
	fs_info->qgroup_flags &= ~(BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT);
	btrfs_set_qgroup_status_flags(leaf, ptr, fs_info->qgroup_flags);
	btrfs_set_qgroup_status_generation(leaf, ptr, trans->transid);
	btrfs_set_qgroup_status_version(leaf, ptr, BTRFS_QGROUP_V2_STATUS_VERSION);
	btrfs_mark_buffer_dirty(leaf);

out:
	btrfs_free_path(path);
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}

int btrfs_syno_qgroup_transfer_limit(struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_root *old_root = NULL;
	struct btrfs_key key;
	struct btrfs_key found_key;
	struct btrfs_path *path = NULL;
	struct extent_buffer *leaf;
	struct btrfs_qgroup_limit_item *ptr;
	struct btrfs_qgroup *qgroup;
	int ret = 0;
	int slot;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (!fs_info->quota_root) {
		ret = -ESRCH;
		goto out;
	}

	if (!fs_info->syno_quota_v2_enabled) {
		ret = -ESRCH;
		goto out;
	}

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	key.objectid = BTRFS_QUOTA_TREE_OBJECTID;
	key.type = BTRFS_ROOT_ITEM_KEY;
	key.offset = 0;
	old_root = btrfs_read_tree_root(fs_info->tree_root, &key);
	if (IS_ERR(old_root)) {
		ret = PTR_ERR(old_root);
		old_root = NULL;
		goto out;
	}

	key.objectid = 0;
	key.type = BTRFS_QGROUP_LIMIT_KEY;
	key.offset = 0;
	ret = btrfs_search_slot_for_read(old_root, &key, path, 1, 0);
	if (ret)
		goto out;

	while (1) {
		slot = path->slots[0];
		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &found_key, slot);

		if (found_key.type > BTRFS_QGROUP_LIMIT_KEY)
			break;

		if (found_key.type == BTRFS_QGROUP_LIMIT_KEY) {
			ptr = btrfs_item_ptr(leaf, slot,
					     struct btrfs_qgroup_limit_item);

			spin_lock(&fs_info->qgroup_lock);
			qgroup = find_qgroup_rb(fs_info, found_key.offset);
			if (qgroup && qgroup->lim_flags == 0
					&& qgroup->max_rfer == 0 && qgroup->max_excl == 0
					&& qgroup->rsv_rfer == 0 && qgroup->rsv_excl == 0) {
				qgroup->lim_flags = btrfs_qgroup_limit_flags(leaf, ptr);
				qgroup->max_rfer = btrfs_qgroup_limit_max_rfer(leaf, ptr);
				qgroup->max_excl = btrfs_qgroup_limit_max_excl(leaf, ptr);
				qgroup->rsv_rfer = btrfs_qgroup_limit_rsv_rfer(leaf, ptr);
				qgroup->rsv_excl = btrfs_qgroup_limit_rsv_excl(leaf, ptr);
				qgroup_dirty(fs_info, qgroup);
			}
			spin_unlock(&fs_info->qgroup_lock);
		}

		ret = btrfs_next_item(old_root, path);
		if (ret)
			break;
	}

out:
	btrfs_free_path(path);
	if (old_root) {
		free_extent_buffer(old_root->node);
		free_extent_buffer(old_root->commit_root);
#ifdef MY_ABC_HERE
		btrfs_free_root_eb_monitor(old_root);
#endif /* MY_ABC_HERE */
		kfree(old_root);
	}
	mutex_unlock(&fs_info->qgroup_ioctl_lock);

	if (ret > 0)
		ret = 0;
	return ret;
}

// We may have no qgroup record in volume migration case.
static void qgroup_zero_tracking(struct btrfs_fs_info *fs_info, u64 subvol_id)
{
	struct btrfs_qgroup *qgroup;

	spin_lock(&fs_info->qgroup_lock);
	qgroup = find_qgroup_rb(fs_info, subvol_id);
	if (qgroup) {
		qgroup->rfer = 0;
		qgroup->rfer_cmpr = 0;
		qgroup->excl = 0;
		qgroup->excl_cmpr = 0;
		qgroup_dirty(fs_info, qgroup);
	}
	spin_unlock(&fs_info->qgroup_lock);
}

int btrfs_syno_quota_rescan(struct btrfs_root *root)
{
	int ret = 0;
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct syno_quota_rescan_ctx *ctx;
	struct ulist *ulist;
	struct ulist_node *node;
	struct btrfs_trans_handle *trans;
	struct syno_quota_rescan_item_updater updater;
	u64 subvol_id = root->root_key.objectid;
	u64 prev_subvol_id = 0;

	trans = btrfs_start_transaction(root, 2);
	if (IS_ERR(trans)) {
		ret = PTR_ERR(trans);
		return ret;
	}

	ret = syno_quota_rescan_init(fs_info, 0, 1);
	if (ret && ret != -EINPROGRESS) {
		btrfs_end_transaction(trans, root);
		return ret;
	}

	// Take qgroup_ioctl_lock after we start the transaction. See comments in btrfs_quota_enable().
	mutex_lock(&fs_info->qgroup_ioctl_lock);
	mutex_lock(&fs_info->qgroup_rescan_lock);
	ctx = fs_info->syno_quota_rescan_ctx;
	ulist = fs_info->syno_quota_rescan_subvol_ulist;
	if (!list_empty(&ulist->nodes)) {
		node = list_entry(ulist->nodes.prev, struct ulist_node, list);
		prev_subvol_id = node->val;
	}
	ret = ulist_add(ulist, subvol_id, 0, GFP_KERNEL);
	if (ret != 1) {
		if (ret == 0)
			ret = -EEXIST;
		else
			ret = -ENOMEM;
		goto out;
	}

	if (prev_subvol_id) {
		syno_quota_rescan_item_init(&updater);
		updater.next_root = subvol_id;
		ret = btrfs_add_update_syno_quota_rescan_item(trans, fs_info->quota_root,
									prev_subvol_id, &updater);
		if (ret)
			goto out;
	}

	/*
	 * Step 1. Update rescan item. This may fail so it must be the first step.
	 * Step 2. Set root->rescan_inode, so existing inodes won't do quota accounting until
	 *         they are scanned.
	 * Step 3. Zero quota.
	 * Step 4. Set root->rescan_end_inode, so new inode will do normal quota accounting.
	 */
	mutex_lock(&root->objectid_mutex);
	syno_quota_rescan_item_init(&updater);
	updater.flags = SYNO_QUOTA_RESCAN_QUEUED;
	updater.version = BTRFS_QGROUP_V2_STATUS_VERSION;
	updater.rescan_inode = 0;
	updater.end_inode = root->highest_objectid;
	updater.tree_size = btrfs_root_used(&root->root_item);
	updater.next_root = 0;
	ret = btrfs_add_update_syno_quota_rescan_item(trans, fs_info->quota_root, subvol_id, &updater);
	if (ret) {
		mutex_unlock(&root->objectid_mutex);
		goto out; // prev_subvol_id will point to a invalid rescan item, but it's OK, no need to abort.
	}

	if (!fs_info->qgroup_rescan_progress.objectid)
		fs_info->qgroup_rescan_progress.objectid = subvol_id;

	root->rescan_inode = 0;
	smp_wmb();
	qgroup_zero_tracking(fs_info, subvol_id);
	btrfs_usrquota_zero_tracking(fs_info, subvol_id);
	root->rescan_end_inode = root->highest_objectid;
	root->invalid_quota = false;
	mutex_unlock(&root->objectid_mutex);

	btrfs_end_transaction(trans, root);
	trans = NULL;

	// Update progress info.
	update_syno_quota_rescan_progress(fs_info->quota_root, ctx,
		subvol_id, SYNO_QUOTA_PROGRESS_ADD_NEW);

	if (!fs_info->qgroup_rescan_running) {
		fs_info->qgroup_rescan_running = true;
		btrfs_queue_work(fs_info->qgroup_rescan_workers,
			 &fs_info->qgroup_rescan_work);
	}
	ret = 0;

out:
	if (trans)
		btrfs_end_transaction(trans, root);
	if (ret && ret != -EEXIST) {
		ulist_del(ulist, subvol_id, 0);
		if (list_empty(&ulist->nodes))
			fs_info->qgroup_flags &= ~BTRFS_QGROUP_STATUS_FLAG_RESCAN;
	}
	mutex_unlock(&fs_info->qgroup_rescan_lock);
	mutex_unlock(&fs_info->qgroup_ioctl_lock);

	/*
	 * We want everything is on-disk.
	 * But we can't commit the transaction with qgroup_rescan_lock, or we deadlock with rescan_worker.
	 */
	if (!ret) {
		trans = btrfs_join_transaction(root);
		if (!IS_ERR(trans))
			btrfs_commit_transaction(trans, root);
	}

	return ret;
}

static int syno_quota_rescan_progress(struct btrfs_root *root,
			struct btrfs_ioctl_syno_quota_status_args *sa,
			bool query_vol_progress, bool query_subvol_progress)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_key key;
	struct syno_quota_rescan_ctx *ctx;
	struct ulist *ulist;
	struct ulist_node *node;
	struct btrfs_path *path = NULL;
	struct btrfs_root *scanning_root = NULL;
	u64 subvol_id = root->root_key.objectid;
	int ret;

	mutex_lock(&fs_info->qgroup_rescan_lock);

	// Update flags for rescan status.
	ctx = fs_info->syno_quota_rescan_ctx;
	ulist = fs_info->syno_quota_rescan_subvol_ulist;
	if (!ctx || !ulist || list_empty(&ulist->nodes) ||
			!(fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_RESCAN)) {
		sa->progress = 0;
		sa->next_subvol_id = 0;
		sa->scanning_subvol_id = 0;
		ret = 0;
		goto out;
	} else if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_PAUSE)
		sa->status |= BTRFS_QUOTA_STATUS_VOL_RESCAN_PAUSED;
	else if (fs_info->qgroup_rescan_running)
		sa->status |= BTRFS_QUOTA_STATUS_VOL_RESCAN_DOING;
	else if (rescan_should_stop(fs_info)) {
		ret = -ECANCELED;
		goto out;
	} else {
		btrfs_warn_rl(fs_info, "Unexpected state in syno_quota_rescan_progress() but no harm");
		ret = -EINVAL;
		goto out;
	}

	// Update current scanning subvol.
	sa->scanning_subvol_id = ctx->subvol_id;

	// Update next subvol id.
	node = ulist_search(ulist, subvol_id);
	if (node) {
		if (subvol_id == ctx->subvol_id)
			sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_RESCANNING;
		else
			sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_RESCAN_QUEUED;

		if (node->list.next != &ulist->nodes) {
			node = list_entry(node->list.next, struct ulist_node, list);
			sa->next_subvol_id = node->val;
		}
	} else
		sa->next_subvol_id = 0;

	/*
	 * Query volume progress, or query subvol progress that is scanning.
	 * In both case we need to update ctx->subvol_id progress.
	 */
	if (query_vol_progress || (query_subvol_progress && ctx->subvol_id == subvol_id)) {
		u64 subvol_progress;
		u64 vol_progress;
		u64 denominator = 0;
		u64 numerator = 0;
		u64 tmp;
		int srcu_index;
		int i;

		path = btrfs_alloc_path();
		if (!path) {
			ret = -ENOMEM;
			goto out;
		}

		key.objectid = ctx->subvol_id;
		key.type = BTRFS_ROOT_ITEM_KEY;
		key.offset = (u64)-1;

		srcu_index = srcu_read_lock(&fs_info->subvol_srcu);
		scanning_root = btrfs_read_fs_root_no_name(fs_info, &key);
		if (IS_ERR(scanning_root)) {
			ret = PTR_ERR(scanning_root);
			scanning_root = NULL;
			srcu_read_unlock(&fs_info->subvol_srcu, srcu_index);
			goto out;
		}
		btrfs_hold_fs_root(scanning_root);
		srcu_read_unlock(&fs_info->subvol_srcu, srcu_index);

		key.objectid = scanning_root->rescan_end_inode;
		key.type = BTRFS_INODE_ITEM_KEY;
		key.offset = 0;

		ret = btrfs_search_slot(NULL, scanning_root, &key, path, 0, 0);
		if (ret < 0)
			goto out;

		syno_quota_update_progress_path(ctx->end_path, path);
		btrfs_release_path(path);

		// Calaulate subvol denominator.
		tmp = SYNO_QUOTA_RESCAN_100_PROGRESS;
		for (i = BTRFS_MAX_LEVEL - 1; i > 0; i--) {
			if (ctx->end_path[i][0]) {
				tmp /= ctx->end_path[i][0];
				denominator += (tmp * ctx->end_path[i][1]);
			}
		}

		// Calaulate subvol numerator.
		tmp = SYNO_QUOTA_RESCAN_100_PROGRESS;
		for (i = BTRFS_MAX_LEVEL - 1; i > 0; i--) {
			if (ctx->current_path[i][0]) {
				tmp /= ctx->current_path[i][0];
				numerator += (tmp * ctx->current_path[i][1]);
			}
		}

		if (denominator == 0)
			denominator = 1;
		if (numerator > denominator)
			numerator = denominator;

		subvol_progress = (numerator * SYNO_QUOTA_RESCAN_100_PROGRESS) / denominator;
		if (subvol_progress < ctx->subvol_progress)
			subvol_progress = ctx->subvol_progress;
		ctx->subvol_progress = subvol_progress;

		if (query_subvol_progress) { // Report subvol progress.
			sa->progress = subvol_progress;
			sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_PROGRESS_VALID;
		} else { // Report vol progress.
			denominator = ctx->total_size;
			numerator = ctx->total_finished_size +
				(ctx->subvol_size * subvol_progress / SYNO_QUOTA_RESCAN_100_PROGRESS);

			if (numerator > denominator) {
				numerator = denominator;
				WARN_ON_ONCE(1);
			}

			vol_progress = (numerator * SYNO_QUOTA_RESCAN_100_PROGRESS) / denominator;
			if (vol_progress < ctx->vol_progress)
				vol_progress = ctx->vol_progress;
			ctx->vol_progress = vol_progress;
			sa->progress = vol_progress;
			sa->status |= BTRFS_QUOTA_STATUS_VOL_PROGRESS_VALID;
		}
	} else if (query_subvol_progress) { // Query subvol progress that is not currently scanning.
		struct btrfs_syno_quota_rescan_item rescan_item;

		ret = btrfs_read_syno_quota_rescan_item(fs_info->quota_root, subvol_id, &rescan_item);
		if (ret)
			goto out;

		if (rescan_item.flags & SYNO_QUOTA_RESCAN_DONE) {
			sa->progress = SYNO_QUOTA_RESCAN_100_PROGRESS;
			sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_PROGRESS_VALID;
		} else if (rescan_item.flags & SYNO_QUOTA_RESCAN_QUEUED) {
			sa->progress = 0;
			sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_PROGRESS_VALID;
		} else {
			WARN_ON_ONCE(1);
			ret = -EINVAL;
			goto out;
		}
	}

	ret = 0;
out:
	if (scanning_root)
		btrfs_release_fs_root(scanning_root);
	btrfs_free_path(path);
	mutex_unlock(&fs_info->qgroup_rescan_lock);
	return ret;
}

int btrfs_syno_quota_status(struct btrfs_root *root,
			struct btrfs_ioctl_syno_quota_status_args *sa)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	int ret = 0;
	bool query_vol_progress = false;
	bool query_subvol_progress = false;

	if (sa->cmd & BTRFS_QUOTA_STATUS_RESCAN_VOL_PROGRESS)
		query_vol_progress = true;
	if (sa->cmd & BTRFS_QUOTA_STATUS_RESCAN_SUBVOL_PROGRESS)
		query_subvol_progress = true;

	// We'll return sa to user, so zero it first.
	memset(sa, 0, sizeof(*sa));

	if (query_vol_progress && query_subvol_progress)
		return -EINVAL;

	mutex_lock(&fs_info->qgroup_ioctl_lock);
	if (!fs_info->quota_root) {
		sa->status |= BTRFS_QUOTA_STATUS_VOL_DISABLED;
		sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_DISABLED;
		goto out;
	}

	if (fs_info->syno_quota_v1_enabled)
		sa->status |= BTRFS_QUOTA_STATUS_VOL_SYNO_V1_ENABLED;
	else if (fs_info->syno_quota_v2_enabled)
		sa->status |= BTRFS_QUOTA_STATUS_VOL_SYNO_V2_ENABLED;
	else {
		sa->status |= BTRFS_QUOTA_STATUS_VOL_DISABLED;
		sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_DISABLED;
		goto out;
	}

#ifdef MY_ABC_HERE
	if (root->invalid_quota)
		sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_DISABLED;
	else
#else
		sa->status |= BTRFS_QUOTA_STATUS_SUBVOL_ENABLED;
#endif /* MY_ABC_HERE */

	if (fs_info->qgroup_flags & BTRFS_QGROUP_STATUS_FLAG_INCONSISTENT)
		sa->status |= BTRFS_QUOTA_STATUS_INCONSISTENT;
	if (fs_info->usrquota_flags & BTRFS_USRQUOTA_STATUS_FLAG_INCONSISTENT)
		sa->status |= BTRFS_USRQUOTA_STATUS_INCONSISTENT;

	ret = syno_quota_rescan_progress(root, sa,
				query_vol_progress, query_subvol_progress);

out:
	mutex_unlock(&fs_info->qgroup_ioctl_lock);
	return ret;
}

void btrfs_read_syno_quota_for_root(struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_syno_quota_rescan_item rescan_item;
	int ret;

#ifdef MY_ABC_HERE
	btrfs_check_usrquota_limit(root);
	btrfs_check_quota_limit(root);
#endif /* MY_ABC_HERE */

	// only quota v2 care about subvol quota version.
	if (!fs_info->syno_quota_v2_enabled) {
		root->invalid_quota = false;
		return;
	}

	ret = btrfs_read_syno_quota_rescan_item(fs_info->quota_root,
							root->root_key.objectid, &rescan_item);
	if (ret) {
		btrfs_info(fs_info, "Failed to read syno quota for root %llu, ret = %d",
							root->root_key.objectid, ret);
		return;
	}

	root->rescan_inode = rescan_item.rescan_inode;
	root->rescan_end_inode = rescan_item.end_inode;
	if (rescan_item.version == BTRFS_QGROUP_V2_STATUS_VERSION)
		root->invalid_quota = false;

	return;
}
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
int btrfs_quota_reserve(struct btrfs_root *root, struct inode *inode, u64 num_bytes)
{
	int ret = 0;

#ifdef MY_ABC_HERE
	if (btrfs_root_disable_quota(root))
		return 0;
#endif /* MY_ABC_HERE */

	if (!is_fstree(root->objectid) ||
#ifdef MY_ABC_HERE
		(!root->fs_info->syno_quota_v1_enabled && !root->fs_info->syno_quota_v2_enabled) ||
#endif /* MY_ABC_HERE */
		num_bytes == 0)
		return 0;

#ifdef MY_ABC_HERE
	num_bytes = round_up(num_bytes, root->fs_info->sectorsize);
#endif /* MY_ABC_HERE */

	ret = qgroup_reserve(root, num_bytes);
	if (ret < 0)
		return ret;
#ifdef MY_ABC_HERE
	ret = btrfs_usrquota_reserve (root, inode, i_uid_read(inode), num_bytes);
	if (ret < 0) {
		qgroup_free(root, num_bytes);
		return ret;
	}
#endif /* MY_ABC_HERE */
	return 0;
}
void btrfs_quota_reserve_free(struct btrfs_root *root, struct inode *inode, u64 num_bytes)
{
#ifdef MY_ABC_HERE
        num_bytes = round_up(num_bytes, root->fs_info->sectorsize);
#endif /* MY_ABC_HERE */

	qgroup_free(root, num_bytes);
#ifdef MY_ABC_HERE
	btrfs_usrquota_free(root, inode, i_uid_read(inode), num_bytes);
#endif /* MY_ABC_HERE */
}
#endif /* MY_ABC_HERE */
/*
 * Reserve qgroup space for range [start, start + len).
 *
 * This function will either reserve space from related qgroups or doing
 * nothing if the range is already reserved.
 *
 * Return 0 for successful reserve
 * Return <0 for error (including -EQUOT)
 *
 * NOTE: this function may sleep for memory allocation.
 */
int btrfs_qgroup_reserve_data(struct inode *inode, u64 start, u64 len)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct extent_changeset changeset;
	struct ulist_node *unode;
	struct ulist_iterator uiter;
	int ret;

#ifdef MY_ABC_HERE
	if (btrfs_root_disable_quota(root))
		return 0;
#endif /* MY_ABC_HERE */

#ifdef MY_ABC_HERE
	if ((!root->fs_info->syno_quota_v1_enabled && !root->fs_info->syno_quota_v2_enabled) ||
		!is_fstree(root->objectid) ||
		len == 0)
#else
	if (!root->fs_info->quota_enabled || !is_fstree(root->objectid) ||
		len == 0)
#endif /* MY_ABC_HERE */
		return 0;

#ifdef MY_ABC_HERE
	if (!btrfs_root_has_usrquota_limit(root) && !btrfs_root_has_quota_limit(root))
		return 0;
#endif /* MY_ABC_HERE */

	changeset.bytes_changed = 0;
	changeset.range_changed = ulist_alloc(GFP_NOFS);
#ifdef MY_ABC_HERE
	changeset.prealloc_ulist_node = NULL;
#endif /* MY_ABC_HERE */
	ret = set_record_extent_bits(&BTRFS_I(inode)->io_tree, start,
			start + len -1, EXTENT_QGROUP_RESERVED, &changeset);
	trace_btrfs_qgroup_reserve_data(inode, start, len,
					changeset.bytes_changed,
					QGROUP_RESERVE);
	if (ret < 0)
		goto cleanup;
	ret = qgroup_reserve(root, changeset.bytes_changed);
#ifdef MY_ABC_HERE
	if (ret != 0) {
		if (ret > 0)
			ret = 0;
		goto cleanup;
	}
#else
	if (ret < 0)
		goto cleanup;
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	ret = btrfs_usrquota_reserve(root, inode, i_uid_read(inode), changeset.bytes_changed);
	if (ret != 0) {
		if (ret > 0)
			ret = 0;
		goto qgroup_cleanup;
	}
#endif /* MY_ABC_HERE */

	ulist_free(changeset.range_changed);
	return ret;

#ifdef MY_ABC_HERE
qgroup_cleanup:
	qgroup_free(root, changeset.bytes_changed);
#endif /* MY_ABC_HERE */
cleanup:
	/* cleanup already reserved ranges */
	ULIST_ITER_INIT(&uiter);
	while ((unode = ulist_next(changeset.range_changed, &uiter)))
		clear_extent_bit(&BTRFS_I(inode)->io_tree, unode->val,
				 unode->aux, EXTENT_QGROUP_RESERVED, 0, 0, NULL,
				 GFP_NOFS);
	ulist_free(changeset.range_changed);
	return ret;
}

static int __btrfs_qgroup_release_data(struct inode *inode, u64 start, u64 len,
				       int free)
{
	struct extent_changeset changeset;
	int trace_op = QGROUP_RELEASE;
	int ret;

#ifdef MY_ABC_HERE
	struct btrfs_fs_info *fs_info = BTRFS_I(inode)->root->fs_info;

        if (!fs_info->syno_quota_v1_enabled && !fs_info->syno_quota_v2_enabled) {
		if (fs_info->need_clear_reserve) {
			clear_extent_bit(&BTRFS_I(inode)->io_tree, start, start + len -1,
				       EXTENT_QGROUP_RESERVED, 0, 0, NULL, GFP_NOFS);
			spin_lock(&fs_info->usrquota_lock);
			BTRFS_I(inode)->uq_reserved = 0;
			spin_unlock(&fs_info->usrquota_lock);
		}
		return 0;
	}
#endif /* MY_ABC_HERE */

	changeset.bytes_changed = 0;
	changeset.range_changed = ulist_alloc(GFP_NOFS);
#ifdef MY_ABC_HERE
	changeset.prealloc_ulist_node = NULL;
#endif /* MY_ABC_HERE */
	if (!changeset.range_changed)
		return -ENOMEM;

	ret = clear_record_extent_bits(&BTRFS_I(inode)->io_tree, start, 
			start + len -1, EXTENT_QGROUP_RESERVED, &changeset);
	if (ret < 0)
		goto out;

	if (free) {
		qgroup_free(BTRFS_I(inode)->root, changeset.bytes_changed);
#ifdef MY_ABC_HERE
		btrfs_usrquota_free(BTRFS_I(inode)->root, inode, i_uid_read(inode),
					changeset.bytes_changed);
#endif /* MY_ABC_HERE */
		trace_op = QGROUP_FREE;
	}
	trace_btrfs_qgroup_release_data(inode, start, len,
					changeset.bytes_changed, trace_op);
	ret = changeset.bytes_changed;
out:
	ulist_free(changeset.range_changed);
	return ret;
}

/*
 * Free a reserved space range from io_tree and related qgroups
 *
 * Should be called when a range of pages get invalidated before reaching disk.
 * Or for error cleanup case.
 *
 * For data written to disk, use btrfs_qgroup_release_data().
 *
 * NOTE: This function may sleep for memory allocation.
 */
int btrfs_qgroup_free_data(struct inode *inode, u64 start, u64 len)
{
	return __btrfs_qgroup_release_data(inode, start, len, 1);
}

/*
 * Release a reserved space range from io_tree only.
 *
 * Should be called when a range of pages get written to disk and corresponding
 * FILE_EXTENT is inserted into corresponding root.
 *
 * Since new qgroup accounting framework will only update qgroup numbers at
 * commit_transaction() time, its reserved space shouldn't be freed from
 * related qgroups.
 *
 * But we should release the range from io_tree, to allow further write to be
 * COWed.
 *
 * NOTE: This function may sleep for memory allocation.
 */
int btrfs_qgroup_release_data(struct inode *inode, u64 start, u64 len)
{
	return __btrfs_qgroup_release_data(inode, start, len, 0);
}

int btrfs_qgroup_reserve_meta(struct btrfs_root *root, int num_bytes)
{
#ifdef MY_ABC_HERE
	return 0;
#else
	int ret;

	if (!root->fs_info->quota_enabled || !is_fstree(root->objectid) ||
	    num_bytes == 0)
		return 0;

	BUG_ON(num_bytes != round_down(num_bytes, root->nodesize));
	ret = qgroup_reserve(root, num_bytes);
	if (ret < 0)
		return ret;
	atomic_add(num_bytes, &root->qgroup_meta_rsv);
	return ret;
#endif /* MY_ABC_HERE */
}

void btrfs_qgroup_free_meta_all(struct btrfs_root *root)
{
#ifdef MY_ABC_HERE
	return;
#else
	int reserved;

	if (!root->fs_info->quota_enabled || !is_fstree(root->objectid))
		return;

	reserved = atomic_xchg(&root->qgroup_meta_rsv, 0);
	if (reserved == 0)
		return;
	qgroup_free(root, reserved);
#endif /* MY_ABC_HERE */
}

void btrfs_qgroup_free_meta(struct btrfs_root *root, int num_bytes)
{
#ifdef MY_ABC_HERE
	return;
#else
	if (!root->fs_info->quota_enabled || !is_fstree(root->objectid))
		return;

	BUG_ON(num_bytes != round_down(num_bytes, root->nodesize));
	WARN_ON(atomic_read(&root->qgroup_meta_rsv) < num_bytes);
	atomic_sub(num_bytes, &root->qgroup_meta_rsv);
	qgroup_free(root, num_bytes);
#endif /* MY_ABC_HERE */
}

/*
 * Check qgroup reserved space leaking, normally at destroy inode
 * time
 */
void btrfs_qgroup_check_reserved_leak(struct inode *inode)
{
	struct extent_changeset changeset;
	struct ulist_node *unode;
	struct ulist_iterator iter;
	int ret;

	changeset.bytes_changed = 0;
	changeset.range_changed = ulist_alloc(GFP_NOFS);
#ifdef MY_ABC_HERE
	changeset.prealloc_ulist_node = NULL;
#endif /* MY_ABC_HERE */
	if (WARN_ON(!changeset.range_changed))
		return;

	ret = clear_record_extent_bits(&BTRFS_I(inode)->io_tree, 0, (u64)-1,
			EXTENT_QGROUP_RESERVED, &changeset);

	WARN_ON(ret < 0);
	if (WARN_ON(changeset.bytes_changed)) {
		ULIST_ITER_INIT(&iter);
		while ((unode = ulist_next(changeset.range_changed, &iter))) {
			btrfs_warn(BTRFS_I(inode)->root->fs_info,
				"leaking qgroup reserved space, ino: %lu, start: %llu, end: %llu",
				inode->i_ino, unode->val, unode->aux);
		}
		qgroup_free(BTRFS_I(inode)->root, changeset.bytes_changed);
	}
	ulist_free(changeset.range_changed);
}

#ifdef MY_ABC_HERE
static bool check_quota_from_disk(struct btrfs_fs_info *fs_info, u64 qgroupid)
{
	int ret;
	int slot;
	struct btrfs_key key;
	struct btrfs_key found_key;
	struct btrfs_root *quota_root = fs_info->quota_root;
	struct btrfs_path *path;
	struct extent_buffer *leaf;
	struct btrfs_qgroup_limit_item *limit_item;
	bool has_limit = false;
	u64 flags;
	u64 max_rfer;
	u64 max_excl;

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	key.objectid = 0;
	key.type = BTRFS_QGROUP_LIMIT_KEY;
	key.offset = qgroupid;
	ret = btrfs_search_slot_for_read(quota_root, &key, path, 1, 0);
	if (ret < 0)
		goto out;
	else if (ret) {
		ret = 0;
		goto out;
	}

	slot = path->slots[0];
	leaf = path->nodes[0];
	btrfs_item_key_to_cpu(leaf, &found_key, slot);
	if (found_key.offset != qgroupid ||
	    found_key.type != BTRFS_QGROUP_LIMIT_KEY) {
		ret = 0;
		goto out;
	}

	limit_item = btrfs_item_ptr(leaf, slot,
				struct btrfs_qgroup_limit_item);
	flags = btrfs_qgroup_limit_flags(leaf, limit_item);
	max_rfer = btrfs_qgroup_limit_max_rfer(leaf, limit_item);
	max_excl = btrfs_qgroup_limit_max_excl(leaf, limit_item);
	if ((flags & BTRFS_QGROUP_LIMIT_MAX_RFER && max_rfer) ||
	    (flags & BTRFS_QGROUP_LIMIT_MAX_EXCL && max_excl))
		has_limit = true;
	ret = 0;
out:
	btrfs_free_path(path);
	// When an error occurr, we always treat it as having quota_limt.
	return (ret) ? true : has_limit;
}

void btrfs_check_quota_limit(struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	bool has_limit = false;
	u64 qgroupid = root->root_key.objectid;
	struct btrfs_qgroup *qgroup;

	spin_lock(&fs_info->qgroup_lock);
	if (!fs_info->quota_root) {
		spin_unlock(&fs_info->qgroup_lock);
		return;
	}

	qgroup = find_qgroup_rb(fs_info, qgroupid);
	if (!qgroup) {
		// subtree is unloaded, read from disk.
		spin_unlock(&fs_info->qgroup_lock);
		has_limit = check_quota_from_disk(fs_info, qgroupid);
	} else {
		if ((qgroup->lim_flags & BTRFS_QGROUP_LIMIT_MAX_RFER && qgroup->max_rfer) ||
		    (qgroup->lim_flags & BTRFS_QGROUP_LIMIT_MAX_EXCL && qgroup->max_excl))
			has_limit = true;
		spin_unlock(&fs_info->qgroup_lock);
	}
	btrfs_root_set_has_quota_limit(root, has_limit);
}
#endif /* MY_ABC_HERE */
