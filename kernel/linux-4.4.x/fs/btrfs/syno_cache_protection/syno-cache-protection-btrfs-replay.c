#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Copyright (C) 2019 Synology Inc.  All rights reserved.
 */

#include <linux/mm.h>
#include <linux/namei.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/task_work.h>
#include <linux/utime.h>
#include <linux/kallsyms.h>
#include <linux/syscalls.h>
#include <linux/xattr.h>
#ifdef MY_ABC_HERE
#include <linux/syno_acl.h>
#include "../syno_acl.h"
#endif /* MY_ABC_HERE */
#include "../ctree.h"
#include "../disk-io.h"
#include "../transaction.h"
#include "../print-tree.h"
#include "../volumes.h"
#include <linux/syno_cache_protection.h>
#include "syno-cache-protection-btrfs.h"
#include "syno-cache-protection-btrfs-command.h"
#include "syno-cache-protection-btrfs-passive-model.h"

/*
 * copy from btrfs/file-item.c
 */
#define MAX_ORDERED_SUM_BYTES(r) ((PAGE_SIZE - \
				   sizeof(struct btrfs_ordered_sum)) / \
				   sizeof(u32) * (r)->sectorsize)

struct syno_cache_protection_replay_mapping {
	struct rb_node node;
	u64 subvolid;
	u64 old_inum;
	u64 new_inum;
};

struct syno_cache_protection_replay_instance {
	char filename_1[PATH_MAX];
	char filename_2[PATH_MAX];
	char eb_buffer[BTRFS_LEAF_SIZE];
	struct rb_root mapping_tree;
	struct btrfs_ordered_sum *csums;
	struct file *file_stdout;
	const char *mount_path;
	size_t mount_path_len;
	u64 root_subvolid;
};

static void *kallsyms_lookup_funcptr(const char *name)
{
	unsigned long addr;

	if (!name)
		return NULL;

	addr = kallsyms_lookup_name(name);
	return (void*)addr;
}

#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINEx(x, name, ...)	\
	static long syno##name(__MAP(x,__SC_DECL,__VA_ARGS__))					\
	{																		\
		long ret = -EOPNOTSUPP;												\
		mm_segment_t old_fs;												\
		static long (*func)(__MAP(x,__SC_DECL,__VA_ARGS__)) = NULL;			\
		static bool initialized = false;									\
		old_fs = get_fs();													\
		set_fs(KERNEL_DS);													\
		if (!initialized) {													\
			func = kallsyms_lookup_funcptr("sys"#name);						\
			initialized = true;												\
		}																	\
		if (!func)															\
			goto out;														\
		ret = func(__MAP(x,__SC_ARGS,__VA_ARGS__));							\
out:																		\
		set_fs(old_fs);														\
		task_work_run();													\
		flush_delayed_fput();												\
		return ret;															\
	}

#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE1(name, ...) SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINEx(1, _##name, __VA_ARGS__)
#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(name, ...) SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINEx(2, _##name, __VA_ARGS__)
#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE3(name, ...) SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINEx(3, _##name, __VA_ARGS__)
#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE4(name, ...) SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINEx(4, _##name, __VA_ARGS__)
#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE5(name, ...) SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINEx(5, _##name, __VA_ARGS__)

SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE3(open, const char *, pathname, int, flags, umode_t, mode)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE3(ioctl, unsigned int, fd, unsigned int, cmd, unsigned long, arg)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE1(close, unsigned int, fd)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(creat, const char *, pathname, umode_t, mode)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(newlstat, const char *, pathname, struct stat *, statbuf)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE3(mknod, const char *, pathname, umode_t, mode, unsigned, dev)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(mkdir, const char *, pathname, umode_t, mode)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(link, const char *, oldpath, const char *, newpath)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(symlink, const char *, target, const char *, linkpath)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE1(rmdir, const char *, pathname)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE1(unlink, const char *, pathname)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE4(utimensat, int, dfd, const char *, pathname, struct timespec *, utimes, int, flags)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE3(fchmodat, int, dfd, const char *, pathname, umode_t, mode)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE5(fchownat, int, dfd, const char *, pathname, uid_t, user, gid_t, group, int, flag)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(rename, const char *, oldname, const char *, newname)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(truncate, const char *, pathname, long, length)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE4(fallocate, int, fd, int, mode, loff_t, offset, loff_t, len)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE4(pwrite64, unsigned int, fd, const char *, buf, size_t, count, loff_t, pos)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE5(lsetxattr, const char *, pathname, const char *, name, const void *, value, size_t, size, int, flags)
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_SYSCALL_DEFINE2(lremovexattr, const char *, pathname, const char *, name)

static int write_buf(struct file *filp, const void *buf, u32 len)
{
	int ret;
	mm_segment_t old_fs;
	u32 pos = 0;
	loff_t off = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	while (pos < len) {
		ret = vfs_write(filp, (__force const char __user *)buf + pos, len - pos, &off);
		if (ret < 0)
			goto out;
		if (ret == 0) {
			ret = -EIO;
			goto out;
		}
		pos += ret;
	}

	ret = 0;

out:
	set_fs(old_fs);
	return ret;
}

static void __verbose_printk(struct file *filp, const char *fmt, ...)
{
	char buf[256];
	va_list args;
	int len;

	if (!filp)
		return;

	va_start(args, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	write_buf(filp, buf, len);
}
#define verbose_printk(...) __verbose_printk(replay_instance ? replay_instance->file_stdout : NULL, __VA_ARGS__)

static struct syno_cache_protection_replay_mapping *syno_cache_protection_replay_mapping_tree_search(struct rb_root *root,
					  u64 subvolid, u64 inum)
{
	struct rb_node *n;
	struct syno_cache_protection_replay_mapping *mapping;

	if (!root)
		return NULL;

	n = root->rb_node;

	while (n) {
		mapping = rb_entry(n, struct syno_cache_protection_replay_mapping, node);

		if (subvolid < mapping->subvolid)
			n = n->rb_left;
		else if (subvolid > mapping->subvolid)
			n = n->rb_right;
		else if (inum < mapping->old_inum)
			n = n->rb_left;
		else if (inum > mapping->old_inum)
			n = n->rb_right;
		else
			return mapping;
	}

	return NULL;
}

static int syno_cache_protection_replay_mapping_tree_insert(struct rb_root *root, struct syno_cache_protection_replay_mapping *mapping)
{
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct syno_cache_protection_replay_mapping *entry;

	if (!root || !mapping)
		return -EINVAL;

	p = &root->rb_node;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct syno_cache_protection_replay_mapping, node);

		if (mapping->subvolid < entry->subvolid)
			p = &(*p)->rb_left;
		else if (mapping->subvolid > entry->subvolid)
			p = &(*p)->rb_right;
		else if (mapping->old_inum < entry->old_inum)
			p = &(*p)->rb_left;
		else if (mapping->old_inum > entry->old_inum)
			p = &(*p)->rb_right;
		else
			return -EEXIST;
	}

	rb_link_node(&mapping->node, parent, p);
	rb_insert_color(&mapping->node, root);
	return 0;
}

static void syno_cache_protection_replay_instance_free(struct syno_cache_protection_replay_instance *replay_instance)
{
	struct rb_node *node;
	struct syno_cache_protection_replay_mapping *mapping;

	if (!replay_instance)
		return;

	while (!RB_EMPTY_ROOT(&replay_instance->mapping_tree)) {
		node = rb_first(&replay_instance->mapping_tree);
		mapping = rb_entry(node, struct syno_cache_protection_replay_mapping, node);

		rb_erase(node, &replay_instance->mapping_tree);
		RB_CLEAR_NODE(node);
		kfree(mapping);
		if (need_resched())
			cond_resched();
	}
	kfree(replay_instance->csums);
	if (replay_instance->file_stdout)
		fput(replay_instance->file_stdout);
	kfree(replay_instance);
}

static int syno_cache_protection_replay_mapping_alloc(struct syno_cache_protection_replay_instance *replay_instance, u64 subvolid, u64 old_inum, u64 new_inum)
{
	int ret;
	struct syno_cache_protection_replay_mapping *mapping;

	mapping = kzalloc(sizeof(*mapping), GFP_NOFS);
	if (!mapping) {
		ret = -ENOMEM;
		goto out;
	}

	mapping->subvolid = subvolid;
	mapping->old_inum = old_inum;
	mapping->new_inum = new_inum;

	ret = syno_cache_protection_replay_mapping_tree_insert(&replay_instance->mapping_tree, mapping);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static void syno_cache_protection_replay_mapping_convert(struct syno_cache_protection_replay_instance *replay_instance, u64 *subvolid, u64 *inum)
{
	struct syno_cache_protection_replay_mapping *mapping = NULL;

	mapping = syno_cache_protection_replay_mapping_tree_search(&replay_instance->mapping_tree, *subvolid, *inum);

	if (mapping)
		*inum = mapping->new_inum;
}

static int build_full_path_with_subvolid_and_inum(struct btrfs_fs_info *fs_info, struct syno_cache_protection_replay_instance *replay_instance,
												u64 subvol_objectid, u64 inum, char *name, size_t cb_name, const char* basename, size_t cb_basename)
{
	struct btrfs_root *root = fs_info->tree_root;
	struct btrfs_root *fs_root;
	struct btrfs_root_ref *root_ref;
	struct btrfs_inode_ref *inode_ref;
	struct btrfs_key key;
	struct btrfs_path *path = NULL;
	char *ptr;
	u64 dirid;
	int len;
	int ret;

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto err;
	}
	path->leave_spinning = 1;

	ptr = name + cb_name - 1;
	ptr[0] = '\0';
	dirid = inum;

	if (basename) {
		ptr -= cb_basename + 1;
		if (ptr < name) {
			ret = -ENAMETOOLONG;
			goto err;
		}
		memcpy(ptr + 1, basename, cb_basename);
		ptr[0] = '/';
	}

	/*
	 * Walk up the subvolume trees in the tree of tree roots by root
	 * backrefs until we hit the top-level subvolume.
	 */
	while ((subvol_objectid != BTRFS_FS_TREE_OBJECTID && subvol_objectid != replay_instance->root_subvolid) || dirid != BTRFS_FIRST_FREE_OBJECTID) {

		if (dirid != BTRFS_FIRST_FREE_OBJECTID) {
			key.objectid = subvol_objectid;
			key.type = BTRFS_ROOT_ITEM_KEY;
			key.offset = (u64)-1;
			fs_root = btrfs_read_fs_root_no_name(fs_info, &key);
			if (IS_ERR(fs_root)) {
				ret = PTR_ERR(fs_root);
				goto err;
			}
			goto walk_inode;
		}

		key.objectid = subvol_objectid;
		key.type = BTRFS_ROOT_BACKREF_KEY;
		key.offset = (u64)-1;

		ret = btrfs_search_slot(NULL, root, &key, path, 0, 0);
		if (ret < 0) {
			goto err;
		} else if (ret > 0) {
			ret = btrfs_previous_item(root, path, subvol_objectid,
						  BTRFS_ROOT_BACKREF_KEY);
			if (ret < 0) {
				goto err;
			} else if (ret > 0) {
				ret = -ENOENT;
				goto err;
			}
		}

		btrfs_item_key_to_cpu(path->nodes[0], &key, path->slots[0]);
		subvol_objectid = key.offset;

		root_ref = btrfs_item_ptr(path->nodes[0], path->slots[0],
					  struct btrfs_root_ref);
		len = btrfs_root_ref_name_len(path->nodes[0], root_ref);
		ptr -= len + 1;
		if (ptr < name) {
			ret = -ENAMETOOLONG;
			goto err;
		}
		read_extent_buffer(path->nodes[0], ptr + 1,
				   (unsigned long)(root_ref + 1), len);
		ptr[0] = '/';
		dirid = btrfs_root_ref_dirid(path->nodes[0], root_ref);
		btrfs_release_path(path);

		key.objectid = subvol_objectid;
		key.type = BTRFS_ROOT_ITEM_KEY;
		key.offset = (u64)-1;
		fs_root = btrfs_read_fs_root_no_name(fs_info, &key);
		if (IS_ERR(fs_root)) {
			ret = PTR_ERR(fs_root);
			goto err;
		}

walk_inode:
		/*
		 * Walk up the filesystem tree by inode refs until we hit the
		 * root directory.
		 */
		while (dirid != BTRFS_FIRST_FREE_OBJECTID) {
			key.objectid = dirid;
			key.type = BTRFS_INODE_REF_KEY;
			key.offset = (u64)-1;

			ret = btrfs_search_slot(NULL, fs_root, &key, path, 0, 0);
			if (ret < 0) {
				goto err;
			} else if (ret > 0) {
				ret = btrfs_previous_item(fs_root, path, dirid,
							  BTRFS_INODE_REF_KEY);
				if (ret < 0) {
					goto err;
				} else if (ret > 0) {
					ret = -ENOENT;
					goto err;
				}
			}

			btrfs_item_key_to_cpu(path->nodes[0], &key, path->slots[0]);
			dirid = key.offset;

			inode_ref = btrfs_item_ptr(path->nodes[0],
						   path->slots[0],
						   struct btrfs_inode_ref);
			len = btrfs_inode_ref_name_len(path->nodes[0],
						       inode_ref);
			ptr -= len + 1;
			if (ptr < name) {
				ret = -ENAMETOOLONG;
				goto err;
			}
			read_extent_buffer(path->nodes[0], ptr + 1,
					   (unsigned long)(inode_ref + 1), len);
			ptr[0] = '/';
			btrfs_release_path(path);
		}
	}

	ptr -= replay_instance->mount_path_len;
	if (ptr < name) {
		ret = -ENAMETOOLONG;
		goto err;
	}
	memcpy(ptr, replay_instance->mount_path, replay_instance->mount_path_len);

	if (ptr == name + cb_name - 1) {
		name[0] = '/';
		name[1] = '\0';
	} else {
		memmove(name, ptr, name + cb_name - ptr);
	}

	ret = 0;
err:
	btrfs_free_path(path);
	return ret;
}

#define SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(name, type)	\
	static inline type * metadata_command_to_##name(struct syno_cache_protection_passive_btrfs_metadata_command *command)		\
	{																															\
		return container_of(command, type, node);																				\
	}

SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(ordered_extent, struct syno_cache_protection_passive_btrfs_ordered_extent);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(inline_extent, struct syno_cache_protection_passive_btrfs_inline_extent);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(create, struct syno_cache_protection_passive_btrfs_create);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(inode_operation, struct syno_cache_protection_passive_btrfs_inode_operation);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(xattr, struct syno_cache_protection_passive_btrfs_xattr);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(subvol_operation, struct syno_cache_protection_passive_btrfs_subvol_operation);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(rename, struct syno_cache_protection_passive_btrfs_rename);
SYNO_CACHE_PROTECTION_REPLAY_DEFINE_METADATA_COMMAND_CONVERT_STRUCT(clone, struct syno_cache_protection_passive_btrfs_clone);

static struct btrfs_root *read_one_root(struct btrfs_fs_info *fs_info, u64 objectid)
{
	struct btrfs_key key;

	key.objectid = objectid;
	key.type = BTRFS_ROOT_ITEM_KEY;
	key.offset = 0;
	return btrfs_read_fs_root_no_name(fs_info, &key);
}

static struct inode *read_one_inode(struct btrfs_root *root, u64 objectid)
{
	struct btrfs_key key;

	key.objectid = objectid;
	key.type = BTRFS_INODE_ITEM_KEY;
	key.offset = 0;
	return btrfs_iget(root->fs_info->sb, &key, root, NULL);
}

static int btrfs_syno_cache_protection_passive_pinned_cached_extents(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command;
	struct syno_cache_protection_passive_btrfs_ordered_extent *ordered_extent;

	/* pin all ordered extent data range */
	list_for_each_entry(metadata_command, &passive_instance->metadata_command_head, list) {
		if (metadata_command->transid <= passive_instance->old_generation)
			continue;
		if (metadata_command->command != SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT)
			continue;
		ordered_extent = metadata_command_to_ordered_extent(metadata_command);
		ret = btrfs_syno_cache_exclude_cached_extent(fs_info, ordered_extent->start, ordered_extent->disk_len);
		if (ret) {
			btrfs_warn(fs_info, "Failed to SYNO Cache Protection exclude cached extent start:%llu len:%llu err %d", ordered_extent->start, ordered_extent->disk_len, ret);
			goto out;
		}
	}

	ret = 0;
out:
	return ret;
}

static int syno_cache_protection_check_and_add_unused_free_space(struct btrfs_fs_info *fs_info, struct extent_io_tree *freed_extents,
																struct syno_cache_protection_passive_btrfs_ordered_extent *ordered_extent, struct btrfs_path *path)
{
	int ret;
	struct btrfs_key key, found_key;
	u64 start = ordered_extent->start, end = ordered_extent->start + ordered_extent->disk_len - 1;
	u64 extent_start, extent_end, len, tmp_end;
	struct extent_buffer *leaf;
	struct btrfs_root *root = fs_info->extent_root;

	key.objectid = start;
	key.offset = 0;
	key.type = BTRFS_EXTENT_ITEM_KEY;
	ret = btrfs_search_slot(NULL, root, &key, path, 0, 0);
	if (ret < 0)
		goto out;

	while (true) {
prev_slot:
		if (path->slots[0] > 0) {
			path->slots[0]--;
		} else {
			ret = btrfs_prev_leaf(root, path);
			if (ret < 0)
				goto out;
			else if (ret > 0)
				break;
		}

		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);

		if (found_key.type == BTRFS_BLOCK_GROUP_ITEM_KEY)
			goto prev_slot;
		WARN_ON_ONCE(found_key.type != BTRFS_EXTENT_ITEM_KEY && found_key.type != BTRFS_METADATA_ITEM_KEY);

		ret = btrfs_comp_cpu_keys(&key, &found_key);
		if (ret > 0) {
			break;
		}
	}

	while (start <= end) {
next_slot:
		if (path->slots[0] >= btrfs_header_nritems(leaf)) {
			ret = btrfs_next_leaf(root, path);
			if (ret < 0)
				goto out;
			if (ret > 0)
				break;
		}
		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);

		if (found_key.type == BTRFS_BLOCK_GROUP_ITEM_KEY) {
			path->slots[0]++;
			goto next_slot;
		}
		WARN_ON_ONCE(found_key.type != BTRFS_EXTENT_ITEM_KEY && found_key.type != BTRFS_METADATA_ITEM_KEY);

		extent_start = found_key.objectid;
		extent_end = found_key.objectid + found_key.offset - 1;

		if (start >= extent_start && start <= extent_end) {
			len = extent_end - start + 1;
			btrfs_free_syno_cache_exclude_cached_extent(fs_info, start, len);
			start = extent_end + 1;
		} else if (start < extent_start) {
			tmp_end = min(end, extent_start - 1);
			set_extent_bits(freed_extents, start, tmp_end, EXTENT_UPTODATE);
			start = extent_end + 1;
		} /* else (start > extent_end) */
		path->slots[0]++;
	}

	if (start <= end)
		set_extent_bits(freed_extents, start, end, EXTENT_UPTODATE);

	ret = 0;
out:
	btrfs_release_path(path);
	return ret;
}

static void btrfs_syno_cache_protection_passive_free_cached_extents(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
																	struct syno_cache_protection_replay_instance *replay_instance)
{
	int err;
	struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command;
	struct syno_cache_protection_passive_btrfs_ordered_extent *ordered_extent;
	struct extent_io_tree freed_extents;
	u64 start, end;
	struct btrfs_path *path = NULL;
	struct btrfs_trans_handle *trans = NULL;

	extent_io_tree_init(&freed_extents, NULL);

	if (!fs_info || !fs_info->syno_cache_protection_recovering)
		goto out;

	trans = btrfs_attach_transaction(fs_info->tree_root);
	if (IS_ERR(trans)) {
		if (PTR_ERR(trans) != -ENOENT) {
			err = PTR_ERR(trans);
			goto out;
		}
	} else {
		err = btrfs_commit_transaction(trans, fs_info->tree_root);
		if (err)
			goto out;
	}

	path = btrfs_alloc_path();
	if (!path)
		goto out;

	/* free unused ordered extent data range */
	list_for_each_entry(metadata_command, &passive_instance->metadata_command_head, list) {
		if (metadata_command->transid <= passive_instance->old_generation)
			continue;
		if (metadata_command->command != SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT)
			continue;
		ordered_extent = metadata_command_to_ordered_extent(metadata_command);

		err = syno_cache_protection_check_and_add_unused_free_space(fs_info, &freed_extents, ordered_extent, path);
		if (err < 0) {
			btrfs_warn(fs_info, "Failed to SYNO Cache Protection lookup data extent start:%llu len:%llu err %d", ordered_extent->start, ordered_extent->disk_len, err);
			goto out;
		}
	}

	while (1) {
		err = find_first_extent_bit(&freed_extents, 0, &start, &end, EXTENT_UPTODATE, NULL);
		if (err)
			break;

		verbose_printk("Syno Cache Protection Replay add unused pinned extent, start:%llu, end:%llu, len:%llu\n", start, end, end - start + 1);
		btrfs_syno_cache_add_unused_extent(fs_info, start, end - start + 1);
		clear_extent_bits(&freed_extents, start, end, EXTENT_UPTODATE);
		cond_resched();
	}

out:
	btrfs_free_path(path);
	clear_extent_bits(&freed_extents, 0, -1, EXTENT_UPTODATE);
	if (fs_info) {
		btrfs_free_syno_cache_exclude_cached_extent(fs_info, 0 , -1);
		fs_info->syno_cache_protection_recovering = false;
		btrfs_clear_opt(fs_info->mount_opt, SYNO_CACHE_PROTECTION_RECOVER);
	}
	return;
}

/*
 * Return value
 * <0 : error
 */
int btrfs_syno_cache_protection_passive_replay_prepare(struct btrfs_fs_info *fs_info)
{
	int ret;
	struct syno_cache_protection_fs *cache_protection_fs = NULL;
	struct syno_cache_protection_passive_btrfs_instance *passive_instance = NULL;

	if (!fs_info) {
		ret = -EINVAL;
		goto out;
	}

	if (!btrfs_test_opt(fs_info->tree_root, SYNO_CACHE_PROTECTION_RECOVER)) {
		ret = 0;
		goto out;
	}

	cache_protection_fs = syno_cache_protection_get_passive_instance(SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
	if (!cache_protection_fs) {
		ret = 0;
		goto out;
	}
	spin_lock(&cache_protection_fs->lock);
	cache_protection_fs->enabled = false;
	spin_unlock(&cache_protection_fs->lock);

	passive_instance = (struct syno_cache_protection_passive_btrfs_instance *)cache_protection_fs->private;

	if (0 == atomic64_read(&passive_instance->last_transid) ||
		fs_info->generation > atomic64_read(&passive_instance->last_transid) + 1) {
		ret = 0;
		goto out;
	}

	fs_info->syno_cache_protection_recovering = true;

	passive_instance->old_generation = fs_info->generation;

	ret = btrfs_syno_cache_protection_passive_pinned_cached_extents(fs_info, passive_instance);
	if (ret)
		goto out;

	ret = 0;
out:
	syno_cache_protection_fs_put(cache_protection_fs);
	if (fs_info) {
		if (ret)
			fs_info->syno_cache_protection_recovering = false;
		if (!fs_info->syno_cache_protection_recovering)
			btrfs_clear_opt(fs_info->mount_opt, SYNO_CACHE_PROTECTION_RECOVER);
	}
	return ret;
}

void btrfs_syno_cache_protection_passive_replay_release(struct btrfs_fs_info *fs_info)
{
	struct syno_cache_protection_fs *cache_protection_fs = NULL;

	if (!fs_info)
		return;

	cache_protection_fs = syno_cache_protection_get_passive_instance(SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
	if (!cache_protection_fs)
		return;

	btrfs_syno_cache_protection_passive_free_cached_extents(fs_info, (struct syno_cache_protection_passive_btrfs_instance *)cache_protection_fs->private, NULL);
	syno_cache_protection_clear_passive_instance_with_fs(SYNO_CACHE_PROTECTION_ROLE_PASSIVE, SYNO_CACHE_PROTECTION_FS_BTRFS, BTRFS_FSID_SIZE, fs_info->fs_devices->fsid);
	syno_cache_protection_fs_put(cache_protection_fs);
}

static int replay_csums(struct btrfs_trans_handle *trans, struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
								struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_ordered_extent* ordered_extent)
{
	int ret;
	struct btrfs_ordered_sum *sum = replay_instance->csums;
	size_t i, j, csum_count,  csum_data_size, virtual_buffer_pos = 0;
	struct syno_cache_protection_stream_btrfs_command_ordered_extent_csum command_ordered_extent_csum;
	__le32 csum_data;

	trans->adding_csums = 1;
	csum_data_size = sizeof(csum_data);
	for (i = 0; i < ordered_extent->total_csums; i++) {
		ret = syno_cache_protection_passive_btrfs_virtual_buffer_read(ordered_extent->csums, virtual_buffer_pos, sizeof(command_ordered_extent_csum), &command_ordered_extent_csum);
		if (ret)
			goto out;
		virtual_buffer_pos += sizeof(command_ordered_extent_csum);
		sum->bytenr = le64_to_cpu(command_ordered_extent_csum.bytenr);
		sum->len = le32_to_cpu(command_ordered_extent_csum.len);
		csum_count = (int)DIV_ROUND_UP(sum->len, fs_info->csum_root->sectorsize);

		for (j = 0; j < csum_count; j++) {
			ret = syno_cache_protection_passive_btrfs_virtual_buffer_read(ordered_extent->csums, virtual_buffer_pos, csum_data_size, &csum_data);
			if (ret)
				goto out;
			virtual_buffer_pos += csum_data_size;
			sum->sums[j] = le32_to_cpu(csum_data);
		}

		ret = btrfs_del_csums(trans, fs_info->csum_root, sum->bytenr, sum->len);
		if (ret)
			goto out;
		ret = btrfs_csum_file_blocks(trans, fs_info->csum_root, sum);
		if (ret)
			goto out;
	}

	ret = 0;
out:
	trans->adding_csums = 0;
	return ret;
}

static int lock_extent_range(struct inode *inode, u64 off, u64 len)
{
	/*
	 * Do any pending delalloc/csum calculations on inode, one way or
	 * another, and lock file content.
	 * The locking order is:
	 *
	 *   1) pages
	 *   2) range in the inode's io tree
	 */
	while (1) {
		struct btrfs_ordered_extent *ordered;
		lock_extent(&BTRFS_I(inode)->io_tree, off, off + len - 1);
		ordered = btrfs_lookup_first_ordered_extent(inode,
							    off + len - 1);
		if ((!ordered ||
		     ordered->file_offset + ordered->len <= off ||
		     ordered->file_offset >= off + len) &&
		    !test_range_bit(&BTRFS_I(inode)->io_tree, off,
				    off + len - 1, EXTENT_DELALLOC, 0, NULL)) {
			if (ordered)
				btrfs_put_ordered_extent(ordered);
			break;
		}
		unlock_extent(&BTRFS_I(inode)->io_tree, off, off + len - 1);
		if (ordered)
			btrfs_put_ordered_extent(ordered);
		btrfs_wait_ordered_range(inode, off, len);
	}
	return 0;
}

static int check_prealloc_match(struct btrfs_fs_info *fs_info, struct inode *inode, u64 start, u64 end, u64 disk_bytenr, u64 disk_len)
{
	int ret;
	u64 ino = btrfs_ino(inode);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_path *path = NULL;
	struct btrfs_key key;
	struct extent_buffer *leaf;
	struct btrfs_file_extent_item *fi;
	u64 bytenr, num_bytes, extent_end;

	path = btrfs_alloc_path();
	if (!path) {
		ret = -ENOMEM;
		goto out;
	}

	key.objectid = ino;
	key.type = BTRFS_EXTENT_DATA_KEY;
	key.offset = start;

	ret = btrfs_search_slot(NULL, root, &key, path, 0, 0);
	if (ret < 0)
		goto out;
	if (ret > 0 && path->slots[0] > 0)
		path->slots[0]--;

	leaf = path->nodes[0];
	btrfs_item_key_to_cpu(leaf, &key, path->slots[0]);
	if (key.objectid != ino || key.type != BTRFS_EXTENT_DATA_KEY) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection Replay prealloc check start:%llu, end:%llu, key[%llu %u %llu] slot:%d", start, end, key.objectid, key.type, key.offset, path->slots[0]);
		btrfs_print_leaf(leaf);
		BUG();
	}
	fi = btrfs_item_ptr(leaf, path->slots[0], struct btrfs_file_extent_item);
	if (btrfs_file_extent_type(leaf, fi) != BTRFS_FILE_EXTENT_PREALLOC) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection Replay prealloc check start:%llu, end:%llu, key[%llu %u %llu] slot:%d", start, end, key.objectid, key.type, key.offset, path->slots[0]);
		btrfs_print_leaf(leaf);
		BUG();
	}
	extent_end = key.offset + btrfs_file_extent_num_bytes(leaf, fi);
	if (key.offset > start || extent_end < end) {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection Replay prealloc check start:%llu, end:%llu, key[%llu %u %llu] slot:%d", start, end, key.objectid, key.type, key.offset, path->slots[0]);
		btrfs_print_leaf(leaf);
		BUG();
	}

	bytenr = btrfs_file_extent_disk_bytenr(leaf, fi);
	num_bytes = btrfs_file_extent_disk_num_bytes(leaf, fi);

	if ((bytenr <= disk_bytenr) && (bytenr + num_bytes >= disk_bytenr + disk_len))
		ret = 1;
	else if ((bytenr + num_bytes <= disk_bytenr) || (bytenr >= disk_bytenr + disk_len))
		ret = 0;
	else {
		btrfs_warn(fs_info, "Failed to SYNO Cache Protection Replay ordered extent prealloc overlapping bytenr:%llu, num_bytes:%llu disk_bytenr:%llu, disk_len:%llu", bytenr, num_bytes, disk_bytenr, disk_len);
		BUG();
	}
out:
	btrfs_free_path(path);
	return ret;
}

static int replay_ordered_extent(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
								struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_ordered_extent* ordered_extent)
{
	int ret;
	struct btrfs_root *root;
	struct inode *inode = NULL;
	u64 logical_len;
	int compress_type = 0;
	u64 start_pos, end_pos, new_i_size;
	unsigned long flags;
	struct btrfs_trans_handle *trans = NULL;
	struct btrfs_block_group_cache *block_group = NULL;
	struct btrfs_space_info *space_info;
	u64 subvolid, inum, csum_leafs = 0;
	int skip_sum;

	flags = ordered_extent->flags;
	subvolid = ordered_extent->subvolid;
	inum = ordered_extent->inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

	verbose_printk("Syno Cache Protection Replay ordered_extent, subvolid:%llu inum:%llu file_offset:%llu len:%llu truncated_len:%llu \
disk_bytenr:%llu disk_len:%llu flags:%llu compress_type:%u i_size:%llu, total_csums:%u, total_csum_size:%u\n",
					subvolid, inum, ordered_extent->file_offset, ordered_extent->len, ordered_extent->truncated_len,
					ordered_extent->start, ordered_extent->disk_len, ordered_extent->flags, ordered_extent->compress_type,
					ordered_extent->i_size, ordered_extent->total_csums, ordered_extent->total_csum_size);

	if (ordered_extent->file_offset >= ordered_extent->i_size) {
		ret = 0;
		goto out;
	}

	logical_len = ordered_extent->len;

	if (test_bit(BTRFS_ORDERED_TRUNCATED, &flags)) {
		logical_len = ordered_extent->truncated_len;
		/* Truncated the entire extent, don't bother adding */
		if (!logical_len) {
			ret = 0;
			goto out;
		}
	}

	root = read_one_root(fs_info, subvolid);
	if (IS_ERR(root)) {
		ret = PTR_ERR(root);
		if (ret == -ENOENT)
			ret = 0;
		else
			btrfs_warn(fs_info, "Failed to read target root [%llu] for syno cache recover, err [%d]", subvolid, ret);
		goto out;
	}

	inode = read_one_inode(root, inum);
	if (IS_ERR(inode)) {
		ret = PTR_ERR(inode);
		inode = NULL;
		/* inode may deleted */
		if (ret == -ESTALE) {
			WARN_ON_ONCE(1);
			ret = 0;
		}
		goto out;
	}

	/* update outstanding i_size */
	new_i_size = ordered_extent->i_size;
	if (new_i_size > i_size_read(inode)) {
		/* Expand hole size to cover write data, preventing empty gap */
		ret = btrfs_cont_expand(inode, i_size_read(inode), new_i_size);
		if (ret)
			goto out;
		i_size_write(inode, new_i_size);
	}

	start_pos = round_down(ordered_extent->file_offset, root->sectorsize);
	end_pos = round_up(ordered_extent->file_offset + logical_len, root->sectorsize);

	ret = lock_extent_range(inode, start_pos, end_pos - start_pos);
	if (ret)
		goto out;

	if (ordered_extent->total_csums)
		csum_leafs = btrfs_csum_bytes_to_leaves(root, ordered_extent->disk_len);
	trans = btrfs_start_transaction(root, 4 + csum_leafs);
	if (IS_ERR(trans)) {
		ret = PTR_ERR(trans);
		trans = NULL;
		goto out_unlock;
	}

	if (test_bit(BTRFS_ORDERED_NOCOW, &flags))
		goto i_size_update;

	if (test_bit(BTRFS_ORDERED_COMPRESSED, &flags))
		compress_type = ordered_extent->compress_type;

	if (test_bit(BTRFS_ORDERED_PREALLOC, &flags)) {
		ret = check_prealloc_match(fs_info, inode, ordered_extent->file_offset, ordered_extent->file_offset + logical_len, ordered_extent->start, ordered_extent->disk_len);
		if (ret < 0)
			goto out_unlock;
	}
	if (test_bit(BTRFS_ORDERED_PREALLOC, &flags) && ret == 1) {
		BUG_ON(compress_type);
		ret = btrfs_mark_extent_written(trans, inode,
						ordered_extent->file_offset,
						ordered_extent->file_offset +
						logical_len);
	} else {
		BUG_ON(root == root->fs_info->tree_root);

		ret = btrfs_lookup_data_extent(root, ordered_extent->start, ordered_extent->disk_len);
		if (ret < 0) {
			goto out_unlock;
		} else if (ret > 0) {
			block_group = btrfs_lookup_block_group(fs_info, ordered_extent->start);
			if (!block_group) {
				btrfs_warn(fs_info, "Failed to lookup block group with [%llu] for syno cache recover.", ordered_extent->start);
				ret = -EINVAL;
				goto out_unlock;
			}
			space_info = block_group->space_info;
			spin_lock(&space_info->lock);
			spin_lock(&block_group->lock);
			space_info->bytes_reserved += ordered_extent->disk_len;
			block_group->reserved += ordered_extent->disk_len;
			spin_unlock(&block_group->lock);
			spin_unlock(&space_info->lock);
		} else {
			BUG();
		}

		ret = insert_reserved_file_extent(trans, inode,
						ordered_extent->file_offset,
						ordered_extent->start,
						ordered_extent->disk_len,
						logical_len, logical_len,
						compress_type, 0, 0,
						BTRFS_FILE_EXTENT_REG,
						true
#ifdef MY_ABC_HERE
						, 0
#endif /* MY_ABC_HERE */
						);
	}
	if (ret < 0)
		goto out_unlock;

	skip_sum = BTRFS_I(inode)->flags & BTRFS_INODE_NODATASUM;
	if ((skip_sum && ordered_extent->total_csums > 0) || (!skip_sum && ordered_extent->total_csums == 0)) {
		btrfs_warn(fs_info, "Failed to Syno Cache Protection Replay csum error with skip_sum[%d], total_csums[%u] for ordered_extent [%llu]", skip_sum ? 1 : 0, ordered_extent->total_csums, ordered_extent->start);
	}
	ret = replay_csums(trans, fs_info, passive_instance, replay_instance, ordered_extent);
	if (ret) {
		btrfs_warn(fs_info, "Failed to Syno Cache Protection Replay csum with ordered_extent [%llu]", ordered_extent->start);
		goto out_unlock;
	}

i_size_update:
	btrfs_ordered_update_i_size(inode, i_size_read(inode), NULL);

	btrfs_drop_extent_cache(inode, start_pos, end_pos -1, 0);

	ret = btrfs_update_inode(trans, root, inode);
	if (ret)
		goto out_unlock;

	ret = 0;
out_unlock:
	unlock_extent(&BTRFS_I(inode)->io_tree, start_pos, end_pos -1);
out:
	if (block_group)
		btrfs_put_block_group(block_group);
	if (trans) {
		if (ret)
			btrfs_abort_transaction(trans, root, ret);
		btrfs_end_transaction(trans, root);
	}
	if (inode) {
		if (!ret)
			invalidate_mapping_pages(inode->i_mapping, start_pos >> SYNO_CACHE_PROTECTION_DATA_SHIFT, (end_pos - 1) >> SYNO_CACHE_PROTECTION_DATA_SHIFT);
		iput(inode);
	}
	return ret;
}

static int replay_pwrite64(struct btrfs_fs_info *fs_info, const char *filename, const char *data, size_t count, off_t offset)
{
	int ret, temp_ret;
	int fd = -1;
	int open_flags = O_RDWR|O_LARGEFILE|O_NOATIME;
	size_t remain_size, total_written;
	char *src = (char *)data;

	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}

	remain_size = count;
	total_written = 0;
	while (remain_size > 0) {
		temp_ret = syno_pwrite64(fd, src + total_written, remain_size, offset + total_written);
		if (temp_ret > 0) {
			remain_size -= temp_ret;
			total_written += temp_ret;
		} else if (temp_ret == 0) {
			btrfs_warn(fs_info, "Failed to Syno Cache Protection Replay pwrite name:%s size:%zd offset:%lu",
									filename, remain_size, offset + total_written);
			ret = -EIO;
			goto out;
		} else {
			ret = temp_ret;
			btrfs_warn(fs_info, "Failed to Syno Cache Protection Replay pwrite name:%s size:%zd offset:%lu ret:%d",
									filename, remain_size, offset + total_written, ret);
			goto out;
		}
	}

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_inline_extent(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
								struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_inline_extent* inline_extent)
{
	int ret;
	char *filename = NULL;
	size_t cb_filename;
	u64 subvolid, inum;

	subvolid = inline_extent->subvolid;
	inum = inline_extent->inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename, cb_filename, NULL, 0);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, inum);
		goto out;
	}

	verbose_printk("Syno Cache Protection Replay inline data, subvolid:%llu inum:%llu name:%s inline_len:%llu\n", subvolid, inum, filename, inline_extent->inline_len);
	ret = replay_pwrite64(fs_info, filename, inline_extent->inline_data, inline_extent->inline_len, 0);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int replay_subvol_delete(const char *dirname, const char *basename, size_t cb_basename, void *arg_buf, size_t cb_arg_buf)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_LARGEFILE|O_NOATIME;
	struct btrfs_ioctl_vol_args *args;

	if (sizeof(*args) > cb_arg_buf) {
		ret = -EINVAL;
		goto out;
	}

	if (cb_basename > BTRFS_PATH_NAME_MAX) {
		ret = -ENAMETOOLONG;
		goto out;
	}

	fd = syno_open(dirname, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}
	args = arg_buf;
	memset(args, 0, sizeof(*args));
	strncpy(args->name, basename, cb_basename);
	args->name[cb_basename] = '\0';
	ret = syno_ioctl(fd, BTRFS_IOC_SNAP_DESTROY, (unsigned long)args);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_create(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_create *create)
{
	int ret;
	char *filename = NULL, *filename2 = NULL;
	size_t cb_filename, cb_filename2;
	u64 subvolid, dir, inum;
	char *symname;
	struct stat statbuf;
	int fd = -1;

	subvolid = create->subvolid;
	dir = create->dir;
	inum = create->inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &dir);
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, dir, filename, cb_filename, create->name, create->name_len);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, dir);
		goto out;
	}

	switch (create->node.command) {
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKFILE:
			verbose_printk("Syno Cache Protection Replay create, name:%s, mode:%llu\n", filename, create->mode);
			fd = syno_creat(filename, create->mode);
			if (fd < 0) {
				ret = fd;
				fd = -1;
				goto out;
			}
			syno_close(fd);
			fd = -1;
			ret = 0;
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKNOD:
			verbose_printk("Syno Cache Protection Replay mknod, name:%s, mode:%llu, rdev:%llu\n", filename, create->mode, create->rdev);
			ret = syno_mknod(filename, create->mode, create->rdev);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKDIR:
			verbose_printk("Syno Cache Protection Replay mkdir, name:%s, mode:%llu\n", filename, create->mode);
			ret = syno_mkdir(filename, create->mode);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_LINK:
			filename2 = replay_instance->filename_2;
			cb_filename2 = sizeof(replay_instance->filename_2);
			ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename2, cb_filename2, NULL, 0);
			if (ret) {
				btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, inum);
				goto out;
			}
			verbose_printk("Syno Cache Protection Replay link, oldpath:%s, newpath:%s\n", filename2, filename);
			ret = syno_link(filename2, filename);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SYMLINK:
			if (create->symname_len >= BTRFS_LEAF_SIZE) {
				ret = -EOVERFLOW;
				goto out;
			}
			symname = replay_instance->eb_buffer;
			ret = syno_cache_protection_passive_btrfs_virtual_buffer_read(create->symname, 0, create->symname_len, symname);
			if (ret)
				goto out;
			symname[create->symname_len] = '\0';
			verbose_printk("Syno Cache Protection Replay symlink, target:%s, linkpath:%s\n", symname, filename);
			ret = syno_symlink(symname, filename);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RMDIR:
			verbose_printk("Syno Cache Protection Replay rmdir, name:%s\n", filename);
			ret = syno_rmdir(filename);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_UNLINK:
			verbose_printk("Syno Cache Protection Replay unlink, name:%s\n", filename);
			ret = syno_unlink(filename);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SUBVOL_DELETE:
			ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, dir, filename, cb_filename, NULL, 0);
			if (ret) {
				btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, dir);
				goto out;
			}
			verbose_printk("Syno Cache Protection Replay subvol delete, name:%s/%s\n", filename, create->name);
			ret = replay_subvol_delete(filename, create->name, create->name_len, replay_instance->eb_buffer, sizeof(replay_instance->eb_buffer));
			break;
		default:
			/* command not found */
			btrfs_warn(fs_info, "Failed to unknown command %u", create->node.command);
			BUG();
		break;
	}
	if (ret)
		goto out;

	if (create->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKFILE ||
		create->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKNOD ||
		create->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKDIR ||
		create->node.command == SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SYMLINK) {
		memset(&statbuf, 0, sizeof(statbuf));
		ret = syno_newlstat(filename, &statbuf);
		if (ret)
			goto out;
		if (statbuf.st_ino != inum) {
			ret = syno_cache_protection_replay_mapping_alloc(replay_instance, subvolid, inum, statbuf.st_ino);
			if (ret)
				goto out;
		}
	}

	ret = 0;
out:
	return ret;
}

static int replay_inode_flags(const char *filename, unsigned int flags)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_LARGEFILE|O_NOATIME;

	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}
	ret = syno_ioctl(fd, FS_IOC_SETFLAGS, (unsigned long)&flags);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_fallocate(const char *filename, int mode, off_t offset, off_t len)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDWR|O_LARGEFILE|O_NOATIME;

	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}
	ret = syno_fallocate(fd, mode, offset, len);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_default_subvol(const char *filename, u64 subvol_id)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_NOATIME;

	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}
	ret = syno_ioctl(fd, BTRFS_IOC_DEFAULT_SUBVOL, (unsigned long)&subvol_id);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_inode_operation(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_inode_operation *operation)
{
	int ret;
	char *filename = NULL;
	size_t cb_filename;
	u64 subvolid, inum;

	subvolid = operation->subvolid;
	inum = operation->inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename, cb_filename, NULL, 0);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, inum);
		goto out;
	}

	switch (operation->node.command) {
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_FLAGS:
			verbose_printk("Syno Cache Protection Replay inode flags, subvolid:%llu inum:%llu filename:%s flags:%llu\n", subvolid, inum, filename, operation->flags);
			ret = replay_inode_flags(filename, operation->flags);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_UTIME:
			verbose_printk("Syno Cache Protection Replay utime, subvolid:%llu inum:%llu filename:%s\n", subvolid, inum, filename);
			ret = syno_utimensat(-1, filename, operation->times, AT_SYMLINK_NOFOLLOW);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHMODE:
			verbose_printk("Syno Cache Protection Replay chmod, subvolid:%llu inum:%llu filename:%s mode:%llu\n", subvolid, inum, filename, operation->mode);
			ret = syno_fchmodat(-1, filename, operation->mode);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHOWN:
			verbose_printk("Syno Cache Protection Replay chown, subvolid:%llu inum:%llu filename:%s uid:%u gid:%u\n", subvolid, inum, filename, operation->uid, operation->gid);
			ret = syno_fchownat(-1, filename, operation->uid, operation->gid, AT_SYMLINK_NOFOLLOW);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_TRUNCATE:
			verbose_printk("Syno Cache Protection Replay truncate, subvolid:%llu inum:%llu filename:%s length:%llu\n", subvolid, inum, filename, operation->length);
			ret = syno_truncate(filename, operation->length);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE:
			verbose_printk("Syno Cache Protection Replay fallocate, subvolid:%llu inum:%llu filename:%s mode:%llu offset:%llu length:%llu\n",
							subvolid, inum, filename, operation->flags, operation->offset, operation->length);
			ret = replay_fallocate(filename, operation->flags, operation->offset, operation->length);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DEFAULT_SUBVOL:
			verbose_printk("Syno Cache Protection Replay default subvol, subvol_id:%llu\n", operation->flags);
			ret = replay_default_subvol(filename, operation->flags);
			break;
		default:
			/* command not found */
			btrfs_warn(fs_info, "Failed to unknown command %u", operation->node.command);
			BUG();
		break;
	}
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int replay_rename(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_rename *rename)
{
	int ret;
	char *old_path = NULL;
	char *new_path = NULL;
	size_t cb_old_path;
	size_t cb_new_path;
	u64 subvolid;
	u64 old_dir;
	u64 new_dir;

	subvolid = rename->subvolid;
	old_dir = rename->old_dir;
	new_dir = rename->new_dir;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &old_dir);
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &new_dir);

	old_path = replay_instance->filename_1;
	cb_old_path = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info,
			replay_instance, subvolid, old_dir,
			old_path, cb_old_path, rename->old_name, rename->old_name_len);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build full path_with_subvolid_and_inum with subvolid [%llu] inode [%llu] name [%.*s]", subvolid, old_dir,
			(int)rename->old_name_len, rename->old_name);
		goto out;
	}

	new_path = replay_instance->filename_2;
	cb_new_path = sizeof(replay_instance->filename_2);
	ret = build_full_path_with_subvolid_and_inum(fs_info,
			replay_instance, subvolid, new_dir,
			new_path, cb_new_path, rename->new_name, rename->new_name_len);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu] name [%.*s]", subvolid, new_dir,
			(int)rename->new_name_len, rename->new_name);
		goto out;
	}

	verbose_printk("Syno Cache Protection Replay rename, oldname:%s newname:%s\n", old_path, new_path);
	ret = syno_rename(old_path, new_path);
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

static int replay_clone(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_clone *clone)
{
	int ret;
	char *src_filename = NULL, *dst_filename;
	size_t cb_src_filename, cb_dst_filename;
	u64 src_subvolid, src_inum, dst_subvolid, dst_inum;
	int src_fd = -1, dst_fd = -1;
	int open_flags = O_RDWR|O_LARGEFILE|O_NOATIME;
	struct btrfs_ioctl_clone_range_args clone_args;
	struct stat statbuf;

	src_subvolid = clone->src_subvolid;
	src_inum = clone->src_inum;
	dst_subvolid = clone->dst_subvolid;
	dst_inum = clone->dst_inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &src_subvolid, &src_inum);
	syno_cache_protection_replay_mapping_convert(replay_instance, &dst_subvolid, &dst_inum);

	src_filename = replay_instance->filename_1;
	cb_src_filename = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, src_subvolid, src_inum, src_filename, cb_src_filename, NULL, 0);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", src_subvolid, src_inum);
		goto out;
	}

	dst_filename = replay_instance->filename_2;
	cb_dst_filename = sizeof(replay_instance->filename_2);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, dst_subvolid, dst_inum, dst_filename, cb_dst_filename, NULL, 0);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", dst_subvolid, dst_inum);
		goto out;
	}

	memset(&statbuf, 0, sizeof(statbuf));
	ret = syno_newlstat(src_filename, &statbuf);
	if (ret)
		goto out;
	if (statbuf.st_size < clone->src_offset + clone->len) {
		ret = syno_truncate(src_filename, clone->src_offset + clone->len);
		if (ret)
			goto out;
	}

	src_fd = syno_open(src_filename, open_flags, 0);
	if (src_fd < 0) {
		ret = src_fd;
		src_fd = -1;
		goto out;
	}

	dst_fd = syno_open(dst_filename, open_flags, 0);
	if (dst_fd < 0) {
		ret = dst_fd;
		dst_fd = -1;
		goto out;
	}

	clone_args.src_fd = src_fd;
	clone_args.src_offset = clone->src_offset;
	clone_args.src_length = clone->len;
	clone_args.dest_offset = clone->dst_offset;
	verbose_printk("Syno Cache Protection Replay clone with src_subvolid:%llu src_inum:%llu src_file:%s src_offset:%llu len:%llu dst_subvolid:%llu dst_inum:%llu dst_file:%s, dst_offset:%llu\n",
					src_subvolid, src_inum, src_filename, clone->src_offset, clone->len, dst_subvolid, dst_inum, dst_filename, clone->dst_offset);
	ret = syno_ioctl(dst_fd, BTRFS_IOC_CLONE_RANGE, (unsigned long)&clone_args);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < src_fd)
		syno_close(src_fd);
	if (-1 < dst_fd)
		syno_close(dst_fd);
	return ret;
}

static int replay_xattr_archive_version_volume(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
		struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_xattr *xattr)
{
	int ret;
	char *value = NULL;
	struct syno_xattr_archive_version *arch_ver_le;
	size_t arch_vel_le_size;
	unsigned int archive_ver;
	int fd = -1;
	int open_flags = O_RDONLY|O_LARGEFILE|O_NOATIME;

	if ((xattr->node.command != SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR) ||
		(xattr->subvolid != BTRFS_FS_TREE_OBJECTID) ||
		(xattr->inum != BTRFS_FIRST_FREE_OBJECTID)) {
		ret = 0;
		goto out;
	}

	arch_vel_le_size = sizeof(*arch_ver_le);

	if (xattr->value_size < arch_vel_le_size) {
		ret = -EINVAL;
		goto out;
	}

	value = replay_instance->eb_buffer;
	ret = syno_cache_protection_passive_btrfs_virtual_buffer_read(xattr->value,
			0, arch_vel_le_size, value);
	if (ret)
		goto out;

	arch_ver_le = (struct syno_xattr_archive_version*)value;
	archive_ver = le32_to_cpu(arch_ver_le->v_archive_version);

	verbose_printk("Syno Cache Protection Replay archive version volume with version:%u\n", archive_ver);

	fd = syno_open(replay_instance->mount_path, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}
	ret = syno_ioctl(fd, FISETVERSION, (unsigned long)&archive_ver);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_xattr_generic(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
		struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_xattr *xattr)
{
	int ret;
	char *filename = NULL, *value = NULL;
	size_t cb_filename;
	u64 subvolid, inum;
	size_t cb_value_size, real_value_size;
#ifdef MY_ABC_HERE
	struct syno_acl *acl = NULL;
#endif /* MY_ABC_HERE */

	subvolid = xattr->subvolid;
	inum = xattr->inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename, cb_filename, NULL, 0);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, inum);
		goto out;
	}

	switch (xattr->node.command) {
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR:
			if (xattr->value_size >= BTRFS_LEAF_SIZE) {
				ret = -EOVERFLOW;
				goto out;
			}
			value = replay_instance->eb_buffer;
			real_value_size = xattr->value_size;
			cb_value_size = sizeof(replay_instance->eb_buffer);
			ret = syno_cache_protection_passive_btrfs_virtual_buffer_read(xattr->value,
				0, real_value_size, value);
			if (ret)
				goto out;
			value[real_value_size] = '\0';

#ifdef MY_ABC_HERE
			if (!strncmp(xattr->name, SYNO_ACL_XATTR_ACCESS, xattr->name_size)) {
				acl = btrfs_syno_acl_from_disk(value, xattr->value_size);
				if (IS_ERR(acl)) {
					ret = PTR_ERR(acl);
					acl = NULL;
					goto out;
				}
				ret = syno_acl_to_xattr(acl, value, cb_value_size);
				if (ret < 0)
					goto out;
				real_value_size = ret;
				value[real_value_size] = '\0';
			}
#endif /* MY_ABC_HERE */

			verbose_printk("Syno Cache Protection Replay setxattr with subvolid:%llu inum:%llu file:%s name:%s, value:%s, flags:%u\n", subvolid, inum, filename, xattr->name, value, xattr->flags);
			ret = syno_lsetxattr(filename, xattr->name, value, real_value_size, xattr->flags);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_REMOVEXATTR:
			verbose_printk("Syno Cache Protection Replay removexattr with subvolid:%llu inum:%llu file:%s name:%s\n", subvolid, inum, filename, xattr->name);
			ret = syno_lremovexattr(filename, xattr->name);
			if (ret && ret == -ENODATA)
				ret = 0;
			break;
		default:
			/* command not found */
			btrfs_warn(fs_info, "Failed to unknown command %u", xattr->node.command);
			BUG();
		break;
	}
	if (ret)
		goto out;

	ret = 0;
out:
#ifdef MY_ABC_HERE
	if (acl)
		syno_acl_release(acl);
#endif /* MY_ABC_HERE */
	return ret;
}

static int replay_xattr(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
		struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_xattr *xattr)
{
	if (!strcmp(xattr->name, XATTR_SYNO_PREFIX XATTR_SYNO_ARCHIVE_VERSION_VOLUME)) {
		return replay_xattr_archive_version_volume(fs_info, passive_instance, replay_instance, xattr);
	} else {
		return replay_xattr_generic(fs_info, passive_instance, replay_instance, xattr);
	}
}

static int replay_dirty_pages_prepare(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
								struct syno_cache_protection_replay_instance *replay_instance)
{
	int ret;
	struct rb_node *inode_node, *page_node;
	struct syno_cache_protection_passive_btrfs_inode *syno_inode;
	struct syno_cache_protection_passive_btrfs_page *syno_page;
	char *filename = NULL;
	size_t cb_filename;
	u64 subvolid, inum;
	struct stat statbuf;
	u64 max_i_size, cur_i_size;
	loff_t tmp_pos, tmp_len;

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	inode_node= rb_first(&passive_instance->inode_tree);
	while (inode_node) {
		syno_inode = rb_entry(inode_node, struct syno_cache_protection_passive_btrfs_inode, inode_node);

		subvolid = syno_inode->subvolid;
		inum = syno_inode->inum;
		syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

		verbose_printk("Syno Cache Protection Replay dirty inode prepare with subvolid:%llu inode:%llu syno_i_size:%llu\n", subvolid, inum, syno_inode->i_size);
		ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename, cb_filename, NULL, 0);
		if (ret) {
			/* inode maybe deleted with non-blocking write , so ignore it. */
			if (ret == -ENOENT) {
				inode_node = rb_next(inode_node);
				verbose_printk("Syno Cache Protection inode non-exist with subvolid [%llu], inode [%llu]\n", subvolid, inum);
				continue;
			}
			btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu] err [%d]", subvolid, inum, ret);
			goto out;
		}

		memset(&statbuf, 0, sizeof(statbuf));
		ret = syno_newlstat(filename, &statbuf);
		if (ret)
			goto out;
		max_i_size = max(syno_inode->i_size, (u64)statbuf.st_size);
		cur_i_size = statbuf.st_size;

		page_node = rb_first(&syno_inode->page_tree);
		while (page_node) {
			syno_page = rb_entry(page_node, struct syno_cache_protection_passive_btrfs_page, page_node);
			tmp_pos = syno_page->pg_offset << SYNO_CACHE_PROTECTION_DATA_SHIFT;
			tmp_len = (max_i_size >= tmp_pos + SYNO_CACHE_PROTECTION_DATA_SIZE) ? SYNO_CACHE_PROTECTION_DATA_SIZE : max_i_size - tmp_pos;
			cur_i_size = max(cur_i_size, (u64)(tmp_pos + tmp_len));
			page_node = rb_next(page_node);
		}

		if ((u64)statbuf.st_size < cur_i_size) {
			verbose_printk("Syno Cache Protection Replay dirty page pre expand with filename:%s, old_i_size:%llu, new_i_size:%llu\n", filename, (u64)statbuf.st_size, cur_i_size);
			ret = syno_truncate(filename, cur_i_size);
			if (ret)
				goto out;
		}

		inode_node = rb_next(inode_node);
	}

	ret = 0;
out:
	return ret;
}

static int replay_dirty_pages(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
								struct syno_cache_protection_replay_instance *replay_instance)
{
	int ret, temp_ret;
	struct rb_node *inode_node, *page_node;
	struct syno_cache_protection_passive_btrfs_inode *syno_inode;
	struct syno_cache_protection_passive_btrfs_page *syno_page;
	char *filename = NULL;
	size_t cb_filename;
	u64 subvolid, inum;
	int fd = -1;
	int open_flags = O_RDWR|O_LARGEFILE|O_NOATIME;
	size_t remain_size, total_written;
	char *src;
	loff_t tmp_pos, tmp_len;
	u64 i_size;
	struct stat statbuf;

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	inode_node= rb_first(&passive_instance->inode_tree);
	while (inode_node) {
		syno_inode = rb_entry(inode_node, struct syno_cache_protection_passive_btrfs_inode, inode_node);

		subvolid = syno_inode->subvolid;
		inum = syno_inode->inum;
		syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

		verbose_printk("Syno Cache Protection Replay dirty inode with subvolid:%llu inode:%llu syno_i_size:%llu\n", subvolid, inum, syno_inode->i_size);
		ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename, cb_filename, NULL, 0);
		if (ret) {
			/* inode maybe deleted with non-blocking write , so ignore it. */
			if (ret == -ENOENT) {
				inode_node = rb_next(inode_node);
				verbose_printk("Syno Cache Protection inode non-exist with subvolid [%llu], inode [%llu]\n", subvolid, inum);
				continue;
			}
			btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu] err [%d]", subvolid, inum, ret);
			goto out;
		}

		memset(&statbuf, 0, sizeof(statbuf));
		ret = syno_newlstat(filename, &statbuf);
		if (ret)
			goto out;
		i_size = max(syno_inode->i_size, (u64)statbuf.st_size);

		fd = syno_open(filename, open_flags, 0);
		if (fd < 0) {
			ret = fd;
			fd = -1;
			goto out;
		}

		page_node = rb_first(&syno_inode->page_tree);
		while (page_node) {
			syno_page = rb_entry(page_node, struct syno_cache_protection_passive_btrfs_page, page_node);
			tmp_pos = syno_page->pg_offset << SYNO_CACHE_PROTECTION_DATA_SHIFT;
			tmp_len = (i_size >= tmp_pos + SYNO_CACHE_PROTECTION_DATA_SIZE) ? SYNO_CACHE_PROTECTION_DATA_SIZE : i_size - tmp_pos;
			verbose_printk("Syno Cache Protection Replay dirty page with filename:%s, i_size:%llu pos:%lld len:%lld\n", filename, i_size, tmp_pos, tmp_len);

			remain_size = tmp_len;
			total_written = 0;
			src = syno_page->value;
			while (remain_size > 0) {
				temp_ret = syno_pwrite64(fd, src + total_written, remain_size, tmp_pos + total_written);
				if (temp_ret > 0) {
					remain_size -= temp_ret;
					total_written += temp_ret;
				} else if (temp_ret == 0) {
					btrfs_warn(fs_info, "Failed to Syno Cache Protection Replay write name:%s syno_i_size:%llu, pos:%lld, len:%zd",
									filename, syno_inode->i_size, tmp_pos + total_written, remain_size);
					ret = -EIO;
					goto out;
				} else {
					ret = temp_ret;
					btrfs_warn(fs_info, "Failed to Syno Cache Protection Replay write name:%s syno_i_size:%llu, pos:%lld, len:%zd, ret:%d",
									filename, syno_inode->i_size, tmp_pos + total_written, remain_size, ret);
					goto out;
				}
			}
			page_node = rb_next(page_node);
		}

		syno_close(fd);
		fd = -1;
		inode_node = rb_next(inode_node);
	}

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_qgroup_create(struct syno_cache_protection_replay_instance *replay_instance, const char *filename, struct btrfs_ioctl_qgroup_create_args *args)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_NOATIME;

	verbose_printk("Syno Cache Protection Replay qgroup create, filename:%s create:%llu qgroupid:%llu\n", filename, args->create, args->qgroupid);
	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}

	ret = syno_ioctl(fd, BTRFS_IOC_QGROUP_CREATE, (unsigned long)args);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_qgroup_assign(struct syno_cache_protection_replay_instance *replay_instance, const char *filename, struct btrfs_ioctl_qgroup_assign_args *args)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_NOATIME;

	verbose_printk("Syno Cache Protection Replay qgroup assign, filename:%s assign:%llu src:%llu dst:%llu\n", filename, args->assign, args->src, args->dst);
	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}

	ret = syno_ioctl(fd, BTRFS_IOC_QGROUP_ASSIGN, (unsigned long)args);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_qgroup_limit(struct syno_cache_protection_replay_instance *replay_instance, const char *filename, struct btrfs_ioctl_qgroup_limit_args *args)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_NOATIME;

	verbose_printk("Syno Cache Protection Replay qgroup limit, filename:%s qgroupid:%llu flag:%llu max_rfer:%llu max_excl:%llu rsv_rfer:%llu rsv_excl:%llu\n",
			filename, args->qgroupid, args->lim.flags, args->lim.max_rfer, args->lim.max_excl, args->lim.rsv_rfer, args->lim.rsv_excl);
	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}

	ret = syno_ioctl(fd, BTRFS_IOC_QGROUP_LIMIT, (unsigned long)args);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_usrquota_limit(struct syno_cache_protection_replay_instance *replay_instance, const char *filename, struct btrfs_ioctl_usrquota_limit_args *args)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_NOATIME;

	verbose_printk("Syno Cache Protection Replay usrqota limit, filename:%s uid:%llu rfer_soft:%llu rfer_hard:%llu\n", filename, args->uid, args->rfer_soft, args->rfer_hard);
	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}

	ret = syno_ioctl(fd, BTRFS_IOC_USRQUOTA_LIMIT, (unsigned long)args);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_usrquota_clean(struct syno_cache_protection_replay_instance *replay_instance, const char *filename, u64 uid)
{
	int ret;
	int fd = -1;
	int open_flags = O_RDONLY|O_NOATIME;

	verbose_printk("Syno Cache Protection Replay usrqota clean, filename:%s uid:%llu\n", filename, uid);
	fd = syno_open(filename, open_flags, 0);
	if (fd < 0) {
		ret = fd;
		fd = -1;
		goto out;
	}

	ret = syno_ioctl(fd, BTRFS_IOC_USRQUOTA_CLEAN, (unsigned long)&uid);
	if (ret)
		goto out;

	ret = 0;
out:
	if (-1 < fd)
		syno_close(fd);
	return ret;
}

static int replay_subvol_operation(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance,
						struct syno_cache_protection_replay_instance *replay_instance, struct syno_cache_protection_passive_btrfs_subvol_operation *operation)
{
	int ret;
	char *filename = NULL;
	size_t cb_filename;
	u64 subvolid, inum;

	subvolid = operation->subvolid;
	inum = operation->inum;
	syno_cache_protection_replay_mapping_convert(replay_instance, &subvolid, &inum);

	filename = replay_instance->filename_1;
	cb_filename = sizeof(replay_instance->filename_1);
	ret = build_full_path_with_subvolid_and_inum(fs_info, replay_instance, subvolid, inum, filename, cb_filename, NULL, 0);
	if (ret) {
		btrfs_warn(fs_info, "Failed to build_full_path_with_subvolid_and_inum with subvolid [%llu] inode [%llu]", subvolid, inum);
		goto out;
	}

	switch (operation->node.command) {
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_CREATE:
			ret = replay_qgroup_create(replay_instance, filename, &operation->qgroup_ca);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_ASSIGN:
			ret = replay_qgroup_assign(replay_instance, filename, &operation->qgroup_aa);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_LIMIT:
			ret = replay_qgroup_limit(replay_instance, filename, &operation->qgroup_la);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_LIMIT:
			ret = replay_usrquota_limit(replay_instance, filename, &operation->usrquota_la);
			break;
		case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_CLEAN:
			ret = replay_usrquota_clean(replay_instance, filename, operation->uid);
			break;
		default:
			/* command not found */
			btrfs_warn(fs_info, "Failed to unknown command %u", operation->node.command);
			BUG();
		break;
	}
	if (ret)
		goto out;

	ret = 0;
out:
	return ret;
}

int syno_cache_protection_recover(struct btrfs_fs_info *fs_info, struct syno_cache_protection_passive_btrfs_instance *passive_instance, struct syno_cache_protection_replay_args *replay_args)
{
	int ret;
	struct syno_cache_protection_passive_btrfs_metadata_command *metadata_command;
	struct syno_cache_protection_replay_instance *replay_instance = NULL;

	if (!fs_info || !passive_instance || !replay_args) {
		ret = -EINVAL;
		goto out;
	}

	if (!fs_info->syno_cache_protection_recovering || 0 == atomic64_read(&passive_instance->last_transid)) {
		ret = 0;
		goto out;
	}

	replay_instance = kzalloc(sizeof(*replay_instance), GFP_KERNEL);
	if (!replay_instance) {
		ret = -ENOMEM;
		goto out;
	}
	replay_instance->mapping_tree = RB_ROOT;
	replay_instance->mount_path = replay_args->mount_path;
	replay_instance->mount_path_len = replay_args->mount_path_len;
	replay_instance->root_subvolid = replay_args->root_subvolid;
	if (replay_args->verbose)
		replay_instance->file_stdout = fget(1);

	replay_instance->csums = kzalloc(btrfs_ordered_sum_size(fs_info->csum_root, MAX_ORDERED_SUM_BYTES(fs_info->csum_root)), GFP_KERNEL);
	if (!replay_instance->csums) {
		ret = -ENOMEM;
		goto out;
	}

	/* replay all commands */
	list_for_each_entry(metadata_command, &passive_instance->metadata_command_head, list) {
		if (metadata_command->transid <= passive_instance->old_generation)
			continue;
		switch (metadata_command->command) {
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_ORDERED_EXTENT:
				ret = replay_ordered_extent(fs_info, passive_instance, replay_instance, metadata_command_to_ordered_extent(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INLINE_EXTENT:
				ret = replay_inline_extent(fs_info, passive_instance, replay_instance, metadata_command_to_inline_extent(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKFILE:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKNOD:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_MKDIR:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_LINK:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SYMLINK:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RMDIR:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_UNLINK:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SUBVOL_DELETE:
				ret = replay_create(fs_info, passive_instance, replay_instance, metadata_command_to_create(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_FLAGS:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_UTIME:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHMODE:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_INODE_CHOWN:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_TRUNCATE:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_FALLOCATE:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_DEFAULT_SUBVOL:
				ret = replay_inode_operation(fs_info, passive_instance, replay_instance, metadata_command_to_inode_operation(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_RENAME:
				ret = replay_rename(fs_info, passive_instance, replay_instance, metadata_command_to_rename(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_CLONE:
				ret = replay_clone(fs_info, passive_instance, replay_instance, metadata_command_to_clone(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_REMOVEXATTR:
				ret = replay_xattr(fs_info, passive_instance, replay_instance, metadata_command_to_xattr(metadata_command));
				break;
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_CREATE:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_ASSIGN:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_QGROUP_LIMIT:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_LIMIT:
			case SYNO_CACHE_PROTECTION_BTRFS_COMMAND_USRQUOTA_CLEAN:
				ret = replay_subvol_operation(fs_info, passive_instance, replay_instance, metadata_command_to_subvol_operation(metadata_command));
				break;
			default:
				/* command not found */
				btrfs_warn(fs_info, "Failed to unknown command %u", metadata_command->command);
				BUG();
				break;
		}
		if (ret) {
			btrfs_warn(fs_info, "Failed to replay command %u, err:%d", metadata_command->command, ret);
			goto out;
		}
	}

	/* prepare dirty page replay with pre expand size */
	ret = replay_dirty_pages_prepare(fs_info, passive_instance, replay_instance);
	if (ret)
		goto out;

	btrfs_syno_cache_protection_passive_free_cached_extents(fs_info, passive_instance, replay_instance);

	verbose_printk("Syno Cache Protection Replay Dirty Pages\n");
	/* replay all dirty page */
	ret = replay_dirty_pages(fs_info, passive_instance, replay_instance);
	if (ret)
		goto out;

	ret = 0;
out:
	btrfs_syno_cache_protection_passive_free_cached_extents(fs_info, passive_instance, replay_instance);
	syno_cache_protection_replay_instance_free(replay_instance);
	return ret;
}

