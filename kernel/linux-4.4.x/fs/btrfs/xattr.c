#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Copyright (C) 2007 Red Hat.  All rights reserved.
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

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#include <linux/xattr.h>
#include <linux/security.h>
#include <linux/posix_acl_xattr.h>
#include <linux/sched.h>
#include "ctree.h"
#include "btrfs_inode.h"
#include "transaction.h"
#include "xattr.h"
#include "disk-io.h"
#include "props.h"
#include "locking.h"


ssize_t __btrfs_getxattr(struct inode *inode, const char *name,
				void *buffer, size_t size)
{
	struct btrfs_dir_item *di;
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_path *path;
	struct extent_buffer *leaf;
	int ret = 0;
	unsigned long data_ptr;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	/* lookup the xattr by name */
	di = btrfs_lookup_xattr(NULL, root, path, btrfs_ino(inode), name,
				strlen(name), 0);
	if (!di) {
		ret = -ENODATA;
		goto out;
	} else if (IS_ERR(di)) {
		ret = PTR_ERR(di);
		goto out;
	}

	leaf = path->nodes[0];
	/* if size is 0, that means we want the size of the attr */
	if (!size) {
		ret = btrfs_dir_data_len(leaf, di);
		goto out;
	}

	/* now get the data out of our dir_item */
	if (btrfs_dir_data_len(leaf, di) > size) {
		ret = -ERANGE;
		goto out;
	}

	/*
	 * The way things are packed into the leaf is like this
	 * |struct btrfs_dir_item|name|data|
	 * where name is the xattr name, so security.foo, and data is the
	 * content of the xattr.  data_ptr points to the location in memory
	 * where the data starts in the in memory leaf
	 */
	data_ptr = (unsigned long)((char *)(di + 1) +
				   btrfs_dir_name_len(leaf, di));
	read_extent_buffer(leaf, buffer, data_ptr,
			   btrfs_dir_data_len(leaf, di));
	ret = btrfs_dir_data_len(leaf, di);

out:
	btrfs_free_path(path);
	return ret;
}

static int do_setxattr(struct btrfs_trans_handle *trans,
		       struct inode *inode, const char *name,
		       const void *value, size_t size, int flags)
{
	struct btrfs_dir_item *di = NULL;
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_path *path;
	size_t name_len = strlen(name);
	int ret = 0;

	if (name_len + size > BTRFS_MAX_XATTR_SIZE(root))
		return -ENOSPC;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;
	path->skip_release_on_error = 1;

	if (!value) {
		di = btrfs_lookup_xattr(trans, root, path, btrfs_ino(inode),
					name, name_len, -1);
		if (!di && (flags & XATTR_REPLACE))
			ret = -ENODATA;
		else if (IS_ERR(di))
			ret = PTR_ERR(di);
		else if (di)
			ret = btrfs_delete_one_dir_name(trans, root, path, di);
		goto out;
	}

	/*
	 * For a replace we can't just do the insert blindly.
	 * Do a lookup first (read-only btrfs_search_slot), and return if xattr
	 * doesn't exist. If it exists, fall down below to the insert/replace
	 * path - we can't race with a concurrent xattr delete, because the VFS
	 * locks the inode's i_mutex before calling setxattr or removexattr.
	 */
	if (flags & XATTR_REPLACE) {
		ASSERT(inode_is_locked(inode));
		di = btrfs_lookup_xattr(NULL, root, path, btrfs_ino(inode),
					name, name_len, 0);
		if (!di)
			ret = -ENODATA;
		else if (IS_ERR(di))
			ret = PTR_ERR(di);
		if (ret)
			goto out;
		btrfs_release_path(path);
		di = NULL;
	}

	ret = btrfs_insert_xattr_item(trans, root, path, btrfs_ino(inode),
				      name, name_len, value, size);
	if (ret == -EOVERFLOW) {
		/*
		 * We have an existing item in a leaf, split_leaf couldn't
		 * expand it. That item might have or not a dir_item that
		 * matches our target xattr, so lets check.
		 */
		ret = 0;
		btrfs_assert_tree_locked(path->nodes[0]);
		di = btrfs_match_dir_item_name(root, path, name, name_len);
		if (!di && !(flags & XATTR_REPLACE)) {
			ret = -ENOSPC;
			goto out;
		}

#ifdef MY_ABC_HERE
		/*
		 * If overflow when replaced by extension, we need to change
		 * the size to the extension size and insert again, otherwise
		 * it may not be able to expand due to insufficient leaf free
		 * space, causing ENOSPC to fail.
		 */
		if (di) {
			struct extent_buffer *leaf = path->nodes[0];
			const u16 old_data_len = btrfs_dir_data_len(leaf, di);

			if (size > old_data_len) {
				u16 expand_size = size - old_data_len;

				if (btrfs_leaf_free_space(root, leaf) < expand_size) {
					di = NULL;
					btrfs_release_path(path);
					ret = btrfs_insert_xattr_item(trans, root, path, btrfs_ino(inode), name, name_len, value, expand_size);
					if (!ret) {
						/* logic error */
						WARN_ON_ONCE(1);
						ret = -EUCLEAN;
						btrfs_abort_transaction(trans, root, ret);
						goto out;
					} else if (ret == -EOVERFLOW) {
						ret = -ENOSPC;
						goto out;
					} else if (ret != -EEXIST) {
						goto out;
					}
					di = btrfs_match_dir_item_name(root, path, name, name_len);
					if (!di) {
						/* logic error */
						WARN_ON_ONCE(1);
						ret = -ENOSPC;
						goto out;
					}
					ret = 0;
				}
			}
		}
#endif /* MY_ABC_HERE */
	} else if (ret == -EEXIST) {
		ret = 0;
		di = btrfs_match_dir_item_name(root, path, name, name_len);
		ASSERT(di); /* logic error */
	} else if (ret) {
		goto out;
	}

	if (di && (flags & XATTR_CREATE)) {
		ret = -EEXIST;
		goto out;
	}

	if (di) {
		/*
		 * We're doing a replace, and it must be atomic, that is, at
		 * any point in time we have either the old or the new xattr
		 * value in the tree. We don't want readers (getxattr and
		 * listxattrs) to miss a value, this is specially important
		 * for ACLs.
		 */
		const int slot = path->slots[0];
		struct extent_buffer *leaf = path->nodes[0];
		const u16 old_data_len = btrfs_dir_data_len(leaf, di);
		const u32 item_size = btrfs_item_size_nr(leaf, slot);
		const u32 data_size = sizeof(*di) + name_len + size;
		struct btrfs_item *item;
		unsigned long data_ptr;
		char *ptr;

		if (size > old_data_len) {
			if (btrfs_leaf_free_space(root, leaf) <
			    (size - old_data_len)) {
				ret = -ENOSPC;
				goto out;
			}
		}

		if (old_data_len + name_len + sizeof(*di) == item_size) {
			/* No other xattrs packed in the same leaf item. */
			if (size > old_data_len)
				btrfs_extend_item(root, path,
						  size - old_data_len);
			else if (size < old_data_len)
				btrfs_truncate_item(root, path, data_size, 1);
		} else {
			/* There are other xattrs packed in the same item. */
			ret = btrfs_delete_one_dir_name(trans, root, path, di);
			if (ret)
				goto out;
			btrfs_extend_item(root, path, data_size);
		}

		item = btrfs_item_nr(slot);
		ptr = btrfs_item_ptr(leaf, slot, char);
		ptr += btrfs_item_size(leaf, item) - data_size;
		di = (struct btrfs_dir_item *)ptr;
		btrfs_set_dir_data_len(leaf, di, size);
		data_ptr = ((unsigned long)(di + 1)) + name_len;
		write_extent_buffer(leaf, value, data_ptr, size);
		btrfs_mark_buffer_dirty(leaf);
	} else {
		/*
		 * Insert, and we had space for the xattr, so path->slots[0] is
		 * where our xattr dir_item is and btrfs_insert_xattr_item()
		 * filled it.
		 */
	}
out:
	btrfs_free_path(path);
	return ret;
}

/*
 * @value: "" makes the attribute to empty, NULL removes it
 */
int __btrfs_setxattr(struct btrfs_trans_handle *trans,
		     struct inode *inode, const char *name,
		     const void *value, size_t size, int flags)
{
	struct btrfs_root *root = BTRFS_I(inode)->root;
	int ret;
#ifdef MY_DEF_HERE
	struct syno_cache_protection_parameter_command_xattr syno_cache_protection_parm;
	struct syno_cache_protection_parameter_command_generic syno_cache_protection_command_generic =
		{.command = SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR, .parm = &syno_cache_protection_parm};
	int syno_cp_err;
#endif /* MY_DEF_HERE */

#ifdef MY_ABC_HERE
	/*
	 * this check has been removed in commit 353c2ea735e4 ("btrfs: remove
	 * redundant readonly root check in btrfs_setxattr_trans")
	 */
#else
	if (btrfs_root_readonly(root))
		return -EROFS;
#endif

	if (trans)
		return do_setxattr(trans, inode, name, value, size, flags);

#ifdef MY_DEF_HERE
	if (inode->i_nlink > 0) {
		syno_cache_protection_parm.value_size = size;
		trans = btrfs_start_transaction_with_cache_protection(root, 2, &syno_cache_protection_command_generic);
	} else {
		trans = btrfs_start_transaction(root, 2);
	}
#else
	trans = btrfs_start_transaction(root, 2);
#endif /* MY_DEF_HERE */
	if (IS_ERR(trans))
		return PTR_ERR(trans);

	ret = do_setxattr(trans, inode, name, value, size, flags);
	if (ret)
		goto out;

	inode_inc_iversion(inode);
	inode->i_ctime = current_fs_time(inode->i_sb);
	set_bit(BTRFS_INODE_COPY_EVERYTHING, &BTRFS_I(inode)->runtime_flags);
	ret = btrfs_update_inode(trans, root, inode);
	BUG_ON(ret);

#ifdef MY_DEF_HERE
	if (!ret && syno_cache_protection_is_enabled(root->fs_info) && trans->syno_cache_protection_req) {
		memset(&syno_cache_protection_parm, 0, sizeof(syno_cache_protection_parm));
		syno_cache_protection_parm.command =
			(value) ? SYNO_CACHE_PROTECTION_BTRFS_COMMAND_SETXATTR :
			SYNO_CACHE_PROTECTION_BTRFS_COMMAND_REMOVEXATTR;
		syno_cache_protection_parm.transid = trans->transid;
		syno_cache_protection_parm.inode = inode;
		syno_cache_protection_parm.name_size = strlen(name);
		syno_cache_protection_parm.value_size = size;
		syno_cache_protection_parm.name = name;
		syno_cache_protection_parm.value = value;
		syno_cache_protection_parm.flags = flags;
		syno_cp_err = btrfs_syno_cache_protection_write_and_send_command(trans->syno_cache_protection_req,
			&syno_cache_protection_parm);
		if (syno_cp_err) {
			btrfs_warn(root->fs_info, "Failed to SYNO Cache Protection send command [%d] err %d",
			(int)syno_cache_protection_command_generic.command, syno_cp_err);
		}
		trans->syno_cache_protection_req = NULL;
	}
#endif /* MY_DEF_HERE */
out:
	btrfs_end_transaction(trans, root);
	return ret;
}

ssize_t btrfs_listxattr(struct dentry *dentry, char *buffer, size_t size)
{
	struct btrfs_key key;
	struct inode *inode = d_inode(dentry);
	struct btrfs_root *root = BTRFS_I(inode)->root;
	struct btrfs_path *path;
	int ret = 0;
	size_t total_size = 0, size_left = size;

	/*
	 * ok we want all objects associated with this id.
	 * NOTE: we set key.offset = 0; because we want to start with the
	 * first xattr that we find and walk forward
	 */
	key.objectid = btrfs_ino(inode);
	key.type = BTRFS_XATTR_ITEM_KEY;
	key.offset = 0;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;
	path->reada = READA_FORWARD;

	/* search for our xattrs */
	ret = btrfs_search_slot(NULL, root, &key, path, 0, 0);
	if (ret < 0)
		goto err;

	while (1) {
		struct extent_buffer *leaf;
		int slot;
		struct btrfs_dir_item *di;
		struct btrfs_key found_key;
		u32 item_size;
		u32 cur;

		leaf = path->nodes[0];
		slot = path->slots[0];

		/* this is where we start walking through the path */
		if (slot >= btrfs_header_nritems(leaf)) {
			/*
			 * if we've reached the last slot in this leaf we need
			 * to go to the next leaf and reset everything
			 */
			ret = btrfs_next_leaf(root, path);
			if (ret < 0)
				goto err;
			else if (ret > 0)
				break;
			continue;
		}

		btrfs_item_key_to_cpu(leaf, &found_key, slot);

		/* check to make sure this item is what we want */
		if (found_key.objectid != key.objectid)
			break;
		if (found_key.type > BTRFS_XATTR_ITEM_KEY)
			break;
		if (found_key.type < BTRFS_XATTR_ITEM_KEY)
			goto next_item;

		di = btrfs_item_ptr(leaf, slot, struct btrfs_dir_item);
		item_size = btrfs_item_size_nr(leaf, slot);
		cur = 0;
		while (cur < item_size) {
			u16 name_len = btrfs_dir_name_len(leaf, di);
			u16 data_len = btrfs_dir_data_len(leaf, di);
			u32 this_len = sizeof(*di) + name_len + data_len;
			unsigned long name_ptr = (unsigned long)(di + 1);

			if (verify_dir_item(root, leaf, di)) {
				ret = -EIO;
				goto err;
			}

			total_size += name_len + 1;
			/*
			 * We are just looking for how big our buffer needs to
			 * be.
			 */
			if (!size)
				goto next;

			if (!buffer || (name_len + 1) > size_left) {
				ret = -ERANGE;
				goto err;
			}

			read_extent_buffer(leaf, buffer, name_ptr, name_len);
			buffer[name_len] = '\0';

#ifdef MY_ABC_HERE
			/* Conceal the syno prefix from user space. Please refer to DSM#69101 */
			if (!strncmp(buffer, XATTR_SYNO_PREFIX, XATTR_SYNO_PREFIX_LEN) ||
					!strncmp(buffer, XATTR_BTRFS_PREFIX, XATTR_BTRFS_PREFIX_LEN)) {
				total_size -= name_len + 1;
				goto next;
			}
#endif /* MY_ABC_HERE */

			size_left -= name_len + 1;
			buffer += name_len + 1;
next:
			cur += this_len;
			di = (struct btrfs_dir_item *)((char *)di + this_len);
		}
next_item:
		path->slots[0]++;
	}
	ret = total_size;

err:
	btrfs_free_path(path);

	return ret;
}

static int btrfs_xattr_handler_get(const struct xattr_handler *handler,
				   struct dentry *dentry, const char *name,
				   void *buffer, size_t size)
{
	struct inode *inode = d_inode(dentry);

	name = xattr_full_name(handler, name);
	return __btrfs_getxattr(inode, name, buffer, size);
}

static int btrfs_xattr_handler_set(const struct xattr_handler *handler,
				   struct dentry *dentry, const char *name,
				   const void *buffer, size_t size,
				   int flags)
{
	struct inode *inode = d_inode(dentry);

	name = xattr_full_name(handler, name);
	return __btrfs_setxattr(NULL, inode, name, buffer, size, flags);
}

static int btrfs_xattr_handler_set_prop(const struct xattr_handler *handler,
					struct dentry *dentry,
					const char *name, const void *value,
					size_t size, int flags)
{
	int ret;
	struct inode *inode = d_inode(dentry);
	struct btrfs_trans_handle *trans;
	struct btrfs_root *root = BTRFS_I(inode)->root;

	name = xattr_full_name(handler, name);
	ret = btrfs_validate_prop(BTRFS_I(inode), name, value, size);
	if (ret)
		return ret;

	trans = btrfs_start_transaction(root, 2);
	if (IS_ERR(trans))
		return PTR_ERR(trans);

	ret = btrfs_set_prop(trans, inode, name, value, size, flags);
	if (!ret) {
		inode_inc_iversion(inode);
		inode->i_ctime = current_fs_time(inode->i_sb);
		set_bit(BTRFS_INODE_COPY_EVERYTHING,
			&BTRFS_I(inode)->runtime_flags);
		ret = btrfs_update_inode(trans, root, inode);
		BUG_ON(ret);
	}

	btrfs_end_transaction(trans, root);

	return ret;
}

static const struct xattr_handler btrfs_security_xattr_handler = {
	.prefix = XATTR_SECURITY_PREFIX,
	.get = btrfs_xattr_handler_get,
	.set = btrfs_xattr_handler_set,
};

static const struct xattr_handler btrfs_trusted_xattr_handler = {
	.prefix = XATTR_TRUSTED_PREFIX,
	.get = btrfs_xattr_handler_get,
	.set = btrfs_xattr_handler_set,
};

static const struct xattr_handler btrfs_user_xattr_handler = {
	.prefix = XATTR_USER_PREFIX,
	.get = btrfs_xattr_handler_get,
	.set = btrfs_xattr_handler_set,
};

static const struct xattr_handler btrfs_btrfs_xattr_handler = {
	.prefix = XATTR_BTRFS_PREFIX,
	.get = btrfs_xattr_handler_get,
	.set = btrfs_xattr_handler_set_prop,
};

const struct xattr_handler *btrfs_xattr_handlers[] = {
	&btrfs_security_xattr_handler,
#ifdef MY_ABC_HERE
	&btrfs_xattr_synoacl_access_handler,
	&btrfs_xattr_synoacl_noperm_access_handler,
#else
#ifdef CONFIG_BTRFS_FS_POSIX_ACL
	&posix_acl_access_xattr_handler,
	&posix_acl_default_xattr_handler,
#endif
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
	&btrfs_xattr_syno_handler,
#endif /* MY_ABC_HERE */
	&btrfs_trusted_xattr_handler,
	&btrfs_user_xattr_handler,
	&btrfs_btrfs_xattr_handler,
	NULL,
};

static int btrfs_initxattrs(struct inode *inode,
			    const struct xattr *xattr_array, void *fs_info)
{
	const struct xattr *xattr;
	struct btrfs_trans_handle *trans = fs_info;
	unsigned int nofs_flag;
	char *name;
	int err = 0;

	/*
	 * We're holding a transaction handle, so use a NOFS memory allocation
	 * context to avoid deadlock if reclaim happens.
	 */
	nofs_flag = memalloc_nofs_save();
	for (xattr = xattr_array; xattr->name != NULL; xattr++) {
		name = kmalloc(XATTR_SECURITY_PREFIX_LEN +
			       strlen(xattr->name) + 1, GFP_KERNEL);
		if (!name) {
			err = -ENOMEM;
			break;
		}
		strcpy(name, XATTR_SECURITY_PREFIX);
		strcpy(name + XATTR_SECURITY_PREFIX_LEN, xattr->name);
		err = __btrfs_setxattr(trans, inode, name,
				       xattr->value, xattr->value_len, 0);
		kfree(name);
		if (err < 0)
			break;
	}
	memalloc_nofs_restore(nofs_flag);
	return err;
}

int btrfs_xattr_security_init(struct btrfs_trans_handle *trans,
			      struct inode *inode, struct inode *dir,
			      const struct qstr *qstr)
{
	return security_inode_init_security(inode, dir, qstr,
					    &btrfs_initxattrs, trans);
}
