// SPDX-License-Identifier: GPL-2.0
/*
 * eCryptfs: Linux filesystem encryption layer
 *
 * Copyright (C) 1997-2004 Erez Zadok
 * Copyright (C) 2001-2004 Stony Brook University
 * Copyright (C) 2004-2007 International Business Machines Corp.
 */
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/exportfs.h>
#include "ecryptfs_kernel.h"

/* Filehandle flags. Now we only have this one. */
#define ECRYPTFS_FH_FLAG_CONNECTABLE	0x1 // parent dentry is encoded or not

struct ecryptfs_decode_ctx {
	u8 flags;	/* ECRYPTFS_FH_FLAG_* */
	struct dentry *mnt_root;
};

struct ecryptfs_fh {
	u8 len;		/* size of this header + size of fid in byte unit */
	u8 flags;	/* ECRYPTFS_FH* */
	u16 reserved;
	u8 fid[0];	/* file identifier */
} __packed;

#define ECRYPTFS_FH_HEADER_SIZE (offsetof(struct ecryptfs_fh, fid))

static int ecryptfs_encode_fh(struct inode *inode, u32 *fid, int *max_dwords,
			      struct inode *parent)
{
	struct dentry *dentry;
	struct ecryptfs_fh *fh = (struct ecryptfs_fh *)fid;
	struct inode *lower_inode;
	int lower_dwords = *max_dwords - (ECRYPTFS_FH_HEADER_SIZE >> 2);
	int type;

	/*
	 * 'lower_dwords' may be negative if '*max_dword' is zero.
	 * That is fine, we can just pass 0 to exportfs_encode_fh to get the
	 * size of fid that we need.
	 */
	if (lower_dwords < 0)
		lower_dwords = 0;

	dentry = d_find_any_alias(inode);
	if (dentry) {
		type = exportfs_encode_fh(ecryptfs_dentry_to_lower(dentry),
			(struct fid *)fh->fid, &lower_dwords, !!parent);
		dput(dentry);
		goto done;
	}

	lower_inode = ecryptfs_inode_to_lower(inode);
	if (lower_inode) {
		type = exportfs_encode_inode_fh(lower_inode, (struct fid *)fh->fid,
			&lower_dwords, NULL);
		goto done;
	}

	// No dentry and no lower inode, return failure.
	return FILEID_INVALID;

done:
	BUILD_BUG_ON(0 != (ECRYPTFS_FH_HEADER_SIZE % 4));
	*max_dwords = lower_dwords + (ECRYPTFS_FH_HEADER_SIZE >> 2);

	if (type < 0 || type == FILEID_INVALID ||
	    WARN_ON_ONCE((*max_dwords << 2) > MAX_HANDLE_SZ))
		return FILEID_INVALID;

	fh->len = *max_dwords << 2;
	if (dentry && parent)
		fh->flags = ECRYPTFS_FH_FLAG_CONNECTABLE;
	else
		fh->flags = 0;

	return type;
}

static int ecryptfs_acceptable(void *ctx, struct dentry *dentry)
{
	struct ecryptfs_decode_ctx *context = (struct ecryptfs_decode_ctx *)ctx;

	if (!d_is_dir(dentry) &&
	    !(context->flags & ECRYPTFS_FH_FLAG_CONNECTABLE))
		return 1;

	if (d_unhashed(dentry))
		return 0;

	/* Check if directory belongs to the layer we are decoding from */
	return is_subdir(dentry, context->mnt_root);
}

/* Find or instantiate an disconnected ecryptfs dentry from lower_dentry */
static struct dentry *ecryptfs_obtain_alias(struct super_block *sb,
					    struct dentry *lower_dentry)
{
	struct dentry *dentry;
	struct inode *inode;
	struct ecryptfs_dentry_info *dentry_info;

	if (d_is_dir(lower_dentry))
		return ERR_PTR(-EIO);

	inode = ecryptfs_get_inode(d_inode(lower_dentry), sb);
	if (IS_ERR(inode))
		return ERR_CAST(inode);

	dentry = d_find_any_alias(inode);
	if (!dentry) {
		dentry = d_alloc_anon(inode->i_sb);
		if (!dentry)
			goto nomem;

		dentry_info = kmem_cache_alloc(ecryptfs_dentry_info_cache,
					       GFP_KERNEL);
		if (!dentry_info)
			goto nomem;

		ecryptfs_set_dentry_private(dentry, dentry_info);
		dentry_info->lower_path.dentry = dget(lower_dentry);
		dentry_info->lower_path.mnt =
			mntget(ecryptfs_superblock_to_lower_mnt(sb));
	}

	return d_instantiate_anon(dentry, inode);

nomem:
	iput(inode);
	dput(dentry);
	return ERR_PTR(-ENOMEM);
}

/* Test if data is the lower inode of input inode */
static int ecryptfs_inode_test(struct inode *inode, void *data)
{
	return ecryptfs_inode_to_lower(inode) == data;
}

/* Lookup a ecryptfs dentry from cache, whose lower dentry is @lower_dentry */
static struct dentry *ecryptfs_lookup_dentry(struct super_block *sb,
					    struct dentry *lower_dentry)
{
	struct dentry *this = NULL;
	struct inode *inode, *lower_inode = d_inode(lower_dentry);

	inode = ilookup5(sb, (unsigned long)lower_inode,
			 ecryptfs_inode_test, lower_inode);

	if (!inode)
		return NULL;

	this = d_find_any_alias(inode);
	iput(inode);

	return this;
}

/* Lookup a child ecryptfs dentry whose lower dentry is @lower_dentry */
static struct dentry *ecryptfs_lookup_one(struct super_block *sb,
					  struct dentry *connected,
					  struct dentry *lower_dentry)
{
	struct inode *dir = d_inode(connected);
	struct dentry *this, *lower_parent = NULL;
	struct name_snapshot lower_name;
	char *decrypt_name = NULL;
	size_t name_size;
	int err;

	/*
	 * The dir mutex protects us from racing with rename.
	 * If the ecryptfs dentry that is above @lower_dentry has been
	 * moved to a parent that is not under the connected ecryptfs dir,
	 * we return -ECHILD.
	 */
	inode_lock_nested(dir, I_MUTEX_PARENT);
	err = -ECHILD;
	lower_parent = dget_parent(lower_dentry);
	if (ecryptfs_dentry_to_lower(connected) != lower_parent) {
		dput(lower_parent);
		inode_unlock(dir);
		return ERR_PTR(err);
	}

	/*
	 * We need to take a snapshot of 'lower_dentry' name to protect us
	 * from racing with lower fs rename.
	 */
	take_dentry_name_snapshot(&lower_name, lower_dentry);
	/* decrypt filename */
	err = ecryptfs_decode_and_decrypt_filename(&decrypt_name, &name_size,
						   sb, lower_name.name,
						   strlen(lower_name.name));
	if (err) {
		ecryptfs_printk(KERN_WARNING,
				"Error attempting to decode and decrypt filename [%s]; rc = [%d]\n",
				lower_name.name, err);
		goto fail;
	}

	/*
	 * Lookup ecryptfs dentry by decrypted name
	 * ecryptfs_lookup would encrypt the filename again and it is possible
	 * to avoid that because we already have the lower filename and lower
	 * dentry. In this version, we just do lookup_one_len to simplify
	 * the implementation.
	 */
	this = lookup_one_len(decrypt_name, connected, name_size);
	err = PTR_ERR(this);
	if (IS_ERR(this)) {
		goto fail;
	} else if (!this || !this->d_inode) {
		dput(this);
		err = -ENOENT;
		goto fail;
	} else if (ecryptfs_dentry_to_lower(this) != lower_dentry) {
		dput(this);
		err = -ESTALE;
		goto fail;
	}

	goto out;

fail:
	pr_warn_ratelimited("ecryptfs: failed to lookup one by lower dentry (%pd2, connected=%pd2, err=%i)\n",
			    lower_dentry, connected, err);
	this = ERR_PTR(err);

out:
	kfree(decrypt_name);
	release_dentry_name_snapshot(&lower_name);
	dput(lower_parent);
	inode_unlock(dir);
	return this;
}

/*
 * Lookup a ecryptfs dentry from cache whose lower dentry is
 * an ancestor of @lower_dentry. This dentry will be connected.
 */
static struct dentry *ecryptfs_lookup_ancestor(struct super_block *sb,
					       struct dentry *lower_dentry)
{
	struct dentry *lower_root = ecryptfs_dentry_to_lower(sb->s_root);
	struct dentry *next, *parent = NULL;	/* these are lower dentry */
	struct dentry *ancestor = ERR_PTR(-EIO);

	if (lower_root == lower_dentry)
		return dget(sb->s_root);

	next = dget(lower_dentry);
	for (;;) {
		parent = dget_parent(next);

		ancestor = ecryptfs_lookup_dentry(sb, next);
		if (ancestor)
			break;

		if (lower_root == parent) {
			ancestor = dget(sb->s_root);
			break;
		}

		if (parent == next) {
			/*
			 * We moved out of ecryptfs root and hit lower fs root.
			 * This may happen if the dentry has been moved out of
			 * ecryptfs, so we return ESTALE.
			 */
			ancestor = ERR_PTR(-ESTALE);
			break;
		}

		dput(next);
		next = parent;
	}

	dput(parent);
	dput(next);
	return ancestor;
}

/* lookup a connected ecryptfs dentry whose lower dentry is @lower_dentry */
static struct dentry *ecryptfs_lookup_connected(struct super_block *sb,
						struct dentry *lower_dentry)
{
	struct dentry *lower_root = ecryptfs_dentry_to_lower(sb->s_root);
	struct dentry *connected;
	int err = 0;

	/*
	 * Lookup a connected ecryptfs dentry whose lower dentry is
	 * an ancestor of 'lower_dentry'.
	 */
	connected = ecryptfs_lookup_ancestor(sb, lower_dentry);
	if (IS_ERR_OR_NULL(connected))
		return connected;

	while (!err) {
		struct dentry *this;
		struct dentry *next, *parent = NULL; /* these are lower */
		struct dentry *lower_connected =
			ecryptfs_dentry_to_lower(connected);

		/* found it */
		if (lower_connected == lower_dentry)
			break;

		/* find the topmost dentry not yet connected */
		next = dget(lower_dentry);
		for (;;) {
			parent = dget_parent(next);

			if (lower_connected == parent)
				break;

			/*
			 * If @lower_dentry has been moved out of
			 * @lower_connected, we will not find @lower_connected
			 * and hit ecryptfs root.
			 * In that case, we need to restart connecting.
			 * This game can go on forever in the worst case. We
			 * may want to consider taking s_vfs_rename_mutex if
			 * this happens more than once.
			 */
			if (parent == lower_root) {
				dput(connected);
				connected = dget(sb->s_root);
				break;
			}

			/*
			 * We moved out of ecryptfs root and hit lower fs root.
			 * This may happen if the dentry has been moved out of
			 * ecryptfs, so we return ESTALE.
			 */
			if (parent == next) {
				err = -ESTALE;
				break;
			}

			dput(next);
			next = parent;
		}

		if (!err) {
			this = ecryptfs_lookup_one(sb, connected, next);
			if (IS_ERR(this))
				err = PTR_ERR(this);

			/*
			 * Lookup child of connected can fail when racing
			 * with rename. If the ecryptfs dentry that is above
			 * 'next' has already been moved to a parent that is
			 * not under the 'connected' dir, we need to restart
			 * the lookup from the top because we cannot trust that
			 * 'lower_connected' is still an ancestor of
			 * 'lower_dentry'.
			 */
			if (err == -ECHILD) {
				this = ecryptfs_lookup_ancestor(sb, lower_dentry);
				err = PTR_ERR_OR_ZERO(this);
			}
			if (!err) {
				dput(connected);
				connected = this;
			}
		}

		dput(parent);
		dput(next);
	}
	if (err)
		goto fail;

	return connected;

fail:
	pr_warn_ratelimited("ecryptfs: failed to lookup by lower_dentry (%pd2, connected=%pd2, err=%i)\n",
			    lower_dentry, connected, err);

	dput(connected);
	return ERR_PTR(err);
}

static struct dentry *ecryptfs_get_dentry(struct super_block *sb,
					  struct dentry *lower,
					  bool connected)
{
	/* Obtain a disconnected dentry. */
	if (!d_is_dir(lower) && !connected)
		return ecryptfs_obtain_alias(sb, lower);

	/* Removed empty directory? */
	if ((lower->d_flags & DCACHE_DISCONNECTED) || d_unhashed(lower))
		return ERR_PTR(-ENOENT);

	return ecryptfs_lookup_connected(sb, lower);
}

static struct dentry *ecryptfs_fh_to_dentry(struct super_block *sb,
					    struct fid *fid,
					    int fh_len,
					    int fh_type)
{
	struct ecryptfs_fh *fh = (struct ecryptfs_fh *)fid;
	struct vfsmount *lower_mnt = ecryptfs_superblock_to_lower_mnt(sb);
	struct dentry *lower_dentry;
	struct dentry *dentry;
	struct ecryptfs_decode_ctx ctx;
	bool connected;

	if (fh_len != (fh->len >> 2))
		return ERR_PTR(-ESTALE);

	/* now fh_len is length of lower fh */
	fh_len -= ECRYPTFS_FH_HEADER_SIZE >> 2;
	ctx.flags = fh->flags;
	ctx.mnt_root = lower_mnt->mnt_root;

	lower_dentry = exportfs_decode_fh(lower_mnt, (struct fid *)fh->fid,
					  fh_len, fh_type,
					  ecryptfs_acceptable, &ctx);

	if (IS_ERR_OR_NULL(lower_dentry))
		return lower_dentry;

	connected = !!(fh->flags & ECRYPTFS_FH_FLAG_CONNECTABLE);

	dentry = ecryptfs_get_dentry(sb, lower_dentry, connected);
	dput(lower_dentry);

	return dentry;
}

static struct dentry *ecryptfs_fh_to_parent(struct super_block *sb,
					    struct fid *fid,
					    int fh_len, int fh_type)
{
	/*
	 * All the dentries should be connected in ecryptfs_fh_to_dentry if
	 * it is possible. If we get here, this file should be -EACCES because
	 * it didn't pass the test of find_acceptable_alias.
	 */
	return ERR_PTR(-EACCES);
}

struct dentry *ecryptfs_get_parent(struct dentry *child)
{
	/*
	 * ecryptfs_fh_to_dentry() returns connected dentries,
	 * ecryptfs_fh_to_parent() is not implemented, so we sould not get here
	 */
	WARN_ON_ONCE(1);
	return ERR_PTR(-EIO);
}

static int ecryptfs_get_name(struct dentry *parent, char *name,
			     struct dentry *child)
{
	/*
	 * ecryptfs_get_parent() is not implemented,
	 * so we should not get here
	 */
	WARN_ON_ONCE(1);
	return -EIO;
}

const struct export_operations ecryptfs_export_ops = {
	.encode_fh	= ecryptfs_encode_fh,
	.fh_to_dentry	= ecryptfs_fh_to_dentry,
	.fh_to_parent	= ecryptfs_fh_to_parent,
	.get_parent	= ecryptfs_get_parent,
	.get_name	= ecryptfs_get_name
};
