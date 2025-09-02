#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Simple MTD partitioning layer
 *
 * Copyright © 2000 Nicolas Pitre <nico@fluxnic.net>
 * Copyright © 2002 Thomas Gleixner <gleixner@linutronix.de>
 * Copyright © 2000-2010 David Woodhouse <dwmw2@infradead.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/kmod.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/err.h>
#include <linux/kconfig.h>
#ifdef MY_ABC_HERE
#include <linux/synolib.h>
#endif /* MY_ABC_HERE */

#include "mtdcore.h"

#ifdef MY_ABC_HERE
extern unsigned char grgbLanMac[SYNO_MAC_MAX_NUMBER][16];
extern int giVenderFormatVersion;
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
extern char gszSerialNum[];
extern char gszCustomSerialNum[];
#define SYNO_SN_TAG "SN="
#define SYNO_CHKSUM_TAG "CHK="
#define SYNO_SN_12_SIG SYNO_SN_TAG  // signature for 12 serial number
#endif /* MY_ABC_HERE */

/* Our partition linked list */
static LIST_HEAD(mtd_partitions);
static DEFINE_MUTEX(mtd_partitions_mutex);

/* Our partition node structure */
struct mtd_part {
	struct mtd_info mtd;
	struct mtd_info *master;
	uint64_t offset;
	struct list_head list;
};

/*
 * Given a pointer to the MTD object in the mtd_part structure, we can retrieve
#if defined(CONFIG_SYNO_LSP_RTD1619)
 * the pointer to that structure.
#else // CONFIG_SYNO_LSP_RTD1619
 * the pointer to that structure with this macro.
#endif // CONFIG_SYNO_LSP_RTD1619
 */
#if defined(CONFIG_SYNO_LSP_RTD1619)
static inline struct mtd_part *mtd_to_part(const struct mtd_info *mtd)
{
	return container_of(mtd, struct mtd_part, mtd);
}
#else /* CONFIG_SYNO_LSP_RTD1619 */
#define PART(x)  ((struct mtd_part *)(x))
#endif /* CONFIG_SYNO_LSP_RTD1619 */


/*
 * MTD methods which simply translate the effective address and pass through
 * to the _real_ device.
 */

static int part_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_ecc_stats stats;
	int res;

	stats = part->master->ecc_stats;
	res = part->master->_read(part->master, from + part->offset, len,
				  retlen, buf);
	if (unlikely(mtd_is_eccerr(res)))
		mtd->ecc_stats.failed +=
			part->master->ecc_stats.failed - stats.failed;
	else
		mtd->ecc_stats.corrected +=
			part->master->ecc_stats.corrected - stats.corrected;
	return res;
}

static int part_point(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, void **virt, resource_size_t *phys)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	return part->master->_point(part->master, from + part->offset, len,
				    retlen, virt, phys);
}

static int part_unpoint(struct mtd_info *mtd, loff_t from, size_t len)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	return part->master->_unpoint(part->master, from + part->offset, len);
}

static unsigned long part_get_unmapped_area(struct mtd_info *mtd,
					    unsigned long len,
					    unsigned long offset,
					    unsigned long flags)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	offset += part->offset;
	return part->master->_get_unmapped_area(part->master, len, offset,
						flags);
}

static int part_read_oob(struct mtd_info *mtd, loff_t from,
		struct mtd_oob_ops *ops)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	int res;

	if (from >= mtd->size)
		return -EINVAL;
	if (ops->datbuf && from + ops->len > mtd->size)
		return -EINVAL;

	/*
	 * If OOB is also requested, make sure that we do not read past the end
	 * of this partition.
	 */
	if (ops->oobbuf) {
		size_t len, pages;

#if defined(CONFIG_SYNO_LSP_RTD1619)
		len = mtd_oobavail(mtd, ops);
#else /* CONFIG_SYNO_LSP_RTD1619 */
		if (ops->mode == MTD_OPS_AUTO_OOB)
			len = mtd->oobavail;
		else
			len = mtd->oobsize;
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		pages = mtd_div_by_ws(mtd->size, mtd);
		pages -= mtd_div_by_ws(from, mtd);
		if (ops->ooboffs + ops->ooblen > pages * len)
			return -EINVAL;
	}

	res = part->master->_read_oob(part->master, from + part->offset, ops);
	if (unlikely(res)) {
		if (mtd_is_bitflip(res))
			mtd->ecc_stats.corrected++;
		if (mtd_is_eccerr(res))
			mtd->ecc_stats.failed++;
	}
	return res;
}

static int part_read_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_read_user_prot_reg(part->master, from, len,
						 retlen, buf);
}

static int part_get_user_prot_info(struct mtd_info *mtd, size_t len,
				   size_t *retlen, struct otp_info *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_get_user_prot_info(part->master, len, retlen,
						 buf);
}

static int part_read_fact_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_read_fact_prot_reg(part->master, from, len,
						 retlen, buf);
}

static int part_get_fact_prot_info(struct mtd_info *mtd, size_t len,
				   size_t *retlen, struct otp_info *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_get_fact_prot_info(part->master, len, retlen,
						 buf);
}

static int part_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_write(part->master, to + part->offset, len,
				    retlen, buf);
}

static int part_panic_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_panic_write(part->master, to + part->offset, len,
					  retlen, buf);
}

static int part_write_oob(struct mtd_info *mtd, loff_t to,
		struct mtd_oob_ops *ops)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	if (to >= mtd->size)
		return -EINVAL;
	if (ops->datbuf && to + ops->len > mtd->size)
		return -EINVAL;
	return part->master->_write_oob(part->master, to + part->offset, ops);
}

static int part_write_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_write_user_prot_reg(part->master, from, len,
						  retlen, buf);
}

static int part_lock_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_lock_user_prot_reg(part->master, from, len);
}

static int part_writev(struct mtd_info *mtd, const struct kvec *vecs,
		unsigned long count, loff_t to, size_t *retlen)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_writev(part->master, vecs, count,
				     to + part->offset, retlen);
}

static int part_erase(struct mtd_info *mtd, struct erase_info *instr)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	int ret;

	instr->addr += part->offset;
	ret = part->master->_erase(part->master, instr);
	if (ret) {
		if (instr->fail_addr != MTD_FAIL_ADDR_UNKNOWN)
			instr->fail_addr -= part->offset;
		instr->addr -= part->offset;
	}
	return ret;
}

void mtd_erase_callback(struct erase_info *instr)
{
	if (instr->mtd->_erase == part_erase) {
#if defined(CONFIG_SYNO_LSP_RTD1619)
		struct mtd_part *part = mtd_to_part(instr->mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
		struct mtd_part *part = PART(instr->mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

		if (instr->fail_addr != MTD_FAIL_ADDR_UNKNOWN)
			instr->fail_addr -= part->offset;
		instr->addr -= part->offset;
	}
	if (instr->callback)
		instr->callback(instr);
}
EXPORT_SYMBOL_GPL(mtd_erase_callback);

static int part_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_lock(part->master, ofs + part->offset, len);
}

static int part_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_unlock(part->master, ofs + part->offset, len);
}

static int part_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_is_locked(part->master, ofs + part->offset, len);
}

static void part_sync(struct mtd_info *mtd)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	part->master->_sync(part->master);
}

static int part_suspend(struct mtd_info *mtd)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return part->master->_suspend(part->master);
}

static void part_resume(struct mtd_info *mtd)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	part->master->_resume(part->master);
}

static int part_block_isreserved(struct mtd_info *mtd, loff_t ofs)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	ofs += part->offset;
	return part->master->_block_isreserved(part->master, ofs);
}

static int part_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	ofs += part->offset;
	return part->master->_block_isbad(part->master, ofs);
}

static int part_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	int res;

	ofs += part->offset;
	res = part->master->_block_markbad(part->master, ofs);
	if (!res)
		mtd->ecc_stats.badblocks++;
	return res;
}

#if defined(CONFIG_SYNO_LSP_RTD1619)
static int part_get_device(struct mtd_info *mtd)
{
	struct mtd_part *part = mtd_to_part(mtd);
	return part->master->_get_device(part->master);
}

static void part_put_device(struct mtd_info *mtd)
{
	struct mtd_part *part = mtd_to_part(mtd);
	part->master->_put_device(part->master);
}

static int part_ooblayout_ecc(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *oobregion)
{
	struct mtd_part *part = mtd_to_part(mtd);

	return mtd_ooblayout_ecc(part->master, section, oobregion);
}

static int part_ooblayout_free(struct mtd_info *mtd, int section,
			       struct mtd_oob_region *oobregion)
{
	struct mtd_part *part = mtd_to_part(mtd);

	return mtd_ooblayout_free(part->master, section, oobregion);
}

static const struct mtd_ooblayout_ops part_ooblayout_ops = {
	.ecc = part_ooblayout_ecc,
	.free = part_ooblayout_free,
};

#endif /* CONFIG_SYNO_LSP_RTD1619 */
static inline void free_partition(struct mtd_part *p)
{
	kfree(p->mtd.name);
	kfree(p);
}

/*
 * This function unregisters and destroy all slave MTD objects which are
 * attached to the given master MTD object.
 */

int del_mtd_partitions(struct mtd_info *master)
{
	struct mtd_part *slave, *next;
	int ret, err = 0;

	mutex_lock(&mtd_partitions_mutex);
	list_for_each_entry_safe(slave, next, &mtd_partitions, list)
		if (slave->master == master) {
			ret = del_mtd_device(&slave->mtd);
			if (ret < 0) {
				err = ret;
				continue;
			}
			list_del(&slave->list);
			free_partition(slave);
		}
	mutex_unlock(&mtd_partitions_mutex);

	return err;
}

static struct mtd_part *allocate_partition(struct mtd_info *master,
			const struct mtd_partition *part, int partno,
			uint64_t cur_offset)
{
	struct mtd_part *slave;
	char *name;

	/* allocate the partition structure */
	slave = kzalloc(sizeof(*slave), GFP_KERNEL);
	name = kstrdup(part->name, GFP_KERNEL);
	if (!name || !slave) {
		printk(KERN_ERR"memory allocation error while creating partitions for \"%s\"\n",
		       master->name);
		kfree(name);
		kfree(slave);
		return ERR_PTR(-ENOMEM);
	}

	/* set up the MTD object for this partition */
	slave->mtd.type = master->type;
	slave->mtd.flags = master->flags & ~part->mask_flags;
	slave->mtd.size = part->size;
	slave->mtd.writesize = master->writesize;
	slave->mtd.writebufsize = master->writebufsize;
	slave->mtd.oobsize = master->oobsize;
	slave->mtd.oobavail = master->oobavail;
	slave->mtd.subpage_sft = master->subpage_sft;
#if defined(CONFIG_SYNO_LSP_RTD1619)
	slave->mtd.pairing = master->pairing;
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	slave->mtd.name = name;
	slave->mtd.owner = master->owner;

	/* NOTE: Historically, we didn't arrange MTDs as a tree out of
	 * concern for showing the same data in multiple partitions.
	 * However, it is very useful to have the master node present,
	 * so the MTD_PARTITIONED_MASTER option allows that. The master
	 * will have device nodes etc only if this is set, so make the
	 * parent conditional on that option. Note, this is a way to
	 * distinguish between the master and the partition in sysfs.
	 */
	slave->mtd.dev.parent = IS_ENABLED(CONFIG_MTD_PARTITIONED_MASTER) ?
				&master->dev :
				master->dev.parent;

	slave->mtd._read = part_read;
	slave->mtd._write = part_write;

	if (master->_panic_write)
		slave->mtd._panic_write = part_panic_write;

	if (master->_point && master->_unpoint) {
		slave->mtd._point = part_point;
		slave->mtd._unpoint = part_unpoint;
	}

	if (master->_get_unmapped_area)
		slave->mtd._get_unmapped_area = part_get_unmapped_area;
	if (master->_read_oob)
		slave->mtd._read_oob = part_read_oob;
	if (master->_write_oob)
		slave->mtd._write_oob = part_write_oob;
	if (master->_read_user_prot_reg)
		slave->mtd._read_user_prot_reg = part_read_user_prot_reg;
	if (master->_read_fact_prot_reg)
		slave->mtd._read_fact_prot_reg = part_read_fact_prot_reg;
	if (master->_write_user_prot_reg)
		slave->mtd._write_user_prot_reg = part_write_user_prot_reg;
	if (master->_lock_user_prot_reg)
		slave->mtd._lock_user_prot_reg = part_lock_user_prot_reg;
	if (master->_get_user_prot_info)
		slave->mtd._get_user_prot_info = part_get_user_prot_info;
	if (master->_get_fact_prot_info)
		slave->mtd._get_fact_prot_info = part_get_fact_prot_info;
	if (master->_sync)
		slave->mtd._sync = part_sync;
	if (!partno && !master->dev.class && master->_suspend &&
	    master->_resume) {
			slave->mtd._suspend = part_suspend;
			slave->mtd._resume = part_resume;
	}
	if (master->_writev)
		slave->mtd._writev = part_writev;
	if (master->_lock)
		slave->mtd._lock = part_lock;
	if (master->_unlock)
		slave->mtd._unlock = part_unlock;
	if (master->_is_locked)
		slave->mtd._is_locked = part_is_locked;
	if (master->_block_isreserved)
		slave->mtd._block_isreserved = part_block_isreserved;
	if (master->_block_isbad)
		slave->mtd._block_isbad = part_block_isbad;
	if (master->_block_markbad)
		slave->mtd._block_markbad = part_block_markbad;
#if defined(CONFIG_SYNO_LSP_RTD1619)

	if (master->_get_device)
		slave->mtd._get_device = part_get_device;
	if (master->_put_device)
		slave->mtd._put_device = part_put_device;

#endif /* CONFIG_SYNO_LSP_RTD1619 */
	slave->mtd._erase = part_erase;
	slave->master = master;
	slave->offset = part->offset;

	if (slave->offset == MTDPART_OFS_APPEND)
		slave->offset = cur_offset;
	if (slave->offset == MTDPART_OFS_NXTBLK) {
		slave->offset = cur_offset;
		if (mtd_mod_by_eb(cur_offset, master) != 0) {
			/* Round up to next erasesize */
			slave->offset = (mtd_div_by_eb(cur_offset, master) + 1) * master->erasesize;
			printk(KERN_NOTICE "Moving partition %d: "
			       "0x%012llx -> 0x%012llx\n", partno,
			       (unsigned long long)cur_offset, (unsigned long long)slave->offset);
		}
	}
	if (slave->offset == MTDPART_OFS_RETAIN) {
		slave->offset = cur_offset;
		if (master->size - slave->offset >= slave->mtd.size) {
			slave->mtd.size = master->size - slave->offset
							- slave->mtd.size;
		} else {
			printk(KERN_ERR "mtd partition \"%s\" doesn't have enough space: %#llx < %#llx, disabled\n",
				part->name, master->size - slave->offset,
				slave->mtd.size);
			/* register to preserve ordering */
			goto out_register;
		}
	}
	if (slave->mtd.size == MTDPART_SIZ_FULL)
		slave->mtd.size = master->size - slave->offset;

	printk(KERN_NOTICE "0x%012llx-0x%012llx : \"%s\"\n", (unsigned long long)slave->offset,
		(unsigned long long)(slave->offset + slave->mtd.size), slave->mtd.name);

	/* let's do some sanity checks */
	if (slave->offset >= master->size) {
		/* let's register it anyway to preserve ordering */
		slave->offset = 0;
		slave->mtd.size = 0;
		printk(KERN_ERR"mtd: partition \"%s\" is out of reach -- disabled\n",
			part->name);
		goto out_register;
	}
	if (slave->offset + slave->mtd.size > master->size) {
		slave->mtd.size = master->size - slave->offset;
		printk(KERN_WARNING"mtd: partition \"%s\" extends beyond the end of device \"%s\" -- size truncated to %#llx\n",
			part->name, master->name, (unsigned long long)slave->mtd.size);
	}
	if (master->numeraseregions > 1) {
		/* Deal with variable erase size stuff */
		int i, max = master->numeraseregions;
		u64 end = slave->offset + slave->mtd.size;
		struct mtd_erase_region_info *regions = master->eraseregions;

		/* Find the first erase regions which is part of this
		 * partition. */
		for (i = 0; i < max && regions[i].offset <= slave->offset; i++)
			;
		/* The loop searched for the region _behind_ the first one */
		if (i > 0)
			i--;

		/* Pick biggest erasesize */
		for (; i < max && regions[i].offset < end; i++) {
			if (slave->mtd.erasesize < regions[i].erasesize) {
				slave->mtd.erasesize = regions[i].erasesize;
			}
		}
		BUG_ON(slave->mtd.erasesize == 0);
	} else {
		/* Single erase size */
		slave->mtd.erasesize = master->erasesize;
	}

	if ((slave->mtd.flags & MTD_WRITEABLE) &&
	    mtd_mod_by_eb(slave->offset, &slave->mtd)) {
		/* Doesn't start on a boundary of major erase size */
		/* FIXME: Let it be writable if it is on a boundary of
		 * _minor_ erase size though */
		slave->mtd.flags &= ~MTD_WRITEABLE;
		printk(KERN_WARNING"mtd: partition \"%s\" doesn't start on an erase block boundary -- force read-only\n",
			part->name);
	}
	if ((slave->mtd.flags & MTD_WRITEABLE) &&
	    mtd_mod_by_eb(slave->mtd.size, &slave->mtd)) {
		slave->mtd.flags &= ~MTD_WRITEABLE;
		printk(KERN_WARNING"mtd: partition \"%s\" doesn't end on an erase block -- force read-only\n",
			part->name);
	}

#if defined(CONFIG_SYNO_LSP_RTD1619)
	mtd_set_ooblayout(&slave->mtd, &part_ooblayout_ops);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	slave->mtd.ecclayout = master->ecclayout;
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	slave->mtd.ecc_step_size = master->ecc_step_size;
	slave->mtd.ecc_strength = master->ecc_strength;
	slave->mtd.bitflip_threshold = master->bitflip_threshold;

	if (master->_block_isbad) {
		uint64_t offs = 0;

		while (offs < slave->mtd.size) {
			if (mtd_block_isreserved(master, offs + slave->offset))
				slave->mtd.ecc_stats.bbtblocks++;
			else if (mtd_block_isbad(master, offs + slave->offset))
				slave->mtd.ecc_stats.badblocks++;
			offs += slave->mtd.erasesize;
		}
	}

out_register:
#if defined(MY_ABC_HERE) || defined(MY_ABC_HERE)
	if ((memcmp(part->name, "vender", 7)==0) ||
		(memcmp(part->name, "vendor", 7)==0)) {
		u_char rgbszBuf[128];
		int i = 0, x = 0;
		size_t retlen;
		unsigned int Sum;
		u_char ucSum;
#ifdef MY_ABC_HERE
		int n = 0;
		int MacNumber = 0;
		char rgbLanMac[SYNO_MAC_MAX_NUMBER][6];
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
		char szSerialBuffer[32];
		char *ptr;
		char szSerial[32];
		char szCheckSum[32];
		unsigned long ulchksum = 0;
		unsigned long ulTemp = 0;
#endif /* MY_ABC_HERE */

		part_read(&slave->mtd, 0, 128, &retlen, rgbszBuf);

#ifdef MY_ABC_HERE
		/**
		 * FIXME: current vender structure on arm platform only support
		 * max 4 lans instead of SYNO_MAC_MAX_NUMBER.
		 * If more lans needed, check DSM #52055
		 */
		x = 0;
		switch (giVenderFormatVersion) {
		case 1:
			MacNumber = 4;
			break;
		case 2:
			MacNumber = 8;
			break;
		default:
			printk(KERN_ERR "Undefined verder version %d\n", giVenderFormatVersion);
		}

		for (n = 0; n < MacNumber; n++) {
			for (Sum=0,ucSum=0,i=0; i<6; i++) {
				Sum+=rgbszBuf[i+x];
				ucSum+=rgbszBuf[i+x];
				rgbLanMac[n][i] = rgbszBuf[i+x];
			}
			x+=6;

			if (0==Sum) {
				printk("vender Mac%d doesn't set ucSum:0x%02x Buf:0x%02x Sum:%d.\n",
						n, ucSum, rgbszBuf[x], Sum);
			} else if (ucSum!=rgbszBuf[x]) {
				printk("vender Mac%d checksum error ucSum:0x%02x Buf:0x%02x Sum:%d.\n",
						n, ucSum, rgbszBuf[x], Sum);
				grgbLanMac[n][0] = '\0';
			} else {
				printk("vender Mac%d address : %02x:%02x:%02x:%02x:%02x:%02x\n",n,rgbLanMac[n][0],
												  rgbLanMac[n][1],
												  rgbLanMac[n][2],
												  rgbLanMac[n][3],
												  rgbLanMac[n][4],
												  rgbLanMac[n][5]);
				snprintf(grgbLanMac[n], sizeof(grgbLanMac),
						"%02x%02x%02x%02x%02x%02x",
				rgbLanMac[n][0],
				rgbLanMac[n][1],
				rgbLanMac[n][2],
				rgbLanMac[n][3],
				rgbLanMac[n][4],
				rgbLanMac[n][5]);
			}

			x++;
		}
#endif /* MY_ABC_HERE */
#ifdef MY_ABC_HERE
		memset(szSerial, 0, sizeof(szSerial));
		memset(szCheckSum, 0, sizeof(szCheckSum));
		memset(gszSerialNum, 0, 32);
		memcpy(szSerialBuffer, &(rgbszBuf[32]), 32);

		// this is new defined SN
		if (0 == strncmp(szSerialBuffer, SYNO_SN_12_SIG,strlen(SYNO_SN_12_SIG))) {
			//paring serial number with format 'SN=1350KKN99999'
			ptr = strstr(szSerialBuffer, SYNO_SN_TAG);
			if (NULL == ptr) {
				printk("no serial tag found, serial buffer='%s'\n", szSerialBuffer);
				goto SKIP_SERIAL;
			}
			ptr += strlen(SYNO_SN_TAG);
			i = 0;
			while (0 != *ptr && ',' != *ptr) {
				szSerial[i++] = *ptr;
				ptr++;
			}
			szSerial[i] = '\0';

			//paring serial number with format 'CHK=125'
			ptr = strstr(szSerialBuffer, SYNO_CHKSUM_TAG);
			if (NULL == ptr) {
				printk("no checksum tag found, serial buffer='%s'\n", szSerialBuffer);
				goto SKIP_SERIAL;
			}
			ptr += strlen(SYNO_CHKSUM_TAG);
			i = 0;
			while (0 != *ptr && ',' != *ptr) {
				szCheckSum[i++] = *ptr;
				ptr++;
			}
			szCheckSum[i] = '\0';

			//calculate checksum
			for (i = 0 ; i < strlen(szSerial); i++) {
				ulchksum += szSerial[i];
			}

			//------ check checksum ------
			if (0 != kstrtoul(szCheckSum, 10, &ulTemp)) {
				printk("string conversion error: '%s'\n", szCheckSum);
				goto SKIP_SERIAL;
			} else if (ulchksum != ulTemp) {
				printk("serial number checksum error, serial='%s', checksum='%lu' not '%lu'\n", szSerial, ulchksum, ulTemp);
				goto SKIP_SERIAL;
			}
		} else {
			unsigned char ucChkSum = 0;
			//calculate checksum
			for (i = 0 ; i < 10; i++) {
				ucChkSum += szSerialBuffer[i];
			}
			//------ check checksum ------
			if (ucChkSum != szSerialBuffer[10]) {
				printk("serial number checksum error, serial='%s', checksum='%d' not '%d'", szSerialBuffer, ucChkSum, szSerialBuffer[10]);
				goto SKIP_SERIAL;
			} else {
				memcpy(szSerial, szSerialBuffer, 10);
			}
		}
		snprintf(gszSerialNum, 32, "%s", szSerial);
SKIP_SERIAL:
		printk("serial number='%s'", gszSerialNum);

		//read custom serial number out, it is in the vender partion shift 64 bytes.
		x = 64;
		for (Sum=0,ucSum=0,i=0; i<31; i++) {
			Sum+=rgbszBuf[i+x];
			ucSum+=rgbszBuf[i+x];
			gszCustomSerialNum[i] = rgbszBuf[i+x];
		}
		x+=31;
		if (Sum==0 || ucSum!=rgbszBuf[x]) {
			for (i=0; i<32; i++) {
				gszCustomSerialNum[i] = 0;
			}
		} else {
			printk("Custom Serial Number: %s\n", gszCustomSerialNum);
		}
#endif /* MY_ABC_HERE */
	}
#endif /* MY_ABC_HERE || MY_ABC_HERE */
	return slave;
}

static ssize_t mtd_partition_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtd_info *mtd = dev_get_drvdata(dev);
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	return snprintf(buf, PAGE_SIZE, "%lld\n", part->offset);
}

static DEVICE_ATTR(offset, S_IRUGO, mtd_partition_offset_show, NULL);

static const struct attribute *mtd_partition_attrs[] = {
	&dev_attr_offset.attr,
	NULL
};

static int mtd_add_partition_attrs(struct mtd_part *new)
{
	int ret = sysfs_create_files(&new->mtd.dev.kobj, mtd_partition_attrs);
	if (ret)
		printk(KERN_WARNING
		       "mtd: failed to create partition attrs, err=%d\n", ret);
	return ret;
}

int mtd_add_partition(struct mtd_info *master, const char *name,
		      long long offset, long long length)
{
	struct mtd_partition part;
	struct mtd_part *new;
	int ret = 0;

	/* the direct offset is expected */
	if (offset == MTDPART_OFS_APPEND ||
	    offset == MTDPART_OFS_NXTBLK)
		return -EINVAL;

	if (length == MTDPART_SIZ_FULL)
		length = master->size - offset;

	if (length <= 0)
		return -EINVAL;

#if defined(CONFIG_SYNO_LSP_RTD1619)
	memset(&part, 0, sizeof(part));
#endif /* CONFIG_SYNO_LSP_RTD1619 */
	part.name = name;
	part.size = length;
	part.offset = offset;
#if defined(CONFIG_SYNO_LSP_RTD1619)
//do nothing
#else /* CONFIG_SYNO_LSP_RTD1619 */
	part.mask_flags = 0;
	part.ecclayout = NULL;
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	new = allocate_partition(master, &part, -1, offset);
	if (IS_ERR(new))
		return PTR_ERR(new);

	mutex_lock(&mtd_partitions_mutex);
	list_add(&new->list, &mtd_partitions);
	mutex_unlock(&mtd_partitions_mutex);

	ret = add_mtd_device(&new->mtd);
	if (ret)
		goto err_remove_part;

	mtd_add_partition_attrs(new);

	return 0;

err_remove_part:
	mutex_lock(&mtd_partitions_mutex);
	list_del(&new->list);
	mutex_unlock(&mtd_partitions_mutex);

	free_partition(new);

	return ret;
}
EXPORT_SYMBOL_GPL(mtd_add_partition);

int mtd_del_partition(struct mtd_info *master, int partno)
{
	struct mtd_part *slave, *next;
	int ret = -EINVAL;

	mutex_lock(&mtd_partitions_mutex);
	list_for_each_entry_safe(slave, next, &mtd_partitions, list)
		if ((slave->master == master) &&
		    (slave->mtd.index == partno)) {
			sysfs_remove_files(&slave->mtd.dev.kobj,
					   mtd_partition_attrs);
			ret = del_mtd_device(&slave->mtd);
			if (ret < 0)
				break;

			list_del(&slave->list);
			free_partition(slave);
			break;
		}
	mutex_unlock(&mtd_partitions_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(mtd_del_partition);

/*
 * This function, given a master MTD object and a partition table, creates
 * and registers slave MTD objects which are bound to the master according to
 * the partition definitions.
 *
 * For historical reasons, this function's caller only registers the master
 * if the MTD_PARTITIONED_MASTER config option is set.
 */

int add_mtd_partitions(struct mtd_info *master,
		       const struct mtd_partition *parts,
		       int nbparts)
{
	struct mtd_part *slave;
	uint64_t cur_offset = 0;
	int i, ret;

	printk(KERN_NOTICE "Creating %d MTD partitions on \"%s\":\n", nbparts, master->name);

	for (i = 0; i < nbparts; i++) {
		slave = allocate_partition(master, parts + i, i, cur_offset);
		if (IS_ERR(slave)) {
			ret = PTR_ERR(slave);
			goto err_del_partitions;
		}

		mutex_lock(&mtd_partitions_mutex);
		list_add(&slave->list, &mtd_partitions);
		mutex_unlock(&mtd_partitions_mutex);

		ret = add_mtd_device(&slave->mtd);
		if (ret) {
			mutex_lock(&mtd_partitions_mutex);
			list_del(&slave->list);
			mutex_unlock(&mtd_partitions_mutex);

			free_partition(slave);
			goto err_del_partitions;
		}

		mtd_add_partition_attrs(slave);

		cur_offset = slave->offset + slave->mtd.size;
	}

	return 0;

err_del_partitions:
	del_mtd_partitions(master);

	return ret;
}

static DEFINE_SPINLOCK(part_parser_lock);
static LIST_HEAD(part_parsers);

#if defined(CONFIG_SYNO_LSP_RTD1619)
static struct mtd_part_parser *mtd_part_parser_get(const char *name)
#else /* CONFIG_SYNO_LSP_RTD1619 */
static struct mtd_part_parser *get_partition_parser(const char *name)
#endif /* CONFIG_SYNO_LSP_RTD1619 */
{
	struct mtd_part_parser *p, *ret = NULL;

	spin_lock(&part_parser_lock);

	list_for_each_entry(p, &part_parsers, list)
		if (!strcmp(p->name, name) && try_module_get(p->owner)) {
			ret = p;
			break;
		}

	spin_unlock(&part_parser_lock);

	return ret;
}

#if defined(CONFIG_SYNO_LSP_RTD1619)
static inline void mtd_part_parser_put(const struct mtd_part_parser *p)
{
	module_put(p->owner);
}

/*
 * Many partition parsers just expected the core to kfree() all their data in
 * one chunk. Do that by default.
 */
static void mtd_part_parser_cleanup_default(const struct mtd_partition *pparts,
					    int nr_parts)
{
	kfree(pparts);
}
#else /* CONFIG_SYNO_LSP_RTD1619 */
#define put_partition_parser(p) do { module_put((p)->owner); } while (0)
#endif /* CONFIG_SYNO_LSP_RTD1619 */

#if defined(CONFIG_SYNO_LSP_RTD1619)
int __register_mtd_parser(struct mtd_part_parser *p, struct module *owner)
#else /* CONFIG_SYNO_LSP_RTD1619 */
void register_mtd_parser(struct mtd_part_parser *p)
#endif /* CONFIG_SYNO_LSP_RTD1619 */
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	p->owner = owner;

	if (!p->cleanup)
		p->cleanup = &mtd_part_parser_cleanup_default;

#endif /* CONFIG_SYNO_LSP_RTD1619 */
	spin_lock(&part_parser_lock);
	list_add(&p->list, &part_parsers);
	spin_unlock(&part_parser_lock);
#if defined(CONFIG_SYNO_LSP_RTD1619)

	return 0;
#endif /* CONFIG_SYNO_LSP_RTD1619 */
}
#if defined(CONFIG_SYNO_LSP_RTD1619)
EXPORT_SYMBOL_GPL(__register_mtd_parser);
#else /* CONFIG_SYNO_LSP_RTD1619 */
EXPORT_SYMBOL_GPL(register_mtd_parser);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

void deregister_mtd_parser(struct mtd_part_parser *p)
{
	spin_lock(&part_parser_lock);
	list_del(&p->list);
	spin_unlock(&part_parser_lock);
}
EXPORT_SYMBOL_GPL(deregister_mtd_parser);

/*
 * Do not forget to update 'parse_mtd_partitions()' kerneldoc comment if you
 * are changing this array!
 */
static const char * const default_mtd_part_types[] = {
	"cmdlinepart",
	"ofpart",
	NULL
};

/**
 * parse_mtd_partitions - parse MTD partitions
 * @master: the master partition (describes whole MTD device)
 * @types: names of partition parsers to try or %NULL
#if defined(CONFIG_SYNO_LSP_RTD1619)
 * @pparts: info about partitions found is returned here
#else // CONFIG_SYNO_LSP_RTD1619
 * @pparts: array of partitions found is returned here
#endif // CONFIG_SYNO_LSP_RTD1619
 * @data: MTD partition parser-specific data
 *
 * This function tries to find partition on MTD device @master. It uses MTD
 * partition parsers, specified in @types. However, if @types is %NULL, then
 * the default list of parsers is used. The default list contains only the
 * "cmdlinepart" and "ofpart" parsers ATM.
 * Note: If there are more then one parser in @types, the kernel only takes the
 * partitions parsed out by the first parser.
 *
 * This function may return:
 * o a negative error code in case of failure
#if defined(CONFIG_SYNO_LSP_RTD1619)
 * o zero otherwise, and @pparts will describe the partitions, number of
 *   partitions, and the parser which parsed them. Caller must release
 *   resources with mtd_part_parser_cleanup() when finished with the returned
 *   data.
#else // CONFIG_SYNO_LSP_RTD1619
 * o zero if no partitions were found
 * o a positive number of found partitions, in which case on exit @pparts will
 *   point to an array containing this number of &struct mtd_info objects.
#endif // CONFIG_SYNO_LSP_RTD1619
 */
int parse_mtd_partitions(struct mtd_info *master, const char *const *types,
#if defined(CONFIG_SYNO_LSP_RTD1619)
			 struct mtd_partitions *pparts,
#else /* CONFIG_SYNO_LSP_RTD1619 */
			 struct mtd_partition **pparts,
#endif /* CONFIG_SYNO_LSP_RTD1619 */
			 struct mtd_part_parser_data *data)
{
	struct mtd_part_parser *parser;
	int ret, err = 0;

	if (!types)
		types = default_mtd_part_types;

	for ( ; *types; types++) {
		pr_debug("%s: parsing partitions %s\n", master->name, *types);
#if defined(CONFIG_SYNO_LSP_RTD1619)
		parser = mtd_part_parser_get(*types);
#else /* CONFIG_SYNO_LSP_RTD1619 */
		parser = get_partition_parser(*types);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		if (!parser && !request_module("%s", *types))
#if defined(CONFIG_SYNO_LSP_RTD1619)
			parser = mtd_part_parser_get(*types);
#else /* CONFIG_SYNO_LSP_RTD1619 */
			parser = get_partition_parser(*types);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		pr_debug("%s: got parser %s\n", master->name,
			 parser ? parser->name : NULL);
		if (!parser)
			continue;
#if defined(CONFIG_SYNO_LSP_RTD1619)
		ret = (*parser->parse_fn)(master, &pparts->parts, data);
#else /* CONFIG_SYNO_LSP_RTD1619 */
		ret = (*parser->parse_fn)(master, pparts, data);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		pr_debug("%s: parser %s: %i\n",
			 master->name, parser->name, ret);
#if defined(CONFIG_SYNO_LSP_RTD1619)
//do nothing
#else /* CONFIG_SYNO_LSP_RTD1619 */
		put_partition_parser(parser);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		if (ret > 0) {
			printk(KERN_NOTICE "%d %s partitions found on MTD device %s\n",
			       ret, parser->name, master->name);
#if defined(CONFIG_SYNO_LSP_RTD1619)
			pparts->nr_parts = ret;
			pparts->parser = parser;
			return 0;
#else /* CONFIG_SYNO_LSP_RTD1619 */
			return ret;
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		}
#if defined(CONFIG_SYNO_LSP_RTD1619)
		mtd_part_parser_put(parser);
#endif /* CONFIG_SYNO_LSP_RTD1619 */
		/*
		 * Stash the first error we see; only report it if no parser
		 * succeeds
		 */
		if (ret < 0 && !err)
			err = ret;
	}
	return err;
}

#if defined(CONFIG_SYNO_LSP_RTD1619)
void mtd_part_parser_cleanup(struct mtd_partitions *parts)
{
	const struct mtd_part_parser *parser;

	if (!parts)
		return;

	parser = parts->parser;
	if (parser) {
		if (parser->cleanup)
			parser->cleanup(parts->parts, parts->nr_parts);

		mtd_part_parser_put(parser);
	}
}

#endif /* CONFIG_SYNO_LSP_RTD1619 */
int mtd_is_partition(const struct mtd_info *mtd)
{
	struct mtd_part *part;
	int ispart = 0;

	mutex_lock(&mtd_partitions_mutex);
	list_for_each_entry(part, &mtd_partitions, list)
		if (&part->mtd == mtd) {
			ispart = 1;
			break;
		}
	mutex_unlock(&mtd_partitions_mutex);

	return ispart;
}
EXPORT_SYMBOL_GPL(mtd_is_partition);

/* Returns the size of the entire flash chip */
uint64_t mtd_get_device_size(const struct mtd_info *mtd)
{
	if (!mtd_is_partition(mtd))
		return mtd->size;

#if defined(CONFIG_SYNO_LSP_RTD1619)
	return mtd_to_part(mtd)->master->size;
#else /* CONFIG_SYNO_LSP_RTD1619 */
	return PART(mtd)->master->size;
#endif /* CONFIG_SYNO_LSP_RTD1619 */
}
EXPORT_SYMBOL_GPL(mtd_get_device_size);

#ifdef MY_ABC_HERE
int SYNOMTDModifyPartInfo(struct mtd_info *mtd, unsigned long offset, unsigned long length)
{
#if defined(CONFIG_SYNO_LSP_RTD1619)
	struct mtd_part *part = mtd_to_part(mtd);
#else /* CONFIG_SYNO_LSP_RTD1619 */
	struct mtd_part *part = PART(mtd);
#endif /* CONFIG_SYNO_LSP_RTD1619 */

	part->offset = offset;
	part->offset &= part->master->size-1;

	mtd->size = length;

	if (part->offset + mtd->size > part->master->size) {
		return -EFAULT;
	}

	return 0;
}
#endif /* MY_ABC_HERE */
