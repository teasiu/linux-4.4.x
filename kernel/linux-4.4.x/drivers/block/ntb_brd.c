/*
 *   This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2020 Synology Corporation. All rights reserved.
 *
 *   PCIe NTB RamDisk driver
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/sizes.h>
#include <linux/ntb.h>
#include <linux/ntb_transport.h>
#include <linux/mutex.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define SECTOR_SHIFT	9
#define DEFAULT_NTB_BRD_NAME	"ntb_md0"

MODULE_LICENSE("Dual BSD/GPL");

struct ntb_brd_ctx {
	struct device *client_dev;
	int num_devices;
};

struct ntb_brd_device {
	int brd_minor;
	struct request_queue	*brd_queue;
	struct gendisk		*brd_disk;
	struct list_head	brd_list;
	struct rw_semaphore	device_lock;
	bool access_remote;
	bool unclean;
	struct ntb_transport_raw_block *raw_block;
};

static struct ntb_brd_ctx *ntb_brd = NULL;
static struct dentry *ntb_brd_debugfs_dir = NULL;
static LIST_HEAD(ntb_brd_devices);
static DEFINE_MUTEX(ntb_brd_devices_mutex);

static void copy_to_remote(struct ntb_brd_device *ntb_brd_device, const void *src,
			sector_t sector, size_t n)
{
	char __iomem *dst = (char __iomem *)(ntb_brd_device->raw_block->tx_buff + (sector << SECTOR_SHIFT));

#ifdef ARCH_HAS_NOCACHE_UACCESS
	/*
	 * Using non-temporal mov to improve performance on non-cached
	 * writes, even though we aren't actually copying from user space.
	*/
	__copy_from_user_inatomic_nocache(dst, src, n);
#else
	memcpy_toio(dst, src, n);
#endif /* ARCH_HAS_NOCACHE_UACCESS */
}

static void copy_from_remote(void *dst, struct ntb_brd_device *ntb_brd_device,
			sector_t sector, size_t n)
{
	char __iomem *src = (char __iomem *)(ntb_brd_device->raw_block->tx_buff + (sector << SECTOR_SHIFT));

	memcpy_fromio(dst, src, n);
}

static void copy_to_local(struct ntb_brd_device *ntb_brd_device, const void *src,
			sector_t sector, size_t n)
{
	void *dst = (void *)(ntb_brd_device->raw_block->rx_buff + (sector << SECTOR_SHIFT));

	memcpy(dst, src, n);
}

static void copy_from_local(void *dst, struct ntb_brd_device *ntb_brd_device,
			sector_t sector, size_t n)
{
	void *src = (void *)(ntb_brd_device->raw_block->rx_buff + (sector << SECTOR_SHIFT));

	memcpy(dst, src, n);
}

static int ntb_brd_do_bvec(struct ntb_brd_device *ntb_brd_device, struct page *page,
			unsigned int len, unsigned int off, int rw,
			sector_t sector)
{
	void *mem;
	int ret = 0;

	down_read(&ntb_brd_device->device_lock);
	mem = kmap_atomic(page);
	if (ntb_brd_device->access_remote == true) {
		if (unlikely(!ntb_brd_device->raw_block || ntb_brd_device->raw_block->link_is_up == false)) {
			ret = -EIO;
			goto out;
		}

		if (rw == READ) {
			copy_from_remote(mem + off, ntb_brd_device, sector, len);
			flush_dcache_page(page);
		} else {
			flush_dcache_page(page);
			copy_to_remote(ntb_brd_device, mem + off, sector, len);
		}

		if (unlikely(!ntb_brd_device->raw_block || ntb_brd_device->raw_block->link_is_up == false))
			ret = -EIO;
	} else {
		if (rw == READ) {
			copy_from_local(mem + off, ntb_brd_device, sector, len);
			flush_dcache_page(page);
		} else {
			flush_dcache_page(page);
			copy_to_local(ntb_brd_device, mem + off, sector, len);
		}
	}

out:
	kunmap_atomic(mem);
	up_read(&ntb_brd_device->device_lock);

	return ret;
}

static blk_qc_t ntb_brd_make_request(struct request_queue *q, struct bio *bio)
{
	struct block_device *bdev = bio->bi_bdev;
	struct ntb_brd_device *ntb_brd_device = bdev->bd_disk->private_data;
	int rw;
	struct bio_vec bvec;
	sector_t sector;
	struct bvec_iter iter;

	sector = bio->bi_iter.bi_sector;
	if (bio_end_sector(bio) > get_capacity(bdev->bd_disk)) {
		bio_io_error(bio);
		goto out;
	}

	rw = bio_rw(bio);
	if (rw == READA)
		rw = READ;

	bio_for_each_segment(bvec, bio, iter) {
		unsigned int len = bvec.bv_len;
		int err;

		err = ntb_brd_do_bvec(ntb_brd_device, bvec.bv_page, len,
					bvec.bv_offset, rw, sector);
		if (err) {
			bio_io_error(bio);
			goto out;
		}
		sector += len >> SECTOR_SHIFT;
	}

	bio_endio(bio);
out:
	return BLK_QC_T_NONE;
}

static int ntb_brd_rw_page(struct block_device *bdev, sector_t sector,
		       struct page *page, int rw)
{
	struct ntb_brd_device *ntb_brd_device = bdev->bd_disk->private_data;
	int err = ntb_brd_do_bvec(ntb_brd_device, page, PAGE_CACHE_SIZE, 0, rw, sector);
	page_endio(page, rw & WRITE, err);
	return err;
}

static const struct block_device_operations ntb_brd_fops = {
	.owner =		THIS_MODULE,
	.rw_page =		ntb_brd_rw_page,
};

static int debugfs_show_access_remote(void *data, u64 *val)
{
	struct ntb_brd_device *brd = (struct ntb_brd_device *)data;

	*val = brd->access_remote;
	return 0;
}
static int debugfs_set_access_remote(void *data, u64 val)
{
	struct ntb_brd_device *brd = (struct ntb_brd_device *)data;
	bool set_remote = (bool)val;
	int ret = 0;

	down_write(&brd->device_lock);
	if (set_remote == true && brd->raw_block && brd->raw_block->link_is_up == true)
		brd->access_remote = true;
	else if (set_remote == false)
		brd->access_remote = false;
	else
		ret = -1;
	up_write(&brd->device_lock);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_ntb_brd_access_remote, debugfs_show_access_remote,
		debugfs_set_access_remote, "%llu\n");

static int debugfs_show_unclean(void *data, u64 *val)
{
	struct ntb_brd_device *brd = (struct ntb_brd_device *)data;

	*val = brd->unclean;
	return 0;
}
static int debugfs_set_unclean(void *data, u64 val)
{
	struct ntb_brd_device *brd = (struct ntb_brd_device *)data;
	bool set_unclean = (bool)val;

	brd->unclean = set_unclean;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_ntb_brd_unclean, debugfs_show_unclean,
		debugfs_set_unclean, "%llu\n");

static int debugfs_show_link_up(void *data, u64 *val)
{
	struct ntb_brd_device *brd = (struct ntb_brd_device *)data;

	if (brd->raw_block)
		*val = brd->raw_block->link_is_up;
	else
		*val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_ntb_brd_link_up, debugfs_show_link_up, NULL, "%llu\n");

static void ntb_brd_event_handler(struct ntb_transport_raw_block *block, int event)
{
	struct ntb_brd_device *brd = block->client_dev;

	if (unlikely(!brd)) {
		WARN_ON_ONCE(1);
		return;
	}

	if (event == NTB_BRD_LINK_DOWN) {
		brd->unclean = true;
		if (brd->brd_disk)
			printk(KERN_INFO "NTB block device %s receive LINK_DOWN event", brd->brd_disk->disk_name);
	} else if (event == NTB_BRD_LINK_UP) {
		if (brd->brd_disk)
			printk(KERN_INFO "NTB block device %s receive LINK_UP event", brd->brd_disk->disk_name);
	}
}

static struct ntb_brd_device *ntb_brd_alloc(const char *name, struct ntb_transport_raw_block *block)
{
	struct ntb_brd_device *brd;
	struct gendisk *disk;
	struct dentry *dir;
	struct dentry *dentry;

	if (ntb_brd == NULL || strlen(name) > DISK_NAME_LEN - 1)
		return NULL;

	brd = kzalloc(sizeof(*brd), GFP_KERNEL);
	if (!brd)
		goto out;

	brd->brd_minor = ntb_brd->num_devices;
	brd->brd_queue = blk_alloc_queue(GFP_KERNEL);
	if (!brd->brd_queue)
		goto out_free_dev;

	blk_queue_make_request(brd->brd_queue, ntb_brd_make_request);
	blk_queue_max_hw_sectors(brd->brd_queue, 1024);
	blk_queue_bounce_limit(brd->brd_queue, BLK_BOUNCE_ANY);
	blk_queue_physical_block_size(brd->brd_queue, PAGE_SIZE);

	disk = brd->brd_disk = alloc_disk(1);
	if (!disk)
		goto out_free_queue;
	disk->major		= NTB_RAMDISK_MAJOR;
	disk->first_minor	= brd->brd_minor;
	disk->fops		= &ntb_brd_fops;
	disk->private_data	= brd;
	disk->queue		= brd->brd_queue;
	disk->flags		= GENHD_FL_EXT_DEVT;
	snprintf(disk->disk_name, DISK_NAME_LEN, "%s", name);
	set_capacity(disk, block->size >> SECTOR_SHIFT);
	init_rwsem(&brd->device_lock);
	brd->raw_block = block;
	block->client_dev = brd;

	dir = debugfs_create_dir(name, ntb_brd_debugfs_dir);
	if (!dir) {
		printk(KERN_ERR "ntb_brd failed to create debugfs dir for %s\n", name);
		goto out_free_gendisk;
	}

	dentry = debugfs_create_file("access_remote", 0600,
			dir, brd, &fops_ntb_brd_access_remote);
	if (!dentry)
		goto out_free_dir;

	dentry = debugfs_create_file("unclean", 0600,
			dir, brd, &fops_ntb_brd_unclean);
	if (!dentry)
		goto out_free_dir;

	dentry = debugfs_create_file("link_up", 0400,
			dir, brd, &fops_ntb_brd_link_up);
	if (!dentry) {
		goto out_free_dir;
	}

	return brd;

out_free_dir:
	debugfs_remove_recursive(dir);
out_free_gendisk:
	kfree(brd->brd_disk);
out_free_queue:
	blk_cleanup_queue(brd->brd_queue);
out_free_dev:
	kfree(brd);
out:
	return NULL;
}

static struct kobject *brd_probe(dev_t dev, int *part, void *data)
{
	struct ntb_brd_device *brd;
	struct kobject *kobj = NULL;

	mutex_lock(&ntb_brd_devices_mutex);
	list_for_each_entry(brd, &ntb_brd_devices, brd_list) {
		if (brd->brd_minor == MINOR(dev)) {
			kobj = get_disk(brd->brd_disk);
			mutex_unlock(&ntb_brd_devices_mutex);
			return kobj;
		}
	}
	mutex_unlock(&ntb_brd_devices_mutex);
	return kobj;
}

static int ntb_brd_alloc_device(const char *name, int idx)
{
	struct ntb_dev *ntb;
	struct ntb_brd_device *brd;
	struct ntb_transport_raw_block *block = NULL;

	if (ntb_brd == NULL)
		return -ENODEV;

	ntb = dev_ntb(ntb_brd->client_dev->parent);

	mutex_lock(&ntb_brd_devices_mutex);
	block = ntb_transport_create_block(ntb_brd->client_dev, idx, ntb_brd_event_handler);
	if (!block) {
		printk(KERN_ERR "ntb_transport_create_block() return failed for %s, idx = %d!\n",
				name, idx);
		mutex_unlock(&ntb_brd_devices_mutex);
		return -ENODEV;
	}

	brd = ntb_brd_alloc(name, block);
	if (!brd) {
		ntb_transport_free_block(block);
		mutex_unlock(&ntb_brd_devices_mutex);
		return -ENOMEM;
	}
	list_add_tail(&brd->brd_list, &ntb_brd_devices);
	add_disk(brd->brd_disk);
	ntb_brd->num_devices++;
	mutex_unlock(&ntb_brd_devices_mutex);

	return 0;
}

static int ntb_brd_register_blkdev(void)
{
	if (register_blkdev(NTB_RAMDISK_MAJOR, "ntb_ramdisk")) {
		printk(KERN_ERR "Failed to register blkdev for ntb_ramdisk\n");
		return -EIO;
	}

	blk_register_region(MKDEV(NTB_RAMDISK_MAJOR, 0), 1UL << MINORBITS,
			THIS_MODULE, brd_probe, NULL, NULL);

	return 0;
}

static void ntb_brd_unregister_blkdev(void)
{
	blk_unregister_region(MKDEV(NTB_RAMDISK_MAJOR, 0), 1UL << MINORBITS);
	unregister_blkdev(NTB_RAMDISK_MAJOR, "ntb_ramdisk");
}

static void ntb_brd_free_devices(void)
{
	struct ntb_brd_device *ntb_brd_device, *next;

	mutex_lock(&ntb_brd_devices_mutex);
	list_for_each_entry_safe(ntb_brd_device, next, &ntb_brd_devices, brd_list) {
		list_del(&ntb_brd_device->brd_list);
		del_gendisk(ntb_brd_device->brd_disk);
		put_disk(ntb_brd_device->brd_disk);
		blk_cleanup_queue(ntb_brd_device->brd_queue);
		ntb_transport_free_block(ntb_brd_device->raw_block);
		kfree(ntb_brd_device);
	}
	mutex_unlock(&ntb_brd_devices_mutex);
}

static int ntb_brd_debugfs_setup(void)
{
	if (!debugfs_initialized() || ntb_brd == NULL)
		return -ENODEV;

	if (!ntb_brd_debugfs_dir) {
		ntb_brd_debugfs_dir = debugfs_create_dir(KBUILD_MODNAME, NULL);
		if (!ntb_brd_debugfs_dir)
			return -EINVAL;
	}

	return 0;
}

static int ntb_brd_probe(struct device *client_dev)
{
	int ret = 0;

	if (ntb_brd)
		return -EEXIST;

	ntb_brd = kzalloc(sizeof(*ntb_brd), GFP_KERNEL);
	if (!ntb_brd)
		return -ENOMEM;
	ntb_brd->client_dev = client_dev;

	ret = ntb_brd_register_blkdev();
	if (ret)
		goto free_ntb_brd;

	ret = ntb_brd_debugfs_setup();
	if (ret)
		goto unregister_blkdev;

	printk(KERN_INFO "Attempt to create default ntb block device: %s\n", DEFAULT_NTB_BRD_NAME);
	ret = ntb_brd_alloc_device(DEFAULT_NTB_BRD_NAME, NTB_RAW_BLOCK_ID_MD_JOURNAL);
	if (ret) {
		printk(KERN_ERR "Failed to create ntb_brd device: %s\n", DEFAULT_NTB_BRD_NAME);
		goto remove_debugfs;
	}
	printk(KERN_INFO "Successfully create ntb block device: %s\n", DEFAULT_NTB_BRD_NAME);

	return 0;

remove_debugfs:
	debugfs_remove_recursive(ntb_brd_debugfs_dir);
unregister_blkdev:
	ntb_brd_unregister_blkdev();
free_ntb_brd:
	kfree(ntb_brd);
	ntb_brd = NULL;
	return ret;
}

static void ntb_brd_remove(struct device *client_dev)
{
	debugfs_remove_recursive(ntb_brd_debugfs_dir);
	ntb_brd_free_devices();
	ntb_brd_unregister_blkdev();
	ntb_brd = NULL;
	return;
}

static struct ntb_transport_client ntb_brd_client = {
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.probe = ntb_brd_probe,
	.remove = ntb_brd_remove,
};

static int __init ntb_brd_init_module(void)
{
	int rc;

	rc = ntb_transport_register_client_dev(KBUILD_MODNAME);
	if (rc)
		return rc;
	return ntb_transport_register_client(&ntb_brd_client);
}
module_init(ntb_brd_init_module);

static void __exit ntb_brd_exit_module(void)
{
	ntb_transport_unregister_client(&ntb_brd_client);
	ntb_transport_unregister_client_dev(KBUILD_MODNAME);
}
module_exit(ntb_brd_exit_module);
