#include <linux/buffer_head.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/f2fs.h>
#include <linux/f2fs_fs.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fs_context.h>
#include <linux/fs_struct.h>
#include <linux/fiemap.h>
#include <linux/gfp.h>
#include <linux/hashtable.h>
#include <linux/interrupt.h>
#include <linux/jhash.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/list.h>
#include <linux/mount.h>
#include <linux/module.h>
#include <linux/namei.h>
#include <linux/printk.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/swap.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/unistd.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/vmscan.h>
#include "../../fs/mount.h"
#include <../../fs/f2fs/f2fs.h>

#define NT_frag_info_print(fmt,arg...) \
pr_info("[File_frag] "fmt, ##arg)

#define NT_frag_err_print(fmt,arg...) \
pr_err("[File_frag] "fmt, ##arg)

#define NT_HASH_SIZE 9

#define DEVICE_NAME "nt_defrag_dev"

#define MAJOR_NUMBER 714

#define NT_FIEMAP_MAX_EXTENTS	(UINT_MAX / sizeof(struct fiemap_extent))

#define F2FS_A_SEG_SIZE SZ_2M

static DEFINE_HASHTABLE(path_hash_table, NT_HASH_SIZE);

static dev_t data_dev = 0;

static dev_t df_dev;
static struct cdev c_df_dev;
static struct class *df_class;


static char readahead_monitor_switch_status[2] = { 0 };

static unsigned long long insert_count = 0;

struct path_table_struct {
	unsigned long long value;
	char nt_full_path[1024];
	unsigned int nt_ino;
	unsigned long long nt_file_size;
	struct hlist_node nodex;
};

static int is_inode_directory(struct inode *inode) {
	return S_ISDIR(inode->i_mode);
}

static unsigned long long inode_get_filesize(struct dentry *n_dentry){
	return (((loff_t)n_dentry->d_inode->i_blocks) << 9) + n_dentry->d_inode->i_bytes;
}

int increment_value(unsigned int nt_ino) {
	struct path_table_struct *entry;

	hash_for_each_possible(path_hash_table, entry, nodex, nt_ino) {
		if (nt_ino == entry->nt_ino) {
			entry->value ++;
			return 1;
		}
	}

	return 0;
}

void insert_key_value(unsigned int nt_ino, struct dentry *nt_path_dentry, unsigned long long nt_file_size) {
	struct path_table_struct *entry;
	unsigned int path_str_len = 0;
	char *nt_full_path = NULL;
	char *path_buf = NULL;

	path_buf = (char *)kzalloc(PAGE_SIZE, GFP_ATOMIC);
	if (!path_buf) {
		NT_frag_err_print("Allocate full path memory failed\n");
		return;
	}

	nt_full_path = dentry_path_raw(nt_path_dentry, path_buf, PAGE_SIZE);
	path_str_len = strlen(nt_full_path);
	if (path_str_len > SZ_1K)
		goto OUT;

	entry = kzalloc(sizeof(struct path_table_struct), GFP_ATOMIC);
	if (!entry) {
		NT_frag_err_print("Failed to allocate memory for new pts\n");
		goto OUT;
	}

	strncpy(entry->nt_full_path, nt_full_path, path_str_len);
	entry->nt_ino = nt_ino;
	entry->value = 1;
	entry->nt_file_size = nt_file_size;
	hash_add(path_hash_table, &entry->nodex, nt_ino);
	insert_count++;

OUT:
	if (path_buf) {
		kfree(path_buf);
		path_buf = NULL;
	}
}

static void show_readahead_status(void *data, struct readahead_control *ractl, unsigned long *max_pages)
{
	unsigned long long nt_file_size = 0;

	if (*max_pages < 128)
		return;

	if (readahead_monitor_switch_status[0] != '1')
		return;

	if (data_dev == 0 && strcmp(ractl->file->f_inode->i_sb->s_type->name, "f2fs") == 0 ) {
		if (ractl->file->f_inode->i_sb->s_flags & SB_INLINECRYPT)
			data_dev = ractl->file->f_inode->i_sb->s_dev;
	}

	if (data_dev != ractl->file->f_inode->i_sb->s_dev)
		return;

	if (is_inode_directory(ractl->file->f_inode))
		return;

	if (is_inode_flag_set(ractl->file->f_inode, FI_COMPRESS_RELEASED))
			return;

	nt_file_size = inode_get_filesize(ractl->file->f_path.dentry);
	if(nt_file_size < F2FS_A_SEG_SIZE)
		return;

	ractl->file->f_inode->android_kabi_reserved2++;

	if (ractl->file->f_inode->android_kabi_reserved2 >= 8) {
		if (!increment_value(ractl->file->f_inode->i_ino))
			insert_key_value(ractl->file->f_inode->i_ino, ractl->file->f_path.dentry, nt_file_size);
		ractl->file->f_inode->android_kabi_reserved2 = 0;
	}

}

static int register_readahead_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_ra_tuning_max_page(show_readahead_status, NULL);
	if (ret != 0) {
		NT_frag_err_print("[NT_readahead] register_readahead_hooks failed! ret=%d\n", ret);
		goto out;
	}

out:
	return ret;
}

static void unregister_readahead_hooks(void)
{
	unregister_trace_android_vh_ra_tuning_max_page(show_readahead_status, NULL);
	return;
}

static int get_readahead_files_main(struct seq_file *s, void *unused) {
	struct path_table_struct *entry;
	struct hlist_node *tmp;
	int i;
	int count = 0;

	hash_for_each_safe(path_hash_table, i, tmp, entry, nodex) {
		seq_printf(s, "%s %llu %lu %llu\n", entry->nt_full_path, entry->value, entry->nt_ino, entry->nt_file_size);
		hash_del(&entry->nodex);
		kfree(entry);
		insert_count--;
		count++;
		if(count >= 30) {
			NT_frag_err_print("insert_count go out\n");
			return 0;
		}
	}
	return 0;
}

static int get_readahead_files_open(struct inode *inode, struct file *file)
{
	return  single_open_size(file, get_readahead_files_main, NULL, SZ_32K);
}

struct f2fs_path_fragment_info {
	char path_str[1024];
	unsigned long long file_size;
	int path_not_exist;
	struct fiemap fiemap;
};

struct f2fs_do_defrag_struct {
	char path_str[1024];
	struct f2fs_defragment range;
};

#define IOCTL_GET_FRAGS _IOWR(MAJOR_NUMBER, 1, struct f2fs_path_fragment_info)
#define IOCTL_DO_DEFRAG _IOWR(MAJOR_NUMBER, 2, struct f2fs_do_defrag_struct)

enum {
	FS_GOOD = 0,
	FS_ERR_FS_TYPE,
	FS_ERR_CB,
	FS_NULL_CB,
	FS_NULL_CB_NAME,
	FS_NULL_CB_INODE,
	FS_NULL_SB,
	FS_ERR_CFU,
	FS_ERR_CTU,
	FS_NOT_FOUND,
	FS_NO_FIEMAP,
	FS_NO_IOCTL,
	FS_OVER_EXT,
	FS_ERR_DEFRAG,
	FS_ERR_FIEMAP,
};

struct nt_dir_info {
	struct dir_context ctx;
	struct dentry *dir_dentry;
	struct dentry *alloc_dentry;
	struct dentry *return_dentry;
	struct dentry *find_dentry;
	int target_is_dir;
	char *target_name;
};

static int process_fs_dentry(struct dentry *n_dentry) {

	if (!n_dentry)
		return -FS_NULL_CB;

	if (IS_ERR(n_dentry))
		return -FS_ERR_CB;

	if (!n_dentry->d_name.name)
		return -FS_NULL_CB_NAME;

	if (!n_dentry->d_inode)
		return -FS_NULL_CB_INODE;

	return FS_GOOD;
}

static int find_dir_sub(struct dir_context *ctx, const char *name, int namlen,
       loff_t offset, u64 ino, unsigned int d_type) {
	struct nt_dir_info *subdir_info = container_of(ctx, struct nt_dir_info, ctx);
	struct dentry *alloc_dentry = NULL;
	struct dentry *return_dentry = NULL;
	char real_name[F2FS_NAME_LEN] = {0};
	int ret = 0;
	int is_dir = subdir_info->target_is_dir;

	if (strlen(subdir_info->target_name) != namlen)
		goto OUT;

	if (strncmp(name, subdir_info->target_name, namlen) !=0)
		goto OUT;

	strncpy(real_name, name, namlen);
	real_name[namlen] = '\0';

	if (is_dot_dotdot(real_name, namlen))
		goto OUT;

	alloc_dentry = d_alloc_name(subdir_info->dir_dentry, real_name);
	return_dentry = subdir_info->dir_dentry->d_inode->i_op->lookup(subdir_info->dir_dentry->d_inode, alloc_dentry, 0);

	ret = process_fs_dentry(alloc_dentry);

	if (ret != FS_GOOD)
		goto TRY_RETURN;

	if (is_dir == is_inode_directory(alloc_dentry->d_inode)) {
		subdir_info->find_dentry = alloc_dentry;
		subdir_info->alloc_dentry = alloc_dentry;
		subdir_info->return_dentry = return_dentry;
	}

	goto OUT;

TRY_RETURN:
	ret = process_fs_dentry(return_dentry);

	if (ret != FS_GOOD)
		goto ERR_OUT;

	if(is_dir == is_inode_directory(return_dentry->d_inode)) {
		subdir_info->find_dentry = return_dentry;
		subdir_info->alloc_dentry = alloc_dentry;
		subdir_info->return_dentry = return_dentry;
	}

	goto OUT;

ERR_OUT:
	NT_frag_err_print("=== it's a lost file: %s %d %ld alloc_dentry: %p return_dentry: %p ,ret %d %d ===\n", real_name, namlen, ino, subdir_info->alloc_dentry, subdir_info->find_dentry, ret, is_dir);
OUT:
	return 0;
}

struct super_sb_data {
	struct super_block *sb;
};

static void sb_iterator(struct super_block *sb, void *arg)
{
	struct super_sb_data *sbd = arg;

	if (sb->s_flags & SB_INLINECRYPT)
		sbd->sb = sb;
}

int find_root_dentry(struct vfsmount **nt_vfsmnt, struct dentry **nt_root_dentry) {
	struct file_system_type *type;
	struct super_block *nt_f2fs_sb = NULL;
	struct mount *t_mnt;
	int ret = 0;
	struct super_sb_data iter_sb = {
		.sb = NULL,
	};
	type = get_fs_type("f2fs");
	if (!type) {
		ret = -FS_ERR_FS_TYPE;
		goto ERR_OUT;
	}

	iterate_supers_type(type, sb_iterator, &iter_sb);
	nt_f2fs_sb = iter_sb.sb;
	*nt_root_dentry = nt_f2fs_sb->s_root;
	list_for_each_entry(t_mnt, &nt_f2fs_sb->s_mounts, mnt_instance) {
		*nt_vfsmnt = &t_mnt->mnt;
		break;
	}

	if (!*nt_root_dentry) {
		ret = -FS_NULL_SB;
		goto ERR_OUT;
	}

	ERR_OUT:
		return ret;
}

int find_target_dentry (struct dentry *nt_root_dentry, char *path_str, struct dentry **found_dentry, struct dentry **last_alloc_dentry, struct dentry **last_return_dentry) {
	char* const delim = "/";
	char *split_item;
	char *ioctl_path_str;
	char *tmp_ptr = NULL;
	struct dentry *target_dentry;
	int target_count = 0;
	int target_level = 0;
	int is_found = 1;

	target_dentry = nt_root_dentry;

	ioctl_path_str = path_str;

	tmp_ptr = ioctl_path_str;

	while (*tmp_ptr != '\0') {
		if (*tmp_ptr == '/') {
			target_level++;
		}
		tmp_ptr++;
	}

	tmp_ptr = ioctl_path_str;
	tmp_ptr++;

	while ((split_item = strsep(&tmp_ptr, delim)) != NULL) {
		struct file target_partition_file;
		struct nt_dir_info p_d;
		target_count++;
		p_d.ctx.actor = &find_dir_sub;
		p_d.dir_dentry = target_dentry;
		p_d.target_name = split_item;
		p_d.target_is_dir = 1;

		if (target_count == target_level)
			p_d.target_is_dir = 0;

		target_partition_file.f_mapping = target_dentry->d_inode->i_mapping;
		target_partition_file.f_flags = O_NOATIME;
		target_partition_file.f_inode = target_dentry->d_inode;
		target_partition_file.f_lock = target_dentry->d_inode->i_lock;
		if (target_dentry->d_inode->i_fop->iterate_shared)
			target_dentry->d_inode->i_fop->iterate_shared(&target_partition_file, &p_d.ctx);
		else {
			is_found = 0;
			break;
		}

		if (*last_alloc_dentry) {
			dput(*last_alloc_dentry);
			*last_alloc_dentry = NULL;
		}

		if (*last_return_dentry) {
			dput(*last_return_dentry);
			*last_return_dentry = NULL;
		}

		*last_alloc_dentry = p_d.alloc_dentry;
		*last_return_dentry = p_d.return_dentry;

		if (p_d.find_dentry)
			target_dentry = p_d.find_dentry;
		else {
			is_found = 0;
			break;
		}
	}

	if (is_found)
		*found_dentry = target_dentry;
	else
		*found_dentry = NULL;

	return is_found;
}

static int do_file_defrag_main(struct f2fs_do_defrag_struct *f_defrag) {
	struct f2fs_do_defrag_struct kern_f_defrag;
	struct dentry *target_dentry = NULL;
	struct dentry *nt_root_dentry = NULL;
	struct dentry *last_alloc_dentry = NULL;
	struct dentry *last_return_dentry = NULL;
	struct vfsmount *nt_vfsmnt = NULL;
	int is_found = 1;
	int ret = 0;

	if (copy_from_user(&kern_f_defrag, f_defrag, sizeof(kern_f_defrag))) {
		ret = -FS_ERR_CFU;
		goto OUT;
	}

	ret = find_root_dentry(&nt_vfsmnt, &nt_root_dentry);
	if (ret)
		goto OUT;

	is_found = find_target_dentry(nt_root_dentry, kern_f_defrag.path_str, &target_dentry, &last_alloc_dentry, &last_return_dentry);

	if (is_found && target_dentry) {
		struct file target_file_struct;
		struct inode *target_inode = target_dentry->d_inode;

		if (!target_inode->i_fop->unlocked_ioctl) {
			ret = -FS_NO_IOCTL;
			goto OUT;
		}

		target_file_struct.f_mapping = target_inode->i_mapping;
		target_file_struct.f_flags = O_NOATIME;
		target_file_struct.f_inode = target_inode;
		target_file_struct.f_path.mnt = nt_vfsmnt;
		target_file_struct.f_path.dentry = target_dentry;
		target_file_struct.f_lock = target_inode->i_lock;
		if (!target_inode->i_crypt_info) {
			ret = fscrypt_file_open(target_inode, &target_file_struct);
			if (ret) {
				NT_frag_err_print("fscrypt_file_open failed %d\n", ret);
				goto OUT;
			}
		}
		ret = target_inode->i_fop->unlocked_ioctl(&target_file_struct, F2FS_IOC_DEFRAGMENT, (unsigned long)&f_defrag->range);
		if (ret) {
			NT_frag_err_print("Defrag failed %d\n", ret);
			ret = -FS_ERR_DEFRAG;
			goto OUT;
		} else
			NT_frag_err_print("Defrag file\n");
	}

	if (!is_found) {
		ret = -FS_NOT_FOUND;
		goto OUT;
	}

OUT:

	if (last_alloc_dentry) {
		dput(last_alloc_dentry);
		last_alloc_dentry = NULL;
	}

	if (last_return_dentry) {
		dput(last_return_dentry);
		last_return_dentry = NULL;
	}

	if (ret)
		NT_frag_err_print("Failed to defrag file, ret %d\n", ret);

	return 0;
}

static int do_get_fraglevel_main(struct f2fs_path_fragment_info __user *f_pdi) {
	struct f2fs_path_fragment_info kern_f_pdi;
	struct dentry *target_dentry = NULL;
	struct dentry *nt_root_dentry = NULL;
	struct dentry *last_alloc_dentry = NULL;
	struct dentry *last_return_dentry = NULL;
	int is_found = 1;
	struct vfsmount *nt_vfsmnt = NULL;
	int ret = 0;

	if (copy_from_user(&kern_f_pdi, f_pdi, sizeof(kern_f_pdi))) {
		ret = -FS_ERR_CFU;
		goto OUT;
	}

	ret = find_root_dentry(&nt_vfsmnt, &nt_root_dentry);
	if(ret)
		goto OUT;

	is_found = find_target_dentry(nt_root_dentry, kern_f_pdi.path_str, &target_dentry, &last_alloc_dentry, &last_return_dentry);

	if (is_found && target_dentry) {
		struct inode *target_inode = target_dentry->d_inode;
		struct fiemap_extent_info fieinfo = { 0, };

		kern_f_pdi.file_size = inode_get_filesize(target_dentry);

		if (!target_inode->i_op->fiemap) {
			ret = -FS_NO_FIEMAP;
			goto OUT;
		}

		if (kern_f_pdi.fiemap.fm_extent_count > NT_FIEMAP_MAX_EXTENTS) {
			ret = -FS_OVER_EXT;
			goto OUT;
		}

		fieinfo.fi_flags = kern_f_pdi.fiemap.fm_flags;
		fieinfo.fi_extents_max = kern_f_pdi.fiemap.fm_extent_count;
		fieinfo.fi_extents_start = f_pdi->fiemap.fm_extents;
		ret = target_inode->i_op->fiemap(target_dentry->d_inode, &fieinfo, kern_f_pdi.fiemap.fm_start, kern_f_pdi.fiemap.fm_length);
		if (ret) {
			ret = -FS_ERR_FIEMAP;
			NT_frag_err_print("Fiemap result %d\n", ret);
			goto OUT;
		}
		kern_f_pdi.fiemap.fm_flags = fieinfo.fi_flags;
		kern_f_pdi.fiemap.fm_mapped_extents = fieinfo.fi_extents_mapped;
		kern_f_pdi.path_not_exist = 0;
		NT_frag_err_print("Get file frag level!");
	} else {
		ret = -FS_NOT_FOUND;
		kern_f_pdi.path_not_exist = 1;
	}

	if (copy_to_user(f_pdi, &kern_f_pdi, sizeof(kern_f_pdi)))
		return -FS_ERR_CTU;

	if (!is_found)
		goto OUT;

OUT:
	if (last_alloc_dentry) {
		dput(last_alloc_dentry);
		last_alloc_dentry = NULL;
	}

	if (last_return_dentry) {
		dput(last_return_dentry);
		last_return_dentry = NULL;
	}

	if (ret)
		NT_frag_err_print("Failed to get file frag level, ret %d\n", ret);

	return 1;

}


static long defrag_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	void __user *argp = (void __user *)arg;

	switch (cmd) {
		case IOCTL_GET_FRAGS:
			do_get_fraglevel_main(argp);
			break;
		case IOCTL_DO_DEFRAG:
			do_file_defrag_main(argp);
			break;
		default:
			return -ENOTTY;
	}

	return 0;
}

static const struct proc_ops get_readahead_files_fops = {
	.proc_open       = get_readahead_files_open,
	.proc_read       = seq_read,
	.proc_lseek      = seq_lseek,
	.proc_release    = single_release,
};

static struct file_operations do_defrag_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = defrag_ioctl,
};


static ssize_t readahead_monitor_switch_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return simple_read_from_buffer(buffer, length, offset, readahead_monitor_switch_status, strlen(readahead_monitor_switch_status));
}

static ssize_t readahead_monitor_switch_write(struct file *file, const char __user *buffer, size_t length, loff_t *off) {
	char tmp_str[2];

	if (length > 2)
		return -EINVAL;

	if (copy_from_user(tmp_str, buffer, 2))
		return -EINVAL;

	tmp_str[1] = '\0';

	if (tmp_str[0] == '0' || tmp_str[0] == '1') {
		strncpy(readahead_monitor_switch_status, tmp_str, 2);
		return length;
	} else
		return -EINVAL;

}

static const struct proc_ops readahead_monitor_switch_file_ops = {
	.proc_read = readahead_monitor_switch_read,
	.proc_write = readahead_monitor_switch_write,
};

static ssize_t file_insert_count_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	char buf[1024] = {0};

	sprintf(buf, "%llu", insert_count);
	return simple_read_from_buffer(buffer, length, offset, buf, strlen(buf));
}

static const struct proc_ops file_insert_count_ops = {
	.proc_read = file_insert_count_read,
};


static int __init readahead_tracing_init(void)
{
	struct proc_dir_entry *root = NULL;
	struct proc_dir_entry *readahead_monitor_switch = NULL;
	struct proc_dir_entry *get_readahead = NULL;
	struct proc_dir_entry *file_insert_count = NULL;
	int ret = 0;

	root = proc_mkdir("nt_readahead", NULL);
	if (!root){
		NT_frag_err_print("mkdir nt_readahead failed!\n");
		return 1;
	}

	get_readahead = proc_create("get_readahead_files", S_IRUGO, root, &get_readahead_files_fops);
	if (!get_readahead) {
		NT_frag_err_print("create node get_readahead_files node failed!\n");
		return 1;
	}

	readahead_monitor_switch = proc_create("readahead_monitor_switch", S_IRUGO, root, &readahead_monitor_switch_file_ops);
	if (!readahead_monitor_switch) {
		NT_frag_err_print("create node readahead_monitor_switch node failed!\n");
		return 1;
	}

	file_insert_count = proc_create("file_insert_count", S_IRUGO, root, &file_insert_count_ops);
	if (!file_insert_count) {
		NT_frag_err_print("create node file_insert_count node failed!\n");
		return 1;
	}

	if (alloc_chrdev_region(&df_dev, 0, 1, DEVICE_NAME) < 0) {
		NT_frag_err_print("Failed to allocate char device region.\n");
		return 1;
	}

	cdev_init(&c_df_dev, &do_defrag_fops);
	c_df_dev.owner = THIS_MODULE;
	if (cdev_add(&c_df_dev, df_dev, 1) < 0) {
		NT_frag_err_print("Failed to add char device.\n");
		unregister_chrdev_region(df_dev, 1);
		return 1;
	}

	df_class = class_create(THIS_MODULE, DEVICE_NAME);
	device_create(df_class, NULL, df_dev, NULL, DEVICE_NAME);

	ret = register_readahead_hooks();
	if (ret != 0)
		return ret;

	NT_frag_info_print("readahead_tracing_init!\n");
	return 0;
}

static void __exit readahead_tracing_exit(void)
{
	unregister_readahead_hooks();

	NT_frag_info_print("readahead_tracing_exit!\n");

	return;
}

module_init(readahead_tracing_init);
module_exit(readahead_tracing_exit);

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING memory information");
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);