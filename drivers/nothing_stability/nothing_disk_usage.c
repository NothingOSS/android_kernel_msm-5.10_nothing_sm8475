#include <linux/buffer_head.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/f2fs_fs.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fs_context.h>
#include <linux/fs_struct.h>
#include <linux/fdtable.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/list.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>
#include <linux/printk.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#define MAX_DIR_STACK_SIZE 8192

enum {
	DISK_GOOD = 0,
	DISK_ERR_FS_TYPE,
	DISK_NULL_RT,
	DISK_ERR_RT,
	DISK_NULL_RT_NAME,
	DISK_NULL_RT_INODE,
	DISK_NULL_CB,
	DISK_ERR_CB,
	DISK_NULL_CB_NAME,
	DISK_NULL_CB_INODE,
	DISK_NULL_RET,
	DISK_ERR_RET,
	DISK_NULL_RET_NAME,
	DISK_NULL_RET_INODE,
};

static int printdir_root(struct dir_context *ctx, const char *name, int namlen,
       loff_t offset, u64 ino, unsigned int d_type);
static int printdir_sub(struct dir_context *ctx, const char *name, int namlen,
       loff_t offset, u64 ino, unsigned int d_type);

struct nt_dir_info {
	struct dir_context ctx;
	struct dentry *dir_dentry;
};

struct seq_file *console_out = NULL;
struct dentry * nt_root_dentry = NULL;
struct super_block *nt_f2fs_sb = NULL;

struct nt_dir_info **n_stack = NULL;
static int stack_size = 0;
char *path_buf = NULL;

static int is_inode_directory(struct inode *inode) {
    return S_ISDIR(inode->i_mode);
}

static long long inode_get_filesize(struct dentry *n_dentry){
	return (((loff_t)n_dentry->d_inode->i_blocks) << 9) + n_dentry->d_inode->i_bytes;
}

static int printfullpath(struct dentry *n_dentry, unsigned long long dir_size) {
	char  *path = NULL;

	memset(path_buf, 0, PAGE_SIZE);
	path = dentry_path_raw(n_dentry, path_buf, PAGE_SIZE);
	if (!IS_ERR(path)) {
		if(dir_size > 0)
			seq_printf(console_out, "type: [file] name: %s size: %lld\n", path, dir_size);
		else
			seq_printf(console_out, "type: [file] name: %s size: %lld\n", path, inode_get_filesize(n_dentry));
	}
	return 0;
}

static int process_fs_dentry(struct dentry *n_dentry) {

	if (!n_dentry)
		return DISK_NULL_CB;

	if (IS_ERR(n_dentry))
		return DISK_ERR_CB;

	if (!n_dentry->d_name.name)
		return DISK_NULL_CB_NAME;

	if (!n_dentry->d_inode)
		return DISK_NULL_CB_INODE;

	return DISK_GOOD;
}

static int join_dir_or_print_file(struct nt_dir_info *subdir_info, struct dentry *n_dentry) {
	if (is_inode_directory(n_dentry->d_inode)) {
		return 1;
	} else {
		printfullpath(n_dentry, 0);
		return 0;
	}
	return 0;
}

static int printdir_sub(struct dir_context *ctx, const char *name, int namlen,
       loff_t offset, u64 ino, unsigned int d_type) {
	struct nt_dir_info *subdir_info = container_of(ctx, struct nt_dir_info, ctx);
	struct dentry *alloc_dentry = NULL;
	struct dentry *return_dentry = NULL;
	struct nt_dir_info *a = NULL;
	char real_name[F2FS_NAME_LEN] = {0};
	int ret = 0;
	int is_dir = 0;

	strncpy(real_name, name, namlen);
	real_name[namlen] = '\0';

	if (strcmp(real_name, ".") == 0 || strcmp(real_name, "..") == 0)
		return 0;


	alloc_dentry = d_alloc_name(subdir_info->dir_dentry, real_name);
	return_dentry = subdir_info->dir_dentry->d_inode->i_op->lookup(subdir_info->dir_dentry->d_inode, alloc_dentry, 0);

	ret = process_fs_dentry(alloc_dentry);

	if (ret != DISK_GOOD)
		goto TRY_RETURN;

	is_dir = join_dir_or_print_file(subdir_info, alloc_dentry);
	if(is_dir) {
		a = vzalloc(sizeof(struct nt_dir_info));
		a->ctx.actor  = &printdir_sub;
		a->dir_dentry = alloc_dentry;
		n_stack[stack_size++] = a;
	}
	goto OUT;

TRY_RETURN:
	ret = process_fs_dentry(return_dentry);
	if (ret != DISK_GOOD) {
		goto ERR_OUT;
	}

	is_dir = join_dir_or_print_file(subdir_info, return_dentry);
	if(is_dir) {
		a = vzalloc(sizeof(struct nt_dir_info));
		a->ctx.actor  = &printdir_sub;
		a->dir_dentry = return_dentry;
		n_stack[stack_size++] = a;
	}
	goto OUT;

ERR_OUT:
	seq_printf(console_out, "=== it's a lost file:  %s %d %ld alloc_dentry: %p return_dentry: %p ,ret %d ===\n", real_name, namlen, ino, alloc_dentry, return_dentry, ret);
OUT:
	dput(alloc_dentry);
	return 0;
}

static int printdir_root(struct dir_context *ctx, const char *name, int namlen,
       loff_t offset, u64 ino, unsigned int d_type) {
	struct nt_dir_info *subdir_info = container_of(ctx, struct nt_dir_info, ctx);
	struct dentry *alloc_dentry = NULL;
	struct dentry *return_dentry = NULL;
	char real_name[F2FS_NAME_LEN] = {0};
	struct nt_dir_info *a = NULL;
	int ret = 0;
	int is_dir = 0;

	strncpy(real_name, name, namlen);
	real_name[namlen] = '\0';

	if (strcmp(real_name, ".") == 0 || strcmp(real_name, "..") == 0)
		return 0;

	alloc_dentry = d_alloc_name(subdir_info->dir_dentry, real_name);
	return_dentry = subdir_info->dir_dentry->d_inode->i_op->lookup(subdir_info->dir_dentry->d_inode, alloc_dentry, 0);

	ret = process_fs_dentry(alloc_dentry);

	if (ret != DISK_GOOD)
		goto TRY_RETURN;

	is_dir = join_dir_or_print_file(subdir_info, alloc_dentry);
	if(is_dir) {
		a = vzalloc(sizeof(struct nt_dir_info));
		a->ctx.actor  = &printdir_sub;
		a->dir_dentry = alloc_dentry;
		n_stack[stack_size++] = a;
		goto START_WHILE;
	}
	goto OUT;

TRY_RETURN:
	ret = process_fs_dentry(return_dentry);
	if (ret != DISK_GOOD) {
		goto ERR_OUT;
	}

	is_dir = join_dir_or_print_file(subdir_info, return_dentry);
	if(is_dir) {
		a = vzalloc(sizeof(struct nt_dir_info));
		a->ctx.actor  = &printdir_sub;
		a->dir_dentry = return_dentry;
		n_stack[stack_size++] = a;
		goto START_WHILE;
	}
	goto OUT;

START_WHILE:
	while (stack_size > 0) {
		struct nt_dir_info *current_path = n_stack[--stack_size];
		struct file sub_file;
		struct inode *n_inode = current_path->dir_dentry->d_inode;

		sub_file.f_mapping = n_inode->i_mapping;
		sub_file.f_flags = O_NOATIME;
		sub_file.f_inode = n_inode;
		sub_file.f_lock = n_inode->i_lock;

		if(n_inode->i_fop->iterate_shared)
			n_inode->i_fop->iterate_shared(&sub_file, &current_path->ctx);
		vfree(current_path);
	}
	goto OUT;

ERR_OUT:
	seq_printf(console_out, "=== it's a lost file:  %s %d %ld alloc_dentry: %p return_dentry: %p ,ret %d %d %d ===\n", real_name, namlen, ino, alloc_dentry, return_dentry, ret, is_dir, stack_size);
OUT:
	dput(alloc_dentry);
	return 0;
}


static void sb_iterator(struct super_block *sb, void *arg)
{
	if(!nt_root_dentry) {
		nt_root_dentry = sb->s_root;
		nt_f2fs_sb = sb;
		seq_printf(console_out, "root dir is %s\n", sb->s_root->d_iname);
	}
}

static int get_data_usage_main(struct seq_file *s, void *unused) {
	struct file_system_type *type;
	struct file target_partition_file;
	struct nt_dir_info p_d;
	int ret = 0;
	LIST_HEAD(dir_list);

	console_out = s;
	n_stack= vzalloc(MAX_DIR_STACK_SIZE * sizeof(struct nt_dir_info *)) ;
	if (!n_stack) {
		seq_printf(console_out, "Allocate full stack failed\n");
		return 1;
	}
	path_buf = (char *)vzalloc(PAGE_SIZE);
	if (!path_buf) {
		seq_printf(console_out, "Allocate full path memory failed\n");
		return 1;
	}

	type = get_fs_type("f2fs");
	if (!type) {
		ret = DISK_ERR_FS_TYPE;
		goto ERR_OUT;
	}
	seq_printf(console_out, "file_system_type is %s\n", type->name);

	iterate_supers_type(type, sb_iterator, NULL);
	if(!nt_root_dentry || !nt_f2fs_sb) {
		ret = DISK_NULL_RT;
		goto ERR_OUT;
	}
	p_d.ctx.actor = &printdir_root;
	p_d.dir_dentry = nt_root_dentry;

	target_partition_file.f_mapping = nt_root_dentry->d_inode->i_mapping;
	target_partition_file.f_flags = O_NOATIME;
	target_partition_file.f_inode = nt_root_dentry->d_inode;
	target_partition_file.f_lock = nt_root_dentry->d_inode->i_lock;

	if(!nt_root_dentry->d_inode) {
		ret = DISK_NULL_RT_INODE;
		goto ERR_OUT;
	}

	nt_root_dentry->d_inode->i_fop->iterate_shared(&target_partition_file, &p_d.ctx);
	goto OUT;


ERR_OUT:
	seq_printf(console_out, "=== error, ret %d ===\n", ret);
OUT:
	if(n_stack) {
		vfree(n_stack);
		n_stack = NULL;
	}

	if(path_buf) {
		vfree(path_buf);
		path_buf = NULL;
	}

	return 0;
}

static int get_data_usage_open(struct inode *inode, struct file *file)
{
	return single_open(file, get_data_usage_main, NULL);
}

static const struct proc_ops get_data_usage_fops = {
	.proc_open       = get_data_usage_open,
	.proc_read       = seq_read,
	.proc_lseek      = seq_lseek,
	.proc_release    = single_release,
};

static int __init nt_disk_usage_init(void)
{
	struct proc_dir_entry *root, *pentry;

	root = proc_mkdir("nt_disk", NULL);
	if(!root){
		pr_err("mkdir nt_disk failed!\n");
		return -1;
	}

	pentry = proc_create("get_data_usage", S_IRUGO, root, &get_data_usage_fops);
	if(!pentry) {
		pr_err("create node get_data_usage node failed!\n");
		return -1;
	}

	return 0;
}

device_initcall(nt_disk_usage_init);

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING disk information");