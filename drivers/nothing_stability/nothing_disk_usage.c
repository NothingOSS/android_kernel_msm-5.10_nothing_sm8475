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
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/hashtable.h>
#include <linux/jhash.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <../../fs/f2fs/f2fs.h>

#define MAX_DIR_STACK_SIZE 524288

#define NT_HASH_SIZE 9

#define NT_disk_usage_print(fmt,arg...) \
pr_info("[Get_disk_usage] "fmt, ##arg)

#define NT_disk_usage_err_print(fmt,arg...) \
pr_err("[Get_disk_usage] "fmt, ##arg)


static DEFINE_HASHTABLE(path_hash_table, NT_HASH_SIZE);

struct path_table_struct {
	unsigned long long value;
	char key[1024];
	struct hlist_node nodex;
};

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
static int printdir_sub(struct dir_context *ctx, const char *name, int namlen,
       loff_t offset, u64 ino, unsigned int d_type);

struct nt_dir_info {
	struct dir_context ctx;
	struct dentry *dir_dentry;
	struct dentry *alloc_dentry;
	struct dentry *return_dentry;
};

static struct nt_dir_info **n_stack = NULL;
static int stack_size = 0;
static char *path_buf = NULL;

static int is_inode_directory(struct inode *inode) {
    return S_ISDIR(inode->i_mode);
}

static long long inode_get_filesize(struct dentry *n_dentry){
	return (((loff_t)n_dentry->d_inode->i_blocks) << 9) + n_dentry->d_inode->i_bytes;
}

void insert_key_value(const char *key, unsigned long long value, unsigned int path_str_len) {
	struct path_table_struct *entry;
	unsigned int hash;

	entry = vzalloc(sizeof(struct path_table_struct));
	if (!entry) {
		NT_disk_usage_err_print("Failed to allocate memory for key-value pair.\n");
		return;
	}

	strncpy(entry->key, key, path_str_len);
	entry->value = value;

	hash = jhash(key, strlen(key), 0);

	hash_add(path_hash_table, &entry->nodex, hash);
}

int increment_value(const char *key, unsigned long long value) {
	struct path_table_struct *entry;
	unsigned int hash = jhash(key, strlen(key), 0);

	hash_for_each_possible(path_hash_table, entry, nodex, hash) {
		if (strcmp(entry->key, key) == 0) {
			entry->value+=value;
			return 1;
		}
	}

	return 0;
}

static void loop_split_tail(char *path, unsigned long long path_file_size) {
	int length = strlen(path);
	int i;
	for (i = length - 1; i >= 0; i--) {
		unsigned int path_str_len = 0;
		if (path[i] == '/') {
			if(i == 0) {
				path[1] = '\0';
			} else {
				path[i] = '\0';
			}
			path_str_len = strlen(path);
			if (path_str_len > 1024 || path_str_len <=0)
				continue;
			if (increment_value(path, path_file_size)) {
			} else {
				insert_key_value(path, path_file_size, path_str_len);
			}
		}
	}
}
static int printfullpath(struct dentry *n_dentry, unsigned long long dir_size) {
	char  *path = NULL;

	memset(path_buf, 0, PAGE_SIZE);
	path = dentry_path_raw(n_dentry, path_buf, PAGE_SIZE);
	if (!IS_ERR(path)) {
		loop_split_tail(path, inode_get_filesize(n_dentry));
		return 1;
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

	if (is_dot_dotdot(real_name, namlen)) {
		return 0;
	}

	alloc_dentry = d_alloc_name(subdir_info->dir_dentry, real_name);
	return_dentry = subdir_info->dir_dentry->d_inode->i_op->lookup(subdir_info->dir_dentry->d_inode, alloc_dentry, 0);

	ret = process_fs_dentry(alloc_dentry);

	if (ret != DISK_GOOD)
		goto TRY_RETURN;

	is_dir = join_dir_or_print_file(subdir_info, alloc_dentry);
	if (is_dir) {
		a = vzalloc(sizeof(struct nt_dir_info));
		a->ctx.actor  = &printdir_sub;
		a->dir_dentry = alloc_dentry;
		a->alloc_dentry = alloc_dentry;
		a->return_dentry = return_dentry;
		n_stack[stack_size++] = a;
	}
	goto OUT;

TRY_RETURN:
	ret = process_fs_dentry(return_dentry);
	if (ret != DISK_GOOD) {
		goto ERR_OUT;
	}

	is_dir = join_dir_or_print_file(subdir_info, return_dentry);
	if (is_dir) {
		a = vzalloc(sizeof(struct nt_dir_info));
		a->ctx.actor  = &printdir_sub;
		a->dir_dentry = return_dentry;
		a->alloc_dentry = alloc_dentry;
		a->return_dentry = return_dentry;
		n_stack[stack_size++] = a;
	}
	goto OUT;

ERR_OUT:
	NT_disk_usage_err_print("=== it's a lost file:  %s %d %ld alloc_dentry: %p return_dentry: %p ,ret %d %d %d ===\n", real_name, namlen, ino, alloc_dentry, return_dentry, ret, is_dir, stack_size);
OUT:
	if (!is_dir) {
		dput(alloc_dentry);
		dput(return_dentry);
	}
	return 0;
}

struct super_sb_data {
	struct super_block *sb;
};

static void sb_iterator(struct super_block *sb, void *arg)
{
	struct super_sb_data *sbd = arg;

	if (sb->s_flags & SB_INLINECRYPT) {
		sbd->sb = sb;
		NT_disk_usage_err_print("root dir is %s\n", sb->s_root->d_iname);
	}
}

static int get_data_usage_main(struct seq_file *s, void *unused) {
	struct file_system_type *type;
	struct file target_partition_file;
	struct dentry * nt_root_dentry = NULL;
	struct super_block *nt_f2fs_sb = NULL;
	struct nt_dir_info p_d;
	int ret = 0;
	struct path_table_struct *entry;
	struct hlist_node *tmp;
	int i;
	struct super_sb_data iter_sb = {
		.sb = NULL,
	};

	n_stack= vzalloc(MAX_DIR_STACK_SIZE * sizeof(struct nt_dir_info *));
	if (!n_stack) {
		NT_disk_usage_err_print("Allocate full stack failed\n");
		return 1;
	}
	path_buf = (char *)vzalloc(PAGE_SIZE);
	if (!path_buf) {
		NT_disk_usage_err_print("Allocate full path memory failed\n");
		return 1;
	}

	type = get_fs_type("f2fs");
	if (!type) {
		ret = DISK_ERR_FS_TYPE;
		goto ERR_OUT;
	}
	NT_disk_usage_err_print("file_system_type is %s\n", type->name);

	iterate_supers_type(type, sb_iterator, &iter_sb);
	nt_f2fs_sb = iter_sb.sb;
	if (!nt_f2fs_sb) {
		ret = DISK_NULL_RT;
		goto ERR_OUT;
	}
	nt_root_dentry = nt_f2fs_sb->s_root;
	if (!nt_root_dentry) {
		ret = DISK_NULL_RT;
		goto ERR_OUT;
	}
	p_d.ctx.actor = &printdir_sub;
	p_d.dir_dentry = nt_root_dentry;

	target_partition_file.f_mapping = nt_root_dentry->d_inode->i_mapping;
	target_partition_file.f_flags = O_NOATIME;
	target_partition_file.f_inode = nt_root_dentry->d_inode;
	target_partition_file.f_lock = nt_root_dentry->d_inode->i_lock;

	if (!nt_root_dentry->d_inode) {
		ret = DISK_NULL_RT_INODE;
		goto ERR_OUT;
	}

	nt_root_dentry->d_inode->i_fop->iterate_shared(&target_partition_file, &p_d.ctx);

	while (stack_size > 0) {
		struct nt_dir_info *current_path = n_stack[--stack_size];
		struct file sub_file;
		struct inode *n_inode = current_path->dir_dentry->d_inode;

		sub_file.f_mapping = n_inode->i_mapping;
		sub_file.f_flags = O_NOATIME;
		sub_file.f_inode = n_inode;
		sub_file.f_lock = n_inode->i_lock;

		if (n_inode->i_fop->iterate_shared)
			n_inode->i_fop->iterate_shared(&sub_file, &current_path->ctx);
		dput(current_path->alloc_dentry);
		dput(current_path->return_dentry);
		vfree(current_path);
	}

	hash_for_each_safe(path_hash_table, i, tmp, entry, nodex) {
		if (entry->value > SZ_1G)
			seq_printf(s, "%s %llu\n", entry->key, entry->value / SZ_1M);
		hash_del(&entry->nodex);
		vfree(entry);
	}

goto OUT;


ERR_OUT:
	NT_disk_usage_err_print("=== error, ret %d ===\n", ret);
OUT:
	if (n_stack) {
		vfree(n_stack);
		n_stack = NULL;
	}

	if (path_buf) {
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
	if (!root){
		NT_disk_usage_err_print("mkdir nt_disk failed!\n");
		return -1;
	}

	pentry = proc_create("get_data_usage", S_IRUGO, root, &get_data_usage_fops);
	if (!pentry) {
		NT_disk_usage_err_print("create node get_data_usage node failed!\n");
		return -1;
	}

	return 0;
}

device_initcall(nt_disk_usage_init);

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING disk information");
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);