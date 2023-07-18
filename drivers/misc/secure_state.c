#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int is_secure_fuse = 0;

static int secure_state_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n",is_secure_fuse);
	return 0;
}

static int secure_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, secure_state_show, inode->i_private);
}

static const struct proc_ops secure_state_fops = {
	.proc_open = secure_state_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int __init init_secure_state(void)
{
	struct proc_dir_entry *dir;
	dir = proc_create("secure_state", 0444, NULL, &secure_state_fops);
	if (!dir) {
		pr_alert("Now in %s. proc_create.dir = %p\n", __func__, dir);
		return -1;
	}
	return 0;
}

static void __exit exit_secure_state(void)
{
	remove_proc_entry("secure_state", NULL);
}

module_init(init_secure_state);
module_exit(exit_secure_state);

MODULE_DESCRIPTION("NT Secureboot State Driver");
MODULE_LICENSE("GPL v2");

module_param(is_secure_fuse, int, 0600);
MODULE_PARM_DESC(is_secure_fuse,
		"secure_state.is_secure_fuse=0/1");
