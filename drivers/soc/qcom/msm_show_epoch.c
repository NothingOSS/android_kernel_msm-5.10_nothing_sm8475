// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <trace/hooks/epoch.h>
#if IS_ENABLED(CONFIG_CPU_IDLE_GOV_QCOM_LPM)
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <trace/events/power.h>
static int utc_s2ilde_mask;
#endif /* CONFIG_CPU_IDLE_GOV_QCOM_LPM */

static u64 suspend_ns;
static u64 suspend_cycles;
static u64 resume_cycles;

static void msm_show_suspend_epoch_val(void *data, u64 ns, u64 cycles)
{
	suspend_ns = ns;
	suspend_cycles = cycles;
	pr_info("suspend ns:%17llu      suspend cycles:%17llu\n",
						suspend_ns, suspend_cycles);
}

static void msm_show_resume_epoch_val(void *data, u64 cycles)
{
	resume_cycles = cycles;
	pr_info("resume cycles:%17llu\n", resume_cycles);
}

#if IS_ENABLED(CONFIG_CPU_IDLE_GOV_QCOM_LPM)
static void pm_suspend_marker(char *annotation)
{
    struct tm tm;
    time64_to_tm(ktime_get_real_seconds(), 0, &tm);
    pr_info("PM: suspend %s UTC %d-%02d-%02d %02d:%02d:%02d\n",
        annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static void utc_debug_suspend_trace_probe(void *unused,
					const char *action, int val, bool start)
{
    /* 1. UTC Time should get during timekeeping awake
       2. Func should in GKI white list */
	if (start && val > 0 && !strcmp("suspend_enter", action))
		pm_suspend_marker("entry");

    if (!start && val > 0 && !strcmp("resume_console", action))
		pm_suspend_marker("exit");
}

static int utc_s2idle_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", utc_s2ilde_mask);
	return 0;
}

static int utc_s2idle_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, utc_s2idle_proc_show, NULL);
}

static ssize_t utc_s2idle_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	char c;
    int ret = 0;

	if (get_user(c, buffer))
		return -EFAULT;
	if (c < '0' || c > '1')
		return -EINVAL;
	utc_s2ilde_mask = c - '0';

	if (utc_s2ilde_mask)
		ret = register_trace_suspend_resume(
			utc_debug_suspend_trace_probe, NULL);
	else
		ret = unregister_trace_suspend_resume(
			utc_debug_suspend_trace_probe, NULL);
	if (ret)
		pr_err("%s: Fail to %s register utc timestamp suspend trace, ret=%d\n",
			__func__, utc_s2ilde_mask ? "" : "un", ret);
	return count;
}

static const struct proc_ops utc_s2idle_proc_ops = {
	.proc_open	= utc_s2idle_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= utc_s2idle_proc_write,
};
#endif /* CONFIG_CPU_IDLE_GOV_QCOM_LPM */


static int __init msm_show_epoch_init(void)
{
	register_trace_android_vh_show_suspend_epoch_val(
					msm_show_suspend_epoch_val, NULL);
	register_trace_android_vh_show_resume_epoch_val(
					msm_show_resume_epoch_val, NULL);

#if IS_ENABLED(CONFIG_CPU_IDLE_GOV_QCOM_LPM)
	proc_create("utc_s2idle", 0, NULL, &utc_s2idle_proc_ops);
#endif /* CONFIG_CPU_IDLE_GOV_QCOM_LPM */

	return 0;
}

#if IS_MODULE(CONFIG_SHOW_SUSPEND_EPOCH)
module_init(msm_show_epoch_init);
#else
pure_initcall(msm_show_epoch_init);
#endif

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. show epoch values driver");
MODULE_LICENSE("GPL v2");
