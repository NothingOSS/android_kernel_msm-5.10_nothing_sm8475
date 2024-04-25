// SPDX-License-Identifier: GPL-2.0
/* Lock down the kernel
 *
 * Copyright (C) 2016 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/security.h>
#include <linux/export.h>
#include <linux/lsm_hooks.h>
#include <linux/early_lockdown.h>

static enum lockdown_reason kernel_locked_down;
static struct mutex lockdown_mutex;

/* Lockdown reason descriptions */
static const char *const lockdown_reasons[LOCKDOWN_CONFIDENTIALITY_MAX + 1] = {
    /* Reason descriptions */
};

static const enum lockdown_reason lockdown_levels[] = {
    LOCKDOWN_NONE,
    LOCKDOWN_INTEGRITY_MAX,
    LOCKDOWN_CONFIDENTIALITY_MAX};

/* Function to lock the kernel down */
static int lock_kernel_down(const char *where, enum lockdown_reason level)
{
    int ret = -EPERM;

    mutex_lock(&lockdown_mutex);

    if (kernel_locked_down < level) {
        kernel_locked_down = level;
        pr_notice("Kernel is locked down from %s; see man kernel_lockdown.7\n", where);
        ret = 0;
    }

    mutex_unlock(&lockdown_mutex);

    return ret;
}

/* Function to handle lockdown parameter initialization */
static int __init lockdown_param(char *level)
{
    if (!level)
        return -EINVAL;

    if (strcmp(level, "integrity") == 0)
        return lock_kernel_down("command line", LOCKDOWN_INTEGRITY_MAX);
    else if (strcmp(level, "confidentiality") == 0)
        return lock_kernel_down("command line", LOCKDOWN_CONFIDENTIALITY_MAX);
    else
        return -EINVAL;
}

early_param("lockdown", lockdown_param);

/* Function to check if the kernel is locked down */
static int lockdown_is_locked_down(enum lockdown_reason what)
{
    int ret = 0;

    mutex_lock(&lockdown_mutex);

    if (what >= LOCKDOWN_CONFIDENTIALITY_MAX || kernel_locked_down >= what)
    {
        if (lockdown_reasons[what])
            pr_notice("Lockdown: %s: %s is restricted; see man kernel_lockdown.7\n",
                      current->comm, lockdown_reasons[what]);
        ret = -EPERM;
    }

    mutex_unlock(&lockdown_mutex);

    return ret;
}

/* File operations for the lockdown securityfs file */
static const struct file_operations lockdown_ops = {
    .read = lockdown_read,
    .write = lockdown_write,
};

/* Initialize lockdown LSM */
static int __init lockdown_lsm_init(void)
{
    mutex_init(&lockdown_mutex);

#if defined(CONFIG_LOCK_DOWN_KERNEL_FORCE_INTEGRITY)
    lock_kernel_down("Kernel configuration", LOCKDOWN_INTEGRITY_MAX);
#elif defined(CONFIG_LOCK_DOWN_KERNEL_FORCE_CONFIDENTIALITY)
    lock_kernel_down("Kernel configuration", LOCKDOWN_CONFIDENTIALITY_MAX);
#endif

    security_add_hooks(lockdown_hooks, ARRAY_SIZE(lockdown_hooks), "lockdown");
    return 0;
}

static int __init lockdown_secfs_init(void)
{
    struct dentry *dentry;

    dentry = securityfs_create_file("lockdown", 0644, NULL, NULL, &lockdown_ops);
    return PTR_ERR_OR_ZERO(dentry);
}

core_initcall(lockdown_secfs_init);

#ifdef CONFIG_SECURITY_LOCKDOWN_LSM_EARLY
DEFINE_EARLY_LSM(lockdown) = {
#else
DEFINE_LSM(lockdown) = {
#endif
    .name = "lockdown",
    .init = lockdown_lsm_init,
};
