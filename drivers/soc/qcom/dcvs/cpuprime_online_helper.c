// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "qcom-prime-helper: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <soc/qcom/prime_helper.h>

static bool trigger_done = false;

/* Refer to dd, defer triggered after device bound driver
   So Do a defer trigger after online once */

static int qcom_prime_helper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	dev_info(dev, "Prime Online Helper probe\n");
	return 0;
}

static struct platform_device qcom_prime_helper_device = {
	.name = "qcom-prime-helper",
	.id = -1,
};

static struct platform_driver qcom_prime_helper_driver = {
	.probe = qcom_prime_helper_probe,
	.driver = {
		.name = "qcom-prime-helper",
	},
};

void prime_helper_trigger(void)
{
    int ret = 0;
    if (trigger_done)
        return;

    ret = platform_device_register(&qcom_prime_helper_device);
    if (ret)
        pr_err("Prime Online Helper Device Register Fail\n");
    else
        pr_info("Prime Online Helper Device Register Succeed\n");

    trigger_done = true;
}
EXPORT_SYMBOL(prime_helper_trigger);

static int __init qcom_prime_helper_init(void)
{
	return platform_driver_register(&qcom_prime_helper_driver);
}
module_init(qcom_prime_helper_init);

static __exit void qcom_prime_helper_exit(void)
{
	platform_driver_unregister(&qcom_prime_helper_driver);
}
module_exit(qcom_prime_helper_exit);

MODULE_DESCRIPTION("QCOM Prime Core Helper Driver");
MODULE_LICENSE("GPL v2");
