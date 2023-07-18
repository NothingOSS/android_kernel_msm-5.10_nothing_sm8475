/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _QCOM_PRIME_HELP_H
#define _QCOM_PRIME_HELP_H

#include <linux/kernel.h>
#if IS_ENABLED(CONFIG_QCOM_PRIME_CORE_ONLINE_HELPER)
void prime_helper_trigger(void);
#else
void prime_helper_trigger(void)
{
	return;
}
#endif
#endif /* _QCOM_PRIME_HELP_H */
