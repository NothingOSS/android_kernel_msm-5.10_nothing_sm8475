/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#else
#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define FTS_DRIVER_PEN_NAME                 "fts_ts,pen"
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      3300000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_SPI_VTG_MIN_UV                  1800000
#define FTS_SPI_VTG_MAX_UV                  1800000
#endif
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#define DRM_CHK_MAX_COUNTS          30
#endif
#endif

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static int drm_check_count = 0;
#endif
#endif


/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
    int i = 0;
    struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
    u8 cid_h = 0x0;

    if (cid->type == 0)
        return -ENODATA;

    for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
        cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
        if (cid_h && (id_h == cid_h)) {
            return 0;
        }
    }

    return -ENODATA;
}

#ifdef CONFIG_FTS_TRUSTED_TOUCH

static void fts_ts_trusted_touch_abort_handler(struct fts_ts_data *fts_data,
						int error);
static struct gh_acl_desc *fts_ts_vm_get_acl(enum gh_vm_names vm_name)
{
	struct gh_acl_desc *acl_desc;
	gh_vmid_t vmid;

	gh_rm_get_vmid(vm_name, &vmid);

	acl_desc = kzalloc(offsetof(struct gh_acl_desc, acl_entries[1]),
			GFP_KERNEL);
	if (!acl_desc)
		return ERR_PTR(ENOMEM);

	acl_desc->n_acl_entries = 1;
	acl_desc->acl_entries[0].vmid = vmid;
	acl_desc->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;

	return acl_desc;
}

static struct gh_sgl_desc *fts_ts_vm_get_sgl(
				struct trusted_touch_vm_info *vm_info)
{
	struct gh_sgl_desc *sgl_desc;
	int i;

	sgl_desc = kzalloc(offsetof(struct gh_sgl_desc,
			sgl_entries[vm_info->iomem_list_size]), GFP_KERNEL);
	if (!sgl_desc)
		return ERR_PTR(ENOMEM);

	sgl_desc->n_sgl_entries = vm_info->iomem_list_size;

	for (i = 0; i < vm_info->iomem_list_size; i++) {
		sgl_desc->sgl_entries[i].ipa_base = vm_info->iomem_bases[i];
		sgl_desc->sgl_entries[i].size = vm_info->iomem_sizes[i];
	}

	return sgl_desc;
}

static int fts_ts_populate_vm_info_iomem(struct fts_ts_data *fts_data)
{
	int i, gpio, rc = 0;
	int num_regs, num_sizes, num_gpios, list_size;
	struct resource res;
	struct device_node *np = fts_data->dev->of_node;
	struct trusted_touch_vm_info *vm_info = fts_data->vm_info;

	num_regs = of_property_count_u32_elems(np, "focaltech,trusted-touch-io-bases");
	if (num_regs < 0) {
		FTS_ERROR("Invalid number of IO regions specified\n");
		return -EINVAL;
	}

	num_sizes = of_property_count_u32_elems(np, "focaltech,trusted-touch-io-sizes");
	if (num_sizes < 0) {
		FTS_ERROR("Invalid number of IO regions specified\n");
		return -EINVAL;
	}

	if (num_regs != num_sizes) {
		FTS_ERROR("IO bases and sizes array lengths mismatch\n");
		return -EINVAL;
	}

	num_gpios = of_gpio_named_count(np, "focaltech,trusted-touch-vm-gpio-list");
	if (num_gpios < 0) {
		dev_warn(fts_data->dev, "Ignoring invalid trusted gpio list: %d\n", num_gpios);
		num_gpios = 0;
	}

	list_size = num_regs + num_gpios;
	vm_info->iomem_list_size = list_size;
	vm_info->iomem_bases = devm_kcalloc(fts_data->dev, list_size, sizeof(*vm_info->iomem_bases),
			GFP_KERNEL);
	if (!vm_info->iomem_bases)
		return -ENOMEM;

	vm_info->iomem_sizes = devm_kcalloc(fts_data->dev, list_size, sizeof(*vm_info->iomem_sizes),
			GFP_KERNEL);
	if (!vm_info->iomem_sizes)
		return -ENOMEM;

	for (i = 0; i < num_gpios; ++i) {
		gpio = of_get_named_gpio(np, "focaltech,trusted-touch-vm-gpio-list", i);
		if (gpio < 0 || !gpio_is_valid(gpio)) {
			FTS_ERROR("Invalid gpio %d at position %d\n", gpio, i);
			return gpio;
		}

		if (!msm_gpio_get_pin_address(gpio, &res)) {
			FTS_ERROR("Failed to retrieve gpio-%d resource\n", gpio);
			return -ENODATA;
		}

		vm_info->iomem_bases[i] = res.start;
		vm_info->iomem_sizes[i] = resource_size(&res);
	}

	rc = of_property_read_u32_array(np, "focaltech,trusted-touch-io-bases",
			&vm_info->iomem_bases[i], list_size - i);
	if (rc) {
		FTS_ERROR("Failed to read trusted touch io bases:%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32_array(np, "focaltech,trusted-touch-io-sizes",
			&vm_info->iomem_sizes[i], list_size - i);
	if (rc) {
		FTS_ERROR("Failed to read trusted touch io sizes:%d\n", rc);
		return rc;
	}

	return 0;
}

static int fts_ts_populate_vm_info(struct fts_ts_data *fts_data)
{
	int rc;
	struct trusted_touch_vm_info *vm_info;
	struct device_node *np = fts_data->dev->of_node;

	vm_info = devm_kzalloc(fts_data->dev, sizeof(struct trusted_touch_vm_info), GFP_KERNEL);
	if (!vm_info)
		return -ENOMEM;

	fts_data->vm_info = vm_info;
	vm_info->vm_name = GH_TRUSTED_VM;
	rc = of_property_read_u32(np, "focaltech,trusted-touch-spi-irq", &vm_info->hw_irq);
	if (rc) {
		pr_err("Failed to read trusted touch SPI irq:%d\n", rc);
		return rc;
	}

	rc = fts_ts_populate_vm_info_iomem(fts_data);
	if (rc) {
		pr_err("Failed to read trusted touch mmio ranges:%d\n", rc);
		return rc;
	}

	rc = of_property_read_string(np, "focaltech,trusted-touch-type",
						&vm_info->trusted_touch_type);
	if (rc) {
		pr_warn("%s: No trusted touch type selection made\n", __func__);
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_PRIMARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_PRIMARY;
		rc = 0;
	} else if (!strcmp(vm_info->trusted_touch_type, "primary")) {
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_PRIMARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_PRIMARY;
	} else if (!strcmp(vm_info->trusted_touch_type, "secondary")) {
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_SECONDARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_SECONDARY;
	}

#ifdef CONFIG_ARCH_QTI_VM
	rc = of_property_read_u32(np, "focaltech,reset-gpio-base", &vm_info->reset_gpio_base);
	if (rc)
		pr_err("Failed to read reset gpio base:%d\n", rc);

	rc = of_property_read_u32(np, "focaltech,intr-gpio-base", &vm_info->intr_gpio_base);
	if (rc)
		pr_err("Failed to read intr gpio base:%d\n", rc);
#endif

	return 0;
}

static void fts_ts_destroy_vm_info(struct fts_ts_data *fts_data)
{
	kfree(fts_data->vm_info->iomem_sizes);
	kfree(fts_data->vm_info->iomem_bases);
	kfree(fts_data->vm_info);
}

static void fts_ts_vm_deinit(struct fts_ts_data *fts_data)
{
	if (fts_data->vm_info->mem_cookie)
		gh_mem_notifier_unregister(fts_data->vm_info->mem_cookie);
	fts_ts_destroy_vm_info(fts_data);
}

static int fts_ts_trusted_touch_get_vm_state(struct fts_ts_data *fts_data)
{
	return atomic_read(&fts_data->vm_info->vm_state);
}

static void fts_ts_trusted_touch_set_vm_state(struct fts_ts_data *fts_data,
							int state)
{
	atomic_set(&fts_data->vm_info->vm_state, state);
}

#ifdef CONFIG_ARCH_QTI_VM
static int fts_ts_vm_mem_release(struct fts_ts_data *fts_data);
static void fts_ts_trusted_touch_tvm_vm_mode_disable(struct fts_ts_data *fts_data);
static void fts_ts_trusted_touch_abort_tvm(struct fts_ts_data *fts_data);
static void fts_ts_trusted_touch_event_notify(struct fts_ts_data *fts_data, int event);

void fts_ts_trusted_touch_tvm_i2c_failure_report(struct fts_ts_data *fts_data)
{
	pr_err("initiating trusted touch abort due to i2c failure\n");
	fts_ts_trusted_touch_abort_handler(fts_data,
			TRUSTED_TOUCH_EVENT_I2C_FAILURE);
}

static void fts_ts_trusted_touch_reset_gpio_toggle(struct fts_ts_data *fts_data)
{
	void __iomem *base;

	if (fts_data->bus_type != BUS_TYPE_I2C)
		return;

	if (!fts_data->vm_info->reset_gpio_base) {
		pr_err("reset_gpio_base is not valid\n");
		return;
	}

	base = ioremap(fts_data->vm_info->reset_gpio_base, TOUCH_RESET_GPIO_SIZE);
	writel_relaxed(0x1, base + TOUCH_RESET_GPIO_OFFSET);
	/* wait until toggle to finish*/
	wmb();
	writel_relaxed(0x0, base + TOUCH_RESET_GPIO_OFFSET);
	/* wait until toggle to finish*/
	wmb();
	iounmap(base);
}

static void fts_trusted_touch_intr_gpio_toggle(struct fts_ts_data *fts_data,
		bool enable)
{
	void __iomem *base;
	u32 val;

	if (fts_data->bus_type != BUS_TYPE_I2C)
		return;

	if (!fts_data->vm_info->intr_gpio_base) {
		pr_err("intr_gpio_base is not valid\n");
		return;
	}

	base = ioremap(fts_data->vm_info->intr_gpio_base, TOUCH_INTR_GPIO_SIZE);
	val = readl_relaxed(base + TOUCH_INTR_GPIO_OFFSET);
	if (enable) {
		val |= BIT(0);
		writel_relaxed(val, base + TOUCH_INTR_GPIO_OFFSET);
		/* wait until toggle to finish*/
		wmb();
	} else {
		val &= ~BIT(0);
		writel_relaxed(val, base + TOUCH_INTR_GPIO_OFFSET);
		/* wait until toggle to finish*/
		wmb();
	}
	iounmap(base);
}

static int fts_ts_sgl_cmp(const void *a, const void *b)
{
	struct gh_sgl_entry *left = (struct gh_sgl_entry *)a;
	struct gh_sgl_entry *right = (struct gh_sgl_entry *)b;

	return (left->ipa_base - right->ipa_base);
}

static int fts_ts_vm_compare_sgl_desc(struct gh_sgl_desc *expected,
		struct gh_sgl_desc *received)
{
	int idx;

	if (expected->n_sgl_entries != received->n_sgl_entries)
		return -E2BIG;
	sort(received->sgl_entries, received->n_sgl_entries,
			sizeof(received->sgl_entries[0]), fts_ts_sgl_cmp, NULL);
	sort(expected->sgl_entries, expected->n_sgl_entries,
			sizeof(expected->sgl_entries[0]), fts_ts_sgl_cmp, NULL);

	for (idx = 0; idx < expected->n_sgl_entries; idx++) {
		struct gh_sgl_entry *left = &expected->sgl_entries[idx];
		struct gh_sgl_entry *right = &received->sgl_entries[idx];

		if ((left->ipa_base != right->ipa_base) ||
				(left->size != right->size)) {
			pr_err("sgl mismatch: left_base:%d right base:%d left size:%d right size:%d\n",
					left->ipa_base, right->ipa_base,
					left->size, right->size);
			return -EINVAL;
		}
	}
	return 0;
}

static int fts_ts_vm_handle_vm_hardware(struct fts_ts_data *fts_data)
{
	int rc = 0;

	if (atomic_read(&fts_data->delayed_vm_probe_pending)) {
		rc = fts_ts_probe_delayed(fts_data);
		if (rc) {
			pr_err(" Delayed probe failure on VM!\n");
			return rc;
		}
		atomic_set(&fts_data->delayed_vm_probe_pending, 0);
		return rc;
	}

	fts_irq_enable();
	fts_ts_trusted_touch_set_vm_state(fts_data, TVM_INTERRUPT_ENABLED);
	return rc;
}

static void fts_ts_trusted_touch_tvm_vm_mode_enable(struct fts_ts_data *fts_data)
{

	struct gh_sgl_desc *sgl_desc, *expected_sgl_desc;
	struct gh_acl_desc *acl_desc;
	struct irq_data *irq_data;
	int rc = 0;
	int irq = 0;

	if (fts_ts_trusted_touch_get_vm_state(fts_data) != TVM_ALL_RESOURCES_LENT_NOTIFIED) {
		pr_err("All lend notifications not received\n");
		fts_ts_trusted_touch_event_notify(fts_data,
				TRUSTED_TOUCH_EVENT_NOTIFICATIONS_PENDING);
		return;
	}

	acl_desc = fts_ts_vm_get_acl(GH_TRUSTED_VM);
	if (IS_ERR(acl_desc)) {
		pr_err("failed to populated acl data:rc=%d\n",
				PTR_ERR(acl_desc));
		goto accept_fail;
	}

	sgl_desc = gh_rm_mem_accept(fts_data->vm_info->vm_mem_handle,
			GH_RM_MEM_TYPE_IO,
			GH_RM_TRANS_TYPE_LEND,
			GH_RM_MEM_ACCEPT_VALIDATE_ACL_ATTRS |
			GH_RM_MEM_ACCEPT_VALIDATE_LABEL |
			GH_RM_MEM_ACCEPT_DONE,  TRUSTED_TOUCH_MEM_LABEL,
			acl_desc, NULL, NULL, 0);
	if (IS_ERR_OR_NULL(sgl_desc)) {
		pr_err("failed to do mem accept :rc=%d\n",
				PTR_ERR(sgl_desc));
		goto acl_fail;
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, TVM_IOMEM_ACCEPTED);

	/* Initiate session on tvm */
	if (fts_data->bus_type == BUS_TYPE_I2C)
		rc = pm_runtime_get_sync(fts_data->client->adapter->dev.parent);
	else
		rc = pm_runtime_get_sync(fts_data->spi->master->dev.parent);

	if (rc < 0) {
		pr_err("failed to get sync rc:%d\n", rc);
		goto acl_fail;
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, TVM_I2C_SESSION_ACQUIRED);

	expected_sgl_desc = fts_ts_vm_get_sgl(fts_data->vm_info);
	if (fts_ts_vm_compare_sgl_desc(expected_sgl_desc, sgl_desc)) {
		pr_err("IO sg list does not match\n");
		goto sgl_cmp_fail;
	}

	kfree(expected_sgl_desc);
	kfree(acl_desc);

	irq = gh_irq_accept(fts_data->vm_info->irq_label, -1, IRQ_TYPE_EDGE_RISING);
	fts_trusted_touch_intr_gpio_toggle(fts_data, false);
	if (irq < 0) {
		pr_err("failed to accept irq\n");
		goto accept_fail;
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, TVM_IRQ_ACCEPTED);


	irq_data = irq_get_irq_data(irq);
	if (!irq_data) {
		pr_err("Invalid irq data for trusted touch\n");
		goto accept_fail;
	}
	if (!irq_data->hwirq) {
		pr_err("Invalid irq in irq data\n");
		goto accept_fail;
	}
	if (irq_data->hwirq != fts_data->vm_info->hw_irq) {
		pr_err("Invalid irq lent\n");
		goto accept_fail;
	}

	pr_debug("irq:returned from accept:%d\n", irq);
	fts_data->irq = irq;

	rc = fts_ts_vm_handle_vm_hardware(fts_data);
	if (rc) {
		pr_err(" Delayed probe failure on VM!\n");
		goto accept_fail;
	}
	atomic_set(&fts_data->trusted_touch_enabled, 1);
	pr_info("trusted touch enabled\n");

	return;
sgl_cmp_fail:
	kfree(expected_sgl_desc);
acl_fail:
	kfree(acl_desc);
accept_fail:
	fts_ts_trusted_touch_abort_handler(fts_data,
			TRUSTED_TOUCH_EVENT_ACCEPT_FAILURE);
}
static void fts_ts_vm_irq_on_lend_callback(void *data,
					unsigned long notif_type,
					enum gh_irq_label label)
{
	struct fts_ts_data *fts_data = data;

	pr_debug("received irq lend request for label:%d\n", label);
	if (fts_ts_trusted_touch_get_vm_state(fts_data) == TVM_IOMEM_LENT_NOTIFIED)
		fts_ts_trusted_touch_set_vm_state(fts_data, TVM_ALL_RESOURCES_LENT_NOTIFIED);
	else
		fts_ts_trusted_touch_set_vm_state(fts_data, TVM_IRQ_LENT_NOTIFIED);
}

static void fts_ts_vm_mem_on_lend_handler(enum gh_mem_notifier_tag tag,
		unsigned long notif_type, void *entry_data, void *notif_msg)
{
	struct gh_rm_notif_mem_shared_payload *payload;
	struct trusted_touch_vm_info *vm_info;
	struct fts_ts_data *fts_data;

	fts_data = (struct fts_ts_data *)entry_data;
	vm_info = fts_data->vm_info;
	if (!vm_info) {
		pr_err("Invalid vm_info\n");
		return;
	}

	if (notif_type != GH_RM_NOTIF_MEM_SHARED ||
			tag != vm_info->mem_tag) {
		pr_err("Invalid command passed from rm\n");
		return;
	}

	if (!entry_data || !notif_msg) {
		pr_err("Invalid entry data passed from rm\n");
		return;
	}


	payload = (struct gh_rm_notif_mem_shared_payload  *)notif_msg;
	if (payload->trans_type != GH_RM_TRANS_TYPE_LEND ||
			payload->label != TRUSTED_TOUCH_MEM_LABEL) {
		pr_err("Invalid label or transaction type\n");
		return;
	}

	vm_info->vm_mem_handle = payload->mem_handle;
	pr_debug("received mem lend request with handle:%d\n",
		vm_info->vm_mem_handle);
	if (fts_ts_trusted_touch_get_vm_state(fts_data) == TVM_IRQ_LENT_NOTIFIED)
		fts_ts_trusted_touch_set_vm_state(fts_data, TVM_ALL_RESOURCES_LENT_NOTIFIED);
	else
		fts_ts_trusted_touch_set_vm_state(fts_data, TVM_IOMEM_LENT_NOTIFIED);
}

static int fts_ts_vm_mem_release(struct fts_ts_data *fts_data)
{
	int rc = 0;

	if (!fts_data->vm_info->vm_mem_handle) {
		pr_err("Invalid memory handle\n");
		return -EINVAL;
	}

	rc = gh_rm_mem_release(fts_data->vm_info->vm_mem_handle, 0);
	if (rc)
		pr_err("VM mem release failed: rc=%d\n", rc);

	rc = gh_rm_mem_notify(fts_data->vm_info->vm_mem_handle,
				GH_RM_MEM_NOTIFY_OWNER_RELEASED,
				fts_data->vm_info->mem_tag, 0);
	if (rc)
		pr_err("Failed to notify mem release to PVM: rc=%d\n");
	pr_debug("vm mem release succeded\n");

	fts_data->vm_info->vm_mem_handle = 0;
	return rc;
}

static void fts_ts_trusted_touch_tvm_vm_mode_disable(struct fts_ts_data *fts_data)
{
	int rc = 0;

	if (atomic_read(&fts_data->trusted_touch_abort_status)) {
		fts_ts_trusted_touch_abort_tvm(fts_data);
		return;
	}

	/*
	 * Acquire the transition lock before disabling the IRQ to avoid the race
	 * condition with fts_irq_handler. For SVM, it is acquired here and, for PVM,
	 * in fts_ts_trusted_touch_pvm_vm_mode_enable().
	 */
	mutex_lock(&fts_data->transition_lock);

	fts_irq_disable();
	fts_ts_trusted_touch_set_vm_state(fts_data, TVM_INTERRUPT_DISABLED);

	rc = gh_irq_release(fts_data->vm_info->irq_label);
	if (rc) {
		pr_err("Failed to release irq rc:%d\n", rc);
		goto error;
	} else {
		fts_ts_trusted_touch_set_vm_state(fts_data, TVM_IRQ_RELEASED);
	}
	rc = gh_irq_release_notify(fts_data->vm_info->irq_label);
	if (rc)
		pr_err("Failed to notify release irq rc:%d\n", rc);

	pr_debug("vm irq release succeded\n");

	fts_release_all_finger();

	if (fts_data->bus_type == BUS_TYPE_I2C)
		pm_runtime_put_sync(fts_data->client->adapter->dev.parent);
	else
		pm_runtime_put_sync(fts_data->spi->master->dev.parent);

	fts_ts_trusted_touch_set_vm_state(fts_data, TVM_I2C_SESSION_RELEASED);
	rc = fts_ts_vm_mem_release(fts_data);
	if (rc) {
		pr_err("Failed to release mem rc:%d\n", rc);
		goto error;
	} else {
		fts_ts_trusted_touch_set_vm_state(fts_data, TVM_IOMEM_RELEASED);
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, TRUSTED_TOUCH_TVM_INIT);
	atomic_set(&fts_data->trusted_touch_enabled, 0);
	mutex_unlock(&fts_data->transition_lock);
	pr_info("trusted touch disabled\n");
	return;
error:
	mutex_unlock(&fts_data->transition_lock);
	fts_ts_trusted_touch_abort_handler(fts_data,
			TRUSTED_TOUCH_EVENT_RELEASE_FAILURE);
}

int fts_ts_handle_trusted_touch_tvm(struct fts_ts_data *fts_data, int value)
{
	int err = 0;

	switch (value) {
	case 0:
		if ((atomic_read(&fts_data->trusted_touch_enabled) == 0) &&
			(atomic_read(&fts_data->trusted_touch_abort_status) == 0)) {
			pr_err("Trusted touch is already disabled\n");
			break;
		}
		if (atomic_read(&fts_data->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			fts_ts_trusted_touch_tvm_vm_mode_disable(fts_data);
		} else {
			pr_err("Unsupported trusted touch mode\n");
		}
		break;

	case 1:
		if (atomic_read(&fts_data->trusted_touch_enabled)) {
			pr_err("Trusted touch usecase underway\n");
			err = -EBUSY;
			break;
		}
		if (atomic_read(&fts_data->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			fts_ts_trusted_touch_tvm_vm_mode_enable(fts_data);
		} else {
			pr_err("Unsupported trusted touch mode\n");
		}
		break;

	default:
		FTS_ERROR("unsupported value: %lu\n", value);
		err = -EINVAL;
		break;
	}

	return err;
}

static void fts_ts_trusted_touch_abort_tvm(struct fts_ts_data *fts_data)
{
	int rc = 0;
	int vm_state = fts_ts_trusted_touch_get_vm_state(fts_data);

	if (vm_state >= TRUSTED_TOUCH_TVM_STATE_MAX) {
		pr_err("invalid tvm driver state: %d\n", vm_state);
		return;
	}

	switch (vm_state) {
	case TVM_INTERRUPT_ENABLED:
		fts_irq_disable();
	case TVM_IRQ_ACCEPTED:
	case TVM_INTERRUPT_DISABLED:
		rc = gh_irq_release(fts_data->vm_info->irq_label);
		if (rc)
			pr_err("Failed to release irq rc:%d\n", rc);
		rc = gh_irq_release_notify(fts_data->vm_info->irq_label);
		if (rc)
			pr_err("Failed to notify irq release rc:%d\n", rc);
	case TVM_I2C_SESSION_ACQUIRED:
	case TVM_IOMEM_ACCEPTED:
	case TVM_IRQ_RELEASED:
		fts_release_all_finger();
		if (fts_data->bus_type == BUS_TYPE_I2C)
			pm_runtime_put_sync(fts_data->client->adapter->dev.parent);
		else
			pm_runtime_put_sync(fts_data->spi->master->dev.parent);
	case TVM_I2C_SESSION_RELEASED:
		rc = fts_ts_vm_mem_release(fts_data);
		if (rc)
			pr_err("Failed to release mem rc:%d\n", rc);
	case TVM_IOMEM_RELEASED:
	case TVM_ALL_RESOURCES_LENT_NOTIFIED:
	case TRUSTED_TOUCH_TVM_INIT:
	case TVM_IRQ_LENT_NOTIFIED:
	case TVM_IOMEM_LENT_NOTIFIED:
		atomic_set(&fts_data->trusted_touch_enabled, 0);
	}

	atomic_set(&fts_data->trusted_touch_abort_status, 0);
	fts_ts_trusted_touch_set_vm_state(fts_data, TRUSTED_TOUCH_TVM_INIT);
}

#else

static void fts_ts_bus_put(struct fts_ts_data *fts_data);

static void fts_ts_trusted_touch_abort_pvm(struct fts_ts_data *fts_data)
{
	int rc = 0;
	int vm_state = fts_ts_trusted_touch_get_vm_state(fts_data);

	if (vm_state >= TRUSTED_TOUCH_PVM_STATE_MAX) {
		pr_err("Invalid driver state: %d\n", vm_state);
		return;
	}

	switch (vm_state) {
	case PVM_IRQ_RELEASE_NOTIFIED:
	case PVM_ALL_RESOURCES_RELEASE_NOTIFIED:
	case PVM_IRQ_LENT:
	case PVM_IRQ_LENT_NOTIFIED:
		rc = gh_irq_reclaim(fts_data->vm_info->irq_label);
		if (rc)
			pr_err("failed to reclaim irq on pvm rc:%d\n", rc);
	case PVM_IRQ_RECLAIMED:
	case PVM_IOMEM_LENT:
	case PVM_IOMEM_LENT_NOTIFIED:
	case PVM_IOMEM_RELEASE_NOTIFIED:
		rc = gh_rm_mem_reclaim(fts_data->vm_info->vm_mem_handle, 0);
		if (rc)
			pr_err("failed to reclaim iomem on pvm rc:%d\n", rc);
		fts_data->vm_info->vm_mem_handle = 0;
	case PVM_IOMEM_RECLAIMED:
	case PVM_INTERRUPT_DISABLED:
		fts_irq_enable();
	case PVM_I2C_RESOURCE_ACQUIRED:
	case PVM_INTERRUPT_ENABLED:
		fts_ts_bus_put(fts_data);
	case TRUSTED_TOUCH_PVM_INIT:
	case PVM_I2C_RESOURCE_RELEASED:
		atomic_set(&fts_data->trusted_touch_enabled, 0);
		atomic_set(&fts_data->trusted_touch_transition, 0);
	}

	atomic_set(&fts_data->trusted_touch_abort_status, 0);

	fts_ts_trusted_touch_set_vm_state(fts_data, TRUSTED_TOUCH_PVM_INIT);
}

static int fts_ts_clk_prepare_enable(struct fts_ts_data *fts_data)
{
	int ret;

	ret = clk_prepare_enable(fts_data->iface_clk);
	if (ret) {
		FTS_ERROR("error on clk_prepare_enable(iface_clk):%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(fts_data->core_clk);
	if (ret) {
		clk_disable_unprepare(fts_data->iface_clk);
		FTS_ERROR("error clk_prepare_enable(core_clk):%d\n", ret);
	}
	return ret;
}

static void fts_ts_clk_disable_unprepare(struct fts_ts_data *fts_data)
{
	clk_disable_unprepare(fts_data->core_clk);
	clk_disable_unprepare(fts_data->iface_clk);
}

static int fts_ts_bus_get(struct fts_ts_data *fts_data)
{
	int rc = 0;
	struct device *dev = NULL;

	cancel_work_sync(&fts_data->resume_work);
	reinit_completion(&fts_data->trusted_touch_powerdown);
	fts_ts_enable_reg(fts_data, true);

	if (fts_data->bus_type == BUS_TYPE_I2C)
		dev = fts_data->client->adapter->dev.parent;
	else
		dev = fts_data->spi->master->dev.parent;

	mutex_lock(&fts_data->fts_clk_io_ctrl_mutex);
	rc = pm_runtime_get_sync(dev);
	if (rc >= 0 &&  fts_data->core_clk != NULL &&
				fts_data->iface_clk != NULL) {
		rc = fts_ts_clk_prepare_enable(fts_data);
		if (rc)
			pm_runtime_put_sync(dev);
	}

	mutex_unlock(&fts_data->fts_clk_io_ctrl_mutex);
	return rc;
}

static void fts_ts_bus_put(struct fts_ts_data *fts_data)
{
	struct device *dev = NULL;

	if (fts_data->bus_type == BUS_TYPE_I2C)
		dev = fts_data->client->adapter->dev.parent;
	else
		dev = fts_data->spi->master->dev.parent;

	mutex_lock(&fts_data->fts_clk_io_ctrl_mutex);
	if (fts_data->core_clk != NULL && fts_data->iface_clk != NULL)
		fts_ts_clk_disable_unprepare(fts_data);
	pm_runtime_put_sync(dev);
	mutex_unlock(&fts_data->fts_clk_io_ctrl_mutex);
	complete(&fts_data->trusted_touch_powerdown);
	fts_ts_enable_reg(fts_data, false);
}

static struct gh_notify_vmid_desc *fts_ts_vm_get_vmid(gh_vmid_t vmid)
{
	struct gh_notify_vmid_desc *vmid_desc;

	vmid_desc = kzalloc(offsetof(struct gh_notify_vmid_desc,
				vmid_entries[1]), GFP_KERNEL);
	if (!vmid_desc)
		return ERR_PTR(ENOMEM);

	vmid_desc->n_vmid_entries = 1;
	vmid_desc->vmid_entries[0].vmid = vmid;
	return vmid_desc;
}

static void fts_trusted_touch_pvm_vm_mode_disable(struct fts_ts_data *fts_data)
{
	int rc = 0;

	atomic_set(&fts_data->trusted_touch_transition, 1);

	if (atomic_read(&fts_data->trusted_touch_abort_status)) {
		fts_ts_trusted_touch_abort_pvm(fts_data);
		return;
	}

	if (fts_ts_trusted_touch_get_vm_state(fts_data) != PVM_ALL_RESOURCES_RELEASE_NOTIFIED)
		pr_err("all release notifications are not received yet\n");

	rc = gh_rm_mem_reclaim(fts_data->vm_info->vm_mem_handle, 0);
	if (rc) {
		pr_err("Trusted touch VM mem reclaim failed rc:%d\n", rc);
		goto error;
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IOMEM_RECLAIMED);
	fts_data->vm_info->vm_mem_handle = 0;
	pr_debug("vm mem reclaim succeded!\n");

	rc = gh_irq_reclaim(fts_data->vm_info->irq_label);
	if (rc) {
		pr_err("failed to reclaim irq on pvm rc:%d\n", rc);
		goto error;
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IRQ_RECLAIMED);
	pr_debug("vm irq reclaim succeded!\n");

	fts_irq_enable();
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_INTERRUPT_ENABLED);
	fts_ts_bus_put(fts_data);
	atomic_set(&fts_data->trusted_touch_transition, 0);
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_I2C_RESOURCE_RELEASED);
	fts_ts_trusted_touch_set_vm_state(fts_data, TRUSTED_TOUCH_PVM_INIT);
	atomic_set(&fts_data->trusted_touch_enabled, 0);
	pr_info("trusted touch disabled\n");
	return;
error:
	fts_ts_trusted_touch_abort_handler(fts_data,
			TRUSTED_TOUCH_EVENT_RECLAIM_FAILURE);
}

static void fts_ts_vm_irq_on_release_callback(void *data,
					unsigned long notif_type,
					enum gh_irq_label label)
{
	struct fts_ts_data *fts_data = data;

	if (notif_type != GH_RM_NOTIF_VM_IRQ_RELEASED) {
		pr_err("invalid notification type\n");
		return;
	}

	if (fts_ts_trusted_touch_get_vm_state(fts_data) == PVM_IOMEM_RELEASE_NOTIFIED)
		fts_ts_trusted_touch_set_vm_state(fts_data, PVM_ALL_RESOURCES_RELEASE_NOTIFIED);
	else
		fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IRQ_RELEASE_NOTIFIED);
}

static void fts_ts_vm_mem_on_release_handler(enum gh_mem_notifier_tag tag,
		unsigned long notif_type, void *entry_data, void *notif_msg)
{
	struct gh_rm_notif_mem_released_payload *release_payload;
	struct trusted_touch_vm_info *vm_info;
	struct fts_ts_data *fts_data;

	fts_data = (struct fts_ts_data *)entry_data;
	vm_info = fts_data->vm_info;
	if (!vm_info) {
		pr_err(" Invalid vm_info\n");
		return;
	}

	if (notif_type != GH_RM_NOTIF_MEM_RELEASED) {
		pr_err(" Invalid notification type\n");
		return;
	}

	if (tag != vm_info->mem_tag) {
		pr_err(" Invalid tag\n");
		return;
	}

	if (!entry_data || !notif_msg) {
		pr_err(" Invalid data or notification message\n");
		return;
	}

	release_payload = (struct gh_rm_notif_mem_released_payload  *)notif_msg;
	if (release_payload->mem_handle != vm_info->vm_mem_handle) {
		pr_err("Invalid mem handle detected\n");
		return;
	}

	if (fts_ts_trusted_touch_get_vm_state(fts_data) == PVM_IRQ_RELEASE_NOTIFIED)
		fts_ts_trusted_touch_set_vm_state(fts_data, PVM_ALL_RESOURCES_RELEASE_NOTIFIED);
	else
		fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IOMEM_RELEASE_NOTIFIED);
}

static int fts_ts_vm_mem_lend(struct fts_ts_data *fts_data)
{
	struct gh_acl_desc *acl_desc;
	struct gh_sgl_desc *sgl_desc;
	struct gh_notify_vmid_desc *vmid_desc;
	gh_memparcel_handle_t mem_handle;
	gh_vmid_t trusted_vmid;
	int rc = 0;

	acl_desc = fts_ts_vm_get_acl(GH_TRUSTED_VM);
	if (IS_ERR(acl_desc)) {
		pr_err("Failed to get acl of IO memories for Trusted touch\n");
		PTR_ERR(acl_desc);
		return -EINVAL;
	}

	sgl_desc = fts_ts_vm_get_sgl(fts_data->vm_info);
	if (IS_ERR(sgl_desc)) {
		pr_err("Failed to get sgl of IO memories for Trusted touch\n");
		PTR_ERR(sgl_desc);
		rc = -EINVAL;
		goto sgl_error;
	}

	rc = gh_rm_mem_lend(GH_RM_MEM_TYPE_IO, 0, TRUSTED_TOUCH_MEM_LABEL,
			acl_desc, sgl_desc, NULL, &mem_handle);
	if (rc) {
		pr_err("Failed to lend IO memories for Trusted touch rc:%d\n",
							rc);
		goto error;
	}

	pr_info("vm mem lend succeded\n");

	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IOMEM_LENT);

	gh_rm_get_vmid(GH_TRUSTED_VM, &trusted_vmid);

	vmid_desc = fts_ts_vm_get_vmid(trusted_vmid);

	rc = gh_rm_mem_notify(mem_handle, GH_RM_MEM_NOTIFY_RECIPIENT_SHARED,
			fts_data->vm_info->mem_tag, vmid_desc);
	if (rc) {
		pr_err("Failed to notify mem lend to hypervisor rc:%d\n", rc);
		goto vmid_error;
	}

	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IOMEM_LENT_NOTIFIED);

	fts_data->vm_info->vm_mem_handle = mem_handle;
vmid_error:
	kfree(vmid_desc);
error:
	kfree(sgl_desc);
sgl_error:
	kfree(acl_desc);

	return rc;
}

static int fts_ts_trusted_touch_pvm_vm_mode_enable(struct fts_ts_data *fts_data)
{
	int rc = 0;
	struct trusted_touch_vm_info *vm_info = fts_data->vm_info;

	atomic_set(&fts_data->trusted_touch_transition, 1);
	mutex_lock(&fts_data->transition_lock);

	if (fts_data->suspended) {
		FTS_ERROR("Invalid power state for operation\n");
		atomic_set(&fts_data->trusted_touch_transition, 0);
		rc =  -EPERM;
		goto error;
	}

	/* i2c session start and resource acquire */
	if (fts_ts_bus_get(fts_data) < 0) {
		FTS_ERROR("fts_ts_bus_get failed\n");
		rc = -EIO;
		goto error;
	}

	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_I2C_RESOURCE_ACQUIRED);
	/* flush pending interurpts from FIFO */
	fts_irq_disable();
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_INTERRUPT_DISABLED);
	fts_release_all_finger();

	rc = fts_ts_vm_mem_lend(fts_data);
	if (rc) {
		pr_err("Failed to lend memory\n");
		goto abort_handler;
	}
	pr_debug("vm mem lend succeded\n");
	rc = gh_irq_lend_v2(vm_info->irq_label, vm_info->vm_name,
		fts_data->irq, &fts_ts_vm_irq_on_release_callback, fts_data);
	if (rc) {
		pr_err("Failed to lend irq\n");
		goto abort_handler;
	}

	pr_debug("vm irq lend succeded for irq:%d\n", fts_data->irq);
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IRQ_LENT);

	rc = gh_irq_lend_notify(vm_info->irq_label);
	if (rc) {
		pr_err("Failed to notify irq\n");
		goto abort_handler;
	}
	fts_ts_trusted_touch_set_vm_state(fts_data, PVM_IRQ_LENT_NOTIFIED);

	mutex_unlock(&fts_data->transition_lock);
	atomic_set(&fts_data->trusted_touch_transition, 0);
	atomic_set(&fts_data->trusted_touch_enabled, 1);
	pr_info("trusted touch enabled\n");
	return rc;

abort_handler:
	fts_ts_trusted_touch_abort_handler(fts_data, TRUSTED_TOUCH_EVENT_LEND_FAILURE);

error:
	mutex_unlock(&fts_data->transition_lock);
	return rc;
}

int fts_ts_handle_trusted_touch_pvm(struct fts_ts_data *fts_data, int value)
{
	int err = 0;

	switch (value) {
	case 0:
		if (atomic_read(&fts_data->trusted_touch_enabled) == 0 &&
			(atomic_read(&fts_data->trusted_touch_abort_status) == 0)) {
			pr_err("Trusted touch is already disabled\n");
			break;
		}
		if (atomic_read(&fts_data->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			fts_trusted_touch_pvm_vm_mode_disable(fts_data);
		} else {
			pr_err("Unsupported trusted touch mode\n");
		}
		break;

	case 1:
		if (atomic_read(&fts_data->trusted_touch_enabled)) {
			pr_err("Trusted touch usecase underway\n");
			err = -EBUSY;
			break;
		}
		if (atomic_read(&fts_data->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			err = fts_ts_trusted_touch_pvm_vm_mode_enable(fts_data);
		} else {
			pr_err("Unsupported trusted touch mode\n");
		}
		break;

	default:
		FTS_ERROR("unsupported value: %lu\n", value);
		err = -EINVAL;
		break;
	}
	return err;
}

#endif

static void fts_ts_trusted_touch_event_notify(struct fts_ts_data *fts_data, int event)
{
	atomic_set(&fts_data->trusted_touch_event, event);
	sysfs_notify(&fts_data->dev->kobj, NULL, "trusted_touch_event");
}

static void fts_ts_trusted_touch_abort_handler(struct fts_ts_data *fts_data, int error)
{
	atomic_set(&fts_data->trusted_touch_abort_status, error);
	pr_err("TUI session aborted with failure:%d\n", error);
	fts_ts_trusted_touch_event_notify(fts_data, error);
#ifdef CONFIG_ARCH_QTI_VM
	pr_err("Resetting touch controller\n");
	if (fts_ts_trusted_touch_get_vm_state(fts_data) >= TVM_IOMEM_ACCEPTED &&
			error == TRUSTED_TOUCH_EVENT_I2C_FAILURE) {
		pr_err("Resetting touch controller\n");
		fts_ts_trusted_touch_reset_gpio_toggle(fts_data);
	}
#endif
}

static int fts_ts_vm_init(struct fts_ts_data *fts_data)
{
	int rc = 0;
	struct trusted_touch_vm_info *vm_info;
	void *mem_cookie;

	rc = fts_ts_populate_vm_info(fts_data);
	if (rc) {
		pr_err("Cannot setup vm pipeline\n");
		rc = -EINVAL;
		goto fail;
	}

	vm_info = fts_data->vm_info;
#ifdef CONFIG_ARCH_QTI_VM
	mem_cookie = gh_mem_notifier_register(vm_info->mem_tag,
			fts_ts_vm_mem_on_lend_handler, fts_data);
	if (!mem_cookie) {
		pr_err("Failed to register on lend mem notifier\n");
		rc = -EINVAL;
		goto init_fail;
	}
	vm_info->mem_cookie = mem_cookie;
	rc = gh_irq_wait_for_lend_v2(vm_info->irq_label, GH_PRIMARY_VM,
			&fts_ts_vm_irq_on_lend_callback, fts_data);
	fts_ts_trusted_touch_set_vm_state(fts_data, TRUSTED_TOUCH_TVM_INIT);
#else
	mem_cookie = gh_mem_notifier_register(vm_info->mem_tag,
			fts_ts_vm_mem_on_release_handler, fts_data);
	if (!mem_cookie) {
		pr_err("Failed to register on release mem notifier\n");
		rc = -EINVAL;
		goto init_fail;
	}
	vm_info->mem_cookie = mem_cookie;
	fts_ts_trusted_touch_set_vm_state(fts_data, TRUSTED_TOUCH_PVM_INIT);
#endif
	return rc;
init_fail:
	fts_ts_vm_deinit(fts_data);
fail:
	return rc;
}

static void fts_ts_dt_parse_trusted_touch_info(struct fts_ts_data *fts_data)
{
	struct device_node *np = fts_data->dev->of_node;
	int rc = 0;
	const char *selection;
	const char *environment;

#ifdef CONFIG_ARCH_QTI_VM
	fts_data->touch_environment = "tvm";
#else
	fts_data->touch_environment = "pvm";
#endif

	rc = of_property_read_string(np, "focaltech,trusted-touch-mode",
								&selection);
	if (rc) {
		dev_warn(fts_data->dev,
			"%s: No trusted touch mode selection made\n", __func__);
		atomic_set(&fts_data->trusted_touch_mode,
						TRUSTED_TOUCH_MODE_NONE);
		return;
	}

	if (!strcmp(selection, "vm_mode")) {
		atomic_set(&fts_data->trusted_touch_mode,
						TRUSTED_TOUCH_VM_MODE);
		pr_err("Selected trusted touch mode to VM mode\n");
	} else {
		atomic_set(&fts_data->trusted_touch_mode,
						TRUSTED_TOUCH_MODE_NONE);
		pr_err("Invalid trusted_touch mode\n");
	}

	rc = of_property_read_string(np, "focaltech,touch-environment",
						&environment);
	if (rc) {
		dev_warn(fts_data->dev,
			"%s: No trusted touch mode environment\n", __func__);
	}
	fts_data->touch_environment = environment;
	pr_err("Trusted touch environment:%s\n",
			fts_data->touch_environment);
}

static void fts_ts_trusted_touch_init(struct fts_ts_data *fts_data)
{
	int rc = 0;

	atomic_set(&fts_data->trusted_touch_initialized, 0);
	fts_ts_dt_parse_trusted_touch_info(fts_data);

	if (atomic_read(&fts_data->trusted_touch_mode) ==
						TRUSTED_TOUCH_MODE_NONE)
		return;

	init_completion(&fts_data->trusted_touch_powerdown);

	/* Get clocks */
	fts_data->core_clk = devm_clk_get(fts_data->dev->parent,
						"m-ahb");
	if (IS_ERR(fts_data->core_clk)) {
		fts_data->core_clk = NULL;
		dev_warn(fts_data->dev,
				"%s: core_clk is not defined\n", __func__);
	}

	fts_data->iface_clk = devm_clk_get(fts_data->dev->parent,
						"se-clk");
	if (IS_ERR(fts_data->iface_clk)) {
		fts_data->iface_clk = NULL;
		dev_warn(fts_data->dev,
			"%s: iface_clk is not defined\n", __func__);
	}

	if (atomic_read(&fts_data->trusted_touch_mode) ==
						TRUSTED_TOUCH_VM_MODE) {
		rc = fts_ts_vm_init(fts_data);
		if (rc)
			pr_err("Failed to init VM\n");
	}
	atomic_set(&fts_data->trusted_touch_initialized, 1);
}

#endif

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 idh = 0;
    struct fts_ts_data *ts_data = fts_data;
    u8 chip_idh = ts_data->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);
        if ((idh == chip_idh) || (fts_check_cid(ts_data, idh) == 0)) {
            FTS_INFO("TP Ready,Device ID:0x%02x", idh);
            return 0;
        } else
            FTS_DEBUG("TP Not Ready,ReadData:0x%02x,ret:%d", idh, ret);

        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return -EIO;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid();
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */
    fts_ex_mode_recovery(ts_data);
    /* recover TP gesture state 0xD0 */
    fts_gesture_recovery(ts_data);
    FTS_FUNC_EXIT();
}

int fts_reset_proc(struct fts_ts_data *ts_data,int hdelayms)
{
    FTS_DEBUG("tp reset");
    gpio_direction_output(fts_data->pdata->reset_gpio, 0);
    msleep(1);
    gpio_direction_output(fts_data->pdata->reset_gpio, 1);
    if (hdelayms) {
        msleep(hdelayms);
    }
    ts_data->fod_info.fp_down_report = 0;
    return 0;
}

void fts_irq_disable(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (!fts_data->irq_disabled) {
        disable_irq_nosync(fts_data->irq);
        fts_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (fts_data->irq_disabled) {
        enable_irq(fts_data->irq);
        fts_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
    int ret = 0;
    u8 buf[3] = {0xEB, 0xAA, 0x09};

    if (fts_data->bus_type != BUS_TYPE_I2C)
        return;

    ret = fts_write(buf, 3);
    if (ret < 0) {
        FTS_ERROR("hid2std cmd write fail");
    } else {
        msleep(10);
        buf[0] = buf[1] = buf[2] = 0;
        ret = fts_read(NULL, 0, buf, 3);
        if (ret < 0) {
            FTS_ERROR("hid2std cmd read fail");
        } else if ((0xEB == buf[0]) && (0xAA == buf[1]) && (0x08 == buf[2])) {
            FTS_DEBUG("hidi2c change to stdi2c successful");
        } else {
            FTS_DEBUG("hidi2c change to stdi2c not support or fail");
        }
    }
}

static int fts_match_cid(struct fts_ts_data *ts_data,
                         u16 type, u8 id_h, u8 id_l, bool force)
{
#ifdef FTS_CHIP_ID_MAPPING
    u32 i = 0;
    u32 j = 0;
    struct ft_chip_id_t chip_id_list[] = FTS_CHIP_ID_MAPPING;
    u32 cid_entries = sizeof(chip_id_list) / sizeof(struct ft_chip_id_t);
    u16 id = (id_h << 8) + id_l;

    memset(&ts_data->ic_info.cid, 0, sizeof(struct ft_chip_id_t));
    for (i = 0; i < cid_entries; i++) {
        if (!force && (type == chip_id_list[i].type)) {
            break;
        } else if (force && (type == chip_id_list[i].type)) {
            FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
            ts_data->ic_info.cid = chip_id_list[i];
            return 0;
        }
    }

    if (i >= cid_entries) {
        return -ENODATA;
    }

    for (j = 0; j < FTS_MAX_CHIP_IDS; j++) {
        if (id == chip_id_list[i].chip_ids[j]) {
            FTS_DEBUG("cid:%x==%x", id, chip_id_list[i].chip_ids[j]);
            FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
            ts_data->ic_info.cid = chip_id_list[i];
            return 0;
        }
    }

    return -ENODATA;
#else
    return -EINVAL;
#endif
}


static int fts_get_chip_types(
    struct fts_ts_data *ts_data,
    u8 id_h, u8 id_l, bool fw_valid)
{
    u32 i = 0;
    struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
    u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

    if ((0x0 == id_h) || (0x0 == id_l)) {
        FTS_ERROR("id_h/id_l is 0");
        return -EINVAL;
    }

    FTS_INFO("verify id:0x%02x%02x", id_h, id_l);
    for (i = 0; i < ctype_entries; i++) {
        if (VALID == fw_valid) {
            if (((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
                || (!fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 0)))
                break;
        } else {
            if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
                || ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
                || ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl))) {
                break;
            }
        }
    }

    if (i >= ctype_entries) {
        return -ENODATA;
    }

    fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 1);
    ts_data->ic_info.ids = ctype[i];
    return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
    int ret = 0;
    u8 chip_id[2] = { 0 };
    u8 id_cmd[4] = { 0 };
    u32 id_cmd_len = 0;

    id_cmd[0] = FTS_CMD_START1;
    id_cmd[1] = FTS_CMD_START2;
    ret = fts_write(id_cmd, 2);
    if (ret < 0) {
        FTS_ERROR("start cmd write fail");
        return ret;
    }

    msleep(FTS_CMD_START_DELAY);
    id_cmd[0] = FTS_CMD_READ_ID;
    id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
    if (ts_data->ic_info.is_incell)
        id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
    else
        id_cmd_len = FTS_CMD_READ_ID_LEN;
    ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
    if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
        FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
        return -EIO;
    }

    id[0] = chip_id[0];
    id[1] = chip_id[1];
    return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not focaltech's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int cnt = 0;
    u8 chip_id[2] = { 0 };

    ts_data->ic_info.is_incell = FTS_CHIP_IDC;
    ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

    for (cnt = 0; cnt < 3; cnt++) {
        fts_reset_proc(ts_data,0);
        mdelay(FTS_CMD_START_DELAY + (cnt * 8));

        ret = fts_read_bootid(ts_data, &chip_id[0]);
        if (ret < 0) {
            FTS_DEBUG("read boot id fail,retry:%d", cnt);
            continue;
        }

        ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
        if (ret < 0) {
            FTS_DEBUG("can't get ic informaton,retry:%d", cnt);
            continue;
        }

        break;
    }

    if (cnt >= 3) {
        FTS_ERROR("get ic informaton fail");
        return -EIO;
    }


    FTS_INFO("get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
             ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
             ts_data->ic_info.cid.type);

    return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
static void fts_show_touch_buffer(u8 *data, u32 datalen)
{
    u32 i = 0;
    u32 count = 0;
    char *tmpbuf = NULL;

    tmpbuf = kzalloc(1024, GFP_KERNEL);
    if (!tmpbuf) {
        FTS_ERROR("tmpbuf zalloc fail");
        return;
    }

    for (i = 0; i < datalen; i++) {
        count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
        if (count >= 1024)
            break;
    }
    FTS_DEBUG("touch_buf:%s", tmpbuf);

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

void fts_release_all_finger(void)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
    u32 max_touches = ts_data->pdata->max_touch_number;
#endif

    mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < max_touches; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);

#if FTS_PEN_EN
    input_report_key(ts_data->pen_dev, BTN_TOOL_PEN, 0);
    input_report_key(ts_data->pen_dev, BTN_TOUCH, 0);
    input_sync(ts_data->pen_dev);
#endif

    ts_data->touch_points = 0;
    ts_data->key_state = 0;
    mutex_unlock(&ts_data->report_mutex);
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*        need report it to key event.
*        x_dim: parse from dts, means key x_coordinate, dimension:+-50
*        y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *ts_data, struct ts_event *kevent)
{
    int i = 0;
    int x = kevent->x;
    int y = kevent->y;
    int *x_dim = &ts_data->pdata->key_x_coords[0];
    int *y_dim = &ts_data->pdata->key_y_coords[0];

    if (!ts_data->pdata->have_key) {
        return -EINVAL;
    }
    for (i = 0; i < ts_data->pdata->key_number; i++) {
        if ((x >= x_dim[i] - FTS_KEY_DIM) && (x <= x_dim[i] + FTS_KEY_DIM) &&
            (y >= y_dim[i] - FTS_KEY_DIM) && (y <= y_dim[i] + FTS_KEY_DIM)) {
            if (EVENT_DOWN(kevent->flag)
                && !(ts_data->key_state & (1 << i))) {
                input_report_key(ts_data->input_dev, ts_data->pdata->keys[i], 1);
                ts_data->key_state |= (1 << i);
                FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
            } else if (EVENT_UP(kevent->flag)
                       && (ts_data->key_state & (1 << i))) {
                input_report_key(ts_data->input_dev, ts_data->pdata->keys[i], 0);
                ts_data->key_state &= ~(1 << i);
                FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
            }
            return 0;
        }
    }
    return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *ts_data, struct ts_event *events)
{
    int i = 0;
    int touch_down_point_cur = 0;
    int touch_point_pre = ts_data->touch_points;
    u32 max_touch_num = ts_data->pdata->max_touch_number;
    bool touch_event_coordinate = false;
    struct input_dev *input_dev = ts_data->input_dev;

    for (i = 0; i < ts_data->touch_event_num; i++) {
        if (fts_input_report_key(ts_data, &events[i]) == 0) {
            continue;
        }

        touch_event_coordinate = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_mt_slot(input_dev, events[i].id);
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
#if FTS_REPORT_PRESSURE_EN
            input_report_abs(input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, events[i].y);

            touch_down_point_cur |= (1 << events[i].id);
            touch_point_pre |= (1 << events[i].id);

            if ((ts_data->log_level >= 2) ||
                ((1 == ts_data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id, events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
        } else {
            input_mt_slot(input_dev, events[i].id);
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
            touch_point_pre &= ~(1 << events[i].id);
            if (ts_data->log_level >= 1) FTS_DEBUG("[B]P%d UP!", events[i].id);
        }
    }

    if (unlikely(touch_point_pre ^ touch_down_point_cur)) {
        for (i = 0; i < max_touch_num; i++)  {
            if ((1 << i) & (touch_point_pre ^ touch_down_point_cur)) {
                if (ts_data->log_level >= 1) FTS_DEBUG("[B]P%d UP!", i);
                input_mt_slot(input_dev, i);
                input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
            }
        }
    }

    if (touch_down_point_cur)
        input_report_key(input_dev, BTN_TOUCH, 1);
    else if (touch_event_coordinate || ts_data->touch_points) {
        if (ts_data->touch_points && (ts_data->log_level >= 1))
            FTS_DEBUG("[B]Points All Up!");
        input_report_key(input_dev, BTN_TOUCH, 0);
    }

    ts_data->touch_points = touch_down_point_cur;
    input_sync(input_dev);
    return 0;
}
#else
static int fts_input_report_a(struct fts_ts_data *ts_data, struct ts_event *events)
{
    int i = 0;
    int touch_down_point_num_cur = 0;
    bool touch_event_coordinate = false;
    struct input_dev *input_dev = ts_data->input_dev;

    for (i = 0; i < ts_data->touch_event_num; i++) {
        if (fts_input_report_key(ts_data, &events[i]) == 0) {
            continue;
        }

        touch_event_coordinate = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_report_abs(input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
            input_report_abs(input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, events[i].y);
            input_mt_sync(input_dev);

            touch_down_point_num_cur++;
            if ((ts_data->log_level >= 2) ||
                ((1 == ts_data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id, events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
        }
    }

    if (touch_down_point_num_cur)
        input_report_key(input_dev, BTN_TOUCH, 1);
    else if (touch_event_coordinate || ts_data->touch_points) {
        if (ts_data->touch_points && (ts_data->log_level >= 1))
            FTS_DEBUG("[A]Points All Up!");
        input_report_key(input_dev, BTN_TOUCH, 0);
        input_mt_sync(input_dev);
    }

    ts_data->touch_points = touch_down_point_num_cur;
    input_sync(input_dev);
    return 0;
}
#endif

#if FTS_PEN_EN
static int fts_input_pen_report(struct fts_ts_data *ts_data, u8 *pen_buf)
{
    struct input_dev *pen_dev = ts_data->pen_dev;
    struct pen_event *pevt = &ts_data->pevent;

    /*get information of stylus*/
    pevt->inrange = (pen_buf[2] & 0x20) ? 1 : 0;
    pevt->tip = (pen_buf[2] & 0x01) ? 1 : 0;
    pevt->flag = pen_buf[3] >> 6;
    pevt->id = pen_buf[5] >> 4;
    pevt->x = ((pen_buf[3] & 0x0F) << 8) + pen_buf[4];
    pevt->y = ((pen_buf[5] & 0x0F) << 8) + pen_buf[6];
    pevt->p = ((pen_buf[7] & 0x0F) << 8) + pen_buf[8];
    pevt->tilt_x = (short)((pen_buf[9] << 8) + pen_buf[10]);
    pevt->tilt_y = (short)((pen_buf[11] << 8) + pen_buf[12]);
    pevt->azimuth = ((pen_buf[13] << 8) + pen_buf[14]);
    pevt->tool_type = BTN_TOOL_PEN;

    input_report_key(pen_dev, BTN_STYLUS, !!(pen_buf[2] & 0x02));
    input_report_key(pen_dev, BTN_STYLUS2, !!(pen_buf[2] & 0x08));

    switch (ts_data->pen_etype) {
    case STYLUS_DEFAULT:
        if (pevt->tip && pevt->p) {
            if ((ts_data->log_level >= 2) || (!pevt->down))
                FTS_DEBUG("[PEN]x:%d,y:%d,p:%d,tip:%d,flag:%d,tilt:%d,%d DOWN",
                          pevt->x, pevt->y, pevt->p, pevt->tip, pevt->flag,
                          pevt->tilt_x, pevt->tilt_y);
            input_report_abs(pen_dev, ABS_X, pevt->x);
            input_report_abs(pen_dev, ABS_Y, pevt->y);
            input_report_abs(pen_dev, ABS_PRESSURE, pevt->p);
            input_report_abs(pen_dev, ABS_TILT_X, pevt->tilt_x);
            input_report_abs(pen_dev, ABS_TILT_Y, pevt->tilt_y);
            input_report_key(pen_dev, BTN_TOUCH, 1);
            input_report_key(pen_dev, BTN_TOOL_PEN, 1);
            pevt->down = 1;
        } else if (!pevt->tip && pevt->down) {
            FTS_DEBUG("[PEN]x:%d,y:%d,p:%d,tip:%d,flag:%d,tilt:%d,%d UP",
                      pevt->x, pevt->y, pevt->p, pevt->tip, pevt->flag,
                      pevt->tilt_x, pevt->tilt_y);
            input_report_abs(pen_dev, ABS_X, pevt->x);
            input_report_abs(pen_dev, ABS_Y, pevt->y);
            input_report_abs(pen_dev, ABS_PRESSURE, pevt->p);
            input_report_key(pen_dev, BTN_TOUCH, 0);
            input_report_key(pen_dev, BTN_TOOL_PEN, 0);
            pevt->down = 0;
        }
        input_sync(pen_dev);
        break;
    case STYLUS_HOVER:
        if (ts_data->log_level >= 1)
            FTS_DEBUG("[PEN][%02X]x:%d,y:%d,p:%d,tip:%d,flag:%d,tilt:%d,%d,%d",
                      pen_buf[2], pevt->x, pevt->y, pevt->p, pevt->tip,
                      pevt->flag, pevt->tilt_x, pevt->tilt_y, pevt->azimuth);
        input_report_abs(pen_dev, ABS_X, pevt->x);
        input_report_abs(pen_dev, ABS_Y, pevt->y);
        input_report_abs(pen_dev, ABS_Z, pevt->azimuth);
        input_report_abs(pen_dev, ABS_PRESSURE, pevt->p);
        input_report_abs(pen_dev, ABS_TILT_X, pevt->tilt_x);
        input_report_abs(pen_dev, ABS_TILT_Y, pevt->tilt_y);
        input_report_key(pen_dev, BTN_TOOL_PEN, EVENT_DOWN(pevt->flag));
        input_report_key(pen_dev, BTN_TOUCH, pevt->tip);
        input_sync(pen_dev);
        break;
    default:
        FTS_ERROR("Unknown stylus event");
        break;
    }

    return 0;
}
#endif

static int fts_read_touchdata(struct fts_ts_data *ts_data, u8 *buf)
{
    int ret = 0;

    ts_data->touch_addr = 0x01;
    ret = fts_read(&ts_data->touch_addr, 1, buf, ts_data->touch_size);

    if (((0xEF == buf[1]) && (0xEF == buf[2]) && (0xEF == buf[3]))
        || ((ret < 0) && (0xEF == buf[0]))) {
        fts_release_all_finger();
        /* check if need recovery fw */
        fts_fw_recovery();
        ts_data->fw_is_running = true;
        return 1;
    } else if (ret < 0) {
        FTS_ERROR("touch data(%x) abnormal,ret:%d", buf[1], ret);
        return ret;
    }


    return 0;
}


static int fts_read_parse_touchdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
    int ret = 0;
    u8 gesture_en = 0xFF;

    memset(touch_buf, 0xFF, FTS_MAX_TOUCH_BUF);
    ts_data->ta_size = ts_data->touch_size;

    /*read touch data*/
    ret = fts_read_touchdata(ts_data, touch_buf);
    if (ret < 0) {
        FTS_ERROR("read touch data fails");
        return TOUCH_ERROR;
    }

    if (ts_data->log_level >= 3)
        fts_show_touch_buffer(touch_buf, ts_data->ta_size);

    if (ret)
        return TOUCH_IGNORE;

    /*gesture*/
    if (ts_data->suspended && ts_data->gesture_support) {
        ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_en);
        if ((ret >= 0) && (gesture_en == ENABLE))
            return TOUCH_GESTURE;
        else
            FTS_DEBUG("gesture not enable in fw, don't process gesture");
    }

    if ((touch_buf[1] == 0xFF) && (touch_buf[2] == 0xFF)
        && (touch_buf[3] == 0xFF) && (touch_buf[4] == 0xFF)) {
        FTS_INFO("touch buff is 0xff, need recovery state");
        return TOUCH_FW_INIT;
    }

    return ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);
}

static int fts_irq_read_report(struct fts_ts_data *ts_data)
{
    int i = 0;
    int max_touch_num = ts_data->pdata->max_touch_number;
    int touch_etype = 0;
    u8 event_num = 0;
    u8 finger_num = 0;
    u8 pointid = 0;
    u8 base = 0;
    u8 *touch_buf = ts_data->touch_buf;
    struct ts_event *events = ts_data->events;

    touch_etype = fts_read_parse_touchdata(ts_data, touch_buf);
    switch (touch_etype) {
    case TOUCH_DEFAULT:
        finger_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
        if (finger_num > max_touch_num) {
            FTS_ERROR("invalid point_num(%d)", finger_num);
            return -EIO;
        }

        for (i = 0; i < max_touch_num; i++) {
            base = FTS_ONE_TCH_LEN * i + 2;
            pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
            if (pointid >= FTS_MAX_ID)
                break;
            else if (pointid >= max_touch_num) {
                FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
                return -EINVAL;
            }

            events[i].id = pointid;
            events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
            events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) \
                          + (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
            events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) \
                          + (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
            events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
            events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
            if (events[i].p <= 0) events[i].p = 0x3F;
            if (events[i].area <= 0) events[i].area = 0x09;

            event_num++;
            if (EVENT_DOWN(events[i].flag) && (finger_num == 0)) {
                FTS_INFO("abnormal touch data from fw");
                return -EIO;
            }
        }

        if (event_num == 0) {
            FTS_INFO("no touch point information(%02x)", touch_buf[2]);
            return -EIO;
        }
        ts_data->touch_event_num = event_num;

        mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
        fts_input_report_b(ts_data, events);
#else
        fts_input_report_a(ts_data, events);
#endif
        mutex_unlock(&ts_data->report_mutex);
        break;

#if FTS_PEN_EN
    case TOUCH_PEN:
        mutex_lock(&ts_data->report_mutex);
        fts_input_pen_report(ts_data, touch_buf);
        mutex_unlock(&ts_data->report_mutex);
        break;
#endif

    case TOUCH_EVENT_NUM:
        event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
        if (!event_num || (event_num > max_touch_num)) {
            FTS_ERROR("invalid touch event num(%d)", event_num);
            return -EIO;
        }

        ts_data->touch_event_num = event_num;
        for (i = 0; i < event_num; i++) {
            base = FTS_ONE_TCH_LEN * i + 2;
            pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
            if (pointid >= max_touch_num) {
                FTS_ERROR("touch point ID(%d) beyond max_touch_number(%d)",
                          pointid, max_touch_num);
                return -EINVAL;
            }

            events[i].id = pointid;
            events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
            events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) \
                          + (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
            events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) \
                          + (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
            events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
            events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
            if (events[i].p <= 0) events[i].p = 0x3F;
            if (events[i].area <= 0) events[i].area = 0x09;
        }

        mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
        fts_input_report_b(ts_data, events);
#else
        fts_input_report_a(ts_data, events);
#endif
        mutex_unlock(&ts_data->report_mutex);
        break;

    case TOUCH_EXTRA_MSG:
        if (!ts_data->touch_analysis_support) {
            FTS_ERROR("touch_analysis is disabled");
            return -EINVAL;
        }

        event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
        if (!event_num || (event_num > max_touch_num)) {
            FTS_ERROR("invalid touch event num(%d)", event_num);
            return -EIO;
        }

        ts_data->touch_event_num = event_num;
        for (i = 0; i < event_num; i++) {
            base = FTS_ONE_TCH_LEN * i + 4;
            pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
            if (pointid >= max_touch_num) {
                FTS_ERROR("touch point ID(%d) beyond max_touch_number(%d)",
                          pointid, max_touch_num);
                return -EINVAL;
            }

            events[i].id = pointid;
            events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
            events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) \
                          + (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
            events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) \
                          + (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
            events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
            events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
            if (events[i].p <= 0) events[i].p = 0x3F;
            if (events[i].area <= 0) events[i].area = 0x09;
        }

        mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
        fts_input_report_b(ts_data, events);
#else
        fts_input_report_a(ts_data, events);
#endif
        mutex_unlock(&ts_data->report_mutex);
        break;

    case TOUCH_GESTURE:
        if (0 == fts_gesture_readdata(ts_data, touch_buf)) {
            FTS_INFO("succuss to get gesture data in irq handler");
        }
        break;

    case TOUCH_FW_INIT:
        fts_release_all_finger();
        fts_tp_state_recovery(ts_data);
        break;

    case TOUCH_IGNORE:
    case TOUCH_ERROR:
        break;

    default:
        FTS_INFO("unknown touch event(%d)", touch_etype);
        break;
    }

    return 0;
}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
    struct fts_ts_data *ts_data = fts_data;
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    int ret = 0;

    if ((ts_data->suspended) && (ts_data->pm_suspend)) {
        ret = wait_for_completion_timeout(
                  &ts_data->pm_completion,
                  msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
        if (!ret) {
            FTS_ERROR("Bus don't resume from pm(deep),timeout,skip irq");
            return IRQ_HANDLED;
        }
    }
#endif
    if ((ts_data->suspended) && (ts_data->gesture_support)) {
        pm_stay_awake(ts_data->dev);
        schedule_delayed_work(&ts_data->gesture_work,msecs_to_jiffies(2000));
        msleep(100);
    }
    ts_data->intr_jiffies = jiffies;
    fts_prc_queue_work(ts_data);
    fts_irq_read_report(ts_data);
    if (ts_data->touch_analysis_support && ts_data->ta_flag) {
        ts_data->ta_flag = 0;
        if (ts_data->ta_buf && ts_data->ta_size)
            memcpy(ts_data->ta_buf, ts_data->touch_buf, ts_data->ta_size);
        wake_up_interruptible(&ts_data->ts_waitqueue);
    }

    return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
    FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
    ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
                               pdata->irq_gpio_flags,
                               FTS_DRIVER_NAME, ts_data);

    return ret;
}

#if FTS_PEN_EN
static int fts_input_pen_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct input_dev *pen_dev;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    FTS_FUNC_ENTER();
    pen_dev = input_allocate_device();
    if (!pen_dev) {
        FTS_ERROR("Failed to allocate memory for input_pen device");
        return -ENOMEM;
    }

    pen_dev->dev.parent = ts_data->dev;
    pen_dev->name = FTS_DRIVER_PEN_NAME;
    pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    __set_bit(ABS_X, pen_dev->absbit);
    __set_bit(ABS_Y, pen_dev->absbit);
    __set_bit(BTN_STYLUS, pen_dev->keybit);
    __set_bit(BTN_STYLUS2, pen_dev->keybit);
    __set_bit(BTN_TOUCH, pen_dev->keybit);
    __set_bit(BTN_TOOL_PEN, pen_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
    input_set_abs_params(pen_dev, ABS_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(pen_dev, ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(pen_dev, ABS_PRESSURE, 0, 4096, 0, 0);
    input_set_abs_params(pen_dev, ABS_TILT_X, -9000, 9000, 0, 0);
    input_set_abs_params(pen_dev, ABS_TILT_Y, -9000, 9000, 0, 0);
    input_set_abs_params(pen_dev, ABS_Z, 0, 36000, 0, 0);

    ret = input_register_device(pen_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_free_device(pen_dev);
        pen_dev = NULL;
        return ret;
    }

    ts_data->pen_dev = pen_dev;
    ts_data->pen_etype = STYLUS_DEFAULT;
    FTS_FUNC_EXIT();
    return 0;
}
#endif

static int fts_input_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    FTS_FUNC_ENTER();
    input_dev = input_allocate_device();
    if (!input_dev) {
        FTS_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = FTS_DRIVER_NAME;
    if (ts_data->bus_type == BUS_TYPE_I2C)
        input_dev->id.bustype = BUS_I2C;
    else
        input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = ts_data->dev;

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    if (pdata->have_key) {
        FTS_INFO("set key capabilities");
        for (key_num = 0; key_num < pdata->key_number; key_num++)
            input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
    }

#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

    ret = input_register_device(input_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

#if FTS_PEN_EN
    ret = fts_input_pen_init(ts_data);
    if (ret) {
        FTS_ERROR("Input-pen device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }
#endif

    ts_data->input_dev = input_dev;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_buffer_init(struct fts_ts_data *ts_data)
{
    ts_data->touch_buf = (u8 *)kzalloc(FTS_MAX_TOUCH_BUF, GFP_KERNEL);
    if (!ts_data->touch_buf) {
        FTS_ERROR("failed to alloc memory for touch buf");
        return -ENOMEM;
    }

    ts_data->touch_size = FTS_TOUCH_DATA_LEN;


    ts_data->touch_analysis_support = 0;
    ts_data->ta_flag = 0;
    ts_data->ta_size = 0;

    return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
    int ret = 0;

    ts->pinctrl = devm_pinctrl_get(ts->dev);
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        FTS_ERROR("Failed to get pinctrl, please check dts");
        ret = PTR_ERR(ts->pinctrl);
        goto err_pinctrl_get;
    }

    ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
    if (IS_ERR_OR_NULL(ts->pins_active)) {
        FTS_ERROR("Pin state[active] not found");
        ret = PTR_ERR(ts->pins_active);
        goto err_pinctrl_lookup;
    }

    ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
    if (IS_ERR_OR_NULL(ts->pins_suspend)) {
        FTS_ERROR("Pin state[suspend] not found");
        ret = PTR_ERR(ts->pins_suspend);
        goto err_pinctrl_lookup;
    }

    ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
    if (IS_ERR_OR_NULL(ts->pins_release)) {
        FTS_ERROR("Pin state[release] not found");
        ret = PTR_ERR(ts->pins_release);
    }

    return 0;
err_pinctrl_lookup:
    if (ts->pinctrl) {
        devm_pinctrl_put(ts->pinctrl);
    }
err_pinctrl_get:
    ts->pinctrl = NULL;
    ts->pins_release = NULL;
    ts->pins_suspend = NULL;
    ts->pins_active = NULL;
    return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_active) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
        if (ret < 0) {
            FTS_ERROR("Set normal pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_suspend) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
        if (ret < 0) {
            FTS_ERROR("Set suspend pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl) {
        if (IS_ERR_OR_NULL(ts->pins_release)) {
            devm_pinctrl_put(ts->pinctrl);
            ts->pinctrl = NULL;
        } else {
            ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
            if (ret < 0)
                FTS_ERROR("Set gesture pin state error:%d", ret);
        }
    }

    return ret;
}
#endif /* FTS_PINCTRL_EN */

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
    int ret = 0;

    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        FTS_ERROR("vdd is invalid");
        return -EINVAL;
    }

    FTS_FUNC_ENTER();
    if (enable) {
        if (ts_data->power_disabled) {
            FTS_DEBUG("regulator enable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            msleep(1);
            if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
                ret = regulator_enable(ts_data->iovdd);
                if (ret) {
                    FTS_ERROR("enable iovdd regulator failed,ret=%d", ret);
                }
            }
            msleep(1);
            ret = regulator_enable(ts_data->vdd);
            if (ret) {
                FTS_ERROR("enable vdd regulator failed,ret=%d", ret);
            }

            ts_data->power_disabled = false;
        }
    } else {
        if (!ts_data->power_disabled) {
            FTS_DEBUG("regulator disable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            msleep(1);
            ret = regulator_disable(ts_data->vdd);
            if (ret) {
                FTS_ERROR("disable vdd regulator failed,ret=%d", ret);
            }
            if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
                ret = regulator_disable(ts_data->iovdd);
                if (ret) {
                    FTS_ERROR("disable iovdd regulator failed,ret=%d", ret);
                }
            }
            ts_data->power_disabled = true;
        }
    }

    FTS_FUNC_EXIT();
    return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after fts_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    ts_data->vdd = regulator_get(ts_data->dev, "vdd");
    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        ret = PTR_ERR(ts_data->vdd);
        FTS_ERROR("get vdd regulator failed,ret=%d", ret);
        return ret;
    }

    if (regulator_count_voltages(ts_data->vdd) > 0) {
        ret = regulator_set_voltage(ts_data->vdd, FTS_VTG_MIN_UV,
                                    FTS_VTG_MAX_UV);
        if (ret) {
            FTS_ERROR("vdd regulator set_vtg failed ret=%d", ret);
            regulator_put(ts_data->vdd);
            return ret;
        }
    }

    ts_data->iovdd = regulator_get(ts_data->dev, "iovdd");
    if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
        if (regulator_count_voltages(ts_data->iovdd) > 0) {
            ret = regulator_set_voltage(ts_data->iovdd,
                                        FTS_SPI_VTG_MIN_UV,
                                        FTS_SPI_VTG_MAX_UV);
            if (ret) {
                FTS_ERROR("iovdd regulator set_vtg failed,ret=%d", ret);
                regulator_put(ts_data->iovdd);
            }
        }
    }

#if FTS_PINCTRL_EN
    fts_pinctrl_init(ts_data);
    fts_pinctrl_select_normal(ts_data);
#endif

    ts_data->power_disabled = true;
    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret) {
        FTS_ERROR("fail to enable power(regulator)");
    }

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
    fts_pinctrl_select_release(ts_data);
#endif

    fts_power_source_ctrl(ts_data, DISABLE);

    if (!IS_ERR_OR_NULL(ts_data->vdd)) {
        if (regulator_count_voltages(ts_data->vdd) > 0)
            regulator_set_voltage(ts_data->vdd, 0, FTS_VTG_MAX_UV);
        regulator_put(ts_data->vdd);
    }

    if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
        if (regulator_count_voltages(ts_data->iovdd) > 0)
            regulator_set_voltage(ts_data->iovdd, 0, FTS_SPI_VTG_MAX_UV);
        regulator_put(ts_data->iovdd);
    }

    return 0;
}

static int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if FTS_PINCTRL_EN
    fts_pinctrl_select_suspend(ts_data);
#endif
    /*
    ret = fts_power_source_ctrl(ts_data, DISABLE);
    if (ret < 0) {
        FTS_ERROR("power off fail, ret=%d", ret);
    }
    */
    return ret;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if FTS_PINCTRL_EN
    fts_pinctrl_select_normal(ts_data);
#endif
    /*
    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret < 0) {
        FTS_ERROR("power on fail, ret=%d", ret);
    }
    */
    return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->irq_gpio)) {
        ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->irq_gpio);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata->reset_gpio, 1);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
    }

    FTS_FUNC_EXIT();
    return 0;

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
        gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
    FTS_FUNC_EXIT();
    return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != FTS_COORDS_ARR_SIZE) {
        FTS_ERROR("invalid:%s, size:%d", name, coords_size);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np, name, coords, coords_size);
    if (ret < 0) {
        FTS_ERROR("Unable to read %s, please check dts", name);
        pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
        pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
        pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
        pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
        return -ENODATA;
    } else {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    }

    FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
             pdata->y_min, pdata->y_max);
    return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;

    FTS_FUNC_ENTER();

    ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
    if (ret < 0)
        FTS_ERROR("Unable to get display-coords");

    /* key */
    pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
    if (pdata->have_key) {
        ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key number undefined!");

        ret = of_property_read_u32_array(np, "focaltech,keys",
                                         pdata->keys, pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Keys undefined!");
        else if (pdata->key_number > FTS_MAX_KEYS)
            pdata->key_number = FTS_MAX_KEYS;

        ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
                                         pdata->key_x_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key Y Coords undefined!");

        ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
                                         pdata->key_y_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key X Coords undefined!");

        FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
                 "coords:(%d,%d),(%d,%d),(%d,%d)",
                 pdata->key_number,
                 pdata->keys[0], pdata->keys[1], pdata->keys[2],
                 pdata->key_x_coords[0], pdata->key_y_coords[0],
                 pdata->key_x_coords[1], pdata->key_y_coords[1],
                 pdata->key_x_coords[2], pdata->key_y_coords[2]);
    }

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        FTS_ERROR("Unable to get reset_gpio");

    pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
                      0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        FTS_ERROR("Unable to get irq_gpio");

    ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
    if (ret < 0) {
        FTS_ERROR("Unable to get max-touch-number, please check dts");
        pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
    } else {
        if (temp_val < 2)
            pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
        else if (temp_val > FTS_MAX_POINTS_SUPPORT)
            pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
        else
            pdata->max_touch_number = temp_val;
    }

    FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

    FTS_FUNC_EXIT();
    return 0;
}

static void fts_resume_work(struct work_struct *work)
{
    struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
                                  resume_work);

    fts_ts_resume(ts_data->dev);
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (FB_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;

static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        FTS_ERROR("find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            FTS_INFO("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    FTS_ERROR("no find drm_panel");
    return -ENODEV;
}

static void fts_ts_panel_notifier_callback(enum panel_event_notifier_tag tag,
                        struct panel_event_notification *notification, void *client_data)
{
    struct fts_ts_data *ts_data = client_data;

    if (!notification) {
        FTS_ERROR("Invalid notification\n");
        return;
    }

    FTS_DEBUG("Notification type:%d, early_trigger:%d",
                     notification->notif_type,
                     notification->notif_data.early_trigger);
    switch (notification->notif_type) {
    case DRM_PANEL_EVENT_UNBLANK:
        if (notification->notif_data.early_trigger)
            FTS_DEBUG("resume notification pre commit\n");
        else
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        break;
    case DRM_PANEL_EVENT_BLANK:
        if (notification->notif_data.early_trigger) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else {
            FTS_DEBUG("suspend notification post commit\n");
        }
        break;
    case DRM_PANEL_EVENT_BLANK_LP:
        FTS_DEBUG("received lp event\n");
        break;
    case DRM_PANEL_EVENT_FPS_CHANGE:
        FTS_DEBUG("shashank:Received fps change old fps:%d new fps:%d\n",
                         notification->notif_data.old_fps,
                         notification->notif_data.new_fps);
        break;
    default:
        FTS_DEBUG("notification serviced :%d\n",
                         notification->notif_type);
        break;
    }
}

#else
static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!((event == MSM_DRM_EARLY_EVENT_BLANK )
          || (event == MSM_DRM_EVENT_BLANK))) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case MSM_DRM_BLANK_UNBLANK:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case MSM_DRM_BLANK_POWERDOWN:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void fts_ts_early_suspend(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    cancel_work_sync(&fts_data->resume_work);
    fts_ts_suspend(ts_data->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
}
#endif

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
    int ret = 0;
    void *cookie = NULL;
    int pdata_size = sizeof(struct fts_ts_platform_data);

    FTS_FUNC_ENTER();
    FTS_INFO("%s", FTS_DRIVER_VERSION);
    ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!ts_data->pdata) {
        FTS_ERROR("allocate memory for platform_data fail");
        return -ENOMEM;
    }

    if (ts_data->dev->of_node) {
        ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
        if (ret)
            FTS_ERROR("device-tree parse fail");

#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
        ret = drm_check_dt(ts_data->dev->of_node);
        if (ret) {
            if (drm_check_count++ < DRM_CHK_MAX_COUNTS) {
                FTS_INFO("parse drm-panel fail, count: %d", drm_check_count);
                return -EPROBE_DEFER;
            }
            FTS_ERROR("parse drm-panel fail, do not retry again");
        }
#endif
#endif
    } else {
        if (ts_data->dev->platform_data) {
            memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
        } else {
            FTS_ERROR("platform_data is null");
            return -ENODEV;
        }
    }

    ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
    if (!ts_data->ts_workqueue) {
        FTS_ERROR("create fts workqueue fail");
    }

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->bus_lock);
    init_waitqueue_head(&ts_data->ts_waitqueue);

    /* Init communication interface */
    ret = fts_bus_init(ts_data);
    if (ret) {
        FTS_ERROR("bus initialize fail");
        goto err_bus_init;
    }

    ret = fts_input_init(ts_data);
    if (ret) {
        FTS_ERROR("input initialize fail");
        goto err_input_init;
    }

    ret = fts_buffer_init(ts_data);
    if (ret) {
        FTS_ERROR("buffer init fail");
        goto err_buffer_init;
    }

    ret = fts_gpio_configure(ts_data);
    if (ret) {
        FTS_ERROR("configure the gpios fail");
        goto err_gpio_config;
    }

#if FTS_POWER_SOURCE_CUST_EN
    ret = fts_power_source_init(ts_data);
    if (ret) {
        FTS_ERROR("fail to get power(regulator)");
        goto err_power_init;
    }
#endif

#if (!FTS_CHIP_IDC)
    fts_reset_proc(ts_data,200);
#endif

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        FTS_ERROR("not focal IC, unregister driver");
        goto err_irq_req;
    }

    ret = fts_create_apk_debug_channel(ts_data);
    if (ret) {
        FTS_ERROR("create apk debug node fail");
    }

    ret = fts_create_sysfs(ts_data);
    if (ret) {
        FTS_ERROR("create sysfs node fail");
    }

    ret = fts_point_report_check_init(ts_data);
    if (ret) {
        FTS_ERROR("init point report check fail");
    }

    ret = fts_ex_mode_init(ts_data);
    if (ret) {
        FTS_ERROR("init glove/cover/charger fail");
    }

    ret = fts_gesture_init(ts_data);
    if (ret) {
        FTS_ERROR("init gesture fail");
    }

#if FTS_TEST_EN
    ret = fts_test_init(ts_data);
    if (ret) {
        FTS_ERROR("init host test fail");
    }
#endif

    ret = fts_esdcheck_init(ts_data);
    if (ret) {
        FTS_ERROR("init esd check fail");
    }

    ret = fts_irq_registration(ts_data);
    if (ret) {
        FTS_ERROR("request irq failed");
        goto err_irq_req;
    }

    ret = fts_fwupg_init(ts_data);
    if (ret) {
        FTS_ERROR("init fw upgrade fail");
    }

    if (ts_data->ts_workqueue) {
        INIT_WORK(&ts_data->resume_work, fts_resume_work);
    }

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    init_completion(&ts_data->pm_completion);
    ts_data->pm_suspend = false;
#endif

#if defined(CONFIG_FB)
    ts_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    }
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
    if (active_panel) {
        cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
                     PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH, active_panel,
                     &fts_ts_panel_notifier_callback, ts_data);
        ts_data->notifier_cookie = cookie;
        if (!cookie){
            FTS_ERROR("[DRM]Failed to register for panel events: %d\n", cookie);
        }
            FTS_DEBUG("registered for panel notifications panel: 0x%x\n",
                     active_panel);
    }
#else
    ret = msm_drm_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[DRM]Unable to register fb_notifier: %d\n", ret);
    }
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    ts_data->early_suspend.suspend = fts_ts_early_suspend;
    ts_data->early_suspend.resume = fts_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif

    FTS_FUNC_EXIT();
    return 0;

err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
    fts_power_source_exit(ts_data);
#endif
    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);
    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
    kfree_safe(ts_data->touch_buf);
err_buffer_init:
    input_unregister_device(ts_data->input_dev);
#if FTS_PEN_EN
    input_unregister_device(ts_data->pen_dev);
#endif
err_input_init:
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
    kfree_safe(ts_data->bus_tx_buf);
    kfree_safe(ts_data->bus_rx_buf);
    kfree_safe(ts_data->pdata);

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();

    cancel_work_sync(&fts_data->resume_work);
    fts_point_report_check_exit(ts_data);
    fts_release_apk_debug_channel(ts_data);
    fts_remove_sysfs(ts_data);
    fts_ex_mode_exit(ts_data);

    fts_fwupg_exit(ts_data);

#if FTS_TEST_EN
    fts_test_exit(ts_data);
#endif

    fts_esdcheck_exit(ts_data);

    fts_gesture_exit(ts_data);

    free_irq(ts_data->irq, ts_data);

    fts_bus_exit(ts_data);

    input_unregister_device(ts_data->input_dev);
#if FTS_PEN_EN
    input_unregister_device(ts_data->pen_dev);
#endif

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);

#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("[FB]Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
    if (active_panel && ts_data->notifier_cookie)
        panel_event_notifier_unregister(&ts_data->notifier_cookie);
#else
    if (msm_drm_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("[DRM]Error occurred while unregistering fb_notifier.\n");
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);

    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
    fts_power_source_exit(ts_data);
#endif

    kfree_safe(ts_data->touch_buf);
    kfree_safe(ts_data->pdata);
    kfree_safe(ts_data);

    FTS_FUNC_EXIT();

    return 0;
}

static int fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_FUNC_ENTER();
    if (ts_data->suspended) {
        FTS_INFO("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
        FTS_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

    fts_esdcheck_suspend(ts_data);

    if (ts_data->gesture_support) {
        fts_gesture_suspend(ts_data);
    } else {

        FTS_INFO("make TP enter into sleep mode");
        ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
        if (ret < 0)
            FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

        if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
            ret = fts_power_source_suspend(ts_data);
            if (ret < 0) {
                FTS_ERROR("power enter suspend fail");
            }
#endif
        }
    }

    fts_release_all_finger();
    ts_data->suspended = true;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    FTS_INFO("down flag = %d,blank flag = %d", ts_data->fod_info.fp_down_report, ts_data->blank_up);
    if((ts_data->fod_info.fp_down_report) && (ts_data->blank_up)){
        ts_data->fod_info.fp_down_report = 0;
        FTS_DEBUG("Gesture Code up supplement=%d", ts_data->fod_gesture_id);
        input_report_key(input_dev, ts_data->fod_gesture_id, 0);
        input_sync(input_dev);
    }
    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
        FTS_DEBUG("Already in awake state");
        return 0;
    }

    ts_data->suspended = false;
    fts_release_all_finger();

    if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
        fts_power_source_resume(ts_data);
#endif
        fts_reset_proc(ts_data,200);
    }

    fts_wait_tp_to_valid();
    fts_ex_mode_recovery(ts_data);

    fts_esdcheck_resume(ts_data);

    if (ts_data->gesture_support) {
        fts_gesture_resume(ts_data);
    }

    FTS_FUNC_EXIT();
    ts_data->blank_up = 0;
    return 0;
}

#if 0//defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
static int fts_pm_suspend(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_INFO("system enters into pm_suspend");
    ts_data->pm_suspend = true;
    reinit_completion(&ts_data->pm_completion);
    return 0;
}

static int fts_pm_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_INFO("system resumes from pm_suspend");
    ts_data->pm_suspend = false;
    complete(&ts_data->pm_completion);
    return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
    .suspend = fts_pm_suspend,
    .resume = fts_pm_resume,
};
#endif

static void gesture_work_func(struct work_struct *work)
{
    struct fts_ts_data *ts_data = fts_data;
    FTS_INFO("enter pm relax!\n");
    pm_relax(ts_data->dev);
}

/*****************************************************************************
* TP Driver
*****************************************************************************/
static int fts_ts_probe(struct spi_device *spi)
{
    int ret = 0;
    struct fts_ts_data *ts_data = NULL;

    FTS_INFO("Touch Screen(SPI BUS) driver prboe...");
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret) {
        FTS_ERROR("spi setup fail");
        return ret;
    }

    /* malloc memory for global struct variable */
    ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        FTS_ERROR("allocate memory for fts_data fail");
        return -ENOMEM;
    }

    fts_data = ts_data;
    ts_data->spi = spi;
    ts_data->dev = &spi->dev;
    ts_data->log_level = 1;

    ts_data->bus_type = BUS_TYPE_SPI_V2;
    spi_set_drvdata(spi, ts_data);

    ret = fts_ts_probe_entry(ts_data);
    if (ret) {
        FTS_ERROR("Touch Screen(SPI BUS) driver probe fail");
        kfree_safe(ts_data);
        return ret;
    }
    device_init_wakeup(ts_data->dev,true);
    INIT_DELAYED_WORK(&ts_data->gesture_work, gesture_work_func);
    FTS_INFO("Touch Screen(SPI BUS) driver prboe successfully");
    return 0;
}

static int fts_ts_remove(struct spi_device *spi)
{
    return fts_ts_remove_entry(spi_get_drvdata(spi));
}

static const struct spi_device_id fts_ts_id[] = {
    {FTS_DRIVER_NAME, 0},
    {},
};
static const struct of_device_id fts_dt_match[] = {
    {.compatible = "focaltech,fts_ts", },
    {},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct spi_driver fts_ts_driver = {
    .probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .driver = {
        .name = FTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#if 0//defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
        .pm = &fts_dev_pm_ops,
#endif
        .of_match_table = of_match_ptr(fts_dt_match),
    },
    .id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    ret = spi_register_driver(&fts_ts_driver);
    if ( ret != 0 ) {
        FTS_ERROR("Focaltech touch screen driver init failed!");
    }
    FTS_FUNC_EXIT();
    return ret;
}

static void __exit fts_ts_exit(void)
{
    spi_unregister_driver(&fts_ts_driver);
}

late_initcall(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
