#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include "include/nothing_secure_element.h"
#include <linux/init.h>
#include <linux/platform_device.h>

#define NOTHING_SEC_COMPATIBLE_NAME       "nothing,security_elementm"
#define OEM_SEC_BOOT_ADDR_NAME            "oem_sec_boot_addr"

static struct proc_dir_entry *nothing_secure_dir = NULL;
static char* nothing_secure_dir_name = "nothing_secure";
static struct secure_data *secure_data_ptr = NULL;
static uint32_t oem_sec_boot_addr = 0;

static int secure_element_parse_parent_dts(struct secure_data *secure_data)
{
	int ret = 0;
	struct device *dev = NULL;
	struct device_node *np = NULL;

	if (!secure_data || !secure_data->dev) {
		ret = -1;
		goto exit;
	}
	dev = secure_data->dev;
	np = dev->of_node;

	ret = of_property_read_u32(np, OEM_SEC_BOOT_ADDR_NAME, &(secure_data->oem_sec_boot_addr));
	if (ret) {
		dev_err(secure_data->dev, "the secure param %s is not found !\n", OEM_SEC_BOOT_ADDR_NAME);
		ret = -1;
		goto exit;
	}

	oem_sec_boot_addr = secure_data->oem_sec_boot_addr;
	dev_info(secure_data->dev, "oem_sec_boot_addr: %x\n", secure_data->oem_sec_boot_addr);

exit:
	return ret;
}

secure_type get_oem_sec_boot(void)
{
	secure_type ret = SECURE_UNKNOWN;
	void __iomem *oem_config_base;
	uint32_t oem_sec_boot = 0;

	oem_config_base = ioremap(oem_sec_boot_addr, 4);
	oem_sec_boot = __raw_readl(oem_config_base);
	iounmap(oem_config_base);
	oem_sec_boot = oem_sec_boot >> 16;
	oem_sec_boot = oem_sec_boot & 0x0003;

	if (oem_sec_boot == 0x0001) {
			ret = SECURE_DEV;
	} else if (oem_sec_boot == 0x0003) {
			ret = SECURE_PROD;
	} else {
			ret = SECURE_OFF;
	}

	return ret;
}

static ssize_t oem_sec_boot_read_proc(struct file *file, char __user *buf,
				size_t count, loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	secure_type oem_sec_boot = get_oem_sec_boot();

	len = sprintf(page, "%d", (int)oem_sec_boot);

	if (len > *off) {
			len -= *off;
	}
	else {
			len = 0;
	}
	if (copy_to_user(buf, page, (len < count ? len : count))) {
			return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static const struct proc_ops oem_sec_boot_proc_fops = {
	.proc_read     = oem_sec_boot_read_proc,
};

static int secure_register_proc_fs(struct secure_data *secure_data)
{
	struct proc_dir_entry *pentry;

	/*  make the dir /proc/oplus_secure_common  */
	nothing_secure_dir =  proc_mkdir(nothing_secure_dir_name, NULL);
	if(!nothing_secure_dir) {
			dev_err(secure_data->dev,"can't create nothing_secure_dir proc\n");
			return -1;
	}


	/*  make the proc /proc/nothing_secure/oem_sec_boot  */
	pentry = proc_create("oem_sec_boot", 0664, nothing_secure_dir, &oem_sec_boot_proc_fops);
	if(!pentry) {
			dev_err(secure_data->dev,"create oem_sec_boot proc failed.\n");
			return -1;
	}

	return 0;
}

static int nothing_secure_element_probe(struct platform_device *secure_dev)
{
	int ret = 0;
	struct device *dev = &secure_dev->dev;

	secure_data_ptr = devm_kzalloc(dev, sizeof(struct secure_data), GFP_KERNEL);
	if (secure_data_ptr == NULL) {
			dev_err(dev,"secure_data probe failed ret = %d\n", ret);
			dev_err(dev,"secure_data kzalloc failed\n");
			ret = -ENOMEM;
			goto exit;
	}

	secure_data_ptr->dev = dev;

	//add to get the parent dts nothing_secure_element
	ret = secure_element_parse_parent_dts(secure_data_ptr);
	if (ret) {
			goto exit;
	}

	ret = secure_register_proc_fs(secure_data_ptr);
	if (ret) {
			goto exit;
	}
	return 0;

exit:
	if (nothing_secure_dir) {
			remove_proc_entry(nothing_secure_dir_name, NULL);
	}

	dev_err(dev,"%s: probe failed ret = %d\n", __func__, ret);
	if (secure_data_ptr) {
			devm_kfree(dev, secure_data_ptr);
	}

	return 0;
}

static int nothing_secure_element_remove(struct platform_device *secure_dev)
{
	if (nothing_secure_dir) {
			remove_proc_entry(nothing_secure_dir_name, NULL);
	}

	if (secure_data_ptr) {
			devm_kfree(&secure_dev->dev, secure_data_ptr);
	}

	return 0;
}

static struct of_device_id nothing_secure_element_match_table[] = {
		{   .compatible = "nothing,secure_element", },
		{}
};

static struct platform_driver nothing_secure_element_driver = {
	.probe = nothing_secure_element_probe,
	.remove = nothing_secure_element_remove,
	.driver = {
			.name = "nothing_secure_element",
			.owner = THIS_MODULE,
			.of_match_table = nothing_secure_element_match_table,
	},
};

static int __init nothing_secure_element_init(void)
{
	return platform_driver_register(&nothing_secure_element_driver);
}

postcore_initcall(nothing_secure_element_init);

static void __exit nothing_secure_element_exit(void)
{
	platform_driver_unregister(&nothing_secure_element_driver);
}

module_exit(nothing_secure_element_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING secure element driver");
