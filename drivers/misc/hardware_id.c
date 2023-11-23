#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/pm.h>

#include <linux/err.h>
#include <linux/input.h>
#include <linux/jiffies.h>

#include <linux/of_gpio.h>

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>


/*HWID | GPIO87 | GPIO88*/
/* PVT  |  0          |  0        */
/* T0     |  0          |  1        */
/* EVT   |  0          |  2        */
/* DVT   |  1          |  0        */
/* DVT2   |  1          |  1        */

/*VERSION_ID0: GPIO87*/
/*VERSION_ID1: GPIO88*/

static int gp87_val_up = 2;
static int gp88_val_up = 2;
static int gp87_val_down = 2;
static int gp88_val_down = 2;

static char *hw_ver[]     = {"PVT","T0","EVT","DVT","DVT2","NA"};
static int ver_gpioflag = 5;

static int hwid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "version = %s  gp87_val_up=%d  gp88_val_up=%d  gp87_val_down=%d  gp88_val_down=%d\n", hw_ver[ver_gpioflag], gp87_val_up, gp88_val_up, gp87_val_down, gp88_val_down);
	return 0;
}

static int hwid_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwid_show, inode->i_private);
}

static const struct proc_ops hwid_proc_ops = {
	.proc_open		= hwid_open,
	.proc_read		= seq_read,
};

static int create_hwid_proc_file(void)
{
	struct proc_dir_entry *dir;

	dir = proc_create("hwid", 0440, NULL, &hwid_proc_ops);
	if (!dir) {
		pr_err("Unable to create /proc/hwid");
		return -1;
	}

	return 0;
}


static int hwid_probe(struct platform_device *pdev)
{
	int ret;
	int ver_id0;			/*VERSION_ID0*/
	int ver_id1;			/*VERSION_ID1*/
	int id87_val_up;		/*the value of gpio87 when pull up*/
	int id88_val_up;		/*the value of gpio88 when pull up*/
	int id87_val_down;	/*the value of gpio87 when pull down*/
	int id88_val_down;	/*the value of gpio88 when pull down*/
	struct pinctrl *hwid_pinctrl;
	struct pinctrl_state *pinctrl_state;

	enum of_gpio_flags flags ;

	printk(KERN_EMERG "enter hwid_probe\n");

	ver_id0 = of_get_named_gpio_flags(pdev->dev.of_node, "ver_id0", 0, &flags);
	if (gpio_is_valid(ver_id0)) {
           ret = gpio_request(ver_id0, "ver_id0");
           if (ret) {
               printk(KERN_ERR"Failed to request gpio");
               return - EINVAL;
           }
	}
	ret = gpio_direction_input(ver_id0);
	if (ret)
	{
	    printk(KERN_ERR"set ver_id0 failed\n");
	}

	ver_id1 = of_get_named_gpio_flags(pdev->dev.of_node, "ver_id1", 0, &flags);
	if (gpio_is_valid(ver_id1)) {
           ret = gpio_request(ver_id1, "ver_id1");
           if (ret) {
               printk(KERN_ERR"Failed to request gpio");
               return - EINVAL;
           }
	}
	ret = gpio_direction_input(ver_id1);
	if (ret)
	{
	    printk(KERN_ERR"set ver_id1 failed\n");
	}

       hwid_pinctrl = devm_pinctrl_get(&pdev->dev);
       if (IS_ERR(hwid_pinctrl)) {
	    printk(KERN_ERR"Failed to get pinctrl, please check dts");
	    return - ENODEV;
       }
       pinctrl_state = pinctrl_lookup_state(hwid_pinctrl, "hardware_up");
       if (IS_ERR(pinctrl_state)) {
	    printk(KERN_ERR"Failed to get pinctrl state");
	    return - EINVAL;
       }
	ret = pinctrl_select_state(hwid_pinctrl, pinctrl_state);
	if (!ret){
	    printk(KERN_INFO"get hardware_up pinctrl state:%d \n", ret);
	} else {
	    printk(KERN_ERR"Failed to select pinctrl hardware_up state \n");
	    return - EINVAL;
	}
	id87_val_up = gpio_get_value(ver_id0);
	id88_val_up = gpio_get_value(ver_id1);

	pinctrl_state = pinctrl_lookup_state(hwid_pinctrl, "hardware_down");
	if (IS_ERR(pinctrl_state)) {
	    printk(KERN_ERR"Failed to get pinctrl state");
	    return - EINVAL;
	}
	ret = pinctrl_select_state(hwid_pinctrl, pinctrl_state);
	if (!ret){
	    printk(KERN_INFO"get hardware_down pinctrl state\n");
	} else {
	    printk(KERN_ERR"Failed to select pinctrl hardware_down state \n");
	    return - EINVAL;
	}
	id87_val_down = gpio_get_value(ver_id0);
	id88_val_down = gpio_get_value(ver_id1);

	gp87_val_up = id87_val_up;
	gp88_val_up = id88_val_up;
	gp87_val_down = id87_val_down;
	gp88_val_down = id88_val_down;

	if (!(id87_val_up || id87_val_down)) {
	    ver_id0 = 1;
	} else if (id87_val_up && id87_val_down) {
	    ver_id0 = 2;
	} else {
	    ver_id0 = 0;
	}

	if (!(id88_val_up || id88_val_down)) {
	    ver_id1 = 1;
	} else if (id88_val_up && id88_val_down) {
	    ver_id1 = 2;
	} else {
	    ver_id1 = 0;
	}
        ver_gpioflag = ver_id0*3 + ver_id1;
	if (ver_gpioflag > 5 ) {
	    ver_gpioflag = 5;
	    printk(KERN_INFO"abnormal results. ver_id0=%d  ver_id1=:%d\n", ver_id0, ver_id1);
	}

	printk(KERN_INFO"%s ver_gpioflag is %d\n", __func__, ver_gpioflag);

	/* create proc for hardware id */
	create_hwid_proc_file();

	pinctrl_state = pinctrl_lookup_state(hwid_pinctrl, "hardware_lpm");
	if (pinctrl_state) {
		ret = pinctrl_select_state(hwid_pinctrl, pinctrl_state);
		if (ret < 0)
			pr_err("HWID: Failed to set lpm pinctrl: %d\n", ret);
		else
			pr_info("HWID: Set lpm state done\n");
	}

        return 0;
}

static const struct of_device_id hwid_of_match[] = {
	{ .compatible = "nt2,hwid", },
	{ },
};

MODULE_DEVICE_TABLE(of, hwid_of_match);
static struct platform_driver hwid_driver = {
	.probe		= hwid_probe,
	.driver		= {
		.name	= "nt2-hwid",
		.of_match_table = hwid_of_match,
	}
};

static int __init hwid_init(void)
{
	printk(KERN_EMERG"%s enter.....\n", __func__);
	return platform_driver_register(&hwid_driver);
}

static void __exit hwid_exit(void)
{
	printk(KERN_EMERG"%s exit.....\n", __func__);
	remove_proc_entry("hwid", NULL);
	platform_driver_unregister(&hwid_driver);
}
module_exit(hwid_exit);
module_init(hwid_init);
MODULE_DESCRIPTION("hardware id function");
MODULE_LICENSE("GPL v2");
