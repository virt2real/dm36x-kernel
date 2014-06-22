/*
 *	Copyright (C) 2011 Ridgerun Engineering
 *	Author: Pablo Barrantes Ch. (pablo.barrantes@ridgerun.com)
 *
 * 	Heavily based on driver created by Shlomo Kut,,, (shl...@infodraw.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/mc146818rtc.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <mach/dm365_adc.h>

#define ADC_VERSION "1.0"

static spinlock_t adc_lock = SPIN_LOCK_UNLOCKED;
void *dm365_adc_base;

int adc_single(unsigned int channel)
{
	if (channel >= ADC_MAX_CHANNELS)
		return -1;

	/* Select channel */
	iowrite32(1 << channel,dm365_adc_base + DM365_ADC_CHSEL);

	/* Start coversion */
	iowrite32(DM365_ADC_ADCTL_BIT_START,dm365_adc_base +
			DM365_ADC_ADCTL);

	/* Wait for conversion to start */
	while (!(ioread32(dm365_adc_base + DM365_ADC_ADCTL) &
			DM365_ADC_ADCTL_BIT_BUSY)) {
		cpu_relax();
	}

	/* Wait for conversion to be complete. */
	while ((ioread32(dm365_adc_base + DM365_ADC_ADCTL) &
			DM365_ADC_ADCTL_BIT_BUSY)){
		cpu_relax();
	}

	return ioread32(dm365_adc_base + DM365_ADC_AD0DAT + 4 * channel);
}

static void adc_read_block(unsigned short *data, size_t length)
{
	int i;
	spin_lock_irq(&adc_lock);
	for(i = 0; i < length; i++) {
		data [i] = adc_single(i);
	}
	spin_unlock_irq(&adc_lock);
}

#ifndef CONFIG_PROC_FS
static int adc_add_proc_fs(void)
{
	return 0;
}

#else
static int adc_proc_read(struct seq_file *seq, void *offset)
{
	int i;
	unsigned short data [ADC_MAX_CHANNELS];

	adc_read_block(data,ADC_MAX_CHANNELS);

	for(i = 0; i < ADC_MAX_CHANNELS; i++) {
		seq_printf(seq, "0x%04X\n", data[i]);
	}

	return 0;
}

static int adc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, adc_proc_read, NULL);
}

static const struct file_operations adc_proc_fops = {
	.owner    = THIS_MODULE,
	.open   = adc_proc_open,
	.read   = seq_read,
	.release  = single_release,
};

static int adc_add_proc_fs(void)
{
	if (!proc_create("driver/dm365_adc", 0, NULL, &adc_proc_fops))
		return -ENOMEM;
	return 0;
}

#endif /* CONFIG_PROC_FS */

static ssize_t adc_read(struct file *file, char __user *buf,
						size_t count, loff_t *ppos)
{
	unsigned short data [ADC_MAX_CHANNELS];

	if (count < sizeof(unsigned short))
		return -ETOOSMALL;

	adc_read_block(data,ADC_MAX_CHANNELS);

	if (copy_to_user(buf, data, count))
		return -EFAULT;

	return count;
 }

static int adc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int adc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations adc_fops = {
	.owner	= THIS_MODULE,
	.read   = adc_read,
	.open   = adc_open,
	.release  = adc_release,
};

static struct miscdevice adc_dev = {
	.minor = NVRAM_MINOR,
	.name = "dm365_adc",
	.fops = &adc_fops
};

static int __init davinci_dm365_adc_probe(struct platform_device *pdev)
{
	struct davinci_dm365_adc_pdata	*pdata = pdev->dev.platform_data;
	struct dm365_adc		*adc;
	struct resource			*res;
	void __iomem			*base;
	int				ret;

	/* We need platform data to register the device */
	if (!pdata)
		return -ENODEV;

	adc = kzalloc(sizeof(*adc), GFP_KERNEL);
	if (!adc) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		ret = -ENOMEM;
		goto err_nomem;
	}

	platform_set_drvdata(pdev, adc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "resource missing\n");
		ret = -EINVAL;
		goto err_nomem;
	}

	/* Get the virtual address of the memory mapped registers */
	base = ioremap(res->start, res->end - res->start);
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	adc->dev		= &pdev->dev;
	adc->base		= base;
	dm365_adc_base	= base;

	/* Enable the ADC clock */
	adc->clk = clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)) {
		ret = PTR_ERR(adc->clk);
		dev_dbg(&pdev->dev, "unable to get ADC clock, err %d\n", ret);
		goto err_clk;
	}

	ret = clk_enable(adc->clk);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unable to enable ADC clock, err %d\n",
			ret);
		goto err_clk_enable;
	}

	/* Register the device */
	ret = misc_register(&adc_dev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register the misc device on minor=%d\n",
				NVRAM_MINOR);
		goto err_dev;
	}

	/* Add the procfs interface */
	ret = adc_add_proc_fs();
	if (ret) {
		dev_err(&pdev->dev, "Can't create /proc/driver/adc\n");
		goto err_reg;
	}

	/* Configure the ADC for one shot conversions */
	if(!pdata->mode)
		iowrite32(ioread32(dm365_adc_base + DM365_ADC_ADCTL) | ONE_SHOT_MODE,
					dm365_adc_base + DM365_ADC_ADCTL);
	else {
		/* TODO: Support free run mode. We don't even write the register here */
		dev_err(&pdev->dev, "Mode not supported, unexpected results may happen\n");
		goto err_mode;
	}

	printk("dm365_adc: DM365 ADC registered succesfully\n");
	return 0;

err_mode:
err_reg:
	misc_deregister(&adc_dev);
err_dev:
err_clk_enable:
	clk_put(adc->clk);
err_clk:
err_ioremap:
	if (base)
		iounmap(base);
err_nomem:
	kfree(adc);
	return ret;
}

static int __exit davinci_dm365_adc_remove(struct platform_device *pdev)
{
	int ret=0;

	remove_proc_entry("driver/dm365_adc", NULL);

	ret = misc_deregister(&adc_dev);
	if(ret) {
		dev_err(&pdev->dev, "Unable to remove ADC procfs entry, error: %d", ret);
	}

	return ret;
}

static struct platform_driver davinci_dm365_adc_driver = {
	.remove		= __exit_p(davinci_dm365_adc_remove),
	.driver		= {
		.name	= "davinci_dm365_adc",
	},
};

static int __init dm365_adc_init(void)
{
	return platform_driver_probe(&davinci_dm365_adc_driver, davinci_dm365_adc_probe);
}
module_init(dm365_adc_init);

static void __exit dm365_adc_exit(void)
{
	platform_driver_unregister(&davinci_dm365_adc_driver);
}
module_exit(dm365_adc_exit);

MODULE_DESCRIPTION("TI Davinci Dm365 ADC");
MODULE_AUTHOR("Pablo Barrantes - pablo.barrantes@ridgerun.com");
MODULE_LICENSE("GPL");
