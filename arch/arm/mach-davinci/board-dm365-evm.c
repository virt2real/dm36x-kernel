/*
 * TI DaVinci DM365 EVM board support
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/mux.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <mach/cputype.h>
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>

#define BITBANG_I2C

#ifdef BITBANG_I2C
#include <linux/i2c-gpio.h>
#endif

static inline int have_tvp7002(void)
{
#ifdef CONFIG_VIDEO_TVP7002
	return 1;
#else
	return 0;
#endif
}

#define DM365_ASYNC_EMIF_CONTROL_BASE	0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE	0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE	0x04000000

#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

static struct snd_platform_data dm365_evm_snd_data = {
	.eventq_no = EVENTQ_3,
};

static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO("ths7303", 0x2c),
	},
};

#ifdef BITBANG_I2C
static struct i2c_gpio_platform_data i2c_pdata = {
  .sda_pin = 21,
  .scl_pin = 20,
  .udelay = 2, /* ~100 kHz */
};
static struct platform_device bitbanged_i2c_device = {
  .name = "i2c-gpio",
  .id = 1,
  .dev.platform_data = &i2c_pdata,
};
#else
static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
};
#endif

static int cpld_mmc_get_ro(int module)
{
        return -ENXIO;
}

static struct gpio_led gpio_leds[] = {
  {
    .name = "leopardboard::debug0",
    .default_trigger = "heartbeat",
    .gpio = 57,
  },
  {
    .name = "leopardboard::debug1",
    .default_trigger = "mmc0",
    .gpio = 58,
  },
};

static struct gpio_led_platform_data gpio_led_info = {
  .leds = gpio_leds,
  .num_leds = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
  .name = "leds-gpio",
  .id = -1,
  .dev = {
    .platform_data = &gpio_led_info,
  },
};

/* Input available at the camera */
static struct v4l2_input camera_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_720P_30,
	}
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
        /* cam board */
		.module_name = "mt9p031",
		.is_camera = 1, /* See vpfe_capture.c's interface setting. 
				   Be sure to pass vpfe_capture.interface=1
				   to kernel command line parameters */
		.grp_id = VPFE_SUBDEV_MT9P031,
		.num_inputs = ARRAY_SIZE(camera_inputs),
		.inputs = camera_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_RAW_BAYER,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("mt9p031", 0x48),
			/* this is for PCLK rising edge */
			.platform_data = (void *)1,
		},
	},
};

/* Set the input mux for TVP7002/TVP5146/MTxxxx sensors */
static int dm365evm_setup_video_input(enum vpfe_subdev_id id)
{
  /* Pull the camera out of reset */
 gpio_request(31, "sensor_reset");
 gpio_direction_output(31, 1);

	return 0;
}

static struct vpfe_config vpfe_cfg = {
       .setup_input = dm365evm_setup_video_input,
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM365 EVM",
       .ccdc = "DM365 ISIF",
       .num_clocks = 1,
       .clocks = {"vpss_master"},
};

static struct davinci_mmc_config dm365evm_mmc_config = {
	.get_ro		= cpld_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void dm365evm_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm365evm_usb_configure(void)
{
	gpio_request(66, "usb");
	gpio_direction_output(66, 0);
	setup_usb(500, 8);
}

static void __init evm_init_i2c(void)
{
#ifdef BITBANG_I2C
  davinci_cfg_reg(DM365_GPIO21);
  davinci_cfg_reg(DM365_GPIO20);

  (void) platform_device_register(&bitbanged_i2c_device);
#else
	davinci_init_i2c(&i2c_pdata);
#endif
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
}

void enable_lcd(void)
{
}
EXPORT_SYMBOL(enable_lcd);

void enable_hd_clk(void)
{
}
EXPORT_SYMBOL(enable_hd_clk);

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init dm365_evm_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init();
}

static __init void dm365_evm_init(void)
{
	evm_init_i2c();
	davinci_serial_init(&uart_config);

	dm365evm_emac_configure();
	dm365evm_usb_configure();

	davinci_setup_mmc(0, &dm365evm_mmc_config);

	dm365_init_asp(&dm365_evm_snd_data);

	(void) platform_device_register(&leds_gpio);
}

static __init void dm365_evm_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DAVINCI_DM365_EVM, "DaVinci DM36x EVM")
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (0x80000100),
	.map_io		= dm365_evm_map_io,
	.init_irq	= dm365_evm_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_evm_init,
MACHINE_END

