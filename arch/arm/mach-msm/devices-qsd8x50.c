/*
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>

#include "devices.h"
#include "proc_comm.h"

#include <asm/mach/flash.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/mmc.h>

#include <mach/msm_hsusb.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb_hw.h>
#ifdef CONFIG_PMIC8058
#include <linux/mfd/pmic8058.h>
#endif

int usb_phy_error;

#define HSUSB_API_INIT_PHY_PROC 2
#define HSUSB_API_PROG          0x30000064
#define HSUSB_API_VERS MSM_RPC_VERS(1, 1)

static void *usb_base;
#define MSM_USB_BASE              ((unsigned)usb_base)
static unsigned ulpi_read(void __iomem *usb_base, unsigned reg)
{
        unsigned timeout = 100000;

        /* initiate read operation */
        writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
               USB_ULPI_VIEWPORT);

        /* wait for completion */
        while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
                cpu_relax();

        if (timeout == 0) {
                printk(KERN_ERR "ulpi_read: timeout %08x\n",
                        readl(USB_ULPI_VIEWPORT));
                return 0xffffffff;
        }
        return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(void __iomem *usb_base, unsigned val, unsigned reg)
{
        unsigned timeout = 10000;

        /* initiate write operation */
        writel(ULPI_RUN | ULPI_WRITE |
               ULPI_ADDR(reg) | ULPI_DATA(val),
               USB_ULPI_VIEWPORT);

        /* wait for completion */
        while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
                cpu_relax();

        if (timeout == 0) {
                printk(KERN_ERR "ulpi_write: timeout\n");
                return -1;
        }

        return 0;
}

#define CLKRGM_APPS_RESET_USBH      37
#define CLKRGM_APPS_RESET_USB_PHY   34
static void msm_hsusb_apps_reset_link(int reset)
{
        int ret;
        unsigned usb_id = CLKRGM_APPS_RESET_USBH;

        if (reset)
                ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
                                &usb_id, NULL);
        else
                ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
                                &usb_id, NULL);
        if (ret)
                printk(KERN_INFO "%s: Cannot set reset to %d (%d)\n",
                        __func__, reset, ret);
}

static void msm_hsusb_apps_reset_phy(void)
{
        int ret;
        unsigned usb_phy_id = CLKRGM_APPS_RESET_USB_PHY;

        ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
                        &usb_phy_id, NULL);
        if (ret) {
                printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
                return;
        }
        msleep(1);
        ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
                        &usb_phy_id, NULL);
        if (ret) {
                printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
                return;
        }
}

#define ULPI_VERIFY_MAX_LOOP_COUNT  3
static int msm_hsusb_phy_verify_access(void __iomem *usb_base)
{
        int temp;

        for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
                if (ulpi_read(usb_base, ULPI_DEBUG) != (unsigned)-1)
                        break;
                msm_hsusb_apps_reset_phy();
        }

        if (temp == ULPI_VERIFY_MAX_LOOP_COUNT) {
                pr_err("%s: ulpi read failed for %d times\n",
                                __func__, ULPI_VERIFY_MAX_LOOP_COUNT);
                return -1;
        }

        return 0;
}

static unsigned msm_hsusb_ulpi_read_with_reset(void __iomem *usb_base, unsigned reg)
{
        int temp;
        unsigned res;

        for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
                res = ulpi_read(usb_base, reg);
                if (res != -1)
                        return res;
                msm_hsusb_apps_reset_phy();
        }

        pr_err("%s: ulpi read failed for %d times\n",
                        __func__, ULPI_VERIFY_MAX_LOOP_COUNT);

        return -1;
}

static int msm_hsusb_ulpi_write_with_reset(void __iomem *usb_base,
                unsigned val, unsigned reg)
{
        int temp;
        int res;

        for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
                res = ulpi_write(usb_base, val, reg);
                if (!res)
                        return 0;
                msm_hsusb_apps_reset_phy();
        }

        pr_err("%s: ulpi write failed for %d times\n",
                        __func__, ULPI_VERIFY_MAX_LOOP_COUNT);
        return -1;
}

static int msm_hsusb_phy_caliberate(void __iomem *usb_base)
{
        int ret;
        unsigned res;

        ret = msm_hsusb_phy_verify_access(usb_base);
        if (ret)
                return -ETIMEDOUT;

        res = msm_hsusb_ulpi_read_with_reset(usb_base, ULPI_FUNC_CTRL_CLR);
        if (res == -1)
                return -ETIMEDOUT;

        res = msm_hsusb_ulpi_write_with_reset(usb_base,
                        res | ULPI_SUSPENDM,
                        ULPI_FUNC_CTRL_CLR);
        if (res)
                return -ETIMEDOUT;

        msm_hsusb_apps_reset_phy();

        return msm_hsusb_phy_verify_access(usb_base);
}

#define USB_LINK_RESET_TIMEOUT      (msecs_to_jiffies(10))
void msm_hsusb_8x50_phy_reset(void)
{
        u32 temp;
        unsigned long timeout;
        printk(KERN_INFO "msm_hsusb_phy_reset\n");
        usb_base = ioremap(MSM_HSUSB_PHYS, 4096);

        msm_hsusb_apps_reset_link(1);
        msm_hsusb_apps_reset_phy();
        msm_hsusb_apps_reset_link(0);

        /* select ULPI phy */
        temp = (readl(USB_PORTSC) & ~PORTSC_PTS_MASK);
        writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

        if (msm_hsusb_phy_caliberate(usb_base)) {
                usb_phy_error = 1;
                return;
        }

        /* soft reset phy */
        writel(USBCMD_RESET, USB_USBCMD);
        timeout = jiffies + USB_LINK_RESET_TIMEOUT;
        while (readl(USB_USBCMD) & USBCMD_RESET) {
                if (time_after(jiffies, timeout)) {
                        pr_err("usb link reset timeout\n");
                        break;
                }
                msleep(1);
        }
        usb_phy_error = 0;

        return;
}

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart3[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_uart3 = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(resources_uart3),
	.resource	= resources_uart3,
};

struct platform_device msm_device_smd = {
	.name   = "msm_smd",
	.id     = -1,
};

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART1DM_RX,
		.end   = INT_UART1DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart2_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART2DM_RX,
		.end   = INT_UART2DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm2_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm2 = {
	.name = "msm_serial_hs",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart2_dm_resources),
	.resource = msm_uart2_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#define GPIO_I2C_CLK 95
#define GPIO_I2C_DAT 96
static int gpio_i2c_clk = -1;
static int gpio_i2c_dat = -1;
void msm_set_i2c_mux(bool gpio, int *gpio_clk, int *gpio_dat)
{
	unsigned id;

	if (gpio_i2c_clk < 0) {
		gpio_request(GPIO_I2C_CLK, "i2c-clk");
		gpio_i2c_clk = GPIO_I2C_CLK;
	}
	if (gpio_i2c_dat < 0) {
		gpio_request(GPIO_I2C_DAT, "i2c-dat");
		gpio_i2c_dat = GPIO_I2C_DAT;
	}

	if (gpio) {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		*gpio_clk = GPIO_I2C_CLK;
		*gpio_dat = GPIO_I2C_DAT;
	} else {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

struct flash_platform_data msm_nand_data = {
	.parts		= NULL,
	.nr_parts	= 0,
};

static struct resource resources_nand[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_PHYS,
		.end	= MSM_SDC1_PHYS + MSM_SDC1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
	{
		.start	= INT_SDC1_1,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_PHYS,
		.end	= MSM_SDC2_PHYS + MSM_SDC2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC2_1,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_PHYS,
		.end	= MSM_SDC3_PHYS + MSM_SDC3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC3_1,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_PHYS,
		.end	= MSM_SDC4_PHYS + MSM_SDC4_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC4_1,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc(unsigned int controller, struct msm_mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags)
{
	struct platform_device	*pdev;
	struct resource *res;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "status_irq");
	if (!res)
		return -EINVAL;
	else if (stat_irq) {
		res->start = res->end = stat_irq;
		res->flags &= ~IORESOURCE_DISABLED;
		res->flags |= stat_irq_flags;
	}

	return platform_device_register(pdev);
}

static struct resource resources_mddi0[] = {
	{
		.start	= MSM_PMDH_PHYS,
		.end	= MSM_PMDH_PHYS + MSM_PMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_PRI,
		.end	= INT_MDDI_PRI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_mddi1[] = {
	{
		.start	= MSM_EMDH_PHYS,
		.end	= MSM_EMDH_PHYS + MSM_EMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_EXT,
		.end	= INT_MDDI_EXT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mddi0 = {
	.name = "msm_mddi",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mddi0),
	.resource = resources_mddi0,
	.dev = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

struct platform_device msm_device_mddi1 = {
	.name = "msm_mddi",
	.id = 1,
	.num_resources = ARRAY_SIZE(resources_mddi1),
	.resource = resources_mddi1,
	.dev = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

static struct resource resources_mdp[] = {
	{
		.start	= MSM_MDP_PHYS,
		.end	= MSM_MDP_PHYS + MSM_MDP_SIZE - 1,
		.name	= "mdp",
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= INT_MDP,
		.end	= INT_MDP,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mdp = {
	.name = "msm_mdp",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mdp),
	.resource = resources_mdp,
};

static struct resource resources_tssc[] = {
	{
		.start	= MSM_TSSC_PHYS,
		.end	= MSM_TSSC_PHYS + MSM_TSSC_SIZE - 1,
		.name	= "tssc",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_TCHSCRN1,
		.end	= INT_TCHSCRN1,
		.name	= "tssc1",
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
	{
		.start	= INT_TCHSCRN2,
		.end	= INT_TCHSCRN2,
		.name	= "tssc2",
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

struct platform_device msm_device_touchscreen = {
	.name = "msm_touchscreen",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_tssc),
	.resource = resources_tssc,
};

#if defined(CONFIG_ARCH_QSD8X50)
static struct resource resources_spi[] = {
	{
		.name   = "spi_base",
		.start  = MSM_SPI_PHYS,
		.end    = MSM_SPI_PHYS + MSM_SPI_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_irq_in",
		.start  = INT_SPI_INPUT,
		.end    = INT_SPI_INPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start  = INT_SPI_OUTPUT,
		.end    = INT_SPI_OUTPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start  = INT_SPI_ERROR,
		.end    = INT_SPI_ERROR,
		.flags  = IORESOURCE_IRQ,
	},
#if defined(CONFIG_SPI_QSD)
	{
		.name   = "spi_clk",
		.start  = 17,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_mosi",
		.start  = 18,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_miso",
		.start  = 19,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_cs0",
		.start  = 20,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_pwr",
		.start  = 21,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_cs0",
		.start  = 22,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
#endif
};

struct platform_device msm_device_spi = {
#if defined(CONFIG_SPI_QSD)
	.name		= "spi_qsd",
#else
	.name		= "msm_spi",
#endif
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_spi),
	.resource	= resources_spi,
};
#endif

#define CLK_ALL(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_ALL)
#define CLK_7X00A(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_MSM7X00A)
#define CLK_8X50(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_QSD8X50)

#define OFF CLKFLAG_AUTO_OFF
#define MINMAX (CLKFLAG_USE_MIN_TO_SET | CLKFLAG_USE_MAX_TO_SET)
#define USE_MIN (CLKFLAG_USE_MIN_TO_SET | CLKFLAG_SHARED)

static struct resource resources_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static u64 dma_mask = 0xffffffffULL;
static struct resource resources_hsusb_host[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_host = {
	.name		= "msm_hsusb_host",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb_host),
	.resource	= resources_hsusb_host,
	.dev		= {
		.dma_mask               = &dma_mask,
		.coherent_dma_mask      = 0xffffffffULL,
	},
};

struct clk msm_clocks_8x50[] = {
	CLK_PCOM("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_PCOM("ebi1_clk",	EBI1_CLK,	NULL, CLK_MIN | CLKFLAG_SHARED),
	CLK_PCOM("ebi2_clk",	EBI2_CLK,	NULL, 0),
	CLK_PCOM("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_PCOM("mddi_clk",	EMDH_CLK,	&msm_device_mddi1.dev, OFF | CLK_MINMAX),
	CLK_PCOM("gp_clk",	GP_CLK,		NULL, 0),
	CLK_PCOM("grp_clk",	GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("i2c_clk",	I2C_CLK,	&msm_device_i2c.dev, 0),
	CLK_PCOM("icodec_rx_clk",	ICODEC_RX_CLK,	NULL, 0),
	CLK_PCOM("icodec_tx_clk",	ICODEC_TX_CLK,	NULL, 0),
	CLK_PCOM("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_PCOM("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_PCOM("mddi_clk",	PMDH_CLK,	&msm_device_mddi0.dev, OFF | CLK_MINMAX),
	CLK_PCOM("mdp_clk",	MDP_CLK,	&msm_device_mdp.dev, OFF),
	CLK_PCOM("lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, &msm_device_mdp.dev, 0),
	CLK_PCOM("lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, &msm_device_mdp.dev, 0),
	CLK_PCOM("mdp_vsync_clk",	MDP_VSYNC_CLK,	&msm_device_mdp.dev, 0),
	CLK_PCOM("pbus_clk",	PBUS_CLK,	NULL, CLK_MIN),
	CLK_PCOM("pcm_clk",	PCM_CLK,	NULL, 0),
	CLK_PCOM("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_PCOM("spi_clk",	SPI_CLK,	&msm_device_spi.dev, 0),
	CLK_PCOM("sdc_clk",	SDC1_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC1_P_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC2_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC2_P_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC3_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC3_P_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC4_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC4_P_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_PCOM("tsif_clk",	TSIF_CLK,	NULL, 0),
	CLK_PCOM("tsif_ref_clk",	TSIF_REF_CLK,	NULL, 0),
	CLK_PCOM("tv_dac_clk",	TV_DAC_CLK,	NULL, 0),
	CLK_PCOM("tv_enc_clk",	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("uart_clk",	UART1_CLK,	&msm_device_uart1.dev, OFF),
	CLK_PCOM("uart_clk",	UART2_CLK,	&msm_device_uart2.dev, OFF),
	CLK_PCOM("uart_clk",	UART3_CLK,	&msm_device_uart3.dev, OFF),
	CLK_PCOM("uartdm_clk",	UART1DM_CLK,	&msm_device_uart_dm1.dev, OFF),
	CLK_PCOM("uartdm_clk",	UART2DM_CLK,	&msm_device_uart_dm2.dev, OFF),
	CLK_PCOM("usb_hs_clk",	USB_HS_CLK,	&msm_device_hsusb.dev, OFF),
	CLK_PCOM("usb_hs_pclk",	USB_HS_P_CLK,	&msm_device_hsusb.dev, OFF),
	CLK_PCOM("usb_otg_clk",	USB_OTG_CLK,	NULL, 0),
	CLK_PCOM("vdc_clk",	VDC_CLK,	NULL, OFF | CLK_MIN),
	CLK_PCOM("vfe_clk",	VFE_CLK,	NULL, OFF),
	CLK_PCOM("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, OFF),
	CLK_PCOM("vfe_axi_clk",	VFE_AXI_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs2_clk",	USB_HS2_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs2_pclk",	USB_HS2_P_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs3_clk",	USB_HS3_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs3_pclk",	USB_HS3_P_CLK,	NULL, OFF),
	CLK_PCOM("usb_phy_clk",	USB_PHY_CLK,	NULL, 0),

};

void msm_i2c_gpio_init(void)
{
        gpio_request(GPIO_I2C_CLK, "i2c_clk");
        gpio_request(GPIO_I2C_DAT, "i2c_data");
}

unsigned msm_num_clocks_8x50 = ARRAY_SIZE(msm_clocks_8x50);

