/* linux/arch/arm/mach-msm/board-htcleo-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wifi_tiwlan.h>

#include "board-htcleo.h"

int htcleo_wifi_power(int on);
int htcleo_wifi_reset(int on);
int htcleo_wifi_set_carddetect(int on);

#if (!defined(CONFIG_BCMDHD) || defined(CONFIG_DHD_USE_STATIC_BUF))

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4

#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *htcleo_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}
#endif

int __init htcleo_init_wifi_mem(void);

int __init htcleo_init_wifi_mem(void)
{
#if (!defined(CONFIG_BCMDHD) || defined(CONFIG_DHD_USE_STATIC_BUF))
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
#endif
	return 0;
}

static struct resource htcleo_wifi_resources[] = {
	[0] = {
#if !defined(CONFIG_BCMDHD)
		.name		= "bcm4329_wlan_irq",
#else
		.name		= "bcmdhd_wlan_irq",
#endif
		.start		= MSM_GPIO_TO_INT(HTCLEO_GPIO_WIFI_IRQ),
		.end		= MSM_GPIO_TO_INT(HTCLEO_GPIO_WIFI_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct wifi_platform_data htcleo_wifi_control = {
	.set_power      = htcleo_wifi_power,
	.set_reset      = htcleo_wifi_reset,
	.set_carddetect = htcleo_wifi_set_carddetect,
#if (!defined(CONFIG_BCMDHD) || defined(CONFIG_DHD_USE_STATIC_BUF))
	.mem_prealloc   = htcleo_wifi_mem_prealloc,
#else
	.mem_prealloc   = NULL,
#endif
};

static struct platform_device htcleo_wifi_device = {
#if !defined(CONFIG_BCMDHD)
		.name			= "bcm4329_wlan",
#else
        .name           = "bcmdhd_wlan",
#endif
        .id             = 1,
        .num_resources  = ARRAY_SIZE(htcleo_wifi_resources),
        .resource       = htcleo_wifi_resources,
        .dev            = {
                .platform_data = &htcleo_wifi_control,
        },
};


static int __init htcleo_wifi_init(void)
{
	if (!machine_is_htcleo())
		return 0;

	printk("%s: start\n", __func__);
	//	htcleo_wifi_update_nvs("sd_oobonly=1\n");
	htcleo_init_wifi_mem();
        return platform_device_register(&htcleo_wifi_device);;
}

device_initcall(htcleo_wifi_init);
