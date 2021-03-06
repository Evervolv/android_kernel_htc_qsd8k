/* linux/arch/arm/mach-msm/board-incrediblec-wifi.c
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
#include <linux/wlan_plat.h>

#include "board-incrediblec.h"

int incrediblec_wifi_power(int on);
int incrediblec_wifi_reset(int on);
int incrediblec_wifi_set_carddetect(int on);

#if defined(CONFIG_DHD_USE_STATIC_BUF) || defined(CONFIG_BCM4329_DHD_USE_STATIC_BUF)

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

static void *incrediblec_wifi_mem_prealloc(int section, unsigned long size)
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

int __init incrediblec_init_wifi_mem(void)
{
#if defined(CONFIG_DHD_USE_STATIC_BUF) || defined(CONFIG_BCM4329_DHD_USE_STATIC_BUF)
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

static struct resource incrediblec_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= MSM_GPIO_TO_INT(INCREDIBLEC_GPIO_WIFI_IRQ),
		.end		= MSM_GPIO_TO_INT(INCREDIBLEC_GPIO_WIFI_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct wifi_platform_data incrediblec_wifi_control = {
	.set_power      = incrediblec_wifi_power,
	.set_reset      = incrediblec_wifi_reset,
	.set_carddetect = incrediblec_wifi_set_carddetect,
#if defined(CONFIG_DHD_USE_STATIC_BUF) || defined(CONFIG_BCM4329_DHD_USE_STATIC_BUF)
	.mem_prealloc   = incrediblec_wifi_mem_prealloc,
#else
	.mem_prealloc   = NULL,
#endif
};

static struct platform_device incrediblec_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(incrediblec_wifi_resources),
        .resource       = incrediblec_wifi_resources,
        .dev            = {
                .platform_data = &incrediblec_wifi_control,
        },
};

extern unsigned char *get_wifi_nvs_ram(void);
extern int wifi_calibration_size_set(void);

static unsigned incrediblec_wifi_update_nvs(char *str, int add_flag)
{
#define NVS_LEN_OFFSET		0x0C
#define NVS_DATA_OFFSET		0x40
	unsigned char *ptr;
	unsigned len;

	if (!str)
		return -EINVAL;
	ptr = get_wifi_nvs_ram();
	/* Size in format LE assumed */
	memcpy(&len, ptr + NVS_LEN_OFFSET, sizeof(len));
	/* if the last byte in NVRAM is 0, trim it */
	if (ptr[NVS_DATA_OFFSET + len - 1] == 0)
		len -= 1;
	if (add_flag) {
		strcpy(ptr + NVS_DATA_OFFSET + len, str);
		len += strlen(str);
	} else {
		if (strnstr(ptr + NVS_DATA_OFFSET, str, len))
			len -= strlen(str);
	}
	memcpy(ptr + NVS_LEN_OFFSET, &len, sizeof(len));
	wifi_calibration_size_set();
	return 0;
}

static int __init incrediblec_wifi_init(void)
{
	if (!machine_is_incrediblec())
		return 0;

	printk("%s: start\n", __func__);
	incrediblec_wifi_update_nvs("sd_oobonly=1\r\n", 0);
	incrediblec_wifi_update_nvs("btc_params70=0x32\r\n", 1);
	incrediblec_init_wifi_mem();
	return platform_device_register(&incrediblec_wifi_device);
}

late_initcall(incrediblec_wifi_init);
