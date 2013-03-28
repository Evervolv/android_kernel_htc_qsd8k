/* arch/arm/mach-msm/dex_comm.c
 *
 * Author: maejrep
 *	   Markinus
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
 * Based on proc_comm.c by Brian Swetland
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>

#include "dex_comm.h"


// from board-htcleo-power.c
void notify_vbus_change_intr(void);

#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)

static unsigned base = 0;

static inline void notify_other_dex_comm(void)
{
	uint8_t dex_irq;
	if(machine_is_htcleo())
		dex_irq = 4;
	else
		dex_irq = 6;
	writel(1, MSM_A2M_INT(dex_irq));
}

#define PC_DEBUG 1

#define PC_COMMAND      0x00
#define PC_STATUS       0x04
#define PC_SERIAL       0x08
#define PC_SERIAL_CHECK 0x0C
#define PC_DATA         0x20
#define PC_DATA_RESULT  0x24
#define PC_NOTIFY       0x28
#define PC_DATA2        0x30
#define PC_DATA_RESULT2 0x34
#define PC_READY        0x3c

#if (PC_DEBUG > 0)
 #define DDEX(fmt, arg...) printk(KERN_DEBUG "[DEX] %s: " fmt "\n", __FUNCTION__, ## arg)
#else
 #define DDEX(fmt, arg...) do {} while (0)
#endif


static DEFINE_SPINLOCK(dex_comm_lock);

#define TIMEOUT (10000000) /* 10s in microseconds */

int dex_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{
	unsigned long flags;
	unsigned timeout;
	unsigned status;
	unsigned num, dex_has_data, dex_cmd=0, dex_data=0, dex_data2=0;
	unsigned base_cmd, base_status;
	
	dex_has_data = data1 ? 1 : data2 ? 1 : 0;
	dex_data = data1 ? *data1 : 0;
	dex_data2 = data2 ? *data2 : 0;
	dex_cmd = cmd;

	spin_lock_irqsave(&dex_comm_lock, flags);

	DDEX("waiting for modem; command=0x%02x data=0x%x data2=0x%x has_data=0x%x", dex_cmd, dex_data, dex_data2, dex_has_data);
	DDEX("Check DEX Status: %d base adress: 0x%x\n", readl(base + PC_READY), base);

	// Store original cmd byte
	base_cmd = dex_cmd & 0xff;

	// Write only lowest byte
	writeb(base_cmd, base + PC_COMMAND);

	// If we have data to pass, add 0x100 bit and store the data
	if ( dex_has_data )
	{
		writel(readl(base + PC_COMMAND) | DEX_HAS_DATA, base + PC_COMMAND);
		writel(dex_data, base + PC_DATA);
		writel(dex_data2, base + PC_DATA2);
	} else {
		writel(readl(base + PC_COMMAND) & ~DEX_HAS_DATA, base + PC_COMMAND);
		writel(0, base + PC_DATA);
		writel(0, base + PC_DATA2);
	}
	
	// Increment last serial counter
	num = readl(base + PC_SERIAL) + 1;
	writel(num, base + PC_SERIAL);

	DDEX("command and data sent (cntr=0x%x) ...", num);

	// Notify ARM9 with int6
	notify_other_dex_comm();

	// Wait for response...  XXX: check irq stat?
	timeout = TIMEOUT;
	while ( --timeout && readl(base + PC_SERIAL_CHECK) != num )
		udelay(1);

	if ( ! timeout )
	{
		printk(KERN_WARNING "%s: DEX cmd timed out. status=0x%x, A2Mcntr=%x, M2Acntr=%x\n", 
			__func__, readl(base + PC_STATUS), num, readl(base + PC_SERIAL_CHECK));
		goto end;
	}
	
	DDEX("command result status = 0x%08x", readl(base + PC_STATUS));

	// Read status of command
	status = readl(base + PC_STATUS);
	writeb(0, base + PC_STATUS);
	base_status = status & 0xff;
	DDEX("status new = 0x%x; status base = 0x%x", 
		readl(base + PC_STATUS), base_status);


	if ( base_status == base_cmd )
	{
		if ( status & DEX_STATUS_FAIL )
		{
			DDEX("DEX cmd failed; status=%x, result=%x",
				readl(base + PC_STATUS),
				readl(base + PC_DATA_RESULT));

			writel(readl(base + PC_STATUS) & ~DEX_STATUS_FAIL, base + PC_STATUS);
		}
		else if ( status & DEX_HAS_DATA )
		{
			writel(readl(base + PC_STATUS) & ~DEX_HAS_DATA, base + PC_STATUS);
			if (data1)
				*data1 = readl(base + PC_DATA_RESULT);
			if (data2)
				*data2 = readl(base + PC_DATA_RESULT2);
			DDEX("DEX output data = 0x%x  data2 = 0x%x ", 
				readl(base + PC_DATA_RESULT), readl(base + PC_DATA_RESULT2));
		}
	} else {
		printk(KERN_WARNING "%s: DEX Code not match! a2m[0x%x], m2a[0x%x], a2m_num[0x%x], m2a_num[0x%x]\n", 
			__func__, base_cmd, base_status, num, readl(base + PC_SERIAL_CHECK));
	}

end:
	writel(0, base + PC_DATA_RESULT);
	writel(0, base + PC_STATUS);

	spin_unlock_irqrestore(&dex_comm_lock, flags);
	return 0;
}
int dex_audio(int param)
{
	return dex_comm(DEX_AUDIO_CALL, &param, 0);
}


#if defined(CONFIG_ARCH_QSD8X50)
#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 32 * (n))
#else
#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#endif
#define TCX0			19200000 // Hz
#define PLL_FREQ(l, m, n)	(TCX0 * (l) + TCX0 * (m) / (n))

#define DUMP_PLL(name, base) { \
	unsigned int mode, L, M, N, freq; \
	mode = readl(base); \
	L = readl(base + 0x4); \
	M = readl(base + 0x8); \
	N = readl(base + 0xc); \
	freq = PLL_FREQ(L, M, N); \
	printk(KERN_INFO "%s @ %p: MODE=%08x L=%08x M=%08x N=%08x freq=%u Hz (%u MHz)\n", \
		name, base, mode, L, M, N, freq, freq / 1000000); \
	}

// Dump useful debug stuff
void dump_debug_stuff(void)
{
	unsigned int pcb_xc, ver_base;
	char amss_ver[16];
	if (machine_is_htcleo()) {
		ver_base = 0xef230;
		// Dump PLL params (for debug purposes, no relation to dex_comm)
		DUMP_PLL("PLL0", PLLn_BASE(0));
		DUMP_PLL("PLL1", PLLn_BASE(1));
		DUMP_PLL("PLL4", PLLn_BASE(4));
		DUMP_PLL("PLL5", PLLn_BASE(5));
	}
	else {
		ver_base = 0xfc030;
		// Dump PLL params (for debug purposes, no relation to dex_comm)
		DUMP_PLL("PLL0", PLLn_BASE(0));
		DUMP_PLL("PLL1", PLLn_BASE(1));
		DUMP_PLL("PLL2", PLLn_BASE(2));
		DUMP_PLL("PLL3", PLLn_BASE(3));
	  
	}
	// Dump PCB XC
	pcb_xc = readl(MSM_SHARED_RAM_BASE + ver_base + 0x18);
	printk(KERN_INFO "PCB XC: %08x\n", pcb_xc);

	// Dump AMMS version
	*(unsigned int *) (amss_ver + 0x0) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x0);
	*(unsigned int *) (amss_ver + 0x4) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x4);
	*(unsigned int *) (amss_ver + 0x8) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x8);
	*(unsigned int *) (amss_ver + 0xc) = readl(MSM_SHARED_RAM_BASE + ver_base + 0xc);
	amss_ver[15] = 0;
	printk(KERN_INFO "AMSS version: %s\n", amss_ver);
}

///////////////////////////////////////////////////////////////////////
// DEX callback
///////////////////////////////////////////////////////////////////////

static irqreturn_t dex_cb_interrupt(int irq, void *dev_id)
{
	uint32_t nt;

	nt = readl(base + PC_NOTIFY);
	writeb(0, base + PC_NOTIFY); // clear state

   	printk("###DEX CB INTR[%08X]###\n", nt);

	if (nt & 2) // VBUS state changed
	{
		notify_vbus_change_intr();
	}

	return IRQ_HANDLED;
}


static struct irqaction dex_callback_irq =
{
	.name       = "dex_cb",
	.flags      = IRQF_TRIGGER_RISING, //IRQF_DISABLED,
	.handler    = dex_cb_interrupt
};


// Initialize DEX registers
int init_dex_comm()
{
	unsigned long flags;

	printk(KERN_INFO "%s: WinCE DEX init.\n", __func__);

	if(machine_is_htcleo())
		base = (unsigned)(MSM_SHARED_RAM_BASE + 0xefe00);
	else
		base = (unsigned)(MSM_SHARED_RAM_BASE + 0xfc100);

	setup_irq(INT_A9_M2A_4, &dex_callback_irq);

	spin_lock_irqsave(&dex_comm_lock, flags);

	writel(0, base + PC_DATA);
	writel(0, base + PC_DATA_RESULT);
	writel(0, base + PC_SERIAL);
	writel(0, base + PC_SERIAL_CHECK);
	writel(0, base + PC_STATUS);

	spin_unlock_irqrestore(&dex_comm_lock, flags);
	printk(KERN_INFO "%s: WinCE DEX initialized.\n", __func__);

	dump_debug_stuff();

	return 0;
}
