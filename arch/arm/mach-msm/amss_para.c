/*
 * Author: Markinus
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
#include <linux/init.h>

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <mach/amss_para.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>

unsigned int __amss_version = 0;

// The struckt array with the AMSS default Values, it have to have the same order and size as the enum with AMSS IDs in the Heaader File!
struct amss_value amss_def_para[] = {
	{AUDMGR_PROG_VERS,0, "rs30000013:e94e8f0c"},  
	{AUDMGR_VERS, 0xe94e8f0c, ""},  
	{AUDMGR_CB_PROG_VERS,0, "rs31000013:21570ba7"},  
	{AUDMGR_CB_VERS, 0x21570ba7, ""},  
	{TIME_REMOTE_MTOA_VERS, 0, ""}, 
	{RPC_TIME_REMOTE_MTOA_NULL, 0, ""}, 
	{RPC_TIME_TOD_SET_APPS_BASES, 1, ""},  
	{PM_LIBVERS, 0x10001, ""},  
	{RPC_SND_VERS, 0x0, ""},  
	{SND_SET_DEVICE_PROC, 1, ""},  
	{SND_SET_VOLUME_PROC, 2, ""},  
	{RPC_ADSP_RTOS_ATOM_PROG_VERS, 0, ""},
	{RPC_ADSP_RTOS_ATOM_VERS, 0x0, ""},  
	{RPC_ADSP_RTOS_MTOA_VERS, 0x0, ""},  
	{RPC_ADSP_RTOS_ATOM_NULL_PROC, 0, ""},
	{RPC_ADSP_RTOS_MTOA_NULL_PROC, 0, ""},
	{RPC_ADSP_RTOS_APP_TO_MODEM_PROC, 1, ""},
	{RPC_ADSP_RTOS_MODEM_TO_APP_PROC, 1, ""},
	{RPC_DOG_KEEPALIVE_NULL, 0, ""},  
	{RPC_DOG_KEEPALIVE_BEACON, 1, ""},  
	{DOG_KEEPALIVE_VERS, 0, ""},  
	{HTC_PROCEDURE_SET_VIB_ON_OFF, 21, ""},  
	{APP_TIMEREMOTE_PDEV_NAME, 0, "rs30000048:0x00010001"},  
};


// Now the version spezificly values
struct amss_value amss_6125_para[] = {
	{AUDMGR_PROG_VERS,0, "rs30000013:00000000"},  
	{AUDMGR_VERS, 0x00000000, ""}, /* TODO: Disabled with wrong version, Sould be 0x0, Kills Topaz Call */ 
	{AUDMGR_CB_PROG_VERS,0, "rs31000013:00000000"},  
	{AUDMGR_CB_VERS, 0x00000000, ""},  
	{PM_LIBVERS, 0x0, ""},  
	{RPC_SND_VERS, 0xaa2b1a44, ""},  
	{SND_SET_DEVICE_PROC, 2, ""},  
	{SND_SET_VOLUME_PROC, 3, ""},  
	{RPC_ADSP_RTOS_ATOM_PROG_VERS, 0, "rs3000000a:00000000"},
	{RPC_ADSP_RTOS_ATOM_VERS, 0x0, ""},  
	{RPC_ADSP_RTOS_MTOA_VERS, 0x0, ""},  
};

struct amss_value amss_6210_para[] = {
	{AUDMGR_PROG_VERS,0, "rs30000013:46255756"},  
	{AUDMGR_VERS, 0x46255756, ""},  
	{AUDMGR_CB_PROG_VERS,0, "rs31000013:5fa922a9"},  
	{AUDMGR_CB_VERS, 0x5fa922a9, ""},  
	{TIME_REMOTE_MTOA_VERS, 0, ""},  
	{RPC_TIME_TOD_SET_APPS_BASES, 2, ""},  
	{PM_LIBVERS, 0x0, ""},  
	{RPC_SND_VERS, 0x94756085, ""},  
	{SND_SET_DEVICE_PROC, 2, ""},  
	{SND_SET_VOLUME_PROC, 3, ""},  
	{RPC_ADSP_RTOS_ATOM_PROG_VERS, 0, "rs3000000a:20f17fd3"},
	{RPC_ADSP_RTOS_ATOM_VERS, 0x20f17fd3, ""},  
	{RPC_ADSP_RTOS_MTOA_VERS, 0x75babbd6, ""},  
	{RPC_ADSP_RTOS_APP_TO_MODEM_PROC, 2, ""},
	{RPC_ADSP_RTOS_MODEM_TO_APP_PROC, 2, ""},
	{RPC_DOG_KEEPALIVE_BEACON, 1, ""},  
	{DOG_KEEPALIVE_VERS, 0, ""},  
	{APP_TIMEREMOTE_PDEV_NAME, 0, "rs30000048:0x00000000"},  
};

struct amss_value amss_6220_para[] = {
	{AUDMGR_PROG_VERS,0, "rs30000013:94e8f0c"},  
	{AUDMGR_VERS, 0xe94e8f0c, ""},  
	{AUDMGR_CB_PROG_VERS,0, "rs31000013:21570ba7"},  
	{AUDMGR_CB_VERS, 0x21570ba7, ""},  
	{TIME_REMOTE_MTOA_VERS, 0x9202a8e4, ""},  
	{RPC_TIME_TOD_SET_APPS_BASES, 2, ""},  
	{PM_LIBVERS, 0xfb837d0b, ""},  
	{RPC_SND_VERS, 0xaa2b1a44, ""},  
	{RPC_ADSP_RTOS_ATOM_PROG_VERS, 0, "rs3000000a:71d1094b"},
	{RPC_ADSP_RTOS_ATOM_VERS, 0x71d1094b, ""},  
	{RPC_ADSP_RTOS_MTOA_VERS, 0xee3a9966, ""},  
	{RPC_ADSP_RTOS_APP_TO_MODEM_PROC, 2, ""},
	{RPC_ADSP_RTOS_MODEM_TO_APP_PROC, 2, ""},
	{RPC_DOG_KEEPALIVE_BEACON, 2, ""},  
	{DOG_KEEPALIVE_VERS, 0x731fa727, ""},  
	{APP_TIMEREMOTE_PDEV_NAME, 0, "rs30000048:0da5b528"},  
};


struct amss_value amss_5225_para[] = {
	{AUDMGR_PROG_VERS,0, "rs30000013:00000000"},  
	{AUDMGR_VERS, 0x0, ""},  
	{AUDMGR_CB_PROG_VERS,0, "rs31000013:5fa922a9"},  
	{AUDMGR_CB_VERS, 0x5fa922a9, ""},  
	{RPC_SND_VERS, 0x0, ""},  
	{SND_SET_DEVICE_PROC, 1, ""},  
	{SND_SET_VOLUME_PROC, 2, ""},  
	{RPC_ADSP_RTOS_ATOM_PROG_VERS, 0, "rs3000000a:00000000"},
	{RPC_ADSP_RTOS_ATOM_VERS, 0x0, ""},  
	{RPC_ADSP_RTOS_MTOA_VERS, 0x0, ""},  
};

struct amss_value amss_6150_para[] = {
	{AUDMGR_PROG_VERS,0, "rs30000013:00000000"},  
	{AUDMGR_VERS, 0x0, ""},  
	{RPC_ADSP_RTOS_ATOM_PROG_VERS, 0, "rs3000000a:00000000"},
};

struct amss_value amss_1550_para[] = {
	{TIME_REMOTE_MTOA_VERS, 0x10002, ""},  
	{HTC_PROCEDURE_SET_VIB_ON_OFF, 22, ""},  
	{RPC_DOG_KEEPALIVE_BEACON, 2, ""},  
	{DOG_KEEPALIVE_VERS, 0x10001, ""},  
	{PM_LIBVERS, 0x30001, ""},  
	{APP_TIMEREMOTE_PDEV_NAME, 0, "rs30000048:0x00010002"},  
};

// Get the short AMSS Version ( like 6120 )
unsigned int get_amss_version(void)
{
	char amss_ver[20];
	char amss_dump[20];
	char *dot1, *dot2;
	int len = 0;
	uint32_t ver_base = 0;

	/* Detection doesn't work on 'old' CDMA, there's no
	 * version string to be found anywhere in SHARED_RAM_BASE
	 */
	if (machine_is_htcdiamond_cdma() || machine_is_htcraphael_cdma() || machine_is_htcraphael_cdma500())
		return 6150;
	else if (machine_is_htcleo())
		ver_base = 0xef230;
	else
		ver_base = 0xfc030;

	// Dump AMMS version
	*(unsigned int *) (amss_dump + 0x0) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x0);
	*(unsigned int *) (amss_dump + 0x4) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x4);
	*(unsigned int *) (amss_dump + 0x8) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x8);
	*(unsigned int *) (amss_dump + 0xc) = readl(MSM_SHARED_RAM_BASE + ver_base + 0xc);
	*(unsigned int *) (amss_dump + 0x10) = readl(MSM_SHARED_RAM_BASE + ver_base + 0x10);
	amss_dump[19] = '\0';
	dot1 = strchr(amss_dump, '.');
	if(dot1 == NULL) {	// CDMA
		dot1 = strchr(amss_dump, '-');
		if(dot1 == NULL)
			return 0;
		strncpy(amss_ver, dot1+1, 4);
		amss_ver[4] = '\0';
		return  simple_strtoul(amss_ver, NULL, 10);
	}
	else { // GSM
		len = (dot1-amss_dump);
		strncpy(amss_ver, amss_dump, len);
		dot1 = strchr(dot1+1, '.');
		if(dot1 == NULL)
			return 0;
		dot2 = strchr(dot1+1, '.');
		if(dot2 == NULL)
			return 0;
		strncpy(amss_ver+len, dot1+1, (dot2-dot1)-1);
		len+= (int)(dot2-dot1)-1;
		amss_ver[len] = '\0';
		return  simple_strtoul(amss_ver, NULL, 10);
	}
}



// Function to init of the struct and get the Values, Init in first call.
int amss_get_value(int id, uint32_t *numval, char* strval, size_t size)
{
	static struct amss_value *active_para = NULL;
	struct amss_value *mach_para;
	static uint8_t init = 0, i;
	uint32_t nbr_para = 0;
	if(!init) { // First run, init the struct
		__amss_version = get_amss_version();
		if(__amss_version == 0)
			__amss_version = CONFIG_MSM_AMSS_VERSION;
		pr_info("Init amss parameters, found AMSS: %d\n", __amss_version);
		// Get the right struct
		switch(__amss_version) {
			case 5225:
				mach_para = amss_5225_para;
				nbr_para = ARRAY_SIZE(amss_5225_para);
				break;
			case 6125:
				mach_para = amss_6125_para;
				nbr_para = ARRAY_SIZE(amss_6125_para);
				break;
			case 6150:
				mach_para = amss_6150_para;
				nbr_para = ARRAY_SIZE(amss_6150_para);
				break;
			case 6210:
				mach_para = amss_6210_para;
				nbr_para = ARRAY_SIZE(amss_6210_para);
				break;
			case 6220:
			case 6225:
				mach_para = amss_6220_para;
				nbr_para = ARRAY_SIZE(amss_6220_para);
				break;
			case 1550:
				mach_para = amss_1550_para;
				nbr_para = ARRAY_SIZE(amss_1550_para);
				break;
			default:
				printk(KERN_ERR "Unsupported device for adsp driver\n");
				strval = "";
				numval = 0;
				return -ENODEV;
				break;
		}
		// Initializes the default patameters
		active_para = kmalloc(sizeof(amss_def_para), GFP_KERNEL);
		memcpy(active_para, &amss_def_para, sizeof(amss_def_para));
		// Set the versionspezificly values		
		for(i=0; i<nbr_para; i++) {
			active_para[mach_para[i].id].numval = mach_para[i].numval;
			active_para[mach_para[i].id].strval = kmalloc(sizeof(mach_para[i].strval)+1, GFP_KERNEL);
			strcpy(active_para[mach_para[i].id].strval, mach_para[i].strval);
		}

		init = 1;
	}
	// Struct ready, get Value and exit
	if(active_para[id].id != id) {	// Check the array integrity
		pr_err("AMSS Array mismatch! Check the parameters!\n");
		strval = "";
		numval = 0;
		return -EINVAL;
	}
	
	*numval = active_para[id].numval;
	memcpy (strval, active_para[id].strval, size);
//	pr_info              ("AMSS ASK: %d   GET:  %x    :%s\n", id, *numval, strval);

	return 0;
	
}


uint32_t amss_get_num_value(int id)
{
	char* str = "";
	uint32_t num = 0;
	amss_get_value(id, &num, str, 0);
	return num;
}
  
void amss_get_str_value(int id, char* str, size_t size)
{
	uint32_t num = 0;
	amss_get_value(id, &num, str, size);
}


