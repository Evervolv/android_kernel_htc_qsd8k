/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "ov9665.h"


static const struct ov9665_i2c_reg_conf const pll_setup_tbl[] = {
	{0x3E, 0xD0, BYTE_LEN, 5},
	{0x3E, 0xD0, BYTE_LEN, 5},
	{0x12, 0x80, BYTE_LEN, 5}
};

/* sensor register init*/
static const struct ov9665_i2c_reg_conf const register_init_tbl[] = {
	/*sensor reset*/
	{0x12, 0x80, BYTE_LEN, 1},
	/*sensor IO output*/
	{0xd5, 0xff, BYTE_LEN, 1},
	{0xd6, 0x3f, BYTE_LEN, 1},
	/*Clock 24MHz 10FPS*/
	{0x3d, 0x3c, BYTE_LEN, 1},
	{0x11, 0x80, BYTE_LEN, 1},/*0x81 24MHz,0x80 48MHz*/
	{0x2a, 0x00, BYTE_LEN, 1},
	{0x2b, 0x00, BYTE_LEN, 1},
	/*power control*/
	{0x3a, 0xd9, BYTE_LEN, 1},
	{0x3b, 0x00, BYTE_LEN, 1},
	{0x3c, 0x58, BYTE_LEN, 1},
	{0x3e, 0x50, BYTE_LEN, 1},
	{0x71, 0x00, BYTE_LEN, 1},
	/*driving strengh*/
	{0x09, 0x03, BYTE_LEN, 1},
	/*Data Format YUV*/
	{0xD7, 0x10, BYTE_LEN, 1},
	{0x6a, 0x24, BYTE_LEN, 1},
	{0x85, 0xe7, BYTE_LEN, 1},
	/*sample option*/
	{0x63, 0x01, BYTE_LEN, 1},
	/*Windowing*/
	{0x17, 0x0c, BYTE_LEN, 1},
	{0x18, 0x5c, BYTE_LEN, 1},
	{0x19, 0x01, BYTE_LEN, 1},
	{0x1a, 0x82, BYTE_LEN, 1},
	{0x03, 0x0f, BYTE_LEN, 1},
	{0x2b, 0x00, BYTE_LEN, 1},
	{0x32, 0x34, BYTE_LEN, 1},
	/*BLC*/
	{0x36, 0xb4, BYTE_LEN, 1},
	{0x65, 0x10, BYTE_LEN, 1},
	{0x70, 0x02, BYTE_LEN, 1},
	{0x71, 0x9c, BYTE_LEN, 1},
	{0x72, 0xc0, BYTE_LEN, 1}, /*For preview greenish in lowlight Weiting*/
	{0x64, 0x24, BYTE_LEN, 1},
	/*AEC ,Average ,9 zone*/
	{0x43, 0x00, BYTE_LEN, 1},
	{0x5d, 0x55, BYTE_LEN, 1},
	{0x5e, 0x57, BYTE_LEN, 1},
	{0x5f, 0x21, BYTE_LEN, 1},
	/*Brightness*/
	{0x24, 0x40, BYTE_LEN, 1}, /*upper bc 35ori 39*/
	{0x25, 0x35, BYTE_LEN, 1}, /*lower bc 2aori 2e*/
	{0x26, 0x82, BYTE_LEN, 1},
	/*BF 60Hz*/
	/*0x48 for 8xgain 28 for 4xgain  68for 16xgain*/
	{0x14, 0x48, BYTE_LEN, 1},
	{0x0c, 0x38, BYTE_LEN, 1},
	{0x4f, 0x9e, BYTE_LEN, 1},
	{0x50, 0x84, BYTE_LEN, 1},
	{0x5a, 0x67, BYTE_LEN, 1},
	/*LC enable*/
	{0x7d, 0x00, BYTE_LEN, 1},
	{0x7e, 0xa0, BYTE_LEN, 1},
	{0x7f, 0x00, BYTE_LEN, 1},
	{0x80, 0x09, BYTE_LEN, 1},
	{0x81, 0x0a, BYTE_LEN, 1},
	{0x82, 0x09, BYTE_LEN, 1},
	{0x83, 0x07, BYTE_LEN, 1}, /*07 enable LC 06 disable*/
	/*AWB advance*/
	{0x96, 0xf0, BYTE_LEN, 1},
	{0x97, 0x0a, BYTE_LEN, 1},
	{0x92, 0x17, BYTE_LEN, 1},
	{0x94, 0x38, BYTE_LEN, 1},
	{0x93, 0x33, BYTE_LEN, 1},
	{0x95, 0x49, BYTE_LEN, 1},
	{0x91, 0xd8, BYTE_LEN, 1},
	{0x90, 0xdf, BYTE_LEN, 1},
	{0x8e, 0x4a, BYTE_LEN, 1},
	{0x8f, 0x59, BYTE_LEN, 1},
	{0x8d, 0x12, BYTE_LEN, 1},
	{0x8c, 0x11, BYTE_LEN, 1},
	{0x8b, 0x0c, BYTE_LEN, 1},
	{0x86, 0x9e, BYTE_LEN, 1},
	{0x87, 0x11, BYTE_LEN, 1},
	{0x88, 0x22, BYTE_LEN, 1},
	{0x89, 0x05, BYTE_LEN, 1},
	{0x8a, 0x03, BYTE_LEN, 1},
	/*Gamma enable for outdoor 1228*/
	{0x9b, 0x05, BYTE_LEN, 1},  /*ori0x08  htc0d*/
	{0x9c, 0x10, BYTE_LEN, 1},  /*ori0x16  htc19*/
	{0x9d, 0x28, BYTE_LEN, 1},  /*ori0x2f   htc2e*/
	{0x9e, 0x51, BYTE_LEN, 1},  /*ori0x56   htc51*/
	{0x9f, 0x60, BYTE_LEN, 1},  /*ori0x66   htc60*/
	{0xa0, 0x6c, BYTE_LEN, 1},  /*ori0x75   htc6c*/
	{0xa1, 0x77, BYTE_LEN, 1},  /*ori0x80   htc77*/
	{0xa2, 0x81, BYTE_LEN, 1},  /*ori0x88   htc81*/
	{0xa3, 0x8a, BYTE_LEN, 1},  /*ori0x8f    htc8a*/
	{0xa4, 0x93, BYTE_LEN, 1},  /*ori0x96   htc93*/
	{0xa5, 0xa1, BYTE_LEN, 1},  /*ori0xa3   htca1*/
	{0xa6, 0xae, BYTE_LEN, 1},  /*ori0xaf    htcae*/
	{0xa7, 0xc4, BYTE_LEN, 1},  /*ori0xc4    htcc4*/
	{0xa8, 0xd6, BYTE_LEN, 1},  /*ori0xd7    htcd6*/
	{0xa9, 0xe7, BYTE_LEN, 1},  /*ori0xe8    htce7*/
	{0xaa, 0x21, BYTE_LEN, 1},  /*ori0x20    htc21*/
	/*De-noise enable auto*/
	{0xab, 0xe7, BYTE_LEN, 1},
	{0xb0, 0x43, BYTE_LEN, 1},
	{0xac, 0x04, BYTE_LEN, 1},
	{0x84, 0x80, BYTE_LEN, 1}, // For stronger de-noise ori0x50
	/*Sharpness*/
	{0xad, 0x24, BYTE_LEN, 1}, //Sharpness of 0-2xgain ori0x22
	{0xd9, 0x13, BYTE_LEN, 1}, //Sharpness of 2-4,4-8xgain ori0x64
	{0xda, 0x00, BYTE_LEN, 1}, //Sharpness of >8xgain ori0xa8
	{0xae, 0x10, BYTE_LEN, 1},
	/*Scaling*/
	{0xab, 0xe7, BYTE_LEN, 1},
	{0xb9, 0xa0, BYTE_LEN, 1},
	{0xba, 0x80, BYTE_LEN, 1},
	{0xbb, 0xa0, BYTE_LEN, 1},
	{0xbc, 0x80, BYTE_LEN, 1},
	/*CMX*/
	{0xbd, 0x04, BYTE_LEN, 1}, /*0x08   unit0a*/
	{0xbe, 0x1f, BYTE_LEN, 1}, /*0x19   unit12*/
	{0xbf, 0x03, BYTE_LEN, 1}, /*0x02   unit03*/
	{0xc0, 0x0d, BYTE_LEN, 1}, /*0x05   unit05  06*/
	{0xc1, 0x24, BYTE_LEN, 1}, /*0x28   unit0b  2c*/
	{0xc2, 0x30, BYTE_LEN, 1}, /*0x2e   unit10  33*/
	{0xc3, 0x34, BYTE_LEN, 1}, /*0x27   unit10  2b*/
	{0xc4, 0x34, BYTE_LEN, 1}, /*0x26   unit0d  2a*/
	{0xc5, 0x01, BYTE_LEN, 1}, /*0x00   unit03*/
	{0xc6, 0x9c, BYTE_LEN, 1}, /*0x98   unit98*/
	{0xc7, 0x18, BYTE_LEN, 1}, /*0x18   unit98*/
	{0x69, 0x48, BYTE_LEN, 1},
	/*UV ave*/
	{0x74, 0xc0, BYTE_LEN, 1},
	/*SAT & Brightness*/
	{0xc7, 0x18, BYTE_LEN, 1},
	{0xc8, 0x06, BYTE_LEN, 1},
	{0xcb, 0x40, BYTE_LEN, 1},
	{0xcc, 0x40, BYTE_LEN, 1},
	{0xcf, 0x00, BYTE_LEN, 1},
	{0xd0, 0x20, BYTE_LEN, 1},
	{0xd1, 0x00, BYTE_LEN, 1},
	/*BLC*/
	{0x0d, 0x82, BYTE_LEN, 1},
	{0x0d, 0x80, BYTE_LEN, 1},
	#if 1
	/*UV adjustment*/
	{0xd2, 0x80, BYTE_LEN, 1},
	{0x7c, 0x18, BYTE_LEN, 1},
	{0x65, 0x01, BYTE_LEN, 1},
	{0x66, 0x00, BYTE_LEN, 1},
	{0x41, 0xa0, BYTE_LEN, 1},
	{0x5b, 0x08, BYTE_LEN, 1},
	{0x60, 0x05, BYTE_LEN, 1},
	{0x05, 0x06, BYTE_LEN, 1},
	{0x03, 0x4f, BYTE_LEN, 1},
	{0x72, 0xc0, BYTE_LEN, 1},
	#endif
	/*output driving current*/
	{0x09, 0x03, BYTE_LEN, 1},
	/*H/V sync signal control*/
	{0xd8, 0xc4, BYTE_LEN, 1},
	{0x15, 0x02, BYTE_LEN, 1},
	/*night mode*/
	{0x03, 0x8f, BYTE_LEN, 1}, //Control the min fps 4f ->1/2 8f ->1/4 cf->1/8
	{0x0f, 0x4e, BYTE_LEN, 1},
	{0x06, 0x50, BYTE_LEN, 1}, //keep fps 30 when gain<4
#if 1
	/*mirror and flip*/
	{0x04, 0xa8, BYTE_LEN, 1},
	{0x33, 0xc8, BYTE_LEN, 1}
#else
	/*reverse mode*/
	{0x04, 0x28, BYTE_LEN, 1},
	{0x33, 0xc0, BYTE_LEN, 1}
#endif
};


struct ov9665_reg ov9665_regs = {
	.register_init = &register_init_tbl,
	.register_init_size = ARRAY_SIZE(register_init_tbl),
	.plltbl = pll_setup_tbl,
	.plltbl_size = ARRAY_SIZE(pll_setup_tbl),
};
