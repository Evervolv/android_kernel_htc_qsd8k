/* arch/arm/mach-msm/proc_engineerid.c
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "devices.h"

extern unsigned engineer_id;

static int c_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", engineer_id);

	return 0;
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations engineerid_op = {
	.start	= c_start,
	.next	= c_next,
	.stop	= c_stop,
	.show	= c_show
};

extern const struct seq_operations engineerid_op;
static int engineerid_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &engineerid_op);
}

static const struct file_operations proc_engineerid_operations = {
	.open		= engineerid_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_engineerid_init(void)
{
	proc_create("engineerid", 0, NULL, &proc_engineerid_operations);
	return 0;
}
module_init(proc_engineerid_init);

