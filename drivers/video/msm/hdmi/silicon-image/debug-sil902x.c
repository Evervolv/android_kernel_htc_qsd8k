#include <linux/delay.h>
#include <linux/debugfs.h>

#include <mach/msm_fb.h>
#include <mach/msm_hdmi.h>

#include "../include/fb-hdmi.h"
#include "../include/sil902x.h"

//#define HDMI_DEBUGFS

#if defined(HDMI_DEBUGFS)
static spinlock_t hdmi_dbgfs_lock;
ssize_t hdmi_dbgfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t hdmi_dbgfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	static char line[80], buffer[80*8*4];
	static char hextab[] = "0123456789abcdefg";
	int i, j, n = 0, v, len, offset, line_size;
	unsigned long irq_flags;
	struct hdmi_info *hdmi = (struct hdmi_info*)filp->private_data;

	len = ((int)hdmi->edid_buf[0x7e]+1) * 128;
	spin_lock_irqsave(&hdmi_dbgfs_lock, irq_flags);
	memset(line, ' ', 79);
	line[79] = '\0';
	offset = strlen("0000 | ");
	line_size = offset + 3 * 16 + 1;

	for (i = 0; i < len / 16 ; i++) {
		scnprintf(line, offset + 1, "%04x | ", (i << 4));
		for (j = 0; j < 16 ; j++) {
			v = hdmi->edid_buf[i * 16 + j];
			line[offset + j * 3] = hextab[v / 16];
			line[offset + j * 3 + 1] = hextab[v % 16];
		}
		line[line_size - 1] = '\n';
		strncpy(buffer + i * line_size, line, line_size);
		n += line_size;
	}
	spin_unlock_irqrestore(&hdmi_dbgfs_lock, irq_flags);
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

#if 0
static ssize_t hdmi_dbgfs_write(struct file *filp, const char __user *buf,
                size_t count, loff_t *ppos)
{
        unsigned long v;
        unsigned long irq_flags;
        char buff[80];
        struct tv_reg_data *trd = (struct tv_reg_data *)filp->private_data;

        if (count >= sizeof(buff))
                return -EINVAL;
        if (copy_from_user(&buff, buf, 80))
                return -EFAULT;
        buff[count] = 0;

#if 0
        spin_lock_irqsave(&hdmi_dbgfs_lock, irq_flags);
        strict_strtoul(buff, 16, &v);
        buff[strlen(buff)]=0;
        writel(v, tvenc_base+trd->offset);
        spin_unlock_irqrestore(&hdmi_dbgfs_lock, irq_flags);
#endif

        return count;
}
#endif

static struct file_operations hdmi_fops[] = {
	{
		.open  = hdmi_dbgfs_open,
		.read  = hdmi_dbgfs_read,
	}
};

int hdmi_debugfs_init(struct hdmi_info *hdmi)
{
	HDMI_DBG("%s\n", __func__);
	spin_lock_init(&hdmi_dbgfs_lock);
	hdmi->debug_dir = debugfs_create_dir(HDMI_DEBUGFS_ROOT, 0);
	if (IS_ERR(hdmi->debug_dir))
		return PTR_ERR(hdmi->debug_dir);

	// FIXME: error handling
	debugfs_create_file("dummy", 0644, hdmi->debug_dir, hdmi,
		&hdmi_fops[0]);
	edid_debugfs_init(hdmi);
	tpi_debugfs_init(hdmi);
	hdcp_debugfs_init(hdmi);
/*
	int ret;
	if (!ret) {
		pr_err("%s: failure on debugfs_create_file()\n", __func__);
		return -1;
	}
*/

	return 0;
}
#endif
