// SPDX-License-Identifier: GPL-2.0-only

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/math.h>
#include <linux/mutex.h>

#define BIMC_MON_INT_STATUS		0x1100
#define BIMC_MON_INT_CLR		0x1108
#define BIMC_MON_INT_EN			0x110c
#define MPORT_INT_STATUS		0x0100
#define MPORT_INT_CLR			0x0108
#define MPORT_INT_THRES_FLAG		BIT(0)
#define MPORT_INT_OVFLOW_FLAG		BIT(1)
#define MPORT_INT_MASK			GENMASK(1, 0)
#define MPORT_INT_EN			0x010c
#define MPORT_EN			0x0280
#define MPORT_CLEAR			0x0284
#define MPORT_CNT			0x0288
#define MPORT_THRES			0x0290

#define MAX_PORTS (7)

static const char *bimc_port_names[MAX_PORTS] = {
	"apps_proc",
	"rproc",
	"oxili",
	"snoc_bimc_0",
	"snoc_bimc_1",
	"snoc_bimc_2",
	"tcu_0",
};

struct bimc_mon {
	struct dentry *dentry;
	struct mutex lock;
	ktime_t sample_ns;
	u64 sample_us;
	u64 period_offset;
	void __iomem *addr, *port0_addr;
	u16 dfs_port_mask;
	u16 dfs_trigger_mask;
	u32 dfs_trigger_thres;
	u16 port_mask;
	u16 valid_port_mask;

	u32 counters[MAX_PORTS];
	u64 bytes[MAX_PORTS];

	u32 dfs_period_us;
	bool dfs_show_bw;
	bool dfs_show_cnt;
	bool inited;
	bool opened;
	int title;
};

#define FOR_EACH_PORT(_mon, _port, port_mask) \
	for (_port = 0; _port < MAX_PORTS; _port ++) \
	     if ((BIT(_port) & port_mask))

#define FOR_EACH_PORT_ADDR(_mon, _port, _addr, port_mask) \
	for (_port = 0, _addr = mon->port0_addr; \
	     _port < MAX_PORTS; \
	     _port ++, _addr += 0x4000) \
	     if ((BIT(_port) & port_mask))

static void bimc_mon_read_counters(struct bimc_mon *mon)
{

	unsigned int port;
	u64 ts;
	void __iomem *addr;

	ts = ktime_get_mono_fast_ns();
	FOR_EACH_PORT_ADDR(mon, port, addr, mon->port_mask) {
		u32 value = ioread32(addr + MPORT_CNT);
		mon->bytes[port] += value - mon->counters[port];
		mon->counters[port] = value;
	}
	mon->sample_ns = ts;
	do_div(ts, 1000);
	mon->sample_us = ts;
}


static void bimc_mon_switch(struct bimc_mon *mon, bool on)
{
	void __iomem *addr;
	u32 port;

	iowrite32(0, mon->addr + BIMC_MON_INT_EN);

	FOR_EACH_PORT_ADDR(mon, port, addr, mon->port_mask) {
		iowrite32(0, addr + MPORT_INT_EN);
		iowrite32(on, addr + MPORT_EN);
	}
}

static u32 calc_bw_KBs(u64 bytes, u64 ns)
{
	if (!ns)
		return 99999999;
	if (!bytes)
		return 0;

	return mult_frac(bytes, 1000000, ns);
}

#define buf_printf(__fmt__, ...) \
	do { \
		wptr += snprintf(wptr, wptr_end - wptr, __fmt__, ##__VA_ARGS__); \
		wptr = min(wptr, wptr_end); \
	} while (0)

static ssize_t bwmon_read(struct file *file, char __user *buf,
		size_t sz, loff_t *ppos)
{
	struct bimc_mon *mon = file->private_data;
	unsigned long flags;
	size_t ret = 0;
	char buffer[(MAX_PORTS * 3) * (32) + 64];
	char *wptr = buffer;
	char *wptr_end = wptr + sizeof(buffer);
	u32 sample_us;
	u64 sample_s, delay_ns;
	ktime_t now, target;
	u16 port;

	u32 period_us = min(100000, mon->dfs_period_us);
	bool show_bw = mon->dfs_show_bw;
	bool show_cnt = mon->dfs_show_cnt;

	if (sz < sizeof(buffer))
		return -ENOSPC;

	mutex_lock(&mon->lock);
	if (!mon->opened)
		goto out;

	FOR_EACH_PORT(mon, port, mon->port_mask)
		mon->bytes[port] = 0;

	mon->period_offset = min(20000000, mon->period_offset);

	if (!mon->inited) {
		local_irq_save(flags);
		bimc_mon_switch(mon, true);
		bimc_mon_read_counters(mon);
		local_irq_restore(flags);
		mon->inited = true;
		goto print_title;
	}

	ktime_t prev_ns = mon->sample_ns;
	u64 prev_us = mon->sample_us;

	delay_ns = 1000ULL * period_us;
	target = prev_ns + delay_ns;

	local_irq_save(flags);

	now = ktime_get_mono_fast_ns();
	bool delayed = false;
	while (now + mon->period_offset < target) {
		u32 delay_us = (target - now - mon->period_offset) / 1000;
		if (delay_us >= 2000) {
			local_irq_restore(flags);
			usleep_range(delay_us - 1000, delay_us - 500);
			local_irq_save(flags);
		} else {
			delayed = true;
			ndelay(target - now - mon->period_offset);
			break;
		}
		now = ktime_get_mono_fast_ns();
	}

	bimc_mon_read_counters(mon);
	local_irq_restore(flags);

	sample_s = div_u64_rem(mon->sample_us, 1000000, &sample_us);

	if (delayed) {
		if (target < mon->sample_ns)
			mon->period_offset += mon->sample_ns - target;
		else
			mon->period_offset -=
				min((u64)mon->period_offset, (u64)(target - mon->sample_ns));
	}

	buf_printf("%6llu.%06u %7llu|", sample_s, sample_us,
			mon->sample_us - prev_us);
	FOR_EACH_PORT(mon, port, mon->port_mask) {
		if (show_bw && show_cnt)
			buf_printf(" %10llu %7u|", mon->bytes[port],
					calc_bw_KBs(mon->bytes[port],
						mon->sample_ns - prev_ns));
		else if (show_cnt)
			buf_printf(" %10llu|", mon->bytes[port]);
		else
			buf_printf(" %10u|", calc_bw_KBs(mon->bytes[port],
						mon->sample_ns - prev_ns));
	}

print_title:
	buf_printf("\n");

	if (!((mon->title++) & 15)) {
		buf_printf("%13s %7s|", "Timestamp", "delta T");
		FOR_EACH_PORT(mon, port, mon->port_mask) {
			if (show_bw && show_cnt)
				buf_printf("%19s|", bimc_port_names[port]);
			else
				buf_printf("%11s|", bimc_port_names[port]);
		}
		buf_printf("\n%13s %7s|", "uSec", "uSec");
		FOR_EACH_PORT(mon, port, mon->port_mask) {
			if (show_bw && show_cnt)
				buf_printf(" %10s %7s|", "B", "KB/s");
			else if (show_cnt)
				buf_printf("%11s|", show_cnt ? "B" : "KB/s");
			else
				buf_printf("%11s|", "KB/s");
		}
		buf_printf("\n");
	}

	ret = min((u32)(wptr - &buffer[0]), (u32)sz);

	if (copy_to_user(buf, buffer, ret))
		ret = -EFAULT;
	else
		*ppos += ret;
out:
	mutex_unlock(&mon->lock);
	return ret;
}

static int bwmon_open(struct inode *inode, struct file *file)
{
	struct bimc_mon *mon = inode->i_private;
	int ret = -EBUSY;

	mutex_lock(&mon->lock);
	if (mon->opened)
		goto out;

	file->private_data = mon;
	mon->opened = true;
	mon->port_mask = mon->valid_port_mask & mon->dfs_port_mask;
	mon->dfs_port_mask = mon->port_mask;
	ret = 0;
out:
	mutex_unlock(&mon->lock);
	return ret;
}

static int bwmon_release(struct inode *inode, struct file *file)
{
	struct bimc_mon *mon = inode->i_private;

	mutex_lock(&mon->lock);
	mon->opened = false;
	mon->inited = false;
	mon->title = 0;
	bimc_mon_switch(mon, false);
	mutex_unlock(&mon->lock);

	return 0;
}

static const struct file_operations bimc_monitor_fops = {
	.owner = THIS_MODULE,
	.open = bwmon_open,
	.read = bwmon_read,
	.release = bwmon_release,
};

static struct bimc_mon bmon = { 0 };

static int __init msm8953_mon_init(void)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "qcom,msm8953-bimc");
	if (!np)
		return -ENODEV;

	of_node_put(np);

	void *addr = ioremap(0x00400000, 0x5a000);
	if (!addr)
		return -ENOMEM;

	bmon.addr = addr;
	bmon.port0_addr = bmon.addr + 0x8000;
	bmon.valid_port_mask |= GENMASK(6, 0);
	bmon.port_mask = bmon.dfs_port_mask = bmon.valid_port_mask;
	bmon.dfs_period_us = 100000;
	bmon.dfs_show_bw = true;

	mutex_init(&bmon.lock);

	bmon.dentry = debugfs_create_dir("bimc_mon", NULL);
	if (IS_ERR_OR_NULL(bmon.dentry)) {
		iounmap(bmon.addr);
		bmon.addr = NULL;
		return PTR_ERR(bmon.dentry);
	}

	debugfs_create_file("monitor", 0444, bmon.dentry, &bmon,
			&bimc_monitor_fops);
	debugfs_create_x16("port_mask", 0644, bmon.dentry,
			&bmon.dfs_port_mask);
	debugfs_create_u32("period_us", 0644, bmon.dentry,
			&bmon.dfs_period_us);
	debugfs_create_bool("show_bw", 0644, bmon.dentry,
			&bmon.dfs_show_bw);
	debugfs_create_bool("show_cnt", 0644, bmon.dentry,
			&bmon.dfs_show_cnt);
	return 0;
}
static void __exit msm8953_mon_exit(void)
{
	if (!bmon.addr)
		return;

	iounmap(bmon.addr);

	if (!IS_ERR_OR_NULL(bmon.dentry))
		debugfs_remove(bmon.dentry);

	while(READ_ONCE(bmon.opened))
		msleep(20);
}

module_init(msm8953_mon_init);
module_exit(msm8953_mon_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM8953 BIMC Bandwidth monitor (debugfs)");
