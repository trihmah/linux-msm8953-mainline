// SPDX-License-Identifier: GPL-2.0-only
//
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_rmnet.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irqflags.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/skbuff.h>
#include <linux/sysfs.h>
#include <linux/time.h>
#include <linux/wait.h>

#define IPA_DBG 0
#define IPA_STATS 0

#include "ipa.h"
#include "ipa_tracepoint.h"

#define IPA_NUM_PIPES			(20)

#define P_CMD_IRQ_MASK			(P_ERR_EN | P_TRNSFR_END_EN)
#define P_TX_IRQ_MASK			(P_ERR_EN | P_PRCSD_DESC_EN | P_OUT_OF_DESC_EN)
#define P_RX_IRQ_MASK			(P_ERR_EN | P_TRNSFR_END_EN)

#define IPA_EP_DMA_DIR(ep)		((ep)->is_rx ? DMA_FROM_DEVICE : DMA_TO_DEVICE)

#define FIFO_WRAP(mask, offset)		((mask) & (offset))
#define FIFO_NEXT(mask, offset)		((mask) & (offset + 1))
#define FIFO_DIST(mask, head, tail)	((mask) & ((tail) - (head)))
#define FIFO_SPACE(mask, head, tail)	((mask) & ((head) - (tail) - 1))

#define IPA_TX_UNIT			(128)
#define IPA_RX_UNIT			(512)
#define IPA_MAX_MTU			(IPA_RX_UNIT * 3)
#define ipa_dbg(fmt, ...) if (IPA_DBG) \
	printk("%s: " fmt "\n", __func__, ##__VA_ARGS__)

#if IPA_STATS && IPA_DBG
#define IPA_STATS_ADD(cntr, val) if (IPA_STATS) (cntr += val)
#define IPA_STATS_INC(cntr) if (IPA_STATS) (cntr ++)
#else
#define IPA_STATS_ADD(cntr, val) do { } while (0)
#define IPA_STATS_INC(cntr) do { } while (0)
#endif

static u8 txfifo_sz_order = 6;
static u8 rxfifo_sz_order = 6; /* 64 desc + 32KB RX BUF */
static u8 txbuf_sz_kb_order = 5; /* 32KB TX BUF */
module_param(txbuf_sz_kb_order, byte, 0444);
module_param(txfifo_sz_order, byte, 0444);
module_param(rxfifo_sz_order, byte, 0444);

static u16 average_bw_mbits = 200;
static u16 peak_bw_mbits = 1200;
static u32 min_rate_khz = 9600;

module_param(average_bw_mbits, ushort, 0644);
module_param(peak_bw_mbits, ushort, 0644);
module_param(min_rate_khz, uint, 0644);

#if IPA_DBG
static bool enable_testpipe = false;
module_param(enable_testpipe, bool, 0444);
MODULE_PARM_DESC(enable_testpipe, "Create a pair of interfaces for testing and experiments only.");
#else
#define enable_testpipe (false)
#endif

/*
 * For kernel to pass packets through ipa network namespace have to be
 * used or the kernel with create short-cut.
 *
 * for X in 0 1
 * do
 *   ip netns add test_ns$X
 *   ip link set ipa_testpipe$X netns test_ns$X
 *   ip netns exec test_ns$X ip link add link ipa_testpipe$X name test_rmnet$X type rmnet mux_id 1
 *   ip netns exec test_ns$X ip link set lo up
 *   ip netns exec test_ns$X ip link set ipa_testpipe$X up
 *   ip netns exec test_ns$X ip link set test_rmnet$X up
 *   ip netns exec test_ns$X ip addr add 192.168.100.10${X}/24 dev test_rmnet$X
 * done
 *
 * ip netns exec test_ns0 iperf3 -s &
 * ip netns exec test_ns1 iperf3 -c 192.168.100.100
 *
 */

static int ipa_modem_tx_id = 4, ipa_modem_rx_id = 5;
static DEVICE_INT_ATTR(rx_endpoint_id, 0444, ipa_modem_rx_id);
static DEVICE_INT_ATTR(tx_endpoint_id, 0444, ipa_modem_tx_id);

static const char *ipa_ep_names[EP_MAX_NUM] = {
	[EP_TX]		= "TX",
	[EP_RX]		= "RX",
	[EP_TEST_TX1]	= "TEST_TX1",
	[EP_TEST_RX1]	= "TEST_RX1",
	[EP_TEST_TX2]	= "TEST_TX2",
	[EP_TEST_RX2]	= "TEST_RX2",
	[EP_LAN_RX]	= "LAN_RX",
	[EP_CMD]	= "CMD",
};

static const u8 ipa_2p5_pipe_map[EP_MAX_NUM] = {
	[EP_TX]		= 4,
	[EP_RX]		= 5,
	[EP_TEST_TX1]	= 0,
	[EP_TEST_RX1]	= 1,
	[EP_TEST_TX2]	= 10,
	[EP_TEST_RX2]	= 15,
	[EP_LAN_RX]	= 2,
	[EP_CMD]	= 3,
};

static const u8 ipa_2p5_mdm_pipes[] = {
	6, 7, 11, 13, /* TX */
	8, 9, 12, 14, /* RX */
};

struct ipa {
	struct mutex lock;
	struct clk *clk;
	struct device *dev;
	struct ipa_ep ep[EP_MAX_NUM];
	struct ipa_partition layout[MEM_PART_NUM];
	struct ipa_qmi *qmi;
	struct net_device *modem, *lan, *tp0, *tp1;
	struct notifier_block ssr_nb;
	struct wait_queue_head cmd_wq;
	struct dentry *dfs;
	struct icc_bulk_data icc_bulk_data[3];
	u32 version, smem_size, smem_restr_bytes;
	void *ssr_cookie;
	void __iomem *mmio;

	/* Coherent memory */
	dma_addr_t cdma_addr;
	void *cdma_virt;
	u32 cdma_size;

	/* Sub-allocated from cdma_addr */
	dma_addr_t zero_rule, test_rules, scratch, sys_hdr;
	u32 scratch_size;

	u8 num_used_ep;
	bool init_done;
	bool uc_loaded;
	bool imm_failed;
};

struct ipa_ndev {
	struct ipa_ep *rx, *tx;
	struct dentry *dfs;

	struct napi_struct napi_rx;
	struct napi_struct napi_tx;

#if IPA_DBG
	u32 rx_delay_us;
#endif

#if IPA_STATS
	u64 tx_poll_total;
	u64 rx_poll_total;
	u64 rx_poll_empty;
	u64 tx_poll_empty;
	u64 rx_poll_full;
	u64 tx_poll_full;
#endif
};

#define IPA_MIN_RULE_SIZE ALIGN((4 * (1 + 2 + 2)), 8)
#define TX2_TO_RT1_OFF (0 * IPA_MIN_RULE_SIZE)
#define TX1_TO_RT2_OFF (1 * IPA_MIN_RULE_SIZE)
#define RT1_TO_RX1_OFF (2 * IPA_MIN_RULE_SIZE)
#define RT2_TO_RX2_OFF (3 * IPA_MIN_RULE_SIZE)
#define IPA_TEST_RULES_SIZE ALIGN(IPA_MIN_RULE_SIZE * 4, 8)
#define IPA_SYS_HDR_SIZE (16)
#define IPA_ZERO_RULE_SIZE (16)

static void ipa_write_flt_rt_rule(void *output, u32 hdr)
{
	static const struct {
		u64 offset :8;
		u64 max_val :16;
		u64 min_val :16;
		u64 padding :24;
	} payload = {
		.offset = 0,
		.max_val = 0xffff,
		.min_val = 0,
	};
	memcpy(output, &hdr, sizeof(hdr));
	memcpy(output + sizeof(hdr), &payload, sizeof(payload));
}

static void ipa_write_flt_rule(void *output, u8 offset, u8 rt_idx)
{
	struct ipa_flt_rule_hw_hdr hdr = {
		.u.hdr = {
			.en_rule = IPA_IHL_OFFSET_RANGE16_0,
			.action = IPA_PASS_TO_ROUTING,
			.rt_tbl_idx = rt_idx,
		},
	};

	ipa_write_flt_rt_rule(output + offset, hdr.u.word);
}

static void ipa_write_rt_rule(void *output, u8 offset, u8 dest_pipe)
{
	struct ipa_rt_rule_hw_hdr hdr = {
		.u.hdr = {
			.en_rule = IPA_IHL_OFFSET_RANGE16_0,
			.pipe_dest_idx = dest_pipe,
			.system = 1,
			.hdr_offset = 0,
		},
	};

	ipa_write_flt_rt_rule(output + offset, hdr.u.word);
}

static void ipa_reset_hw(struct ipa *ipa)
{
	bool full_reset = true;

	/* Can't make modem to re-configure ipa */
	full_reset = !ioread32(ipa->mmio + REG_IPA_EP_STATUS(ipa_2p5_mdm_pipes[0]));

	ipa_dbg("doing %s reset", full_reset ? "full" : "partial");

	if (full_reset) {
		iowrite32(1, ipa->mmio + REG_IPA_COMP_SW_RESET_OFST);
		iowrite32(0, ipa->mmio + REG_IPA_COMP_SW_RESET_OFST);
		iowrite32(1, ipa->mmio + REG_IPA_COMP_CFG_OFST);
		if (ipa->version >= 25)
			iowrite32(0x1fff7f, ipa->mmio + REG_IPA_BCR_OFST);

		iowrite32(BAM_SW_RST, ipa->mmio + REG_BAM_CTRL);
		iowrite32(0, ipa->mmio + REG_BAM_CTRL);
	}

	iowrite32(0x10, ipa->mmio + REG_BAM_DESC_CNT_TRSHLD);
	iowrite32((u32)~BIT(11), ipa->mmio + REG_BAM_CNFG_BITS);

	iowrite32(ioread32(ipa->mmio + REG_BAM_CTRL) | BAM_EN,
		  ipa->mmio + REG_BAM_CTRL);

	iowrite32(0, ipa->mmio + REG_BAM_IRQ_EN);
	iowrite32(0, ipa->mmio + REG_BAM_IRQ_SRCS_MSK_EE0);

	iowrite32(0, ipa->mmio + REG_IPA_IRQ_EN_EE0);
}

static int ipa_ep_reset_pipe(struct ipa_ep *ep, bool shutdown)
{
	struct ipa *ipa = ep->ipa;
	void *mmio = ep->ipa->mmio;
	u32 val, hwidx = ep->hwidx;
	dma_addr_t fifo_addr;

	ipa_dbg("ep %d shutdown %d", ep->id, shutdown);

	iowrite32(ep->id != EP_CMD, mmio + REG_IPA_EP_CTRL(hwidx));
	iowrite32(ep->is_rx ? 1 : 0, mmio + REG_IPA_EP_HOL_BLOCK_EN(hwidx));

	iowrite32(0, mmio + REG_BAM_P_CTRL(hwidx));
	iowrite32(1, mmio + REG_BAM_P_RST(hwidx));
	iowrite32(0, mmio + REG_BAM_P_RST(hwidx));

	ep->head = ep->tail = 0;

	if (WARN_ON(ioread32(ep->hw_head) != 0 || ioread32(ep->hw_tail) != 0)) {
		dev_err(ipa->dev, "head=%x tail=%x\n",
				ioread32(ep->hw_head),
				ioread32(ep->hw_tail));
		return -EIO;
	}

	if (shutdown)
		return 0;

	fifo_addr = ipa->cdma_addr + (((void*)ep->fifo) - ipa->cdma_virt);

	iowrite32(fifo_addr, mmio + REG_BAM_P_DESC_FIFO_ADDR(hwidx));
	iowrite32((ep->fifo_mask + 1) * 8,
			mmio + REG_BAM_P_FIFO_SIZES(hwidx));
	iowrite32(0, mmio + REG_BAM_P_IRQ_EN(hwidx));

	val = ep->is_rx ? P_DIRECTION : 0;

	iowrite32(P_SYS_MODE | P_EN | val, mmio + REG_BAM_P_CTRL(hwidx));
	return 0;
}

static void ipa_setup_ep(struct ipa *ipa, struct ipa_ep *ep)
{
	int hwidx = ep->hwidx, id = ep->id;

	u32 val = readl_relaxed(ipa->mmio + REG_BAM_IRQ_SRCS_MSK_EE0);
	writel_relaxed(BIT(hwidx) | val, ipa->mmio + REG_BAM_IRQ_SRCS_MSK_EE0);

	switch (id) {
	case EP_LAN_RX:
		ep->is_sts = 1;
		iowrite32(FIELD_PREP(IPA_EP_HDR_LEN_BMSK, 2),
			  ipa->mmio + REG_IPA_EP_HDR(hwidx));
		iowrite32(IPA_EP_HDR_EXT_ENDIANNESS_BMSK | // BE
			  IPA_EP_HDR_EXT_TOTAL_LEN_OR_PAD_VALID_BMSK |
			  FIELD_PREP(IPA_EP_HDR_EXT_PAD_TO_ALIGNMENT_BMSK_v2_0, 2),
			  ipa->mmio + REG_IPA_EP_HDR_EXT(hwidx));
		iowrite32(0x00000000, ipa->mmio + REG_IPA_EP_HDR_METADATA_MASK(hwidx));
		iowrite32(IPA_EP_STATUS_EN_BMSK, ipa->mmio + REG_IPA_EP_STATUS(hwidx));
		break;
	case EP_TX:
	case EP_TEST_TX1:
	case EP_TEST_TX2:
		u32 meta_ofst = (id == EP_TX) ? 0 : 1;
		iowrite32(FIELD_PREP(IPA_EP_HDR_LEN_BMSK, 4) |
			  IPA_EP_HDR_OFST_METADATA_VALID_BMSK |
			  FIELD_PREP(IPA_EP_HDR_OFST_METADATA_BMSK, meta_ofst),
			  ipa->mmio + REG_IPA_EP_HDR(hwidx));

		iowrite32(IPA_EP_HDR_EXT_ENDIANNESS_BMSK, // BE
			  ipa->mmio + REG_IPA_EP_HDR_EXT(hwidx));
		iowrite32(IPA_EP_STATUS_EN_BMSK |
			  FIELD_PREP(IPA_EP_STATUS_EP_BMSK, ipa->ep[EP_LAN_RX].hwidx),
			  ipa->mmio + REG_IPA_EP_STATUS(hwidx));
		iowrite32(FIELD_PREP(IPA_EP_ROUTE_TABLE_INDEX_BMSK, 7),
			  ipa->mmio + REG_IPA_EP_ROUTE(hwidx));
		iowrite32(FIELD_PREP(IPA_EP_MODE_DEST_PIPE_INDEX_BMSK_v2_0, 2),
			  ipa->mmio + REG_IPA_EP_MODE(hwidx));
		break;
	case EP_RX:
	case EP_TEST_RX1:
	case EP_TEST_RX2:
		iowrite32(FIELD_PREP(IPA_EP_HDR_LEN_BMSK, 4) |
			  IPA_EP_HDR_OFST_PKT_SIZE_VALID_BMSK |
			  IPA_EP_HDR_OFST_METADATA_VALID_BMSK |
			  FIELD_PREP(IPA_EP_HDR_OFST_METADATA_BMSK, 1) |
			  FIELD_PREP(IPA_EP_HDR_OFST_PKT_SIZE_BMSK, 2),
			  ipa->mmio + REG_IPA_EP_HDR(hwidx));

		iowrite32(IPA_EP_HDR_EXT_ENDIANNESS_BMSK | // BE
			  IPA_EP_HDR_EXT_TOTAL_LEN_OR_PAD_VALID_BMSK |
			  IPA_EP_HDR_EXT_PAYLOAD_LEN_INC_PADDING_BMSK,
			  ipa->mmio + REG_IPA_EP_HDR_EXT(hwidx));
		iowrite32(0xff000000, ipa->mmio + REG_IPA_EP_HDR_METADATA_MASK(hwidx));
		break;
	}
}

static inline void* ipa_cdma_to_virt(struct ipa *ipa, dma_addr_t addr)
{
	return ipa->cdma_virt + (addr - ipa->cdma_addr);
}

static inline bool ipa_ep_check_tx_space(struct ipa_ep *ep, u32 len)
{
	u32 avail = FIFO_SPACE(ep->dfifo_mask,
			READ_ONCE(ep->tx_head), READ_ONCE(ep->tx_tail));

	return len <= avail * IPA_TX_UNIT &&
		FIFO_SPACE(ep->fifo_mask, READ_ONCE(ep->head),
				READ_ONCE(ep->tail)) >= 2;
}

static void ipa_uc_loaded_update(struct ipa *ipa, bool is_loaded)
{
	if (is_loaded)
		WRITE_ONCE(ipa->uc_loaded, true);

	if (ioread32(ipa->mmio + REG_IPA_EP_STATUS(ipa_2p5_mdm_pipes[0])))
		WRITE_ONCE(ipa->uc_loaded, true);
	else if (ipa->uc_loaded)
		dev_warn(ipa->dev, "unexpected modem endpoint status: 0");

	if (!ipa->uc_loaded)
		return;

	dev_info(ipa->dev, "UC is loaded\n");

	guard(mutex)(&ipa->lock);

	if (ipa->qmi)
		ipa_qmi_uc_loaded(ipa->qmi);

	if (pm_runtime_enabled(ipa->dev))
		return;

	pm_runtime_get_noresume(ipa->dev);
	pm_runtime_enable(ipa->dev);
	pm_runtime_put(ipa->dev);
}

static int ipa_uc_cmd(struct ipa *ipa, u8 cmd_op, u32 cmd_param, u32 resp_status)
{
	uint32_t resp_compl = IPA_UC_RESPONSE_CMD_COMPLETED;
	unsigned long timeout = msecs_to_jiffies(1000);
	int val;
	void __iomem *resp_reg = ipa->mmio + REG_IPA_UC_RESP;

	if (!READ_ONCE(ipa->uc_loaded))
		return 0;

	guard(mutex)(&ipa->lock);

	ipa_dbg("%d", cmd_op);

	iowrite32(cmd_op, ipa->mmio + REG_IPA_UC_CMD);
	iowrite32(cmd_param, ipa->mmio + REG_IPA_UC_CMD_PARAM);
	iowrite32(0, resp_reg);
	iowrite32(0, ipa->mmio + REG_IPA_UC_RESP_PARAM);

	iowrite32(1, ipa->mmio + REG_IPA_IRQ_UC_EE0);
	iowrite32(BIT(IPA_IRQ_UC_IRQ_1), ipa->mmio + REG_IPA_IRQ_CLR_EE0);
	iowrite32(BIT(IPA_IRQ_UC_IRQ_1), ipa->mmio + REG_IPA_IRQ_EN_EE0);

	if (cmd_op == IPA_UC_CMD_CLK_GATE || cmd_op == IPA_UC_CMD_CLK_UNGATE)
		readb_poll_timeout(resp_reg, val,
				val == resp_compl, 100, 1000000);
	else
		wait_event_timeout(ipa->cmd_wq, resp_compl ==
				FIELD_GET(IPA_UC_RESP_OP_MASK, ioread32(resp_reg)),
				timeout);

	if (FIELD_GET(IPA_UC_RESP_OP_MASK, ioread32(resp_reg)) != resp_compl)
		return -ETIMEDOUT;

	val = FIELD_GET(IPA_UC_RESP_OP_PARAM_STATUS_MASK,
			ioread32(ipa->mmio + REG_IPA_UC_RESP_PARAM));
	if (val != resp_status) {
		dev_err(ipa->dev, "cmd %d returned unexpected status: %d\n",
			cmd_op, val);
		return -EINVAL;
	}

	return 0;
}

static void ipa_reset_modem_pipes(struct ipa *ipa)
{
	int i, ret;
	u32 param;

	for (i = 0; i < ARRAY_SIZE(ipa_2p5_mdm_pipes); i++) {
		param = IPA_UC_CMD_RESET_PIPE_PARAM(ipa_2p5_mdm_pipes[i], i >= 4);
		ret = ipa_uc_cmd(ipa, IPA_UC_CMD_RESET_PIPE, param, 0);
		if (!ret)
			continue;

		dev_err(ipa->dev, "failed to reset modem pipe (%d): %d\n",
				ipa_2p5_mdm_pipes[i], ret);
	}
}

static void ipa_add_part(struct ipa *ipa, u32 *offset,
		enum ipa_part_id id, u32 size_words, u32 canary_cnt)
{
	if (!size_words)
		return;

	ipa->layout[id].offset = *offset;
	ipa->layout[id].size = size_words * 4;

	*offset += size_words * 4;

	u32 __iomem *ptr = ipa->mmio + IPA_SRAM_BASE + *offset;
	if (ipa->version < 25)
		ptr = ((void*)ptr) + ipa->smem_restr_bytes;

	while (canary_cnt-- > 0 && *offset < ipa->smem_size) {
		*(ptr++) = 0xdeadbeaf;
		*offset += 4;
	}

	ipa_dbg("part %d offset:%#x size: %#x", id,
			ipa->layout[id].offset, ipa->layout[id].size);
}

static int ipa_partition_mem(struct ipa *ipa)
{
	u32 off = 0, val;

	val = ioread32(ipa->mmio + REG_IPA_SHARED_MEM);

	ipa->smem_restr_bytes = FIELD_GET(IPA_SHARED_MEM_BADDR_BMSK, val);
	ipa->smem_size = FIELD_GET(IPA_SHARED_MEM_SIZE_BMSK, val);

	if (WARN_ON(ipa->smem_restr_bytes > ipa->smem_size ||
		    (ipa->smem_restr_bytes & 3) || ipa->smem_size & 3))
		return -EINVAL;

	ipa_dbg("smem size:%#x start:%#x",
			ipa->smem_size, ipa->smem_restr_bytes);

	off = ipa->smem_restr_bytes;

	/* UC MEM + UC Info */
	ipa_add_part(ipa, &off, MEM_UC_INFO, 160, 2);
	ipa_add_part(ipa, &off, MEM_FT_V4, IPA_NUM_PIPES + 2, 2);
	ipa_add_part(ipa, &off, MEM_FT_V6, IPA_NUM_PIPES + 2, 2);
	/* 2.0 modem needs 4 for RT4 or RT6 */
	ipa_add_part(ipa, &off, MEM_RT_V4, 7, 0);
	ipa_add_part(ipa, &off, MEM_RT_AP_V4, 8, 1);
	ipa_add_part(ipa, &off, MEM_RT_V6, 7, 0);
	ipa_add_part(ipa, &off, MEM_RT_AP_V6, 8, 1);
	ipa_add_part(ipa, &off, MEM_MDM_HDR, 80, 2);
	ipa_add_part(ipa, &off, MEM_MDM_HDR_PCTX, (ipa->version == 25) * 128, 1);
	ipa_add_part(ipa, &off, MEM_MDM_COMP, (ipa->version == 26) * 128, 1);
	ipa_add_part(ipa, &off, MEM_MDM, (ipa->smem_size - off) / 4 - 1, 1);

	return (off > ipa->smem_size) ? -EOVERFLOW : 0;
}

static void ipa_imm_cmd(struct ipa *ipa, int *status, void *cmd, int opcode)
{
	unsigned long timeout = msecs_to_jiffies(3000);
	struct ipa_ep *ep = &ipa->ep[EP_CMD];
	struct fifo_desc desc = { 0 };
	u32 val;

	ipa_dbg("opc %d", opcode);

	if (*status)
		return;

	lockdep_assert_held(&ipa->lock);

	ep->head = FIFO_WRAP(ep->fifo_mask, ioread32(ep->hw_head) / 8);
	if (ep->head != ep->tail || ipa->imm_failed) {
		int ret = ipa_ep_reset_pipe(ep, false);
		if (ret) {
			*status = ret;
			return;
		}
	}

	val = ioread32(ipa->mmio + REG_BAM_P_IRQ_STTS(ep->hwidx));
	iowrite32(val, ipa->mmio + REG_BAM_P_IRQ_CLR(ep->hwidx));
	iowrite32(P_CMD_IRQ_MASK, ipa->mmio + REG_BAM_P_IRQ_EN(ep->hwidx));

	desc.addr = cmd - ipa->cdma_virt + ipa->cdma_addr;
	desc.flags = DESC_FLAG_IMMCMD | DESC_FLAG_EOT;
	desc.opcode = opcode;

	ep->fifo[ep->tail] = desc;
	WRITE_ONCE(ep->tail, FIFO_NEXT(ep->fifo_mask, ep->tail));
	wmb();
	iowrite32(ep->tail * 8, ep->hw_tail);

	wait_event_timeout(ipa->cmd_wq,
			ioread32(ep->hw_head) == ioread32(ep->hw_tail), timeout);

	iowrite32(0, ipa->mmio + REG_BAM_P_IRQ_EN(ep->hwidx));

	ep->head = FIFO_WRAP(ep->fifo_mask, ioread32(ep->hw_head) / 8);

	if (ep->head == ep->tail) {
		ipa->imm_failed = false;
		return;
	}

	dev_err(ipa->dev, "timeout head=%x tail=%x hh=%x ht=%x\n",
			ioread32(ep->hw_head), ioread32(ep->hw_tail),
			ep->head, ep->tail);

	if (ipa->imm_failed) {
		*status = -ETIMEDOUT;
		return;
	}

	ipa->imm_failed = true;
	ipa_imm_cmd(ipa, status, cmd, opcode);
}

static int ipa_init_sram_part(struct ipa *ipa, enum ipa_part_id mem_id)
{
	u32 part_offset, part_size, pld_size, *pld, idx;
	struct ipa_partition *part = ipa->layout + mem_id;
	union ipa_cmd *cmd, zero_cmd = { 0 };
	int ret = 0;

	if (!part->size)
		return 0;

	cmd = ipa_cdma_to_virt(ipa, ipa->scratch);

	dma_addr_t pld_addr = ipa->scratch + ALIGN(sizeof(*cmd), 8);
	pld_size = ipa->scratch_size - ALIGN(sizeof(*cmd), 8);
	pld = ipa_cdma_to_virt(ipa, pld_addr);

	part_offset = part->offset;
	part_size = part->size;

	/* Merge our (virtual) AP partitions for custom rules
	 * Modem only controls lower section
	 */
	part_size += (mem_id == MEM_RT_V4) ? ipa->layout[MEM_RT_AP_V6].size : 0;
	part_size += (mem_id == MEM_RT_V6) ? ipa->layout[MEM_RT_AP_V6].size : 0;

	memset(pld, 0, min(part_size, pld_size));

	idx = 0;

	switch (mem_id) {
	case MEM_UC_INFO:
	case MEM_RT_AP_V4:
	case MEM_RT_AP_V6:
		return 0;

	case MEM_MDM_HDR:
		/* this doesn't have to be here */
		if (part_size > pld_size)
			return -EOVERFLOW;

		*cmd = zero_cmd;
		cmd->hdr_system_init.hdr_table_addr = ipa->sys_hdr;
		ipa_imm_cmd(ipa, &ret, cmd, IPA_CMD_HDR_SYSTEM_INIT);

		*cmd = zero_cmd;
		cmd->hdr_local_init.hdr_table_src_addr = pld_addr;
		cmd->hdr_local_init.hdr_table_dst_addr = part_offset;
		cmd->hdr_local_init.size_hdr_table = part_size;
		ipa_imm_cmd(ipa, &ret, cmd, IPA_CMD_HDR_LOCAL_INIT);
		fallthrough;
	case MEM_MDM_HDR_PCTX:
	case MEM_MDM_COMP:
	case MEM_MDM:
		while (part_size && ret == 0) {
			*cmd = zero_cmd;
			cmd->dma_smem.system_addr = pld_addr;
			cmd->dma_smem.local_addr = part_offset;
			cmd->dma_smem.size = min(pld_size, part_size);
			ipa_imm_cmd(ipa, &ret, cmd, IPA_CMD_DMA_SHARED_MEM);

			part_offset += min(pld_size, part_size);
			part_size -= min(pld_size, part_size);
		}

		return ret;
	case MEM_FT_V4:
	case MEM_FT_V6:
		pld[idx++] = 0x1fffff;
		fallthrough;
	case MEM_RT_V4:
	case MEM_RT_V6:
		if (part_size > pld_size)
			return -EOVERFLOW;

		while (idx < part_size / 4)
			pld[idx++] = ipa->zero_rule;
		break;
	default:
		return WARN_ON(-EINVAL);
	}

	/*
	 * Install loopback (pipe) rules:
	 * EP_TEST_TX1 -> TX1_TO_RT2_OFF -> RT2_TO_RX2_OFF -> EP_TEST_RX2
	 * EP_TEST_TX2 -> TX2_TO_RT1_OFF -> RT1_TO_RX1_OFF -> EP_TEST_RX1
	 */
	if (enable_testpipe && mem_id == MEM_FT_V4) {
		pld[2 + ipa->ep[EP_TEST_TX1].hwidx] = ipa->test_rules + TX1_TO_RT2_OFF;
		pld[2 + ipa->ep[EP_TEST_TX2].hwidx] = ipa->test_rules + TX2_TO_RT1_OFF;
	} else if (enable_testpipe && mem_id == MEM_RT_V4) {
		/* Modify MEM_RT_APPS_V4 */
		pld[part->size / 4 + 1] = ipa->test_rules + RT1_TO_RX1_OFF;
		pld[part->size / 4 + 2] = ipa->test_rules + RT2_TO_RX2_OFF;
	}

	switch (mem_id) {
	case MEM_RT_V4:
	case MEM_FT_V4:
		*cmd = zero_cmd;
		cmd->rule_v4_init.ipv4_addr = part_offset;
		cmd->rule_v4_init.size_ipv4_rules = part_size;
		cmd->rule_v4_init.ipv4_rules_addr = pld_addr;
		ipa_imm_cmd(ipa, &ret, cmd, (mem_id == MEM_RT_V4) ?
				IPA_CMD_RT_V4_INIT : IPA_CMD_FT_V4_INIT);
		break;
	case MEM_RT_V6:
	case MEM_FT_V6:
		*cmd = zero_cmd;
		cmd->rule_v6_init.ipv6_addr = part_offset;
		cmd->rule_v6_init.size_ipv6_rules = part_size;
		cmd->rule_v6_init.ipv6_rules_addr = pld_addr;
		ipa_imm_cmd(ipa, &ret, cmd, (mem_id == MEM_RT_V6) ?
				IPA_CMD_RT_V6_INIT : IPA_CMD_FT_V6_INIT);
	default:
		break;
	}

	return ret;
}

static int ipa_init_sram(struct ipa *ipa)
{
	enum ipa_part_id part;
	int ret;

	guard(mutex)(&ipa->lock);

	for (part = 0; part < MEM_PART_NUM; part++) {
		ret = ipa_init_sram_part(ipa, part);
		if (ret)
			return ret;
	}

	return 0;
}

static int ipa_ep_manage_data_fifo(struct ipa_ep *ep, bool alloc)
{
	u32 dma_size = ep->dfifo_mask + 1;
	dma_size *= (ep->is_rx ? IPA_RX_UNIT : IPA_TX_UNIT);

	if (!alloc) {
		dma_free_noncoherent(ep->ipa->dev, dma_size, ep->dfifo,
				ep->dfifo_dma, IPA_EP_DMA_DIR(ep));
		ep->dfifo = NULL;
		return 0;
	}

	ep->dfifo = dma_alloc_noncoherent(ep->ipa->dev, dma_size,
			&ep->dfifo_dma, IPA_EP_DMA_DIR(ep), GFP_KERNEL);
	if (!ep->dfifo)
		return -ENOMEM;

	dma_sync_single_for_device(ep->ipa->dev, ep->dfifo_dma,
			dma_size, IPA_EP_DMA_DIR(ep));
	return 0;
}

static int ipa_ep_reset_flush(struct ipa_ep *ep, bool shutdown)
{
	int ret = ipa_ep_reset_pipe(ep, shutdown);

	ep->tx_head = ep->tx_tail = 0;
	ep->tx_int_desc = ep->tx_int_unit = 0;
	ep->rx_len = 0;

	if (shutdown)
		ipa_ep_manage_data_fifo(ep, false);

	return ret;
}

static int ipa_ssr_notifier(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	struct ipa *ipa = container_of(nb, struct ipa, ssr_nb);

	if (action == QCOM_SSR_BEFORE_SHUTDOWN) {
		ipa_modem_set_present(ipa->dev, false);
	} else if (action == QCOM_SSR_AFTER_SHUTDOWN) {
		guard(pm_runtime_active)(ipa->dev);
		ipa_reset_modem_pipes(ipa);
		ipa_init_sram(ipa);
	} else {
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static irqreturn_t ipa_isr_thread(int irq, void *data)
{
	struct ipa *ipa = data;
	u32 val;

	guard(pm_runtime_active)(ipa->dev);

	val = ioread32(ipa->mmio + REG_IPA_IRQ_STTS_EE0);
	iowrite32(val, ipa->mmio + REG_IPA_IRQ_CLR_EE0);

	if (val & BIT(IPA_IRQ_UC_IRQ_1)) {
		val = ioread32(ipa->mmio + REG_IPA_UC_RESP);
		val &= IPA_UC_RESP_OP_MASK;
		if (val == IPA_UC_RESPONSE_INIT_COMPLETED) {
			ipa_uc_loaded_update(ipa, true);
		} else if (val == IPA_UC_RESPONSE_CMD_COMPLETED) {
			wake_up_all(&ipa->cmd_wq);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t ipa_dma_isr(int irq, void *data)
{
	struct ipa *ipa = data;
	enum ipa_ep_id id;
	u32 srcs;

	srcs = readl_relaxed(ipa->mmio + REG_BAM_IRQ_SRCS_EE0);

	for (id = 0; srcs && id < ipa->num_used_ep; id++) {
		struct ipa_ep *ep = ipa->ep + id;

		u32 hwidx = ep->hwidx;
		if (!(srcs & BIT(hwidx)))
			continue;

		u32 val = ioread32(ipa->mmio + REG_BAM_P_IRQ_STTS(hwidx));

		srcs &= ~BIT(hwidx);

		iowrite32(val, ipa->mmio + REG_BAM_P_IRQ_CLR(hwidx));

		if (unlikely(id == EP_CMD)) {
			wake_up_all(&ipa->cmd_wq);
			continue;
		}

		if (napi_schedule_prep(ep->napi)) {
			iowrite32(0, ipa->mmio + REG_BAM_P_IRQ_EN(hwidx));
			__napi_schedule_irqoff(ep->napi);
			trace_ipa2_irq(ep, true, val);
		} else {
			trace_ipa2_irq(ep, false, val);
		}

		IPA_STATS_INC(ep->irq_cnt);

		if (unlikely(val & P_ERR_EN)) {
			if (ep->is_rx)
				ep->napi->dev->stats.rx_fifo_errors++;
			else
				ep->napi->dev->stats.tx_fifo_errors++;
		}
	}

	return IRQ_HANDLED;
}

#if IPA_DBG
static int ipa_dump_state(struct seq_file *s, void *data)
{
	struct ipa *ipa = dev_get_drvdata(s->private);

	guard(pm_runtime_active)(ipa->dev);

	for (int i = 0; i < ipa->num_used_ep; i ++) {
		struct ipa_ep *ep = ipa->ep + i;
		int j = 0;
		struct fifo_desc desc;

		u32 dma_size = ep->dfifo_mask + 1;
		dma_size *= (ep->is_rx ? IPA_RX_UNIT : IPA_TX_UNIT);

		seq_printf(s, "\nendpoint: %s (id %d hwid %d)\n", ep->name,
				ep->id, ep->hwidx);
		seq_printf(s, "max_idx: %d\n", ep->fifo_mask);
		seq_printf(s, "max_data_idx: %d\n", ep->dfifo_mask);
		seq_printf(s, "Data FIFO addr: %llx\n", ep->dfifo_dma);
		seq_printf(s, "Data FIFO size: %x\n", dma_size);
		seq_printf(s, "BAM FIFO: ");
		for (j = 0; j <= ep->fifo_mask; j ++) {
			desc = ep->fifo[j];

			if (!(j & 3))
				seq_printf(s, "\n%4d:", j);

			seq_printf(s, "{%8x,%4x,%4x}, ", desc.addr, desc.size,
					desc.flags);
		}

		seq_printf(s, "\nhead: %d tail: %d\n",
				READ_ONCE(ep->head),
				READ_ONCE(ep->tail));
		seq_printf(s, "hwhead: %d hwtail: %d\n",
				ioread32(ep->hw_head),
				ioread32(ep->hw_tail));
#define SEQ_SHOW_EP_REG(reg) seq_printf(s, #reg ": %#x\n", \
		ioread32(ipa->mmio + reg(ep->hwidx)))

		SEQ_SHOW_EP_REG(REG_BAM_P_IRQ_STTS);
		SEQ_SHOW_EP_REG(REG_BAM_P_IRQ_EN);
		SEQ_SHOW_EP_REG(REG_BAM_P_CTRL);
		SEQ_SHOW_EP_REG(REG_IPA_EP_HOL_BLOCK_EN);
		SEQ_SHOW_EP_REG(REG_IPA_EP_CTRL);

		if (!ep->is_rx) {
			seq_printf(s, "txhead: %d txtail: %d enabled: %d\n",
					READ_ONCE(ep->tx_head),
					READ_ONCE(ep->tx_tail),
					READ_ONCE(ep->enabled));
			seq_printf(s, "tx_int_desc: %d tx_int_unit: %d\n",
					ep->tx_int_desc, ep->tx_int_unit);
			if (ep->napi && ep->napi->dev &&
			    netif_queue_stopped(ep->napi->dev))
				seq_printf(s, "queue stopped\n");
		} else {
			seq_printf(s, "rx_len: %d\n", ep->rx_len);
		}

		if (ep->napi) {
			typeof(ep->napi->state) val = READ_ONCE(ep->napi->state);
#define NAPI_SBIT(n) (val & NAPI_STATE_##n) ? (" " #n) : ""
			seq_printf(s, "napi_state: %s%s%s%s%s%s%s%s%s%s%s\n",
					NAPI_SBIT(SCHED), NAPI_SBIT(MISSED),
					NAPI_SBIT(DISABLE), NAPI_SBIT(NPSVC),
					NAPI_SBIT(LISTED), NAPI_SBIT(NO_BUSY_POLL),
					NAPI_SBIT(IN_BUSY_POLL), NAPI_SBIT(PREFER_BUSY_POLL),
					NAPI_SBIT(THREADED), NAPI_SBIT(SCHED_THREADED),
					NAPI_SBIT(HAS_NOTIFIER));
		}
	}

	return 0;
}
#endif

static inline void ipa_poll_done(struct napi_struct *napi,
		struct ipa_ep *ep, int done, int budget,
		u32 head, u32 hwhead, u32 irq_mask)
{
	const char *sts = "poll done";
	if (done < budget && napi_complete_done(napi, done)) {
		iowrite32(irq_mask, ep->ipa->mmio + REG_BAM_P_IRQ_CLR(ep->hwidx));

		if (ioread32(ep->hw_head) / 8 != head &&
		    napi_schedule_prep(napi)) {
			__napi_schedule(napi);
			sts = "poll resched";
			goto out;
		}

		iowrite32(irq_mask, ep->ipa->mmio + REG_BAM_P_IRQ_EN(ep->hwidx));
		sts = "poll done+irq";
	}

out:
	trace_ipa2_fifo_state(ep, sts);
}

static int ipa_poll_tx(struct napi_struct *napi, int budget)
{
	struct ipa_ndev *idev = container_of(napi, struct ipa_ndev, napi_tx);
	struct ipa_ep *ep = idev->tx;
	int done = 0, bytes = 0, packets = 0;
	u32 hwhead = readl_relaxed(ep->hw_head) / 8;
	u32 txhead = ep->tx_head;
	u32 head = ep->head;

	trace_ipa2_fifo_state(ep, "poll start");

	while (done < budget &&
	       head != READ_ONCE(ep->tail) && head != hwhead) {
		struct fifo_desc desc = ep->fifo[head];

		hwhead = readl_relaxed(ep->hw_head) / 8;

		trace_ipa2_desc(ep, false, head, desc, "poll_tx");

		head = FIFO_NEXT(ep->fifo_mask, head);

		u32 cnt = DIV_ROUND_UP(desc.size, IPA_TX_UNIT);
		u32 dest = DIV_ROUND_UP(desc.addr - ep->dfifo_dma, IPA_TX_UNIT);
		if (unlikely(dest != txhead))
			dev_err_ratelimited(ep->ipa->dev, "fifo corruption!\n");

		WRITE_ONCE(ep->head, head);

		txhead = FIFO_WRAP(ep->dfifo_mask, dest + cnt);
		bytes += desc.size;
		packets += !!(desc.flags & DESC_FLAG_EOT);

		WRITE_ONCE(ep->tx_head, txhead);
		done++;
	}

	napi->dev->stats.tx_packets += packets;
	napi->dev->stats.tx_bytes += bytes;

	if (READ_ONCE(ep->enabled) && netif_queue_stopped(napi->dev) &&
	    ipa_ep_check_tx_space(ep, napi->dev->mtu)) {
		trace_ipa2_event(ep, "start queue");
		netif_wake_queue(napi->dev);
	}

	IPA_STATS_ADD(idev->tx_poll_empty, done == 0);
	IPA_STATS_ADD(idev->tx_poll_full, done == budget);
	IPA_STATS_INC(idev->tx_poll_total);

	ipa_poll_done(napi, ep, done, budget, head, hwhead, P_TX_IRQ_MASK);

	return done;
}

static int ipa_poll_rx(struct napi_struct *napi, int budget)
{
	struct ipa_ndev *idev = container_of(napi, struct ipa_ndev, napi_rx);
	struct ipa_ep *ep = idev->rx;
	int done = 0;

	u32 hwhead = ep->fifo_mask & (readl_relaxed(ep->hw_head) / 8);
	u32 head = ep->head;
	u32 tail = ep->tail;
	u32 rx_len = ep->rx_len;
	u32 synchead = head;

	trace_ipa2_fifo_state(ep, "poll start");

	struct fifo_desc ndesc = { 0 };
	struct sk_buff *skb = NULL;
	ndesc.size = IPA_RX_UNIT;

	while (done < budget && head != tail && head != hwhead) {
		struct fifo_desc desc = ep->fifo[head];

		if (unlikely(synchead == head)) {
			u32 cnt = (budget - done);
			cnt = min(cnt, FIFO_DIST(ep->fifo_mask, synchead, hwhead));
			cnt = min(cnt, FIFO_DIST(ep->fifo_mask, synchead, tail));

			dma_sync_single_for_cpu(ep->ipa->dev,
				ep->dfifo_dma + IPA_RX_UNIT * synchead,
				IPA_RX_UNIT * min(cnt, (ep->fifo_mask + 1 - synchead)),
				DMA_FROM_DEVICE);

			synchead = FIFO_WRAP(ep->fifo_mask, synchead + cnt);
			if (synchead < head && synchead) {
				dma_sync_single_for_cpu(ep->ipa->dev,
					ep->dfifo_dma + 0, IPA_RX_UNIT * synchead,
					DMA_FROM_DEVICE);
			}
		}

#if IPA_DBG
		udelay(min(5000, idev->rx_delay_us));
#endif

		hwhead = readl_relaxed(ep->hw_head) / 8;

		done++;

		ndesc.addr = ep->dfifo_dma + IPA_RX_UNIT * head;

		trace_ipa2_desc(ep, false, head, desc, "poll_rx");
		trace_ipa2_desc(ep, true, head, ndesc, "poll_rx");
		ep->fifo[head] = ndesc;
		rx_len += min(IPA_RX_UNIT, desc.size);

		wmb();

		head = FIFO_NEXT(ep->fifo_mask, head);

		if (!(desc.flags & DESC_FLAG_EOT))
			continue;

		u32 rx_head = FIFO_NEXT(ep->fifo_mask, tail);

		if (likely(rx_len <= IPA_MAX_MTU))
			skb = napi_alloc_skb(napi, rx_len);

		tail = FIFO_WRAP(ep->fifo_mask, head - 1);
		if (likely(skb)) {
			void *next_src = ep->dfifo + rx_head * IPA_RX_UNIT;

			while (rx_len) {
				void *src = next_src;
				rx_head = FIFO_NEXT(ep->fifo_mask, rx_head);
				u32 len = min(rx_len, IPA_RX_UNIT);
				next_src = ep->dfifo + rx_head * IPA_RX_UNIT;
				__skb_put_data(skb, src, len);
				rx_len -= len;
			}

			if (unlikely(ep->is_sts)) {
				dev_kfree_skb_any(skb);
				napi->dev->stats.rx_dropped ++;
			} else {
				skb->protocol = htons(ETH_P_MAP);
				napi->dev->stats.rx_packets ++;
				napi->dev->stats.rx_bytes += skb->len;
				netif_receive_skb(skb);
			}
		} else {
			napi->dev->stats.rx_dropped++;
		}

		iowrite32(tail * 8, ep->hw_tail);
	}

	ep->head = head;
	ep->tail = tail;
	ep->rx_len = rx_len;

	IPA_STATS_ADD(idev->rx_poll_empty, done == 0);
	IPA_STATS_ADD(idev->rx_poll_full, done == budget);
	IPA_STATS_INC(idev->rx_poll_total);

	ipa_poll_done(napi, ep, done, budget, head, hwhead, P_RX_IRQ_MASK);

	return done;
}

static int ipa_ep_enable(struct ipa_ep *ep)
{
	int ret;

	if (!ep)
		return 0;

	ret = ipa_ep_manage_data_fifo(ep, true);
	if (ret)
		return ret;

	WRITE_ONCE(ep->enabled, true);

	ret = ipa_ep_reset_flush(ep, false);
	if (ret)
		goto err_disable;

	if (ep->is_rx) {
		struct fifo_desc desc = { 0 };
		desc.size = IPA_RX_UNIT;
		for (u32 idx; idx <= ep->fifo_mask; idx++) {
		       desc.addr = ep->dfifo_dma + idx * IPA_RX_UNIT;
		       ep->fifo[idx] = desc;
		}

		ep->tail = ep->fifo_mask;
		wmb();
		iowrite32(ep->tail * 8, ep->hw_tail);
		iowrite32(0, ep->ipa->mmio + REG_IPA_EP_HOL_BLOCK_EN(ep->hwidx));
	}

	iowrite32(0, ep->ipa->mmio + REG_IPA_EP_CTRL(ep->hwidx));
	napi_enable(ep->napi);
	iowrite32(ep->is_rx ? P_RX_IRQ_MASK : P_TX_IRQ_MASK,
		  ep->ipa->mmio + REG_BAM_P_IRQ_EN(ep->hwidx));
	return 0;

err_disable:
	ipa_ep_reset_flush(ep, true);
	return ret;
}

static void ipa_ep_disable(struct ipa_ep *ep)
{
	if (!ep)
		return;

	if (ep->is_rx) {
		iowrite32(1, ep->ipa->mmio + REG_IPA_EP_CTRL(ep->hwidx));
		iowrite32(1, ep->ipa->mmio + REG_IPA_EP_HOL_BLOCK_EN(ep->hwidx));
	}

	WRITE_ONCE(ep->enabled, false);

	if (!ep->is_rx) {
		trace_ipa2_event(ep, "stop queue");
		netif_stop_queue(ep->napi->dev);
	}

	iowrite32(1, ep->ipa->mmio + REG_IPA_EP_CTRL(ep->hwidx));
	iowrite32(0, ep->ipa->mmio + REG_BAM_P_IRQ_EN(ep->hwidx));

	napi_disable(ep->napi);

	msleep(50);

	ipa_ep_reset_flush(ep, true);
}

static inline int ipa_ep_prep_tx_desc(struct ipa_ep *ep, struct sk_buff *skb,
		u32 *tail, u32 *txtail, u32 *offset)
{
	struct ipa *ipa = ep->ipa;
	struct fifo_desc desc;
	void *dest;
	u32 prev_tail = *tail;
	int ret;

	if (likely(*offset >= skb->len))
		return 0;

	dest = ep->dfifo + IPA_TX_UNIT * (*txtail);

	desc.size = min(skb->len - *offset,
		       IPA_TX_UNIT * (ep->dfifo_mask + 1 - *txtail));
	desc.addr = ep->dfifo_dma + (dest - ep->dfifo);
	desc.flags = 0;
	if (*offset + desc.size == skb->len)
		desc.flags = DESC_FLAG_EOT;

	//dma_sync_single_for_cpu(ipa->dev, desc.addr,
			//ALIGN(desc.size, IPA_TX_UNIT), DMA_TO_DEVICE);

	ret = skb_copy_bits(skb, *offset, dest, desc.size);

	dma_sync_single_for_device(ipa->dev, desc.addr, desc.size,
				   DMA_TO_DEVICE);
	if (ret)
		return ret;

	*tail = FIFO_NEXT(ep->fifo_mask, *tail);
	*txtail = FIFO_WRAP(ep->dfifo_mask, *txtail +
			    DIV_ROUND_UP(desc.size, IPA_TX_UNIT));

	/*
	 * When TX is idle (hwhead == hwtail) P_OUT_OF_DESC_EN will trigger
	 * tx napi. Otherwise we periodically set FLAG_INT to not get out
	 * of descriptors or data buffer space because IRQ/poll on every
	 * finished transfer is not efficient.
	 */
	if (desc.flags & DESC_FLAG_EOT &&
	    (*tail - ep->tx_int_desc > ep->fifo_mask / 2 ||
	     *txtail - ep->tx_int_unit > ep->dfifo_mask / 2)) {
		desc.flags |= DESC_FLAG_INT;
		ep->tx_int_desc = *tail;
		ep->tx_int_unit = *txtail;
	}

	*offset += desc.size;

	trace_ipa2_desc(ep, true, prev_tail, desc, "tx");

	ep->fifo[prev_tail] = desc;
	return 0;
}

static int ipa_ndev_open(struct net_device *ndev)
{
	struct ipa_ndev *idev = netdev_priv(ndev);
	struct ipa *ipa = idev->rx->ipa;

	scoped_guard(pm_runtime_active_try, ipa->dev) {
		icc_bulk_set_bw(3, ipa->icc_bulk_data);
		clk_set_rate(ipa->clk, 1000 *
				clamp(min_rate_khz, 1000, 200000));

		int ret = ipa_ep_enable(idev->rx);
		if (ret)
			return ret;

		ret = ipa_ep_enable(idev->tx);
		if (ret) {
			ipa_ep_disable(idev->rx);
			return ret;
		}

		netif_start_queue(ndev);
		pm_runtime_get_noresume(ipa->dev);
		return 0;
	}

	return -ESHUTDOWN;
}

static int ipa_ndev_stop(struct net_device *ndev)
{
	struct ipa_ndev *idev = netdev_priv(ndev);
	struct ipa *ipa = idev->rx->ipa;

	if (idev->tx)
		trace_ipa2_event(idev->tx, "stop queue");
	netif_stop_queue(ndev);
	ipa_ep_disable(idev->tx);
	ipa_ep_disable(idev->rx);

	pm_runtime_put(ipa->dev);
	return 0;
}

static void ipa_ndev_suspend_resume(struct net_device *ndev, bool resume)
{
	if (!ndev || !netif_running(ndev))
		return;

	if (resume)
		ipa_ndev_open(ndev);
	else
		ipa_ndev_stop(ndev);
}

static netdev_tx_t ipa_ndev_start_xmit(struct sk_buff *skb,
				       struct net_device *ndev)
{
	struct ipa_ndev *idev = netdev_priv(ndev);
	struct ipa_ep *ep = idev->tx;
	u32 len = skb->len;

	if (!ep || skb->protocol != htons(ETH_P_MAP) || len > ndev->mtu)
		goto drop_tx;

	/* Shouldn't happen unless mtu changes */
	if (unlikely(!ipa_ep_check_tx_space(ep, len)))
		goto drop_stop_tx;

	u32 tail = ep->tail;
	u32 txtail = ep->tx_tail;
	u32 off = 0;

	if (ipa_ep_prep_tx_desc(ep, skb, &tail, &txtail, &off) ||
	    ipa_ep_prep_tx_desc(ep, skb, &tail, &txtail, &off))
		goto drop_tx;

	WRITE_ONCE(ep->tx_tail, txtail);
	WRITE_ONCE(ep->tail, tail);
	wmb();
	iowrite32(tail * 8, ep->hw_tail);

	dev_consume_skb_any(skb);
	skb = NULL;


	if (ipa_ep_check_tx_space(ep, ndev->mtu))
		return NETDEV_TX_OK;

drop_stop_tx:
	trace_ipa2_event(ep, "stop queue");
	netif_stop_queue(ndev);

drop_tx:
	if (skb) {
		trace_ipa2_event(ep, "drop pkt");
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
	}

	return NETDEV_TX_OK;
}

static const struct net_device_ops ipa_ndev_ops = {
	.ndo_open = ipa_ndev_open,
	.ndo_stop = ipa_ndev_stop,
	.ndo_start_xmit = ipa_ndev_start_xmit,
};

static void ipa_ndev_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &ipa_ndev_ops;
	ndev->addr_len = 0;
	ndev->hard_header_len = 0;
	ndev->min_header_len = ETH_HLEN;
	ndev->needed_headroom = 4; /* QMAP_HDR */
	ndev->max_mtu = ndev->mtu = IPA_MAX_MTU;
	ndev->needed_tailroom = 0;
	ndev->priv_flags |= IFF_TX_SKB_SHARING;
	ndev->tx_queue_len = 1000;
	ndev->type = ARPHRD_RAWIP;
	ndev->watchdog_timeo = 1000;
	eth_broadcast_addr(ndev->broadcast);
}

static void ipa_remove_netdev(void *data)
{
	struct net_device *ndev = data;
	struct ipa_ndev *idev = netdev_priv(ndev);

	ipa_dbg("");

#if IPA_DBG
	if (!IS_ERR_OR_NULL(idev->dfs))
		debugfs_remove_recursive(idev->dfs);
#endif

	unregister_netdev(ndev);

	netif_napi_del(idev->rx->napi);
	if (idev->tx)
		netif_napi_del(idev->tx->napi);

	free_netdev(ndev);
	ipa_dbg("done");
}

static struct net_device *
ipa_create_netdev(struct device *dev, const char *name,
			     struct ipa_ep *rx, struct ipa_ep *tx)
{
	struct ipa_ndev *idev;
	struct net_device *ndev;
	int ret;

	ndev = alloc_netdev(sizeof(*idev), name,
			NET_NAME_UNKNOWN, ipa_ndev_setup);
	if (IS_ERR_OR_NULL(ndev))
		return ERR_PTR(-ENOMEM);

	if (rx->id == EP_RX)
		SET_NETDEV_DEV(ndev, dev);

	idev = netdev_priv(ndev);
	idev->rx = rx;
	idev->tx = tx;

	rx->idev = idev;
	rx->napi = &idev->napi_rx;

	netif_napi_add(ndev, rx->napi, ipa_poll_rx);
	if (tx) {
		tx->idev = idev;
		tx->napi = &idev->napi_tx;
		netif_napi_add_tx(ndev, tx->napi, ipa_poll_tx);
	}

	ret = register_netdev(ndev);
	if (ret) {
		netif_napi_del(rx->napi);
		if (tx)
			netif_napi_del(tx->napi);
		free_netdev(ndev);
		return ERR_PTR(ret);
	}

#if IPA_DBG
	struct ipa *ipa = dev_get_drvdata(dev);

#define DBG_FIELD_RW(d, type, s, f) \
	debugfs_create_##type(#f, 0644, d, &s->f)

#define DBG_FIELD_NAMED(d, type, s, f, name) \
	debugfs_create_##type(#name, 0444, d, &s->f)

#define DBG_FIELD(d, type, s, f) \
	debugfs_create_##type(#f, 0444, d, &s->f)

	idev->dfs = debugfs_create_dir(netdev_name(ndev), ipa->dfs);
	if (!IS_ERR_OR_NULL(idev->dfs)) {
		DBG_FIELD_RW(idev->dfs, u32, idev, rx_delay_us);
#if IPA_STATS
		DBG_FIELD(idev->dfs, u64, idev, rx_poll_total);
		DBG_FIELD(idev->dfs, u64, idev, rx_poll_empty);
		DBG_FIELD(idev->dfs, u64, idev, rx_poll_full);
		DBG_FIELD(idev->dfs, u64, idev, tx_poll_total);
		DBG_FIELD(idev->dfs, u64, idev, tx_poll_empty);
		DBG_FIELD(idev->dfs, u64, idev, tx_poll_full);

		if (idev->tx)
			DBG_FIELD_NAMED(idev->dfs, u32, idev->tx, irq_cnt, tx_irqs);

		DBG_FIELD_NAMED(idev->dfs, u32, idev->rx, irq_cnt, rx_irqs);

		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), rx_dropped);
		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), rx_fifo_errors);
		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), rx_packets);
		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), rx_bytes);

		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), tx_dropped);
		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), tx_fifo_errors);
		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), tx_packets);
		DBG_FIELD(idev->dfs, ulong, (&ndev->stats), tx_bytes);

#endif
	}
#endif

	ret = devm_add_action_or_reset(dev, ipa_remove_netdev, ndev);

	return ret ? ERR_PTR(ret) : ndev;
}


void ipa_modem_set_present(struct device *dev, bool present)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	(present ? netif_device_attach : netif_device_detach) (ipa->modem);
}

static void ipa_shutdown_modem(struct ipa *ipa)
{
	struct device_node *np;

	/* Shutting down IPA/driver while modem is up is asking for trouble */
	np = of_parse_phandle(ipa->dev->of_node, "modem-remoteproc", 0);
	if (np)
		return;

	struct rproc *rproc = rproc_get_by_phandle(np->phandle);
	of_node_put(np);

	if (rproc) {
		if (rproc->state == RPROC_RUNNING)
			rproc_shutdown(rproc);

		rproc_put(rproc);
	}
}

static void ipa_reset_action(void *ipa_ptr)
{
	struct ipa *ipa = ipa_ptr;
	int id;

	pm_runtime_get_sync(ipa->dev);

	if (!IS_ERR_OR_NULL(ipa->ssr_cookie))
		qcom_unregister_ssr_notifier(ipa->ssr_cookie, &ipa->ssr_nb);

	if (ipa->qmi)
		ipa_shutdown_modem(ipa);

	iowrite32(0, ipa->mmio + REG_IPA_IRQ_CLR_EE0);
	iowrite32(0, ipa->mmio + REG_IPA_IRQ_EN_EE0);

	scoped_guard(mutex, &ipa->lock) {
		if (pm_runtime_enabled(ipa->dev))
			pm_runtime_disable(ipa->dev);
	}

	pm_runtime_put_noidle(ipa->dev);

#if IPA_DBG
	if (!IS_ERR_OR_NULL(ipa->dfs))
		debugfs_remove_recursive(ipa->dfs);
#endif

	for (id = 0; id < ipa->num_used_ep; id++)
		ipa_ep_reset_pipe(&ipa->ep[id], true);

	ipa_reset_modem_pipes(ipa);
	ipa_uc_cmd(ipa, IPA_UC_CMD_CLK_GATE, 0, 0);

	icc_bulk_disable(3, ipa->icc_bulk_data);
}

static int ipa_init_cdma(struct ipa *ipa, bool allocated)
{
	u32 id, offset;

	/* Ensure cmda_addr + offset is aligned */
	offset = allocated ? ipa->cdma_addr % 8 : 8;

	for (id = 0; id < ipa->num_used_ep; id++) {
		ipa->ep[id].fifo = ipa->cdma_virt + offset;

		ipa_dbg("ep %d offset %#x", id, offset);
		offset += (ipa->ep[id].fifo_mask + 1) * 8;
	}

	ipa_dbg("zero_rule offset %#x", offset);
	ipa->zero_rule = ipa->cdma_addr + offset;
	offset += IPA_ZERO_RULE_SIZE;

	ipa_dbg("test_rules offset %#x", offset);
	ipa->test_rules = ipa->cdma_addr + offset;
	offset += IPA_TEST_RULES_SIZE;

	ipa_dbg("sys_hdr offset %#x", offset);
	ipa->sys_hdr = ipa->cdma_addr + offset;
	offset += IPA_SYS_HDR_SIZE;

	ipa_dbg("scratch offset %#x", offset);
	/* MDM_HDR needs the most (80 * 4 + size of ipa_cmd) */
	ipa->scratch_size = allocated ? ipa->cdma_size - offset : 400;
	ipa->scratch = ipa->cdma_addr + offset;

	if (allocated)
		return 0;

	ipa->cdma_size = ALIGN(offset + ipa->scratch_size, 0x1000);

	ipa_dbg("allocating %#x bytes", ipa->cdma_size);

	ipa->cdma_virt = dmam_alloc_coherent(ipa->dev,
			ipa->cdma_size, &ipa->cdma_addr, GFP_KERNEL);
	if (!ipa->cdma_virt)
		return -ENOMEM;

	memset(ipa->cdma_virt, 0, ipa->cdma_size);

	return ipa_init_cdma(ipa, true);
}

static int ipa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipa *ipa;
	int id, ret, dma_irq, ipa_irq;

	ipa = devm_kzalloc(dev, sizeof(*ipa), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ipa))
		return -ENOMEM;

	mutex_init(&ipa->lock);
	init_waitqueue_head(&ipa->cmd_wq);
	platform_set_drvdata(pdev, ipa);
	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	ipa->version = (long)of_device_get_match_data(dev);
	ipa->dev = dev;
	ipa->ssr_nb.notifier_call = ipa_ssr_notifier;
	ipa->num_used_ep = EP_MAX_NUM - (enable_testpipe ? 0 : 4);
	ipa->mmio = devm_platform_ioremap_resource(pdev, 0);

	ipa->icc_bulk_data[0].name = "ipa-mem";
	ipa->icc_bulk_data[0].avg_bw = Mbps_to_icc(average_bw_mbits);
	ipa->icc_bulk_data[0].peak_bw = Mbps_to_icc(peak_bw_mbits);
	ipa->icc_bulk_data[1].name = "ipa-imem";
	ipa->icc_bulk_data[1].avg_bw = MBps_to_icc(10);
	ipa->icc_bulk_data[1].peak_bw = MBps_to_icc(100);
	ipa->icc_bulk_data[2].name = "cpu-cfg";
	ipa->icc_bulk_data[2].avg_bw = MBps_to_icc(10);
	ipa->icc_bulk_data[2].peak_bw = MBps_to_icc(150);

	WARN_ON(dma_get_cache_alignment() > IPA_TX_UNIT);

	if (IS_ERR_OR_NULL(ipa->mmio))
		return PTR_ERR(ipa->mmio) ?: -ENOMEM;

	for (id = 0; id < ipa->num_used_ep; id++) {
		struct ipa_ep *ep = &ipa->ep[id];

		ep->ipa = ipa;
		ep->is_rx = EP_ID_IS_RX(id);

		u32 fifo_sz_order = ep->is_rx ? rxfifo_sz_order : txfifo_sz_order;
		if (id == EP_LAN_RX || id == EP_CMD)
			fifo_sz_order = 4;

		ep->fifo_mask = BIT(clamp(fifo_sz_order, 3, 12)) - 1;
		if (ep->is_rx) {
			ep->dfifo_mask = ep->fifo_mask;
		} else {
			ep->dfifo_mask = 1 << clamp(txbuf_sz_kb_order + 10, 12, 20);
			ep->dfifo_mask = ep->dfifo_mask / IPA_TX_UNIT - 1;
		}

		ep->name = ipa_ep_names[id];
		ep->id = id;
		ep->hwidx = ipa_2p5_pipe_map[id];
		ep->hw_head = ipa->mmio + REG_BAM_P_RD_OFF_REG(ep->hwidx);
		ep->hw_tail = ipa->mmio + REG_BAM_P_WR_OFF_REG(ep->hwidx);
	}

	ret = ipa_init_cdma(ipa, false);
	if (ret)
		return ret;

	ret = devm_of_icc_bulk_get(dev, 3, ipa->icc_bulk_data);
	if (ret)
		return ret;

	ipa->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(ipa->clk))
		return dev_err_probe(dev, PTR_ERR(ipa->clk),
				     "failed to get clock\n");

	clk_set_rate(ipa->clk, clamp(min_rate_khz, 4, 200) * 1000);

	ipa_reset_hw(ipa);

	ret = ipa_partition_mem(ipa);
	if (ret)
		return ret;

	if (enable_testpipe) {
		void *payload = ipa_cdma_to_virt(ipa, ipa->test_rules);

		ipa_write_flt_rule(payload, TX2_TO_RT1_OFF,
				ipa->layout[MEM_RT_V4].size / 4 + 1);
		ipa_write_flt_rule(payload, TX1_TO_RT2_OFF,
				ipa->layout[MEM_RT_V4].size / 4 + 2);
		ipa_write_rt_rule(payload, RT1_TO_RX1_OFF,
				ipa->ep[EP_TEST_RX1].hwidx);
		ipa_write_rt_rule(payload, RT2_TO_RX2_OFF,
				ipa->ep[EP_TEST_RX2].hwidx);
	}

	for (id = 0; id < ipa->num_used_ep; id++) {
		ret = ipa_ep_reset_pipe(&ipa->ep[id], true);
		if (ret && id == EP_CMD)
			return ret;

		if (!ret)
			ipa_setup_ep(ipa, &ipa->ep[id]);
	}

	iowrite32(FIELD_PREP(IPA_ROUTE_DEF_PIPE_BMSK, 2) |
		  FIELD_PREP(IPA_ROUTE_FRAG_DEF_PIPE_BMSK, 2) |
		  FIELD_PREP(IPA_ROUTE_DEF_HDR_TABLE_BMSK, 1),
		  ipa->mmio + REG_IPA_ROUTE_OFST);

	ret = ipa_uc_cmd(ipa, IPA_UC_CMD_CLK_UNGATE, 0, 0);
	if (ret)
		return ret;

	ipa_irq = of_irq_get_byname(dev->of_node, "ipa");
	dma_irq = of_irq_get_byname(dev->of_node, "dma");
	if (ipa_irq < 0 || dma_irq < 0)
		return min(ipa_irq, dma_irq);

	ret = devm_request_threaded_irq(dev, ipa_irq, NULL, ipa_isr_thread,
					IRQF_ONESHOT, "ipa", ipa);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, dma_irq, ipa_dma_isr, 0, "ipa-dma", ipa);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);

	ret = ipa_ep_reset_pipe(&ipa->ep[EP_CMD], false);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ipa_reset_action, ipa);
	if (ret)
		return ret;

	ret = ipa_init_sram(ipa);
	if (ret)
		return ret;

	iowrite32(BIT(IPA_IRQ_UC_IRQ_1), ipa->mmio + REG_IPA_IRQ_CLR_EE0);
	iowrite32(BIT(IPA_IRQ_UC_IRQ_1), ipa->mmio + REG_IPA_IRQ_EN_EE0);

	ipa->ssr_cookie = qcom_register_ssr_notifier("mpss", &ipa->ssr_nb);
	if (IS_ERR(ipa->ssr_cookie))
		return dev_err_probe(dev, PTR_ERR(ipa->ssr_cookie),
				     "failed to register SSR notifier\n");

#if IPA_DBG
	ipa->dfs = debugfs_create_dir("ipa2_lite", NULL);

	debugfs_create_devm_seqfile(ipa->dev, "state",
			ipa->dfs, ipa_dump_state);
#endif

	if (enable_testpipe) {
		ipa->tp0 = ipa_create_netdev(dev, "ipa_testpipe%d",
				ipa->ep + EP_TEST_RX1, ipa->ep + EP_TEST_TX1);
		if (IS_ERR(ipa->tp0))
			return PTR_ERR(ipa->tp0);

		ipa->tp1 = ipa_create_netdev(dev, "ipa_testpipe%d",
				ipa->ep + EP_TEST_RX2, ipa->ep + EP_TEST_TX2);
		if (IS_ERR(ipa->tp1))
			return PTR_ERR(ipa->tp1);
	}

	ipa->modem = ipa_create_netdev(dev, "rmnet_ipa%d",
			ipa->ep + EP_RX, ipa->ep + EP_TX);
	if (IS_ERR(ipa->modem))
		return PTR_ERR(ipa->modem);

	ipa->lan = ipa_create_netdev(dev, "ipa_lan%d", ipa->ep + EP_LAN_RX, NULL);
	if (IS_ERR(ipa->lan))
		return PTR_ERR(ipa->lan);

	ipa_modem_set_present(dev, false);

	ipa_uc_loaded_update(ipa, false);

	ipa->qmi = ipa_qmi_setup(ipa->dev, ipa->layout, ipa->uc_loaded);
	if (IS_ERR(ipa->qmi))
		return PTR_ERR(ipa->qmi);

	/* Just in case we got early IRQ before ipa->qmi is set */
	ipa_uc_loaded_update(ipa, false);
	return 0;
}

static void ipa_remove(struct platform_device *pdev)
{
	/* We cannot fully reset ipa once modem did its part */
	dev_crit(&pdev->dev, "reloading this module isn't safe");
	dev_crit(&pdev->dev, "reboot device or expect problems");
}

static int ipa_runtime_resume(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);
	int ret = 0;

	ret = icc_bulk_enable(3, ipa->icc_bulk_data);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ipa->clk);
	if (ret)
		goto fail_icc;

	ret = ipa_uc_cmd(ipa, IPA_UC_CMD_CLK_UNGATE, 0, 0);
	if (!ret)
		return 0;

	clk_disable_unprepare(ipa->clk);
fail_icc:
	icc_bulk_disable(3, ipa->icc_bulk_data);
	return ret;
}

static int ipa_runtime_suspend(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	if (ipa_uc_cmd(ipa, IPA_UC_CMD_CLK_GATE, 0, 0))
		dev_err(dev, "failed to send clock gating notification");

	clk_disable_unprepare(ipa->clk);
	icc_bulk_disable(3, ipa->icc_bulk_data);
	return 0;
}

static int ipa_system_resume(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	ipa_ndev_suspend_resume(ipa->modem, true);
	ipa_ndev_suspend_resume(ipa->lan, true);

	if (ipa->tp0) {
		ipa_ndev_suspend_resume(ipa->tp0, true);
		ipa_ndev_suspend_resume(ipa->tp1, true);
	}

	return 0;
}

static int ipa_system_suspend(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	ipa_ndev_suspend_resume(ipa->modem, false);
	ipa_ndev_suspend_resume(ipa->lan, false);

	if (ipa->tp0) {
		ipa_ndev_suspend_resume(ipa->tp0, false);
		ipa_ndev_suspend_resume(ipa->tp1, false);
	}

	return 0;
}

static struct attribute *ipa_modem_attrs[] = {
	&dev_attr_rx_endpoint_id.attr.attr,
	&dev_attr_tx_endpoint_id.attr.attr,
	NULL
};

const struct attribute_group ipa_modem_group = {
	.name		= "modem",
	.attrs		= ipa_modem_attrs,
};

const struct attribute_group *ipa_groups[] = {
	&ipa_modem_group,
	NULL
};

static const struct of_device_id ipa_match[] = {
	{ .compatible	= "qcom,ipa-v2.5", (void *)25 },
	{ .compatible	= "qcom,ipa-lite-v2.6", (void *)26 },
	{ },
};

static const struct dev_pm_ops ipa_pm = {
	SET_RUNTIME_PM_OPS(ipa_runtime_suspend, ipa_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(ipa_system_suspend, ipa_system_resume)
};

static struct platform_driver ipa2_lite_driver = {
	.probe		= ipa_probe,
	.remove		= ipa_remove,
	.driver	= {
		.name		= "ipa",
		.dev_groups	= ipa_groups,
		.of_match_table	= ipa_match,
		.pm		= &ipa_pm
	},
};

module_platform_driver(ipa2_lite_driver);

MODULE_DEVICE_TABLE(of, ipa_match);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm IP Accelerator v2.X driver");
