// SPDX-License-Identifier: GPL-2.0-only
//
#ifndef _IPA_H_
#define _IPA_H_

#include "ipa-hw.h"

enum ipa_ep_id {
	EP_TX,
	EP_RX,
	EP_CMD,
	EP_LAN_RX,
	EP_TEST_TX1,
	EP_TEST_RX1,
	EP_TEST_TX2,
	EP_TEST_RX2,
	EP_MAX_NUM,
};

#define EP_ID_IS_RX(id) (id & 1)

enum ipa_part_id {
	MEM_UC_INFO,
	MEM_FT_V4,
	MEM_FT_V6,
	MEM_RT_V4,
	MEM_RT_AP_V4,
	MEM_RT_V6,
	MEM_RT_AP_V6,
	MEM_MDM_HDR,
	MEM_MDM_COMP,
	MEM_MDM_HDR_PCTX,
	MEM_MDM,
	MEM_PART_NUM,
};

union ipa_cmd {
	struct ipa_hw_imm_cmd_dma_shared_mem dma_smem;
	struct ipa_ip_packet_init ip_pkt_init;
	struct ipa_ip_v4_rule_init rule_v4_init;
	struct ipa_ip_v6_rule_init rule_v6_init;
	struct ipa_hdr_init_local hdr_local_init;
	struct ipa_hdr_init_system hdr_system_init;
};

struct ipa_ndev;
struct ipa;
struct napi_struct;
struct ipa_ep {
	struct ipa *ipa;
	struct ipa_ndev *idev;
	struct napi_struct *napi;
	void __iomem *hw_head, *hw_tail;
	const char *name;

	/* descriptor FIFO */
	struct fifo_desc *fifo;
	u32 head, tail;
	u32 fifo_mask;

	/* Data FIFO */
	dma_addr_t dfifo_dma;
	void *dfifo;
	u32 dfifo_mask;
	u32 tx_head, tx_tail;
	u32 tx_int_desc, tx_int_unit;
	u32 rx_len;

	u32 irq_cnt;
	u8 id, hwidx;
	bool is_sts, is_rx;
	bool enabled;
};

struct ipa_partition {
	u16 offset, size;
};

struct device;
struct ipa_qmi;

struct ipa_qmi *ipa_qmi_setup(struct device *dev, const struct ipa_partition *layout,
		bool loaded);
bool ipa_qmi_is_modem_ready(struct ipa_qmi *ipa_qmi);
void ipa_qmi_uc_loaded(struct ipa_qmi *ipa_qmi);
void ipa_modem_set_present(struct device *dev, bool present);
#endif
