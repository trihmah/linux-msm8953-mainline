// SPDX-License-Identifier: GPL-2.0
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/cleanup.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <uapi/linux/v4l2-subdev.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-rect.h>

#include "ccs/ccs-regs.h"
#include "ccs/smiapp-reg-defs.h"

#define S5K_DEFAULT_FPS			(30)
#define S5K_DEFAULT_MCLK		(24 * 1000 * 1000)
#define S5K_LLP_MAX			(0xfff8u)
#define S5K_FLL_MAX			(0xfffeu)
#define S5K_FLP_MAX			(S5K_FLL_MAX * S5K_LLP_MAX)
#define S5K_NUM_LANES			(4)
#define S5K_MAX_BINNING			(4)
#define S5K_DIG_GAIN_STEP		(256)
#define S5K_PAD_SRC			(0)

#define S5K_LLP_STEP			(8)
#define S5K_FLL_STEP(info)		((info)->tetracell + 1)
#define S5K_LLP_ALIGN(val)		ALIGN(val, S5K_LLP_STEP)
#define S5K_FLL_ALIGN(info, val)	ALIGN(val, S5K_FLL_STEP(info))

#define MIN_OUTPUT_WH			(128)

/* Some vendor registers */
#define S5K_R_SYSTEM_ERROR		CCI_REG8(0x3000)
#define S5K_R_POST_PLL_CLK_DIV		CCI_REG16(0x300a)
#define S5K_R_OP_POST_PLL_CLK_DIV	CCI_REG16(0x300c)
#define S5K_R_DIG_HORIZ_SCALE		CCI_REG8(0x304f)
/* Pre-scaling offsets */
#define S5K_R_DIG_X_PRE_OFFSET		CCI_REG16(0xbc2)
#define S5K_R_DIG_Y_PRE_OFFSET		CCI_REG16(0xbc4)

/* Hack for tetracell format */
#define MEDIA_BUS_FMT_SGGRR10_1X10	MEDIA_BUS_FMT_Y10_1X10

#define sd_to_ctx(_sd) \
	container_of(_sd, struct s5k_ctx, sd)

#define dev_to_ctx(_dev) \
	sd_to_ctx(dev_get_drvdata(_dev))

#define sdbg(fmt, ...) \
	{ \
		if (sd_debug) \
			dev_info(ctx->dev, "%s: " fmt "\n", __func__, ##__VA_ARGS__); \
	}

static bool sd_debug = false;
module_param(sd_debug, bool, 0644);

#define WHICH_STR(val) (val ? "'ACTIVE'" : "'TRY'")
#define S5K_DEBUG_IFACE 0
#ifdef S5K_DEBUG_IFACE
struct s5k_ctx;
struct s5k_init_seq_file {
	struct s5k_ctx *ctx;
	char *cur;
	char *next;
	size_t next_size;
	size_t next_allocated;
};

static void s5k_debug_write_seq(struct s5k_ctx *ctx, struct s5k_init_seq_file *isf);
#endif

static const char * const s5k_test_pattern_menu[] = {
	"Disabled", "Solid Colour", "100% Colour Bars",
	"Fade To Grey Colour Bars", "PN9",
};

struct s5k_clk_cfg {
	u32 inp;
	u16 mult;
	u16 pre_div  :4;
	u16 post_div :4; /* Optional */
	u16 sys_div  :4;
	u16 pix_div  :4;
};

struct s5k_info {
	const char *model;
	u16 chip_id;
	u16 unit_size_nm;
	u16 monochrome	    :1;
	u16 tetracell	    :1;
	u16 has_postdiv	    :1;
	u16 has_dig_horiz2  :1;
	u16 op_clk_div	    :2;
	u16 support_bpp8    :1;
	u16 max_digital_gain :8;
	/* Crop size override only for broken sensors */
	u16 min_crop_w;
	u32 min_exposure :4;
	u32 max_exposure_margin : 4;

	/* Minimal LLP may vary depending on horizontal binning factor,
	 * pixel rate and bpp. Should be 0 if horizontal binning factor
	 * and bpp combination isn't supported.
	 * This is the lowest value sensor allows regardless of link_freq.
	 * (shouldn't be adjusted, use fifo_margin_bytes instead)
	 * Tested with smallest crop->width.
	 */
	u16 min_llp[S5K_MAX_BINNING];
	u16 min_llp_8b[S5K_MAX_BINNING];

	/* This is meaningless (and can't be tested) unless for some mode
	 * crop->w / bin_h + min_hbl is greater than min_llp
	 * Seems to be reported right by SMIA register (MIN_LINE_BLANKING_PCK).
	 */
	u16 min_hbl;

	/* Minimal vblank may vary depending on vertical binning
	 * factor, llp, pixel rate and color depth. Thus it should be
	 * tested at max pixel rate and minimal llp for each binning factor
	 * and bpp. Should be 0 if binning factor and bpp combination isn't
	 * supported.
	 */
	u8 min_vbl[S5K_MAX_BINNING];
	u8 min_vbl_8b[S5K_MAX_BINNING];

	/* This (made-up) parameter defines relation between CSI link rate
	 * and min_llp. It comes out from following:
	 *
	 * mipi_data_bitrate = N_CSI_LANES * 2 * link_freq
	 * bytes_per_line = bpp * format->width
	 * bytes_per_line_adj = bpp * format->width + 8 * fifo_margin_bytes
	 * time_per_line = llp / pclk
	 * average_bitrate = bytes_per_line / time_per_line
	 * average_bitrate_adj = bytes_per_line_adj / time_per_line
	 *
	 * Driver tries to satisfy mipi_data_bitrate > average_bitrate_adj
	 */
	u16 fifo_margin_bytes;

	/* Supported down-scaling ratios (combined H+V) in step of 0.5x:
	 * BIT(0) - 1.0x, BIT(1) - 1.5x, BIT(2) - 2.0x
	 * S5K*** sensor have 3 ways of scaling image:
	 * binning - (separate Horiz/Vertical, only integer ratios 1-4)
	 * scale_m (SMIA) - united H+V or just horizontal, 1-4x with step
	 * of 0.5x.
	 * dig_horiz (vendor) - additional horizontal. Might not be supported.
	 * Also doesn't work if digital_m already used in horizontal-only mode.
	 * Range is 1.0 - 2.0x with step of 0.25x
	 *
	 * Driver has a bias toward digital scaling, so 3.0x is going to be
	 * 1.0x binning + 3.0 digital instead of 2.0x binning + 1.5 digital.
	 */
	u32 scaling_mask;
	/* One time init delay clock configuration (after power off)*/
	u16 init_delay_us;

	const struct s5k_clk_cfg *op_cfgs;
	const struct s5k_clk_cfg *vt_cfgs;
};

/*
 * Extra state on top of subdev_state for try_* helpers.
 */

struct s5k_state {
	const struct s5k_clk_cfg *op, *vt;
	u64 vt_rate, op_rate;
	u16 ratio : 8;
	u16 binv : 3;
	u16 binh : 3;
	u16 tbin : 1;
	u16 fll_min, llp_min;
	/* Active values */
	u16 fll, llp;
};

struct s5k_ctx {
	const struct s5k_info *info;
	struct clk *mclk;
	struct device *dev;
	struct gpio_desc *rst;
	struct regmap *rmap;
	struct regulator_bulk_data *vregs;
	int num_vregs;

	struct media_pad pad;
	struct v4l2_area unit_size;
	struct v4l2_ctrl_handler chdl;
	struct v4l2_rect min_crop, max_crop;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *vbl, *hbl, *exp;
	struct v4l2_ctrl *prate, *lfreq;

	struct s5k_state cfg;
	struct mutex lock;

	u32 mclk_rate, mclk_actual;
	u32 min_lfreq, max_lfreq;

	bool uninited;
	bool clks_changed;
	bool mode_changed;
	bool interval_locked;

	u32 dbg_init_delay_us;

#ifdef S5K_DEBUG_IFACE
	u8 force_binh, force_binv, force_ratio;
	struct dentry *dfs_dir;
	u16 dbg_reg, dbg_reg_cnt;
	struct s5k_init_seq_file dbg_pre_init, dbg_pre_stream;
#endif
	int n_link_freqs;
	s64 lfreq_menu[4];
};

#define SCL_RANGE_MASK(smin, smax, step) \
	(GENMASK((u16) (smax * 2) - 2, (u16) (smin * 2) - 2) & (step * 0x11111111u))
#define SCL_STEP_0X5 (15u)
#define SCL_STEP_1X0 (5u)
#define SCL_STEP_2X0 (4u)

static const struct s5k_info s5k2p6_sensor_info = {
	/* 16Mpix (4656x3504) @ 30FPS */
	.model = "s5k2p6",
	.chip_id = 0x2106,
	.unit_size_nm = 1120,
	.support_bpp8 = true,
	.has_dig_horiz2 = true,
	.min_llp = { 4464, 4464 },
	.min_vbl = { 89, 73, 0, 65 },
	.min_hbl = 184,
	.min_llp_8b = { 4464, 4464 },
	.min_vbl_8b = { 89, 73, 0, 65 },
	.max_digital_gain = 64,
	.min_exposure = 5,
	.max_exposure_margin = 4,
	.fifo_margin_bytes = 340,
	.init_delay_us = 1000,
	.scaling_mask = SCL_RANGE_MASK(1.0, 4.0, SCL_STEP_0X5)
		      | SCL_RANGE_MASK(4.0, 8.0, SCL_STEP_1X0),

	.op_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 744MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 4,
			.mult = 124,
			.sys_div = 1,
			.pix_div = 8,
		},
		{ 0 },
	},

	.vt_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 560Mpix/s */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 6,
			/* Other (downstream) drivers use mult 105 and pix_div 3
			 * like on s5k3p8 but min_llp in 8-bit mode becomes too
			 * high for 30FPS (6144) compared to 5120 in 10-bit.
			 * Underclocking PLL and sys_clk helps to reduce it for
			 * both modes.
			 */
			.mult = 70,
			.sys_div = 1,
			.pix_div = 2,
		},
		{ 0 },
	},
};

static const struct s5k_info s5k3l8_sensor_info = {
	/* Config based on init_seqs */
	/* Real capabilities are not tested */
	.model = "s5k3l8",
	.chip_id = 0x30c8,
	.unit_size_nm = 1120,
	.op_clk_div = 2,
	.min_hbl = 184,
	.min_llp = { 5808, 0, 5808 },
	.min_vbl = { 130, 0, 92 },
	.max_digital_gain = 64,
	.min_exposure = 8,
	.max_exposure_margin = 8,
	.fifo_margin_bytes = 480,
	.init_delay_us = 1000,
	/* This assumes only vertical binning of 3 is supported */
	.scaling_mask = SCL_RANGE_MASK(1.0, 4.0, SCL_STEP_0X5),
	.vt_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 566.4Mpix/s */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 6,
			.mult = 177,
			.sys_div = 1,
			.pix_div = 5,
		},
		{ 0 },
	},
	.op_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 562MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 6,
			.mult = 281,
			.sys_div = 1,
			.pix_div = 8,
		},
		{ 0 },
	},
};

/* Minimal config */
static const struct s5k_info s5k3p8_sensor_info = {
	/* Config based on init_seqs */
	/* Real capabilities are not tested */
	/* Seems to be very similar to s5k2p6 */
	.model = "s5k3p8",
	.chip_id = 0x3108,
	.unit_size_nm = 1000,
	.has_dig_horiz2 = true,
	.min_hbl = 184,
	.min_llp = { 5120 },
	.min_vbl = { 170 },
	.max_digital_gain = 64,
	.min_exposure = 8,
	.max_exposure_margin = 8,
	.fifo_margin_bytes = 408,
	.init_delay_us = 1000,
	.scaling_mask = SCL_RANGE_MASK(1.0, 4.0, SCL_STEP_0X5)
		      | SCL_RANGE_MASK(4.0, 8.0, SCL_STEP_1X0),
	.vt_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 560Mpix/s */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 6,
			.mult = 105,
			.sys_div = 1,
			.pix_div = 3,
		},
		{ 0 },
	},
	.op_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 678MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 4,
			.mult = 113,
			.sys_div = 1,
			.pix_div = 8,
		},
		{ 0 },
	},
};

static const struct s5k_info s5k2x7_sensor_info = {
	/* 24Mpix (5664 x 4256) Tetra bayer pattern:
	 * G G R R
	 * G G R R
	 * B B G G
	 * B B G G
	 * Such mediabus format is not available so Y10 is used
	 * instead (to pass through camera subsystem)
	 */
	.model = "s5k2x7",
	.chip_id = 0x2187,
	.unit_size_nm = 900,
	.tetracell  = true,
	.has_postdiv = true,
	.op_clk_div = 2,
	.support_bpp8 = true,
	.fifo_margin_bytes = 576,
	.min_hbl = 184,
	.min_llp = { 6528, 3264 },
	.min_vbl = { 116, 116 },
	/* 8-bit tetra bayer configuration is not supported */
	.min_llp_8b = { 0, 3264 },
	.min_vbl_8b = { 0, 116 },
	.min_crop_w = 1140,
	.max_digital_gain = 64,
	.min_exposure = 3,
	.max_exposure_margin = 5,
	.init_delay_us = 8000,
	/*
	 * Can't have binning > 2 and 1.5x digital because of
	 * tetracell pattern
	 */
	.scaling_mask = SCL_RANGE_MASK(1.0, 1.0, SCL_STEP_1X0)
		      | SCL_RANGE_MASK(2.0, 8.0, SCL_STEP_2X0),
	.vt_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 240Mpix/s, Half-res @ 30FPS */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 4,
			.mult = 120,
			.post_div = 2,
			.sys_div = 1,
			.pix_div = 3,
		},
		{	/* 480Mpix/s, Half-res @ 60FPS */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 4,
			.mult = 120,
			.post_div = 1,
			.sys_div = 1,
			.pix_div = 3,
		},
		{	/* 860Mpix/s, Full-res @ 30FPS (barely)
			 * or Half-res @ 120FPS
			 */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 4,
			.mult = 215,
			.post_div = 1,
			.sys_div = 1,
			.pix_div = 3,
		},
		{ 0 },
	},
	.op_cfgs = (const struct s5k_clk_cfg[]) {
		{	/* 324MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 2,
			.mult = 108,
			.post_div = 1,
			.sys_div = 1,
			.pix_div = 8,
		},
		{	/* 468MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 2,
			.mult = 156,
			.post_div = 1,
			.sys_div = 1,
			.pix_div = 8,
		},
		{	/* 744MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 2,
			.mult = 124,
			.post_div = 0,
			.sys_div = 1,
			.pix_div = 8,
		},
		{	/* 996MHz */
			.inp = S5K_DEFAULT_MCLK,
			.pre_div = 2,
			.mult = 166,
			.post_div = 0,
			.sys_div = 1,
			.pix_div = 8,
		},
		{ 0 },
	},
};

static u32 s5k_ctrl_to_reg(struct s5k_ctx *ctx, int id, int *ret)
{
	switch (id) {
	case V4L2_CID_EXPOSURE:
		return CCS_R_COARSE_INTEGRATION_TIME;
	case V4L2_CID_ANALOGUE_GAIN:
		return CCS_R_ANALOG_GAIN_CODE_GLOBAL;
	case V4L2_CID_DIGITAL_GAIN:
		return CCS_R_DIGITAL_GAIN_GLOBAL;
	case V4L2_CID_TEST_PATTERN:
		return CCS_R_TEST_PATTERN_MODE;
	case V4L2_CID_TEST_PATTERN_RED:
		return CCS_R_TEST_DATA_RED;
	case V4L2_CID_TEST_PATTERN_GREENR:
		return CCS_R_TEST_DATA_GREENR;
	case V4L2_CID_TEST_PATTERN_BLUE:
		return CCS_R_TEST_DATA_BLUE;
	case V4L2_CID_TEST_PATTERN_GREENB:
		return CCS_R_TEST_DATA_GREENB;
	case V4L2_CID_VBLANK:
		return CCS_R_FRAME_LENGTH_LINES;
	case V4L2_CID_HBLANK:
		return CCS_R_LINE_LENGTH_PCK;
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		return 0;
	default:
		break;
	}

	*ret = -EINVAL;
	return 0;
}

static void s5k_update_frame_interval(struct s5k_ctx *ctx,
		struct v4l2_subdev_state *state,
		struct s5k_state *cfg)
{
	u64 num = cfg->fll * cfg->llp, den = cfg->vt_rate;
	struct v4l2_fract *ival;

	if (ctx->interval_locked)
		return;

	if (num > den) {
		num = DIV_ROUND_CLOSEST_ULL(num * 1000u, den);
		den = 1000u;
	} else {
		den = DIV_ROUND_CLOSEST_ULL(den * 1000u, num);
		num = 1000u;
	}

	while  (!((num % 10) || (den % 10)))
		num /= 10, den /= 10;

	ival = v4l2_subdev_state_get_interval(state, S5K_PAD_SRC);
	ival->numerator = num;
	ival->denominator = den;

}

static int s5k_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k_ctx *ctx = container_of(ctrl->handler,
					     struct s5k_ctx, chdl);
	const struct s5k_info *info = ctx->info;
	struct s5k_state *cfg = &ctx->cfg;
	struct v4l2_subdev_state *state;
	struct v4l2_rect *crop;
	u32 llp, fll, reg, id = ctrl->id;
	u32 binv = cfg->binv, binh = cfg->binh;
	int val = ctrl->val, regval, ret = 0;

	sdbg("ctl=%s value=%d", v4l2_ctrl_get_name(id), val);

	reg = s5k_ctrl_to_reg(ctx, id, &ret);
	if (ret)
		return ret;

	state = v4l2_subdev_get_locked_active_state(&ctx->sd);
	llp = cfg->llp;
	fll = cfg->fll;
	crop = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);

	switch (id) {
	case V4L2_CID_EXPOSURE:
		regval = val / binv;
		break;
	case V4L2_CID_HBLANK:
		llp = regval = (val + crop->width) / binh;
		break;
	case V4L2_CID_VBLANK:
		fll = regval = (val + crop->height) / binv;
		break;
	default:
		regval = val;
	}

	if (reg && (regval > U16_MAX || regval < 0)) {
		ret = -EINVAL;
	} else if (reg && pm_runtime_enabled(ctx->dev) &&
		   pm_runtime_get_if_in_use(ctx->dev)) {

		sdbg("writing reg=%x val=%d regval=%d", reg, val, regval);
		ret = cci_write(ctx->rmap, reg, regval, NULL);
		pm_runtime_put_autosuspend(ctx->dev);
		if (ret)
			return ret;
	}

	if ((fll != cfg->fll) || (llp != cfg->llp)) {
		cfg->llp = llp;
		cfg->fll = fll;

		s5k_update_frame_interval(ctx, state, cfg);
	}

	if (id == V4L2_CID_VBLANK || id == V4L2_CID_PIXEL_RATE) {
		u32 min_exp, max_exp;
		/*
		 * Tetra binning increases exposure step and min/max_margin
		 * by factor of 2
		 */
		min_exp = info->min_exposure << cfg->tbin;
		max_exp = fll - (info->max_exposure_margin << cfg->tbin);
		ret = __v4l2_ctrl_modify_range(ctx->exp,
				binv * min_exp,
				binv * max_exp,
				binv, binv * max_exp);
	}

	return 0;
}

static u64 s5k_calc_rate(struct s5k_ctx *ctx, const struct s5k_clk_cfg *cfg,
		bool is_pix_clk)
{
	u64 rate = cfg->inp / cfg->pre_div * cfg->mult;

	if (ctx->info->has_postdiv)
		rate >>= cfg->post_div;
	if (is_pix_clk)
		rate = mult_frac(rate, 4, (u32)cfg->pix_div);
	else if (ctx->info->op_clk_div)
		rate /= ctx->info->op_clk_div;

	return rate;
}

static u64 s5k_find_rate(struct s5k_ctx *ctx, u32 req_rate,
		const struct s5k_clk_cfg **out, bool is_pix_clk)
{
	const struct s5k_clk_cfg *cfg;
	u64 rate = 0;

	cfg = is_pix_clk ? ctx->info->vt_cfgs : ctx->info->op_cfgs;

	for (; cfg && cfg->inp; cfg++) {
		if (ctx->mclk_rate == cfg->inp) {
			*out = cfg;
			rate = s5k_calc_rate(ctx, cfg, is_pix_clk);
			if (rate >= req_rate)
				break;
		}
	}

	return rate;
}

static int s5k_index_to_mbus_code(struct s5k_ctx *ctx, int index, u32 *code)
{
	bool mono = ctx->info->monochrome;

	if (!ctx->info->support_bpp8)
		index *= 2;

	if (index == 0)
		*code = !mono ? MEDIA_BUS_FMT_SGRBG10_1X10 : MEDIA_BUS_FMT_Y10_1X10;
	else if (index == 1)
		*code = !mono ? MEDIA_BUS_FMT_SGRBG8_1X8 : MEDIA_BUS_FMT_Y8_1X8;
	else if (index == 2 && ctx->info->tetracell)
		*code = MEDIA_BUS_FMT_SGGRR10_1X10;
	else	/* 8bpp works only with tetra-binning */
		return -EINVAL;
	return 0;
}

static u8 s5k_check_format_bpp(struct s5k_ctx *ctx,
		struct v4l2_mbus_framefmt *format)
{
	u32 code = 0, i;

	for (i = 0; i < 4; i++) {
		if (s5k_index_to_mbus_code(ctx, i, &code))
			return 0;

		if (code == format->code)
			return (i & 1) ? 8 : 10;
	}

	return 0;
}

static inline bool s5k_is_tetra_format(struct s5k_ctx *ctx,
		struct v4l2_mbus_framefmt *format)
{
	if (!ctx->info->tetracell)
		return false;

	return format->code == MEDIA_BUS_FMT_SGGRR10_1X10;
}

static void s5k_try_format(struct s5k_ctx *ctx,
		struct v4l2_subdev_state *state,
		struct s5k_state *cfg)
{
	const struct s5k_info *info = ctx->info;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	u32 bin, r1, r2, min_ratio = 2, max_ratio = 32;
	bool is_8bpp;

	format = v4l2_subdev_state_get_format(state, S5K_PAD_SRC);
	crop = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);

	is_8bpp = s5k_check_format_bpp(ctx, format) == 8;

	format->width = max(MIN_OUTPUT_WH, format->width);
	format->height = max(MIN_OUTPUT_WH, format->height);

	r1 = crop->width * 2;
	r2 = crop->height * 2;

	r1 = (r1 + format->width / 2) / format->width;
	r2 = (r2 + format->height / 2) / format->height;

	r1 = min(r1, r2);

	cfg->binv = cfg->binh = 1;

	if (s5k_is_tetra_format(ctx, format))
		max_ratio = 2;
	else if (info->tetracell) {
		min_ratio = 4;
		cfg->binv = cfg->binh = 2;
	}

	r1 = clamp(r1, min_ratio, max_ratio);

	cfg->ratio = min_ratio;

	for (r2 = min_ratio; r2 <= r1 && r2 <= max_ratio; r2++)
		if (info->scaling_mask & BIT(r2 - 2))
			cfg->ratio = r2;

	for (bin = S5K_MAX_BINNING; bin >= 1; bin--) {
		if (cfg->ratio % (bin * 2))
			continue;
		if ((is_8bpp ? info->min_vbl_8b : info->min_vbl)[bin - 1])
			cfg->binv = max_t(u8, cfg->binv, bin);
		if ((is_8bpp ? info->min_llp_8b : info->min_llp)[bin - 1])
			cfg->binh = max_t(u8, cfg->binh, bin);
	}

#ifdef S5K_DEBUG_IFACE
	if (ctx->force_ratio)
		cfg->ratio = clamp(ctx->force_ratio, 2, 32);
	if (ctx->force_binh)
		cfg->binh = clamp(ctx->force_binh, 1, S5K_MAX_BINNING);
	if (ctx->force_binv)
		cfg->binv = clamp(ctx->force_binv, 1, S5K_MAX_BINNING);
#endif

	format->width = mult_frac(crop->width, 2, (u32)cfg->ratio) & ~3;
	format->height = mult_frac(crop->height, 2, (u32)cfg->ratio) & ~1;

	cfg->tbin = (cfg->binh > 1) ? info->tetracell : 0;

	sdbg("result %ux%u ratio=%u/2 binning=%ux%u tbin=%u", format->width, format->height,
			cfg->ratio, cfg->binh, cfg->binv, cfg->tbin);
}

static void s5k_fine_tune_intervals(struct s5k_ctx *ctx, struct s5k_state *cfg,
		u64 target_flp)
{
	int total_rounds = 512, rounds = 128;
	u8 tetracell = ctx->info->tetracell;
	u64 llp = cfg->llp, fll = cfg->fll;
	u64 best_flp, flp;

	best_flp = llp * fll;

	do {
		if (llp > S5K_LLP_MAX - S5K_LLP_STEP)
			break;

		llp += S5K_LLP_STEP;
		fll = DIV_ROUND_CLOSEST((u32)target_flp, llp << tetracell);
		fll = max(cfg->fll_min, fll << tetracell);
		flp = fll * llp;

		if (abs_diff(flp, target_flp) <= abs_diff(best_flp, target_flp)) {
			sdbg("better frame time: round=%d fll=%llu llp=%llu flp=%llu best=%llu",
					512 - total_rounds, fll, llp, flp, best_flp);
			rounds = 128;
			best_flp = flp;
			cfg->fll = fll;
			cfg->llp = llp;
		}
	} while (fll > cfg->fll_min && --total_rounds > 0 && --rounds > 0);
}


static void s5k_try_frame_interval(struct s5k_ctx *ctx,
		struct v4l2_subdev_state *state,
		struct s5k_state *cfg, bool reset_ival)
{
	struct v4l2_fract *ival;
	const struct s5k_info *info = ctx->info;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	u64 vbl_min, llp_min, fll_min;
	u64 fll = 0, flp, llp = 0;
	u64 req_prate, req_lfreq, prate, lfreq, bpl, bpp;
	u64 target_flp;

	format = v4l2_subdev_state_get_format(state, S5K_PAD_SRC);
	crop = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);
	ival = v4l2_subdev_state_get_interval(state, S5K_PAD_SRC);
	sdbg("requested interval %u/%u sec", ival->numerator, ival->denominator);

	bpp = s5k_check_format_bpp(ctx, format);

	if (!ival->numerator || !ival->denominator || reset_ival) {
		ival->numerator = 1;
		ival->denominator = S5K_DEFAULT_FPS;
	}

	llp_min = (bpp == 8 ? info->min_llp_8b : info->min_llp)[cfg->binh - 1];

	/* Hardware llp_min limit can be smaller than crop->width + min_hbl */
	llp = DIV_ROUND_UP(crop->width, cfg->binh) + info->min_hbl;
	llp_min = S5K_LLP_ALIGN(max(llp, llp_min));

	vbl_min = (bpp == 8 ? info->min_vbl_8b : info->min_vbl)[cfg->binv - 1];
	fll_min = S5K_FLL_ALIGN(info, crop->height / cfg->binv + vbl_min);

	sdbg("llp_min=%llu fll_min=%llu vbl_min=%llu", llp_min, fll_min, vbl_min);

	/* Figure out pixel clock needed for requested interval */
	req_prate = mult_frac(fll_min * llp_min,
			ival->denominator, ival->numerator);
	prate = s5k_find_rate(ctx, req_prate, &cfg->vt, true);

	sdbg("required prate=%llu available prate=%llu", req_prate, prate);

	bpl = format->width * bpp;
	bpl += info->fifo_margin_bytes * 8;
	req_lfreq = mult_frac(prate, bpl, S5K_NUM_LANES * 2 * llp_min);

	lfreq = s5k_find_rate(ctx, req_lfreq, &cfg->op, false);

	sdbg("required lfreq=%llu available lfreq=%llu", req_lfreq, lfreq);

	if (WARN_ON(!lfreq))
		lfreq = 1 << 20;

	if (WARN_ON(!prate))
		prate = 1 << 20;

	/* Adjust llp_min to avoid FIFO overflow */
	llp = mult_frac(prate, bpl, S5K_NUM_LANES * 2 * lfreq);
	llp = S5K_LLP_ALIGN(llp);
	if (llp > llp_min) {
		llp_min = llp;
		sdbg("adjusted llp_min=%llu", llp_min);
	}

	fll = fll_min;
	llp = llp_min;
	target_flp = mult_frac(prate, ival->numerator, ival->denominator);

	sdbg("step 1 min_fll=%llu min_llp=%llu target_flp=%llu",
			fll, llp, target_flp);
	if (target_flp > S5K_FLP_MAX) {
		target_flp = S5K_FLP_MAX;
		llp = S5K_LLP_MAX;
	}

	fll = DIV_ROUND_UP(target_flp, llp);

	sdbg("step 2 fll=%llu llp=%llu", fll, llp);

	/* Trying to set too high interval, clamp it down */
	if (fll > S5K_FLL_MAX || fll < fll_min) {
		fll = fll > S5K_FLL_MAX ? S5K_FLL_MAX : fll_min;
		llp = DIV_ROUND_DOWN_ULL(flp, fll);
		llp = S5K_LLP_ALIGN(llp);
		llp = clamp(llp, llp_min, S5K_LLP_MAX);
		fll = DIV_ROUND_UP(flp, llp);
	}

	fll = S5K_FLL_ALIGN(info, fll);
	fll = clamp(fll, fll_min, S5K_FLL_MAX);
	sdbg("step 3 fll=%llu llp=%llu", fll, llp);

	cfg->op_rate = lfreq;
	cfg->vt_rate = prate;
	cfg->fll_min = fll_min;
	cfg->llp_min = llp_min;
	cfg->llp = llp;
	cfg->fll = fll;

	s5k_fine_tune_intervals(ctx, cfg, target_flp);

	s5k_update_frame_interval(ctx, state, cfg);

	sdbg("final link_freq=%llu min_llp=%u min_fll=%u", lfreq,
			cfg->llp_min, cfg->fll_min);
	sdbg("final llp=%u fll=%u interval=%u/%u sec", cfg->llp,
			cfg->fll, ival->numerator, ival->denominator);
}

static void s5k_update_controls(struct s5k_ctx *ctx)
{
	struct s5k_state *cfg = &ctx->cfg;
	struct v4l2_subdev_state *state;
	struct v4l2_rect *crop;
	int link_idx, binv, binh;
	u32 fll, llp;

	/* V4L2 Controls need crop interval from active state */
	state = v4l2_subdev_get_locked_active_state(&ctx->sd);
	if (!state)
		return;

	binv = cfg->binv;
	binh = cfg->binh;
	crop = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);

	for (link_idx = 0; link_idx < ctx->n_link_freqs; link_idx++)
		if (ctx->lfreq_menu[link_idx] == cfg->op_rate)
			break;

	fll = cfg->fll;
	llp = cfg->llp;

	ctx->interval_locked = true;

	__v4l2_ctrl_s_ctrl(ctx->lfreq, link_idx);

	__v4l2_ctrl_s_ctrl_int64(ctx->prate,
			binv * binh * ((u64) cfg->vt_rate));

	__v4l2_ctrl_modify_range(ctx->vbl,
			binv * cfg->fll_min - crop->height,
			binv * S5K_FLL_MAX - crop->height,
			max(binv, 1 + ctx->info->tetracell),
			binv * fll - crop->height);
	__v4l2_ctrl_modify_range(ctx->hbl,
			binh * cfg->llp_min - crop->width,
			binh * S5K_LLP_MAX - crop->width,
			binh * S5K_LLP_STEP,
			binh * llp - crop->width);

	__v4l2_ctrl_s_ctrl(ctx->vbl, binv * fll - crop->height);
	__v4l2_ctrl_s_ctrl(ctx->hbl, binh * llp - crop->width);

	ctx->interval_locked = false;
}

static void s5k_update_subdev_state(struct s5k_ctx *ctx,
		struct v4l2_subdev_state *state,
		int which, bool reset_ival)
{
	struct s5k_state cfg = { 0 };

	s5k_try_format(ctx, state, &cfg);
	s5k_try_frame_interval(ctx, state, &cfg, reset_ival);

	if (which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return;

	if (ctx->cfg.vt != cfg.vt || ctx->cfg.op != cfg.op)
		ctx->clks_changed = true;

	ctx->mode_changed = !!memcmp(&cfg, &ctx->cfg, sizeof(cfg));
	ctx->cfg = cfg;

	s5k_update_controls(ctx);
}

static int s5k_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	return s5k_index_to_mbus_code(sd_to_ctx(sd), code->index, &code->code);
}

static int s5k_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k_ctx *ctx = sd_to_ctx(sd);
	u32 scaling_mask = ctx->info->scaling_mask;
	struct v4l2_rect *crop;
	int w, h, factor = 0, idx = 0;

	if (fse->pad != S5K_PAD_SRC)
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);

	w = crop->width;
	h = crop->height;

	if (ctx->info->tetracell) {
		if (fse->code == MEDIA_BUS_FMT_SGGRR10_1X10)
			scaling_mask &= BIT(0);
		else
			scaling_mask &= ~BIT(1);
	}

	for (factor = 2; scaling_mask; factor++, scaling_mask >>= 1) {
		if (!(scaling_mask & 1))
			continue;

		if (idx++ != fse->index)
			continue;

		w = mult_frac(w, 2, factor) & ~3;
		h = mult_frac(h, 2, factor) & ~1;

		if (min(w, h) < MIN_OUTPUT_WH)
			return -EINVAL;

		fse->max_width = fse->min_width = w;
		fse->max_height = fse->min_height = h;
		sdbg("index=%d code=%#x factor=%d/2 w=%d h=%d",
				fse->index, fse->code, factor, w, h);
		return 0;
	}

	return -EINVAL;
}

static int s5k_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct s5k_ctx *ctx = sd_to_ctx(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = ctx->max_crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s5k_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct s5k_ctx *ctx = sd_to_ctx(sd);
	struct v4l2_rect *target, rect;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	sdbg("%s flags=%#x rect=%ux%u+%d+%d", WHICH_STR(sel->which),
			sel->flags,
			sel->r.width, sel->r.height,
			sel->r.left, sel->r.top);

	int align_offset = (sel->flags & V4L2_SEL_FLAG_GE) ? 3 :
		      ((sel->flags & V4L2_SEL_FLAG_LE) ? 0 : 2);

	rect = sel->r;
	rect.top &= ~1;
	rect.left &= ~1;
	rect.width = (rect.width + align_offset) & ~3;
	rect.height = (rect.height + align_offset) & ~3;

	v4l2_rect_set_min_size(&rect, &ctx->min_crop);
	v4l2_rect_map_inside(&rect, &ctx->max_crop);

	target = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);
	sel->r = *target = rect;

	s5k_update_subdev_state(ctx, state, sel->which, true);

	sdbg("%s result flags=%#x rect=%ux%u+%d+%d", WHICH_STR(sel->which),
			sel->flags,
			sel->r.width, sel->r.height,
			sel->r.left, sel->r.top);

	return 0;
}


static int s5k_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt)
{
	struct s5k_ctx *ctx = sd_to_ctx(sd);
	struct v4l2_mbus_framefmt *format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	sdbg("%s %dx%d code %#x", WHICH_STR(fmt->which), fmt->format.width,
			fmt->format.height, fmt->format.code);

	format = v4l2_subdev_state_get_format(state, S5K_PAD_SRC);
	format->code = fmt->format.code;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->field = V4L2_FIELD_NONE;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->width = fmt->format.width;
	format->height = fmt->format.height;

	if (!s5k_check_format_bpp(ctx, format))
		s5k_index_to_mbus_code(ctx, 0, &format->code);

	s5k_update_subdev_state(ctx, state, fmt->which, true);

	fmt->format = *format;

	sdbg("%s result %dx%d code %#x", WHICH_STR(fmt->which), fmt->format.width,
			fmt->format.height, fmt->format.code);

	return 0;
}

static int s5k_set_frame_interval(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_frame_interval *interval)
{
	struct v4l2_mbus_framefmt *format;
	struct s5k_ctx *ctx = sd_to_ctx(sd);
	struct v4l2_fract *ival;
	int ret = 0;

	if (interval->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	format = v4l2_subdev_state_get_format(state, S5K_PAD_SRC);
	ival = v4l2_subdev_state_get_interval(state, S5K_PAD_SRC);
	*ival = interval->interval;

	sdbg("%s %d/%d sec", WHICH_STR(interval->which),
			ival->numerator, ival->denominator);

	s5k_update_subdev_state(ctx, state, interval->which, false);
	interval->interval = *ival;

	sdbg("%s result interval=%d/%d sec", WHICH_STR(interval->which),
			ival->numerator, ival->denominator);
	return ret;
}

static int s5k_apply_state(struct s5k_ctx *ctx, struct v4l2_subdev_state *state)
{
	struct s5k_state *cfg = &ctx->cfg;
	struct v4l2_mbus_framefmt *format;
	struct s5k_clk_cfg op, vt;
	struct v4l2_rect *r;
	u8 evenx, eveny, oddx, oddy;
	u8 binmode, bintype;
	u16 scl_h, scl_m, scl_mode;
	int ret = 0;

	op = cfg->op[0];
	vt = cfg->vt[0];

	r = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);
	format = v4l2_subdev_state_get_format(state, S5K_PAD_SRC);

	sdbg("crop: %ux%u+%d+%d output: %ux%u", r->width, r->height,
			r->left, r->top, format->width, format->height);
	sdbg("pixel clock: %llu link rate: %llu", cfg->vt_rate, cfg->op_rate);
	sdbg("vt clocks: pre_div=%u mult=%u post_div=%u sys_div=%u pix_div=%u",
			vt.pre_div, vt.mult, 1 << vt.post_div,
			vt.sys_div, vt.pix_div);
	sdbg("op clocks: pre_div=%u mult=%u post_div=%u sys_div=%u pix_div=%u",
			op.pre_div, op.mult, 1 << op.post_div,
			op.sys_div, op.pix_div);
	sdbg("LLP: %u FLL: %u", cfg->llp, cfg->fll);

	if (ctx->info->chip_id == 0x2187 && ctx->uninited) {
		/* Fixes slightly varying brightness across lines */
		cci_write(ctx->rmap, CCI_REG8(0x3843), 15, &ret);
		/* Fixes occasional broken lines */
		cci_write(ctx->rmap, CCI_REG8(0x655f), 0xe8, &ret);
		/* Fixes slightly varying brightness across lines with binning */
		cci_write(ctx->rmap, CCI_REG16(0x6214), 0x7971, &ret);
		cci_write(ctx->rmap, CCI_REG16(0x6218), 0x7150, &ret);
		cci_write(ctx->rmap, CCI_REG16(0xf4b0), 0x1124, &ret);
	}

	if (ctx->clks_changed) {
		cci_write(ctx->rmap, CCS_R_EXTCLK_FREQUENCY_MHZ,
				mult_frac(vt.inp, BIT(8), 1000000), &ret);
		cci_write(ctx->rmap, CCS_R_PRE_PLL_CLK_DIV, vt.pre_div, &ret);
		cci_write(ctx->rmap, CCS_R_PLL_MULTIPLIER, vt.mult, &ret);
		cci_write(ctx->rmap, CCS_R_VT_PIX_CLK_DIV, vt.pix_div, &ret);
		cci_write(ctx->rmap, CCS_R_VT_SYS_CLK_DIV, vt.sys_div, &ret);
		cci_write(ctx->rmap, CCS_R_OP_PRE_PLL_CLK_DIV, op.pre_div, &ret);
		cci_write(ctx->rmap, CCS_R_OP_PLL_MULTIPLIER, op.mult, &ret);
		cci_write(ctx->rmap, CCS_R_OP_PIX_CLK_DIV, op.pix_div, &ret);
		cci_write(ctx->rmap, CCS_R_VT_SYS_CLK_DIV, op.sys_div, &ret);

		if (ctx->info->has_postdiv) {
			cci_write(ctx->rmap, S5K_R_POST_PLL_CLK_DIV, vt.post_div, &ret);
			cci_write(ctx->rmap, S5K_R_OP_POST_PLL_CLK_DIV, op.post_div, &ret);
		}
	}

	if (!ctx->mode_changed)
		return 0;

	cci_write(ctx->rmap, CCS_R_CSI_DATA_FORMAT,
			s5k_check_format_bpp(ctx, format) * 0x101, &ret);
	cci_write(ctx->rmap, CCS_R_X_ADDR_START, r->left, &ret);
	cci_write(ctx->rmap, CCS_R_Y_ADDR_START, r->top, &ret);
	cci_write(ctx->rmap, CCS_R_X_ADDR_END, r->left + r->width - 1, &ret);
	cci_write(ctx->rmap, CCS_R_Y_ADDR_END, r->top + r->height - 1, &ret);

	cci_write(ctx->rmap, CCS_R_X_OUTPUT_SIZE, format->width, &ret);
	cci_write(ctx->rmap, CCS_R_Y_OUTPUT_SIZE, format->height, &ret);

	evenx = eveny = 1;
	binmode = (cfg->binh | cfg->binv) > 1;
	bintype = (cfg->binh << 4) | cfg->binv;

	if (ctx->info->chip_id == 0x2187) {
		cci_write(ctx->rmap, CCI_REG8(0x3002), binmode ? 1 : 0, &ret);
		/* 5 for binning of 4 but G R rows get darker */
		cci_write(ctx->rmap, CCI_REG8(0x324a), binmode ? 3 : 1, &ret);
		evenx = eveny = binmode ? 2 : 1;

		/* Causes yellowish image because B G rows are darker. */
		bintype = binmode = 0;
	}

	oddx = cfg->binh * 2 - evenx;
	oddy = cfg->binv * 2 - eveny;

	sdbg("step even x=%u y=%u odd x=%u y=%u", evenx, eveny, oddx, oddy);
	cci_write(ctx->rmap, CCS_R_X_EVEN_INC, evenx, &ret);
	cci_write(ctx->rmap, CCS_R_Y_EVEN_INC, eveny, &ret);
	cci_write(ctx->rmap, CCS_R_X_ODD_INC, oddx, &ret);
	cci_write(ctx->rmap, CCS_R_Y_ODD_INC, oddy, &ret);

	sdbg("binning h=%u v=%u", cfg->binh, cfg->binv);
	scl_mode = 0;
	scl_m = 8 * cfg->ratio / cfg->binv;
	scl_h = 16 * cfg->binv / cfg->binh;

	if (scl_m == 16 && scl_h != 16 && !(scl_h % 8)) {
		scl_mode = 1;
		scl_m = scl_h;
		scl_h = 16;
	} else if (scl_m != 16 || scl_h != 16) {
		scl_mode = 2;
	}

	sdbg("scale mode=%u m=%u/16 h=%u/16 bintype=%x",
			scl_mode, scl_m, scl_h, bintype);
	cci_write(ctx->rmap, CCS_R_BINNING_MODE, binmode, &ret);
	cci_write(ctx->rmap, CCS_R_BINNING_TYPE, bintype, &ret);
	if (!ctx->info->has_dig_horiz2 && scl_h != 16)
		dev_err(ctx->dev, "extra horizontal scaling is not supported");

	cci_write(ctx->rmap, S5K_R_DIG_HORIZ_SCALE, scl_h, &ret);

	cci_write(ctx->rmap, CCS_R_SCALE_M, scl_m, &ret);
	cci_write(ctx->rmap, CCS_R_SCALING_MODE, scl_mode, &ret);
	if (ret)
		dev_err(ctx->dev, "failed to update registers: %d\n", ret);

	return ret;
}

static int s5k_start_streaming(struct s5k_ctx *ctx,
		struct v4l2_subdev_state *state)
{
	int ret;

#ifdef S5K_DEBUG_IFACE
	if (ctx->uninited)
		s5k_debug_write_seq(ctx, &ctx->dbg_pre_init);
#endif

	ret = s5k_apply_state(ctx, state);
	if (ret)
		return ret;

	ret = !ctx->uninited ? 0 : __v4l2_ctrl_handler_setup(&ctx->chdl);
	if (ret)
		return ret;

#ifdef S5K_DEBUG_IFACE
	s5k_debug_write_seq(ctx, &ctx->dbg_pre_stream);
#endif

	if (ctx->uninited || ctx->clks_changed) {
		u32 delay = ctx->info->init_delay_us;

		if (ctx->dbg_init_delay_us)
			delay = min(30000, ctx->dbg_init_delay_us);

		udelay(delay);
	}

	ret = cci_write(ctx->rmap, CCS_R_MODE_SELECT, 1, NULL);

	ctx->uninited = ctx->mode_changed = ctx->clks_changed = !ret;
	return ret;
}

static int s5k_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k_ctx *ctx = sd_to_ctx(sd);
	struct v4l2_subdev_state *state;
	int ret;

	sdbg("enable=%d", enable);

	if (!enable) {
		u64 err = 0xdead;

		cci_read(ctx->rmap, S5K_R_SYSTEM_ERROR, &err, NULL);
		if (err)
			dev_err(ctx->dev, "sys error %d\n", (int)err);

		ret = cci_write(ctx->rmap, CCS_R_MODE_SELECT, 0, NULL);

		pm_runtime_mark_last_busy(ctx->dev);

		if (!ret) {
			pm_runtime_put_autosuspend(ctx->dev);
		} else {
			dev_err(ctx->dev, "failed to enter standby: %d\n", ret);
			pm_runtime_put_sync(ctx->dev);
			ret = 0;
		}
	} else {
		ret = pm_runtime_resume_and_get(ctx->dev);
		if (ret)
			return ret;

		state = v4l2_subdev_lock_and_get_active_state(sd);
		ret = s5k_start_streaming(ctx, state);
		v4l2_subdev_unlock_state(state);

		if (ret)
			pm_runtime_put_sync(ctx->dev);
	}

	sdbg("ret=%d", ret);

	return ret;
}

static int s5k_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { 0 };
	struct s5k_ctx *ctx = sd_to_ctx(sd);
	struct v4l2_fract *ival;
	struct v4l2_rect *crop;

	ival = v4l2_subdev_state_get_interval(state, S5K_PAD_SRC);
	ival->numerator = 1;
	ival->denominator = S5K_DEFAULT_FPS;

	crop = v4l2_subdev_state_get_crop(state, S5K_PAD_SRC);
	*crop = ctx->max_crop;

	fmt.format.width = crop->width;
	fmt.format.height = crop->height;
	fmt.which = V4L2_SUBDEV_FORMAT_TRY;
	/* Detect active state initialization (by v4l2_subdev_init_finalize) */
	if (!ctx->cfg.llp_min)
		fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	s5k_set_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_ctrl_ops s5k_ctrl_ops = {
	.s_ctrl = s5k_set_ctrl,
};

static const struct v4l2_subdev_video_ops s5k_video_ops = {
	.s_stream = s5k_set_stream,
};

static const struct v4l2_subdev_pad_ops s5k_pad_ops = {
	.enum_mbus_code = s5k_enum_mbus_code,
	.enum_frame_size = s5k_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k_set_format,
	.get_selection = s5k_get_selection,
	.set_selection = s5k_set_selection,
	.get_frame_interval = v4l2_subdev_get_frame_interval,
	.set_frame_interval = s5k_set_frame_interval,
};

static const struct v4l2_subdev_ops s5k_subdev_ops = {
	.video = &s5k_video_ops,
	.pad = &s5k_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k_internal_ops = {
	.init_state = s5k_init_state,
};

static const struct media_entity_operations s5k_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k_probe_sensor(struct s5k_ctx *ctx)
{
	struct v4l2_fwnode_device_properties props;
	const struct v4l2_ctrl_ops *ops = &s5k_ctrl_ops;
	const struct s5k_info *info = ctx->info;
	const struct s5k_clk_cfg *cfg;
	struct v4l2_ctrl_handler *hdl;
	union v4l2_ctrl_ptr unit_size = v4l2_ctrl_ptr_create(&ctx->unit_size);
	u64 vmin, vmax, vstep, val;
	int ret, i, max_binv = 1, max_binh = 1;

	ret = cci_read(ctx->rmap, CCS_R_MODULE_MODEL_ID, &val, NULL);
	if (ret)
		return dev_err_probe(ctx->dev, ret, "failed to read ID\n");

	if (info->chip_id != val) {
		dev_err(ctx->dev, "unexpected device ID: %x\n", (u32)val);
		ret = cci_read(ctx->rmap, CCI_REG16(0x2000), &val, NULL);
		if (ret || info->chip_id != val)
			return -ENODEV;
	}

	hdl = &ctx->chdl;
	ret = v4l2_ctrl_handler_init(hdl, 13);
	if (ret)
		return ret;

	cci_read(ctx->rmap, CCS_R_MIN_X_OUTPUT_SIZE, &vmin, &ret);
	cci_read(ctx->rmap, CCS_R_MAX_X_OUTPUT_SIZE, &vmax, &ret);
	ctx->min_crop.width = min(info->min_crop_w, vmin);
	ctx->max_crop.width = vmax;

	cci_read(ctx->rmap, CCS_R_MIN_Y_OUTPUT_SIZE, &vmin, &ret);
	cci_read(ctx->rmap, CCS_R_MAX_Y_OUTPUT_SIZE, &vmax, &ret);
	ctx->min_crop.height = vmin;
	ctx->max_crop.height = vmax;
	ctx->unit_size.width = ctx->unit_size.height = info->unit_size_nm;

	v4l2_ctrl_new_std_compound(hdl, ops, V4L2_CID_UNIT_CELL_SIZE,
			unit_size, unit_size, unit_size);

	cci_read(ctx->rmap, CCS_R_ANALOG_GAIN_CODE_MIN, &vmin, &ret);
	cci_read(ctx->rmap, CCS_R_ANALOG_GAIN_CODE_MAX, &vmax, &ret);
	cci_read(ctx->rmap, CCS_R_ANALOG_GAIN_CODE_STEP, &vstep, &ret);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			vmin, vmax, vstep, vmin);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN, S5K_DIG_GAIN_STEP,
			S5K_DIG_GAIN_STEP * info->max_digital_gain,
			S5K_DIG_GAIN_STEP, S5K_DIG_GAIN_STEP);

	ctx->exp = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
			info->min_exposure, info->min_exposure, 1,
			info->min_exposure);

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(s5k_test_pattern_menu) - 1,
			0, 0, s5k_test_pattern_menu);

	for (i = 0; i < 4; i++)
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_RED + i,
				  0, 0x3ff, 1, 0x1ff);

	for (i = 1; i < S5K_MAX_BINNING; i++) {
		max_binh = info->min_llp[i] ? i + 1 : max_binh;
		max_binv = info->min_vbl[i] ? i + 1 : max_binv;
	}

	vmin = s5k_find_rate(ctx, 0u, &cfg, true);
	vmax = s5k_find_rate(ctx, ~0u, &cfg, true);

	ctx->prate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
			vmin, max_binv * max_binh * vmax, 1, vmin);
	ctx->lfreq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
			ctx->n_link_freqs - 1, 0, ctx->lfreq_menu);
	ctx->lfreq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* These will be corrected after active state initialization */
	ctx->vbl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK, 1, 128, 1, 1);
	ctx->hbl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK, 1, 128, 1, 1);

	/* Parse orientation and rotation */
	ret = ret ?: v4l2_fwnode_device_parse(ctx->dev, &props);
	ret = ret ?: v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
	ret = ret ?: hdl->error;
	if (ret)
		v4l2_ctrl_handler_free(hdl);

	ctx->sd.ctrl_handler = hdl;

	return ret;
}

static int s5k_check_hwcfg(struct s5k_ctx *ctx)
{
	struct fwnode_handle *fwnode = dev_fwnode(ctx->dev);
	struct device *dev = ctx->dev;
	struct v4l2_fwnode_endpoint bus_cfg = { 0 };
	struct fwnode_handle *ep;
	const struct s5k_clk_cfg *cfg;
	u32 rate, prev;
	int ret = 0;

	if (!fwnode)
		return -ENXIO;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	fwnode_property_read_u32(fwnode, "clock-frequency", &ctx->mclk_rate);

	/* For now all devices seem to use 4 lanes anyways */
	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K_NUM_LANES) {
		v4l2_fwnode_endpoint_free(&bus_cfg);
		return dev_err_probe(dev, -EINVAL, "unsupported lane count\n");
	}

	v4l2_fwnode_endpoint_free(&bus_cfg);

	prev = 0;
	rate = s5k_find_rate(ctx, prev, &cfg, false);
	while (ctx->n_link_freqs < ARRAY_SIZE(ctx->lfreq_menu) && rate > prev) {
		ctx->lfreq_menu[ctx->n_link_freqs++] = rate;
		prev = rate;
		rate = s5k_find_rate(ctx, prev + 1, &cfg, false);
	}

	if (!ctx->n_link_freqs)
		return dev_err_probe(dev, -EINVAL,
				"no supported link frequencies\n");

	ret = (int) clk_round_rate(ctx->mclk, ctx->mclk_rate);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to round mclk\n");

	ctx->mclk_actual = ret;
	if (abs_diff((u32)ret, ctx->mclk_rate) > (ctx->mclk_rate >> 8))
		return dev_err_probe(dev, -EINVAL, "unsupported mclk: %d\n", ret);

	return 0;
}

static int s5k_power_on(struct s5k_ctx *ctx)
{
	int ret;

	gpiod_set_value_cansleep(ctx->rst, 1);

	ret = clk_set_rate(ctx->mclk, ctx->mclk_rate);
	if (ret) {
		dev_err(ctx->dev, "failed to set mclk frequency: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ctx->num_vregs, ctx->vregs);
	if (ret) {
		dev_err(ctx->dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	usleep_range(4000, 6000);

	gpiod_set_value_cansleep(ctx->rst, 0);

	usleep_range(2000, 3000);

	ret = clk_prepare_enable(ctx->mclk);
	if (ret) {
		dev_err(ctx->dev, "failed to enable mclk: %d\n", ret);
		regulator_bulk_disable(ctx->num_vregs, ctx->vregs);
		return ret;
	}

	usleep_range(1000, 2000);

	sdbg("");

	return 0;
}

static void s5k_power_off(struct s5k_ctx *ctx)
{
	sdbg("");
	ctx->uninited = ctx->mode_changed = ctx->clks_changed = true;
	gpiod_set_value_cansleep(ctx->rst, 1);
	clk_disable_unprepare(ctx->mclk);
	regulator_bulk_disable(ctx->num_vregs, ctx->vregs);
}

#ifdef S5K_DEBUG_IFACE
static void s5k_debug_write_seq(struct s5k_ctx *ctx, struct s5k_init_seq_file *isf)
{
	char *text = isf->cur;
	int ret = 0, cnt = 0;

	while (text && !ret && text[0]) {
		u32 addr, val;

		while (text[0] != 0 && text[0] == '\n')
			text++;

		if (sscanf(text, "{ 0x%x, 0x%x },", &addr, &val) == 2 ||
		    sscanf(text, "0x%x 0x%x", &addr, &val) == 2) {
			ret = cci_write(ctx->rmap, CCI_REG16(addr), val, NULL);
			cnt += !ret;
		}

		while (text[0] != 0 && text[0] != '\n')
			text++;
	}

	if (ret)
		dev_err(ctx->dev, "failed to write sequence: %d\n", ret);

	sdbg("wrote %d registers", cnt);
}

static int s5k_debug_reg_read(struct seq_file *s, void *data)
{
	struct s5k_ctx *ctx = dev_to_ctx(s->private);
	u32 reg = ctx->dbg_reg & ~1;
	u32 cnt = ctx->dbg_reg_cnt ?: 1;
	u64 val;
	int i, ret;

	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);

	for (i = 0; i < cnt; i++) {
		ret = cci_read(ctx->rmap, CCI_REG16(reg), &val, NULL);
		if (ret)
			break;

		if ((i & 7) == 0)
			seq_printf(s, "%04x: ", reg);
		seq_printf(s, " %04x%s", (u32) val, (i & 7) == 7 ? "\n" : "");
		reg += 2;
		reg &= 0xffff;
	}

	if ((i & 7) != 0)
		seq_puts(s, "\n");

	mutex_unlock(&ctx->lock);
	pm_runtime_put_autosuspend(ctx->dev);

	return 0;
}

static int s5k_debug_reg_write(void *data, u64 val)
{
	struct s5k_ctx *ctx = data;
	u32 reg = ctx->dbg_reg & ~1;
	int ret;

	if (val > 0xffff)
		return -ERANGE;

	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	ret = cci_write(ctx->rmap, CCI_REG16(reg), val, NULL);
	mutex_unlock(&ctx->lock);

	pm_runtime_put_autosuspend(ctx->dev);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(s5k_debug_reg_write_fops,
		NULL, s5k_debug_reg_write, "0x%02llx\n");

static ssize_t s5k_init_seq_read(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct s5k_init_seq_file *isf = file->private_data;
	struct s5k_ctx *ctx = isf->ctx;
	ssize_t r;

	mutex_lock(&ctx->lock);
	r = simple_read_from_buffer(user_buf, count, ppos, isf->next, isf->next_size);
	mutex_unlock(&ctx->lock);
	return r;
}

static ssize_t s5k_init_seq_write(struct file *file, const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct s5k_init_seq_file *isf = file->private_data;
	struct s5k_ctx *ctx = isf->ctx;
	ssize_t r;
	size_t new_size = *ppos + count;

	if (new_size)
		new_size++;

	mutex_lock(&ctx->lock);

	if (*ppos > isf->next_size) {
		r = -EINVAL;
		goto unlock;
	}

	if (ALIGN(new_size, 0x1000) != isf->next_allocated) {
		char *next = krealloc(isf->next, ALIGN(new_size, 0x1000), GFP_KERNEL);
		if (!next && new_size) {
			r = -ENOMEM;
			goto unlock;
		}

		isf->next = next;
		isf->next_allocated = ALIGN(new_size, 0x1000);
	}

	r = simple_write_to_buffer(isf->next, isf->next_allocated, ppos, user_buf, count);
	isf->next_size = *ppos;
	WARN_ON(isf->next_size >= isf->next_allocated);
	if (isf->next)
		isf->next[isf->next_size] = 0;

unlock:
	mutex_unlock(&ctx->lock);
	return r;
}

static int s5k_init_seq_release(struct inode *inode, struct file *file)
{
	struct s5k_init_seq_file *isf = file->private_data;
	struct s5k_ctx *ctx = isf->ctx;

	mutex_lock(&ctx->lock);
	kfree(isf->cur);
	isf->cur = NULL;
	if (isf->next)
		isf->cur = kmemdup(isf->next, isf->next_size, GFP_KERNEL);
	mutex_unlock(&ctx->lock);
	return 0;
}

static const struct file_operations s5k_init_seq_fops = {
	.owner	 = THIS_MODULE,
	.open	 = simple_open,
	.llseek = default_llseek,
	.release = s5k_init_seq_release,
	.read = s5k_init_seq_read,
	.write = s5k_init_seq_write,
};
#endif

static void s5k_free_regulators(void *param)
{
	struct s5k_ctx *ctx = param;

	regulator_bulk_free(ctx->num_vregs, ctx->vregs);
}

static int s5k_probe(struct i2c_client *client)
{
	const struct s5k_info *info = of_device_get_match_data(&client->dev);
	struct device *dev = &client->dev;
	struct s5k_ctx *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);
	ctx->dev = dev;
	ctx->info = info;
	ctx->mclk_rate = S5K_DEFAULT_MCLK;

	ctx->rmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR_OR_NULL(ctx->rmap))
		return dev_err_probe(dev, PTR_ERR(ctx->rmap) ?: -ENODATA,
				"failed to initialize cci\n");

	ret = of_regulator_bulk_get_all(dev, dev->of_node, &ctx->vregs);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ctx->num_vregs = ret;

	ret = devm_add_action_or_reset(dev, s5k_free_regulators, ctx);
	if (ret)
		return ret;

	ctx->rst = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->rst))
		return dev_err_probe(dev, PTR_ERR(ctx->rst),
				"failed to get reset gpio");

	ctx->mclk = devm_clk_get(ctx->dev, NULL);
	if (IS_ERR(ctx->mclk))
		return dev_err_probe(dev, PTR_ERR(ctx->mclk),
				"failed to get mclk\n");

	ret = s5k_check_hwcfg(ctx);
	if (ret)
		return ret;

	ret = s5k_power_on(ctx);
	if (ret)
		return ret;

	ret = s5k_probe_sensor(ctx);
	if (ret)
		goto err_power_off;

	v4l2_i2c_subdev_init(&ctx->sd, client, &s5k_subdev_ops);

	snprintf(ctx->sd.name, sizeof(ctx->sd.name),
		 "%s %s", ctx->info->model, dev_name(dev));

	ctx->sd.internal_ops = &s5k_internal_ops;
	ctx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	ctx->sd.entity.ops = &s5k_subdev_entity_ops;
	ctx->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ctx->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ctx->sd.entity, 1, &ctx->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto err_handler_free;
	}

	/* Put ctrl handler and subdev ops under same lock to simplify things */
	ctx->sd.state_lock = ctx->chdl.lock = &ctx->lock;

	ret = v4l2_subdev_init_finalize(&ctx->sd);
	if (ret)
		goto err_entity_cleanup;

	mutex_lock(&ctx->lock);
	s5k_update_controls(ctx);
	mutex_unlock(&ctx->lock);

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&ctx->sd);
	if (ret) {
		dev_err(dev, "failed to register subdev: %d\n", ret);
		goto err_rpm_disable;
	}

	pm_runtime_put_autosuspend(dev);

#ifdef S5K_DEBUG_IFACE
	ctx->dbg_pre_stream.ctx = ctx->dbg_pre_init.ctx = ctx;
	ctx->dfs_dir = debugfs_create_dir(ctx->sd.name, v4l2_debugfs_root());
	debugfs_create_u8("force_binh", 0644, ctx->dfs_dir, &ctx->force_binh);
	debugfs_create_u8("force_binv", 0644, ctx->dfs_dir, &ctx->force_binv);
	debugfs_create_u8("force_scaling", 0644, ctx->dfs_dir, &ctx->force_ratio);
	debugfs_create_u32("init_delay_us", 0644, ctx->dfs_dir, &ctx->dbg_init_delay_us);
	debugfs_create_u16("reg", 0644, ctx->dfs_dir, &ctx->dbg_reg);
	debugfs_create_u16("reg_cnt", 0644, ctx->dfs_dir, &ctx->dbg_reg_cnt);
	debugfs_create_devm_seqfile(ctx->dev, "reg_read", ctx->dfs_dir, s5k_debug_reg_read);
	debugfs_create_file("reg_write", 0200, ctx->dfs_dir, ctx, &s5k_debug_reg_write_fops);
	debugfs_create_file("pre_init_seq", 0644, ctx->dfs_dir,
			&ctx->dbg_pre_init, &s5k_init_seq_fops);
	debugfs_create_file("pre_stream_seq", 0644, ctx->dfs_dir,
			&ctx->dbg_pre_stream, &s5k_init_seq_fops);
#endif

	return 0;

err_rpm_disable:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);

	v4l2_subdev_cleanup(&ctx->sd);

err_entity_cleanup:
	media_entity_cleanup(&ctx->sd.entity);

err_handler_free:
	v4l2_ctrl_handler_free(&ctx->chdl);

err_power_off:
	s5k_power_off(ctx);

	return ret;
}


static void s5k_remove(struct i2c_client *client)
{
	struct s5k_ctx *ctx = dev_to_ctx(&client->dev);

	v4l2_async_unregister_subdev(&ctx->sd);
	v4l2_subdev_cleanup(&ctx->sd);
	media_entity_cleanup(&ctx->sd.entity);
	v4l2_ctrl_handler_free(&ctx->chdl);

	pm_runtime_disable(ctx->dev);
	if (!pm_runtime_status_suspended(ctx->dev))
		s5k_power_off(ctx);
	pm_runtime_set_suspended(ctx->dev);
	pm_runtime_dont_use_autosuspend(ctx->dev);

#ifdef S5K_DEBUG_IFACE
	debugfs_remove_recursive(ctx->dfs_dir);
	kfree(ctx->dbg_pre_stream.cur);
	kfree(ctx->dbg_pre_init.cur);
	kfree(ctx->dbg_pre_stream.next);
	kfree(ctx->dbg_pre_init.next);
#endif
}

static int __maybe_unused s5k_runtime_resume(struct device *dev)
{
	return s5k_power_on(dev_to_ctx(dev));
}

static int __maybe_unused s5k_runtime_suspend(struct device *dev)
{
	s5k_power_off(dev_to_ctx(dev));

	return 0;
}

static const struct dev_pm_ops s5k_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k_runtime_suspend, s5k_runtime_resume, NULL)
};

static const struct of_device_id s5k_of_match[] = {
	{ .compatible = "samsung,s5k2p6", &s5k2p6_sensor_info },
	{ .compatible = "samsung,s5k2x7", &s5k2x7_sensor_info },
	{ .compatible = "samsung,s5k3l8-untested", &s5k3l8_sensor_info },
	{ .compatible = "samsung,s5k3p8-untested", &s5k3p8_sensor_info },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k_of_match);

static struct i2c_driver s5k_i2c_driver = {
	.driver = {
		.name		= "s5kxxx",
		.of_match_table	= of_match_ptr(s5k_of_match),
		.pm		= pm_ptr(&s5k_pm_ops),
	},
	.probe	= s5k_probe,
	.remove = s5k_remove,
};

module_i2c_driver(s5k_i2c_driver);

MODULE_DESCRIPTION("Samsung S5Kxxx (Draft SMIA) RAW Bayer sensor driver");
MODULE_LICENSE("GPL");
