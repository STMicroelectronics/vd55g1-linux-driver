// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for VD55G1 global shutter sensor family driver
 *
 * Copyright (C) 2023 STMicroelectronics SA
 */

#define TRACE(msg, ...) (printk("%s: " msg " (%s:%i)\n", module_name(THIS_MODULE), ##__VA_ARGS__, __func__, __LINE__))

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <asm/unaligned.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Backward compatibility */
#include <linux/version.h>

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
/*
 * Warning : CCI_REGxy_LE definitions doesn't fit exactly with v4l2-cci.h .
 * In fact endianness is managed directly in vd55g1_read/write() functions.
 */
#include <linux/bitfield.h>
#define CCI_REG_ADDR_MASK		GENMASK(15, 0)
#define CCI_REG_WIDTH_SHIFT		16
#define CCI_REG_ADDR(x)			FIELD_GET(CCI_REG_ADDR_MASK, x)
#define CCI_REG8(x)			((1 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG16_LE(x)			((2 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG32_LE(x)			((4 << CCI_REG_WIDTH_SHIFT) | (x))
#else
#include <media/v4l2-cci.h>
#endif

#if KERNEL_VERSION(5, 18, 0) > LINUX_VERSION_CODE
#define MIPI_CSI2_DT_RAW8	0x2a
#define MIPI_CSI2_DT_RAW10	0x2b
#else
#include <media/mipi-csi2.h>
#endif

#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
#define HZ_PER_MHZ		1000000UL
#define MEGA			1000000UL
#else
#include <linux/units.h>
#endif

/* Register Map */
#define VD55G1_REG_MODEL_ID				CCI_REG32_LE(0x0000)
#define VD55G1_MODEL_ID					0x53354731
#define VD55G1_REG_REVISION				CCI_REG16_LE(0x0004)
#define VD55G1_REVISION_CUT_1				0x1010
#define VD55G1_REVISION_CUT_2				0x2020
#define VD55G1_REG_FWPATCH_REVISION			CCI_REG16_LE(0x0012)
#define VD55G1_REG_FWPATCH_START_ADDR			CCI_REG8(0x2000)
#define VD55G1_REG_SYSTEM_FSM				CCI_REG8(0x001c)
#define VD55G1_SYSTEM_FSM_READY_TO_BOOT			0x01
#define VD55G1_SYSTEM_FSM_SW_STBY			0x02
#define VD55G1_SYSTEM_FSM_STREAMING			0x03
#define VD55G1_REG_TEMPERATURE				CCI_REG16_LE(0x003c)
#define VD55G1_REG_BOOT					CCI_REG8(0x0200)
#define VD55G1_BOOT_BOOT				1
#define VD55G1_BOOT_PATCH_SETUP				2
#define VD55G1_REG_STBY					CCI_REG8(0x0201)
#define VD55G1_STBY_START_STREAM			1
#define VD55G1_STBY_THSENS_READ				4
#define VD55G1_REG_STREAMING				CCI_REG8(0x0202)
#define VD55G1_STREAMING_STOP_STREAM			1
#define VD55G1_REG_EXT_CLOCK				CCI_REG32_LE(0x0220)
#define VD55G1_REG_LINE_LENGTH				CCI_REG16_LE(0x0300)
#define VD55G1_REG_ORIENTATION				CCI_REG8(0x0302)
#define VD55G1_REG_FORMAT_CTRL				CCI_REG8(0x030a)
#define VD55G1_REG_OIF_CTRL				CCI_REG16_LE(0x030c)
#define VD55G1_REG_ISL_ENABLE				CCI_REG16_LE(0x326)
#define VD55G1_REG_OIF_IMG_CTRL				CCI_REG8(0x030f)
#define VD55G1_REG_MIPI_DATA_RATE			CCI_REG32_LE(0x0224)
#define VD55G1_REG_PATGEN_CTRL				CCI_REG16_LE(0x0304)
#define VD55G1_PATGEN_TYPE_SHIFT			4
#define VD55G1_PATGEN_ENABLE				BIT(0)
#define VD55G1_REG_MANUAL_ANALOG_GAIN			CCI_REG8(0x0501)
#define VD55G1_REG_MANUAL_COARSE_EXPOSURE		CCI_REG16_LE(0x0502)
#define VD55G1_REG_MANUAL_DIGITAL_GAIN			CCI_REG16_LE(0x0504)
#define VD55G1_REG_APPLIED_COARSE_EXPOSURE		CCI_REG16_LE(0x00e8)
#define VD55G1_REG_APPLIED_ANALOG_GAIN			CCI_REG16_LE(0x00ea)
#define VD55G1_REG_APPLIED_DIGITAL_GAIN			CCI_REG16_LE(0x00ec)
#define VD55G1_REG_AE_FORCE_COLDSTART			CCI_REG16_LE(0x0308)
#define VD55G1_REG_AE_COLDSTART_EXP_TIME		CCI_REG32_LE(0x0374) //TODO
#define VD55G1_REG_READOUT_CTRL				CCI_REG8(0x052e)
#define VD55G1_REG_DARKCAL_CTRL				CCI_REG8(0x032a)
#define VD55G1_DARKCAL_BYPASS				0
#define VD55G1_DARKCAL_AUTO				1
#define VD55G1_REG_DUSTER_CTRL				CCI_REG8(0x03ea)
#define VD55G1_DUSTER_ENABLE				BIT(0)
#define VD55G1_DUSTER_DISABLE				0
#define VD55G1_DUSTER_DYN_ENABLE			BIT(1)
#define VD55G1_DUSTER_RING_ENABLE			BIT(4)
#define VD55G1_REG_AE_TARGET_PERCENTAGE			CCI_REG8(0x0486)
#define VD55G1_REG_VT_CTRL				CCI_REG8(0x0309)
#define VD55G1_VTSLAVE_GPIO				0
#define VD55G1_REG_ERROR_CODE				CCI_REG16_LE(0x0010)
#define VD55G1_REG_NEXT_CTX				CCI_REG16_LE(0x03e4)
#define VD55G1_REG_EXPOSURE_USE_CASES			CCI_REG8(0x0312)
#define VD55G1_EXPOSURE_USE_CASES_MULTI_CONTEXT		BIT(2)

#define VD55G1_REG_EXP_MODE(ctx) \
	CCI_REG8(0x0500 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_EXP_MODE_AUTO				0
#define VD55G1_EXP_MODE_FREEZE				1
#define VD55G1_EXP_MODE_MANUAL				2
#define VD55G1_REG_FRAME_LENGTH(ctx) \
	CCI_REG32_LE(0x050c + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_X_START(ctx) \
	CCI_REG16_LE(0x0514 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_X_WIDTH(ctx) \
	CCI_REG16_LE(0x0516 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_Y_START(ctx) \
	CCI_REG16_LE(0x0510 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_Y_HEIGHT(ctx) \
	CCI_REG16_LE(0x0512 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_GPIO_0_CTRL(ctx) \
	CCI_REG8(0x051d + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_DARKCAL_PEDESTAL(ctx) \
	CCI_REG16_LE(0x0526 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_CTX_REPEAT_COUNT(ctx) \
	CCI_REG16_LE(0x03dc + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_VT_MODE(ctx) \
	CCI_REG8(0x0536 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_VT_MODE_NORMAL 0
#define VD55G1_VT_MODE_SUBTRACTION 1
#define VD55G1_REG_MASK_FRAME_CTRL(ctx) \
	CCI_REG8(0x0537 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_MASK_FRAME_CTRL_OUTPUT 0
#define VD55G1_MASK_FRAME_CTRL_MASK 1
#define VD55G1_REG_EXPOSURE_INSTANCE(ctx) \
	CCI_REG32_LE(0x52D + VD55G1_CTX_OFFSET * (ctx))

#define VD55G1_WIDTH					804
#define VD55G1_HEIGHT					704
#define VD55G1_DEFAULT_MODE				1
#define VD55G1_WRITE_MULTIPLE_CHUNK_MAX			16
#define VD55G1_NB_GPIOS					4
#define VD55G1_NB_POLARITIES				3
#define VD55G1_VBLANK_MIN				86
#define VD55G1_FRAME_LENGTH_DEF				1860 /* 60 fps */
#define VD55G1_TIMEOUT_MS				500
#define VD55G1_MEDIA_BUS_FMT_DEF			MEDIA_BUS_FMT_Y8_1X8
#define VD55G1_DARKCAL_PEDESTAL_DEF			0x40
#define VD55G1_DGAIN_DEF				256
#define VD55G1_AGAIN_DEF				19
#define VD55G1_VBLANK_DEF				4000 //TODO must be always 60fps, use default framelength instead ?
#define VD55G1_EXPO_MAX_TERM				64
#define VD55G1_EXPO_DEF					500
#define VD55G1_MIN_LINE_LENGTH				1128
#define VD55G1_MIN_LINE_LENGTH_SUB			1344
#define VD55G1_MAX_LINE_LENGTH				0xffff
#define VD55G1_MIPI_MARGIN				900
#define VD55G1_PCLK_DIVISOR				5
#define VD55G1_CTX_OFFSET				0x50

#define V4L2_CID_TEMPERATURE			(V4L2_CID_USER_BASE | 0x1020)
#define V4L2_CID_DARKCAL_PEDESTAL		(V4L2_CID_USER_BASE | 0x1021)
#define V4L2_CID_SLAVE_MODE				(V4L2_CID_USER_BASE | 0x1022)
#if KERNEL_VERSION(6, 2, 0) > LINUX_VERSION_CODE
#define V4L2_CID_HDR_SENSOR_MODE		(V4L2_CID_USER_BASE | 0x1004)
#endif

#include "vd55g1_patch.c"

static const char * const vd55g1_tp_menu[] = {
	"Disabled",
	"Dgrey",
	"PN28",
};

static const s64 vd55g1_ev_bias_menu[] = {
	-3000, -2500, -2000, -1500, -1000, -500,
	    0,
	  500,  1000,  1500,  2000,  2500, 3000,
};

static const char * const vd55g1_hdr_mode_menu[] = {
	"No HDR",
	/*
	 * This mode acquires 2 frames on the sensor, the first one is ditched
	 * out and only used for auto exposure data, the second one is output to
	 * the host
	 */
	"Internal subtraction",
};

static const char * const vd55g1_supply_name[] = {
	"vcore",
	"vddio",
	"vana",
};

/* Will be filled on device tree parse */
static u64 link_freq[1];

enum vd55g1_hdr_mode {
	VD55G1_NO_HDR,
	VD55G1_HDR_SUB,
};

enum vd55g1_bin_mode {
	VD55G1_BIN_MODE_NORMAL,
	VD55G1_BIN_MODE_DIGITAL_X2,
	VD55G1_BIN_MODE_DIGITAL_X4,
};

enum vd55g1_gpio_mode {
	VD55G1_GPIO_MODE_FSYNC_OUT = 0x00,
	VD55G1_GPIO_MODE_IN = 0x01,
	VD55G1_GPIO_MODE_STROBE = 0x02,
	VD55G1_GPIO_MODE_VTSLAVE= 0x0a,
};

struct vd55g1_mode {
	u32 width;
	u32 height;
	enum vd55g1_bin_mode bin_mode;
	struct v4l2_rect crop;
};

struct vd55g1_fmt_desc {
	u32 code;
	u8 bpp;
	u8 data_type;
};

static const struct vd55g1_fmt_desc vd55g1_mbus_codes[] = {
	{
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.bpp = 8,
		.data_type = MIPI_CSI2_DT_RAW8,
	},
	{
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.bpp = 10,
		.data_type = MIPI_CSI2_DT_RAW10,
	},
};

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
/* Big endian register addresses and 8b, 16b or 32b little endian values. */
static const struct regmap_config vd55g1_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
};
#endif


static const struct vd55g1_mode vd55g1_supported_modes[] = {
	{
		.width = VD55G1_WIDTH,
		.height = VD55G1_HEIGHT,
	},
	{
		.width = 800,
		.height = VD55G1_HEIGHT,
	},
	{
		.width = 800,
		.height = 600,
	},
	{
		.width = 640,
		.height = 480,
	},
	{
		.width = 320,
		.height = 240,
	},
};

enum vd55g1_expo_state {
	VD55G1_EXP_AUTO,
	VD55G1_EXP_FREEZE,
	VD55G1_EXP_MANUAL,
	VD55G1_EXP_SINGLE_STEP,
	VD55G1_EXP_BYPASS,
};

struct hblank_limits {
	u16 min;
	u16 max;
};

struct vd55g1 {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regulator_bulk_data supplies[ARRAY_SIZE(vd55g1_supply_name)];
	struct gpio_desc *reset_gpio;
	struct clk *xclk;
	struct regmap *regmap;
	u32 xclk_freq;
	u16 oif_ctrl;
	enum vd55g1_gpio_mode gpios[VD55G1_NB_GPIOS];
	bool ext_vt_sync;
	unsigned long ext_leds_mask;
	int data_rate_in_mbps;
	u32 pixel_clock;
	struct {
		u16 expo;
		u16 dgain;
		u8 again;
	} cold_start;
	/* Lock to protect all members below */
	struct mutex lock;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct {
		struct v4l2_ctrl *hflip_ctrl;
		struct v4l2_ctrl *vflip_ctrl;
	};
	struct v4l2_ctrl *patgen_ctrl;
	struct {
		struct v4l2_ctrl *ae_ctrl;
		struct v4l2_ctrl *expo_ctrl;
		struct v4l2_ctrl *again_ctrl;
		struct v4l2_ctrl *dgain_ctrl;
	};
	struct v4l2_ctrl *ae_lock_ctrl;
	struct v4l2_ctrl *ae_bias_ctrl;
	struct v4l2_ctrl *darkcal_ctrl;
	struct v4l2_ctrl *slave_ctrl;
	struct v4l2_ctrl *led_ctrl;
	struct v4l2_ctrl *hdr_ctrl;
	bool streaming;
	struct v4l2_mbus_framefmt active_fmt;
	struct v4l2_rect active_crop;
};

static inline struct vd55g1 *to_vd55g1(struct v4l2_subdev *sd)
{
#if KERNEL_VERSION(6, 2, 0) > LINUX_VERSION_CODE
	return container_of(sd, struct vd55g1, sd);
#else
	return container_of_const(sd, struct vd55g1, sd);
#endif
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct vd55g1,
		ctrl_handler)->sd;
}

static u8 get_bpp_by_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vd55g1_mbus_codes); i++) {
		if (vd55g1_mbus_codes[i].code == code)
			return vd55g1_mbus_codes[i].bpp;
	}
	/* Should never happen */
	WARN(1, "Unsupported code %d. default to 8 bpp", code);
	return 8;
}

static u8 get_data_type_by_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vd55g1_mbus_codes); i++) {
		if (vd55g1_mbus_codes[i].code == code)
			return vd55g1_mbus_codes[i].data_type;
	}
	/* Should never happen */
	WARN(1, "Unsupported code %d. default to MIPI_CSI2_DT_RAW8 data type",
	     code);
	return MIPI_CSI2_DT_RAW8;
}

static s32 get_pixel_rate(struct vd55g1 *sensor)
{
	return div64_u64((u64)sensor->data_rate_in_mbps,
			 get_bpp_by_code(sensor->active_fmt.code));
}

static s32 get_min_line_length(struct vd55g1 *sensor)
{
	u32 mipi_req_line_time;
	u32 mipi_req_line_length;
	u32 min_line_length;

	/* MIPI required time */
	mipi_req_line_time = (sensor->active_crop.width *
			       get_bpp_by_code(sensor->active_fmt.code) +
			       VD55G1_MIPI_MARGIN) / (sensor->data_rate_in_mbps / MEGA);
	mipi_req_line_length = mipi_req_line_time * sensor->pixel_clock / HZ_PER_MHZ;

	/* Absolute time required for ADCs to convert pixels */
	min_line_length = VD55G1_MIN_LINE_LENGTH;
	if (sensor->hdr_ctrl->val == VD55G1_HDR_SUB)
		min_line_length = VD55G1_MIN_LINE_LENGTH_SUB;

	/* Respect both constraint */
	return max(min_line_length, mipi_req_line_length);
}

static struct hblank_limits get_hblank_limits(struct vd55g1 *sensor)
{
	struct hblank_limits limits;
	struct v4l2_rect crop = sensor->active_crop;

	limits.min = get_min_line_length(sensor) - crop.width;
	limits.max = VD55G1_MAX_LINE_LENGTH - crop.width;

	return limits;
}


#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
static int vd55g1_read(struct vd55g1 *sensor, u32 reg, u32 *val, int *err)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int len = (reg >> CCI_REG_WIDTH_SHIFT) & 7;
	u8 buf[4];
	int ret;

	if (err && *err)
		return *err;

	reg = reg & CCI_REG_ADDR_MASK;

	ret = regmap_bulk_read(sensor->regmap, reg, buf, len);
	if (ret) {
		dev_err(&client->dev, "%s: Error reading reg 0x%4x: %d\n",
			__func__, reg, ret);
		goto out;
	}

	switch (len) {
	case 1:
		*val = buf[0];
		break;
	case 2:
		*val = get_unaligned_le16(buf);
		break;
	case 4:
		*val = get_unaligned_le32(buf);
		break;
	default:
		dev_err(&client->dev,
			"%s: Error invalid reg-width %u for reg 0x%04x\n",
			__func__, len, reg);
		ret = -EINVAL;
		break;
	}

out:
	if (ret && err)
		*err = ret;

	return ret;
}

static int vd55g1_write(struct vd55g1 *sensor, u32 reg, u32 val, int *err)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int len = (reg >> CCI_REG_WIDTH_SHIFT) & 7;
	u8 buf[4];
	int ret;

	if (err && *err)
		return *err;

	reg = reg & CCI_REG_ADDR_MASK;
	switch (len) {
	case 1:
		buf[0] = val;
		break;
	case 2:
		put_unaligned_le16(val, buf);
		break;
	case 4:
		put_unaligned_le32(val, buf);
		break;
	default:
		dev_err(&client->dev,
			"%s: Error invalid reg-width %u for reg 0x%04x\n",
			__func__, len, reg);
		ret = -EINVAL;
		goto out;
	}

	ret = regmap_bulk_write(sensor->regmap, reg, buf, len);
	if (ret)
		dev_err(&client->dev, "%s: Error writing reg 0x%4x: %d\n",
			__func__, reg, ret);

out:
	if (ret && err)
		*err = ret;

	return ret;
}
#else
#define vd55g1_read(sensor, reg, val, err) \
	cci_read((sensor)->regmap, reg, (u64 *)val, err)

#define vd55g1_write(sensor, reg, val, err) \
	cci_write((sensor)->regmap, reg, (u64)val, err)
#endif

static int vd55g1_write_array(struct vd55g1 *sensor, u32 reg, unsigned int len,
			      const u8 *array, int *err)
{
	unsigned int chunk_sz = 1024;
	unsigned int sz;
	int ret;

	if (err && *err)
		return *err;

	/*
	 * This loop isn't necessary but in certains conditions (platforms, cpu
	 * load, etc.) it has been observed that the bulk write could timeout.
	 */
	while (len) {
		sz = min(len, chunk_sz);
		ret = regmap_bulk_write(sensor->regmap, reg, array, sz);
		if (ret < 0)
			goto out;
		len -= sz;
		reg += sz;
		array += sz;
	}

out:
	if (ret && err)
		*err = ret;

	return ret;
}

static int vd55g1_poll_reg(struct vd55g1 *sensor, u32 reg, u8 poll_val,
			   int *err)
{
	unsigned int val = 0;
	int ret;

	if (err && *err)
		return *err;

	ret = regmap_read_poll_timeout(sensor->regmap, CCI_REG_ADDR(reg), val,
				       (val == poll_val), 2000,
				       500 * USEC_PER_MSEC);

	if (ret && err)
		*err = ret;

	return ret;
}

static int vd55g1_wait_state(struct vd55g1 *sensor, int state, int *err)
{
	return vd55g1_poll_reg(sensor, VD55G1_REG_SYSTEM_FSM, state, err);
}

static int vd55g1_get_regulators(struct vd55g1 *sensor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vd55g1_supply_name); i++)
		sensor->supplies[i].supply = vd55g1_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       ARRAY_SIZE(vd55g1_supply_name),
				       sensor->supplies);
}

static int vd55g1_prepare_clock_tree(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u64 mipi_freq = link_freq[0];
	u32 sys_clk, mipi_div, pixel_div;
	int ret = 0;

	if (sensor->xclk_freq < 6 * HZ_PER_MHZ ||
	    sensor->xclk_freq > 27 * HZ_PER_MHZ) {
		dev_err(&client->dev,
			"Only 6Mhz-27Mhz clock range supported. Provided %lu MHz\n",
			sensor->xclk_freq / HZ_PER_MHZ);
		return -EINVAL;
	}

	if (mipi_freq < 250 * HZ_PER_MHZ ||
	    mipi_freq > 1200 * HZ_PER_MHZ) {
		dev_err(&client->dev,
			"Only 250Mhz-1200Mhz link frequency range supported. Provided %llu MHz\n",
			mipi_freq / HZ_PER_MHZ);
		return -EINVAL;
	}

	if (mipi_freq < 300 * HZ_PER_MHZ)
		mipi_div = 4;
	else if (mipi_freq < 600 * HZ_PER_MHZ)
		mipi_div = 2;
	else
		mipi_div = 1;

	sys_clk = mipi_freq * mipi_div;

	if (sys_clk < 780 * HZ_PER_MHZ)
		pixel_div = 5;
	else if (sys_clk < 900 * HZ_PER_MHZ)
		pixel_div = 6;
	else
		pixel_div = 8;

	sensor->pixel_clock = sys_clk / pixel_div;
	/* Frequency to data rate is 1:1 ratio for MIPI */
	sensor->data_rate_in_mbps = mipi_freq;

	return ret;
}

static int vd55g1_update_patgen(struct vd55g1 *sensor, u32 patgen_index)
{
	static const u8 index2val[] = {
		0x0, 0x22, 0x28
	};
	u32 pattern = index2val[patgen_index];
	u32 reg = pattern << VD55G1_PATGEN_TYPE_SHIFT;
	u8 darkcal = VD55G1_DARKCAL_AUTO;
	u8 duster = VD55G1_DUSTER_RING_ENABLE | VD55G1_DUSTER_DYN_ENABLE |
		    VD55G1_DUSTER_ENABLE;
	int ret = 0;

	if (pattern != 0) {
		reg |= VD55G1_PATGEN_ENABLE;
		/*
		 * Take care of dark calibaration and duster to not mess up the
		 * test pattern output.
		 */
		darkcal = VD55G1_DARKCAL_BYPASS;
		duster = VD55G1_DUSTER_DISABLE;
	}

	vd55g1_write(sensor, VD55G1_REG_DARKCAL_CTRL, darkcal, &ret);
	vd55g1_write(sensor, VD55G1_REG_DUSTER_CTRL, duster, &ret);
	if (ret)
		return ret;

	return vd55g1_write(sensor, VD55G1_REG_PATGEN_CTRL, reg, NULL);
}

static int vd55g1_update_expo_cluster(struct vd55g1 *sensor, bool is_auto)
{
	enum vd55g1_expo_state expo_state = is_auto ? VD55G1_EXP_MODE_AUTO :
						      VD55G1_EXP_MODE_MANUAL;
	int ret = 0;

	if (sensor->ae_ctrl->is_new)
		vd55g1_write(sensor, VD55G1_REG_EXP_MODE(0), expo_state, &ret);

	if (sensor->hdr_ctrl->val == VD55G1_HDR_SUB &&
	    sensor->hdr_ctrl->is_new) {
		ret = vd55g1_write(sensor, VD55G1_REG_EXP_MODE(1),
				       VD55G1_EXP_BYPASS, NULL);
		if (ret)
			return ret;
	}

	if (!is_auto && sensor->expo_ctrl->is_new)
		vd55g1_write(sensor, VD55G1_REG_MANUAL_COARSE_EXPOSURE,
			     sensor->expo_ctrl->val, &ret);

	if (!is_auto && sensor->again_ctrl->is_new)
		vd55g1_write(sensor, VD55G1_REG_MANUAL_ANALOG_GAIN,
			     sensor->again_ctrl->val, &ret);

	if (!is_auto && sensor->dgain_ctrl->is_new) {
		vd55g1_write(sensor, VD55G1_REG_MANUAL_DIGITAL_GAIN,
			     sensor->dgain_ctrl->val, &ret);
	}

	return ret;
}

static int vd55g1_lock_exposure(struct vd55g1 *sensor, u32 lock_val)
{
	bool ae_lock = lock_val & V4L2_LOCK_EXPOSURE;
	enum vd55g1_expo_state expo_state = ae_lock ? VD55G1_EXP_MODE_FREEZE :
						      VD55G1_EXP_MODE_AUTO;
	int ret = 0;

	if (sensor->ae_ctrl->val == V4L2_EXPOSURE_AUTO)
		vd55g1_write(sensor, VD55G1_REG_EXP_MODE(0), expo_state, &ret);

	return ret;
}

static int vd55g1_get_temp_stream_enable(struct vd55g1 *sensor, int *temp)
{
	u32 temperature;
	int ret;

	ret = vd55g1_read(sensor, VD55G1_REG_TEMPERATURE, &temperature, NULL);
	if (ret)
		return ret;

	/* temperature is signed 10 bits value. extend sign */
	temperature = (temperature << 6) >> 6;
	*temp = temperature;

	return 0;
}

static int vd55g1_get_temp_stream_disable(struct vd55g1 *sensor, int *temp)
{
	int ret;

	/* request temperature read */
	ret = vd55g1_write(sensor, VD55G1_REG_STBY,
			       VD55G1_STBY_THSENS_READ, NULL);
	if (ret)
		return ret;
	ret = vd55g1_poll_reg(sensor, VD55G1_REG_STBY, 0, NULL);
	if (ret)
		return ret;

	return vd55g1_get_temp_stream_enable(sensor, temp);
}

static int vd55g1_get_temp(struct vd55g1 *sensor, int *temp)
{
	*temp = 0;
	if (sensor->streaming)
		return vd55g1_get_temp_stream_enable(sensor, temp);
	else
		return vd55g1_get_temp_stream_disable(sensor, temp);
}

static int vd55g1_read_expo_cluster(struct vd55g1 *sensor, bool force_cur_val)
{
	int exposure = 0;
	int again = 0;
	int dgain = 0;
	int ret = 0;

	/*
	 * When 'force_cur_val' is enabled, save the ctrl value in 'cur.val'
	 * instead of the normal 'val', this is used during poweroff to cache
	 * volatile ctrls and enable coldstart.
	 */
	vd55g1_read(sensor, VD55G1_REG_APPLIED_COARSE_EXPOSURE, &exposure,
		    &ret);
	vd55g1_read(sensor, VD55G1_REG_APPLIED_ANALOG_GAIN, &again, &ret);
	vd55g1_read(sensor, VD55G1_REG_APPLIED_DIGITAL_GAIN, &dgain, &ret);
	if (ret)
		return ret;

	sensor->expo_ctrl->cur.val = exposure;
	sensor->again_ctrl->cur.val = again;
	sensor->dgain_ctrl->cur.val = dgain;
#if 0
	if (force_cur_val) {
		sensor->expo_ctrl->cur.val = exposure;
		sensor->again_ctrl->cur.val = again;
		sensor->dgain_ctrl->cur.val = dgain;
	} else {
		sensor->expo_ctrl->val = exposure;
		sensor->again_ctrl->val = again;
		sensor->dgain_ctrl->val = dgain;
	}
	TRACE("");
	ret |= __v4l2_ctrl_s_ctrl(sensor->expo_ctrl, exposure);
	TRACE("");
	ret |= __v4l2_ctrl_s_ctrl(sensor->again_ctrl, again);
	ret |= __v4l2_ctrl_s_ctrl(sensor->dgain_ctrl, dgain);
#endif

	if (ret)
		return -EINVAL; //TODO better
	return ret;
}

static int vd55g1_update_frame_length(struct vd55g1 *sensor, unsigned int frame_length)
{
	int ret = 0;

	if (sensor->hdr_ctrl->val == VD55G1_HDR_SUB) {
		vd55g1_write(sensor, VD55G1_REG_FRAME_LENGTH(1),
				 frame_length, &ret);
		if (ret)
			return ret;
	}

	return vd55g1_write(sensor, VD55G1_REG_FRAME_LENGTH(0),
			    frame_length, NULL);
}

static int vd55g1_update_exposure_target(struct vd55g1 *sensor, int index)
{
	/*
	 * Find auto exposure target with: default target exposure * 2^EV
	 * Defaut target exposure being 27 for the sensor.
	 */
	static const unsigned int index2exposure_target[] = {
		3, 5, 7, 10, 14, 19, 27, 38, 54, 76, 108, 153, 216,
	};

	int exposure_target = index2exposure_target[index];
	return vd55g1_write(sensor, VD55G1_REG_AE_TARGET_PERCENTAGE,
				exposure_target, NULL);
}

static int vd55g1_apply_reset(struct vd55g1 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	return vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_READY_TO_BOOT, NULL);
}

static void vd55g1_update_img_pad_format(struct vd55g1 *sensor,
				 const struct vd55g1_mode *mode,
				 u32 code,
				 struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->field = V4L2_FIELD_NONE;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int vd55g1_update_hdr_mode(struct vd55g1 *sensor)
{
	int ret = 0;

	switch (sensor->hdr_ctrl->val) {
	case VD55G1_NO_HDR:
		vd55g1_write(sensor, VD55G1_REG_EXPOSURE_USE_CASES, 0,
				 &ret);
		vd55g1_write(sensor, VD55G1_REG_NEXT_CTX, 0x0, &ret);
		vd55g1_write(sensor, VD55G1_REG_CTX_REPEAT_COUNT(0), 1,
				 &ret);
		vd55g1_write(sensor, VD55G1_REG_VT_MODE(0),
				 VD55G1_VT_MODE_NORMAL, &ret);
		vd55g1_write(sensor, VD55G1_REG_MASK_FRAME_CTRL(0),
				 VD55G1_MASK_FRAME_CTRL_OUTPUT, &ret);
		break;
	case VD55G1_HDR_SUB:
		vd55g1_write(sensor, VD55G1_REG_EXPOSURE_USE_CASES,
				 VD55G1_EXPOSURE_USE_CASES_MULTI_CONTEXT, &ret);
		vd55g1_write(sensor, VD55G1_REG_NEXT_CTX, 0x1, &ret);

		vd55g1_write(sensor, VD55G1_REG_CTX_REPEAT_COUNT(0), 1,
				 &ret);
		vd55g1_write(sensor, VD55G1_REG_VT_MODE(0),
				 VD55G1_VT_MODE_NORMAL, &ret);
		vd55g1_write(sensor, VD55G1_REG_MASK_FRAME_CTRL(0),
				 VD55G1_MASK_FRAME_CTRL_MASK, &ret);
		vd55g1_write(sensor, VD55G1_REG_EXPOSURE_INSTANCE(0), 0,
				 &ret);

		vd55g1_write(sensor, VD55G1_REG_CTX_REPEAT_COUNT(1), 1,
				 &ret);
		vd55g1_write(sensor, VD55G1_REG_VT_MODE(1),
				 VD55G1_VT_MODE_SUBTRACTION, &ret);
		vd55g1_write(sensor, VD55G1_REG_MASK_FRAME_CTRL(1),
				 VD55G1_MASK_FRAME_CTRL_OUTPUT, &ret);
		vd55g1_write(sensor, VD55G1_REG_EXPOSURE_INSTANCE(1), 1,
				 &ret);
		break;
	default:
		/* Should never happen */
		WARN(1, "Unsupported hdr mode %d", sensor->hdr_ctrl->val);
		ret = -EINVAL;
	}

	return ret;
}

static int vd55g1_set_framefmt(struct vd55g1 *sensor)
{
	const struct v4l2_rect *crop = &sensor->active_crop;
	enum vd55g1_bin_mode binning;
	int ret = 0;

	vd55g1_write(sensor, VD55G1_REG_FORMAT_CTRL,
			 get_bpp_by_code(sensor->active_fmt.code), &ret);
	vd55g1_write(sensor, VD55G1_REG_OIF_IMG_CTRL,
			 get_data_type_by_code(sensor->active_fmt.code), &ret);

	switch (crop->width / sensor->active_fmt.width) {
	case 1:
	default:
		binning = VD55G1_BIN_MODE_NORMAL;
		break;
	case 2:
		binning = VD55G1_BIN_MODE_DIGITAL_X2;
		break;
	}
	vd55g1_write(sensor, VD55G1_REG_READOUT_CTRL, binning, &ret);

	vd55g1_write(sensor, VD55G1_REG_X_START(0), crop->left, &ret);
	vd55g1_write(sensor, VD55G1_REG_X_WIDTH(0), crop->width, &ret);
	vd55g1_write(sensor, VD55G1_REG_Y_START(0), crop->top, &ret);
	vd55g1_write(sensor, VD55G1_REG_Y_HEIGHT(0), crop->height, &ret);

	vd55g1_write(sensor, VD55G1_REG_X_START(1), crop->left, &ret);
	vd55g1_write(sensor, VD55G1_REG_X_WIDTH(1), crop->width, &ret);
	vd55g1_write(sensor, VD55G1_REG_Y_START(1), crop->top, &ret);
	vd55g1_write(sensor, VD55G1_REG_Y_HEIGHT(1), crop->height, &ret);

	return ret;
}

//TODO rename update_gpio
static int vd55g1_write_gpios(struct vd55g1 *sensor, unsigned long gpio_mask)
{
	unsigned long io;
	u32 gpio_val;
	int ret = 0;

	for_each_set_bit(io, &gpio_mask, VD55G1_NB_GPIOS) {
		gpio_val = sensor->gpios[io];

		if (gpio_val == VD55G1_GPIO_MODE_VTSLAVE &&
		    !sensor->slave_ctrl->val)
			gpio_val = VD55G1_GPIO_MODE_IN;

		if (gpio_val == VD55G1_GPIO_MODE_STROBE &&
		    sensor->led_ctrl->val == V4L2_FLASH_LED_MODE_NONE) {
			gpio_val = VD55G1_GPIO_MODE_IN;
			if (sensor->hdr_ctrl->val == VD55G1_HDR_SUB) {
				/* Make its context 1 counterpart strobe too */
				ret = vd55g1_write(sensor, VD55G1_REG_GPIO_0_CTRL(1) + io,
						       gpio_val, NULL);
				if (ret)
					return ret;
			}
		}

		vd55g1_write(sensor, VD55G1_REG_GPIO_0_CTRL(0) + io, gpio_val,
			     &ret);
	}

	return ret;
}

static int vd55g1_stream_on(struct vd55g1 *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret = 0;

	vd55g1_write(sensor, VD55G1_REG_EXT_CLOCK, sensor->xclk_freq, &ret);

	/* configure ouput */
	vd55g1_write(sensor, VD55G1_REG_MIPI_DATA_RATE, sensor->data_rate_in_mbps, &ret);
	vd55g1_write(sensor, VD55G1_REG_OIF_CTRL, sensor->oif_ctrl, &ret);
	vd55g1_write(sensor, VD55G1_REG_ISL_ENABLE, 0, &ret);
	if (ret)
		goto err_rpm_put;

	ret = vd55g1_set_framefmt(sensor);
	if (ret)
		goto err_rpm_put;

	/* Setup default GPIO values; could be overridden by V4L2 ctrl setup */
	ret = vd55g1_write_gpios(sensor, GENMASK(VD55G1_NB_GPIOS - 1, 0));
	if (ret)
		return ret;

	/* Apply settings from V4L2 ctrls */
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	if (ret)
		return ret;

	/* start streaming */
	vd55g1_write(sensor, VD55G1_REG_STBY, VD55G1_STBY_START_STREAM, &ret);
	vd55g1_poll_reg(sensor, VD55G1_REG_STBY, 0, &ret);
	vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_STREAMING, &ret);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static int vd55g1_stream_off(struct vd55g1 *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret;

	/* Retrieve Expo cluster to enable coldstart of AE */
	ret = vd55g1_read_expo_cluster(sensor, true);

	ret = vd55g1_write(sensor, VD55G1_REG_STREAMING,
			       VD55G1_STREAMING_STOP_STREAM, NULL);
	if (ret)
		goto err_str_dis;

	ret = vd55g1_poll_reg(sensor, VD55G1_REG_STREAMING, 0, NULL);
	if (ret)
		goto err_str_dis;

	ret = vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_SW_STBY, NULL);

err_str_dis:
	if (ret)
		WARN(1, "Can't disable stream");
	pm_runtime_put(&client->dev);

	return ret;
}

static int vd55g1_patch(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u32 patch;
	int ret;

	ret = vd55g1_write_array(sensor, VD55G1_REG_FWPATCH_START_ADDR,
				 sizeof(patch_array), patch_array, NULL);
	if (ret)
		return ret;

	ret = vd55g1_write(sensor, VD55G1_REG_BOOT, VD55G1_BOOT_PATCH_SETUP,
			   NULL);
	if (ret)
		return ret;

	ret = vd55g1_poll_reg(sensor, VD55G1_REG_BOOT, 0, NULL);
	if (ret)
		return ret;

	vd55g1_read(sensor, VD55G1_REG_FWPATCH_REVISION, &patch, NULL);
	if (ret)
		return ret;

	if (patch != (VD55G1_FWPATCH_REVISION_MAJOR << 8) +
	    VD55G1_FWPATCH_REVISION_MINOR) {
		dev_err(&client->dev, "bad patch version expected %d.%d got %d.%d",
			VD55G1_FWPATCH_REVISION_MAJOR,
			VD55G1_FWPATCH_REVISION_MINOR,
			patch >> 8, patch & 0xff);
		return -ENODEV;
	}
	dev_dbg(&client->dev, "patch %d.%d applied", patch >> 8, patch & 0xff);

	return 0;
}

static int vd55g1_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vd55g1 *sensor = to_vd55g1(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (enable) {
#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock;
		}
#else
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto unlock;
#endif
		ret = vd55g1_stream_on(sensor);
		if (ret) {
			dev_err(&client->dev, "Failed to start streaming\n");
			pm_runtime_put_sync(&client->dev);
		}
	} else {
		vd55g1_stream_off(sensor);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	if (!ret)
		sensor->streaming = enable;

#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
unlock:
	mutex_unlock(&sensor->lock);

	if (!ret) {
		/* These settings cannot change during streaming */
		v4l2_ctrl_grab(sensor->hflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->vflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->patgen_ctrl, enable);
		v4l2_ctrl_grab(sensor->hdr_ctrl, enable);
		v4l2_ctrl_grab(sensor->hblank_ctrl, enable);
		if (sensor->ext_vt_sync)
			v4l2_ctrl_grab(sensor->slave_ctrl, enable);
	}
#else
	if (!ret) {
		sensor->streaming = enable;

		/* These settings cannot change during streaming */
		__v4l2_ctrl_grab(sensor->hflip_ctrl, enable);
		__v4l2_ctrl_grab(sensor->vflip_ctrl, enable);
		__v4l2_ctrl_grab(sensor->patgen_ctrl, enable);
		__v4l2_ctrl_grab(sensor->hdr_ctrl, enable);
		__v4l2_ctrl_grab(sensor->hblank_ctrl, enable);
		if (sensor->ext_vt_sync) {
			__v4l2_ctrl_grab(sensor->slave_ctrl, enable);
		}
	}

unlock:
	mutex_unlock(&sensor->lock);
#endif

	return ret;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
#else
static int vd55g1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
#endif
{
	struct vd55g1 *sensor = to_vd55g1(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = sensor->active_crop;
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = VD55G1_WIDTH;
		sel->r.height = VD55G1_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int vd55g1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	if (code->index >= ARRAY_SIZE(vd55g1_mbus_codes))
		return -EINVAL;

	code->code = vd55g1_mbus_codes[code->index].code;

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_get_pad_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *sd_fmt)
#else
static int vd55g1_get_pad_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *sd_fmt)
#endif
{
	struct vd55g1 *sensor = to_vd55g1(sd);
	struct v4l2_mbus_framefmt *pad_fmt;

	mutex_lock(&sensor->lock);

	if (sd_fmt->which == V4L2_SUBDEV_FORMAT_TRY)
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
		pad_fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						     sd_fmt->pad);
#elif KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
		pad_fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state,
						     sd_fmt->pad);
#else
		pad_fmt = v4l2_subdev_get_pad_format(&sensor->sd, sd_state,
						     sd_fmt->pad);
#endif
	else
		pad_fmt = &sensor->active_fmt;

	sd_fmt->format = *pad_fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_set_pad_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *sd_fmt)
#else
static int vd55g1_set_pad_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *sd_fmt)
#endif
{
	struct vd55g1 *sensor = to_vd55g1(sd);
	const struct vd55g1_mode *new_mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect pad_crop;
	unsigned int binning;
	struct hblank_limits hblank;
	unsigned int vblank_max;

	if (sensor->streaming) {
		return -EBUSY;
	}

	mutex_lock(&sensor->lock);

	new_mode = v4l2_find_nearest_size(vd55g1_supported_modes,
					  ARRAY_SIZE(vd55g1_supported_modes),
					  width, height, sd_fmt->format.width,
					  sd_fmt->format.height);

	vd55g1_update_img_pad_format(sensor, new_mode, sd_fmt->format.code,
				     &sd_fmt->format);

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
	format = v4l2_subdev_get_try_format(sd, cfg, sd_fmt->pad);
#elif KERNEL_VERSION(5, 19, 0) > LINUX_VERSION_CODE
	format = v4l2_subdev_get_try_format(sd, sd_state, sd_fmt->pad);
#else
	format = v4l2_subdev_get_pad_format(sd, sd_state, sd_fmt->pad);
#endif
	*format = sd_fmt->format;

	/*
	 * Use binning to maximize the crop rectangle size, and centre it in the
	 * sensor.
	 */
	binning = min(VD55G1_WIDTH / sd_fmt->format.width,
		      VD55G1_HEIGHT / sd_fmt->format.height);
	binning = min(binning, 2U);
	pad_crop.width = sd_fmt->format.width * binning;
	pad_crop.height = sd_fmt->format.height * binning;
	pad_crop.left = (VD55G1_WIDTH - pad_crop.width) / 2;
	pad_crop.top = (VD55G1_HEIGHT - pad_crop.height) / 2;

	if (sd_fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		//TODO remove once active state is ready
		sensor->active_fmt = sd_fmt->format;
		sensor->active_crop = pad_crop;
		/* Reset vblank and frame length to default */
		//TODO factorize as vblank limits ?
		vblank_max = 0xffff - sensor->active_crop.height;
		__v4l2_ctrl_modify_range(sensor->vblank_ctrl, VD55G1_VBLANK_DEF, vblank_max, 1,
					 VD55G1_VBLANK_DEF);
		__v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, VD55G1_VBLANK_DEF);
#if 0
		/* Max exposure changes with vblank */
		expo_max = sensor->frame_length - VD55G1_EXPO_MAX_TERM;
		__v4l2_ctrl_modify_range(sensor->expo_ctrl, 0, expo_max, 1,
					 VD55G1_EXPO_DEF);
#endif
		/* Update controls to reflect new mode */
		__v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate_ctrl,
					 get_pixel_rate(sensor));
		/* Update hblank according to new width */
		hblank = get_hblank_limits(sensor);
		__v4l2_ctrl_modify_range(sensor->hblank_ctrl, hblank.min, hblank.max, 1,
					 hblank.min);
		__v4l2_ctrl_s_ctrl(sensor->hblank_ctrl, hblank.min);
	}

	mutex_unlock(&sensor->lock);

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
#else
static int vd55g1_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state)
#endif
{
	unsigned int def_mode = VD55G1_DEFAULT_MODE;
	struct vd55g1 *sensor = to_vd55g1(sd);
	struct v4l2_subdev_format fmt = { 0 };

	vd55g1_update_img_pad_format(sensor, &vd55g1_supported_modes[def_mode],
			     VD55G1_MEDIA_BUS_FMT_DEF, &fmt.format);

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
	return vd55g1_set_pad_fmt(sd, cfg, &fmt);
#else
	return vd55g1_set_pad_fmt(sd, sd_state, &fmt);
#endif
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
#else
static int vd55g1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
#endif
{
	if (fse->index >= ARRAY_SIZE(vd55g1_supported_modes))
		return -EINVAL;

	fse->min_width = vd55g1_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = vd55g1_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static const struct v4l2_subdev_core_ops vd55g1_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops vd55g1_video_ops = {
	.s_stream = vd55g1_s_stream,
};

static const struct v4l2_subdev_pad_ops vd55g1_pad_ops = {
	.init_cfg = vd55g1_init_cfg,
	.enum_mbus_code = vd55g1_enum_mbus_code,
	.get_fmt = vd55g1_get_pad_fmt,
	.set_fmt = vd55g1_set_pad_fmt,
	.get_selection = vd55g1_get_selection,
	.enum_frame_size = vd55g1_enum_frame_size,
};

static const struct v4l2_subdev_ops vd55g1_subdev_ops = {
	.core = &vd55g1_core_ops,
	.video = &vd55g1_video_ops,
	.pad = &vd55g1_pad_ops,
};

static const struct media_entity_operations vd55g1_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int vd55g1_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd55g1 *sensor = to_vd55g1(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int temperature;
	int ret = 0;

	/* Interact with HW only when it is powered ON */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_TEMPERATURE:
		ret = vd55g1_get_temp(sensor, &temperature);
		if (ret)
			break;
		ret = __v4l2_ctrl_s_ctrl(ctrl, temperature);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = vd55g1_read_expo_cluster(sensor, false);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static int vd55g1_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd55g1 *sensor = to_vd55g1(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned int frame_length = 0;
	//unsigned int expo_max;
	struct hblank_limits hblank = get_hblank_limits(sensor);
	bool is_auto = false;
	int ret;

	vd55g1_read_expo_cluster(sensor, true);

	if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
		return 0;

	/* Update controls state, range, etc. whatever the state of the HW */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		frame_length = sensor->active_crop.height + ctrl->val;
#if 0
		expo_max = frame_length - VD55G1_EXPO_MAX_TERM;
		__v4l2_ctrl_modify_range(sensor->expo_ctrl, 0, expo_max, 1,
					 VD55G1_EXPO_DEF);
#endif
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		is_auto = (ctrl->val == V4L2_EXPOSURE_AUTO);
#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
		mutex_unlock(&sensor->lock);
		v4l2_ctrl_grab(sensor->ae_lock_ctrl, !is_auto);
		v4l2_ctrl_grab(sensor->ae_bias_ctrl, !is_auto);
		mutex_lock(&sensor->lock);
#else
		__v4l2_ctrl_grab(sensor->ae_lock_ctrl, !is_auto);
		__v4l2_ctrl_grab(sensor->ae_bias_ctrl, !is_auto);
#endif
		break;
	case V4L2_CID_HDR_SENSOR_MODE:
		/* Discriminate if the userspace changed the control value */
		if (ctrl->val != ctrl->cur.val ) {
			/* Max horizontal blanking changes with hdr mode */
			__v4l2_ctrl_modify_range(sensor->hblank_ctrl, hblank.min, hblank.max, 1,
						 hblank.min);
		}
		break;
	default:
		break;
	}

	/* Interact with HW only when it is powered ON */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ret = vd55g1_write(sensor, VD55G1_REG_ORIENTATION,
				   sensor->hflip_ctrl->val |
					   (sensor->vflip_ctrl->val << 1),
				   NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = vd55g1_update_patgen(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = vd55g1_update_expo_cluster(sensor, is_auto);
		break;
	case V4L2_CID_3A_LOCK:
		ret = vd55g1_lock_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		/*
		 * We use auto exposure target percentage register to control
		 * exposure bias for more precision.
		 */
		ret = vd55g1_update_exposure_target(sensor, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = vd55g1_update_frame_length(sensor, frame_length);
		break;
	case V4L2_CID_SLAVE_MODE:
		ret = vd55g1_write_gpios(sensor, VD55G1_VTSLAVE_GPIO);
		if (ret)
			break;
		ret = vd55g1_write(sensor, VD55G1_REG_VT_CTRL, ctrl->val, &ret);
		break;
	case V4L2_CID_DARKCAL_PEDESTAL:
		vd55g1_write(sensor, VD55G1_REG_DARKCAL_PEDESTAL(0),
				 ctrl->val, &ret);
		vd55g1_write(sensor, VD55G1_REG_DARKCAL_PEDESTAL(1),
				 ctrl->val, &ret);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		ret = vd55g1_write_gpios(sensor, sensor->ext_leds_mask);
		break;
	case V4L2_CID_HDR_SENSOR_MODE:
		ret = vd55g1_update_hdr_mode(sensor);
		break;
	case V4L2_CID_HBLANK:
		ret =  vd55g1_write(sensor, VD55G1_REG_LINE_LENGTH,
				    sensor->active_crop.width + ctrl->val,
				    NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops vd55g1_ctrl_ops = {
	.g_volatile_ctrl = vd55g1_g_volatile_ctrl,
	.s_ctrl = vd55g1_s_ctrl,
};

static const struct v4l2_ctrl_config vd55g1_temp_ctrl = {
	.ops		= &vd55g1_ctrl_ops,
	.id		= V4L2_CID_TEMPERATURE,
	.name		= "Temperature in celsius",
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.min		= -1024,
	.max		= 1023,
	.step		= 1,
};

static const struct v4l2_ctrl_config vd55g1_darkcal_pedestal_ctrl = {
	.ops		= &vd55g1_ctrl_ops,
	.id		= V4L2_CID_DARKCAL_PEDESTAL,
	.name		= "Dark Calibration Pedestal",
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.min		= 0,
	.max		= 512,
	.step		= 1,
	.def		= VD55G1_DARKCAL_PEDESTAL_DEF,
};

static const struct v4l2_ctrl_config vd55g1_slave_ctrl = {
	.ops		= &vd55g1_ctrl_ops,
	.id		= V4L2_CID_SLAVE_MODE,
	.name		= "VT Slave Mode",
	.type		= V4L2_CTRL_TYPE_BOOLEAN,
	.min		= 0,
	.max		= 1,
	.step		= 1,
	.def		= 1,
};

//TODO use standard control if available
static const struct v4l2_ctrl_config vd55g1_hdr_ctrl = {
	.ops		= &vd55g1_ctrl_ops,
	.id		= V4L2_CID_HDR_SENSOR_MODE,
	.name		= "HDR Sensor Mode",
	.type		= V4L2_CTRL_TYPE_MENU,
	.min		= 0,
	.max		= ARRAY_SIZE(vd55g1_hdr_mode_menu) - 1,
	.def		= VD55G1_NO_HDR,
	.qmenu		= vd55g1_hdr_mode_menu,
};

static int vd55g1_init_ctrls(struct vd55g1 *sensor)
{
	const struct v4l2_ctrl_ops *ops = &vd55g1_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
	struct v4l2_ctrl *ctrl;
	unsigned int vblank_max = 0xffff - sensor->active_crop.height;
	struct hblank_limits hblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 16);
	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Flip cluster */
	sensor->hflip_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					       0, 1, 1, 0);
	sensor->vflip_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					       0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &sensor->hflip_ctrl);

	/* Exposition cluster */
	sensor->ae_ctrl = v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO, 1, ~0x3,
			       V4L2_EXPOSURE_AUTO);
	sensor->again_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, 0, 0x1c, 1,
			  VD55G1_AGAIN_DEF);
	sensor->dgain_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN, 256, 0xffff, 1,
			  VD55G1_DGAIN_DEF); //TODO only integer part?
	sensor->expo_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0,
				  VD55G1_FRAME_LENGTH_DEF - VD55G1_EXPO_MAX_TERM,
				  1, VD55G1_EXPO_DEF);
	v4l2_ctrl_auto_cluster(4, &sensor->ae_ctrl, V4L2_EXPOSURE_MANUAL, true);

	sensor->patgen_ctrl =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(vd55g1_tp_menu) - 1, 0,
					     0, vd55g1_tp_menu);
	ctrl = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq) - 1, 0, link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->pixel_rate_ctrl = v4l2_ctrl_new_std(hdl, ops,
						    V4L2_CID_PIXEL_RATE, 1,
						    INT_MAX, 1,
						    get_pixel_rate(sensor));
	if (sensor->pixel_rate_ctrl)
		sensor->pixel_rate_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->ae_lock_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK, 0, 1, 0, 0);
	sensor->ae_bias_ctrl = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_AUTO_EXPOSURE_BIAS,
			       ARRAY_SIZE(vd55g1_ev_bias_menu) - 1,
			       ARRAY_SIZE(vd55g1_ev_bias_menu) / 2,
			       vd55g1_ev_bias_menu);
	sensor->hdr_ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_hdr_ctrl, NULL);
	hblank = get_hblank_limits(sensor);
	sensor->hblank_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
						hblank.min, hblank.max, 1,
						hblank.min);
	sensor->vblank_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
						VD55G1_VBLANK_DEF, vblank_max,
						1, VD55G1_VBLANK_DEF);
	ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_temp_ctrl, NULL);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE |
			       V4L2_CTRL_FLAG_READ_ONLY;
	sensor->darkcal_ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_darkcal_pedestal_ctrl, NULL);

	/* Additional controls based on device tree properties */
	if (sensor->ext_vt_sync)
		sensor->slave_ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_slave_ctrl,
						  NULL);
	if (sensor->ext_leds_mask) {
		sensor->led_ctrl =
			v4l2_ctrl_new_std_menu(hdl, ops,
					       V4L2_CID_FLASH_LED_MODE,
					       V4L2_FLASH_LED_MODE_FLASH, 0,
					       V4L2_FLASH_LED_MODE_NONE);
	}

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int vd55g1_detect_cut_version(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u32 device_rev;
	int ret;

	ret = vd55g1_read(sensor, VD55G1_REG_REVISION, &device_rev, NULL);
	if (ret)
		return ret;

	switch (device_rev) {
	case VD55G1_REVISION_CUT_1:
		dev_err(&client->dev, "Cut 1 is not supported\n");
		return -ENODEV;
	case VD55G1_REVISION_CUT_2:
		dev_dbg(&client->dev, "Cut 2 detected\n");
		return 0;
	default:
		dev_err(&client->dev, "Unable to detect cut version (0x%x)\n",
			device_rev);
		return -ENODEV;
	}
}

static int vd55g1_detect(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u32 id;
	int ret;

	ret = vd55g1_read(sensor, VD55G1_REG_MODEL_ID, &id, NULL);
	if (ret)
		return ret;

	if (id != VD55G1_MODEL_ID) {
		dev_warn(&client->dev, "Unsupported sensor id %x", id);
		return -ENODEV;
	}

	ret = vd55g1_detect_cut_version(sensor);
	if (ret)
		return ret;

	return 0;
}

/* Power/clock management functions */
static int vd55g1_power_on(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(vd55g1_supply_name),
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "failed to enable regulators %d", ret);
		return ret;
	}

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "failed to enable clock %d", ret);
		goto disable_bulk;
	}

	if (sensor->reset_gpio) {
		ret = vd55g1_apply_reset(sensor);
		if (ret) {
			dev_err(&client->dev, "sensor reset failed %d\n", ret);
			goto disable_clock;
		}
	}

	ret = vd55g1_detect(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor detect failed %d", ret);
		goto disable_clock;
	}

	ret = vd55g1_patch(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor patch failed %d", ret);
		goto disable_clock;
	}

	ret = vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_SW_STBY, NULL);
	if (ret)
		return ret;

	return 0;

disable_clock:
	clk_disable_unprepare(sensor->xclk);
disable_bulk:
	regulator_bulk_disable(ARRAY_SIZE(vd55g1_supply_name),
			       sensor->supplies);

	return ret;
}

static int vd55g1_power_off(struct vd55g1 *sensor)
{
	clk_disable_unprepare(sensor->xclk);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	return 0;
}

static int vd55g1_check_csi_conf(struct vd55g1 *sensor,
				 struct fwnode_handle *endpoint)
{
	struct i2c_client *client = sensor->i2c_client;
#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2 };
#else
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
#endif
	u8 n_lanes;
	int ret = 0;

#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	struct v4l2_fwnode_endpoint *ep_ptr =
		v4l2_fwnode_endpoint_alloc_parse(endpoint);
	if (IS_ERR(ep_ptr))
		return -EINVAL;
	ep = (*ep_ptr);
#else
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	if (ret)
		return -EINVAL;
#endif

	/* Check lanes number */
	n_lanes = ep.bus.mipi_csi2.num_data_lanes;
	if (n_lanes != 1) {
		dev_err(&client->dev, "Sensor only supports 1 lane, found %d\n", n_lanes);
		ret = -EINVAL;
		goto done;
	}

	/* Clock lane must be first */
	if (ep.bus.mipi_csi2.clock_lane != 0) {
		dev_err(&client->dev, "Clk lane must be mapped to lane 0\n");
		ret = -EINVAL;
		goto done;
	}

	/* Handle polarities in sensor configuration */
	sensor->oif_ctrl = (ep.bus.mipi_csi2.lane_polarities[0] << 3) |
			   (ep.bus.mipi_csi2.lane_polarities[1] << 6);

	/* Check the link frequency set in device tree */
	if (!ep.nr_of_link_frequencies) {
		dev_err(&client->dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto done;
	}
	//TODO support multiple link freq
	if (ep.nr_of_link_frequencies != 1) {
		dev_err(&client->dev, "Multiple link frequencies not supported\n");
		ret = -EINVAL;
		goto done;
	}
	link_freq[0] = ep.link_frequencies[0];

done:
#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	v4l2_fwnode_endpoint_free(ep_ptr);
#else
	v4l2_fwnode_endpoint_free(&ep);
#endif

	return ret;
}

static int vd55g1_parse_dt_gpios_array(struct vd55g1 *sensor,
				       char *prop_name, u32 *array, int *nb)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device_node *np = client->dev.of_node;
	unsigned int i;

	*nb = of_property_read_variable_u32_array(np, prop_name, array, 0,
						  VD55G1_NB_GPIOS);
	if (*nb == -EINVAL) {
		/* Property not found */
		*nb = 0;
		return 0;
	} else if (*nb < 0) {
		dev_err(&client->dev, "Failed to read %s prop\n", prop_name);
		return *nb;
	}

	for (i = 0; i < *nb;  i++) {
		if (array[i] >= VD55G1_NB_GPIOS) {
			dev_err(&client->dev, "invalid GPIO number %d\n",
				array[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static int vd55g1_parse_dt_gpios(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device_node *np = client->dev.of_node;
	u32 led_gpios[VD55G1_NB_GPIOS];
	int nb_gpios_leds;
	u32 out_sync_gpios[VD55G1_NB_GPIOS];
	int nb_gpios_out;
	u32 in_sync_gpio;
	unsigned int i;
	int ret;

	/* Initialize GPIOs to default */
	for (i = 0; i < VD55G1_NB_GPIOS; i++)
		sensor->gpios[i] = VD55G1_GPIO_MODE_IN;
	sensor->ext_leds_mask = 0;

	/* Take into account optional 'st,leds' output for GPIOs */
	ret = vd55g1_parse_dt_gpios_array(sensor, "st,leds", led_gpios,
					  &nb_gpios_leds);
	if (ret)
		return ret;

	for (i = 0; i < nb_gpios_leds; i++) {
		sensor->gpios[led_gpios[i]] = VD55G1_GPIO_MODE_STROBE;
		set_bit(led_gpios[i], &sensor->ext_leds_mask);
	}

	/* Take into account optional 'st,out-sync' output for GPIOs */
	ret = vd55g1_parse_dt_gpios_array(sensor, "st,out-sync", out_sync_gpios,
					  &nb_gpios_out);
	if (ret)
		return ret;

	for (i = 0; i < nb_gpios_out; i++) {
		if (sensor->gpios[out_sync_gpios[i]] != VD55G1_GPIO_MODE_IN) {
			dev_err(&client->dev, "Multiple use of GPIO %d\n",
				out_sync_gpios[i]);
			return -EINVAL;
		}
		sensor->gpios[out_sync_gpios[i]] = VD55G1_GPIO_MODE_FSYNC_OUT;
	}

	/* Take into account optional 'st,in-sync' input for GPIO0 */
	ret = of_property_read_u32(np, "st,in-sync", &in_sync_gpio);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&client->dev, "Failed to read st,in-sync prop\n");
		return ret;
	} else if (ret == -EINVAL) {
		sensor->ext_vt_sync = false;
	} else {
		if (in_sync_gpio != VD55G1_VTSLAVE_GPIO) {
			dev_err(&client->dev, "in-sync GPIO must be gpio0\n");
			return -EINVAL;
		}
		if (sensor->gpios[in_sync_gpio] != VD55G1_GPIO_MODE_IN) {
			dev_err(&client->dev, "Multiple use of GPIO %d\n",
				in_sync_gpio);
			return -EINVAL;
		}
		sensor->gpios[in_sync_gpio] = VD55G1_GPIO_MODE_VTSLAVE;
		sensor->ext_vt_sync = true;
	}

	return 0;
}

static int vd55g1_parse_dt(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	int ret;

#if KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE
	endpoint =
		fwnode_graph_get_next_endpoint(of_fwnode_handle(dev->of_node),
					       NULL);
#else
	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0, 0);
#endif
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = vd55g1_check_csi_conf(sensor, endpoint);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	return vd55g1_parse_dt_gpios(sensor);
}

static int vd55g1_subdev_init(struct vd55g1 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int def_mode = VD55G1_DEFAULT_MODE;
	int ret;

	mutex_init(&sensor->lock);

	/* Init sub device */
	v4l2_i2c_subdev_init(&sensor->sd, client, &vd55g1_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.ops = &vd55g1_subdev_entity_ops;

	/* Init source pad */
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(&client->dev, "Failed to init media entity : %d", ret);
		return ret;
	}

	/* Init vd56g3 struct : default resolution + raw8 */
	sensor->streaming = false;
	vd55g1_update_img_pad_format(sensor, &vd55g1_supported_modes[def_mode],
				     VD55G1_MEDIA_BUS_FMT_DEF,
				     &sensor->active_fmt);
	sensor->active_crop.width = vd55g1_supported_modes[def_mode].width;
	sensor->active_crop.height = vd55g1_supported_modes[def_mode].height;
	sensor->active_crop.left = 2;
	sensor->active_crop.top = 2;

	/*
	 * Initiliaze controls after update_img_pad_format to make sure default
	 * values are set.
	 */
	ret = vd55g1_init_ctrls(sensor);
	if (ret) {
		dev_err(&client->dev, "Controls initialization failed %d", ret);
		goto err_media;
	}

err_media:
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void vd55g1_subdev_cleanup(struct vd55g1 *sensor)
{
	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);
}

static int vd55g1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct vd55g1 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	ret = vd55g1_parse_dt(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to parse Device Tree.");

	/* Get (and check) resources : power regs, ext clock, reset gpio */
	ret = vd55g1_get_regulators(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators.");

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		return dev_err_probe(dev, PTR_ERR(sensor->xclk),
				     "Failed to get xclk.");
	}
	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	ret = vd55g1_prepare_clock_tree(sensor);
	if (ret)
		return ret;

	sensor->reset_gpio =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "Failed to get reset gpio.");

#if KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE
	sensor->regmap = devm_regmap_init_i2c(client, &vd55g1_regmap_config);
#else
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
#endif
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "Failed to init regmap.");

	TRACE("power on");
	ret = vd55g1_power_on(sensor);
	if (ret)
		return ret;

	TRACE("pm runtime");
	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 4000);
	pm_runtime_use_autosuspend(dev);

	TRACE("subdev init");
	ret = vd55g1_subdev_init(sensor);
	if (ret) {
		dev_err(&client->dev, "V4l2 init failed : %d", ret);
		goto err_power_off;
	}

	TRACE("");
	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "async subdev register failed %d", ret);
		goto err_subdev;
	}

	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_use_autosuspend(&client->dev);

	dev_dbg(&client->dev, "vd55g1 probe successfully");

	return 0;

err_subdev:
	vd55g1_subdev_cleanup(sensor);
err_power_off:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	vd55g1_power_off(sensor);

	return ret;
}

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
static int vd55g1_remove(struct i2c_client *client)
#else
static void vd55g1_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g1 *sensor = to_vd55g1(sd);

	vd55g1_subdev_cleanup(sensor);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		vd55g1_power_off(sensor);
	pm_runtime_set_suspended(&client->dev);
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	return 0;
#endif
}

static const struct of_device_id vd55g1_dt_ids[] = {
	{ .compatible = "st,vd55g1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vd55g1_dt_ids);

static int vd55g1_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct vd55g1 *vd55g1 = to_vd55g1(sd);

	return vd55g1_power_on(vd55g1);
}

static int vd55g1_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct vd55g1 *vd55g1 = to_vd55g1(sd);

	return vd55g1_power_off(vd55g1);
}

static const struct dev_pm_ops vd55g1_pm_ops = {
	SET_RUNTIME_PM_OPS(vd55g1_runtime_suspend, vd55g1_runtime_resume, NULL)
};

static struct i2c_driver vd55g1_i2c_driver = {
	.driver = {
		.name  = "vd55g1",
		.of_match_table = vd55g1_dt_ids,
		.pm = &vd55g1_pm_ops,
	},
#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
	.probe_new = vd55g1_probe,
#else
	.probe = vd55g1_probe,
#endif
	.remove = vd55g1_remove,
};

module_i2c_driver(vd55g1_i2c_driver);

MODULE_AUTHOR("Benjamin Mugnier <benjamin.mugnier@foss.st.com>");
MODULE_DESCRIPTION("VD55G1 camera subdev driver");
MODULE_LICENSE("GPL v2");
