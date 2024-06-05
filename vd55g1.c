// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for VD55G1 global shutter sensor family driver
 *
 * Copyright (C) 2023 STMicroelectronics SA
 */

#include <linux/version.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <asm/unaligned.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Backward compatibility */
#if KERNEL_VERSION(5, 18, 0) > LINUX_VERSION_CODE
#define MIPI_CSI2_DT_RAW8	0x2a
#define MIPI_CSI2_DT_RAW10	0x2b
#define MIPI_CSI2_DT_RAW12	0x2c
#define MIPI_CSI2_DT_RAW14	0x2d
#define MIPI_CSI2_DT_RAW16	0x2e
#else
#include <media/mipi-csi2.h>
#endif

#if KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE
#define HZ_PER_MHZ		1000000UL
#define MEGA			1000000UL
#else
#include <linux/units.h>
#endif

#define VD55G1_REG_8BIT(n)				((1 << 16) | (n))
#define VD55G1_REG_16BIT(n)				((2 << 16) | (n))
#define VD55G1_REG_32BIT(n)				((4 << 16) | (n))
#define VD55G1_REG_SIZE_SHIFT				16
#define VD55G1_REG_ADDR_MASK				0xffff

#define VD55G1_REG_MODEL_ID				VD55G1_REG_32BIT(0x0000)
#define VD55G1_MODEL_ID					0x53354731
#define VD55G1_REG_REVISION				VD55G1_REG_16BIT(0x0004)
#define VD55G1_REVISION_CUT_1				0x1010
#define VD55G1_REVISION_CUT_2				0x2020
#define VD55G1_REG_FWPATCH_REVISION			VD55G1_REG_16BIT(0x0012)
#define VD55G1_REG_FWPATCH_START_ADDR			VD55G1_REG_8BIT(0x2000)
#define VD55G1_REG_SYSTEM_FSM				VD55G1_REG_8BIT(0x001c)
#define VD55G1_SYSTEM_FSM_READY_TO_BOOT			0x01
#define VD55G1_SYSTEM_FSM_SW_STBY			0x02
#define VD55G1_SYSTEM_FSM_STREAMING			0x03
#define VD55G1_REG_TEMPERATURE				VD55G1_REG_16BIT(0x003c)
#define VD55G1_REG_BOOT					VD55G1_REG_8BIT(0x0200)
#define VD55G1_BOOT_BOOT				1
#define VD55G1_BOOT_PATCH_SETUP				2
#define VD55G1_REG_STBY					VD55G1_REG_8BIT(0x0201)
#define VD55G1_STBY_START_STREAM			1
#define VD55G1_STBY_THSENS_READ				4
#define VD55G1_REG_STREAMING				VD55G1_REG_8BIT(0x0202)
#define VD55G1_STREAMING_STOP_STREAM			1
#define VD55G1_REG_EXT_CLOCK				VD55G1_REG_32BIT(0x0220)
#define VD55G1_REG_LINE_LENGTH				VD55G1_REG_16BIT(0x0300)
#define VD55G1_REG_ORIENTATION				VD55G1_REG_8BIT(0x0302)
#define VD55G1_REG_FORMAT_CTRL				VD55G1_REG_8BIT(0x030a)
#define VD55G1_REG_OIF_CTRL				VD55G1_REG_16BIT(0x030c)
#define VD55G1_REG_OIF_IMG_CTRL				VD55G1_REG_8BIT(0x030f)
#define VD55G1_REG_MIPI_DATA_RATE			VD55G1_REG_32BIT(0x0224)
#define VD55G1_REG_PATGEN_CTRL				VD55G1_REG_16BIT(0x0304)
#define VD55G1_PATGEN_TYPE_SHIFT			4
#define VD55G1_PATGEN_ENABLE				BIT(0)
#define VD55G1_REG_MANUAL_ANALOG_GAIN			VD55G1_REG_8BIT(0x0501)
#define VD55G1_REG_MANUAL_COARSE_EXPOSURE		VD55G1_REG_16BIT(0x0502)
#define VD55G1_REG_MANUAL_DIGITAL_GAIN			VD55G1_REG_16BIT(0x0504)
#define VD55G1_REG_APPLIED_COARSE_EXPOSURE		VD55G1_REG_16BIT(0x00e8)
#define VD55G1_REG_APPLIED_ANALOG_GAIN			VD55G1_REG_16BIT(0x00ea)
#define VD55G1_REG_APPLIED_DIGITAL_GAIN			VD55G1_REG_16BIT(0x00ec)
#define VD55G1_REG_AE_FORCE_COLDSTART			VD55G1_REG_16BIT(0x0308)
#define VD55G1_REG_AE_COLDSTART_EXP_TIME		VD55G1_REG_32BIT(0x0374)
#define VD55G1_REG_READOUT_CTRL				VD55G1_REG_8BIT(0x052e)
#define VD55G1_REG_DARKCAL_CTRL				VD55G1_REG_8BIT(0x032a)
#define VD55G1_DARKCAL_BYPASS				0
#define VD55G1_DARKCAL_AUTO				1
#define VD55G1_REG_DUSTER_CTRL				VD55G1_REG_8BIT(0x0316)
#define VD55G1_DUSTER_ENABLE				BIT(0)
#define VD55G1_DUSTER_DISABLE				0
#define VD55G1_DUSTER_DYN_ENABLE			BIT(1)
#define VD55G1_DUSTER_RING_ENABLE			BIT(4)
#define VD55G1_REG_AE_TARGET_PERCENTAGE			VD55G1_REG_8BIT(0x0486)
#define VD55G1_REG_VT_CTRL				VD55G1_REG_8BIT(0x0309)
#define VD55G1_VT_SLAVE_GPIO				1
#define VD55G1_REG_ERROR_CODE				VD55G1_REG_16BIT(0x0010)
#define VD55G1_REG_NEXT_CTX				VD55G1_REG_16BIT(0x03e4)
#define VD55G1_REG_EXPOSURE_USE_CASES			VD55G1_REG_8BIT(0x0312)
#define VD55G1_EXPOSURE_USE_CASES_MULTI_CONTEXT		BIT(2)

#define VD55G1_REG_EXP_MODE(ctx) \
	VD55G1_REG_8BIT(0x0500 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_EXP_MODE_AUTO				0
#define VD55G1_EXP_MODE_FREEZE				1
#define VD55G1_EXP_MODE_MANUAL				2
#define VD55G1_REG_FRAME_LENGTH(ctx) \
	VD55G1_REG_32BIT(0x050c + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_X_START(ctx) \
	VD55G1_REG_16BIT(0x0514 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_X_WIDTH(ctx) \
	VD55G1_REG_16BIT(0x0516 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_Y_START(ctx) \
	VD55G1_REG_16BIT(0x0510 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_Y_HEIGHT(ctx) \
	VD55G1_REG_16BIT(0x0512 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_GPIO_0_CTRL(ctx) \
	VD55G1_REG_8BIT(0x051d + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_DARKCAL_PEDESTAL(ctx) \
	VD55G1_REG_16BIT(0x0526 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_CTX_REPEAT_COUNT(ctx) \
	VD55G1_REG_16BIT(0x03dc + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_REG_VT_MODE(ctx) \
	VD55G1_REG_8BIT(0x0536 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_VT_MODE_NORMAL 0
#define VD55G1_VT_MODE_SUBTRACTION 1
#define VD55G1_REG_MASK_FRAME_CTRL(ctx) \
	VD55G1_REG_8BIT(0x0537 + VD55G1_CTX_OFFSET * (ctx))
#define VD55G1_MASK_FRAME_CTRL_OUTPUT 0
#define VD55G1_MASK_FRAME_CTRL_MASK 1
#define VD55G1_REG_EXPOSURE_INSTANCE(ctx) \
	VD55G1_REG_32BIT(0x52D + VD55G1_CTX_OFFSET * (ctx))

#define VD55G1_WIDTH					804
#define VD55G1_HEIGHT					704
#define VD55G1_DEFAULT_MODE				1
#define VD55G1_WRITE_MULTIPLE_CHUNK_MAX			16
#define VD55G1_NB_GPIOS					4
#define VD55G1_NB_POLARITIES				3
#define VD55G1_MIN_VBLANK				86
#define VD55G1_FRAME_LENGTH_DEF				1860 /* 60 fps */
#define VD55G1_TIMEOUT_MS				500
#define VD55G1_MEDIA_BUS_FMT_DEF			MEDIA_BUS_FMT_Y8_1X8
#define VD55G1_DARKCAL_PEDESTAL_DEF			0x40
#define VD55G1_EXPO_MAX_TERM				64
#define VD55G1_EXPO_DEF					500
#define VD55G1_MIN_LINE_LENGTH				1128
#define VD55G1_MIN_LINE_LENGTH_SUB			1344
#define VD55G1_MIPI_MARGIN				900
#define VD55G1_PCLK_DIVISOR				5
#define VD55G1_CTX_OFFSET				0x50

#define V4L2_CID_TEMPERATURE			(V4L2_CID_USER_BASE | 0x1020)
#define V4L2_CID_DARKCAL_PEDESTAL		(V4L2_CID_USER_BASE | 0x1021)
#define V4L2_CID_SLAVE				(V4L2_CID_USER_BASE | 0x1022)
#if KERNEL_VERSION(6, 2, 0) > LINUX_VERSION_CODE
#define V4L2_CID_HDR_SENSOR_MODE		(V4L2_CID_USER_BASE | 0x1004)
#endif

#include "vd55g1_patch.c"

static const char * const vd55g1_test_pattern_menu[] = {
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
	/*
	 * This mode acquires 2 frames on the sensor, the first on is ditched
	 * out and only used for auto exposure data, the second one is output to
	 * the host
	 */
	"Internal subtraction",
	"No HDR",
};

static const char * const vd55g1_supply_name[] = {
	"VCORE",
	"VDDIO",
	"VANA",
};

static const s64 link_freq[] = {
	/*
	 * MIPI output freq is sensor datarate / 2, as it uses both rising edge
	 * and falling edges to send data.
	 * Sensor outputs at 1.2Ghz.
	 */
	600000000ULL
};

enum vd55g1_hdr_mode {
	VD55G1_HDR_SUB,
	VD55G1_NO_HDR,
};

enum vd55g1_bin_mode {
	VD55G1_BIN_MODE_NORMAL,
	VD55G1_BIN_MODE_DIGITAL_X2,
	VD55G1_BIN_MODE_DIGITAL_X4,
};

enum vd55g1_gpio_modes {
	VD55G1_GPIO_MODE_DISABLED,
	VD55G1_GPIO_MODE_STROBE,
	VD55G1_GPIO_MODE_VSYNC_OUT_0,
	VD55G1_GPIO_MODE_VTSLAVE,
};

struct vd55g1_mode_info {
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

static const struct vd55g1_fmt_desc vd55g1_supported_codes[] = {
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

static const struct vd55g1_mode_info vd55g1_mode_data[] = {
	{
		.width = VD55G1_WIDTH,
		.height = VD55G1_HEIGHT,
		.bin_mode = VD55G1_BIN_MODE_NORMAL,
		.crop = {
			.left = 0,
			.top = 0,
			.width = VD55G1_WIDTH,
			.height = VD55G1_HEIGHT,
		},
	},
	{
		.width = 800,
		.height = VD55G1_HEIGHT,
		.bin_mode = VD55G1_BIN_MODE_NORMAL,
		.crop = {
			.left = 2,
			.top = 0,
			.width = 800,
			.height = VD55G1_HEIGHT,
		},
	},
	{
		.width = 800,
		.height = 600,
		.bin_mode = VD55G1_BIN_MODE_NORMAL,
		.crop = {
			.left = 2,
			.top = 52,
			.width = 800,
			.height = 600,
		},
	},
	{
		.width = 640,
		.height = 480,
		.bin_mode = VD55G1_BIN_MODE_NORMAL,
		.crop = {
			.left = 82,
			.top = 112,
			.width = 640,
			.height = 480,
		},
	},
	{
		.width = 320,
		.height = 240,
		.bin_mode = VD55G1_BIN_MODE_DIGITAL_X2,
		.crop = {
			.left = 82,
			.top = 112,
			.width = 640,
			.height = 480,
		},
	},
};

enum vd55g1_expo_state {
	VD55G1_EXP_AUTO,
	VD55G1_EXP_FREEZE,
	VD55G1_EXP_MANUAL,
	VD55G1_EXP_SINGLE_STEP,
	VD55G1_EXP_BYPASS,
};

struct vd55g1_gpios {
	u32 leds[VD55G1_NB_GPIOS];
	u32 out_sync[VD55G1_NB_GPIOS];
	u32 in_sync;
};

struct vd55g1_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regulator_bulk_data supplies[ARRAY_SIZE(vd55g1_supply_name)];
	struct gpio_desc *reset_gpio;
	struct clk *xclk;
	u32 clk_freq;
	u16 oif_ctrl;
	int nb_of_lane;
	int data_rate_in_mbps;
	int pclk;
	u16 line_length;
	/* Lock to protect all members below */
	struct mutex lock;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vflip_ctrl;
	struct v4l2_ctrl *hflip_ctrl;
	struct v4l2_ctrl *pattern_ctrl;
	struct v4l2_ctrl *slave_ctrl;
	struct v4l2_ctrl *expo_ctrl;
	struct v4l2_ctrl *hdr_ctrl;
	bool streaming;
	struct v4l2_mbus_framefmt fmt;
	const struct vd55g1_mode_info *current_mode;
	bool hflip;
	bool vflip;
	enum vd55g1_hdr_mode hdr;
	u16 manual_expo;
	enum vd55g1_expo_state expo_state;
	u16 digital_gain;
	u8 analog_gain;
	u16 vblank;
	u16 vblank_min;
	u16 frame_length;
	bool ae_frozen;
	u32 pattern;
	u16 darkcal_pedestal;
	u16 exposure_target;
	struct vd55g1_gpios gpios;
	bool is_slave;
	bool flash_en;
	struct {
		u16 expo;
		u16 digital_gain;
		u8 analog_gain;
	} cold_start;
};

static inline struct vd55g1_dev *to_vd55g1_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vd55g1_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct vd55g1_dev,
		ctrl_handler)->sd;
}

static u8 get_bpp_by_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vd55g1_supported_codes); i++) {
		if (vd55g1_supported_codes[i].code == code)
			return vd55g1_supported_codes[i].bpp;
	}
	/* Should never happen */
	WARN(1, "Unsupported code %d. default to 8 bpp", code);
	return 8;
}

static u8 get_data_type_by_code(__u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vd55g1_supported_codes); i++) {
		if (vd55g1_supported_codes[i].code == code)
			return vd55g1_supported_codes[i].data_type;
	}
	/* Should never happen */
	WARN(1, "Unsupported code %d. default to MIPI_CSI2_DT_RAW8 data type",
	     code);
	return MIPI_CSI2_DT_RAW8;
}

static s32 get_pixel_rate(struct vd55g1_dev *sensor)
{
	return div64_u64((u64)sensor->data_rate_in_mbps * sensor->nb_of_lane,
			 get_bpp_by_code(sensor->fmt.code));
}

static int get_chunk_size(struct vd55g1_dev *sensor)
{
	int max_write_len = VD55G1_WRITE_MULTIPLE_CHUNK_MAX;
	struct i2c_adapter *adapter = sensor->i2c_client->adapter;

	if (adapter->quirks && adapter->quirks->max_write_len)
		max_write_len = adapter->quirks->max_write_len - 2;

	max_write_len = min(max_write_len, VD55G1_WRITE_MULTIPLE_CHUNK_MAX);

	return max(max_write_len, 1);
}

static int vd55g1_read_multiple(struct vd55g1_dev *sensor, u32 reg,
				unsigned int len)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	u8 val[sizeof(u32)] = {0};
	int ret;

	if (len > sizeof(u32))
		return -EINVAL;
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: %x i2c_transfer, reg: %x => %d\n",
			__func__, client->addr, reg, ret);
		return ret;
	}

	return get_unaligned_le32(val);
}

static inline int vd55g1_read_reg(struct vd55g1_dev *sensor, u32 reg)
{
	return vd55g1_read_multiple(sensor, reg & VD55G1_REG_ADDR_MASK,
				     (reg >> VD55G1_REG_SIZE_SHIFT) & 7);
}

static int vd55g1_write_multiple(struct vd55g1_dev *sensor, u32 reg,
				 const u8 *data, unsigned int len, int *err)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[VD55G1_WRITE_MULTIPLE_CHUNK_MAX + 2];
	unsigned int i;
	int ret;

	if (err && *err)
		return *err;

	if (len > VD55G1_WRITE_MULTIPLE_CHUNK_MAX || len == 0)
		return -EINVAL;
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	for (i = 0; i < len; i++)
		buf[i + 2] = data[i];

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = len + 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: i2c_transfer, reg: %x => %d\n",
			__func__, reg, ret);
		if (err)
			*err = ret;
		return ret;
	}

	return 0;
}

static int vd55g1_write_array(struct vd55g1_dev *sensor, u32 reg,
			      unsigned int nb, const u8 *array)
{
	const unsigned int chunk_size = get_chunk_size(sensor);
	int ret;
	unsigned int sz;

	while (nb) {
		sz = min(nb, chunk_size);
		ret = vd55g1_write_multiple(sensor, reg, array, sz, NULL);
		if (ret < 0)
			return ret;
		nb -= sz;
		reg += sz;
		array += sz;
	}

	return 0;
}

static inline int vd55g1_write_reg(struct vd55g1_dev *sensor, u32 reg, u32 val,
				   int *err)
{
	return vd55g1_write_multiple(sensor, reg & VD55G1_REG_ADDR_MASK,
				     (u8 *)&val,
				     (reg >> VD55G1_REG_SIZE_SHIFT) & 7, err);
}

static int vd55g1_poll_reg(struct vd55g1_dev *sensor, u32 reg, u8 poll_val,
			   unsigned int timeout_ms)
{
	const unsigned int loop_delay_ms = 10;
	int ret;
#if KERNEL_VERSION(5, 7, 0) > LINUX_VERSION_CODE
	int loop_nb = timeout_ms / loop_delay_ms;

	while (--loop_nb) {
		ret = vd55g1_read_reg(sensor, reg);
		if (ret < 0)
			return ret;
		if (ret == poll_val)
			return 0;
		msleep(loop_delay_ms);
	}
	return -ETIMEDOUT;
#else
	return read_poll_timeout(vd55g1_read_reg, ret,
				 ((ret < 0) || (ret == poll_val)),
				 loop_delay_ms * 1000, timeout_ms * 1000,
				 false, sensor, reg);
#endif
}

static int vd55g1_wait_state(struct vd55g1_dev *sensor, int state,
			     unsigned int timeout_ms)
{
	return vd55g1_poll_reg(sensor, VD55G1_REG_SYSTEM_FSM, state,
			       timeout_ms);
}

static int vd55g1_update_gpio_mode(struct vd55g1_dev *sensor, u32 mode,
				   int gpio)
{
	u8 index2val[] = {0x01, 0x02, 0x06, 0x0a};
	int ret;

	if (sensor->hdr == VD55G1_HDR_SUB && mode == VD55G1_GPIO_MODE_STROBE) {
		/* Make its context 1 counterpart strobe too */
		ret = vd55g1_write_reg(sensor, VD55G1_REG_GPIO_0_CTRL(0) + gpio,
				       index2val[mode], NULL);
		if (ret)
			return ret;
	}

	return vd55g1_write_reg(sensor, VD55G1_REG_GPIO_0_CTRL(0) + gpio,
				index2val[mode], NULL);
}

static int vd55g1_set_gpios_array(struct vd55g1_dev *sensor, u32 *array,
				  int size, enum vd55g1_gpio_modes mode)
{
	unsigned int i;
	int ret;

	for (i = 0; i < size;  i++) {
		if (array[i] == ~0)
			break;
		ret = vd55g1_update_gpio_mode(sensor, mode, array[i]);
		if (ret)
			return -EINVAL;
	}

	return 0;
}

static int vd55g1_apply_exposure_auto(struct vd55g1_dev *sensor)
{
	enum vd55g1_expo_state exp = sensor->expo_state;
	int ret;

	if (sensor->hdr == VD55G1_HDR_SUB) {
		ret = vd55g1_write_reg(sensor, VD55G1_REG_EXP_MODE(1),
				       VD55G1_EXP_BYPASS, NULL);
		if (ret)
			return ret;
	}

	if (sensor->ae_frozen && sensor->expo_state == VD55G1_EXP_AUTO)
		exp = VD55G1_EXP_FREEZE;

	return vd55g1_write_reg(sensor, VD55G1_REG_EXP_MODE(0), exp, NULL);
}

static int vd55g1_get_regulators(struct vd55g1_dev *sensor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vd55g1_supply_name); i++)
		sensor->supplies[i].supply = vd55g1_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       ARRAY_SIZE(vd55g1_supply_name),
				       sensor->supplies);
}

static int vd55g1_apply_patgen(struct vd55g1_dev *sensor)
{
	static const u8 index2val[] = {
		0x0, 0x22, 0x28
	};
	u32 pattern = index2val[sensor->pattern];
	u32 reg = pattern << VD55G1_PATGEN_TYPE_SHIFT;
	u8 darkcal = VD55G1_DARKCAL_AUTO;
	u8 duster = VD55G1_DUSTER_RING_ENABLE | VD55G1_DUSTER_DYN_ENABLE |
		    VD55G1_DUSTER_ENABLE;
	int ret = 0;

	if (sensor->pattern != 0) {
		reg |= VD55G1_PATGEN_ENABLE;
		/*
		 * Take care of dark calibaration and duster to not mess up the
		 * test pattern output.
		 */
		darkcal = VD55G1_DARKCAL_BYPASS;
		duster = VD55G1_DUSTER_DISABLE;
	}

	vd55g1_write_reg(sensor, VD55G1_REG_DARKCAL_CTRL, darkcal, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_DUSTER_CTRL, duster, &ret);
	if (ret)
		return ret;

	return vd55g1_write_reg(sensor, VD55G1_REG_PATGEN_CTRL, reg, NULL);
}

static int vd55g1_apply_flash(struct vd55g1_dev *sensor)
{
	struct vd55g1_gpios *gpios = &sensor->gpios;

	enum vd55g1_gpio_modes mode = sensor->flash_en ?
				      VD55G1_GPIO_MODE_STROBE :
				      VD55G1_GPIO_MODE_DISABLED;

	return vd55g1_set_gpios_array(sensor, gpios->leds,
				      ARRAY_SIZE(gpios->leds), mode);
}

static int vd55g1_apply_darkcal_pedestal(struct vd55g1_dev *sensor)
{
	int ret = 0;

	vd55g1_write_reg(sensor, VD55G1_REG_DARKCAL_PEDESTAL(0),
			 sensor->darkcal_pedestal, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_DARKCAL_PEDESTAL(1),
			 sensor->darkcal_pedestal, &ret);

	return ret;
}

static int vd55g1_update_exposure_auto(struct vd55g1_dev *sensor, u32 index)
{
	int ret;

	switch (index) {
	case V4L2_EXPOSURE_AUTO:
		sensor->expo_state = VD55G1_EXP_AUTO;
		break;
	case V4L2_EXPOSURE_MANUAL:
		sensor->expo_state = VD55G1_EXP_MANUAL;
		break;
	default:
		/* Should never happen */
		ret = -EINVAL;
	}

	if (sensor->streaming)
		return vd55g1_apply_exposure_auto(sensor);
	return 0;
}

static int vd55g1_lock_exposure(struct vd55g1_dev *sensor,
				struct v4l2_ctrl *ctrl)
{
	/* Only exposure lock is supported */
	bool ae_lock = ctrl->val & V4L2_LOCK_EXPOSURE;
	int ret;

	sensor->ae_frozen = ae_lock;

	/* Cap control value to reflect the hardware state */
	ctrl->val = ae_lock;

	ret = vd55g1_apply_exposure_auto(sensor);
	if (ret)
		return ret;
	return 0;
}

static int vd55g1_get_temp_stream_enable(struct vd55g1_dev *sensor, int *temp)
{
	int temperature;

	temperature = vd55g1_read_reg(sensor, VD55G1_REG_TEMPERATURE);
	if (temperature < 0)
		return temperature;

	/* temperature is signed 10 bits value. extend sign */
	temperature = (temperature << 6) >> 6;
	*temp = temperature;

	return 0;
}

static int vd55g1_apply_framelength(struct vd55g1_dev *sensor)
{
	int ret = 0;

	if (sensor->hdr == VD55G1_HDR_SUB) {
		vd55g1_write_reg(sensor, VD55G1_REG_FRAME_LENGTH(1),
				 sensor->frame_length, &ret);
		if (ret)
			return ret;
	}

	return vd55g1_write_reg(sensor, VD55G1_REG_FRAME_LENGTH(0),
				sensor->frame_length, NULL);
}

static int vd55g1_update_vblank(struct vd55g1_dev *sensor, u16 vblank)
{
	sensor->frame_length = sensor->current_mode->crop.height +
			       sensor->vblank;

	if (sensor->streaming)
		return vd55g1_apply_framelength(sensor);
	return 0;
}

static int vd55g1_get_temp_stream_disable(struct vd55g1_dev *sensor, int *temp)
{
	int ret;

	/* request temperature read */
	ret = vd55g1_write_reg(sensor, VD55G1_REG_STBY,
			       VD55G1_STBY_THSENS_READ, NULL);
	if (ret)
		return ret;
	ret = vd55g1_poll_reg(sensor, VD55G1_REG_STBY, 0, VD55G1_TIMEOUT_MS);
	if (ret)
		return ret;

	return vd55g1_get_temp_stream_enable(sensor, temp);
}

static int vd55g1_get_temp(struct vd55g1_dev *sensor, int *temp)
{
	*temp = 0;
	if (sensor->streaming)
		return vd55g1_get_temp_stream_enable(sensor, temp);
	else
		return vd55g1_get_temp_stream_disable(sensor, temp);
}

static int vd55g1_update_analog_gain(struct vd55g1_dev *sensor, u32 target)
{
	sensor->analog_gain = target;

	if (sensor->streaming)
		return vd55g1_write_reg(sensor, VD55G1_REG_MANUAL_ANALOG_GAIN,
					target, NULL);
	return 0;
}

static int vd55g1_update_digital_gain(struct vd55g1_dev *sensor, u32 target)
{
	sensor->digital_gain = target;

	if (sensor->streaming)
		return vd55g1_write_reg(sensor, VD55G1_REG_MANUAL_DIGITAL_GAIN,
					target, NULL);
	return 0;
}

static int vd55g1_update_exposure(struct vd55g1_dev *sensor, int expo_ms)
{
	sensor->manual_expo = expo_ms;
	if (sensor->streaming)
		return vd55g1_write_reg(sensor,
					VD55G1_REG_MANUAL_COARSE_EXPOSURE,
					sensor->manual_expo, NULL);

	return 0;
}

static int vd55g1_update_darkcal_pedestal(struct vd55g1_dev *sensor,
					  int pedestal)
{
	sensor->darkcal_pedestal = pedestal;
	if (sensor->streaming)
		return vd55g1_apply_darkcal_pedestal(sensor);

	return 0;
}

static int vd55g1_update_exposure_target(struct vd55g1_dev *sensor, int index)
{
	/*
	 * Find auto exposure target with: default target exposure * 2^EV
	 * Defaut target exposure being 27 for the sensor.
	 */
	static const unsigned int index2exposure_target[] = {
		3, 5, 7, 10, 14, 19, 27, 38, 54, 76, 108, 153, 216,
	};

	sensor->exposure_target = index2exposure_target[index];
	if (sensor->streaming)
		return vd55g1_write_reg(sensor, VD55G1_REG_AE_TARGET_PERCENTAGE,
					sensor->exposure_target, NULL);

	return 0;
}

static int vd55g1_update_flash(struct vd55g1_dev *sensor, int flash_en)
{
	sensor->flash_en = flash_en;
	if (sensor->streaming)
		return vd55g1_apply_flash(sensor);

	return 0;
}

static int vd55g1_apply_reset(struct vd55g1_dev *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
	return vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_READY_TO_BOOT,
				 VD55G1_TIMEOUT_MS);
}

static void vd55g1_fill_framefmt(struct vd55g1_dev *sensor,
				 const struct vd55g1_mode_info *mode,
				 struct v4l2_mbus_framefmt *fmt, u32 code)
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

static int vd55g1_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   const struct vd55g1_mode_info **new_mode)
{
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	const struct vd55g1_mode_info *mode = vd55g1_mode_data;
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(vd55g1_supported_codes); index++) {
		if (vd55g1_supported_codes[index].code == fmt->code)
			break;
	}
	if (index == ARRAY_SIZE(vd55g1_supported_codes))
		index = 0;

	mode = v4l2_find_nearest_size(vd55g1_mode_data,
				      ARRAY_SIZE(vd55g1_mode_data), width,
				      height, fmt->width, fmt->height);
	if (new_mode)
		*new_mode = mode;

	vd55g1_fill_framefmt(sensor, mode, fmt,
			     vd55g1_supported_codes[index].code);

	return 0;
}

static int vd55g1_apply_cold_start(struct vd55g1_dev *sensor)
{
	/*
	 * Cold start register is a single register expressed as exposure time
	 * in us. This differ from status registers being a combination of
	 * exposure, digital gain, and analog gain, requiring the following
	 * format conversion.
	 */
	unsigned int line_time_us = DIV_ROUND_UP(sensor->line_length * MEGA,
						 sensor->pclk);
	u8 d_gain = DIV_ROUND_CLOSEST(sensor->cold_start.digital_gain, 1 << 8);
	u8 a_gain = DIV_ROUND_CLOSEST(32,
				      (32 - sensor->cold_start.analog_gain));
	unsigned int expo_us = sensor->cold_start.expo * d_gain * a_gain *
			       line_time_us;
	int ret = 0;

	vd55g1_write_reg(sensor, VD55G1_REG_AE_FORCE_COLDSTART, 1, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_AE_COLDSTART_EXP_TIME, expo_us,
			 &ret);

	return ret;
}

static int vd55g1_apply_hdr_mode(struct vd55g1_dev *sensor)
{
	int ret = 0;

	switch (sensor->hdr) {
	case VD55G1_NO_HDR:
		vd55g1_write_reg(sensor, VD55G1_REG_EXPOSURE_USE_CASES, 0,
				 &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_NEXT_CTX, 0x0, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_CTX_REPEAT_COUNT(0), 1,
				 &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_VT_MODE(0),
				 VD55G1_VT_MODE_NORMAL, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_MASK_FRAME_CTRL(0),
				 VD55G1_MASK_FRAME_CTRL_OUTPUT, &ret);
		break;
	case VD55G1_HDR_SUB:
		vd55g1_write_reg(sensor, VD55G1_REG_EXPOSURE_USE_CASES,
				 VD55G1_EXPOSURE_USE_CASES_MULTI_CONTEXT, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_NEXT_CTX, 0x1, &ret);

		vd55g1_write_reg(sensor, VD55G1_REG_CTX_REPEAT_COUNT(0), 1,
				 &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_VT_MODE(0),
				 VD55G1_VT_MODE_NORMAL, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_MASK_FRAME_CTRL(0),
				 VD55G1_MASK_FRAME_CTRL_MASK, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_EXPOSURE_INSTANCE(0), 0,
				 &ret);

		vd55g1_write_reg(sensor, VD55G1_REG_CTX_REPEAT_COUNT(1), 1,
				 &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_VT_MODE(1),
				 VD55G1_VT_MODE_SUBTRACTION, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_MASK_FRAME_CTRL(1),
				 VD55G1_MASK_FRAME_CTRL_OUTPUT, &ret);
		vd55g1_write_reg(sensor, VD55G1_REG_EXPOSURE_INSTANCE(1), 1,
				 &ret);
		break;
	default:
		/* Should never happen */
		WARN(1, "Unsupported hdr mode %d", sensor->hdr);
		ret = -EINVAL;
	}

	return ret;
}

static int vd55g1_apply_settings(struct vd55g1_dev *sensor)
{
	int ret;

	ret = vd55g1_apply_framelength(sensor);
	if (ret)
		return ret;

	ret = vd55g1_apply_exposure_auto(sensor);
	if (ret)
		return ret;

	ret = vd55g1_apply_hdr_mode(sensor);
	if (ret)
		return ret;

	vd55g1_write_reg(sensor, VD55G1_REG_MANUAL_COARSE_EXPOSURE,
			 sensor->manual_expo, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_MANUAL_ANALOG_GAIN,
			 sensor->analog_gain, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_MANUAL_DIGITAL_GAIN,
			 sensor->digital_gain, &ret);
	if (ret)
		return ret;

	ret = vd55g1_apply_cold_start(sensor);
	if (ret)
		return ret;

	vd55g1_write_reg(sensor, VD55G1_REG_ORIENTATION,
			 sensor->hflip | (sensor->vflip << 1), &ret);

	ret = vd55g1_apply_darkcal_pedestal(sensor);
	if (ret)
		return ret;

	ret = vd55g1_apply_patgen(sensor);
	if (ret)
		return ret;

	return 0;
}

static int vd55g1_apply_frame_format(struct vd55g1_dev *sensor)
{
	const struct v4l2_rect *crop = &sensor->current_mode->crop;
	int ret = 0;

	vd55g1_write_reg(sensor, VD55G1_REG_READOUT_CTRL,
			 sensor->current_mode->bin_mode, &ret);

	vd55g1_write_reg(sensor, VD55G1_REG_X_START(0), crop->left, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_X_WIDTH(0), crop->width, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_Y_START(0), crop->top, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_Y_HEIGHT(0), crop->height, &ret);

	vd55g1_write_reg(sensor, VD55G1_REG_X_START(1), crop->left, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_X_WIDTH(1), crop->width, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_Y_START(1), crop->top, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_Y_HEIGHT(1), crop->height, &ret);

	return ret;
}

static int vd55g1_set_gpios(struct vd55g1_dev *sensor)
{
	struct vd55g1_gpios *gpios = &sensor->gpios;
	int ret;
	unsigned int i;

	/* GPIOs in input (disabled) by default */
	for (i = 0; i < VD55G1_NB_GPIOS; i++) {
		ret = vd55g1_update_gpio_mode(sensor,
					      VD55G1_GPIO_MODE_DISABLED, i);
		if (ret)
			return ret;
	}

	ret = vd55g1_apply_flash(sensor);
	if (ret)
		return ret;

	ret = vd55g1_set_gpios_array(sensor, gpios->out_sync,
				     ARRAY_SIZE(gpios->out_sync),
				     VD55G1_GPIO_MODE_VSYNC_OUT_0);
	if (ret)
		return ret;

	if (!sensor->is_slave)
		return 0;

	ret = vd55g1_update_gpio_mode(sensor, VD55G1_GPIO_MODE_VTSLAVE,
				      gpios->in_sync);
	if (ret)
		return -EINVAL;
	ret = vd55g1_write_reg(sensor, VD55G1_REG_VT_CTRL, VD55G1_VT_SLAVE_GPIO,
			       NULL);

	return ret;
}

static int vd55g1_stream_enable(struct vd55g1_dev *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret;

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		pm_runtime_put_autosuspend(&client->dev);
		return ret;
	}

	/* pm_runtime_get_sync() can return 1 as a valid return code */
	ret = 0;

	ret = vd55g1_set_gpios(sensor);
	if (ret)
		goto err_rpm_put;

	vd55g1_write_reg(sensor, VD55G1_REG_FORMAT_CTRL,
			 get_bpp_by_code(sensor->fmt.code), &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_OIF_IMG_CTRL,
			 get_data_type_by_code(sensor->fmt.code), &ret);
	if (ret)
		goto err_rpm_put;

	ret = vd55g1_apply_frame_format(sensor);
	if (ret)
		goto err_rpm_put;

	ret = vd55g1_apply_settings(sensor);
	if (ret)
		goto err_rpm_put;

	ret = vd55g1_write_reg(sensor, VD55G1_REG_STBY,
			       VD55G1_STBY_START_STREAM, NULL);
	if (ret)
		goto err_rpm_put;

	ret = vd55g1_poll_reg(sensor, VD55G1_REG_STBY, 0, VD55G1_TIMEOUT_MS);
	if (ret)
		goto err_rpm_put;

	ret = vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_STREAMING,
				VD55G1_TIMEOUT_MS);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void vd55g1_save_exposure(struct vd55g1_dev *sensor)
{
	int ret;

	ret = vd55g1_read_reg(sensor, VD55G1_REG_APPLIED_COARSE_EXPOSURE);
	sensor->cold_start.expo = ret < 0 ? 0 : ret;
	ret = vd55g1_read_reg(sensor, VD55G1_REG_APPLIED_DIGITAL_GAIN);
	sensor->cold_start.digital_gain = ret < 0 ? 0 : ret;
	ret = vd55g1_read_reg(sensor, VD55G1_REG_APPLIED_ANALOG_GAIN);
	sensor->cold_start.analog_gain = ret < 0 ? 0 : ret;
}

static int vd55g1_stream_disable(struct vd55g1_dev *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret;

	/* Keep exposure values for next cold start boot */
	vd55g1_save_exposure(sensor);

	ret = vd55g1_write_reg(sensor, VD55G1_REG_STREAMING,
			       VD55G1_STREAMING_STOP_STREAM, NULL);
	if (ret)
		goto err_str_dis;

	ret = vd55g1_poll_reg(sensor, VD55G1_REG_STREAMING, 0, 2000);
	if (ret)
		goto err_str_dis;

	ret = vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_SW_STBY,
				VD55G1_TIMEOUT_MS);

err_str_dis:
	if (ret)
		WARN(1, "Can't disable stream");
	pm_runtime_put(&client->dev);

	return ret;
}

static int vd55g1_tx_from_ep(struct vd55g1_dev *sensor,
			     struct fwnode_handle *handle)
{
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_fwnode_endpoint *ep;
	u32 log2phy[VD55G1_NB_POLARITIES] = {~0, ~0, ~0};
	u32 phy2log[VD55G1_NB_POLARITIES] = {~0, ~0, ~0};
	int polarities[VD55G1_NB_POLARITIES] = {0, 0, 0};
	int l_nb;
	int p, l;
	int i;

#if KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE
	ep = v4l2_fwnode_endpoint_alloc_parse(handle);
#else
	struct v4l2_fwnode_endpoint ep_node = { .bus_type =
		V4L2_MBUS_CSI2_DPHY
	};
	int ret;

	ep = &ep_node;
	ret = v4l2_fwnode_endpoint_alloc_parse(handle, ep);
	if (ret)
		return -EINVAL;
#endif

	l_nb = ep->bus.mipi_csi2.num_data_lanes;
	if (l_nb != 1 && l_nb != 2) {
		dev_err(&client->dev, "invalid data lane number %d\n", l_nb);
		goto error_ep;
	}

	/* build  log2phy, phy2log and polarities from ep info */
	log2phy[0] = ep->bus.mipi_csi2.clock_lane;
	phy2log[log2phy[0]] = 0;
	for (l = 1; l < l_nb + 1; l++) {
		log2phy[l] = ep->bus.mipi_csi2.data_lanes[l - 1];
		phy2log[log2phy[l]] = l;
	}
	/*
	 * then fill remaining slots for every physical slot have something
	 * valid for hardware stuff.
	 */
	for (p = 0; p < VD55G1_NB_POLARITIES; p++) {
		if (phy2log[p] != ~0)
			continue;
		phy2log[p] = l;
		log2phy[l] = p;
		l++;
	}
	for (l = 0; l < l_nb + 1; l++)
		polarities[l] = ep->bus.mipi_csi2.lane_polarities[l];

	if (log2phy[0] != 0) {
		dev_err(&client->dev, "clk lane must be map to physical lane 0\n");
		goto error_ep;
	}
	sensor->oif_ctrl = l_nb |
			   (polarities[0] << 3) |
			   ((phy2log[1] - 1) << 4) |
			   (polarities[1] << 6) |
			   ((phy2log[2] - 1) << 7) |
			   (polarities[2] << 9);
	sensor->nb_of_lane = l_nb;

	dev_dbg(&client->dev, "tx uses %d lanes", l_nb);
	for (i = 0; i < VD55G1_NB_POLARITIES; i++) {
		dev_dbg(&client->dev, "log2phy[%d] = %d", i, log2phy[i]);
		dev_dbg(&client->dev, "phy2log[%d] = %d", i, phy2log[i]);
		dev_dbg(&client->dev, "polarity[%d] = %d", i, polarities[i]);
	}
	dev_dbg(&client->dev, "oif_ctrl = 0x%04x\n", sensor->oif_ctrl);

	v4l2_fwnode_endpoint_free(ep);

	return 0;

error_ep:
	v4l2_fwnode_endpoint_free(ep);

	return -EINVAL;
}

static int vd55g1_patch(struct vd55g1_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int patch, ret;

	ret = vd55g1_write_array(sensor, VD55G1_REG_FWPATCH_START_ADDR,
				 sizeof(patch_array), patch_array);
	if (ret)
		return ret;

	ret = vd55g1_write_reg(sensor, VD55G1_REG_BOOT,
			       VD55G1_BOOT_PATCH_SETUP, NULL);
	if (ret)
		return ret;

	ret = vd55g1_poll_reg(sensor, VD55G1_REG_BOOT, 0, VD55G1_TIMEOUT_MS);
	if (ret)
		return ret;

	patch = vd55g1_read_reg(sensor, VD55G1_REG_FWPATCH_REVISION);
	if (patch < 0)
		return patch;

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

static int vd55g1_configure(struct vd55g1_dev *sensor)
{
	/* Double data rate */
	u32 mipi_bps = link_freq[0] * 2;
	u32 req_line_length = (sensor->current_mode->crop.width *
			       get_bpp_by_code(sensor->fmt.code) +
			       VD55G1_MIPI_MARGIN) / VD55G1_PCLK_DIVISOR;
	u32 min_line_length;
	int ret = 0;

	/* Frequency to data rate is 1:1 ratio for MIPI */
	sensor->data_rate_in_mbps = mipi_bps;
	/* Video timing ISP path (pixel clock)  requires 804/5 mhz = 160 mhz */
	sensor->pclk = mipi_bps / VD55G1_PCLK_DIVISOR;

	min_line_length = VD55G1_MIN_LINE_LENGTH;
	if (sensor->hdr == VD55G1_HDR_SUB)
		min_line_length = VD55G1_MIN_LINE_LENGTH_SUB;
	sensor->line_length = max(min_line_length, req_line_length);

	vd55g1_write_reg(sensor, VD55G1_REG_LINE_LENGTH, sensor->line_length,
			 &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_EXT_CLOCK, sensor->clk_freq, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_OIF_CTRL, sensor->oif_ctrl, &ret);
	vd55g1_write_reg(sensor, VD55G1_REG_MIPI_DATA_RATE, mipi_bps, &ret);

	return ret;
}

static inline bool vd55g1_can_be_slave(struct vd55g1_dev *sensor)
{
	return sensor->gpios.in_sync != ~0;
}

static void vd55g1_update_hblank_ctrl(struct vd55g1_dev *sensor)
{
	int height = sensor->current_mode->crop.height;

	if (sensor->hdr == VD55G1_HDR_SUB)
		__v4l2_ctrl_modify_range(sensor->vblank_ctrl,
					 sensor->vblank_min * 2 + height,
					 0xffff - height * 2, 1,
					 sensor->vblank);
	else
		__v4l2_ctrl_modify_range(sensor->vblank_ctrl,
					 sensor->vblank_min,
					 0xffff - height, 1,
					 sensor->vblank);
	__v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, sensor->vblank);
}

static int vd55g1_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	ret = enable ? vd55g1_stream_enable(sensor) :
		       vd55g1_stream_disable(sensor);
	if (!ret)
		sensor->streaming = enable;

	mutex_unlock(&sensor->lock);

	if (!ret) {
		/* These settings cannot change during streaming */
		v4l2_ctrl_grab(sensor->vflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->hflip_ctrl, enable);
		v4l2_ctrl_grab(sensor->pattern_ctrl, enable);
		v4l2_ctrl_grab(sensor->hdr_ctrl, enable);
		if (vd55g1_can_be_slave(sensor))
			v4l2_ctrl_grab(sensor->slave_ctrl, enable);
	}

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
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = sensor->current_mode->crop;
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
	if (code->index >= ARRAY_SIZE(vd55g1_supported_codes))
		return -EINVAL;

	code->code = vd55g1_supported_codes[code->index].code;

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
#else
static int vd55g1_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
#endif
{
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
#else
		fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state,
						 format->pad);
#endif
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
#else
static int vd55g1_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
#endif
{
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	const struct vd55g1_mode_info *new_mode;
	struct v4l2_mbus_framefmt *fmt;
	unsigned int expo_max, hblank;
	int ret;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = vd55g1_try_fmt_internal(sd, &format->format, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
#else
		fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
#endif
		*fmt = format->format;
	} else if (sensor->current_mode != new_mode ||
		   sensor->fmt.code != format->format.code) {
		fmt = &sensor->fmt;
		*fmt = format->format;

		sensor->current_mode = new_mode;

		/* Reset vblank and framelength to default */
		ret = vd55g1_update_vblank(sensor,
					   VD55G1_FRAME_LENGTH_DEF -
					   new_mode->crop.height);

		/* Update controls to reflect new mode */
		__v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate_ctrl,
					 get_pixel_rate(sensor));
		vd55g1_update_hblank_ctrl(sensor);
		/* Max exposure changes with vblank */
		expo_max = sensor->frame_length - VD55G1_EXPO_MAX_TERM;
		__v4l2_ctrl_modify_range(sensor->expo_ctrl, 0, expo_max, 1,
					 VD55G1_EXPO_DEF);
		/* Update hblank according to new width */
		hblank = sensor->line_length - sensor->current_mode->width;
		__v4l2_ctrl_modify_range(sensor->hblank_ctrl, hblank, hblank, 1,
					 hblank);
		ret = __v4l2_ctrl_s_ctrl(sensor->hblank_ctrl, hblank);
	}

out:
	mutex_unlock(&sensor->lock);

	return ret;
}

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
static int vd55g1_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
#else
static int vd55g1_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state)
#endif
{
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	struct v4l2_subdev_format fmt = { 0 };

	vd55g1_fill_framefmt(sensor, sensor->current_mode, &fmt.format,
			     VD55G1_MEDIA_BUS_FMT_DEF);

#if KERNEL_VERSION(5, 14, 0) > LINUX_VERSION_CODE
	return vd55g1_set_fmt(sd, cfg, &fmt);
#else
	return vd55g1_set_fmt(sd, sd_state, &fmt);
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
	if (fse->index >= ARRAY_SIZE(vd55g1_mode_data))
		return -EINVAL;

	fse->min_width = vd55g1_mode_data[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = vd55g1_mode_data[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static const struct v4l2_subdev_core_ops vd55g1_core_ops = {
};

static const struct v4l2_subdev_video_ops vd55g1_video_ops = {
	.s_stream = vd55g1_s_stream,
};

static const struct v4l2_subdev_pad_ops vd55g1_pad_ops = {
	.init_cfg = vd55g1_init_cfg,
	.enum_mbus_code = vd55g1_enum_mbus_code,
	.get_fmt = vd55g1_get_fmt,
	.set_fmt = vd55g1_set_fmt,
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
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	int temperature;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		ret = __v4l2_ctrl_s_ctrl_int64(ctrl, get_pixel_rate(sensor));
		break;
	case V4L2_CID_TEMPERATURE:
		ret = vd55g1_get_temp(sensor, &temperature);
		if (ret)
			break;
		ret = __v4l2_ctrl_s_ctrl(ctrl, temperature);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vd55g1_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
	unsigned int expo_max;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		if (sensor->streaming) {
			ret = -EBUSY;
			break;
		}
		if (ctrl->id == V4L2_CID_VFLIP)
			sensor->vflip = ctrl->val;
		if (ctrl->id == V4L2_CID_HFLIP)
			sensor->hflip = ctrl->val;
		ret = 0;
		break;
	case V4L2_CID_TEST_PATTERN:
		/* Can't be done while streaming because of duster disabling */
		sensor->pattern = ctrl->val;
		ret = 0;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = vd55g1_update_exposure_auto(sensor, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = vd55g1_update_analog_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = vd55g1_update_digital_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = vd55g1_update_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_3A_LOCK:
		ret = vd55g1_lock_exposure(sensor, ctrl);
		break;
	case V4L2_CID_DARKCAL_PEDESTAL:
		ret = vd55g1_update_darkcal_pedestal(sensor, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = vd55g1_update_vblank(sensor, ctrl->val);
		/* Max exposure changes with vblank */
		expo_max = sensor->frame_length - VD55G1_EXPO_MAX_TERM;
		__v4l2_ctrl_modify_range(sensor->expo_ctrl, 0, expo_max, 1,
					 VD55G1_EXPO_DEF);
		break;
	case V4L2_CID_HBLANK:
		/* Read only control, can only be activated by V4L2 framework */
		ret = 0;
		break;
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		/*
		 * We use auto exposure target percentage register to control
		 * exposure bias for more precision.
		 */
		ret = vd55g1_update_exposure_target(sensor, ctrl->val);
		break;
	case V4L2_CID_SLAVE:
		sensor->is_slave = ctrl->val;
		ret = 0;
		break;
	case V4L2_CID_FLASH_LED_MODE:
		ret = vd55g1_update_flash(sensor, ctrl->val);
		break;
	case V4L2_CID_HDR_SENSOR_MODE:
		sensor->hdr = ctrl->val;
		/* Max blanking changes with hdr mode */
		vd55g1_update_hblank_ctrl(sensor);
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

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
	.id		= V4L2_CID_SLAVE,
	.name		= "VT Slave Mode",
	.type		= V4L2_CTRL_TYPE_BOOLEAN,
	.min		= 0,
	.max		= 1,
	.step		= 1,
	.def		= 1,
};

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

static int vd55g1_init_controls(struct vd55g1_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &vd55g1_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
	const struct vd55g1_mode_info *cur_mode = sensor->current_mode;
	struct v4l2_ctrl *ctrl;
	unsigned int patgen_size = ARRAY_SIZE(vd55g1_test_pattern_menu) - 1;
	unsigned int hblank = sensor->line_length - sensor->current_mode->width;
	unsigned int expo_mode = sensor->expo_state == VD55G1_EXP_AUTO ?
		V4L2_EXPOSURE_AUTO :  V4L2_EXPOSURE_MANUAL;
	unsigned int vblank_default = sensor->vblank_min * 2 +
		sensor->current_mode->crop.height;
	unsigned int vblank_max = 0xffff - cur_mode->crop.height * 2;
	int ret;

	v4l2_ctrl_handler_init(hdl, 16);
	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;
	v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_AUTO_EXPOSURE_BIAS,
			       ARRAY_SIZE(vd55g1_ev_bias_menu) - 1,
			       ARRAY_SIZE(vd55g1_ev_bias_menu) / 2,
			       vd55g1_ev_bias_menu);
	ctrl = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq) - 1, 0, link_freq);
	ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO, 1, ~0x3,
			       expo_mode);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, 0, 24, 1,
			  sensor->analog_gain);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN, 256, 2048, 1,
			  sensor->digital_gain);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK, 0, 1, 0, 0);
	ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_temp_ctrl, NULL);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_custom(hdl, &vd55g1_darkcal_pedestal_ctrl, NULL);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_FLASH, ~0x7,
			       sensor->flash_en);

	/*
	 * Keep a pointer to these controls as we need to update them when
	 * setting the format
	 */
	sensor->pixel_rate_ctrl = v4l2_ctrl_new_std(hdl, ops,
						    V4L2_CID_PIXEL_RATE, 1,
						    INT_MAX, 1,
						    get_pixel_rate(sensor));
	if (sensor->pixel_rate_ctrl)
		sensor->pixel_rate_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->vblank_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
						vblank_default, vblank_max,
						1, sensor->vblank);
	hblank = sensor->line_length - sensor->current_mode->width;
	sensor->hblank_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
						hblank, hblank, 1, hblank);
	if (sensor->hblank_ctrl)
		sensor->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->vflip_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					       0, 1, 1, sensor->vflip);
	sensor->hflip_ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					       0, 1, 1, sensor->hflip);
	sensor->pattern_ctrl =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     patgen_size, 0, 0,
					     vd55g1_test_pattern_menu);
	sensor->slave_ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_slave_ctrl,
						  NULL);
	sensor->expo_ctrl =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0,
				  sensor->frame_length - VD55G1_EXPO_MAX_TERM,
				  1, sensor->manual_expo);
	sensor->hdr_ctrl = v4l2_ctrl_new_custom(hdl, &vd55g1_hdr_ctrl, NULL);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	/* Disable this control if not possible by device tree */
	if (!vd55g1_can_be_slave(sensor)) {
		v4l2_ctrl_s_ctrl(sensor->slave_ctrl, false);
		v4l2_ctrl_grab(sensor->slave_ctrl, true);
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int vd55g1_detect_cut_version(struct vd55g1_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int device_rev;

	device_rev = vd55g1_read_reg(sensor, VD55G1_REG_REVISION);
	if (device_rev < 0)
		return device_rev;

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

static int vd55g1_detect(struct vd55g1_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret, id = 0;

	id = vd55g1_read_reg(sensor, VD55G1_REG_MODEL_ID);
	if (id < 0)
		return id;

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
static int vd55g1_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);
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

	ret = vd55g1_wait_state(sensor, VD55G1_SYSTEM_FSM_SW_STBY,
				VD55G1_TIMEOUT_MS);
	if (ret)
		return ret;

	ret = vd55g1_configure(sensor);
	if (ret) {
		dev_err(&client->dev, "sensor configuration failed %d", ret);
		goto disable_clock;
	}

	return 0;

disable_clock:
	clk_disable_unprepare(sensor->xclk);
disable_bulk:
	regulator_bulk_disable(ARRAY_SIZE(vd55g1_supply_name),
			       sensor->supplies);

	return ret;
}

static int vd55g1_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);

	clk_disable_unprepare(sensor->xclk);
	regulator_bulk_disable(ARRAY_SIZE(vd55g1_supply_name),
			       sensor->supplies);
	return 0;
}

static int vd55g1_parse_dt_gpios_array(struct vd55g1_dev *sensor,
				       char *prop_name, u32 *array, int *nb)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device_node *np = client->dev.of_node;
	unsigned int i;

	*nb = of_property_read_variable_u32_array(np, prop_name, array, 0,
						  VD55G1_NB_GPIOS);
	*nb = max(0, *nb);
	if (*nb > 0) {
		for (i = 0; i < *nb;  i++) {
			if (array[i] >= VD55G1_NB_GPIOS) {
				dev_err(&client->dev, "invalid GPIO %d for leds\n",
					array[i]);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int vd55g1_parse_dt_gpios(struct vd55g1_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device_node *np = client->dev.of_node;
	struct vd55g1_gpios *gpios = &sensor->gpios;
	int nb_gpios_leds, nb_gpios_out;
	int ret;
	unsigned int i, j;

	memset(gpios->leds, ~0,
	       ARRAY_SIZE(gpios->leds) * sizeof(gpios->leds[0]));
	memset(gpios->out_sync, ~0,
	       ARRAY_SIZE(gpios->out_sync) * sizeof(gpios->out_sync[0]));
	gpios->in_sync = ~0;

	ret = vd55g1_parse_dt_gpios_array(sensor, "st,leds",
					  (u32 *)&gpios->leds,
					  &nb_gpios_leds);
	if (ret)
		return ret;

	ret = vd55g1_parse_dt_gpios_array(sensor, "st,out-sync",
					  (u32 *)&gpios->out_sync,
					  &nb_gpios_out);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "st,in-sync", &gpios->in_sync);
	if (ret == 0) {
		if (gpios->in_sync != 0) {
			dev_err(&client->dev, "input sync gpio must be 0 if present, found %d\n",
				gpios->in_sync);
			return -EINVAL;
		}

		/* Check no other gpios array use gpio 0 */
		for (i = 0; i < nb_gpios_leds;  i++) {
			if (gpios->leds[i] == gpios->in_sync) {
				dev_err(&client->dev, "in-sync GPIO %d is used by another led gpio\n",
					gpios->in_sync);
				return -EINVAL;
			}
		}
		for (i = 0; i < nb_gpios_out;  i++) {
			if (gpios->out_sync[i] == gpios->in_sync) {
				dev_err(&client->dev, "in-sync GPIO %d is used by another out-sync gpio\n",
					gpios->in_sync);
				return -EINVAL;
			}
		}

		dev_dbg(&client->dev, "GPIO %d in input slave mode\n",
			gpios->in_sync);

		sensor->is_slave = true;
	}

	/* Check mutual exclusivity between leds and out_sync */
	for (i = 0; i < nb_gpios_leds;  i++) {
		for (j = 0; j < nb_gpios_out;  j++) {
			if (gpios->leds[i] == gpios->out_sync[j]) {
				dev_err(&client->dev, "GPIO %d used in both leds and out-sync\n",
					gpios->leds[i]);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int vd55g1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *handle;
	struct vd55g1_dev *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->manual_expo = VD55G1_EXPO_DEF;
	sensor->analog_gain = 19;
	sensor->digital_gain = 256;

	sensor->i2c_client = client;
	sensor->streaming = false;
	sensor->vflip = false;
	sensor->hflip = false;
	sensor->darkcal_pedestal = VD55G1_DARKCAL_PEDESTAL_DEF;
	sensor->flash_en = false;
	sensor->hdr = VD55G1_NO_HDR;
	sensor->vblank = 4000;
	sensor->vblank_min = VD55G1_MIN_VBLANK;

	sensor->cold_start.expo = sensor->manual_expo;
	sensor->cold_start.digital_gain = sensor->digital_gain;
	sensor->cold_start.analog_gain = sensor->analog_gain;

	sensor->current_mode = &vd55g1_mode_data[VD55G1_DEFAULT_MODE];

	handle = fwnode_graph_get_next_endpoint(of_fwnode_handle(dev->of_node),
						NULL);
	if (!handle) {
		dev_err(dev, "handle node not found\n");
		return -EINVAL;
	}

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	ret = vd55g1_parse_dt_gpios(sensor);
	if (ret) {
		dev_err(dev, "Failed to get gpios\n");
		return ret;
	}

	ret = vd55g1_tx_from_ep(sensor, handle);
	fwnode_handle_put(handle);
	if (ret) {
		dev_err(dev, "Failed to parse handle %d\n", ret);
		return ret;
	}

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}
	sensor->clk_freq = clk_get_rate(sensor->xclk);
	if (sensor->clk_freq < 6 * HZ_PER_MHZ ||
	    sensor->clk_freq > 27 * HZ_PER_MHZ) {
		dev_err(dev, "Only 6Mhz-27Mhz clock range supported. provide %lu MHz\n",
			sensor->clk_freq / HZ_PER_MHZ);
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &vd55g1_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &vd55g1_subdev_entity_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = vd55g1_get_regulators(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to get regulators %d", ret);
		return ret;
	}

	vd55g1_fill_framefmt(sensor, sensor->current_mode, &sensor->fmt,
			     VD55G1_MEDIA_BUS_FMT_DEF);

	/* Check the sensor is the correct one and can be powered on */
	ret = vd55g1_power_on(dev);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = vd55g1_update_vblank(sensor, VD55G1_FRAME_LENGTH_DEF -
				   sensor->current_mode->crop.height);
	if (ret)
		goto error_power_off;

	ret = vd55g1_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "controls initialization failed %d", ret);
		goto error_power_off;
	}

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err(&client->dev, "pads init failed %d", ret);
		goto error_handler_free;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "async subdev register failed %d", ret);
		goto error_pm_runtime;
	}

	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);

	dev_dbg(&client->dev, "vd55g1 probe successfully");

	return 0;

error_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	media_entity_cleanup(&sensor->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);
error_power_off:
	mutex_destroy(&sensor->lock);
	vd55g1_power_off(dev);

	return ret;
}

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
static int vd55g1_remove(struct i2c_client *client)
#else
static void vd55g1_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vd55g1_dev *sensor = to_vd55g1_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		vd55g1_power_off(&client->dev);
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

static const struct dev_pm_ops vd55g1_pm_ops = {
	SET_RUNTIME_PM_OPS(vd55g1_power_off, vd55g1_power_on, NULL)
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
