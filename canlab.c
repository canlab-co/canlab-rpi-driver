// SPDX-License-Identifier: GPL-2.0
/*
 * CANLAB MIPI CSI-2 dual virtual-channel test source (DOWNSTREAM variant)
 *
 * For the Raspberry Pi downstream RP1 CFE driver ("raspberrypi,rp1-cfe"),
 * which uses the classic per-pad-link model (NO V4L2 streams/routing API,
 * so it works on the stock RPi OS kernel where the streams API is disabled).
 *
 * Presents THREE source pads, linked by the CFE (in ascending order) to
 * csi2 sink pads / channels:
 *   pad0 -> csi2 sink0 -> csi2_ch0 : VC0 1920x1080 UYVY
 *   pad1 -> csi2 sink1 -> embedded : filler for the meta-only ch1 (unused)
 *   pad2 -> csi2 sink2 -> csi2_ch2 : VC1 640x480  UYVY
 *
 * The pad layout is DICTATED BY THE DOWNSTREAM DRIVER, not copied from any
 * example:
 *   - cfe_link_node_pads() (rp1_cfe/cfe.c) links the sensor's source pads to
 *     csi2 sink pads / channels strictly in ascending order (pad0->ch0,
 *     pad1->ch1, pad2->ch2, ...).
 *   - node_desc[CSI2_CH1] is "embedded" (V4L2_CAP_META_CAPTURE only), so an
 *     image VC cannot use ch1.
 * Hence two image VCs must land on ch0 and ch2, which forces a filler source
 * pad (pad1) to occupy the ch1 slot.
 *
 * 4 D-PHY data lanes, CONTINUOUS clock, 500 MHz link (1000 Mbps/lane).
 *
 * Per-channel VC/DT is read by csi2_get_vc_dt() from this driver's
 * pad::get_frame_desc (exactly ONE CSI-2 entry per source pad). The source
 * is started/stopped via the video .s_stream op.
 *
 * NOTE: The device-specific register programming that makes the source
 * actually transmit is left as a TODO (depends on the CANLAB datasheet).
 * The V4L2/Media plumbing is complete.
 *
 * Copyright (c) 2026
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

/* Source pads (all MEDIA_PAD_FL_SOURCE). Index == CFE channel it maps to. */
#define CANLAB_PAD_VC0		0	/* -> ch0 */
#define CANLAB_PAD_EMB		1	/* -> ch1 (embedded/meta, filler) */
#define CANLAB_PAD_VC1		2	/* -> ch2 */
#define CANLAB_NUM_PADS		3

#define CANLAB_NUM_DATA_LANES		4
#define CANLAB_DEFAULT_LINK_FREQ_HZ	500000000ULL	/* clock lane = 500 MHz */

#define CANLAB_MBUS_CODE	MEDIA_BUS_FMT_UYVY8_1X16	/* UYVY, CSI-2 DT 0x1e */
#define CANLAB_BPP		16

/*
 * Reset = the "vdd" supply (CAM IO0 / cam1_reg) wired to the FPGA i_arstn
 * (active-low): enable -> run, disable -> hold in reset.
 */

#define CANLAB_MIN_WIDTH	64u
#define CANLAB_MIN_HEIGHT	64u
#define CANLAB_MAX_WIDTH	1920u
#define CANLAB_MAX_HEIGHT	1080u

/* VC1 firmware variants: VGA (default) or QVGA ("canlab,vc1-qvga" DT prop) */
#define CANLAB_VC1_VGA_WIDTH	640u
#define CANLAB_VC1_VGA_HEIGHT	480u
#define CANLAB_VC1_QVGA_WIDTH	320u
#define CANLAB_VC1_QVGA_HEIGHT	240u

struct canlab_pad_cfg {
	u32 width;
	u32 height;
	u8 vc;
	u8 dt;
};

/* Template; VC1 width/height are overridden per-instance in probe(). */
static const struct canlab_pad_cfg canlab_pad_cfg_template[CANLAB_NUM_PADS] = {
	[CANLAB_PAD_VC0] = { 1920, 1080, 0, MIPI_CSI2_DT_YUV422_8B },
	[CANLAB_PAD_EMB] = { 1920, 1080, 0, MIPI_CSI2_DT_YUV422_8B }, /* filler */
	[CANLAB_PAD_VC1] = { CANLAB_VC1_VGA_WIDTH, CANLAB_VC1_VGA_HEIGHT,
			      1, MIPI_CSI2_DT_YUV422_8B },
};

struct canlab {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pads[CANLAB_NUM_PADS];

	struct v4l2_ctrl_handler ctrls;

	struct clk *xclk;			/* optional */
	struct gpio_desc *reset_gpio;		/* optional secondary reset */
	struct regulator *supply;		/* CAM IO0 (cam1_reg) = i_arstn reset line */

	unsigned int num_data_lanes;
	unsigned int bus_flags;			/* V4L2_MBUS_CSI2_* */
	s64 link_freq;				/* Hz */
	s64 link_freq_menu[1];

	struct mutex lock;			/* protects streaming state + HW */
	bool streaming;

	/* Per-instance copy of canlab_pad_cfg_template; VC1 entry may be
	 * overridden to QVGA dimensions based on the "canlab,vc1-qvga"
	 * device property. */
	struct canlab_pad_cfg pad_cfg[CANLAB_NUM_PADS];
};

static inline struct canlab *to_canlab(struct v4l2_subdev *sd)
{
	return container_of(sd, struct canlab, sd);
}

static void canlab_fill_default_fmt(struct v4l2_mbus_framefmt *fmt,
				    const struct canlab_pad_cfg *cfg)
{
	fmt->code = CANLAB_MBUS_CODE;
	fmt->width = cfg->width;
	fmt->height = cfg->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_709;
}

/* -------------------------------------------------------------------------
 * Device-specific register programming 
 * ------------------------------------------------------------------------- */

static int canlab_start(struct canlab *c)
{
	struct device *dev = &c->client->dev;

	dev_dbg(dev, "start: %u lanes, %lld Hz, %s clock\n",
		c->num_data_lanes, c->link_freq,
		(c->bus_flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) ?
			"non-continuous" : "continuous");
	return 0;
}

static void canlab_stop(struct canlab *c)
{
	dev_dbg(&c->client->dev, "stop\n");
}

/* -------------------------------------------------------------------------
 * Power management
 * ------------------------------------------------------------------------- */

/*
 * Generate a genuine i_arstn reset PULSE rather than only deasserting it.
 * If the camera is already powered and running, i_arstn sits high (FPGA
 * design already up), so merely driving it high again produces no edge:
 * the design's reset logic (and any dependent PLL relock / CSI-2
 * clock-lane LP->HS entry) never re-triggers. Force the line low first,
 * hold it briefly, then release it and wait for PLL relock + design
 * bring-up before the CSI-2 output is valid.
 */
static int canlab_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct canlab *c = to_canlab(sd);
	int ret;

	/*
	 * Non-continuous clock only (release build): the D-PHY clock lane
	 * performs a genuine LP->HS entry every frame, so CFE's automatic
	 * FSM re-syncs on its own. No deliberate i_arstn reset pulse is
	 * needed or supported here -- i_arstn's physical connection to the
	 * FPGA is expected to be absent on this board class. This function
	 * only brings up steady power.
	 */
	ret = clk_prepare_enable(c->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	if (c->supply) {
		ret = regulator_enable(c->supply);
		if (ret) {
			dev_err(dev, "failed to enable supply: %d\n", ret);
			clk_disable_unprepare(c->xclk);
			return ret;
		}
	}
	gpiod_set_value_cansleep(c->reset_gpio, 0);

	return 0;
}

static int canlab_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct canlab *c = to_canlab(sd);

	/*
	 * Assert i_arstn to hold the FPGA design in reset so it stops driving
	 * the CSI-2 output (D-PHY -> LP-11). Held until the next stream start.
	 */
	gpiod_set_value_cansleep(c->reset_gpio, 1);
	if (c->supply)
		regulator_disable(c->supply);
	clk_disable_unprepare(c->xclk);

	return 0;
}

/* -------------------------------------------------------------------------
 * V4L2 subdev operations
 * ------------------------------------------------------------------------- */

static int canlab_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	unsigned int pad;

	for (pad = 0; pad < CANLAB_NUM_PADS; pad++) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_state_get_format(state, pad);
		if (fmt)
			canlab_fill_default_fmt(fmt, &to_canlab(sd)->pad_cfg[pad]);
	}

	return 0;
}

static int canlab_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;

	/* Only UYVY is generated; clamp geometry to even, in-range values. */
	format->format.code = CANLAB_MBUS_CODE;
	format->format.width = clamp(format->format.width,
				     CANLAB_MIN_WIDTH, CANLAB_MAX_WIDTH) & ~1u;
	format->format.height = clamp(format->format.height,
				      CANLAB_MIN_HEIGHT, CANLAB_MAX_HEIGHT) & ~1u;
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
	format->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	format->format.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	format->format.xfer_func = V4L2_XFER_FUNC_709;

	fmt = v4l2_subdev_state_get_format(state, format->pad);
	if (!fmt)
		return -EINVAL;
	*fmt = format->format;

	return 0;
}

static int canlab_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;

	if (pad >= CANLAB_NUM_PADS)
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	fmt = v4l2_subdev_state_get_format(state, pad);

	fd->entry[0].stream = 0;
	fd->entry[0].flags = 0;
	fd->entry[0].pixelcode = fmt ? fmt->code : CANLAB_MBUS_CODE;
	fd->entry[0].bus.csi2.vc = to_canlab(sd)->pad_cfg[pad].vc;
	fd->entry[0].bus.csi2.dt = to_canlab(sd)->pad_cfg[pad].dt;

	v4l2_subdev_unlock_state(state);

	return 0;
}

static int canlab_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct canlab *c = to_canlab(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = c->num_data_lanes;
	cfg->bus.mipi_csi2.flags = c->bus_flags;

	return 0;
}

static int canlab_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct canlab *c = to_canlab(sd);
	struct device *dev = &c->client->dev;
	int ret = 0;

	mutex_lock(&c->lock);

	if (enable) {
		if (c->streaming) {
			//dev_info(dev, "s_stream(1): duplicate enable ignored\n");
			goto out;
		}

		ret = pm_runtime_resume_and_get(dev);
		if (ret < 0)
			goto out;

		ret = canlab_start(c);
		if (ret) {
			pm_runtime_put(dev);
			goto out;
		}
		c->streaming = true;
		//dev_info(dev, "s_stream(1): streaming started\n");
	} else {
		if (!c->streaming) {
			//dev_info(dev, "s_stream(0): duplicate disable ignored\n");
			goto out;
		}

		canlab_stop(c);
		c->streaming = false;
		pm_runtime_put_sync(dev);
		/*dev_info(dev, "s_stream(0): stopped, runtime status now %s\n",
			 pm_runtime_status_suspended(dev) ? "suspended" : "active");*/
	}

out:
	mutex_unlock(&c->lock);
	return ret;
}

static const struct v4l2_subdev_pad_ops canlab_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = canlab_set_fmt,
	.get_frame_desc = canlab_get_frame_desc,
	.get_mbus_config = canlab_get_mbus_config,
};

static const struct v4l2_subdev_video_ops canlab_video_ops = {
	.s_stream = canlab_s_stream,
};

static const struct v4l2_subdev_ops canlab_subdev_ops = {
	.video = &canlab_video_ops,
	.pad = &canlab_pad_ops,
};

static const struct v4l2_subdev_internal_ops canlab_internal_ops = {
	.init_state = canlab_init_state,
};

static const struct media_entity_operations canlab_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -------------------------------------------------------------------------
 * Controls
 * ------------------------------------------------------------------------- */

static int canlab_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;	/* all controls are read-only */
}

static const struct v4l2_ctrl_ops canlab_ctrl_ops = {
	.s_ctrl = canlab_s_ctrl,
};

static int canlab_init_controls(struct canlab *c)
{
	struct v4l2_ctrl_handler *hdl = &c->ctrls;
	struct v4l2_ctrl *ctrl;
	s64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 2);
	if (ret)
		return ret;

	c->link_freq_menu[0] = c->link_freq;
	ctrl = v4l2_ctrl_new_int_menu(hdl, &canlab_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, c->link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = div_s64(c->link_freq * 2 * c->num_data_lanes, CANLAB_BPP);
	ctrl = v4l2_ctrl_new_std(hdl, &canlab_ctrl_ops, V4L2_CID_PIXEL_RATE,
				 pixel_rate, pixel_rate, 1, pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	c->sd.ctrl_handler = hdl;
	return 0;
}

/* -------------------------------------------------------------------------
 * Probe / remove
 * ------------------------------------------------------------------------- */

static int canlab_parse_dt(struct canlab *c)
{
	struct device *dev = &c->client->dev;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	c->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
	c->bus_flags = bus_cfg.bus.mipi_csi2.flags;

	if (c->num_data_lanes != CANLAB_NUM_DATA_LANES)
		dev_warn(dev, "expected %u data lanes, DT provides %u\n",
			 CANLAB_NUM_DATA_LANES, c->num_data_lanes);
	if (!(c->bus_flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK))
		dev_warn(dev,
			 "'clock-noncontinuous' not set; this release driver has no\n"
			 "i_arstn reset-pulse fallback and REQUIRES non-continuous\n"
			 "clock -- add 'clock-noncontinuous;' to the sensor endpoint.\n");

	if (bus_cfg.nr_of_link_frequencies)
		c->link_freq = bus_cfg.link_frequencies[0];
	else
		c->link_freq = CANLAB_DEFAULT_LINK_FREQ_HZ;

	v4l2_fwnode_endpoint_free(&bus_cfg);
	return 0;
}

static int canlab_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct canlab *c;
	unsigned int i;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->client = client;
	mutex_init(&c->lock);

	memcpy(c->pad_cfg, canlab_pad_cfg_template, sizeof(c->pad_cfg));
	if (device_property_read_bool(dev, "canlab,vc1-qvga")) {
		c->pad_cfg[CANLAB_PAD_VC1].width = CANLAB_VC1_QVGA_WIDTH;
		c->pad_cfg[CANLAB_PAD_VC1].height = CANLAB_VC1_QVGA_HEIGHT;
	}

	ret = canlab_parse_dt(c);
	if (ret)
		goto err_mutex;

	c->xclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(c->xclk)) {
		ret = dev_err_probe(dev, PTR_ERR(c->xclk), "failed to get clock\n");
		goto err_mutex;
	}

	c->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c->reset_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(c->reset_gpio),
				    "failed to get reset gpio\n");
		goto err_mutex;
	}

	/* "vdd" supply (CAM IO0 / cam1_reg) = the i_arstn reset line. */
	c->supply = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(c->supply)) {
		ret = PTR_ERR(c->supply);
		if (ret != -ENODEV) {
			dev_err_probe(dev, ret, "failed to get vdd supply\n");
			goto err_mutex;
		}
		c->supply = NULL;
	}

	v4l2_i2c_subdev_init(&c->sd, client, &canlab_subdev_ops);
	c->sd.internal_ops = &canlab_internal_ops;
	c->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;	/* no FL_STREAMS */
	c->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	c->sd.entity.ops = &canlab_entity_ops;

	for (i = 0; i < CANLAB_NUM_PADS; i++)
		c->pads[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&c->sd.entity, CANLAB_NUM_PADS, c->pads);
	if (ret)
		goto err_mutex;

	ret = canlab_init_controls(c);
	if (ret)
		goto err_entity;

	ret = v4l2_subdev_init_finalize(&c->sd);
	if (ret)
		goto err_ctrls;

	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev(&c->sd);
	if (ret) {
		dev_err(dev, "failed to register async subdev: %d\n", ret);
		goto err_pm;
	}

	dev_info(dev,
		 "CANLAB(downstream) 2-VC: VC0 %ux%u(pad0/ch0) + VC1 %ux%u(pad2/ch2) UYVY, %u lanes, %lld Hz %s\n",
		 c->pad_cfg[CANLAB_PAD_VC0].width, c->pad_cfg[CANLAB_PAD_VC0].height,
		 c->pad_cfg[CANLAB_PAD_VC1].width, c->pad_cfg[CANLAB_PAD_VC1].height,
		 c->num_data_lanes, c->link_freq,
		 (c->bus_flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) ?
			"non-continuous" : "continuous");

	return 0;

err_pm:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(&c->sd);
err_ctrls:
	v4l2_ctrl_handler_free(&c->ctrls);
err_entity:
	media_entity_cleanup(&c->sd.entity);
err_mutex:
	mutex_destroy(&c->lock);
	return ret;
}

static void canlab_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct canlab *c = to_canlab(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&c->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		canlab_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&c->lock);
}

static const struct dev_pm_ops canlab_pm_ops = {
	SET_RUNTIME_PM_OPS(canlab_power_off, canlab_power_on, NULL)
};

static const struct of_device_id canlab_of_match[] = {
	{ .compatible = "canlab,csi2-2vc" },
	{ }
};
MODULE_DEVICE_TABLE(of, canlab_of_match);

static const struct i2c_device_id canlab_id[] = {
	{ "canlab-csi2-2vc" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, canlab_id);

static struct i2c_driver canlab_i2c_driver = {
	.driver = {
		.name = "canlab",
		.of_match_table = canlab_of_match,
		.pm = &canlab_pm_ops,
	},
	.probe = canlab_probe,
	.remove = canlab_remove,
	.id_table = canlab_id,
};
module_i2c_driver(canlab_i2c_driver);

MODULE_DESCRIPTION("CANLAB MIPI CSI-2 dual virtual-channel test source (downstream)");
MODULE_AUTHOR("Camera Driver Development");
MODULE_LICENSE("GPL");
