// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 AXI Backend driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/units.h>

#include <linux/iio/backend.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>

#include <linux/iio/frequency/ad9910.h>

/* Register addresses */
#define AD9910_AXI_REG_RESET		0x040
#define AD9910_AXI_REG_DRG		0x084
#define AD9910_AXI_REG_PROFILE		0x088
#define AD9910_AXI_REG_IO_UPDATE	0x08C
#define AD9910_AXI_REG_RAMP_CFG		0x090
#define AD9910_AXI_REG_RAMP_DELAY_BST	0x094
#define AD9910_AXI_REG_RAMP_PERIOD	0x098
#define AD9910_AXI_REG_BURST_DELAY	0x09C
#define AD9910_AXI_REG_BURST_COUNT	0x0A0
#define AD9910_AXI_REG_UPDATE_CTRL	0x104
#define AD9910_AXI_REG_PAR_RATE		0x108
#define AD9910_AXI_REG_F_CFG		0x10C

#define AD9910_AXI_REG_RESET_CORE_MSK		BIT(0)
#define AD9910_AXI_REG_RESET_IO_MSK		BIT(1)
#define AD9910_AXI_REG_RESET_DEV_MSK		BIT(2)
#define AD9910_AXI_REG_RESET_PW_DOWN_MSK	BIT(3)

#define AD9910_AXI_REG_DRG_DRCTL_MSK		BIT(2)
#define AD9910_AXI_REG_DRG_TOGGLE_EN_MSK	BIT(3)
#define AD9910_AXI_REG_DRG_OPER_MODE_MSK	GENMASK(5, 4)

#define AD9910_AXI_REG_RAMP_CFG_ONE_SHOT_MSK	GENMASK(1, 0)
#define AD9910_AXI_REG_RAMP_PERIOD_MSK		GENMASK(23, 0)
#define AD9910_AXI_REG_BURST_COUNT_MSK		GENMASK(19, 0)

#define AD9910_AXI_REG_UPDATE_RATE_MSK		BIT(0)
#define AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK	BIT(1)
#define AD9910_AXI_REG_UPDATE_EXT_TRIG_MSK	BIT(2)

#define AD9910_AXI_REG_PROFILE_MSK		GENMASK(2, 0)
#define AD9910_AXI_REG_IO_UPDATE_MSK		BIT(0)
#define AD9910_AXI_REG_F_CFG_MSK		GENMASK(1, 0)

#define AD9910_AXI_PD_CLK_DEFAULT_FREQ_UHZ	(250ULL * HZ_PER_MHZ * MICROHZ_PER_HZ)
#define AD9910_AXI_PAR_RATE_MAX			U16_MAX
#define AD9910_AXI_ONE_SHOT_BURST		1

enum {
	AD9910_AXI_PP_SAMPLE_RATE,
	AD9910_AXI_DRG_RAMP_FREQUENCY,
	AD9910_AXI_DRG_BURST_COUNT,
	AD9910_AXI_DRG_BURST_DELAY,
};

struct ad9910_axi_state {
	struct reset_controller_dev rc;
	struct device *dev;
	struct regmap *regmap;
	/*
	 * lock to protect multiple accesses to the device registers and global
	 * data/variables.
	 */
	struct mutex lock;
	u64 pd_clk_freq_uhz;
};

#define ad9910_axi_from_rst_ctrl(st)	\
	container_of(st, struct ad9910_axi_state, rc)

static int ad9910_axi_chan_enable(struct iio_backend *back, unsigned int chan)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	int ret;

	guard(mutex)(&st->lock);

	switch (chan) {
	case AD9910_CHANNEL_PHY:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
					 AD9910_AXI_REG_RESET_PW_DOWN_MSK);
	case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
		chan -= AD9910_CHANNEL_PROFILE_0;
		return regmap_update_bits(st->regmap, AD9910_AXI_REG_PROFILE,
					  AD9910_AXI_REG_PROFILE_MSK,
					  FIELD_PREP(AD9910_AXI_REG_PROFILE_MSK, chan));
	case AD9910_CHANNEL_PARALLEL_AMP ... AD9910_CHANNEL_PARALLEL_POLAR:
		chan -= AD9910_CHANNEL_PARALLEL_AMP;
		ret = regmap_update_bits(st->regmap, AD9910_AXI_REG_F_CFG,
					 AD9910_AXI_REG_F_CFG_MSK, chan);
		if (ret)
			return ret;

		return regmap_set_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
				       AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_chan_disable(struct iio_backend *back, unsigned int chan)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	guard(mutex)(&st->lock);

	switch (chan) {
	case AD9910_CHANNEL_PHY:
		return regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
				       AD9910_AXI_REG_RESET_PW_DOWN_MSK);
	case AD9910_CHANNEL_PARALLEL_AMP ... AD9910_CHANNEL_PARALLEL_POLAR:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
					 AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK);
	default:
		return -EINVAL;
	}
}

static struct iio_buffer *ad9910_axi_request_buffer(struct iio_backend *back,
						    struct iio_dev *indio_dev)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	const char *dma_name;

	if (device_property_read_string(st->dev, "dma-names", &dma_name))
		dma_name = "tx";

	return iio_dmaengine_buffer_setup_ext(st->dev, indio_dev, dma_name,
					      IIO_BUFFER_DIRECTION_OUT);
}

static void ad9910_axi_free_buffer(struct iio_backend *back,
				   struct iio_buffer *buffer)
{
	iio_dmaengine_buffer_teardown(buffer);
}

static int ad9910_axi_ext_info_set(struct iio_backend *back, uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	u32 tmp32;
	u64 tmp64;
	int ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_AXI_PP_SAMPLE_RATE:
		ret = kstrtoudec64(buf, 6, &tmp64);
		if (ret)
			return ret;

		if (!tmp64)
			return -EINVAL;

		tmp64 = DIV64_U64_ROUND_CLOSEST(st->pd_clk_freq_uhz, tmp64);
		tmp32 = clamp(tmp64, 1U, AD9910_AXI_PAR_RATE_MAX);
		ret = regmap_write(st->regmap, AD9910_AXI_REG_PAR_RATE, tmp32 - 1);
		if (ret)
			return ret;

		ret = regmap_set_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
				      AD9910_AXI_REG_UPDATE_RATE_MSK);
		break;
	case AD9910_AXI_DRG_RAMP_FREQUENCY:
		ret = kstrtoudec64(buf, 6, &tmp64);
		if (ret)
			return ret;

		if (!tmp64) {
			ret = regmap_clear_bits(st->regmap, AD9910_AXI_REG_DRG,
						AD9910_AXI_REG_DRG_TOGGLE_EN_MSK);
			break;
		}

		ret = regmap_set_bits(st->regmap, AD9910_AXI_REG_DRG,
				      AD9910_AXI_REG_DRG_TOGGLE_EN_MSK);

		tmp64 = DIV64_U64_ROUND_CLOSEST(st->pd_clk_freq_uhz, tmp64);
		tmp32 = clamp_t(u32, tmp64, 2U, FIELD_MAX(AD9910_AXI_REG_RAMP_PERIOD_MSK));
		ret = regmap_write(st->regmap, AD9910_AXI_REG_RAMP_PERIOD, tmp32);
		break;
	case AD9910_AXI_DRG_BURST_COUNT:
		ret = kstrtou32(buf, 10, &tmp32);
		if (ret)
			return ret;

		tmp32 = min(tmp32, FIELD_MAX(AD9910_AXI_REG_BURST_COUNT_MSK));
		ret = regmap_write(st->regmap, AD9910_AXI_REG_BURST_COUNT, tmp32);
		break;
	case AD9910_AXI_DRG_BURST_DELAY:
		ret = kstrtoudec64(buf, 9, &tmp64);
		if (ret)
			return ret;

		ret = regmap_clear_bits(st->regmap, AD9910_AXI_REG_RAMP_CFG,
					AD9910_AXI_REG_RAMP_CFG_ONE_SHOT_MSK);
		if (ret)
			return ret;

		if (!tmp64) {
			ret = regmap_update_bits(st->regmap, AD9910_AXI_REG_RAMP_CFG,
						 AD9910_AXI_REG_RAMP_CFG_ONE_SHOT_MSK,
						 AD9910_AXI_ONE_SHOT_BURST);
			if (ret)
				return ret;
		}

		tmp64 = mul_u64_u64_div_u64(tmp64, st->pd_clk_freq_uhz,
					    1ULL * NANO * MICROHZ_PER_HZ);
		tmp32 = min_t(u32, tmp64, U32_MAX);
		ret = regmap_write(st->regmap, AD9910_AXI_REG_BURST_DELAY, tmp32);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

static int ad9910_axi_ext_info_get(struct iio_backend *back, uintptr_t private,
				   const struct iio_chan_spec *chan, char *buf)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	int vals[2], ret;
	u32 tmp32;
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_AXI_PP_SAMPLE_RATE:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_PAR_RATE, &tmp32);
		if (ret)
			return ret;

		tmp32++;
		tmp64 = div_u64(st->pd_clk_freq_uhz, tmp32);
		iio_val_s64_decompose(tmp64, &vals[0], &vals[1]);
		return iio_format_value(buf, IIO_VAL_DECIMAL64_MICRO,
					ARRAY_SIZE(vals), vals);
	case AD9910_AXI_DRG_RAMP_FREQUENCY:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_DRG, &tmp32);
		if (ret)
			return ret;

		if (!FIELD_GET(AD9910_AXI_REG_DRG_TOGGLE_EN_MSK, tmp32))
			return sysfs_emit(buf, "0\n");

		ret = regmap_read(st->regmap, AD9910_AXI_REG_RAMP_PERIOD, &tmp32);
		if (ret)
			return ret;

		if (!tmp32)
			return -ERANGE;

		tmp64 = div_u64(st->pd_clk_freq_uhz, tmp32);
		iio_val_s64_decompose(tmp64, &vals[0], &vals[1]);
		return iio_format_value(buf, IIO_VAL_DECIMAL64_MICRO,
					ARRAY_SIZE(vals), vals);
	case AD9910_AXI_DRG_BURST_COUNT:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_BURST_COUNT, &tmp32);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n", tmp32);
	case AD9910_AXI_DRG_BURST_DELAY:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_BURST_DELAY, &tmp32);
		if (ret)
			return ret;

		tmp64 = mul_u64_u64_div_u64((u64)tmp32 * NANO, MICROHZ_PER_HZ,
					    st->pd_clk_freq_uhz);
		iio_val_s64_decompose(tmp64, &vals[0], &vals[1]);
		return iio_format_value(buf, IIO_VAL_DECIMAL64_NANO,
					ARRAY_SIZE(vals), vals);
	default:
		return -EINVAL;
	}
}

#define AD9910_AXI_BACKEND_EXT_INFO(_name, _private) { \
	.name = _name, \
	.shared = IIO_SEPARATE, \
	.read = ad9910_axi_ext_info_get, \
	.write = ad9910_axi_ext_info_set, \
	.private = _private, \
}

static const struct iio_backend_chan_ext_info ad9910_axi_pp_ext_info[] = {
	/*
	 * Even though sampling_frequency is a standard IIO channel attribute,
	 * we define it as an extended attribute here since the backend
	 * channel extension mechanism is designed for extended attributes only.
	 */
	AD9910_AXI_BACKEND_EXT_INFO("sampling_frequency", AD9910_AXI_PP_SAMPLE_RATE),
	{ }
};

static const struct iio_backend_chan_ext_info ad9910_axi_drg_ext_info[] = {
	AD9910_AXI_BACKEND_EXT_INFO("frequency", AD9910_AXI_DRG_RAMP_FREQUENCY),
	AD9910_AXI_BACKEND_EXT_INFO("burst_count", AD9910_AXI_DRG_BURST_COUNT),
	AD9910_AXI_BACKEND_EXT_INFO("burst_delay", AD9910_AXI_DRG_BURST_DELAY),
	{ }
};

static int ad9910_axi_chan_spec(struct iio_backend *back,
				const struct iio_chan_spec *chan,
				const struct iio_backend_chan_ext_info **ext_info)
{
	switch (chan->channel) {
	case AD9910_CHANNEL_PARALLEL_AMP ... AD9910_CHANNEL_PARALLEL_POLAR:
		*ext_info = ad9910_axi_pp_ext_info;
		break;
	case AD9910_CHANNEL_DRG:
		*ext_info = ad9910_axi_drg_ext_info;
		break;
	default:
		*ext_info = NULL;
		break;
	}

	return 0;
}

static int ad9910_axi_set_sample_rate(struct iio_backend *back,
				      unsigned int chan,
				      u64 sample_rate)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	guard(mutex)(&st->lock);

	switch (chan) {
	case AD9910_CHANNEL_PHY:
		if (!sample_rate || sample_rate > HZ_PER_GHZ)
			return -EINVAL;
		st->pd_clk_freq_uhz = sample_rate * MICROHZ_PER_HZ >> 2;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_reg_access(struct iio_backend *back, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	guard(mutex)(&st->lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_backend_ops ad9910_axi_iio_back_ops = {
	.chan_enable = ad9910_axi_chan_enable,
	.chan_disable = ad9910_axi_chan_disable,
	.request_buffer = ad9910_axi_request_buffer,
	.free_buffer = ad9910_axi_free_buffer,
	.extend_chan_spec = ad9910_axi_chan_spec,
	.set_sample_rate = ad9910_axi_set_sample_rate,
	.debugfs_reg_access = iio_backend_debugfs_ptr(ad9910_axi_reg_access),
};

static int ad9910_axi_io_update(struct iio_backend *back)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	guard(mutex)(&st->lock);

	return regmap_set_bits(st->regmap, AD9910_AXI_REG_IO_UPDATE,
			       AD9910_AXI_REG_IO_UPDATE_MSK);
}

static int ad9910_axi_drg_oper_mode_set(struct iio_backend *back,
					enum ad9910_drg_oper_mode mode)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, AD9910_AXI_REG_DRG,
				  AD9910_AXI_REG_DRG_OPER_MODE_MSK,
				  FIELD_PREP(AD9910_AXI_REG_DRG_OPER_MODE_MSK, mode));
}

static const struct ad9910_backend_ops ad9910_axi_back_ops = {
	.io_update = ad9910_axi_io_update,
	.drg_oper_mode_set = ad9910_axi_drg_oper_mode_set,
};

static const struct ad9910_backend_info ad9910_axi_back_info = {
	.base = {
		.name = "ad9910-axi",
		.ops = &ad9910_axi_iio_back_ops,
	},
	.ops = &ad9910_axi_back_ops,
};

static int ad9910_axi_reset_assert(struct reset_controller_dev *rc,
				   unsigned long id)
{
	struct ad9910_axi_state *st = ad9910_axi_from_rst_ctrl(rc);

	guard(mutex)(&st->lock);

	switch (id) {
	case AD9910_RESET_CTRL_DEVICE:
		return regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
				       AD9910_AXI_REG_RESET_DEV_MSK);
	case AD9910_RESET_CTRL_IO:
		return regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
				       AD9910_AXI_REG_RESET_IO_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_reset_deassert(struct reset_controller_dev *rc,
				     unsigned long id)
{
	struct ad9910_axi_state *st = ad9910_axi_from_rst_ctrl(rc);

	guard(mutex)(&st->lock);

	switch (id) {
	case AD9910_RESET_CTRL_DEVICE:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
					 AD9910_AXI_REG_RESET_DEV_MSK);
	case AD9910_RESET_CTRL_IO:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
					 AD9910_AXI_REG_RESET_IO_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_reset_status(struct reset_controller_dev *rc,
				   unsigned long id)
{
	struct ad9910_axi_state *st = ad9910_axi_from_rst_ctrl(rc);
	int ret;
	u32 tmp32;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AD9910_AXI_REG_RESET, &tmp32);
	if (ret)
		return ret;

	switch (id)
	{
	case AD9910_RESET_CTRL_DEVICE:
		return FIELD_GET(AD9910_AXI_REG_RESET_DEV_MSK, tmp32);
	case AD9910_RESET_CTRL_IO:
		return FIELD_GET(AD9910_AXI_REG_RESET_IO_MSK, tmp32);
	default:
		return -EINVAL;
	}
}

static const struct reset_control_ops ad9910_axi_reset_ops = {
	.assert = ad9910_axi_reset_assert,
	.deassert = ad9910_axi_reset_deassert,
	.status = ad9910_axi_reset_status,
};

static const struct regmap_config ad9910_axi_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int ad9910_axi_setup(struct ad9910_axi_state *st)
{
	int ret;

	st->pd_clk_freq_uhz = AD9910_AXI_PD_CLK_DEFAULT_FREQ_UHZ;

	ret = regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
			      AD9910_AXI_REG_RESET_CORE_MSK |
			      AD9910_AXI_REG_RESET_DEV_MSK |
			      AD9910_AXI_REG_RESET_IO_MSK);
	if (ret)
		return ret;

	return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
				 AD9910_AXI_REG_RESET_CORE_MSK);
}

static int ad9910_axi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad9910_axi_state *st;
	void __iomem *base;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->dev = dev;
	st->regmap = devm_regmap_init_mmio(dev, base,
					   &ad9910_axi_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to init register map\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->rc.ops = &ad9910_axi_reset_ops;
	st->rc.owner = THIS_MODULE;
	st->rc.of_node = dev->of_node;
	st->rc.nr_resets = AD9910_RESET_CTRL_MAX;

	ret = devm_reset_controller_register(dev, &st->rc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register reset controller\n");

	ret = ad9910_axi_setup(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup device\n");

	return devm_ad9910_backend_register(dev, &ad9910_axi_back_info, st);
}

static const struct of_device_id ad9910_axi_of_match[] = {
	{ .compatible = "adi,axi-ad9910" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9910_axi_of_match);

static struct platform_driver ad9910_axi_driver = {
	.driver = {
		.name = "ad9910-axi",
		.of_match_table = ad9910_axi_of_match,
	},
	.probe = ad9910_axi_probe,
};
module_platform_driver(ad9910_axi_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9910 AXI Backend driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
MODULE_IMPORT_NS(IIO_BACKEND);
MODULE_IMPORT_NS(AD9910);
