// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD5766, AD5767
 * Digital to Analog Converters driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>

#define AD5766_CMD_NOP_MUX_OUT			0x00
#define AD5766_CMD_SDO_CNTRL			0x01
#define AD5766_CMD_WR_IN_REG(x)			(0x10 | ((x) & 0xF))
#define AD5766_CMD_WR_DAC_REG(x)		(0x20 | ((x) & 0xF))
#define AD5766_CMD_SW_LDAC			0x30
#define AD5766_CMD_SPAN_REG			0x40
#define AD5766_CMD_WR_PWR_DITHER		0x51
#define AD5766_CMD_WR_DAC_REG_ALL		0x60
#define AD5766_CMD_SW_FULL_RESET		0x70
#define AD5766_CMD_READBACK_REG(x)		(0x80 | ((x) & 0xF))
#define AD5766_CMD_DITHER_SIG_1			0x90
#define AD5766_CMD_DITHER_SIG_2			0xA0
#define AD5766_CMD_INV_DITHER			0xB0
#define AD5766_CMD_DITHER_SCALE_1		0xC0
#define AD5766_CMD_DITHER_SCALE_2		0xD0

#define AD5766_FULL_RESET_CODE			0x1234

enum ad5766_type {
	ID_AD5766,
	ID_AD5767,
};

#define AD576x_CHANNEL(_chan, _bits) {						\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.output = 1,								\
	.channel = (_chan),							\
	.address = (_chan),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET) |			\
	 	BIT(IIO_CHAN_INFO_SCALE),					\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_OFFSET) |	\
		BIT(IIO_CHAN_INFO_SCALE),					\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = (_bits),						\
		.storagebits = 16,						\
		.shift = 16 - (_bits),						\
	},									\
}

#define DECLARE_AD576x_CHANNELS(_name, _bits)			\
const struct iio_chan_spec _name[] = {				\
	AD576x_CHANNEL(0, (_bits)),				\
	AD576x_CHANNEL(1, (_bits)),				\
	AD576x_CHANNEL(2, (_bits)),				\
	AD576x_CHANNEL(3, (_bits)),				\
	AD576x_CHANNEL(4, (_bits)),				\
	AD576x_CHANNEL(5, (_bits)),				\
	AD576x_CHANNEL(6, (_bits)),				\
	AD576x_CHANNEL(7, (_bits)),				\
	AD576x_CHANNEL(8, (_bits)),				\
	AD576x_CHANNEL(9, (_bits)),				\
	AD576x_CHANNEL(10, (_bits)),				\
	AD576x_CHANNEL(11, (_bits)),				\
	AD576x_CHANNEL(12, (_bits)),				\
	AD576x_CHANNEL(13, (_bits)),				\
	AD576x_CHANNEL(14, (_bits)),				\
	AD576x_CHANNEL(15, (_bits)),				\
}

enum ad5766_voltage_range {
	AD5766_VOLTAGE_RANGE_M20V_0V,
	AD5766_VOLTAGE_RANGE_M16V_to_0V,
	AD5766_VOLTAGE_RANGE_M10V_to_0V,
	AD5766_VOLTAGE_RANGE_M12V_to_14V,
	AD5766_VOLTAGE_RANGE_M16V_to_10V,
	AD5766_VOLTAGE_RANGE_M10V_to_6V,
	AD5766_VOLTAGE_RANGE_M5V_to_5V,
	AD5766_VOLTAGE_RANGE_M10V_to_10V,
	AD5766_VOLTAGE_RANGE_MAX,
};

/**
 * struct ad5766_chip_info - chip specific information
 * @num_channels:	number of channels
 * @channel:	        channel specification
 */

struct ad5766_chip_info {
	unsigned int			num_channels;
	const struct iio_chan_spec	*channels;
};

/**
 * struct ad5766_state - driver instance specific data
 * @spi:		Spi device
 * @lock:		Mutex lock
 * @chip_info:		Chip model specific constants
 * @data:		Spi transfer buffers
 * @span_range:		Current span range
 */
struct ad5766_state {
	struct spi_device		*spi;
	struct mutex			lock;
	const struct ad5766_chip_info 	*chip_info;
	enum ad5766_voltage_range	crt_range;
	enum ad5766_voltage_range	crt_scales_avail;
	s32 			scale_avail[AD5766_VOLTAGE_RANGE_MAX][2];
	s32 			crt_scale_avail[AD5766_VOLTAGE_RANGE_MAX][2];
	s32 			offset_avail[AD5766_VOLTAGE_RANGE_MAX][2];
	u8 			possible_scales[AD5766_VOLTAGE_RANGE_MAX];
	union {
		u32	d32;
		u16	w16[2];
		u8	b8[4];
	} data[3] ____cacheline_aligned;
};

// int scale_avail[AD5766_VOLTAGE_RANGE_MAX][2] = 
// 	{
// 		{0, 305175781},
// 		{0, 244140625},
// 		{0, 152587890},
// 		{0, 396728515},
// 		{0, 396728515},
// 		{0, 244140625},
// 		{0, 152587890},
// 		{0, 305175781},
// 	};

// int offset_avail[AD5766_VOLTAGE_RANGE_MAX][2] = 
// 	{
// 		{-65536, 0},
// 		{-65536, 0},
// 		{-65536, 0},
// 		{-30247, 384615384},
// 		{-40329, 846153846},
// 		{-40960, 0},
// 		{-32768, 0},
// 		{-32768, 0},
// 	};

struct ad5766_span_params {
	int		min;
	int		max;
};

static const struct ad5766_span_params ad5766_span_params[] = {
	[AD5766_VOLTAGE_RANGE_M20V_0V] = {
		.min = -20,
		.max = 0,
	},
	[AD5766_VOLTAGE_RANGE_M16V_to_0V] = {
		.min = -16,
		.max = 0,
	},
	[AD5766_VOLTAGE_RANGE_M10V_to_0V] = {
		.min = -10,
		.max = 0,
	},
	[AD5766_VOLTAGE_RANGE_M12V_to_14V] = {
		.min = -12,
		.max = 14,
	},
	[AD5766_VOLTAGE_RANGE_M16V_to_10V] = {
		.min = -16,
		.max = 10,
	},
	[AD5766_VOLTAGE_RANGE_M10V_to_6V] = {
		.min = -10,
		.max = 6,
	},
	[AD5766_VOLTAGE_RANGE_M5V_to_5V] = {
		.min = -5,
		.max = 5,
	},
	[AD5766_VOLTAGE_RANGE_M10V_to_10V] = {
		.min = -10,
		.max = 10,
	},
};

static const char *ad5766_span_ranges[] = {
	"-20V_to_0V",
	"-16V_to_0V",
	"-10V_to_0V",
	"-12V_to_14V",
	"-16V_to_10V",
	"-10V_to_6V",
	"-5V_to_5V",
	"-10V_to_10V",
	NULL
};

static int ad5766_set_offset_avail(struct ad5766_state *st)
{
	int i;
	u8 realbits = st->chip_info->channels[0].scan_type.realbits;
	printk("ad5766_set_offset_avail\n");
	for(i = 0; i < AD5766_VOLTAGE_RANGE_MAX; i++) {
		st->offset_avail[i][0] = (1 << realbits) * ad5766_span_params[i].min / (ad5766_span_params[i].max - ad5766_span_params[i].min);
		st->offset_avail[i][1] = 0;
		printk("avail0: %d, avail1: %d, min: %d, max: %d, realbits: %d\n", st->offset_avail[i][0], st->offset_avail[i][1],
		ad5766_span_params[i].min, ad5766_span_params[i].max, realbits);
	}

	return 0;
}

static int ad5766_set_scale_avail1(struct ad5766_state *st)
{
	int i;
	u8 realbits = st->chip_info->channels[0].scan_type.realbits;
	enum ad5766_voltage_range crt_range = st->crt_range;
	s32 crt_offset = (1 << realbits) * ad5766_span_params[crt_range].min / (ad5766_span_params[crt_range].max - ad5766_span_params[crt_range].min);
	s32 offset;
	u32 scale;

	printk("ad5766_set_scale_avail do_div\n");
	
	st->crt_scales_avail = 0;

	for(i = 0; i < AD5766_VOLTAGE_RANGE_MAX; i++) {
		offset = (1 << realbits) * ad5766_span_params[i].min / (ad5766_span_params[i].max - ad5766_span_params[i].min);
		if (offset == crt_offset)
		{
			scale = ad5766_span_params[i].max - ad5766_span_params[i].min;
			scale = (scale * 1000000) / (1 << realbits);

			st->scale_avail[i][0] = scale / 1000;
			st->scale_avail[i][1] = scale % 1000;
			st->scale_avail[i][1] = st->scale_avail[i][1] * 1000;
			printk("aavail0: %d, avail1: %d, min: %d, max: %d, realbits: %d, i: %d\n", st->scale_avail[i][0], st->scale_avail[i][1],
			ad5766_span_params[i].min, ad5766_span_params[i].max, realbits, i);
			st->crt_scales_avail++;
		}
	}

	return 0;
}

static int ad5766_calc_scales(struct ad5766_state *st)
{
	int i;
	u8 realbits = st->chip_info->channels[0].scan_type.realbits;
	u32 scale;

	printk("ad5766_calc_scales \n");
	
	for(i = 0; i < AD5766_VOLTAGE_RANGE_MAX; i++) {
		scale = ad5766_span_params[i].max - ad5766_span_params[i].min;
		scale = (scale * 1000000) / (1 << realbits);
		st->scale_avail[i][0] = scale / 1000;
		st->scale_avail[i][1] = scale % 1000;
		st->scale_avail[i][1] = st->scale_avail[i][1] * 1000;
		printk("avail0: %d, avail1: %d, min: %d, max: %d, realbits: %d, i: %d\n", st->scale_avail[i][0], st->scale_avail[i][1],
		ad5766_span_params[i].min, ad5766_span_params[i].max, realbits, i);
	}

	return 0;
}

static int ad5766_set_scale_avail(struct ad5766_state *st)
{
	int i;
	enum ad5766_voltage_range crt_range = st->crt_range;

	printk("ad5766_set_scale_avail crt_range %d\n", crt_range);
	
	st->crt_scales_avail = 0;

	for (i = 0; i < AD5766_VOLTAGE_RANGE_MAX; i++) {
		if (st->possible_scales[crt_range] & (1 << i)) {
			st->crt_scale_avail[st->crt_scales_avail][0] = st->scale_avail[i][0];
			st->crt_scale_avail[st->crt_scales_avail][1] = st->scale_avail[i][1];
			st->crt_scales_avail++;
		}
	}

	return 0;
}

static int ad5766_set_possible_scales(struct ad5766_state *st)
{
	int i, j;
	s32 crt_offset;
	s32 offset;
	u8 realbits = st->chip_info->channels[0].scan_type.realbits;

	printk("ad5766_set_possible_scales\n");

	for(i = 0; i < AD5766_VOLTAGE_RANGE_MAX; i++) {
		crt_offset = (1 << realbits) * ad5766_span_params[i].min / (ad5766_span_params[i].max - ad5766_span_params[i].min);
		st->possible_scales[i] = 0;
		for(j = 0; j < AD5766_VOLTAGE_RANGE_MAX; j++) {
			offset = (1 << realbits) * ad5766_span_params[j].min / (ad5766_span_params[j].max - ad5766_span_params[j].min);
			if (offset == crt_offset)
				st->possible_scales[i] |= 1 << j;
		}
		printk("st->possible_scales[%d] = %x", i, st->possible_scales[i]);
	}

	return 0;
}

static int _ad5766_spi_write(struct ad5766_state *st,
			     u8 command,
			     u16 data)
{
	st->data[0].b8[0] = command;
	st->data[0].b8[1] = (data & 0xFF00) >> 8;
	st->data[0].b8[2] = (data & 0x00FF) >> 0;

	return spi_write(st->spi, &st->data[0].b8[0], 3);
}

static int ad5766_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info);

static int ad5766_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m);

static int ad5766_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type, int *length,
				long mask);

static ssize_t ad5766_show_span_range(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad5766_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%s\n", ad5766_span_ranges[st->crt_range]);
}

static ssize_t ad5766_set_span_range(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad5766_state *st = iio_priv(indio_dev);
	long span_range;
	int ret;

	span_range = sysfs_match_string(ad5766_span_ranges, buf);
	if (span_range < 0)
		return span_range;

	ret =_ad5766_spi_write(st, 
			      AD5766_CMD_SW_FULL_RESET, 
			      AD5766_FULL_RESET_CODE);
	if (ret < 0)
		return ret;

	ret = _ad5766_spi_write(st, AD5766_CMD_SPAN_REG, span_range);
	if (ret < 0)
		return ret;

	st->crt_range = span_range;

	return len;
}

static ssize_t ad5766_show_available_span_ranges(struct device *dev,
				   		 struct device_attribute *attr,
				   		 char *buf)
{
	int ret = 0, i = 0;

	while (ad5766_span_ranges[i])
	{
		if (i > 0)
			ret += sprintf(buf + ret, " ");

		ret += sprintf(buf + ret, "%s", ad5766_span_ranges[i]);
		i++;
	}

	ret++;

	return ret;
}

static IIO_DEVICE_ATTR(span_range,
		       0644,
		       ad5766_show_span_range,
		       ad5766_set_span_range,
		       0);

static IIO_DEVICE_ATTR(span_range_available,
		       0444,
		       ad5766_show_available_span_ranges,
		       NULL,
		       0);

static struct attribute *ad5766_attributes[] = {
	&iio_dev_attr_span_range.dev_attr.attr,
	&iio_dev_attr_span_range_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad5766_attribute_group = {
	.attrs = ad5766_attributes,
};

static const struct iio_info ad5766_info = {
	.read_raw = ad5766_read_raw,
	.write_raw = ad5766_write_raw,
	.attrs = &ad5766_attribute_group,
	.read_avail = ad5766_read_avail,
};

static DECLARE_AD576x_CHANNELS(ad5766_channels, 16);
static DECLARE_AD576x_CHANNELS(ad5767_channels, 12);

static const struct ad5766_chip_info ad5766_chip_infos[] = {
	[ID_AD5766] = {
		.num_channels = ARRAY_SIZE(ad5766_channels),
		.channels = ad5766_channels,
	},
	[ID_AD5767] = {
		.num_channels = ARRAY_SIZE(ad5767_channels),
		.channels = ad5767_channels,
	},
};

static int ad5766_write(struct iio_dev *indio_dev, u8 dac, u16 data)
{
	struct ad5766_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = _ad5766_spi_write(st, AD5766_CMD_WR_DAC_REG(dac), data);
	mutex_unlock(&st->lock);

	return ret;
}

static int ad5766_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	const int max_val = (1 << chan->scan_type.realbits);
	// printk("ad5766_write_raw\n");
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val >= max_val || val < 0)
			return -EINVAL;
		val <<= chan->scan_type.shift;
		break;
	case IIO_CHAN_INFO_OFFSET:
		// printk("IIO_CHAN_INFO_OFFSET val: %d, val2: %d\n", val, val2);
		break;
	case IIO_CHAN_INFO_SCALE:
		// printk("IIO_CHAN_INFO_SCALE val: %d, val2: %d\n", val, val2);
		break;
	default:
		return -EINVAL;
	}

	return ad5766_write(indio_dev, chan->address, val);
}

static int _ad5766_spi_read(struct ad5766_state *st, u8 dac, int *val)
{
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &st->data[0].d32,
			.bits_per_word = 8,
			.len = 3,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d32,
			.rx_buf = &st->data[2].d32,
			.bits_per_word = 8,
			.len = 3,
		},
	};

	st->data[0].d32 = AD5766_CMD_READBACK_REG(dac);
	st->data[1].d32 = AD5766_CMD_NOP_MUX_OUT;

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));

	if (ret)
		return ret;

	*val = st->data[2].w16[1];

	return ret;
}

static int ad5766_read(struct iio_dev *indio_dev, u8 dac, int *val)
{
	struct ad5766_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = _ad5766_spi_read(st, dac, val);
	mutex_unlock(&st->lock);

	return ret;
}


static int ad5766_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	struct ad5766_state *st = iio_priv(indio_dev);
	// printk("ad5766_read_avail\n");
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (int *)st->crt_scale_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		/* Values are stored in a 2D matrix  */
		*length = st->crt_scales_avail * 2;

		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_OFFSET:
		*vals = (int *)st->offset_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		/* Values are stored in a 2D matrix  */
		*length = AD5766_VOLTAGE_RANGE_MAX * 2;

		return IIO_AVAIL_LIST;
		
	default:
		return -EINVAL;
	}
}

static int ad5766_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret/*, min, max*/;
	struct ad5766_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ad5766_read(indio_dev, chan->address, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		// min = ad5766_span_params[st->crt_range].min;
		// max = ad5766_span_params[st->crt_range].max;
		// *val = max - min;
		// *val2 = indio_dev->channels->scan_type.realbits;
		
		*val = st->scale_avail[st->crt_range][0];
		*val2 = st->scale_avail[st->crt_range][1];

		printk("Scale read crt_range: %d, val: %d, val2: %d", st->crt_range, *val, *val2);
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		// min = ad5766_span_params[st->crt_range].min;
		// max = ad5766_span_params[st->crt_range].max;
		// *val = ((1 << indio_dev->channels->scan_type.realbits) * min) / (max - min);
		// *val = st->offset_avail[st->crt_range]
		// // printk("realbits: %d, min: %d, max: %d, val: %d\n", indio_dev->channels->scan_type.realbits, min, max, *val);

		// return IIO_VAL_INT;
		*val = st->offset_avail[st->crt_range][0];
		*val2 = st->offset_avail[st->crt_range][1];
		printk("Offset read crt_range: %d, val: %d, val2: %d", st->crt_range, *val, *val2);

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

}

static int ad5766_setup(struct ad5766_state *st)
{
	int ret;

	st->crt_range = AD5766_VOLTAGE_RANGE_M5V_to_5V;
	_ad5766_spi_write(st, AD5766_CMD_SW_FULL_RESET, AD5766_FULL_RESET_CODE);
	_ad5766_spi_write(st, AD5766_CMD_SPAN_REG, AD5766_VOLTAGE_RANGE_M5V_to_5V);

	ret = ad5766_set_offset_avail(st);
	if (ret)
		return ret;

	ret = ad5766_calc_scales(st);
	if (ret)
		return ret;

	ret = ad5766_set_possible_scales(st);
	if (ret)
		return ret;
	
	ret = ad5766_set_scale_avail(st);
	if (ret)
		return ret;

	return 0;
}

static int ad5766_probe(struct spi_device *spi)
{
	enum ad5766_type type = spi_get_device_id(spi)->driver_data;
	struct iio_dev *indio_dev;
	struct ad5766_state *st;
	int ret;

	printk("ad5766_probe33");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	mutex_init(&st->lock);

	st->spi = spi;
	st->chip_info = &ad5766_chip_infos[type];

	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad5766_info;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad5766_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad5766_dt_match[] = {
	{ .compatible = "adi,ad5766" },
	{ .compatible = "adi,ad5767" },
	{},
};
MODULE_DEVICE_TABLE(of, ad5766_dt_match);

static const struct spi_device_id ad5766_spi_ids[] = {
	{ "ad5766", ID_AD5766 },
	{ "ad5767", ID_AD5767 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5766_spi_ids);

static struct spi_driver ad5766_driver = {
	.driver = {
		.name = "ad5766",
		.of_match_table = ad5766_dt_match,
	},
	.probe = ad5766_probe,
	.id_table = ad5766_spi_ids,
};
module_spi_driver(ad5766_driver);

MODULE_AUTHOR("Denis-Gabriel Gheorghescu <denis.gheorghescu@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5766/AD5767 DACs");
MODULE_LICENSE("GPL v2");
