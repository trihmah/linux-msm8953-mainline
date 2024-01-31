/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DRIVER_NAME				"cm36658"

/* Chip registers */
#define CM36658_REG_AL_CFG0			0x00
#define CM36658_REG_AL_TH_HIGH			0x01
#define CM36658_REG_AL_TH_LOW			0x02
#define CM36658_REG_PS_CFG3			0x03
#define CM36658_REG_PS_CFG4			0x04
#define CM36658_REG_PS_TH_LOW			0x05
#define CM36658_REG_PS_TH_HIGH			0x06
#define CM36658_REG_PS_OFFSET			0x07
#define CM36658_REG_PS_AC_L			0x08
#define CM36658_REG_AL_RED			0x10
#define CM36658_REG_AL_GREEN			0x11
#define CM36658_REG_AL_BLUE			0x12
#define CM36658_REG_AL_IR			0x13
#define CM36658_REG_PS_OUT			0x14
#define CM36658_REG_INT_STATUS			0x15
#define CM36658_REG_ID				0x16
#define CM36658_REG_PS_AC_DATA			0x17

/* ALS configuration register fields */
#define CM36658_AL_CFG0_DISABLE_FLAG		BIT(0)
/* Required for both light and proximity sensor to work */
#define CM36658_AL_CFG0_STANDBY_FLAG		BIT(1)
#define CM36658_AL_CFG0_IT_FMASK		GENMASK(3, 2)
/* Reads as zero, doesn't actually change anything */
#define CM36658_AL_CFG0_START_FLAG		BIT(7)
#define CM36658_AL_CFG0_INT_EN_FLAG		BIT(8)
/* This bit increases sensitivity by ~2% */
#define CM36658_AL_CFG0_UNKNOWN9		BIT(9)
//#define CM36658_AL_CFG0_PERS			BIT(10)
//#define CM36658_AL_CFG0_PERS			BIT(11)
//#define CM36658_AL_CFG0_GAIN_OFF_FMASK		BIT(12) /* 2 x Scale */
//#define CM36658_AL_CFG0_GAIN_OFF_FMASK		BIT(13) /* 1/8 x Scale */
#define CM36658_AL_CFG0_UNKNOWN14		BIT(14)

/* PS configuration register fields */
#define CM36658_PS_CFG3_DISABLE_FLAG		BIT(0)
#define CM36658_PS_CFG3_SMART_PERS_FLAG		BIT(1)
/* Drive IRQ pin according to proximity */
#define CM36658_PS_CFG3_INT_DRDY_FLAG		BIT(2)
/* Sets INT_STATUS, needs re-enable to clear */
#define CM36658_PS_CFG3_INT_TRIGGER_FLAG	BIT(3)
#define CM36658_PS_CFG3_PERS_FMASK		GENMASK(5, 4)
#define CM36658_PS_CFG3_PERIOD_FMASK		GENMASK(7, 6)
#define CM36658_PS_CFG3_INT_SEL_FLAG		BIT(8)
#define CM36658_PS_CFG3_START_FLAG		BIT(11)
#define CM36658_PS_CFG3_DUTY_FMASK		GENMASK(13, 12)
#define CM36658_PS_CFG3_IT_FMASK		GENMASK(15, 14)

#define CM36658_PS_CFG4_SUNLIGHT_ENA_FMASK	BIT(0)
#define CM36658_PS_CFG4_START_FMASK		BIT(3)
#define CM36658_PS_CFG4_AF_TRIGGER_FLAG		BIT(5)
#define CM36658_PS_CFG4_AF_ENABLED_FLAG		BIT(6)
#define CM36658_PS_CFG4_IRLED_CUR_FMASK		GENMASK(10, 8)
/* Despite 16-bit output PS_TH_LOW/PS_TH_HIGH remains 12-bit */
#define CM36658_PS_CFG4_OUTPUT_16BIT		BIT(12)
#define CM36658_PS_CFG4_SUNLIGHT_LVL_FMASK	GENMASK(14, 13)
#define CM36658_PS_CFG4_UNKNOWN15

/* Interrupt status flags */
#define CM36658_INT_STATUS_PS_LOW_FLAG		BIT(8)
#define CM36658_INT_STATUS_PS_HIGH_FLAG		BIT(9)
#define CM36658_INT_STATUS_AL_HIGH_FLAG		BIT(10)
#define CM36658_INT_STATUS_AL_LOW_FLAG		BIT(11)
#define CM36658_INT_STATUS_PS_AC_DONE_FLAG	BIT(12)
#define CM36658_INT_STATUS_PS_SP_MODE_FLAG	BIT(13)

/* Device ID */
#define CM36658_ID_DEV_ID_FMASK			GENMASK(7, 0)
#define CM36658_ID_DEV_ID_VAL			0x58

struct cm36658_sensor_info {
	enum iio_chan_type type;
	uint16_t config_reg;
	uint16_t config_it_mask, config_int_mask, config_disable_mask;
	uint16_t status_low_mask, status_high_mask;
	uint16_t threshold_reg[2];
	uint16_t threshold_max;
	unsigned int int_time_base_us;
};

struct cm36658_chip;
struct cm36658_sensor {
	const struct cm36658_sensor_info *info;
	struct cm36658_chip *chip;
	struct delayed_work off_work;
	struct mutex lock;

	bool enabled : 1;
	bool int_enabled : 1;

	/* Cache some registers to allow read without power */
	unsigned int config;
	unsigned int threshold[2];
};

enum {
	CM36658_SENSOR_ALS,
	CM36658_SENSOR_PS,
};

struct cm36658_chip {
	struct cm36658_sensor sensor[2];
	struct device *dev;
	struct i2c_client *client;
	struct iio_chan_spec chan[5];
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct regulator *vdd, *vled;
	atomic_t int_ena_count;
};

static int cm36658_sensor_get_int_time_us(struct cm36658_sensor *sensor, int *val)
{
	int field, bit;

	bit = ffs(sensor->info->config_it_mask) - 1;
	field = (sensor->config & sensor->info->config_it_mask) >> bit;

	/* Every register step changes integration time by factor of 2 */
	*val = sensor->info->int_time_base_us * (1 << field);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int cm36658_sensor_set_int_time_us(struct cm36658_sensor *sensor,
					  int val, int val2)
{
	int ret, bit, it_shift, regval;

	if (val || !val2 || val2 % sensor->info->int_time_base_us)
		return -EINVAL;

	val2 /= sensor->info->int_time_base_us;
	bit = ffs(val2) - 1;
	it_shift = ffs(sensor->info->config_it_mask) - 1;

	/* Only one bit should be set */
	if ((val2 ^ BIT(bit)) || ((bit << it_shift) & ~sensor->info->config_it_mask))
		return -EINVAL;

	mutex_lock(&sensor->lock);
	pm_runtime_get_sync(sensor->chip->dev);

	regval = sensor->config & ~sensor->info->config_it_mask;
	regval |= bit << it_shift;

	ret = regmap_update_bits(sensor->chip->regmap,
				 sensor->info->config_reg,
				 sensor->info->config_it_mask,
				 regval);
	if (!ret)
		sensor->config = regval;

	pm_runtime_put(sensor->chip->dev);
	mutex_unlock(&sensor->lock);

	return ret;
}

static int cm36658_sensor_set_thresh(struct cm36658_sensor *sensor,
				     unsigned int value,
				     bool high)
{
	//unsigned int *other_val;
	int ret = 0;

	if (value > sensor->info->threshold_max)
		return -ERANGE;

	mutex_lock(&sensor->lock);
	pm_runtime_get_sync(sensor->chip->dev);

#if 0
	/* Configuration with TH_HIGH < TH_LOW will cause countless IRQs when
	 * sensor output is in that range, which isn't wanted behaviour.
	 * Instead of failing we set other threshold to same value in hope
	 * that userspace will update other threshold after */
	other_val = &sensor->threshold[!high];
	if ((high && value < *other_val) ||
	    (!high && value > *other_val)) {
		ret = regmap_write(sensor->chip->regmap,
				   sensor->info->threshold_reg[!high],
				   value);
		if (!ret)
			*other_val = ret;
	}
#endif

	if (!ret) {
		ret = regmap_write(sensor->chip->regmap,
				   sensor->info->threshold_reg[high],
				   value);
		sensor->threshold[high] = value;
	}

	pm_runtime_put(sensor->chip->dev);
	mutex_unlock(&sensor->lock);

	return ret;
}

/* Enable sensor before read or event enable and schedule off work */
static int cm36658_sensor_enable(struct cm36658_sensor *sensor,
				 bool int_enable)
{
	int delay, ret = 0;

	mutex_lock(&sensor->lock);

	cancel_delayed_work(&sensor->off_work);

	if (!sensor->enabled) {
		pm_runtime_get_sync(sensor->chip->dev);

		if (sensor->info->type == IIO_PROXIMITY) {
			ret = regulator_enable(sensor->chip->vled);
			if (ret)
				goto err_put_pm;
		}
		
		ret = regmap_clear_bits(sensor->chip->regmap,
					sensor->info->config_reg,
					sensor->info->config_disable_mask);
		if (ret)
			goto err_disable;

		cm36658_sensor_get_int_time_us(sensor, &delay);
		msleep(delay / USEC_PER_MSEC);

		sensor->enabled = true;
	}

	if (!sensor->int_enabled && int_enable) {
		if (atomic_fetch_inc(&sensor->chip->int_ena_count) == 0)
			enable_irq(sensor->chip->client->irq);

		ret = regmap_set_bits(sensor->chip->regmap,
				      sensor->info->config_reg,
				      sensor->info->config_int_mask);
		if (!ret)
			sensor->int_enabled = true;
		else if (atomic_fetch_dec(&sensor->chip->int_ena_count) == 1)
			disable_irq(sensor->chip->client->irq);
	}

	mutex_unlock(&sensor->lock);

	return ret;

err_disable:
	if (sensor->info->type == IIO_PROXIMITY)
		regulator_disable(sensor->chip->vled);

err_put_pm:
	pm_runtime_put(sensor->chip->dev);

	mutex_unlock(&sensor->lock);

	return ret;
}


static void cm36658_sensor_disable(struct cm36658_sensor *sensor,
				   bool int_disable, bool disable_now)
{
	mutex_lock(&sensor->lock);

	if (sensor->int_enabled && int_disable) {
		if (atomic_fetch_dec(&sensor->chip->int_ena_count) == 1)
			disable_irq(sensor->chip->client->irq);

		regmap_clear_bits(sensor->chip->regmap,
				  sensor->info->config_reg,
				  sensor->info->config_int_mask);

		sensor->int_enabled = false;
	}

	if (sensor->enabled && !sensor->int_enabled) {
		if (disable_now) {
			regmap_set_bits(sensor->chip->regmap,
					sensor->info->config_reg,
					sensor->info->config_disable_mask);

			if (sensor->info->type == IIO_PROXIMITY)
				regulator_disable(sensor->chip->vled);

			sensor->enabled = false;

			pm_runtime_put(sensor->chip->dev);
		} else {
			schedule_delayed_work(&sensor->off_work, 5 * HZ);
		}
	}

	mutex_unlock(&sensor->lock);
}

/* Automatically disable sensor after inactivity period */
static void cm36658_sensor_off_work(struct work_struct *work)
{
	struct cm36658_sensor *sensor =
		container_of(work, struct cm36658_sensor, off_work.work);

	cm36658_sensor_disable(sensor, false, true);
}

static bool cm36658_sensor_handle_irq(struct cm36658_sensor *sensor,
					unsigned int status)
{
	struct cm36658_chip *chip = sensor->chip;
	enum iio_event_direction dir;
	u64 code;

	if (!sensor->int_enabled)
		return true;

	if (status & sensor->info->status_high_mask)
		dir = IIO_EV_DIR_RISING;
	else if (status & sensor->info->status_low_mask)
		dir = IIO_EV_DIR_FALLING;
	else
		return false; /* Must be from another sensor */

	regmap_clear_bits(chip->regmap, 
			  sensor->info->config_reg,
			  sensor->info->config_int_mask);

	regmap_set_bits(chip->regmap, 
			sensor->info->config_reg,
			sensor->info->config_int_mask);

	code = IIO_UNMOD_EVENT_CODE(sensor->info->type, 0, IIO_EV_TYPE_THRESH, dir);
	iio_push_event(chip->indio_dev, code, iio_get_time_ns(chip->indio_dev));

	return true;
}

static ktime_t last_irq;
static ktime_t irq_diff;

static irqreturn_t cm36658_irq_handler_no_thread(int irq, void *data)
{
	ktime_t this = ktime_get();

	irq_diff = this - last_irq;
	last_irq = this;
	return IRQ_WAKE_THREAD;
}


static irqreturn_t cm36658_irq_handler(int irq, void *data)
{
	struct cm36658_chip *chip = data;
	unsigned int status;
	bool handled;
	int ret;

	ret = regmap_read(chip->regmap, CM36658_REG_INT_STATUS, &status);
	if (ret < 0) {
		dev_err_ratelimited(chip->dev, "failed to read status: %d\n", ret);
		return IRQ_HANDLED;
	}

	handled = cm36658_sensor_handle_irq(&chip->sensor[CM36658_SENSOR_ALS], status);
	handled |= cm36658_sensor_handle_irq(&chip->sensor[CM36658_SENSOR_PS], status);

	if (!handled)
		dev_err_ratelimited(chip->dev, "unexpected IRQ: status=%#06X\n", status);

	dev_info(chip->dev, "%s %llu\n", __func__, ktime_to_ns(irq_diff));


	return IRQ_HANDLED;
}

static struct cm36658_sensor* cm36658_get_sensor(struct iio_dev *indio_dev,
						 const struct iio_chan_spec *chan)
{
	struct cm36658_chip *chip = iio_priv(indio_dev);

	switch (chan->type) {
	case IIO_LIGHT:
		return &chip->sensor[CM36658_SENSOR_ALS];
	case IIO_PROXIMITY:
		return &chip->sensor[CM36658_SENSOR_PS];
	default:
		return NULL;
	}
}

static int cm36658_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct cm36658_sensor *sensor = cm36658_get_sensor(indio_dev, chan);
	int ret = -EINVAL;

	if (!sensor)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = cm36658_sensor_enable(sensor, false);
		ret = ret ?: regmap_read(sensor->chip->regmap, chan->address, val);
		ret = ret ?: IIO_VAL_INT;
		cm36658_sensor_disable(sensor, false, false);
		break;
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		ret = cm36658_sensor_get_int_time_us(sensor, val2);
		break;
	case IIO_CHAN_INFO_SCALE:
		break;
	default:
		break;
	}

	return ret;
}

static int cm36658_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct cm36658_sensor *sensor = cm36658_get_sensor(indio_dev, chan);
	if (!sensor)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		return cm36658_sensor_set_int_time_us(sensor, val, val2);
	default:
		break;
	}

	return -EINVAL;
}

static int cm36658_read_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct cm36658_sensor *sensor = cm36658_get_sensor(indio_dev, chan);
	if (!sensor)
		return -EINVAL;

	switch (dir) {
	case IIO_EV_DIR_RISING:
	case IIO_EV_DIR_FALLING:
		*val = sensor->threshold[dir == IIO_EV_DIR_RISING];
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int cm36658_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct cm36658_sensor *sensor = cm36658_get_sensor(indio_dev, chan);
	if (!sensor)
		return -EINVAL;

	switch (dir) {
	case IIO_EV_DIR_RISING:
	case IIO_EV_DIR_FALLING:
		return cm36658_sensor_set_thresh(sensor, val,
						 dir == IIO_EV_DIR_RISING);
	default:
		break;
	}

	return -EINVAL;
}

static int cm36658_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct cm36658_sensor *sensor = cm36658_get_sensor(indio_dev, chan);

	return sensor->int_enabled;
}


static int cm36658_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      bool state)
{
	struct cm36658_sensor *sensor = cm36658_get_sensor(indio_dev, chan);
	int ret = 0;

	if (!sensor->chip->client->irq)
		return -EINVAL;

	if (state)
		ret = cm36658_sensor_enable(sensor, true);

	if (!state || ret)
		cm36658_sensor_disable(sensor, true, false);

	return ret;
}

#define CM36658_EVENTSPEC(_dir, _mask_bit) {	\
	.type = IIO_EV_TYPE_THRESH,		\
	.dir = _dir,				\
	.mask_separate = BIT(_mask_bit),	\
}

static const struct iio_event_spec cm36658_event_spec[] = {
	CM36658_EVENTSPEC(IIO_EV_DIR_EITHER, IIO_EV_INFO_ENABLE),
	CM36658_EVENTSPEC(IIO_EV_DIR_RISING, IIO_EV_INFO_VALUE),
	CM36658_EVENTSPEC(IIO_EV_DIR_FALLING, IIO_EV_INFO_VALUE),
};

static const struct iio_info cm36658_info = {
	.read_event_config	= &cm36658_read_event_config,
	.read_event_value	= &cm36658_read_event_value,
	.read_raw		= &cm36658_read_raw,
	.write_event_config	= &cm36658_write_event_config,
	.write_event_value	= &cm36658_write_event_value,
	.write_raw		= &cm36658_write_raw,
};

static const struct cm36658_sensor_info cm36658_sensor_info[] = {
	[CM36658_SENSOR_ALS] = {
		.type = IIO_LIGHT,
		.config_reg = CM36658_REG_AL_CFG0,
		.config_it_mask = CM36658_AL_CFG0_IT_FMASK,
		.config_int_mask = CM36658_AL_CFG0_INT_EN_FLAG,
		.config_disable_mask = CM36658_AL_CFG0_DISABLE_FLAG,
		.status_low_mask = CM36658_INT_STATUS_AL_LOW_FLAG,
		.status_high_mask = CM36658_INT_STATUS_AL_HIGH_FLAG,
		.threshold_reg = { CM36658_REG_AL_TH_LOW, CM36658_REG_AL_TH_HIGH },
		.threshold_max = GENMASK(15, 0),
		.int_time_base_us = 50 * USEC_PER_MSEC,
	},
	[CM36658_SENSOR_PS] = {
		.type = IIO_PROXIMITY,
		.config_reg = CM36658_REG_PS_CFG3,
		.config_it_mask = CM36658_PS_CFG3_IT_FMASK,
		.config_int_mask = CM36658_PS_CFG3_INT_TRIGGER_FLAG,
		.config_disable_mask = CM36658_PS_CFG3_DISABLE_FLAG,
		.status_low_mask = CM36658_INT_STATUS_PS_LOW_FLAG,
		.status_high_mask = CM36658_INT_STATUS_PS_HIGH_FLAG,
		.threshold_reg = { CM36658_REG_PS_TH_LOW, CM36658_REG_PS_TH_HIGH },
		.threshold_max = GENMASK(11, 0),
		.int_time_base_us = 20 * USEC_PER_MSEC,
	},
};

static const struct cm36658_sensor_info cm36658_ps_info = {
};

static bool cm36658_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg >= CM36658_REG_AL_RED;
}

static const struct regmap_config cm36658_regmap_config = {
	.max_register	= CM36658_REG_PS_AC_DATA,
	.reg_bits	= 8,
	.val_bits	= 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.volatile_reg	= cm36658_is_volatile_reg,
};

static void cm36658_chan_init(struct iio_chan_spec *spec, bool have_events,
			      enum iio_modifier mod, int reg)
{
	spec->address = reg;
	spec->type = mod ? IIO_LIGHT : IIO_PROXIMITY;
	spec->channel2 = mod;
	spec->modified = !!mod;
	spec->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	spec->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_INT_TIME);

	if (!have_events || (mod && mod != IIO_MOD_LIGHT_GREEN))
		return;

	spec->event_spec = cm36658_event_spec;
	spec->num_event_specs = ARRAY_SIZE(cm36658_event_spec);
}

static int cm36658_sensor_init(struct cm36658_sensor *sensor)
{
	struct reg_sequence init_seq[] = {
		{ sensor->info->config_reg, sensor->config },
		{ sensor->info->threshold_reg[1], sensor->threshold[1] },
		{ sensor->info->threshold_reg[0], sensor->threshold[0] },
		{ CM36658_REG_PS_CFG4, FIELD_PREP(CM36658_PS_CFG4_IRLED_CUR_FMASK, 0) },
		{ CM36658_REG_PS_OFFSET, 0 },
	};

	return regmap_multi_reg_write(sensor->chip->regmap, init_seq,
				      sensor->info->type == IIO_LIGHT ? 3 : 5);
}

static void cm36658_sensor_setup(struct cm36658_chip *chip,
				 struct cm36658_sensor *sensor,
				 const struct cm36658_sensor_info *info)
{
	sensor->config = info->config_disable_mask;
	sensor->threshold[0] = info->threshold_max / 16;
	sensor->threshold[1] = info->threshold_max - info->threshold_max / 16;

	if (info->type == IIO_LIGHT)
		sensor->config |= CM36658_AL_CFG0_STANDBY_FLAG;
	else
		sensor->config |= //CM36658_PS_CFG3_INT_SEL_FLAG |
			FIELD_PREP(CM36658_PS_CFG3_IT_FMASK, 3) |
			FIELD_PREP(CM36658_PS_CFG3_PERS_FMASK, 3);

	INIT_DELAYED_WORK(&sensor->off_work, cm36658_sensor_off_work);
	mutex_init(&sensor->lock);

	sensor->chip = chip;
	sensor->info = info;
}

static void cm36658_disable_regulator(void *data)
{
	struct cm36658_chip *chip = data;

	dev_info(chip->dev, "%s\n", __func__);

	regulator_disable(chip->vdd);
}

static int cm36658_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_chan_spec *chan;
	struct iio_dev *indio_dev;
	struct cm36658_chip *chip;
	unsigned int dev_id;
	bool have_irq;
	int ret, i;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->client = client;
	chip->dev = dev;
	chip->indio_dev = indio_dev;
	atomic_set(&chip->int_ena_count, 0);

	i2c_set_clientdata(client, chip);

	chip->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(chip->vdd))
		return dev_err_probe(dev, PTR_ERR(chip->vdd),
				"failed to get vdd regulator\n");

	chip->vled = devm_regulator_get_optional(dev, "vled");
	if (IS_ERR(chip->vdd))
		return dev_err_probe(dev, PTR_ERR(chip->vled),
				"failed to get vled regulator\n");

	ret = regulator_enable(chip->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulator\n");

	ret = devm_add_action_or_reset(&client->dev,
			cm36658_disable_regulator, chip);
	if (ret) {
		regulator_disable(chip->vdd);
		return dev_err_probe(dev, ret, "failed to add cleanup action\n");
	}

	chip->regmap = devm_regmap_init_i2c(client, &cm36658_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				"failed to init regmap");
	
	for (i = 0; i < 50; i ++) {
		ret = regmap_read(chip->regmap, CM36658_REG_ID, &dev_id);
		if (!ret) {
			dev_info(dev, "id ok: %d %x\n", i, dev_id);
			break;
		}
	}

	if (ret)
		return dev_err_probe(dev, ret, "failed to read dev_id\n"); 

	ret = regmap_read(chip->regmap, CM36658_REG_ID, &dev_id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read dev_id\n");

	dev_id = FIELD_GET(CM36658_ID_DEV_ID_FMASK, dev_id);
	if (dev_id != CM36658_ID_DEV_ID_VAL)
		return dev_err_probe(dev, -ENODEV, "unexpected device id: %02x\n", dev_id);

	chan = chip->chan;

	for (i = 0; i < ARRAY_SIZE(chip->sensor); i ++) {
		cm36658_sensor_setup(chip, &chip->sensor[i], &cm36658_sensor_info[i]);

		ret = cm36658_sensor_init(&chip->sensor[i]);
		if (ret)
			return dev_err_probe(dev, ret, "failed to init sensor registers");
	}

	have_irq = !!client->irq;

	cm36658_chan_init(chan++, have_irq, IIO_NO_MOD, CM36658_REG_PS_OUT);
	cm36658_chan_init(chan++, have_irq, IIO_MOD_LIGHT_RED, CM36658_REG_AL_RED);
	cm36658_chan_init(chan++, have_irq, IIO_MOD_LIGHT_GREEN, CM36658_REG_AL_GREEN);
	cm36658_chan_init(chan++, have_irq, IIO_MOD_LIGHT_BLUE, CM36658_REG_AL_BLUE);
	cm36658_chan_init(chan++, have_irq, IIO_MOD_LIGHT_IR, CM36658_REG_AL_IR);

	indio_dev->name = DRIVER_NAME;
	indio_dev->info = &cm36658_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->chan;
	indio_dev->num_channels = ARRAY_SIZE(chip->chan);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register iio device\n");

	if (have_irq) {
		ret = devm_request_threaded_irq(dev, client->irq, 
						cm36658_irq_handler_no_thread,
						cm36658_irq_handler,
						IRQF_ONESHOT, DRIVER_NAME, chip);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");

		disable_irq(client->irq);
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}

static void cm36658_remove(struct i2c_client *client)
{
	struct cm36658_chip *chip = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < 2; i ++) {
		cancel_delayed_work(&chip->sensor[i].off_work);
		cm36658_sensor_disable(&chip->sensor[i], true, true);
	}

	pm_runtime_get_sync(chip->dev);
	pm_runtime_put_noidle(chip->dev);
	pm_runtime_disable(chip->dev);

	enable_irq(chip->client->irq);
}

static int __maybe_unused cm36658_runtime_resume(struct device *dev)
{

	struct cm36658_chip *chip = dev_get_drvdata(dev);
	int ret, i;
	unsigned dev_id;

	ret = regulator_enable(chip->vdd);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "%s\n", __func__);

	for (i = 0; i < 50; i ++) {
		ret = regmap_read(chip->regmap, CM36658_REG_ID, &dev_id);
		if (!ret) {
			dev_info(dev, "id ok: %i %x\n", i, dev_id);
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(chip->sensor); i ++) {
		ret = cm36658_sensor_init(&chip->sensor[i]);
		if (ret)
			return ret;
	}

	return regmap_set_bits(chip->regmap, CM36658_REG_AL_CFG0,
			       CM36658_AL_CFG0_STANDBY_FLAG);
}

static int __maybe_unused cm36658_runtime_suspend(struct device *dev)
{
	struct cm36658_chip *chip = dev_get_drvdata(dev);

	dev_info(chip->dev, "%s\n", __func__);

	regmap_clear_bits(chip->regmap, CM36658_REG_AL_CFG0,
			  CM36658_AL_CFG0_STANDBY_FLAG);

	cm36658_disable_regulator(chip);

	return 0;
}

static const struct dev_pm_ops __maybe_unused cm36658_pm_ops = {
	SET_RUNTIME_PM_OPS(cm36658_runtime_suspend,
			   cm36658_runtime_resume, NULL)
};

static const struct of_device_id cm36658_of_match[] = {
	{ .compatible = "capella,cm36658" },
	{ }
};
MODULE_DEVICE_TABLE(of, cm36658_of_match);

static struct i2c_driver cm36658_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.pm		= &cm36658_pm_ops,
		.of_match_table	= cm36658_of_match,
	},
	.probe		= cm36658_probe,
	.remove		= cm36658_remove,
};

module_i2c_driver(cm36658_driver);

MODULE_AUTHOR("Vladimir Lypak <vladimir.lypak@gmail.com>");
MODULE_DESCRIPTION("CM36658 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL v2");
