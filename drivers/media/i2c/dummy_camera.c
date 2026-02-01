#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <linux/delay.h>

static const char * const dummy_camera_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
	"afvdd",
};

#define DUMMY_CAMERA_SUPPLY_NUM ARRAY_SIZE(dummy_camera_supply_names)

struct dummy_camera {
	struct clk *mclk;
	struct regulator_bulk_data supplies[DUMMY_CAMERA_SUPPLY_NUM];
	struct gpio_desc *reset_gpio;
};

static int dummy_camera_probe(struct platform_device *pdev)
{
	int ret;
	struct device* dev = &pdev->dev;
	struct dummy_camera* dummy_camera;

	dummy_camera = devm_kzalloc(dev, sizeof(*dummy_camera), GFP_KERNEL);
	if (!dummy_camera)
		return -ENOMEM;

	for (int i=0; i < DUMMY_CAMERA_SUPPLY_NUM; i++)
		dummy_camera->supplies[i].supply = dummy_camera_supply_names[i];

	ret = devm_regulator_bulk_get(dev, DUMMY_CAMERA_SUPPLY_NUM, dummy_camera->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	dummy_camera->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(dummy_camera->mclk))
		return dev_err_probe(dev, PTR_ERR(dummy_camera->mclk), "failed to get mclk\n");

	dummy_camera->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(dummy_camera->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(dummy_camera->reset_gpio), "failed to get gpio\n");

	// Power on the device
	ret = regulator_bulk_enable(DUMMY_CAMERA_SUPPLY_NUM, dummy_camera->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");

	gpiod_set_value_cansleep(dummy_camera->reset_gpio, 1);

	ret = clk_prepare_enable(dummy_camera->mclk);
	if (ret < 0) {
		regulator_bulk_disable(DUMMY_CAMERA_SUPPLY_NUM, dummy_camera->supplies);
		return dev_err_probe(dev, ret, "failed to enable MCLK\n");
	}

	usleep_range(2000, 2200);
	gpiod_set_value_cansleep(dummy_camera->reset_gpio, 0);
	usleep_range(1500, 1600);

	return 0;
}

static void dummy_camera_remove(struct platform_device *pdev) {
	struct dummy_camera *dummy_camera = platform_get_drvdata(pdev);

	clk_disable_unprepare(dummy_camera->mclk);
	gpiod_set_value_cansleep(dummy_camera->reset_gpio, 1);
	regulator_bulk_disable(DUMMY_CAMERA_SUPPLY_NUM, dummy_camera->supplies);
}

static const struct of_device_id dummy_camera_match_table[] = {
	{ .compatible = "dummy,camera" },
	{ }
};
MODULE_DEVICE_TABLE(of, dummy_camera_match_table);

static struct platform_driver dummy_camera_driver = {
	.probe = dummy_camera_probe,
	.remove = dummy_camera_remove,
	.driver = {
		.name = "dummy-camera",
		.of_match_table = dummy_camera_match_table,
	},
};
module_platform_driver(dummy_camera_driver);

MODULE_DESCRIPTION("Dummy camera driver");
MODULE_LICENSE("GPL v2");

