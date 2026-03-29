#define DEBUG 1
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/cleanup.h>
#include <linux/mutex.h>
#include <linux/kernel_read_file.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>

#include <media/v4l2-cci.h>

#define MAX_CAMERA_SLOTS 4

static char overlay_path[256] = "/boot/dtbs";
module_param_string(overlay_path, overlay_path, sizeof(overlay_path), 0440);

struct camera_info {
	const char *filename;

	int overlay_id;
	bool applied;
	bool untested;

	void *overlay_fdt;
	size_t overlay_fdt_size;
};

static DEFINE_MUTEX(mgr_mutex);
static struct camera_info camera_slots[MAX_CAMERA_SLOTS] = { };
static struct device *mgr_dev = NULL;
static bool ocs_applied = false;
static struct of_changeset mgr_ocs;
static bool plat_subsys_enabled = false;

/* Since we put regulators in subnodes of "dev" anything using regulator_get
 * instead of of_regulator_get (like of_regulator_bulk_get_all) might get
 * wrong regulator from wrong camera subnode.
 */
static int camera_get_all_regulators(struct device *dev,
		struct device_node *np, int *count,
		struct regulator_bulk_data *vregs)
{
	const char *suffix = "-supply";
	struct regulator *vreg;
	struct property *p;
	int max_cnt = *count;
	int slen = strlen(suffix);
	char name[256];
	int cnt = 0;

	*count = 0;

	for_each_property_of_node(np, p) {
		int len = p->name ? strlen(p->name) : 0;
		if (len <= slen ||
		    strncmp(suffix, p->name + len - slen, slen))
			continue;

		if (cnt >= max_cnt || (len - slen + 1) > ARRAY_SIZE(name)) {
			regulator_bulk_free(cnt, vregs);
			return -ERANGE;
		}

		strncpy(name, p->name, len - slen);
		name[len - slen] = 0;

		vreg = of_regulator_get(dev, np, name);
		if (IS_ERR(vreg)) {
			regulator_bulk_free(cnt, vregs);
			return PTR_ERR(vreg);
		}

		vregs[cnt].supply = "(doesn't matter)";
		vregs[cnt++].consumer = vreg;
	}

	*count = cnt;
	return 0;
}

static void camera_manager_enable_platform_subsys(struct device *dev)
{
	const char *psubsys_name = "platform-camera-system";
	struct device_node *np;
	const __be32 *psubsys;
	int psubsys_len, ret;

	if (plat_subsys_enabled)
		return;

	psubsys = of_get_property(dev->of_node, psubsys_name, &psubsys_len);
	if (!psubsys)
		return;

	if (psubsys_len != 4) {
		dev_err(dev, "Invalid phandle in '%s'\n", psubsys_name);
		return;
	}

	np = of_find_node_by_phandle(be32_to_cpup(psubsys));
	if (!np) {
		dev_err(dev, "Invalid phandle in '%s'\n", psubsys_name);
		return;
	}

	of_changeset_init(&mgr_ocs);
	ret = of_changeset_update_prop_string(&mgr_ocs, np, "status", "okay");
	ret = ret ?: of_changeset_apply(&mgr_ocs);
	of_node_put(np);
	if (ret) {
		dev_err(dev, "failed to update status property\n");
		of_changeset_destroy(&mgr_ocs);
	} else {
		plat_subsys_enabled = true;
	}

	dev_info(dev, "enabled camera subsystem\n");
}

static int camera_manager_load_overlays(struct device *dev)
{
	int ret, cnt = 0, i;

	for (i = 0; i < MAX_CAMERA_SLOTS; i++) {
		struct camera_info* cam = &camera_slots[i];
		cnt += !!(cam->filename && cam->overlay_fdt);
		if (!cam->filename || cam->overlay_fdt)
			continue;

		char *path __free(kfree) = kasprintf(GFP_KERNEL,
				"%s/%s", overlay_path, cam->filename);
		if (!path)
			return -ENOMEM;

		ret = kernel_read_file_from_path_initns(path, 0,
				&cam->overlay_fdt, INT_MAX,
				&cam->overlay_fdt_size, READING_FIRMWARE);
		if (ret < 0) {
			if (ret != -ENOENT)
				dev_err(dev, "Failed to load '%s': %d\n", path, ret);
			continue;
		}

		dev_info(dev, "Loaded '%s' size=%lu\n", cam->filename,
				cam->overlay_fdt_size);

		cnt ++;
	}

	if (!cnt)
		return -EPROBE_DEFER;

	if (ocs_applied)
		return 0;

	cnt = 0;

	for (i = 0; i < MAX_CAMERA_SLOTS; i++) {
		struct camera_info* cam = &camera_slots[i];

		cnt += !!cam->applied;

		if (cam->applied || !cam->overlay_fdt)
			continue;

		ret = of_overlay_fdt_apply(cam->overlay_fdt,
				cam->overlay_fdt_size, &cam->overlay_id, NULL);
		if (ret)
			dev_err(dev, "Failed to apply overlay: %d\n", ret);

		dev_info(dev, "Applied '%s' for slot %d\n", cam->filename, i);
		cam->applied = true;
		cnt ++;
	}

	if (!cnt)
		return -ENODEV;

	camera_manager_enable_platform_subsys(dev);

	return 0;
}

static int camera_find_i2c_adapter(struct device_node *cam_node,
				   struct i2c_adapter **adapter)
{
	struct of_phandle_args pargs = { 0 };
	int ret;

	ret = of_parse_phandle_with_fixed_args(cam_node, "i2c-bus", 0, 0, &pargs);
	if (ret || !pargs.np)
		return ret ?: -ENODATA;

	*adapter = of_get_i2c_adapter_by_node(pargs.np);
	of_node_put(pargs.np);
	if (IS_ERR_OR_NULL(*adapter))
		return PTR_ERR(*adapter) ?: -EPROBE_DEFER;

	return 0;
}

static int camera_manager_check_register(struct device *dev,
					 struct device_node *node,
					 struct regmap *rmap)
{
	u32 reg_info[3];
	u64 reg_val;
	u32 reg;
	int ret;

	ret = of_property_read_u32_array(node, "test-register", reg_info, 3);
	if (ret)
		return ret;

	switch (reg_info[0]) {
	case 8:
		reg = CCI_REG8(reg_info[1]);
		break;
	case 16:
		reg = CCI_REG16(reg_info[1]);
		break;
	case 24:
		reg = CCI_REG24(reg_info[1]);
		break;
	case 32:
		reg = CCI_REG32(reg_info[1]);
		break;
	default:
		return -EINVAL;
	}

	if (rmap) {
		ret = cci_read(rmap, reg, &reg_val, NULL);
		if (ret) {
			dev_info(dev, "read failed: %d", ret);
			return ret;
		}

		if (reg_val != reg_info[2]) {
			dev_info(dev,
				"register (%#x) value is %#x, expected %#x\n",
				reg_info[1], (u32) reg_val, reg_info[2]);
			return -ENODEV;
		}
	}

	return 0;
}

static int camera_manager_probe_cci(struct device *dev,
				    struct camera_info *cam,
				    struct i2c_adapter *adapter,
				    struct device_node *node)
{
	struct i2c_client *client;
	struct regmap *rmap;
	u32 dev_addr;
	int ret;

	if (of_property_read_u32(node, "i2c-addr", &dev_addr))
		return -EINVAL;

	client = i2c_new_dummy_device(adapter, dev_addr);
	if (IS_ERR(client))
		return PTR_ERR(client);

	rmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR_OR_NULL(rmap))
		return PTR_ERR(rmap) ?: -ENOMEM;

	ret = camera_manager_check_register(dev, node, rmap);
	i2c_unregister_device(client);
	return ret;
}

/* of_regulators_get_all We can't put regulators in camera specific nodes (for now)
 * because of_regulator_bulk_get_all uses regulator_get instead
 * of of_regulator_get and would get regulator from parent node
 * (and its children) using names from passed device_node in
 * parameter 2.
 */
static int camera_manager_probe_node(struct device *dev,
			struct device_node *node)
{
	struct camera_info *cam;
	struct i2c_adapter *adapter;
	struct gpio_desc *rst;
	struct clk *inp_clk;
	struct regulator_bulk_data cam_vregs[8] = { 0 };
	int num_vregs = ARRAY_SIZE(cam_vregs);
	u32 slot = 0, freq = 24000000;
	const char *overlay = NULL;
	int ret = 0;
	bool untested;

	untested = of_property_read_bool(node, "untested");
	of_property_read_u32(node, "slot", &slot);
	of_property_read_string(node, "overlay", &overlay);

	ret = camera_manager_check_register(dev, node, NULL);
	if (ret)
		return ret;

	if (!of_property_present(node, "i2c-addr") ||
	    slot >= MAX_CAMERA_SLOTS || !overlay)
		return -EINVAL;

	cam = &camera_slots[slot];
	if (cam->filename)
		return 0;

	ret = camera_find_i2c_adapter(node, &adapter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to find i2c adapter\n");

	ret = camera_get_all_regulators(dev, node, &num_vregs, cam_vregs);
	if (ret) {
		dev_err_probe(dev, ret, "failed to get regulators\n");
		goto put_adapter;
	}

	/* Get reset GPIO */
	rst = fwnode_gpiod_get_index(of_fwnode_handle(node),
			"reset", 0, GPIOD_ASIS, "camera-reset");
	if (IS_ERR(rst)) {
		ret = dev_err_probe(dev, PTR_ERR(rst),
				"failed to get reset gpio\n");
		goto put_regulators;
	}

	/* Get input clk */
	inp_clk = of_clk_get(node, 0);
	if (IS_ERR(inp_clk)) {
		ret = dev_err_probe(dev, PTR_ERR(inp_clk),
				"failed to get input clock\n");
		goto put_gpio;
	}

	/* Assert reset */
	gpiod_set_value_cansleep(rst, 1);

	ret = regulator_bulk_enable(num_vregs, cam_vregs);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enabled regulators\n");
		goto put_clock;
	}

	msleep(100);

	of_property_read_u32(node, "clock-frequency", &freq);

	ret = clk_set_rate(inp_clk, freq);
	if (ret) {
		dev_err_probe(dev, ret, "failed to set clock rate\n");
		goto regulators_disable;
	}

	if (abs_diff(clk_get_rate(inp_clk), (unsigned long) freq) > (freq >> 8)) {
		ret = dev_err_probe(dev, -EINVAL,
				"failed to set requested clock rate\n");
		goto regulators_disable;
	}

	ret = clk_prepare_enable(inp_clk);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable clock\n");
		goto regulators_disable;
	}

	msleep(20);

	gpiod_set_value_cansleep(rst, 0);

	msleep(20);

	dev_info(dev, "probing camera %s for slot %d\n",
			node->name, slot);

	ret = camera_manager_probe_cci(dev, cam, adapter, node);
	if (ret == 0) {
		cam->filename = overlay;
		cam->untested = untested;
	}

	ret = 0;

	gpiod_set_value_cansleep(rst, 1);

	clk_disable_unprepare(inp_clk);

regulators_disable:
	regulator_bulk_disable(num_vregs, cam_vregs);

put_clock:
	clk_put(inp_clk);

put_gpio:
	gpiod_put(rst);

put_regulators:
	regulator_bulk_free(num_vregs, cam_vregs);

put_adapter:
	i2c_put_adapter(adapter);

	return ret;
}

static int camera_manager_probe(struct platform_device *pdev)
{
	guard(mutex)(&mgr_mutex);
	struct device *dev = &pdev->dev;
	struct device_node *np;
	int ret = 0;


	/* Only support one instance */
	if (mgr_dev)
		return -EEXIST;

	for_each_child_of_node(dev->of_node, np)
		ret = ret ?: camera_manager_probe_node(dev, np);

	ret = camera_manager_load_overlays(dev);
	if (ret)
		return ret;

	mgr_dev = dev;
	return 0;
}

static void camera_manager_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	guard(mutex)(&mgr_mutex);
	int i;

	if (plat_subsys_enabled) {
		of_changeset_revert(&mgr_ocs);
		of_changeset_destroy(&mgr_ocs);
		plat_subsys_enabled = ocs_applied = false;
	}

	for (i = MAX_CAMERA_SLOTS - 1; i >= 0; i--) {
		struct camera_info* cam = &camera_slots[i];
		int ret;

		if (!(cam->applied && cam->filename))
			continue;

		ret = of_overlay_remove(&cam->overlay_id);
		if (ret) {
			dev_err(dev, "Failed to remove slot %d overlay: %d\n",
					i, ret);
			continue;
		}

		dev_info(dev, "Removed slot %d overlay '%s'\n", i, cam->filename);
		cam->overlay_id = 0;
		cam->applied = false;
	}

	mgr_dev = NULL;
}

static const struct of_device_id camera_manager_match_table[] = {
	{ .compatible = "camera-sensor-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, camera_manager_match_table);

static struct platform_driver camera_manager_driver = {
	.probe = camera_manager_probe,
	.remove = camera_manager_remove,
	.driver = {
		.name = "camera-sensor-manager",
		.of_match_table = camera_manager_match_table,
	},
};
module_platform_driver(camera_manager_driver);

MODULE_DESCRIPTION("I2C Camera sensor overlay manager");
MODULE_LICENSE("GPL v2");
