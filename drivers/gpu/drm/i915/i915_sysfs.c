/*
 * Copyright © 2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Ben Widawsky <ben@bwidawsk.net>
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/sysfs.h>

#include "gt/intel_gt_regs.h"
#include "gt/intel_rc6.h"
#include "gt/intel_rps.h"
#include "gt/sysfs_engines.h"

#include "i915_drv.h"
#include "i915_sysfs.h"

struct drm_i915_private *kdev_minor_to_i915(struct device *kdev)
{
	struct drm_minor *minor = dev_get_drvdata(kdev);
	return to_i915(minor->dev);
}

static int l3_access_valid(struct drm_i915_private *i915, loff_t offset)
{
	if (!HAS_L3_DPF(i915))
		return -EPERM;

	if (!IS_ALIGNED(offset, sizeof(u32)))
		return -EINVAL;

	if (offset >= GEN7_L3LOG_SIZE)
		return -ENXIO;

	return 0;
}

static ssize_t
i915_l3_read(struct file *filp, struct kobject *kobj,
	     struct bin_attribute *attr, char *buf,
	     loff_t offset, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	int slice = (int)(uintptr_t)attr->private;
	int ret;

	ret = l3_access_valid(i915, offset);
	if (ret)
		return ret;

	count = round_down(count, sizeof(u32));
	count = min_t(size_t, GEN7_L3LOG_SIZE - offset, count);
	memset(buf, 0, count);

	spin_lock(&i915->gem.contexts.lock);
	if (i915->l3_parity.remap_info[slice])
		memcpy(buf,
		       i915->l3_parity.remap_info[slice] + offset / sizeof(u32),
		       count);
	spin_unlock(&i915->gem.contexts.lock);

	return count;
}

static ssize_t
i915_l3_write(struct file *filp, struct kobject *kobj,
	      struct bin_attribute *attr, char *buf,
	      loff_t offset, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	int slice = (int)(uintptr_t)attr->private;
	u32 *remap_info, *freeme = NULL;
	struct i915_gem_context *ctx;
	int ret;

	ret = l3_access_valid(i915, offset);
	if (ret)
		return ret;

	if (count < sizeof(u32))
		return -EINVAL;

	remap_info = kzalloc(GEN7_L3LOG_SIZE, GFP_KERNEL);
	if (!remap_info)
		return -ENOMEM;

	spin_lock(&i915->gem.contexts.lock);

	if (i915->l3_parity.remap_info[slice]) {
		freeme = remap_info;
		remap_info = i915->l3_parity.remap_info[slice];
	} else {
		i915->l3_parity.remap_info[slice] = remap_info;
	}

	count = round_down(count, sizeof(u32));
	memcpy(remap_info + offset / sizeof(u32), buf, count);

	/* NB: We defer the remapping until we switch to the context */
	list_for_each_entry(ctx, &i915->gem.contexts.list, link)
		ctx->remap_slice |= BIT(slice);

	spin_unlock(&i915->gem.contexts.lock);
	kfree(freeme);

	/*
	 * TODO: Ideally we really want a GPU reset here to make sure errors
	 * aren't propagated. Since I cannot find a stable way to reset the GPU
	 * at this point it is left as a TODO.
	*/

	return count;
}

static const struct bin_attribute dpf_attrs = {
	.attr = {.name = "l3_parity", .mode = (S_IRUSR | S_IWUSR)},
	.size = GEN7_L3LOG_SIZE,
	.read = i915_l3_read,
	.write = i915_l3_write,
	.mmap = NULL,
	.private = (void *)0
};

static const struct bin_attribute dpf_attrs_1 = {
	.attr = {.name = "l3_parity_slice_1", .mode = (S_IRUSR | S_IWUSR)},
	.size = GEN7_L3LOG_SIZE,
	.read = i915_l3_read,
	.write = i915_l3_write,
	.mmap = NULL,
	.private = (void *)1
};

/*
 * RC6 related definitions
 */

static ssize_t rc6_enable_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t rc6_residency_ms_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t rc6p_residency_ms_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t rc6pp_residency_ms_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t media_rc6_residency_ms_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static struct device_attribute dev_attr_rc6_enable = __ATTR_RO(rc6_enable);
static struct device_attribute dev_attr_rc6_residency_ms = __ATTR_RO(rc6_residency_ms);
static struct device_attribute dev_attr_rc6p_residency_ms = __ATTR_RO(rc6p_residency_ms);
static struct device_attribute dev_attr_rc6pp_residency_ms = __ATTR_RO(rc6pp_residency_ms);
static struct device_attribute dev_attr_media_rc6_residency_ms = __ATTR_RO(media_rc6_residency_ms);

static struct attribute *rc6_dev_attrs[] = {
	&dev_attr_rc6_enable.attr,
	&dev_attr_rc6_residency_ms.attr,
	NULL
};

static struct attribute *rc6p_dev_attrs[] = {
	&dev_attr_rc6p_residency_ms.attr,
	&dev_attr_rc6pp_residency_ms.attr,
	NULL
};

static struct attribute *media_rc6_dev_attrs[] = {
	&dev_attr_media_rc6_residency_ms.attr,
	NULL
};

static const struct attribute_group rc6_attr_group = {
	.name = power_group_name,
	.attrs = rc6_dev_attrs
};

static const struct attribute_group rc6p_attr_group = {
	.name = power_group_name,
	.attrs = rc6p_dev_attrs
};

static const struct attribute_group media_rc6_attr_group = {
	.name = power_group_name,
	.attrs = media_rc6_dev_attrs
};

/*
 * RPS related definitions
 */

static ssize_t gt_act_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_cur_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_boost_freq_mhz_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buff, size_t count)
{
	return count;
}

static ssize_t gt_boost_freq_mhz_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_RP0_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_RP1_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_RPn_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_max_freq_mhz_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buff, size_t count)
{
	return count;
}

static ssize_t gt_max_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_min_freq_mhz_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buff, size_t count)
{
	return count;
}

static ssize_t gt_min_freq_mhz_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static ssize_t gt_vlv_rpe_freq_mhz_show(struct device *dev,
					struct device_attribute *attr,
					char *buff)
{
	return sysfs_emit(buff, "%d\n", 0);
}

static struct device_attribute dev_attr_gt_act_freq_mhz = __ATTR_RO(gt_act_freq_mhz);
static struct device_attribute dev_attr_gt_cur_freq_mhz = __ATTR_RO(gt_cur_freq_mhz);
static struct device_attribute dev_attr_gt_boost_freq_mhz = __ATTR_RW(gt_boost_freq_mhz);
static struct device_attribute dev_attr_gt_RP0_freq_mhz = __ATTR_RO(gt_RP0_freq_mhz);
static struct device_attribute dev_attr_gt_RP1_freq_mhz = __ATTR_RO(gt_RP1_freq_mhz);
static struct device_attribute dev_attr_gt_RPn_freq_mhz = __ATTR_RO(gt_RPn_freq_mhz);
static struct device_attribute dev_attr_gt_max_freq_mhz = __ATTR_RW(gt_max_freq_mhz);
static struct device_attribute dev_attr_gt_min_freq_mhz = __ATTR_RW(gt_min_freq_mhz);
static struct device_attribute dev_attr_gt_vlv_rpe_freq_mhz = __ATTR_RO(gt_vlv_rpe_freq_mhz);

static const struct attribute * const gen6_gt_attrs[]  = {
	&dev_attr_gt_act_freq_mhz.attr,
	&dev_attr_gt_cur_freq_mhz.attr,
	&dev_attr_gt_boost_freq_mhz.attr,
	&dev_attr_gt_max_freq_mhz.attr,
	&dev_attr_gt_min_freq_mhz.attr,
	&dev_attr_gt_RP0_freq_mhz.attr,
	&dev_attr_gt_RP1_freq_mhz.attr,
	&dev_attr_gt_RPn_freq_mhz.attr,
	NULL,
};

void i915_setup_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;
	int ret;

	if (HAS_L3_DPF(dev_priv)) {
		ret = device_create_bin_file(kdev, &dpf_attrs);
		if (ret)
			drm_err(&dev_priv->drm,
				"l3 parity sysfs setup failed\n");

		if (NUM_L3_SLICES(dev_priv) > 1) {
			ret = device_create_bin_file(kdev,
						     &dpf_attrs_1);
			if (ret)
				drm_err(&dev_priv->drm,
					"l3 parity slice 1 setup failed\n");
		}
	}

	dev_priv->sysfs_gt = kobject_create_and_add("gt", &kdev->kobj);
	if (!dev_priv->sysfs_gt)
		drm_warn(&dev_priv->drm,
			 "failed to register GT sysfs directory\n");

	i915_gpu_error_sysfs_setup(dev_priv);

	intel_engines_add_sysfs(dev_priv);

	/*
	 * Create i915-wide sysfs interfaces.
	 */

	/* rc6 related sysfs interfaces */
	ret = sysfs_merge_group(&kdev->kobj, &rc6_attr_group);
	if (ret)
		drm_warn(&dev_priv->drm,
			 "failed to create RC6 sysfs files (%pe)\n",
			 ERR_PTR(ret));

	if (HAS_RC6p(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj, &rc6p_attr_group);
		if (ret)
			drm_warn(&dev_priv->drm,
				 "failed to create RC6p sysfs files (%pe)\n",
				 ERR_PTR(ret));
	}

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj, &media_rc6_attr_group);
		if (ret)
			drm_warn(&dev_priv->drm,
				 "failed to create media RC6 sysfs files (%pe)\n",
				 ERR_PTR(ret));
	}

	/* rps related sysfs interfaces */
	if (GRAPHICS_VER(dev_priv) < 6)
		return;

	ret = sysfs_create_files(&kdev->kobj, gen6_gt_attrs);
	if (ret)
		drm_warn(&dev_priv->drm,
			 "failed to create RPS sysfs files (%pe)", ERR_PTR(ret));

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		ret = sysfs_create_file(&kdev->kobj,
					&dev_attr_gt_vlv_rpe_freq_mhz.attr);
		if (ret)
			drm_warn(&dev_priv->drm,
				 "failed to create RPS sysfs files (%pe)",
				 ERR_PTR(ret));
	}
}

void i915_teardown_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;

	i915_gpu_error_sysfs_teardown(dev_priv);

	device_remove_bin_file(kdev,  &dpf_attrs_1);
	device_remove_bin_file(kdev,  &dpf_attrs);

	kobject_put(dev_priv->sysfs_gt);
}
