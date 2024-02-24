// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include <drm/drm_device.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/printk.h>
#include <linux/sysfs.h>

#include "i915_drv.h"
#include "i915_sysfs.h"
#include "intel_gt.h"
#include "intel_gt_print.h"
#include "intel_gt_sysfs.h"
#include "intel_gt_sysfs_pm.h"
#include "intel_gt_types.h"
#include "intel_rc6.h"

bool is_object_gt(struct kobject *kobj)
{
	return !strncmp(kobj->name, "gt", 2);
}

static ssize_t id_show(struct kobject *kobj,
		       struct kobj_attribute *attr,
		       char *buf)
{
	struct intel_gt *gt = kobj_to_gt(kobj);

	return sysfs_emit(buf, "%u\n", gt->info.id);
}
static struct kobj_attribute attr_id = __ATTR_RO(id);

static struct attribute *id_attrs[] = {
	&attr_id.attr,
	NULL,
};
ATTRIBUTE_GROUPS(id);

/* A kobject needs a release() method even if it does nothing */
static void kobj_gt_release(struct kobject *kobj)
{
}

static const struct kobj_type kobj_gt_type = {
	.release = kobj_gt_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = id_groups,
};

void intel_gt_sysfs_register(struct intel_gt *gt)
{
	/* init and xfer ownership to sysfs tree */
	if (kobject_init_and_add(&gt->sysfs_gt, &kobj_gt_type,
				 gt->i915->sysfs_gt, "gt%d", gt->info.id))
		goto exit_fail;

	gt->sysfs_defaults = kobject_create_and_add(".defaults", &gt->sysfs_gt);
	if (!gt->sysfs_defaults)
		goto exit_fail;

	intel_gt_sysfs_pm_init(gt, &gt->sysfs_gt);

	return;

exit_fail:
	kobject_put(&gt->sysfs_gt);
	gt_warn(gt, "failed to initialize sysfs root\n");
}

void intel_gt_sysfs_unregister(struct intel_gt *gt)
{
	kobject_put(gt->sysfs_defaults);
	kobject_put(&gt->sysfs_gt);
}
