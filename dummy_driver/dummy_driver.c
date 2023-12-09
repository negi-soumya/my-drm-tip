// SPDX-License-Identifier: GPL-2.0
/*
 * Outreachy Task- Write a dummy driver for learning sysfs.
 * This dummy driver (external LKM) creates sysfs interface for a dummy device
 * "dev1" under the root directory "dummy", mounted at /sys/kernel.
 *
 * Created:
 *	/sys/kernel/dummy		   -- top level sysfs directory
 *	/sys/kernel/dummy/dev1		   -- sysfs entry(dir) for dummy_dev
 *	/sys/kernel/dummy/dev1/ro_state    -- sysfs file (read-only attribute)
 *	/sys/kernel/dummy/dev1/rw_state	   -- sysfs file (read-write attribute)
 *
 * Notes for myself:
 *	- If modprobe fails..check dmesg for call trace.
 *	- If modprobe freezes..probably code has a segfault. Check.
 *	- Some kobject functions need kobject_put() for handling failures, else
 *	  sysfs teardown is now clean. Check.
 *	- How does driver modify attribute values. load and show are triggered
 *	  by read & write syscalls from uapi.
 */

#define pr_fmt(fmt) "DUMMY DRIVER: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#define kobj_to_dummy_dev(p) container_of(p, struct dummy_t, intf)
#define attr_to_dummy_attr(p) container_of(p, struct dummy_attribute, attr)

MODULE_AUTHOR("Soumya Negi");
MODULE_LICENSE("GPL");

/* A dummy device */
struct dummy_t {
	unsigned int ro_state;
	unsigned int rw_state;
	struct kobject intf;
};

struct dummy_attribute {
	struct attribute attr;
	ssize_t (*show)(struct dummy_t *dev, struct dummy_attribute *attr,
			char *buf);
	ssize_t (*store)(struct dummy_t *dev, struct dummy_attribute *attr,
			 const char *buf, size_t count);
};

/* ------------------------------------------------------------------------- */

static struct dummy_t *dummy_dev;
static struct kobject *rootdir;

/*
 * Attributes show/store functions
 */
static ssize_t ro_state_show(struct dummy_t *dev, struct dummy_attribute *attr,
			     char *buf)
{
	/* driver writes value to sysfs file, so it can be read by uapi */
	return sysfs_emit(buf, "%d\n", dev->ro_state);
}

static ssize_t rw_state_show(struct dummy_t *dev, struct dummy_attribute *attr,
			     char *buf)
{
	/* driver writes value to sysfs file, so it can be read by uapi */
	return sysfs_emit(buf, "%d\n", dev->rw_state);
}

static ssize_t rw_state_store(struct dummy_t *dev, struct dummy_attribute *attr,
			      const char *buf, size_t count)
{
	int retval, value;

	/* driver reads value from sysfs file */
	retval = kstrtoint(buf, 10, &value);
	if (retval < 0)
		return retval;

	/* driver updates device attribute value (changed externally by uapi) */
	WRITE_ONCE(dev->rw_state, value);

	return count;
}

/* Dummy attributes */
static struct dummy_attribute ro_state_attr = __ATTR_RO(ro_state);
static struct dummy_attribute rw_state_attr = __ATTR_RW(rw_state);

/* List of attributes */
static struct attribute *dummy_attrs[] = {
	&ro_state_attr.attr,
	&rw_state_attr.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/* Attribute group */
static struct attribute_group dummy_attr_group = {
	.attrs = dummy_attrs
};

/* sysfs ops struct definiction and show/store function definitions */

/* This fn takes a generic kobject and attribute and converts is to dummy_dev
 * and dummy_attribute. It then calls the actual show function registered for
 * the passed attribute (set during the __ATTR*() macros calls) during dummy
 * attribute creation.
 */
static ssize_t dummy_attr_show(struct kobject *kobj, struct attribute *attr,
			       char *buf)
{
	struct dummy_attribute *dum_attr = attr_to_dummy_attr(attr);
	struct dummy_t *dev = kobj_to_dummy_dev(kobj);
	ssize_t ret = -EIO;

	if (dum_attr->show)
		ret = dum_attr->show(dev, dum_attr, buf);
	if (ret >= (ssize_t)PAGE_SIZE) {
		pr_warn("%s: %pS returned bad count\n",
			__func__,
			dum_attr->show);
	}

	return ret;
}

/* This fn takes a generic kobject and attribute and converts is to dummy_dev
 * and dummy_attribute. It then calls the actual store function registered for
 * the passed attribute (set during the __ATTR*() macros calls) during dummy
 * attribute creation.
 */
static ssize_t dummy_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct dummy_attribute *dum_attr = attr_to_dummy_attr(attr);
	struct dummy_t *dev = kobj_to_dummy_dev(kobj);
	ssize_t ret = -EIO;

	if (dum_attr->store)
		ret = dum_attr->store(dev, dum_attr, buf, count);

	if (ret >= (ssize_t)PAGE_SIZE) {
		pr_warn("%s: %pS returned bad count\n",
			__func__,
			dum_attr->show);
	}

	return ret;
}

static const struct sysfs_ops dummy_sysfs_ops = {
	.show = dummy_attr_show,
	.store = dummy_attr_store
};

/*
 * Release callback (registered for a ktype)
 */
static void dummy_release(struct kobject *kobj)
{
	struct dummy_t *dev = kobj_to_dummy_dev(kobj);

	kfree(dev); /* Free dummy device pointer */
}

/* kobject ktype definition */
static const struct kobj_type dummy_ktype = {
	.release = &dummy_release,
	.sysfs_ops = &dummy_sysfs_ops
};

/* This fn sets up sysfs entries for dummy device & its attributes */
static int create_device_sysfs_interfaces(void)
{
	int retval;

	retval = kobject_init_and_add(&dummy_dev->intf, &dummy_ktype, rootdir,
				      "dev1");
	if (retval) {
		pr_err("Failed to register dummy device with sysfs\n");
		kobject_put(&dummy_dev->intf);
		return retval;
	}

	/* Create attributes as a group (sysfs files under the dev directory) */
	retval = sysfs_create_group(&dummy_dev->intf, &dummy_attr_group);
	if (retval) {
		kobject_put(&dummy_dev->intf);
		pr_err("Failed to register dummy device with sysfs\n");
	}

	return retval;
}

/* This fn tears down sysfs entries for dummy device & its attributes */
static void destroy_device_sysfs_interfaces(void)
{
	/* If kobject is not cleaned up properly (or kobject initialization
	 * failure is not handled properly) then sysfs entries would remain in
	 * sysfs. To remove them, we would either need a reboot or a call
	 * to kobject_del() on the same kobject from inside the kernel(another
	 * driver)
	 */

	sysfs_remove_group(&dummy_dev->intf, &dummy_attr_group);
	kobject_put(&dummy_dev->intf);
}

/* This fn sets up the top-level sysfs directory for dummy driver */
static int init_sysfs_interfaces(void)
{
	int retval;

	/* Create a directory in sysfs for dummy driver (at /sys/kernel) */
	rootdir = kobject_create_and_add("dummy", kernel_kobj);
	if (!rootdir) {
		pr_err("Failed to register dummy rootdir with sysfs\n");
		return -ENOMEM;
	}

	retval = create_device_sysfs_interfaces();
	return retval;
}

static int __init init_dummy_driver(void)
{
	int retval;

	pr_info("Initializing module...\n");

	/* create device */
	dummy_dev = kzalloc(sizeof(*dummy_dev), GFP_KERNEL);

	retval = init_sysfs_interfaces();
	if (retval) {
		pr_info("Initialization failed.\n");
		goto finish;
	}

	/* At this point, attribute sysfs file should be zeroed. */

	/* Initialize device attributes..will reflect in sysfs files */
	dummy_dev->ro_state = 9090;
	WRITE_ONCE(dummy_dev->rw_state, 5050);

	/* Announce that kobject has been created to userspace */
	kobject_uevent(&dummy_dev->intf, KOBJ_ADD);

	pr_info("Initialization done. sysfs interfaces have been created.\n");
finish:
	return retval;
}

static void __exit exit_dummy_driver(void)
{
	destroy_device_sysfs_interfaces();
	kobject_put(rootdir);
	pr_info("Exited module\n");
}

module_init(init_dummy_driver);
module_exit(exit_dummy_driver);
