// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * System Memory Protection Unit (SMPU) for ADI SC59x processors
 *
 * Copyright (C) 2026 - Analog Devices, Inc.
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/soc/adi/smpu.h>

/* Register offsets */
#define ADI_SMPU_REG_CTL		0x00
#define ADI_SMPU_REG_STAT		0x04
#define ADI_SMPU_REG_IADDR		0x08
#define ADI_SMPU_REG_IDTLS		0x0C
#define ADI_SMPU_REG_BADDR		0x10
#define ADI_SMPU_REG_BDTLS		0x14

/* Region configuration registers (stride = 0x18 per region) */
#define ADI_SMPU_RCTL(n)		(0x20 + (n) * 0x18)
#define ADI_SMPU_RADDR(n)		(0x24 + (n) * 0x18)
#define ADI_SMPU_RIDA(n)		(0x28 + (n) * 0x18)
#define ADI_SMPU_RIDMSKA(n)		(0x2C + (n) * 0x18)
#define ADI_SMPU_RIDB(n)		(0x30 + (n) * 0x18)
#define ADI_SMPU_RIDMSKB(n)		(0x34 + (n) * 0x18)

/* Security registers */
#define ADI_SMPU_REG_SECURECTL		0x800
#define ADI_SMPU_SECURERCTL(n)		(0x820 + (n) * 0x04)
#define ADI_SMPU_REG_REVID		0x220

/* SMPU_SECURECTL register bits */
#define ADI_SMPU_SECURECTL_LOCK		BIT(31)
#define ADI_SMPU_SECURECTL_WSECDIS	BIT(11)
#define ADI_SMPU_SECURECTL_WNSEN	BIT(10)
#define ADI_SMPU_SECURECTL_RSECDIS	BIT(9)
#define ADI_SMPU_SECURECTL_RNSEN	BIT(8)
#define ADI_SMPU_SECURECTL_RLOCK	BIT(3)
#define ADI_SMPU_SECURECTL_SINTEN	BIT(2)
#define ADI_SMPU_SECURECTL_SBETYPE	BIT(1)
#define ADI_SMPU_SECURECTL_SBEDIS	BIT(0)

/* SMPU_SECURERCTL[n] register bits */
#define ADI_SMPU_SECURERCTL_WSECDIS	BIT(3)
#define ADI_SMPU_SECURERCTL_WNSEN	BIT(2)
#define ADI_SMPU_SECURERCTL_RSECDIS	BIT(1)
#define ADI_SMPU_SECURERCTL_RNSEN	BIT(0)

/* SMPU_CTL register bits */
#define ADI_SMPU_CTL_LOCK		BIT(31)
#define ADI_SMPU_CTL_RLOCK		BIT(4)
#define ADI_SMPU_CTL_PINTEN		BIT(3)
#define ADI_SMPU_CTL_PBETYPE		BIT(2)
#define ADI_SMPU_CTL_PBEDIS		BIT(1)
#define ADI_SMPU_CTL_RSDIS		BIT(0)

/* SMPU_STAT register bits (W1C) */
#define ADI_SMPU_STAT_LWERR		BIT(17)
#define ADI_SMPU_STAT_ADRERR		BIT(16)
#define ADI_SMPU_STAT_BEOVR		BIT(3)
#define ADI_SMPU_STAT_BERR		BIT(2)
#define ADI_SMPU_STAT_IOVR		BIT(1)
#define ADI_SMPU_STAT_IRQ		BIT(0)
#define ADI_SMPU_STAT_W1C_MASK		0x3002F

/* SMPU_RCTL register bits */
#define ADI_SMPU_RCTL_WIDCINV		BIT(11)
#define ADI_SMPU_RCTL_WPROTEN		BIT(10)
#define ADI_SMPU_RCTL_RIDCINV		BIT(9)
#define ADI_SMPU_RCTL_RPROTEN		BIT(8)
#define ADI_SMPU_RCTL_SIZE_SHIFT	1
#define ADI_SMPU_RCTL_SIZE_MASK		GENMASK(5, 1)
#define ADI_SMPU_RCTL_EN		BIT(0)

/* SMPU_IDTLS/BDTLS register bits */
#define ADI_SMPU_DTLS_ID_SHIFT		8
#define ADI_SMPU_DTLS_ID_MASK		GENMASK(20, 8)
#define ADI_SMPU_DTLS_RW		BIT(1)
#define ADI_SMPU_DTLS_SECURE		BIT(0)

/* Maximum number of SMPU instances */
#define ADI_SMPU_MAX_INSTANCES		9

/* Violation history buffer size per instance */
#define ADI_SMPU_VIOL_HISTORY_SIZE	32

/**
 * struct adi_smpu_violation - SMPU violation record
 * @timestamp: Timestamp of violation from jiffies
 * @addr: Violated address
 * @id: Transaction ID that caused violation
 * @is_write: True if write violation, false if read
 * @is_secure: True if secure transaction
 */
struct adi_smpu_violation {
	unsigned long timestamp;
	u32 addr;
	u16 id;
	bool is_write;
	bool is_secure;
};

/**
 * struct adi_smpu_instance - Per-instance SMPU data
 * @base: Memory-mapped I/O base address
 * @instance_id: SMPU instance ID
 * @name: Instance name
 * @region_bitmap: Bitmap of allocated regions
 * @lock: Spinlock for register access
 * @smpu: Back pointer to main SMPU structure
 * @violations: Circular buffer of violations
 * @viol_head: Head index of violation buffer
 * @viol_count: Total violation count
 * @debugfs_dir: Debugfs directory for this instance
 */
struct adi_smpu_instance {
	void __iomem *base;
	int instance_id;
	const char *name;
	unsigned long region_bitmap;
	spinlock_t lock;
	struct adi_smpu *smpu;
	struct adi_smpu_violation violations[ADI_SMPU_VIOL_HISTORY_SIZE];
	unsigned int viol_head;
	unsigned long viol_count;
	struct dentry *debugfs_dir;
};

/**
 * struct adi_smpu - Main SMPU driver data
 * @dev: Device pointer
 * @instances: Array of SMPU instances
 * @num_instances: Number of active instances
 * @debugfs_root: Root debugfs directory
 * @secure_regs_available: Can access 0x800+ registers (Errata, 20000003)
 */
struct adi_smpu {
	struct device *dev;
	struct adi_smpu_instance instances[ADI_SMPU_MAX_INSTANCES];
	int num_instances;
	struct dentry *debugfs_root;
	bool secure_regs_available;
};

/* Register access helpers */
static inline u32 adi_smpu_readl(struct adi_smpu_instance *inst, u32 offset)
{
	return readl(inst->base + offset);
}

static inline void adi_smpu_writel(struct adi_smpu_instance *inst,
				    u32 val, u32 offset)
{
	writel(val, inst->base + offset);
}

/**
 * adi_smpu_write_secure_reg - Write to secure register
 * @inst: SMPU instance
 * @val: Value to write
 * @offset: Register offset (must be >= 0x800)
 *
 * Writes to secure registers.
 *
 * Return: 0 on success, -ENOTSUPP if secure registers unavailable, -EINVAL on error
 */
static int adi_smpu_write_secure_reg(struct adi_smpu_instance *inst,
				      u32 val, u32 offset)
{
	if (!inst->smpu->secure_regs_available) {
		dev_warn_once(inst->smpu->dev,
			      "Secure register access blocked (Anomaly 20000003)\n");
		return -ENOTSUPP;
	}

	if (offset < 0x800) {
		dev_err(inst->smpu->dev, "Invalid secure register offset 0x%x\n", offset);
		return -EINVAL;
	}

	adi_smpu_writel(inst, val, offset);
	return 0;
}

/**
 * adi_smpu_read_secure_reg - Read from secure register
 * @inst: SMPU instance
 * @val: Pointer to store read value
 * @offset: Register offset (must be >= 0x800)
 *
 * Reads from secure registers.
 *
 * Return: 0 on success, -ENOTSUPP if secure registers unavailable, -EINVAL on error
 */
static int adi_smpu_read_secure_reg(struct adi_smpu_instance *inst,
				     u32 *val, u32 offset)
{
	if (!inst->smpu->secure_regs_available)
		return -ENOTSUPP;

	if (offset < 0x800)
		return -EINVAL;

	*val = adi_smpu_readl(inst, offset);
	return 0;
}

/**
 * adi_smpu_get_instance - Get SMPU instance by ID
 * @smpu: SMPU driver data
 * @instance_id: Instance ID to find
 *
 * Return: Pointer to instance or NULL if not found
 */
static struct adi_smpu_instance *
adi_smpu_get_instance(struct adi_smpu *smpu, int instance_id)
{
	int i;

	for (i = 0; i < smpu->num_instances; i++) {
		if (smpu->instances[i].instance_id == instance_id)
			return &smpu->instances[i];
	}

	return NULL;
}

/* Forward declarations */
static int adi_smpu_configure_region_secure(struct adi_smpu *smpu,
					     int instance_id, int region_num,
					     const struct adi_smpu_region_config *config);

/**
 * adi_smpu_calc_size_bits - Calculate size bits for region size
 * @size: Region size in bytes
 *
 * SMPU hardware encodes region size as power-of-2 bits:
 * Size = 4KB  -> bits = 0  (1 << 12)
 * Size = 4GB  -> bits = 20 (1 << 32)
 *
 * Return: Size bits value, or negative error
 */
static int adi_smpu_calc_size_bits(size_t size)
{
	int bits;

	if (size < SZ_4K || size > SZ_4G)
		return -EINVAL;

	if (!is_power_of_2(size))
		return -EINVAL;

	bits = ilog2(size) - 12;
	if (bits < 0 || bits > 20)
		return -EINVAL;

	return bits;
}

/**
 * adi_smpu_validate_region - Validate region configuration
 * @config: Region configuration to validate
 *
 * Return: 0 if valid, negative error otherwise
 */
static int adi_smpu_validate_region(const struct adi_smpu_region_config *config)
{
	int size_bits;

	if (!config)
		return -EINVAL;

	/* Validate size */
	size_bits = adi_smpu_calc_size_bits(config->size);
	if (size_bits < 0)
		return -EINVAL;

	/* Validate alignment */
	if (!IS_ALIGNED(config->base, config->size))
		return -EINVAL;

	return 0;
}

/**
 * adi_smpu_store_violation - Store violation in history buffer
 * @inst: SMPU instance
 * @addr: Violation address
 * @details: Violation details register value
 */
static void adi_smpu_store_violation(struct adi_smpu_instance *inst,
				      u32 addr, u32 details)
{
	struct adi_smpu_violation *viol;
	unsigned int idx;

	idx = inst->viol_head % ADI_SMPU_VIOL_HISTORY_SIZE;
	viol = &inst->violations[idx];

	viol->timestamp = jiffies;
	viol->addr = addr;
	viol->id = (details & ADI_SMPU_DTLS_ID_MASK) >> ADI_SMPU_DTLS_ID_SHIFT;
	viol->is_write = !!(details & ADI_SMPU_DTLS_RW);
	viol->is_secure = !!(details & ADI_SMPU_DTLS_SECURE);

	inst->viol_head++;
	inst->viol_count++;
}

/**
 * adi_smpu_log_violation - Log protection violation
 * @inst: SMPU instance
 * @addr: Violation address
 * @details: Violation details
 */
static void adi_smpu_log_violation(struct adi_smpu_instance *inst,
				    u32 addr, u32 details)
{
	u16 trans_id = (details & ADI_SMPU_DTLS_ID_MASK) >> ADI_SMPU_DTLS_ID_SHIFT;
	bool is_write = !!(details & ADI_SMPU_DTLS_RW);
	bool is_secure = !!(details & ADI_SMPU_DTLS_SECURE);

	dev_err(inst->smpu->dev,
		"%s violation: addr=0x%08x id=0x%03x %s %s\n",
		inst->name, addr, trans_id,
		is_write ? "write" : "read",
		is_secure ? "secure" : "non-secure");

	adi_smpu_store_violation(inst, addr, details);
}

/**
 * adi_smpu_irq_handler - SMPU interrupt handler
 * @irq: IRQ number
 * @dev_id: Device data (struct adi_smpu pointer)
 *
 * Handles protection violations for all SMPU instances.
 * Multiple instances share the same IRQ line (SECID 217).
 *
 * Return: IRQ_HANDLED if any violation processed, IRQ_NONE otherwise
 */
static irqreturn_t adi_smpu_irq_handler(int irq, void *dev_id)
{
	struct adi_smpu *smpu = dev_id;
	irqreturn_t ret = IRQ_NONE;
	int i;

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		unsigned long flags;
		u32 status, iaddr, idtls;

		spin_lock_irqsave(&inst->lock, flags);

		status = adi_smpu_readl(inst, ADI_SMPU_REG_STAT);

		if (status & ADI_SMPU_STAT_IRQ) {
			iaddr = adi_smpu_readl(inst, ADI_SMPU_REG_IADDR);
			idtls = adi_smpu_readl(inst, ADI_SMPU_REG_IDTLS);

			adi_smpu_log_violation(inst, iaddr, idtls);

			/* Check for overrun conditions */
			if (status & ADI_SMPU_STAT_IOVR)
				dev_warn(inst->smpu->dev,
					 "%s: interrupt overrun\n", inst->name);

			if (status & ADI_SMPU_STAT_BERR) {
				u32 baddr = adi_smpu_readl(inst, ADI_SMPU_REG_BADDR);
				dev_err(inst->smpu->dev,
					"%s: bus error at 0x%08x\n",
					inst->name, baddr);
			}

			/* Clear W1C status bits */
			adi_smpu_writel(inst, status & ADI_SMPU_STAT_W1C_MASK,
					ADI_SMPU_REG_STAT);

			ret = IRQ_HANDLED;
		}

		spin_unlock_irqrestore(&inst->lock, flags);
	}

	return ret;
}

/**
 * adi_smpu_reset_instance - Reset SMPU instance to safe state
 * @inst: SMPU instance
 *
 * Disables all regions and clears pending violations.
 */
static void adi_smpu_reset_instance(struct adi_smpu_instance *inst)
{
	unsigned long flags;
	u32 val;
	int i;

	spin_lock_irqsave(&inst->lock, flags);

	/* Disable all regions */
	for (i = 0; i < ADI_SMPU_NUM_REGIONS; i++)
		adi_smpu_writel(inst, 0, ADI_SMPU_RCTL(i));

	/* Clear any pending violations */
	adi_smpu_writel(inst, ADI_SMPU_STAT_W1C_MASK, ADI_SMPU_REG_STAT);

	/* Leave violation interrupts disabled until an IRQ is registered. */
	val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
	val &= ~ADI_SMPU_CTL_PINTEN;
	adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);

	spin_unlock_irqrestore(&inst->lock, flags);
}

/* Public API implementation */

struct adi_smpu *adi_smpu_get_from_node(struct device *dev)
{
	struct platform_device *smpu_pdev;
	struct device_node *smpu_node;
	struct adi_smpu *ret = NULL;

	smpu_node = of_parse_phandle(dev->of_node, "adi,smpu", 0);
	if (!smpu_node) {
		dev_err(dev, "Missing adi,smpu phandle in device tree\n");
		return ERR_PTR(-ENODEV);
	}

	smpu_pdev = of_find_device_by_node(smpu_node);
	if (!smpu_pdev) {
		ret = ERR_PTR(-EPROBE_DEFER);
		goto cleanup;
	}

	ret = dev_get_drvdata(&smpu_pdev->dev);
	if (!ret)
		ret = ERR_PTR(-EPROBE_DEFER);

cleanup:
	of_node_put(smpu_node);
	return ret;
}
EXPORT_SYMBOL(adi_smpu_get_from_node);

void adi_smpu_put(struct adi_smpu *smpu)
{
	if (smpu)
		put_device(smpu->dev);
}
EXPORT_SYMBOL(adi_smpu_put);

int adi_smpu_configure_region(struct adi_smpu *smpu, int instance_id,
			       int region_num,
			       const struct adi_smpu_region_config *config)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	u32 rctl, raddr;
	int size_bits;
	int ret;

	if (!smpu || !config)
		return -EINVAL;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return -EINVAL;

	ret = adi_smpu_validate_region(config);
	if (ret)
		return ret;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	size_bits = adi_smpu_calc_size_bits(config->size);

	spin_lock_irqsave(&inst->lock, flags);

	/* Disable region during configuration */
	adi_smpu_writel(inst, 0, ADI_SMPU_RCTL(region_num));

	/* Configure base address (bits 31:12) */
	raddr = (config->base >> 12) & 0xFFFFF;
	adi_smpu_writel(inst, raddr, ADI_SMPU_RADDR(region_num));

	/* Configure transaction ID filters */
	adi_smpu_writel(inst, config->allowed_ids[0] & 0x1FFF,
			ADI_SMPU_RIDA(region_num));
	adi_smpu_writel(inst, config->id_masks[0] & 0x1FFF,
			ADI_SMPU_RIDMSKA(region_num));
	adi_smpu_writel(inst, config->allowed_ids[1] & 0x1FFF,
			ADI_SMPU_RIDB(region_num));
	adi_smpu_writel(inst, config->id_masks[1] & 0x1FFF,
			ADI_SMPU_RIDMSKB(region_num));

	/* Configure control register */
	rctl = (size_bits << ADI_SMPU_RCTL_SIZE_SHIFT) & ADI_SMPU_RCTL_SIZE_MASK;

	if (config->permissions & ADI_SMPU_PERM_READ)
		rctl |= ADI_SMPU_RCTL_RPROTEN;
	if (config->permissions & ADI_SMPU_PERM_WRITE)
		rctl |= ADI_SMPU_RCTL_WPROTEN;
	if (config->id_invert[0])
		rctl |= ADI_SMPU_RCTL_RIDCINV;
	if (config->id_invert[1])
		rctl |= ADI_SMPU_RCTL_WIDCINV;

	/* Write control register (region still disabled) */
	adi_smpu_writel(inst, rctl, ADI_SMPU_RCTL(region_num));

	spin_unlock_irqrestore(&inst->lock, flags);

	dev_dbg(smpu->dev, "%s: configured region %d: base=0x%llx size=0x%zx\n",
		inst->name, region_num, (u64)config->base, config->size);

	/* Configure secure access if requested and available */
	if (config->secure_mode != ADI_SMPU_SEC_BOTH) {
		ret = adi_smpu_configure_region_secure(smpu, instance_id,
						       region_num, config);
		/* Errors are logged but don't fail configuration */
	}

	return 0;
}
EXPORT_SYMBOL(adi_smpu_configure_region);

/**
 * adi_smpu_configure_region_secure - Configure per-region secure access
 * @smpu: SMPU handle
 * @instance_id: SMPU instance
 * @region_num: Region number
 * @config: Region configuration with secure_mode field
 *
 * Configures SECURERCTL[n] register for the specified region.
 * Only works if secure_regs_available flag is set.
 *
 * Return: 0 on success, -ENOTSUPP if secure regs unavailable, -EINVAL on error
 */
static int adi_smpu_configure_region_secure(struct adi_smpu *smpu,
					     int instance_id, int region_num,
					     const struct adi_smpu_region_config *config)
{
	struct adi_smpu_instance *inst;
	u32 securerctl = 0;
	int ret;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return -EINVAL;

	/* Build SECURERCTL value based on mode */
	switch (config->secure_mode) {
	case ADI_SMPU_SEC_BOTH:
		/* Allow both secure and non-secure */
		securerctl = ADI_SMPU_SECURERCTL_RNSEN |   /* Non-secure reads */
			     ADI_SMPU_SECURERCTL_WNSEN;    /* Non-secure writes */
		/* Secure enabled by default (RSECDIS=0, WSECDIS=0) */
		break;

	case ADI_SMPU_SEC_SECURE_ONLY:
		/* Allow only secure transactions */
		securerctl = 0;  /* Non-secure enabled by default */
		/* Secure enabled by default */
		break;

	case ADI_SMPU_SEC_NONSECURE_ONLY:
		/* Allow only non-secure transactions */
		securerctl = ADI_SMPU_SECURERCTL_RNSEN |
			     ADI_SMPU_SECURERCTL_WNSEN |
			     ADI_SMPU_SECURERCTL_RSECDIS |  /* Block secure reads */
			     ADI_SMPU_SECURERCTL_WSECDIS;   /* Block secure writes */
		break;

	case ADI_SMPU_SEC_NONE:
		/* Block all */
		securerctl = ADI_SMPU_SECURERCTL_RSECDIS |
			     ADI_SMPU_SECURERCTL_WSECDIS;
		/* RNSEN=0, WNSEN=0 already blocks non-secure */
		break;

	default:
		return -EINVAL;
	}

	/* Write to SECURERCTL[n] register */
	ret = adi_smpu_write_secure_reg(inst, securerctl,
					 ADI_SMPU_SECURERCTL(region_num));
	if (ret == -ENOTSUPP) {
		/* Just warning */
		dev_warn_once(smpu->dev,
			      "Secure mode not available, using TID filtering only\n");
		return 0;
	}

	dev_dbg(smpu->dev, "%s region %d: secure_mode=%d (SECURERCTL=0x%x)\n",
		inst->name, region_num, config->secure_mode, securerctl);

	return ret;
}

int adi_smpu_enable_region(struct adi_smpu *smpu, int instance_id,
			    int region_num)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	u32 rctl;

	if (!smpu)
		return -EINVAL;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return -EINVAL;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	rctl = adi_smpu_readl(inst, ADI_SMPU_RCTL(region_num));
	rctl |= ADI_SMPU_RCTL_EN;
	adi_smpu_writel(inst, rctl, ADI_SMPU_RCTL(region_num));

	spin_unlock_irqrestore(&inst->lock, flags);

	dev_dbg(smpu->dev, "%s: enabled region %d\n", inst->name, region_num);

	return 0;
}
EXPORT_SYMBOL(adi_smpu_enable_region);

int adi_smpu_disable_region(struct adi_smpu *smpu, int instance_id,
			     int region_num)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	u32 rctl;

	if (!smpu)
		return -EINVAL;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return -EINVAL;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	rctl = adi_smpu_readl(inst, ADI_SMPU_RCTL(region_num));
	rctl &= ~ADI_SMPU_RCTL_EN;
	adi_smpu_writel(inst, rctl, ADI_SMPU_RCTL(region_num));

	spin_unlock_irqrestore(&inst->lock, flags);

	dev_dbg(smpu->dev, "%s: disabled region %d\n", inst->name, region_num);

	return 0;
}
EXPORT_SYMBOL(adi_smpu_disable_region);

int adi_smpu_lock_config(struct adi_smpu *smpu, int instance_id)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	u32 val;

	if (!smpu)
		return -EINVAL;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
	val |= ADI_SMPU_CTL_LOCK | ADI_SMPU_CTL_RLOCK;
	adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);

	spin_unlock_irqrestore(&inst->lock, flags);

	dev_info(smpu->dev, "%s: configuration locked\n", inst->name);

	return 0;
}
EXPORT_SYMBOL(adi_smpu_lock_config);

int adi_smpu_allocate_region(struct adi_smpu *smpu, int instance_id)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	int region;

	if (!smpu)
		return -EINVAL;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	region = find_first_zero_bit(&inst->region_bitmap, ADI_SMPU_NUM_REGIONS);
	if (region >= ADI_SMPU_NUM_REGIONS) {
		spin_unlock_irqrestore(&inst->lock, flags);
		return -ENOSPC;
	}

	set_bit(region, &inst->region_bitmap);

	spin_unlock_irqrestore(&inst->lock, flags);

	return region;
}
EXPORT_SYMBOL(adi_smpu_allocate_region);

void adi_smpu_free_region(struct adi_smpu *smpu, int instance_id,
			  int region_num)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;

	if (!smpu || region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return;

	spin_lock_irqsave(&inst->lock, flags);

	/* Disable region */
	adi_smpu_writel(inst, 0, ADI_SMPU_RCTL(region_num));

	/* Mark as free */
	clear_bit(region_num, &inst->region_bitmap);

	spin_unlock_irqrestore(&inst->lock, flags);
}
EXPORT_SYMBOL(adi_smpu_free_region);

/* Debugfs interface */

static int smpu_status_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;
	unsigned long flags;
	u32 ctl, stat;

	spin_lock_irqsave(&inst->lock, flags);
	ctl = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
	stat = adi_smpu_readl(inst, ADI_SMPU_REG_STAT);
	spin_unlock_irqrestore(&inst->lock, flags);

	seq_printf(s, "Control:   0x%08x\n", ctl);
	seq_printf(s, "  Locked:        %s\n", ctl & ADI_SMPU_CTL_LOCK ? "yes" : "no");
	seq_printf(s, "  Region locked: %s\n", ctl & ADI_SMPU_CTL_RLOCK ? "yes" : "no");
	seq_printf(s, "  IRQ enabled:   %s\n", ctl & ADI_SMPU_CTL_PINTEN ? "yes" : "no");
	seq_printf(s, "\nStatus:    0x%08x\n", stat);
	seq_printf(s, "  IRQ pending:   %s\n", stat & ADI_SMPU_STAT_IRQ ? "yes" : "no");
	seq_printf(s, "  Bus error:     %s\n", stat & ADI_SMPU_STAT_BERR ? "yes" : "no");

	/* Show secure control (if available) */
	if (inst->smpu->secure_regs_available) {
		u32 securectl;
		int ret = adi_smpu_read_secure_reg(inst, &securectl, ADI_SMPU_REG_SECURECTL);

		if (ret == 0) {
			seq_printf(s, "\nSECURECTL: 0x%08x\n", securectl);
			seq_printf(s, "  Secure reads:      %s\n",
				   (securectl & ADI_SMPU_SECURECTL_RSECDIS) ? "disabled" : "enabled");
			seq_printf(s, "  Secure writes:     %s\n",
				   (securectl & ADI_SMPU_SECURECTL_WSECDIS) ? "disabled" : "enabled");
			seq_printf(s, "  Non-secure reads:  %s\n",
				   (securectl & ADI_SMPU_SECURECTL_RNSEN) ? "enabled" : "disabled");
			seq_printf(s, "  Non-secure writes: %s\n",
				   (securectl & ADI_SMPU_SECURECTL_WNSEN) ? "enabled" : "disabled");
			seq_printf(s, "  Security IRQ:      %s\n",
				   (securectl & ADI_SMPU_SECURECTL_SINTEN) ? "enabled" : "disabled");
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_status);

static int smpu_regions_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;
	unsigned long flags;
	int i;

	seq_printf(s, "%-6s %-10s %-12s %-10s %-6s %-8s %-8s\n",
		   "Region", "Enabled", "Base", "Size", "Perms", "IDA", "IDB");
	seq_puts(s, "------------------------------------------------------------\n");

	spin_lock_irqsave(&inst->lock, flags);

	for (i = 0; i < ADI_SMPU_NUM_REGIONS; i++) {
		u32 rctl = adi_smpu_readl(inst, ADI_SMPU_RCTL(i));
		u32 raddr = adi_smpu_readl(inst, ADI_SMPU_RADDR(i));
		u32 rida = adi_smpu_readl(inst, ADI_SMPU_RIDA(i));
		u32 ridb = adi_smpu_readl(inst, ADI_SMPU_RIDB(i));

		if (rctl & ADI_SMPU_RCTL_EN) {
			int size_bits = (rctl & ADI_SMPU_RCTL_SIZE_MASK) >>
					ADI_SMPU_RCTL_SIZE_SHIFT;
			size_t size = 1UL << (size_bits + 12);
			phys_addr_t base = (phys_addr_t)raddr << 12;
			char perms[4] = "---";

			if (rctl & ADI_SMPU_RCTL_RPROTEN)
				perms[0] = 'R';
			if (rctl & ADI_SMPU_RCTL_WPROTEN)
				perms[1] = 'W';

			seq_printf(s, "%-6d %-10s 0x%08llx 0x%-8zx %-6s 0x%04x 0x%04x",
				   i, "yes", (u64)base, size, perms,
				   rida & 0x1FFF, ridb & 0x1FFF);

			/* Show secure control for this region (if available) */
			if (inst->smpu->secure_regs_available) {
				u32 securerctl;
				int ret = adi_smpu_read_secure_reg(inst, &securerctl,
								    ADI_SMPU_SECURERCTL(i));
				if (ret == 0) {
					seq_printf(s, " | Sec:%c%c NS:%c%c",
						   (securerctl & ADI_SMPU_SECURERCTL_RSECDIS) ? '-' : 'R',
						   (securerctl & ADI_SMPU_SECURERCTL_WSECDIS) ? '-' : 'W',
						   (securerctl & ADI_SMPU_SECURERCTL_RNSEN) ? 'R' : '-',
						   (securerctl & ADI_SMPU_SECURERCTL_WNSEN) ? 'W' : '-');
				}
			}
			seq_puts(s, "\n");
		} else {
			seq_printf(s, "%-6d %-10s\n", i, "no");
		}
	}

	spin_unlock_irqrestore(&inst->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_regions);

static int smpu_violations_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;
	unsigned long flags;
	unsigned int i, count, start;

	seq_printf(s, "Total violations: %lu\n\n", inst->viol_count);

	if (inst->viol_count == 0) {
		seq_puts(s, "No violations recorded\n");
		return 0;
	}

	seq_printf(s, "%-10s %-12s %-8s %-6s %-8s\n",
		   "Time", "Address", "ID", "Type", "Secure");
	seq_puts(s, "---------------------------------------------------\n");

	spin_lock_irqsave(&inst->lock, flags);

	count = min(inst->viol_count, (unsigned long)ADI_SMPU_VIOL_HISTORY_SIZE);
	if (inst->viol_count > ADI_SMPU_VIOL_HISTORY_SIZE)
		start = inst->viol_head % ADI_SMPU_VIOL_HISTORY_SIZE;
	else
		start = 0;

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % ADI_SMPU_VIOL_HISTORY_SIZE;
		struct adi_smpu_violation *viol = &inst->violations[idx];
		unsigned long age = jiffies - viol->timestamp;

		seq_printf(s, "%-10lu 0x%08x 0x%04x %-6s %-8s\n",
			   age / HZ, viol->addr, viol->id,
			   viol->is_write ? "write" : "read",
			   viol->is_secure ? "yes" : "no");
	}

	spin_unlock_irqrestore(&inst->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_violations);

static int smpu_stats_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;

	seq_printf(s, "Total violations: %lu\n", inst->viol_count);
	seq_printf(s, "History size:     %u\n", ADI_SMPU_VIOL_HISTORY_SIZE);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_stats);

static int smpu_instances_show(struct seq_file *s, void *data)
{
	struct adi_smpu *smpu = s->private;
	int i;

	seq_printf(s, "Total instances: %d\n\n", smpu->num_instances);
	seq_printf(s, "%-10s %-12s %-10s\n", "Name", "Base", "ID");
	seq_puts(s, "------------------------------------\n");

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		seq_printf(s, "%-10s 0x%08lx %-10d\n",
			   inst->name, (unsigned long)inst->base,
			   inst->instance_id);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_instances);

static int smpu_summary_show(struct seq_file *s, void *data)
{
	struct adi_smpu *smpu = s->private;
	unsigned long total_violations = 0;
	int i;

	seq_printf(s, "ADSP-SC59x System Memory Protection Unit\n");
	seq_printf(s, "Instances: %d\n", smpu->num_instances);
	seq_printf(s, "Secure registers: %s\n",
		   smpu->secure_regs_available ? "available" : "unavailable (Errata, 20000003)");
	seq_puts(s, "\n");

	for (i = 0; i < smpu->num_instances; i++)
		total_violations += smpu->instances[i].viol_count;

	seq_printf(s, "Total violations: %lu\n", total_violations);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_summary);

static void adi_smpu_debugfs_init(struct adi_smpu *smpu)
{
	struct dentry *root;
	int i;

	root = debugfs_create_dir("smpu", NULL);
	smpu->debugfs_root = root;

	/* Everybody can read, nobody can write */
	debugfs_create_file("instances", 0444, root, smpu,
			    &smpu_instances_fops);
	debugfs_create_file("summary", 0444, root, smpu,
			    &smpu_summary_fops);

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		struct dentry *inst_dir;

		inst_dir = debugfs_create_dir(inst->name, root);
		inst->debugfs_dir = inst_dir;

		debugfs_create_file("status", 0444, inst_dir, inst,
				    &smpu_status_fops);
		debugfs_create_file("regions", 0444, inst_dir, inst,
				    &smpu_regions_fops);
		debugfs_create_file("violations", 0444, inst_dir, inst,
				    &smpu_violations_fops);
		debugfs_create_file("stats", 0444, inst_dir, inst,
				    &smpu_stats_fops);
	}
}

/* Static region parsing from device tree */

static int adi_smpu_parse_static_regions(struct adi_smpu *smpu)
{
	struct device *dev = smpu->dev;
	struct device_node *np = dev->of_node;
	struct device_node *regions_node, *child;
	int region_count = 0;

	/* Look for "static-regions" child node */
	regions_node = of_get_child_by_name(np, "static-regions");
	if (!regions_node) {
		dev_dbg(dev, "No static-regions defined in device tree\n");
		return 0;  /* static regions are optional so no error */
	}

	dev_info(dev, "Parsing static protection regions from device tree...\n");

	/* Iterate through each region definition */
	for_each_child_of_node(regions_node, child) {
		struct adi_smpu_region_config config = {0};
		struct device_node *mem_node;
		struct resource res;
		u32 instance, permissions;
		int region_num, ret;
		u32 allowed_ids[2] = {0};
		u32 id_masks[2] = {0};
		int id_count;

		/* Parse instance ID */
		ret = of_property_read_u32(child, "instance", &instance);
		if (ret) {
			dev_err(dev, "Missing 'instance' property in region '%s'\n",
				child->name);
			of_node_put(child);
			of_node_put(regions_node);
			return ret;
		}

		/* Parse memory-region phandle to get base and size */
		mem_node = of_parse_phandle(child, "memory-region", 0);
		if (mem_node) {
			ret = of_address_to_resource(mem_node, 0, &res);
			of_node_put(mem_node);
			if (ret) {
				dev_err(dev, "Failed to parse memory-region for '%s'\n",
					child->name);
				of_node_put(child);
				of_node_put(regions_node);
				return ret;
			}
			config.base = res.start;
			config.size = resource_size(&res);
		} else {
			/* Fallback: direct base/size properties */
			u64 base, size;

			ret = of_property_read_u64(child, "base", &base);
			if (ret) {
				dev_err(dev, "Missing 'base' or 'memory-region' in '%s'\n",
					child->name);
				of_node_put(child);
				of_node_put(regions_node);
				return ret;
			}

			ret = of_property_read_u64(child, "size", &size);
			if (ret) {
				dev_err(dev, "Missing 'size' in '%s'\n", child->name);
				of_node_put(child);
				of_node_put(regions_node);
				return ret;
			}

			config.base = base;
			config.size = size;
		}

		/* Parse permissions (default to RW if not specified) */
		ret = of_property_read_u32(child, "permissions", &permissions);
		if (ret)
			permissions = ADI_SMPU_PERM_RW;
		config.permissions = permissions;

		/* Parse allowed transaction IDs (max 2) */
		id_count = of_property_count_u32_elems(child, "allowed-ids");
		if (id_count > 0) {
			if (id_count > 2)
				id_count = 2;
			of_property_read_u32_array(child, "allowed-ids", allowed_ids, id_count);
			config.allowed_ids[0] = (u16)allowed_ids[0];
			if (id_count > 1)
				config.allowed_ids[1] = (u16)allowed_ids[1];
		}

		/* Parse ID masks (default to exact match(0x1FFF)) */
		id_count = of_property_count_u32_elems(child, "id-masks");
		if (id_count > 0) {
			if (id_count > 2)
				id_count = 2;
			of_property_read_u32_array(child, "id-masks", id_masks, id_count);
			config.id_masks[0] = (u16)id_masks[0];
			if (id_count > 1)
				config.id_masks[1] = (u16)id_masks[1];
		} else {
			/* Default to exact match */
			config.id_masks[0] = 0x1FFF;
			config.id_masks[1] = 0x1FFF;
		}

		/* Parse ID inversion (default to false(no inversion)) */
		config.id_invert[0] = of_property_read_bool(child, "id-invert-a");
		config.id_invert[1] = of_property_read_bool(child, "id-invert-b");

		/* Parse secure mode */
		if (of_property_read_bool(child, "secure-only")) {
			config.secure_mode = ADI_SMPU_SEC_SECURE_ONLY;
		} else if (of_property_read_bool(child, "non-secure-only")) {
			config.secure_mode = ADI_SMPU_SEC_NONSECURE_ONLY;
		} else {
			config.secure_mode = ADI_SMPU_SEC_BOTH;  /* Default */
		}

		/* Allocate a region for this configuration */
		region_num = adi_smpu_allocate_region(smpu, instance);
		if (region_num < 0) {
			dev_err(dev, "Failed to allocate region for '%s' on SMPU%d: %d\n",
				child->name, instance, region_num);
			of_node_put(child);
			of_node_put(regions_node);
			return region_num;
		}

		/* Configure the region */
		ret = adi_smpu_configure_region(smpu, instance, region_num, &config);
		if (ret) {
			dev_err(dev, "Failed to configure region %d on SMPU%d: %d\n",
				region_num, instance, ret);
			adi_smpu_free_region(smpu, instance, region_num);
			of_node_put(child);
			of_node_put(regions_node);
			return ret;
		}

		/* Enable the region */
		ret = adi_smpu_enable_region(smpu, instance, region_num);
		if (ret) {
			dev_err(dev, "Failed to enable region %d on SMPU%d: %d\n",
				region_num, instance, ret);
			adi_smpu_free_region(smpu, instance, region_num);
			of_node_put(child);
			of_node_put(regions_node);
			return ret;
		}

		dev_info(dev, "Static region %d: SMPU%d [0x%llx-0x%llx] perm=0x%x ids=[0x%03x,0x%03x]\n",
			 region_num, instance,
			 (unsigned long long)config.base,
			 (unsigned long long)(config.base + config.size - 1),
			 config.permissions,
			 config.allowed_ids[0], config.allowed_ids[1]);

		region_count++;
	}

	of_node_put(regions_node);

	if (region_count > 0)
		dev_info(dev, "Configured %d static protection regions\n", region_count);

	return 0;
}

/* Platform driver */

static int adi_smpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_smpu *smpu;
	int ret, irq, i;

	smpu = devm_kzalloc(dev, sizeof(*smpu), GFP_KERNEL);
	if (!smpu)
		return -ENOMEM;

	smpu->dev = dev;
	dev_set_drvdata(dev, smpu);

	/* Check if secure registers are accessible (Errata, 20000003) */
	smpu->secure_regs_available =
		of_property_read_bool(np, "adi,secure-access");

	/* Parse and map all SMPU instances from device tree */
	for (i = 0; i < ADI_SMPU_MAX_INSTANCES; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		struct resource *res;
		const char *name;
		void __iomem *base;
		int instance_id;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;

		ret = of_property_read_string_index(np, "reg-names", i, &name);
		if (ret) {
			dev_err(dev, "Missing reg-names entry %d\n", i);
			return ret;
		}

		/* Extract instance ID from name (like, "smpu0" -> 0) */
		ret = sscanf(name, "smpu%d", &instance_id);
		if (ret != 1) {
			dev_err(dev, "Invalid reg-name: %s\n", name);
			return -EINVAL;
		}

		base = devm_ioremap(dev, res->start, resource_size(res));
		if (!base) {
			dev_err(dev, "Failed to map %s\n", name);
			return -ENOMEM;
		}

		inst->base = base;
		inst->instance_id = instance_id;
		inst->name = devm_kasprintf(dev, GFP_KERNEL, "smpu%d", instance_id);
		if (!inst->name)
			return -ENOMEM;
		inst->smpu = smpu;
		inst->region_bitmap = 0;
		inst->viol_head = 0;
		inst->viol_count = 0;
		spin_lock_init(&inst->lock);

		dev_dbg(dev, "Mapped %s at 0x%08lx\n", inst->name,
			(unsigned long)inst->base);
	}

	smpu->num_instances = i;

	if (smpu->num_instances == 0) {
		dev_err(dev, "No SMPU instances found\n");
		return -ENODEV;
	}

	/* Reset all instances to safe state */
	for (i = 0; i < smpu->num_instances; i++)
		adi_smpu_reset_instance(&smpu->instances[i]);

	/* Configure global secure control if available */
	if (smpu->secure_regs_available) {
		for (i = 0; i < smpu->num_instances; i++) {
			struct adi_smpu_instance *inst = &smpu->instances[i];
			u32 securectl;

			/* Enable both secure and non-secure by default, 
			secure is enabled as default already */
			securectl = ADI_SMPU_SECURECTL_RNSEN |
				    ADI_SMPU_SECURECTL_WNSEN |
				    ADI_SMPU_SECURECTL_SINTEN;

			ret = adi_smpu_write_secure_reg(inst, securectl,
							 ADI_SMPU_REG_SECURECTL);
			if (ret) {
				dev_warn(dev, "%s: Failed to init SECURECTL: %d\n",
					 inst->name, ret);
				smpu->secure_regs_available = false;
				break;
			}

			dev_dbg(dev, "%s: SECURECTL initialized (0x%x)\n",
				inst->name, securectl);
		}
	}

	/* Request IRQ for violations (shared across all instances) */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;

	if (irq > 0) {
		ret = devm_request_irq(dev, irq, adi_smpu_irq_handler,
				       IRQF_SHARED, dev_name(dev), smpu);
		if (ret) {
			dev_err(dev, "Failed to request IRQ %d: %d\n", irq, ret);
			return ret;
		}

		for (i = 0; i < smpu->num_instances; i++) {
			struct adi_smpu_instance *inst = &smpu->instances[i];
			u32 val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);

			val |= ADI_SMPU_CTL_PINTEN;
			adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);
		}

		dev_info(dev, "Registered IRQ %d for violations\n", irq);
	} else {
		dev_warn(dev, "No IRQ specified, violations will not be reported\n");
	}

	/* Parse and configure static regions from device tree */
	ret = adi_smpu_parse_static_regions(smpu);
	if (ret) {
		dev_err(dev, "Failed to parse static regions: %d\n", ret);
		return ret;
	}

	/* Create debugfs interface */
	adi_smpu_debugfs_init(smpu);

	dev_info(dev, "Initialized %d SMPU instances\n", smpu->num_instances);

	if (!smpu->secure_regs_available)
		dev_warn(dev, "Security registers unavailable (Errata, 20000003)\n");

	return 0;
}

static void adi_smpu_remove(struct platform_device *pdev)
{
	struct adi_smpu *smpu = dev_get_drvdata(&pdev->dev);

	debugfs_remove_recursive(smpu->debugfs_root);
}

static const struct of_device_id adi_smpu_match[] = {
	{ .compatible = "adi,sc59x-smpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_smpu_match);

static struct platform_driver adi_smpu_driver = {
	.probe = adi_smpu_probe,
	.remove = adi_smpu_remove,
	.driver = {
		.name = "adi-sc59x-smpu",
		.of_match_table = of_match_ptr(adi_smpu_match)
	},
};

module_platform_driver(adi_smpu_driver);

MODULE_DESCRIPTION("System Memory Protection Unit for ADI SC59x SoCs");
MODULE_AUTHOR("OZAN DURGUT");
MODULE_LICENSE("GPL");
