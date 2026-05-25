/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices SC59x System Memory Protection Unit
 *
 * Copyright (C) 2025 - Analog Devices, Inc.
 * Author: Ozan Durgutt <ozan.durgutt@analog.com>
 */

#ifndef SOC_ADI_SMPU_H
#define SOC_ADI_SMPU_H

#include <linux/device.h>
#include <linux/types.h>

/* SMPU instance IDs (sparse numbering: 0, 2-6, 9, 11-12) */
#define ADI_SMPU0		0
#define ADI_SMPU2		2
#define ADI_SMPU3		3
#define ADI_SMPU4		4
#define ADI_SMPU5		5
#define ADI_SMPU6		6
#define ADI_SMPU9		9
#define ADI_SMPU11		11
#define ADI_SMPU12		12

/* Number of regions per SMPU instance */
#define ADI_SMPU_NUM_REGIONS	8

/* Forward declaration */
struct adi_smpu;

/**
 * enum adi_smpu_secure_mode - Secure access modes (TrustZone)
 * @ADI_SMPU_SEC_BOTH: Allow both secure & non-secure transactions (default)
 * @ADI_SMPU_SEC_SECURE_ONLY: Allow only secure transactions
 * @ADI_SMPU_SEC_NONSECURE_ONLY: Allow only non-secure transactions
 * @ADI_SMPU_SEC_NONE: Block all transactions (for debugging)
 *
 * Only used if secure registers available (adi,secure-access property).
 */
enum adi_smpu_secure_mode {
	ADI_SMPU_SEC_BOTH = 0,
	ADI_SMPU_SEC_SECURE_ONLY,
	ADI_SMPU_SEC_NONSECURE_ONLY,
	ADI_SMPU_SEC_NONE,
};

/**
 * struct adi_smpu_region_config - SMPU region configuration
 * @base: Physical base address of the protected region (must be aligned to size)
 * @size: Size of the region (must be power of 2, minimum 4KB, maximum 4GB)
 * @permissions: Access permissions (ADI_SMPU_PERM_* flags)
 * @allowed_ids: Transaction IDs allowed to access (up to 2 ID ranges)
 * @id_masks: Masks for ID matching (0x0000 = match all, 0x1FFF = exact match)
 * @id_invert: Invert ID match logic (true = block specified IDs)
 * @secure_mode: Secure/non-secure access control (only if secure regs available)
 */
struct adi_smpu_region_config {
	phys_addr_t base;
	size_t size;
	u32 permissions;
	u16 allowed_ids[2];
	u16 id_masks[2];
	bool id_invert[2];
	enum adi_smpu_secure_mode secure_mode;
};

/* Permission flags */
#define ADI_SMPU_PERM_READ	BIT(0)
#define ADI_SMPU_PERM_WRITE	BIT(1)
#define ADI_SMPU_PERM_EXECUTE	BIT(2)
#define ADI_SMPU_PERM_RW	(ADI_SMPU_PERM_READ | ADI_SMPU_PERM_WRITE)
#define ADI_SMPU_PERM_RWX	(ADI_SMPU_PERM_RW | ADI_SMPU_PERM_EXECUTE)

/**
 * adi_smpu_get_from_node - Get SMPU driver instance from device tree
 * @dev: Device requesting SMPU access
 *
 * Parses "adi,smpu" phandle from device tree and returns the SMPU driver
 * instance. Call adi_smpu_put() when done.
 *
 * Return: Pointer to SMPU instance or ERR_PTR on error
 */
struct adi_smpu *adi_smpu_get_from_node(struct device *dev);

/**
 * adi_smpu_put - Release SMPU driver instance
 * @smpu: SMPU instance to release
 */
void adi_smpu_put(struct adi_smpu *smpu);

/**
 * adi_smpu_configure_region - Configure an SMPU protection region
 * @smpu: SMPU driver instance
 * @instance_id: SMPU instance ID (0, 2-6, 9, 11-12)
 * @region_num: Region number (0-7)
 * @config: Region configuration
 *
 * Configures the specified protection region. The region will be disabled
 * during configuration. Call adi_smpu_enable_region() to activate.
 *
 * Return: 0 on success, negative error code on failure
 */
int adi_smpu_configure_region(struct adi_smpu *smpu, int instance_id,
			       int region_num,
			       const struct adi_smpu_region_config *config);

/**
 * adi_smpu_enable_region - Enable an SMPU protection region
 * @smpu: SMPU driver instance
 * @instance_id: SMPU instance ID (0, 2-6, 9, 11-12)
 * @region_num: Region number (0-7)
 *
 * Enables a previously configured protection region. The region must be
 * configured via adi_smpu_configure_region() first.
 *
 * Return: 0 on success, negative error code on failure
 */
int adi_smpu_enable_region(struct adi_smpu *smpu, int instance_id,
			    int region_num);

/**
 * adi_smpu_disable_region - Disable an SMPU protection region
 * @smpu: SMPU driver instance
 * @instance_id: SMPU instance ID (0, 2-6, 9, 11-12)
 * @region_num: Region number (0-7)
 *
 * Disables an active protection region, removing all access restrictions.
 *
 * Return: 0 on success, negative error code on failure
 */
int adi_smpu_disable_region(struct adi_smpu *smpu, int instance_id,
			     int region_num);

/**
 * adi_smpu_lock_config - Lock SMPU configuration
 * @smpu: SMPU driver instance
 * @instance_id: SMPU instance ID (0, 2-6, 9, 11-12)
 *
 * Locks the SMPU configuration, preventing further changes until system reset.
 * This is a security feature to prevent malicious reconfiguration.
 *
 * WARNING: This is irreversible until system reset!
 *
 * Return: 0 on success, negative error code on failure
 */
int adi_smpu_lock_config(struct adi_smpu *smpu, int instance_id);

/**
 * adi_smpu_allocate_region - Allocate an unused SMPU region
 * @smpu: SMPU driver instance
 * @instance_id: SMPU instance ID (0, 2-6, 9, 11-12)
 *
 * Finds and allocates an unused region. The region is marked as in-use
 * but not configured or enabled. Call adi_smpu_configure_region() and
 * adi_smpu_enable_region() to activate.
 *
 * Return: Region number (0-7) on success, negative error code on failure
 */
int adi_smpu_allocate_region(struct adi_smpu *smpu, int instance_id);

/**
 * adi_smpu_free_region - Free an allocated SMPU region
 * @smpu: SMPU driver instance
 * @instance_id: SMPU instance ID (0, 2-6, 9, 11-12)
 * @region_num: Region number (0-7)
 *
 * Frees a previously allocated region, making it available for reuse.
 * The region will be disabled automatically.
 */
void adi_smpu_free_region(struct adi_smpu *smpu, int instance_id,
			  int region_num);

#endif /* SOC_ADI_SMPU_H */
