// SPDX-License-Identifier: GPL-2.0
/*
 * pci-of-overlay: Generic PCI driver that applies a device-tree overlay.
 *
 * Loads on matching VID:DID, applies a firmware-provided DT overlay that
 * describes the peripherals behind the PCI endpoint (typically a
 * simple-bus with BAR-derived ranges), registers an irqdomain backed
 * by MSI-X/MSI (one hwirq per vector) or by shared INTx (single
 * hwirq 0 demuxed in leaf drivers), and lets stock subsystem drivers
 * probe the resulting platform devices.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/slab.h>

struct pci_of_overlay_info {
	const char *fw_name;
	u16 nvec_max;
	u8 dma_mask_bits;
};

struct pci_of_overlay {
	struct pci_dev *pdev;
	struct irq_domain *irq_domain;
	const struct pci_of_overlay_info *info;
	int ovcs_id;
	int intx_irq;
};

static char *overlay;
module_param(overlay, charp, 0644);
MODULE_PARM_DESC(overlay, "Override the DTB overlay");

static void pci_of_overlay_irq_mask(struct irq_data *d)
{
	irq_chip_mask_parent(d);
}

static void pci_of_overlay_irq_unmask(struct irq_data *d)
{
	irq_chip_unmask_parent(d);
}

static const struct irq_chip pci_of_overlay_irq_chip = {
	.name		= "pci-of-overlay",
	.irq_mask	= pci_of_overlay_irq_mask,
	.irq_unmask	= pci_of_overlay_irq_unmask,
	.irq_eoi	= irq_chip_eoi_parent,
};

static int pci_of_overlay_irq_alloc(struct irq_domain *d, unsigned int virq,
				    unsigned int nr_irqs, void *arg)
{
	struct pci_of_overlay *pov = d->host_data;
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hwirq;
	int parent_irq;

	if (nr_irqs != 1 || fwspec->param_count < 1)
		return -EINVAL;

	hwirq = fwspec->param[0];
	parent_irq = pci_irq_vector(pov->pdev, hwirq);
	if (parent_irq < 0)
		return parent_irq;

	irq_domain_set_info(d, virq, hwirq, &pci_of_overlay_irq_chip, pov,
			    handle_simple_irq, NULL, NULL);

	return irq_set_parent(virq, parent_irq);
}

static const struct irq_domain_ops pov_msi_irq_domain_ops = {
	.alloc	= pci_of_overlay_irq_alloc,
	.free	= irq_domain_free_irqs_common,
	.xlate	= irq_domain_xlate_onecell,
};

static int pci_of_overlay_intx_map(struct irq_domain *d, unsigned int virq,
				   irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &dummy_irq_chip, handle_simple_irq);
	return 0;
}

static const struct irq_domain_ops pov_intx_irq_domain_ops = {
	.map	= pci_of_overlay_intx_map,
	.xlate	= irq_domain_xlate_onecell,
};

static irqreturn_t pci_of_overlay_intx_handler(int irq, void *data)
{
	struct pci_of_overlay *pov = data;

	return generic_handle_domain_irq(pov->irq_domain, 0) ?
		IRQ_NONE : IRQ_HANDLED;
}

static void pci_of_overlay_free_intx_irq(void *data)
{
	struct pci_of_overlay *pov = data;

	free_irq(pov->intx_irq, pov);
}

static void pci_of_overlay_remove_irq_domain(void *data)
{
	irq_domain_remove(data);
}

static int pci_of_overlay_msi_irq_domain(struct pci_of_overlay *pov, int nvec)
{
	struct pci_dev *pdev = pov->pdev;
	struct device *dev = &pdev->dev;

	pov->irq_domain = irq_domain_create_linear(dev_fwnode(dev), nvec,
						   &pov_msi_irq_domain_ops,
						   pov);
	if (!pov->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create MSI irqdomain\n");

	return devm_add_action_or_reset(dev, pci_of_overlay_remove_irq_domain,
					pov->irq_domain);
}

static int pci_of_overlay_intx_irq_domain(struct pci_of_overlay *pov)
{
	struct pci_dev *pdev = pov->pdev;
	struct device *dev = &pdev->dev;
	int ret;

	pov->irq_domain = irq_domain_create_linear(dev_fwnode(dev), 1,
						   &pov_intx_irq_domain_ops,
						   pov);
	if (!pov->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create INTx irqdomain\n");

	ret = devm_add_action_or_reset(dev,
				       pci_of_overlay_remove_irq_domain,
				       pov->irq_domain);
	if (ret)
		return ret;

	pov->intx_irq = pci_irq_vector(pdev, 0);
	ret = request_irq(pov->intx_irq, pci_of_overlay_intx_handler,
			  IRQF_SHARED, dev_name(dev), pov);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request INTx irq %d\n",
				     pov->intx_irq);

	return devm_add_action_or_reset(dev, pci_of_overlay_free_intx_irq, pov);
}

static void pci_of_overlay_remove(void *data)
{
	of_overlay_remove(data);
}

static void pci_of_overlay_depopulate(void *data)
{
	of_platform_depopulate(data);
}

static const struct pci_of_overlay_info pov_default_info = {
	.nvec_max = 32,
	.dma_mask_bits = 64,
};

static int pci_of_overlay_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	const struct pci_of_overlay_info *info =
		(const struct pci_of_overlay_info *)id->driver_data;
	struct device *dev = &pdev->dev;
	struct pci_of_overlay *pov;
	const struct firmware *fw;
	const char *dtbo_name;
	int ret, nvec;

	if (!info)
		info = &pov_default_info;

	dtbo_name = overlay ?: info->fw_name;
	if (!dtbo_name)
		return -EINVAL;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	if (!dev_fwnode(dev))
		return -ENODEV;

	pci_set_master(pdev);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(info->dma_mask_bits));
	if (ret)
		return ret;

	pov = devm_kzalloc(dev, sizeof(*pov), GFP_KERNEL);
	if (!pov)
		return -ENOMEM;

	pov->pdev = pdev;
	pov->info = info;
	pci_set_drvdata(pdev, pov);

	nvec = pci_alloc_irq_vectors(pdev, 1, pov->info->nvec_max,
				     PCI_IRQ_ALL_TYPES);
	if (nvec < 0)
		return dev_err_probe(dev, nvec,
				     "IRQ vector allocation failed (max=%u)\n",
				     info->nvec_max);

	if (pci_dev_msi_enabled(pdev))
		ret = pci_of_overlay_msi_irq_domain(pov, nvec);
	else
		ret = pci_of_overlay_intx_irq_domain(pov);
	if (ret)
		return ret;

	ret = request_firmware(&fw, dtbo_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "missing overlay %s\n", dtbo_name);

	ret = of_overlay_fdt_apply(fw->data, fw->size, &pov->ovcs_id,
				   dev_of_node(dev));
	release_firmware(fw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to apply overlay\n");

	ret = devm_add_action_or_reset(dev, pci_of_overlay_remove,
				       &pov->ovcs_id);
	if (ret)
		return ret;

	ret = of_platform_default_populate(dev_of_node(dev), NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to populate platform devs\n");

	return devm_add_action_or_reset(dev, pci_of_overlay_depopulate, dev);
}

static const struct pci_device_id pci_of_overlay_ids[] = {
	{ }
};
MODULE_DEVICE_TABLE(pci, pci_of_overlay_ids);

static struct pci_driver pci_of_overlay_driver = {
	.name		= "pci-of-overlay",
	.id_table	= pci_of_overlay_ids,
	.probe		= pci_of_overlay_probe,
};
module_pci_driver(pci_of_overlay_driver);

MODULE_DESCRIPTION("Generic PCI OF overlay driver");
MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_LICENSE("GPL");