/*
 * Virtual hardware watchdog.
 *
 * Copyright (C) 2009 Red Hat Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * By Richard W.M. Jones (rjones@redhat.com).
 */

#include <inttypes.h>

#include "qemu-common.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/osdep.h"
#include <time.h>

#define I6300ESB_DEBUG 1

static FILE *debugfp = NULL;

#ifdef I6300ESB_DEBUG
#define au6601_debug(fmt,...) \
	do {			\
		if (!debugfp)	\
			debugfp = fopen("/tmp/qemu_hw_pci.log", "a+");	\
		if (debugfp)	\
    			fprintf(debugfp, "%ld %s: " fmt, (long)time(NULL),	\
				__func__ , __VA_ARGS__);			\
	} while (0)
#else
#define au6601_debug(fs,...)
#endif

/* Device state. */
struct au6601State {
    PCIDevice dev;
    MemoryRegion io_mem;

    unsigned int card_is_on;
    int trig;
    uint32_t reg_30;

    uint8_t reg_76;
    uint8_t reg_7f;
    uint32_t reg_90;
};

typedef struct au6601State au6601State;

static void au6601_reset(DeviceState *dev)
{
    PCIDevice *pdev = PCI_DEVICE(dev);
    au6601State *d = DO_UPCAST(au6601State, dev, pdev);

    au6601_debug("au6601State = %p\n", d);

}

static void au6601_config_write(PCIDevice *dev, uint32_t addr,
                                  uint32_t data, int len)
{
    //au6601State *d = DO_UPCAST(au6601State, dev, dev);

    au6601_debug("addr = %x, data = %x, len = %d\n", addr, data, len);

    pci_default_write_config(dev, addr, data, len);
}

static uint32_t au6601_config_read(PCIDevice *dev, uint32_t addr, int len)
{
//    au6601State *d = DO_UPCAST(au6601State, dev, dev);
//    uint32_t data;

    au6601_debug ("addr = %x, len = %d\n", addr, len);

    return pci_default_read_config(dev, addr, len);

}

#if 0
static void au6601_process_sd(au6601State *priv)
{
	
}
#endif

static uint32_t au6601_mem_readb(void *vp, hwaddr addr)
{
    au6601State *d = vp;
    uint8_t val;

    switch (addr) {
    case 0x70:
    case 0x7a:
      val = 0xff;
      break;
    case 0x76:
      val = d->reg_76;
      if (d->card_is_on)
        val |= 0x1;
      break;
    case 0x77:
      val = 0x70;
      break;
    case 0x7f:
      val = d->reg_7f;
      break;
    case 0x86:
      val = 0xff;
      break;
    case 0x90:
      val = d->reg_90;
      break;
    default:
      val = 0;
    }

    au6601_debug ("addr = %x, val = %x\n", (int) addr, val);
    return val;
}

static uint32_t au6601_mem_readw(void *vp, hwaddr addr)
{
    uint32_t data = 0;
  //  au6601State *d = vp;

    au6601_debug("addr = %x\n", (int) addr);

    return data;
}

static uint32_t au6601_mem_readl(void *vp, hwaddr addr)
{
    au6601State *d = vp;
    uint32_t val;

    switch (addr) {
    case 0x30:
      val = d->reg_30;
      break;
    case 0x90:
      val = d->reg_90;
      break;
    default:
      val = 0;
    }

    au6601_debug("addr = %x, val = %x\n", (int) addr, val);
    return val;
}

static void au6601_mem_writeb(void *vp, hwaddr addr, uint32_t val)
{
    au6601State *d = vp;

    au6601_debug("addr = %x, val = %x\n", (int) addr, val);
    switch (addr) {
    case 0x23:
      //val &= 0x1F;
      d->reg_90 = 0x1;
      if (val == 0x48)
        d->reg_30 = 0xaa010000;
      else if (val == 0x77) {
	if (d->trig == 1) {
	  d->reg_90 = 0x18000;
	  d->trig = 0;
	}
        d->reg_30 = 0x20010000;
      } else if (val == 0x69) {
	d->trig = 1;
        d->reg_30 = 0x0080ff00;
      } else if (val == 0x41) {
	 d->reg_90 = 0x18000;
      }

      break;
    case 0x76:
      d->reg_76 = val;
      break;
    case 0x7f:
      if(val == 0)
        d->reg_7f = 0x66;
      else if (val == 1)
        d->reg_7f = 0x0a;
      break;
    case 0x81:
      pci_irq_pulse(&d->dev);
      break;
    case 0x90:
      d->reg_90 = 0;
      break;
    default:
      break;
    }

}

static void au6601_mem_writew(void *vp, hwaddr addr, uint32_t val)
{
//    au6601State *d = vp;

    au6601_debug("addr = %x, val = %x\n", (int) addr, val);
}

static void au6601_mem_writel(void *vp, hwaddr addr, uint32_t val)
{
    au6601_debug ("addr = %x, val = %x\n", (int) addr, val);
    au6601State *d = vp;

    switch (addr) {
    case 0x90:
      d->reg_90 = 0;
      break;
    default:
      break;
    }
}

static const MemoryRegionOps au6601_ops = {
    .old_mmio = {
        .read = {
            au6601_mem_readb,
            au6601_mem_readw,
            au6601_mem_readl,
        },
        .write = {
            au6601_mem_writeb,
            au6601_mem_writew,
            au6601_mem_writel,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void au6601_init_priv(au6601State *priv)
{
	priv->card_is_on = 1;
}

#define AU6601_PM_OFFSET 0x80
#define AU6601_MSI_OFFSET 0x88
#define AU6601_PCIE_OFFSET 0x98

static int au6601_init(PCIDevice *dev)
{
    au6601State *d = DO_UPCAST(au6601State, dev, dev);
    uint8_t *pci_conf;

    au6601_debug("au6601State = %p\n", d);

    pci_conf = dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 1;    /* interrupt pin A */
    pci_add_capability(dev, PCI_CAP_ID_PM, AU6601_PM_OFFSET, PCI_PM_SIZEOF);
    pci_set_word(pci_conf + AU6601_PM_OFFSET + 2, 0x0003);

    pci_add_capability(dev, PCI_CAP_ID_MSI, AU6601_MSI_OFFSET, 16);
    pci_set_word(pci_conf + AU6601_MSI_OFFSET + 2, 0x0080);
   // assert(ret >= 0);
    /* FIXME 0x3c is size of PCI_EXP_VER2_SIZEOF, is it same for VER1? */
    pci_add_capability(dev, PCI_CAP_ID_EXP, AU6601_PCIE_OFFSET, 0x3c);
    pci_set_word(pci_conf + AU6601_PCIE_OFFSET + 2, 0x0001);
    pci_set_long(pci_conf + AU6601_PCIE_OFFSET + 4, 0x05908fc0);
    pci_set_long(pci_conf + 0xa0 + 0, 0x00002800);
    pci_set_long(pci_conf + 0xa0 + 4, 0x0103ec11);
    pci_set_long(pci_conf + 0xa0 + 8, 0x10110040);

    memory_region_init_io(&d->io_mem, OBJECT(d), &au6601_ops, d,
                          "au6601-io", 0x100);
    pci_register_bar(&d->dev, 0, 0, &d->io_mem);

    au6601_init_priv(d);
    pci_irq_deassert(dev);
    /* qemu_register_coalesced_mmio (addr, 0x10); ? */


    return 0;
}

static void au6601_exit(PCIDevice *dev)
{
    au6601State *d = DO_UPCAST(au6601State, dev, dev);

    memory_region_destroy(&d->io_mem);
}

static void au6601_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->config_read = au6601_config_read;
    k->config_write = au6601_config_write;
    k->init = au6601_init;
    k->exit = au6601_exit;
    k->vendor_id = 0x1aea;
    k->device_id = 0x6601;
    k->subsystem_vendor_id = 0x0001;
    k->subsystem_id = 0x0001;
    k->class_id = PCI_CLASS_OTHERS;
    dc->reset = au6601_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo au6601_info = {
    .name          = "au6601",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(au6601State),
    .class_init    = au6601_class_init,
};

static void au6601_register_types(void)
{
    type_register_static(&au6601_info);
}

type_init(au6601_register_types)
