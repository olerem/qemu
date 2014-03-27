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

#define I6300ESB_DEBUG 1

#ifdef I6300ESB_DEBUG
#define au6601_debug(fs,...) \
    fprintf(stderr,"au6601: %s: "fs,__func__,##__VA_ARGS__)
#else
#define au6601_debug(fs,...)
#endif

/* Device state. */
struct au6601State {
    PCIDevice dev;
    MemoryRegion io_mem;

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

static uint32_t au6601_mem_readb(void *vp, hwaddr addr)
{
    au6601_debug ("addr = %x\n", (int) addr);

    return 0;
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
    au6601_debug("addr = %x\n", (int) addr);

    return 0;
}

static void au6601_mem_writeb(void *vp, hwaddr addr, uint32_t val)
{
//    au6601State *d = vp;

    au6601_debug("addr = %x, val = %x\n", (int) addr, val);
}

static void au6601_mem_writew(void *vp, hwaddr addr, uint32_t val)
{
//    au6601State *d = vp;

    au6601_debug("addr = %x, val = %x\n", (int) addr, val);
}

static void au6601_mem_writel(void *vp, hwaddr addr, uint32_t val)
{
//    au6601State *d = vp;

    au6601_debug ("addr = %x, val = %x\n", (int) addr, val);
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

static int au6601_init(PCIDevice *dev)
{
    au6601State *d = DO_UPCAST(au6601State, dev, dev);

    au6601_debug("au6601State = %p\n", d);

    memory_region_init_io(&d->io_mem, OBJECT(d), &au6601_ops, d,
                          "au6601", 0x100);
    pci_register_bar(&d->dev, 0, 0, &d->io_mem);
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
    k->class_id = PCI_CLASS_SYSTEM_OTHER;
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
