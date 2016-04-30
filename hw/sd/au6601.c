/*
 * Virtual hardware SD/MMC host cotroller for Alcor Micro au6601
 *
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <inttypes.h>

#include "qemu-common.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include <time.h>
#include "sysemu/blockdev.h"
#include "hw/sd.h"

static FILE *debugfp = NULL;

#define AU6601_ENABLE_DEBUG 1

#ifdef AU6601_ENABLE_DEBUG
#define au6601_debug(fmt,...)						   \
	do {								   \
		if (!debugfp)						   \
			debugfp = fopen("/tmp/qemu_hw_pci.log", "a+");	   \
		if (debugfp)						   \
			fprintf(debugfp, "%ld %s: " fmt, (long)time(NULL), \
				__func__ , __VA_ARGS__);		   \
	} while (0)
#else
#define au6601_debug(fs,...)
#endif

/* SDMA phy address. Higer then 0x0800.0000? */
#define AU6601_REG_SDMA_ADDR	0x00
 #define AU6601_SDMA_MASK	0xfffff000
/* ADMA block count? AU6621 only. */
#define REG_05	0x05
/* PIO */
#define AU6601_REG_BUFFER	0x08
/* ADMA ctrl? AU6621 only. */
#define REG_0C	0x0c
/* ADMA phy address. AU6621 only. */
#define REG_10	0x10
/* CMD index */
#define AU6601_REG_CMD_OPCODE	0x23
/* CMD parametr */
#define AU6601_REG_CMD_ARG	0x24
/* CMD response 4x4 Bytes */
#define AU6601_REG_CMD_RSP0	0x30
#define AU6601_REG_CMD_RSP1	0x34
#define AU6601_REG_CMD_RSP2	0x38
#define AU6601_REG_CMD_RSP3	0x3C
/* LED ctrl? */
#define REG_51	0x51
/* ??? */
#define REG_52	0x52
/* LED related? Always toggled BIT0 */
#define REG_61	0x61
/* Same as REG_61? */
#define REG_63	0x63
/* ??? */
#define REG_69	0x69
/* Block size for SDMA or PIO */
#define AU6601_REG_BLOCK_SIZE	0x6c
/* Some power related reg, used together with REG_7A */
#define REG_70	0x70
/* PLL ctrl */
#define AU6601_REG_PLL_CTRL	0x72
 #define AU6601_PLL_DIV8_MASK	0xff
 #define AU6601_PLL_DIV4_MASK	0xf
 #define AU6601_PLL_DIV_S	8
 #define AU6601_PLL_MOD4	0xb	/* x 13,5 */
 #define AU6601_PLL_MOD3	0x3	/* x 12,5 */
 #define AU6601_PLL_MOD2	0x2	/* x 4	  */
 #define AU6601_PLL_MOD1	0x1	/* x 1,5  */
 #define AU6601_PLL_MOD0	0x0	/* x 1	  */
 #define AU6601_PLL_MOD_S	4
 #define AU6601_PLL_EN		BIT(0)

/* ??? */
#define REG_74	0x74
/* ??? */
#define REG_75	0x75
/* card slot state? */
#define REG_76	0x76
/* ??? */
#define REG_77	0x77
/* looks like soft reset? */
#define AU6601_REG_SW_RESET	0x79
 #define AU6601_RESET_UNK	BIT(7)	/* unknown bit */
 #define AU6601_RESET_DATA	BIT(3)
 #define AU6601_RESET_CMD	BIT(0)
/* see REG_70 */
#define REG_7A	0x7a
/* ??? Padding? Timeing? */
#define REG_7B	0x7b
/* ??? Padding? Timeing? */
#define REG_7C	0x7c
/* ??? Padding? Timeing? */
#define REG_7D	0x7d
/* read EEPROM? */
#define REG_7F	0x7f

#define AU6601_REG_CMD_CTRL	0x81
#define AU6601_REG_BUS_CTRL	0x82
 #define AU6601_BUS_WIDTH_4BIT	BIT(5)
#define REG_83	0x83

#define AU6601_REG_BUS_STATUS	0x84
 #define AU6601_BUS_STAT_CMD	BIT(15)
/* BIT(4) - BIT(7) are permanently 1.
 *  * May be reseved or not attached DAT4-DAT7 */
 #define AU6601_BUS_STAT_DAT3		BIT(3)
 #define AU6601_BUS_STAT_DAT2		BIT(2)
 #define AU6601_BUS_STAT_DAT1		BIT(1)
 #define AU6601_BUS_STAT_DAT0		BIT(0)
 #define AU6601_BUS_STAT_DAT_MASK	0xf

#define REG_85	0x85
 #define AU6601_REG_85_CLK_OFF		BIT(2)
 #define AU6601_REG_85_CLK_DIV2		BIT(1)
 #define AU6601_REG_85_VDD_180		BIT(0)
/* ??? */
#define REG_86	0x86
#define AU6601_REG_INT_STATUS	0x90 /* IRQ intmask */
#define AU6601_REG_INT_ENABLE	0x94
/* ??? */
#define REG_A1	0xa1
/* ??? */
#define REG_A2	0xa2
/* ??? */
#define REG_A3	0xa3
/* ??? */
#define REG_B0	0xb0
/* ??? */
#define REG_B4	0xb4

 /* AU6601_REG_INT_STATUS is identical or almost identical with sdhci.h */
 /* OK - are tested and confirmed bits */
 #define  AU6601_INT_RESPONSE		0x00000001	/* ok */
 #define  AU6601_INT_DATA_END		0x00000002	/* fifo, ok */
 #define  AU6601_INT_BLK_GAP		0x00000004
 #define  AU6601_INT_DMA_END		0x00000008
 #define  AU6601_INT_SPACE_AVAIL	0x00000010	/* fifo, ok */
 #define  AU6601_INT_DATA_AVAIL		0x00000020	/* fifo, ok */
 #define  AU6601_INT_CARD_REMOVE	0x00000040
 #define  AU6601_INT_CARD_INSERT	0x00000080	/* 0x40 and 0x80 flip */
 #define  AU6601_INT_CARD_INT		0x00000100
 #define  AU6601_INT_ERROR		0x00008000	/* ok */
 #define  AU6601_INT_TIMEOUT		0x00010000	/* seems to be ok */
 #define  AU6601_INT_CRC		0x00020000	/* seems to be ok */
 #define  AU6601_INT_END_BIT		0x00040000
 #define  AU6601_INT_INDEX		0x00080000
 #define  AU6601_INT_DATA_TIMEOUT	0x00100000
 #define  AU6601_INT_DATA_CRC		0x00200000
 #define  AU6601_INT_DATA_END_BIT	0x00400000
 #define  AU6601_INT_BUS_POWER		0x00800000
 #define  AU6601_INT_ACMD12ERR		0x01000000
 #define  AU6601_INT_ADMA_ERROR		0x02000000

 #define  AU6601_INT_NORMAL_MASK	0x00007FFF
 #define  AU6601_INT_ERROR_MASK		0xFFFF8000

/* magic 0xF0001 */
 #define  AU6601_INT_CMD_MASK	(AU6601_INT_RESPONSE | AU6601_INT_TIMEOUT | \
				AU6601_INT_CRC | AU6601_INT_END_BIT | AU6601_INT_INDEX)
 /* magic 0x70003A */
 #define  AU6601_INT_DATA_MASK	(AU6601_INT_DATA_END | AU6601_INT_DMA_END | \
				AU6601_INT_DATA_AVAIL | AU6601_INT_SPACE_AVAIL | \
				AU6601_INT_DATA_TIMEOUT | AU6601_INT_DATA_CRC | \
				AU6601_INT_DATA_END_BIT)

//#define AU6601_INT_ALL_MASK	((uint32_t)-1)

#define AU6601_RSP_CTRL_MASK 0xC0
#define AU6601_RSP_CTRL_R1   0x40
#define AU6601_RSP_CTRL_R2   0xC0
#define AU6601_RSP_CTRL_R3   0x80

#define AU6601_FIFO_LEN 0x200
#define AU6601_FIFO_FLAG_READ_START 0x01

/* Device state. */
struct au6601State {
    PCIDevice dev;
    SDState *card;
    MemoryRegion io_mem;

    uint8_t  cmd;
    uint32_t cmdarg;

    uint32_t response[4];

    unsigned int card_is_on;
    int trig;
    uint32_t reg_30;

    uint8_t reg_76;
    uint8_t reg_7f;
    uint8_t reg_81;
    uint8_t reg_83;
    uint32_t reg_90;

    uint32_t datacnt;
    uint32_t fifocnt;
    int32_t fifo_pos;
    int32_t fifo_len;
    uint32_t fifo[AU6601_FIFO_LEN];
    unsigned int fifo_flags;

    QEMUTimer *irq_timer;
    int irq_timer_enabled;
};

typedef struct au6601State au6601State;

static void au6601_irq_pulse(au6601State *d)
{
    if(d->reg_90) {
        au6601_debug("trigger irq (%x)\n", d->reg_90);
        pci_irq_pulse(&d->dev);
    }
}

static void au6601_irq_timer_expired(void *vp)
{
    au6601State *d = vp;
    au6601_irq_pulse(d);
    d->irq_timer_enabled = 0;
}

static void au6601_irq_delay(au6601State *d)
{
    if (d->irq_timer_enabled == 1)
        return;
    d->irq_timer_enabled = 1;
    timer_mod(d->irq_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

static void au6601_fifo_push(au6601State *d, uint32_t value)
{
    int n;

    if (d->fifo_len == AU6601_FIFO_LEN) {
        au6601_debug("FIFO overflowi %08x\n", (int)value);
        return;
    }
    n = (d->fifo_pos + d->fifo_len) & (AU6601_FIFO_LEN - 1);
    d->fifo_len++;
    d->fifo[n] = value;
    au6601_debug("FIFO push %08x\n", (int)value);
}

static uint32_t au6601_fifo_pop(au6601State *d)
{
    uint32_t value = 0;

    if (d->fifo_len == 0) {
        au6601_debug("FIFO underflow %x\n", value);
        return 0;
    }
    value = d->fifo[d->fifo_pos];
    d->fifo_len--;
    d->fifo_pos = (d->fifo_pos + 1) & (AU6601_FIFO_LEN - 1);
    au6601_debug("FIFO pop %08x\n", (int)value);
    return value;
}

static void au6601_fifo_run(au6601State *d)
{
    uint32_t value = 0;
    int n;
    int is_read;

    is_read = d->reg_83 & 0x1;
    au6601_debug("read %d\n", is_read);
    if (d->datacnt != 0 && (!is_read || sd_data_ready(d->card))) {
        if (is_read) {
            au6601_debug("is_read %p\n", d);
            d->reg_90 |= 0x20;
            d->fifo_flags |= AU6601_FIFO_FLAG_READ_START;
            n = 0;
            while (d->datacnt && d->fifo_len < AU6601_FIFO_LEN) {
                value |= (uint32_t)sd_read_data(d->card) << (n * 8);
                d->datacnt--;
                n++;
                if (n == 4) {
                    au6601_fifo_push(d, value);
                    n = 0;
                    value = 0;
                }
            }
            if (n != 0) {
                au6601_fifo_push(d, value);
            }
        } else { /* write */
            au6601_debug("is_write %p\n", d);
            n = 0;
            while (d->datacnt > 0 && (d->fifo_len > 0 || n > 0)) {
                if (n == 0) {
                    value = au6601_fifo_pop(d);
                    n = 4;
                }
                n--;
                d->datacnt--;
                sd_write_data(d->card, value & 0xff);
                value >>= 8;
            }
        }
    }
}

static void au6601_send_command(au6601State *d)
{
    SDRequest request;
    uint32_t response[4];
    int rlen;

    request.cmd = d->cmd & 0x3f;
    request.arg = cpu_to_be32(d->cmdarg);

    rlen = sd_do_command(d->card, &request, (uint8_t *)response);
    if (rlen < 0) {
        au6601_debug("%i SD: Timeout\n", __LINE__);
        d->reg_90 = 0x18000;
        return;
    }

    if (1) {
        au6601_debug("sd res len = %d; %x\n", rlen, d->reg_81);
        if (rlen == 0 && d->reg_81 & AU6601_RSP_CTRL_MASK) {
            d->reg_90 = 0x18000;
            return;
	}
#if 0
        if (rlen != 4 && rlen != 16)
            goto error;
#endif

        d->response[0] = cpu_to_le32(response[0]);
        if (rlen == 4) {
            d->response[1] = d->response[2] = d->response[3] = 0;
        } else {
            d->response[1] = cpu_to_le32(response[1]);
            d->response[2] = cpu_to_le32(response[2]);
            d->response[3] = cpu_to_le32(response[3]);
        }
        d->reg_90 = 0x1;
    }
}

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
    au6601State *d = vp;
    uint8_t val;

    switch (addr) {
    case REG_70:
    case REG_7A:
      val = 0xff;
      break;
    case REG_76:
      val = d->reg_76;
      if (d->card_is_on)
        val |= 0x1;
      break;
    case REG_77:
      val = 0x70;
      break;
    case AU6601_REG_SW_RESET:
      /* TODO: need to return some thing? */
      val = 0;
      break;
    case REG_7F:
      val = d->reg_7f;
      break;
    case REG_86:
      val = 0xff;
      break;
    case AU6601_REG_INT_STATUS:
      val = d->reg_90;
      break;
    default:
      au6601_debug(":%i unknown offset\n", __LINE__);
      val = 0;
    }

    au6601_debug("addr = %x, val = %x\n", (int) addr, val);
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
    case AU6601_REG_BUFFER:
      val = au6601_fifo_pop(d);
      au6601_fifo_run(d);
      d->fifocnt -= 4;
      au6601_debug("fifocnt = %d\n", (int) d->fifocnt);
      if (d->fifocnt == 0) {
          //d->reg_83 &= ~0x1;
          d->reg_90 = 0x2;
      }
      au6601_irq_delay(d);
      break;
    case AU6601_REG_CMD_RSP0:
      val = d->response[0];
      break;
    case AU6601_REG_CMD_RSP1:
      val = d->response[1];
      break;
    case AU6601_REG_CMD_RSP2:
      val = d->response[2];
      break;
    case AU6601_REG_CMD_RSP3:
      val = d->response[3];
      break;
    case AU6601_REG_INT_STATUS:
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
    case AU6601_REG_CMD_OPCODE: /* Command */
      d->cmd = val;
#if 1
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
#endif
      break;
    case REG_76:
      d->reg_76 = val;
      break;
    case REG_7F:
      if (val == 0)
        d->reg_7f = 0x66;
      else if (val == 1)
        d->reg_7f = 0x0a;
      break;
    case AU6601_REG_CMD_CTRL:
      d->reg_81 = val;
      /* FIXME: set command options */
      au6601_send_command(d);
      au6601_fifo_run(d);
      au6601_irq_pulse(d);
      break;
    case REG_83:
      d->reg_83 = val;
      if (val & 0x40) {
          au6601_fifo_run(d);
          au6601_irq_pulse(d);
      }
      break;
    case AU6601_REG_INT_STATUS:
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
    case AU6601_REG_CMD_ARG:
      d->cmdarg = val;
      break;
    case AU6601_REG_BLOCK_SIZE: /* FIFO datacount */
      d->datacnt = d->fifocnt = val;
      break;
    case AU6601_REG_INT_STATUS:
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
    DriveInfo *dinfo;
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

    dinfo = drive_get_next(IF_SD);
    d->card = sd_init(dinfo ? blk_by_legacy_dinfo(dinfo) : NULL, false);
    if (d->card == NULL) {
        return -1;
    }

    d->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, au6601_irq_timer_expired, d);

    return 0;
}

static void au6601_exit(PCIDevice *dev)
{
    au6601State *d = DO_UPCAST(au6601State, dev, dev);

    object_unparent(OBJECT(&d->io_mem));
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
