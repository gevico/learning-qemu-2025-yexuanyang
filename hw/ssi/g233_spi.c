/*
 * QEMU model of the G233 SPI Controller
 *
 * Copyright (c) 2025 Yexuan Yang, yyxrust@bupt.edu.cn
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/registerfields.h"
#include "hw/ssi/g233_spi.h"

REG32(CR1, 0x00)
    FIELD(CR1, RESERVED_FLD1, 7, 25)
    FIELD(CR1, SPE, 6, 1)
    FIELD(CR1, RESERVED_FLD2, 3, 3)
    FIELD(CR1, MSTR, 2, 1)
    FIELD(CR1, RESERVED_FLD3, 0, 2)
REG32(CR2, 0x04)
    FIELD(CR2, RESERVED_FLD1, 8, 24)
    FIELD(CR2, TXEIE, 7, 1)
    FIELD(CR2, RXNEIE, 6, 1)
    FIELD(CR2, ERRIE, 5, 1)
    FIELD(CR2, SSOE, 4, 1)
    FIELD(CR2, RESERVED_FLD2, 0, 4)
REG32(SR, 0x08)
    FIELD(SR, RESERVED_FLD1, 8, 24)
    FIELD(SR, BSY, 7, 1)
    FIELD(SR, RESERVED_FLD2, 4, 3)
    FIELD(SR, OVERRUN, 3, 1)
    FIELD(SR, UNDERRUN, 2, 1)
    FIELD(SR, TXE, 1, 1)
    FIELD(SR, RXNE, 0, 1)
REG32(DR, 0x0C)
    FIELD(DR, RESERVED_FLD1, 8, 24)
    FIELD(DR, DATA, 0, 8)
REG32(CSCTRL, 0x10)
    FIELD(CSCTRL, RESERVED_FLD2, 8, 24)
    FIELD(CSCTRL, CS3_ACT, 7, 1)
    FIELD(CSCTRL, CS2_ACT, 6, 1)
    FIELD(CSCTRL, CS1_ACT, 5, 1)
    FIELD(CSCTRL, CS0_ACT, 4, 1)
    FIELD(CSCTRL, CS3_EN, 3, 1)
    FIELD(CSCTRL, CS2_EN, 2, 1)
    FIELD(CSCTRL, CS1_EN, 1, 1)
    FIELD(CSCTRL, CS0_EN, 0, 1)

#define FIFO_CAPACITY   1

static void g233_spi_txfifo_reset(G233SPIState *s)
{
    /* Reset transmit FIFO */
    fifo8_reset(&s->tx_fifo);

    /* TX FIFO empty, set TXE flag */
    s->regs[R_SR] |= R_SR_TXE_MASK;
    
    /* Clear underrun error flag */
    s->regs[R_SR] &= ~R_SR_UNDERRUN_MASK;
}

static void g233_spi_rxfifo_reset(G233SPIState *s)
{
    /* Reset receive FIFO */
    fifo8_reset(&s->rx_fifo);

    /* RX FIFO empty, clear RXNE flag */
    s->regs[R_SR] &= ~R_SR_RXNE_MASK;
    
    /* Clear overrun error flag */
    s->regs[R_SR] &= ~R_SR_OVERRUN_MASK;
}

static void g233_spi_update_cs(G233SPIState *s)
{
    int i;
    uint32_t csctrl = s->regs[R_CSCTRL];

    /*
     * Bit 0-3: CS0_EN ~ CS3_EN - Chip select enable bits
     * Bit 4-7: CS0_ACT ~ CS3_ACT - Chip select active bits
     */
    for (i = 0; i < s->num_cs; i++) {
        bool cs_en = (csctrl >> i) & 0x1;
        bool cs_act = (csctrl >> (i + 4)) & 0x1;
        qemu_set_irq(s->cs_lines[i], !(cs_en && cs_act));
    }
}

static void g233_spi_update_irq(G233SPIState *s)
{
    uint32_t sr = s->regs[R_SR];
    uint32_t cr2 = s->regs[R_CR2];
    bool irq_level = false;

    /* Update interrupt status based on CR2 and SR registers
     * 
     * CR2 interrupt enable bits:
     *   Bit 7: TXEIE  - TX buffer empty interrupt enable
     *   Bit 6: RXNEIE - RX buffer not empty interrupt enable
     *   Bit 5: ERRIE  - Error interrupt enable
     * 
     * SR status bits:
     *   Bit 1: TXE      - TX buffer empty flag
     *   Bit 0: RXNE     - RX buffer not empty flag
     *   Bit 3: OVERRUN  - Overrun error
     *   Bit 2: UNDERRUN - Underrun error
     */

    /* TXE interrupt: TXEIE=1 and TXE=1 */
    if ((cr2 & R_CR2_TXEIE_MASK) && (sr & R_SR_TXE_MASK)) {
        irq_level = true;
    }

    /* RXNE interrupt: RXNEIE=1 and RXNE=1 */
    if ((cr2 & R_CR2_RXNEIE_MASK) && (sr & R_SR_RXNE_MASK)) {
        irq_level = true;
    }

    /* Error interrupt: ERRIE=1 and (OVERRUN=1 or UNDERRUN=1) */
    if ((cr2 & R_CR2_ERRIE_MASK) && 
        (sr & (R_SR_OVERRUN_MASK | R_SR_UNDERRUN_MASK))) {
        irq_level = true;
    }

    qemu_set_irq(s->irq, irq_level);
}

static void g233_spi_reset(DeviceState *dev)
{
    G233SPIState *s = G233_SPI(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[R_SR] = 0x02;
    s->regs[R_DR] = 0x0C;

    g233_spi_update_cs(s);
    g233_spi_update_irq(s);

    g233_spi_txfifo_reset(s);
    g233_spi_rxfifo_reset(s);
}

static void g233_spi_flush_txfifo(G233SPIState *s)
{
    uint8_t tx;
    uint8_t rx;

    /* Check if SPI is enabled (CR1.SPE) and in master mode (CR1.MSTR) */
    if (!(s->regs[R_CR1] & R_CR1_SPE_MASK) || 
        !(s->regs[R_CR1] & R_CR1_MSTR_MASK)) {
        return;
    }

    /* Set busy flag */
    s->regs[R_SR] |= R_SR_BSY_MASK;

    /* Transfer all data from TX FIFO through SPI bus */
    while (!fifo8_is_empty(&s->tx_fifo)) {
        tx = fifo8_pop(&s->tx_fifo);        
        rx = ssi_transfer(s->spi, tx);

        if (!fifo8_is_full(&s->rx_fifo)) {
            fifo8_push(&s->rx_fifo, rx);
            /* Set RXNE flag */
            s->regs[R_SR] |= R_SR_RXNE_MASK;
        } else {
            /* RX FIFO overflow, set overrun error flag */
            s->regs[R_SR] |= R_SR_OVERRUN_MASK;
        }
    }
    s->regs[R_SR] |= R_SR_TXE_MASK;
    s->regs[R_SR] &= ~R_SR_BSY_MASK;
}

static bool g233_spi_is_bad_reg(hwaddr addr)
{
    /* G233 SPI register map:
     * 0x00: CR1    - Control register 1
     * 0x04: CR2    - Control register 2
     * 0x08: SR     - Status register
     * 0x0C: DR     - Data register
     * 0x10: CSCTRL - Chip select control register
     * 
     * Address range check: 0x00 ~ 0x10 (5 registers)
     */
    
    /* check boundary */
    if (addr >= (G233_SPI_REG_NUM << 2)) {
        return true;
    }

    /* check alignment */
    if (addr & 0x3) {
        return true;
    }

    switch (addr) {
    case 0x00:  /* CR1 */
    case 0x04:  /* CR2 */
    case 0x08:  /* SR */
    case 0x0C:  /* DR */
    case 0x10:  /* CSCTRL */
        return false;
    default:
        return true;
    }
}

static uint64_t g233_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    G233SPIState *s = opaque;
    uint32_t r = 0;

    if (g233_spi_is_bad_reg(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read at address 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return 0;
    }

    addr >>= 2;
    
    switch (addr) {
    case R_CR1:
    case R_CR2:
    case R_SR:
    case R_CSCTRL:
        r = s->regs[addr];
        break;

    case R_DR:
        if (!fifo8_is_empty(&s->rx_fifo)) {
            r = fifo8_pop(&s->rx_fifo);
            if (fifo8_is_empty(&s->rx_fifo)) {
                s->regs[R_SR] &= ~R_SR_RXNE_MASK;
            }
        } else {
            r = 0;
            s->regs[R_SR] |= R_SR_UNDERRUN_MASK;
        }
        break;

    default:
        r = s->regs[addr];
        break;
    }

    /* Update interrupt status */
    g233_spi_update_irq(s);

    return r;
}

static void g233_spi_write(void *opaque, hwaddr addr,
                             uint64_t val64, unsigned int size)
{
    G233SPIState *s = opaque;
    uint32_t value = val64;

    if (g233_spi_is_bad_reg(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write at addr=0x%"
                      HWADDR_PRIx " value=0x%x\n", __func__, addr, value);
        return;
    }

    addr >>= 2;
    
    switch (addr) {
    case R_CR1:
        s->regs[R_CR1] = value & 0x44;
        break;

    case R_CR2:
        s->regs[R_CR2] = value & 0xF0;
        break;

    case R_SR:
        /* Clear error flags: OVERRUN and UNDERRUN */
        if (value & R_SR_OVERRUN_MASK) {
            s->regs[R_SR] &= ~R_SR_OVERRUN_MASK;
        }
        if (value & R_SR_UNDERRUN_MASK) {
            s->regs[R_SR] &= ~R_SR_UNDERRUN_MASK;
        }
        break;

    case R_DR:
        /* Data register - write data to TX FIFO */
        if (!fifo8_is_full(&s->tx_fifo)) {
            fifo8_push(&s->tx_fifo, (uint8_t)(value & 0xFF));
            if (fifo8_is_full(&s->tx_fifo)) {
                s->regs[R_SR] &= ~R_SR_TXE_MASK;
            }
            g233_spi_flush_txfifo(s);
        } else {
            /* FIFO full, set OVERRUN */
            s->regs[R_SR] |= R_SR_OVERRUN_MASK;
            qemu_log_mask(LOG_GUEST_ERROR, 
                         "%s: TX FIFO full, data lost\n", __func__);
        }
        break;

    case R_CSCTRL:
        s->regs[R_CSCTRL] = value & 0xFF;
        g233_spi_update_cs(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to unknown register 0x%"
                      HWADDR_PRIx " with 0x%x\n", __func__, addr << 2, value);
        break;
    }

    g233_spi_update_irq(s);
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233SPIState *s = G233_SPI(dev);
    int i;

    s->spi = ssi_create_bus(dev, "spi");
    sysbus_init_irq(sbd, &s->irq);

    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    for (i = 0; i < s->num_cs; i++) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &g233_spi_ops, s,
                          TYPE_G233_SPI, 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);

    fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo8_create(&s->rx_fifo, FIFO_CAPACITY);
}

static const Property g233_spi_properties[] = {
    DEFINE_PROP_UINT32("num-cs", G233SPIState, num_cs, 4),
};

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, g233_spi_properties);
    device_class_set_legacy_reset(dc, g233_spi_reset);
    dc->realize = g233_spi_realize;
}

static const TypeInfo g233_spi_info = {
    .name           = TYPE_G233_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233SPIState),
    .class_init     = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)
