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

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "qemu/fifo8.h"
#include "hw/sysbus.h"

#define G233_SPI_REG_NUM  (0x10 / 4 + 1)

#define TYPE_G233_SPI "g233.spi"
#define G233_SPI(obj) OBJECT_CHECK(G233SPIState, (obj), TYPE_G233_SPI)

typedef struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;

    uint32_t num_cs;
    qemu_irq *cs_lines;

    SSIBus *spi;

    Fifo8 tx_fifo;
    Fifo8 rx_fifo;

    uint32_t regs[G233_SPI_REG_NUM];
} G233SPIState;

#endif /* HW_G233_SPI_H */
