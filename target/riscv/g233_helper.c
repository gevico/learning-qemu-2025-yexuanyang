/*
 * RISC-V G233 Custom Instruction Helpers
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
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"

void HELPER(dma)(CPURISCVState *env, target_ulong dst, target_ulong src, target_ulong gran)
{
    int size = 8 << gran;
    
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            uint32_t val = cpu_ldl_data(env, src + (i * size + j) * sizeof(uint32_t));
            cpu_stl_data(env, dst + (j * size + i) * sizeof(uint32_t), (uint32_t)val);
        }
    }
}

void HELPER(sort)(CPURISCVState *env, target_ulong addr, target_ulong array_size, target_ulong sort_num)
{
    int len = (sort_num < array_size) ? sort_num : array_size;

    for (int i = 0; i < len - 1; i++) {
        for (int j = 0; j < len - i - 1; j++) {
            int32_t a = (int32_t)cpu_ldl_data(env, addr + j * 4);
            int32_t b = (int32_t)cpu_ldl_data(env, addr + (j + 1) * 4);
            if (a > b) {
                cpu_stl_data(env, addr + j * 4, (uint32_t)b);
                cpu_stl_data(env, addr + (j + 1) * 4, (uint32_t)a);
            }
        }
    }
}

void HELPER(crush)(CPURISCVState *env, target_ulong dst, target_ulong src, target_ulong num)
{
    for (target_ulong i = 0; i < num / 2; i++) {
        uint8_t low = cpu_ldub_data(env, src + i * 2) & 0xf;
        uint8_t high = cpu_ldub_data(env, src + i * 2 + 1) & 0xf;
        uint8_t packed = (high << 4) | low;
        cpu_stb_data(env, dst + i, packed);
    }
    
}

void HELPER(expand)(CPURISCVState *env, target_ulong dst, target_ulong src, target_ulong num)
{
    for (target_ulong i = 0; i < num; i++) {
        uint8_t packed = cpu_ldub_data(env, src + i);
        uint8_t low = packed & 0xf;
        uint8_t high = (packed >> 4) & 0xf;
        cpu_stb_data(env, dst + i * 2, low);
        cpu_stb_data(env, dst + i * 2 + 1, high);
    }
}
