/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Amstrad PC5086 computer emulation.
 *
 * Authors: Lubomir Rintel, <lkundrak@v3.sk>
 *          Davide Bresolin, <https://github.com/davidebreso>
 *
 *          Copyright 2021 Lubomir Rintel.
 *          Copyright 2023 Davide Bresolin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free  Software  Foundation; either  version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is  distributed in the hope that it will be useful, but
 * WITHOUT   ANY  WARRANTY;  without  even   the  implied  warranty  of
 * MERCHANTABILITY  or FITNESS  FOR A PARTICULAR  PURPOSE. See  the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 *
 *   Free Software Foundation, Inc.
 *   59 Temple Place - Suite 330
 *   Boston, MA 02111-1307
 *   USA.
 */
#include <stdio.h>
#include <stdint.h>
#include <86box/86box.h>
#include "cpu.h"
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/rom.h>
#include <86box/machine.h>
#include <86box/device.h>
#include <86box/fdd.h>
#include <86box/fdc.h>
#include <86box/fdc_ext.h>
#include <86box/nmi.h>
#include <86box/nvr.h>
#include <86box/hdc.h>
#include <86box/keyboard.h>
#include <86box/mouse.h>
#include <86box/chipset.h>
#include <86box/sio.h>
#include <86box/video.h>

int
machine_pc5086_init(const machine_t *model)
{
    int ret;

    ret = bios_load_linear("roms/machines/pc5086/sys_rom.bin",
                               0x000f0000, 65536, 0);

    if (bios_only || !ret)
        return ret;

    machine_common_init(model);

    nmi_init();
    device_add(&ibmat_nvr_device);

    device_add(&ct_82c100_device);
    device_add(&pc5086_sio_device);

    device_add(&keyboard_extclone_device);

    /*
     * Set up and enable the XTA disk controller.
     *
     * We only do this if we have not configured another one.
     */
    if (hdc_current == 1)
        device_add(&xta_pc5086_device);

    /*
     * Set up and enable the internal C&T 82C451 video chip.
     *
     * We only do this if we have not configured another one.
     */
    if (gfxcard[0] == VID_INTERNAL)
        device_add(&ct451_pc5086_device);

    return ret;
}
