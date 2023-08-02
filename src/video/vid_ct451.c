/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Chips & Technologies 82C451 emulation.
 *
 *
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Davide Bresolin
 *
 *          Copyright 2008-2018 Sarah Walker.
 *          Copyright 2016-2018 Miran Grca.
 *          Copyright 2023 Davide Bresolin.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/video.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>
#include <86box/plat_unused.h>

#define BIOS_CT451_PATH        "roms/video/ct451/evga451.bin"
#define BIOS_CT451_PC5086_PATH "roms/machines/pc5086/c000.bin"

enum {
    CT451,
    CT451_PC5086
};

typedef struct ct451_t
{
    svga_t svga;
    rom_t bios_rom;

    /* Setup registers */
    uint8_t sleep;      /* Video subsystem sleep */
    uint8_t xena;       /* Extended Enable */
    uint8_t xrx;        /* Extension index register */
    uint8_t xreg[128];  /* Extension registers */
    uint8_t setup;      /* Setup control register */
    
    /* Pixel clocks */
    double clock[3];
} ct451_t;

#ifdef ENABLE_CT451_LOG
int ct451_do_log = ENABLE_CT451_LOG;

static void
ct451_log(const char *fmt, ...)
{
    va_list ap;

    if (ct451_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define ct451_log(fmt, ...)
#endif

static int
ct451_wp_group0(ct451_t *ct451)
{
        /* Group 0 write protection is enabled if CR11 bit 7 OR XR15 bit 6 are 1 */
        svga_t *svga = &ct451->svga;
        return (svga->crtc[0x11] & 0x80) || (ct451->xreg[0x15] & 0x40);
}

static void
ct451_out(uint16_t addr, uint8_t val, void *priv)
{
        ct451_t *ct451 = (ct451_t *)priv;
        svga_t *svga = &ct451->svga;
        uint8_t old;

        /* if VGA chip is disabled, respond only to ports 0x10x and setup control */
        if (!(ct451->setup & 8) && ((addr&0xFF00) == 0x300))
                return;

        /* Extension registers address selection. 
         * Address depends on bit 6 of Extension enable register:
         *      0: address 0x3D6/0x3D7
         *      1: address 0x3B6/0x3B7
         */
        if ((addr&0xFFFE) == 0x3D6 || (addr&0xFFFE) == 0x3B6) 
        {
                if(ct451->xena & 0x40)
                {
                        addr ^= 0x60;        
                }
        }
        /* mono / color addr selection */
        else if (((addr&0xFFF0) == 0x3D0 || (addr&0xFFF0) == 0x3B0) && !(svga->miscout & 1))
        {
                addr ^= 0x60;        
        }

/*
        ct451_log("ct451_out : %04X %02X  %02X ", addr, val, ram[0x489]);
        ct451_log("  %04X:%04X\n", CS,cpu_state.pc); 
*/        
        switch(addr)
        {
                case 0x102:     /* Video subsystem sleep control */
                ct451->sleep = val;
                break;
                
                case 0x103:     /* Extension Enable Register */
                /* The register is available only in Setup mode (bit 4 of setup register set) */
                if(ct451->setup & 0x10)
                        ct451->xena = val;
                break;
            
                case 0x104:     /* Global ID (read-only) */
                break;

                case 0x3D4:     /* CRTC Index register */
                svga->crtcreg = val & 0x3f;
                return;
                
                case 0x3D5:     /* CRCT Register data */
                if (svga->crtcreg > 0x18)
                {
                        ct451_log("Write to undocumented CRTC register %02X\n", svga->crtcreg);
                        return;                
                }
                /* If group protect 0 is enabled, disable write to CR00-CR06 */
                if ((svga->crtcreg < 7) && ct451_wp_group0(ct451)) 
                        return;
                /* If group protect 0 is enabled, enable write only to bit 4 of CR07 */
                if ((svga->crtcreg == 7) && ct451_wp_group0(ct451))
                        val = (svga->crtc[7] & ~0x10) | (val & 0x10);
                old = svga->crtc[svga->crtcreg];
                svga->crtc[svga->crtcreg] = val;
                if (old != val)
                {
                        if (svga->crtcreg < 0xe || svga->crtcreg > 0x10)
                        {
                                svga->fullchange = changeframecount;
                                svga_recalctimings(svga);
                        }
                }
                break;

                case 0x3D6:     /* Extension index register. Active only when bit 7 of XENA = 1 */
                if(ct451->xena & 0x80)
                    ct451->xrx = val & 0x7F;
                break;
            
                case 0x3D7:     /* Extension register data. Active only when bit 7 of XENA = 1 */
                if(ct451->xena & 0x80)
                    ct451->xreg[ct451->xrx] = val; 
                break;               
                
                case 0x46E8:    /* Setup control register (write only) */
                ct451->setup = val;
                break;                
                
                default:
                svga_out(addr, val, svga);            
        }       

}

static uint8_t
ct451_in(uint16_t addr, void *p)
{
        ct451_t *ct451 = (ct451_t *)p;
        svga_t *svga = &ct451->svga;
        uint8_t temp = 0xff;

        /* if VGA chip is disabled, respond only to ports 0x10x and setup control */
        if (!(ct451->setup & 8) && ((addr&0xFF00) == 0x300))
                return temp;

        /* Extension registers address selection. 
         * Address depends on bit 6 of Extension enable register:
         *      0: address 0x3D6/0x3D7
         *      1: address 0x3B6/0x3B7
         */
        if ((addr&0xFFFE) == 0x3D6 || (addr&0xFFFE) == 0x3B6) 
        {
                if(ct451->xena & 0x40)
                {
                        addr ^= 0x60;        
                }
        }
        /* mono / color addr selection */
        else if (((addr&0xFFF0) == 0x3D0 || (addr&0xFFF0) == 0x3B0) && !(svga->miscout & 1))
        {
                addr ^= 0x60;        
        }
        
        switch(addr)
        {
                case 0x102:     /* Video subsystem sleep control */
                temp = ct451->sleep;
                break;
            
                case 0x103:     /* Extension Enable Register */
                /* The register is available only in Setup mode (bit 4 of setup register set) */
                if(ct451->setup & 0x10)
                        temp = ct451->xena;
                break;
            
                case 0x104:     /* Global ID (0xA5 read-only) */
                temp = 0xA5;
                break;

                case 0x3D4:
                temp = svga->crtcreg;
                break;
            
                case 0x3D5:
                if (svga->crtcreg > 0x18)
                {
                        ct451_log("Read from undocumented CRTC register %02X\n", svga->crtcreg);
                        temp = 0xff;                
                } 
                else
                    temp = svga->crtc[svga->crtcreg & 31];
                break;
          
                case 0x3D6:     /* Extension index register. Active only when bit 7 of XENA = 1 */
                if(ct451->xena & 0x80)
                    temp = ct451->xrx;
                break;
            
                case 0x3D7:     /* Extension register data. Active only when bit 7 of XENA = 1 */
                if(ct451->xena & 0x80)
                    temp = ct451->xreg[ct451->xrx];                
                break;

                case 0x46E8:    /* Setup control register (write only) */
                temp = 0xff;
                break;                
            
                default:
                temp = svga_in(addr, svga);
        }
/*        
        ct451_log("ct451_in : %04X %02X  %02X ", addr, temp, ram[0x489]);
        ct451_log("  %04X:%04X\n", CS,cpu_state.pc);        
*/
        return temp;
}

static void
ct451_recalctimings(svga_t *svga)
{
    ct451_t *ct451     = (ct451_t *) svga->priv;
    int    clk_sel = (svga->miscout >> 2) & 3;

    /* Set CLK0 if selected clock is not valid */
    if (clk_sel > 2)
        clk_sel = 0;
    ct451_log("CT451: Pixel clock %f MHz\n", ct451->clock[clk_sel]/1000000.0);
    svga->clock = (cpuclock * (double) (1ULL << 32)) / ct451->clock[clk_sel];
}


static void *
ct451_init(const device_t *info)
{
    ct451_t *ct451 = malloc(sizeof(ct451_t));
    char  *romfn = NULL;
    memset(ct451, 0, sizeof(ct451_t));

    /* Set standard VGA pixel clocks */
    ct451->clock[0] = 25175000.0;
    ct451->clock[1] = 28322000.0;

    switch (info->local) {
        case CT451:
            romfn = BIOS_CT451_PATH;
            /* 40 MHz CLK2 for 132 columns and 800x600 mode */
            ct451->clock[2] = 40000000.0;
            break;
        
        case CT451_PC5086:
            romfn = BIOS_CT451_PC5086_PATH;
             /* CLK2 is 32MHz on the PC5086? TODO: check the actual hardware */
            ct451->clock[2] = 32000000.0;
           break;
    }
    
    ct451_log("CT451: setting up BIOS from %s\n", romfn);
    rom_init(&ct451->bios_rom, romfn, 0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);

    ct451_log("CT451: calling SVGA init\n");
    svga_init(info, &ct451->svga, ct451, 256 << 10,
               ct451_recalctimings,
               ct451_in, ct451_out,
               NULL,
               NULL);

    ct451_log("CT451: setting up I/O handler\n");
    /* handler for setup registers */
    io_sethandler(0x0100, 0x0005, ct451_in, NULL, NULL, ct451_out, NULL, NULL, ct451);
    /* handler for VGA registers */
    io_sethandler(0x03c0, 0x0020, ct451_in, NULL, NULL, ct451_out, NULL, NULL, ct451);
    /* handler for setup control register */
    io_sethandler(0x46E8, 0x0001, ct451_in, NULL, NULL, ct451_out, NULL, NULL, ct451);
        
    /* Set default register values */
    // ct451->svga.miscout = 1;
    ct451->xreg[0x00] = 0x4;    /* Chip version */
    ct451->xreg[0x01] = 0x5A;   /* DIP switch */
    ct451->xreg[0x28] = 0x2;    /* Video interface */
    
    return ct451;
}

static void
ct451_close(void *priv)
{
        ct451_log("ct451_close %p\n", priv);

        ct451_t *ct451 = (ct451_t *)priv;

        svga_close(&ct451->svga);

        free(ct451);
}


static void
ct451_speed_changed(void *priv)
{
    ct451_t *ct451 = (ct451_t *) priv;

    svga_recalctimings(&ct451->svga);
}

static void
ct451_force_redraw(void *priv)
{
    ct451_t *ct451 = (ct451_t *) priv;

    ct451->svga.fullchange = changeframecount;
}

static int
ct451_available(void)
{
    return (rom_present(BIOS_CT451_PATH));
}

static int
ct451_pc5086_available(void)
{
    return (rom_present(BIOS_CT451_PC5086_PATH));
}

const device_t ct451_device =
{
    .name          = "Chips and Technologies 82C451",
    .internal_name = "ct451",
    .flags         = DEVICE_ISA,
    .local         = CT451,
    .init          = ct451_init,
    .close         = ct451_close,
    .reset         = NULL,
    { .available = ct451_available },
    .speed_changed = ct451_speed_changed,
    .force_redraw  = ct451_force_redraw,
    .config        = NULL
};

const device_t ct451_pc5086_device =
{
    .name          = "C&T 82C451 (Amstrad PC5086)",
    .internal_name = "ct451_pc5086",
    .flags         = DEVICE_ISA,
    .local         = CT451_PC5086,
    .init          = ct451_init,
    .close         = ct451_close,
    .reset         = NULL,
    { .available = ct451_pc5086_available },
    .speed_changed = ct451_speed_changed,
    .force_redraw  = ct451_force_redraw,
    .config        = NULL
};

