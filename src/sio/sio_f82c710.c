/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Chips & Technologies F82C710 Universal Peripheral
 *          Controller (UPC) and 82C606 CHIPSpak Multifunction Controller.
 *
 * Relevant literature:
 *
 *          [1] Chips and Technologies, Inc.,
 *              82C605/82C606 CHIPSpak/CHIPSport MULTIFUNCTION CONTROLLERS,
 *              PRELIMINARY Data Sheet, Revision 1, May 1987.
 *              <https://archive.org/download/82C606/82C606.pdf>
 *
 *
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Eluan Costa Miranda <eluancm@gmail.com>
 *          Lubomir Rintel <lkundrak@v3.sk>
 *          Davide Bresolin 
 *
 *          Copyright 2020 Sarah Walker.
 *          Copyright 2020 Eluan Costa Miranda.
 *          Copyright 2021 Lubomir Rintel.
 *          Copyright 2023 Davide Bresolin.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/lpt.h>
#include <86box/serial.h>
#include <86box/gameport.h>
#include <86box/hdc.h>
#include <86box/hdc_ide.h>
#include <86box/fdd.h>
#include <86box/fdc.h>
#include <86box/nvr.h>
#include <86box/sio.h>
#include <86box/keyboard.h>
#include <86box/mouse.h>

#define UPC_MOUSE_DEV_IDLE     0x01      /* bit 0, Device Idle */
#define UPC_MOUSE_RX_FULL      0x02      /* bit 1, Device Char received */
#define UPC_MOUSE_TX_IDLE      0x04      /* bit 2, Device XMIT Idle */
#define UPC_MOUSE_RESET        0x08      /* bit 3, Device Reset */
#define UPC_MOUSE_INTS_ON      0x10      /* bit 4, Device Interrupt On */
#define UPC_MOUSE_ERROR_FLAG   0x20      /* bit 5, Device Error */
#define UPC_MOUSE_CLEAR        0x40      /* bit 6, Device Clear */
#define UPC_MOUSE_ENABLE       0x80      /* bit 7, Device Enable */

typedef struct upc_t {
    uint32_t local;
    int      configuration_state; /* state of algorithm to enter configuration mode */
    int      configuration_mode;
    uint16_t cri_addr; /* cri = configuration index register, addr is even */
    uint16_t cap_addr; /* cap = configuration access port, addr is odd and is cri_addr + 1 */
    uint8_t  cri;      /* currently indexed register */
    uint8_t  last_write;

    /* these regs are not affected by reset */
    uint8_t   regs[15]; /* there are 16 indexes, but there is no need to store the last one which is: R = cri_addr / 4, W = exit config mode */
    fdc_t    *fdc;
    nvr_t    *nvr;
    void     *gameport;
    serial_t *uart[2];
    atkbc_dev_t *mouse;
    
    
    int      mouse_irq;     // PS/2 mouse IRQ
    uint16_t mdata_addr;    // Address of PS/2 data register
    uint16_t mstat_addr;    // Address of PS/2 status register
    uint8_t  mouse_status;   // Mouse interface status register
    pc_timer_t mouse_delay_timer;   // Mouse interface timer
} upc_t;

#ifdef ENABLE_F82C710_LOG
int f82c710_do_log = ENABLE_F82C710_LOG;

static void
f82c710_log(const char *fmt, ...)
{
    va_list ap;

    if (f82c710_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define f82c710_log(fmt, ...)
#endif

static void f82c710_mouse_disable(upc_t *dev);
static void f82c710_mouse_enable(upc_t *dev);
static uint8_t f82c710_mouse_read(uint16_t port, void *priv);
static void f82c710_mouse_write(uint16_t port, uint8_t val, void *priv);
static void f82c710_mouse_poll(void *priv);

static void
f82c710_update_ports(upc_t *dev, int set)
{
    uint16_t com_addr = 0;
    uint16_t lpt_addr = 0;

    serial_remove(dev->uart[0]);
    serial_remove(dev->uart[1]);
    lpt1_remove();
    lpt2_remove();
    fdc_remove(dev->fdc);
    ide_pri_disable();
    f82c710_mouse_disable(dev);

    if (!set)
        return;

    if (dev->regs[0] & 4) {
        com_addr = dev->regs[4] * 4;
        if (com_addr == COM1_ADDR)
            serial_setup(dev->uart[0], com_addr, COM1_IRQ);
        else if (com_addr == COM2_ADDR)
            serial_setup(dev->uart[1], com_addr, COM2_IRQ);
    }

    if (dev->regs[0] & 8) {
        lpt_addr = dev->regs[6] * 4;
        lpt1_init(lpt_addr);
        if ((lpt_addr == LPT1_ADDR) || (lpt_addr == LPT_MDA_ADDR))
            lpt1_irq(LPT1_IRQ);
        else if (lpt_addr == LPT2_ADDR)
            lpt1_irq(LPT2_IRQ);
    }

    /* Enable primary IDE controller if:
     * IDE is enabled (bit 7 == 1) in AT mode (bit 6 == 0) */
    if ((dev->regs[12] & 0x80) && !(dev->regs[12] & 0x40))
        ide_pri_enable();

    if (dev->regs[12] & 0x20)
        fdc_set_base(dev->fdc, FDC_PRIMARY_ADDR);
        
    if (dev->regs[13] != 0 && dev->mouse != NULL) {
        dev->mdata_addr = dev->regs[13] * 4;
        dev->mstat_addr = dev->mdata_addr + 1;
        f82c710_log("PS/2 mouse port at %04X\n", dev->mdata_addr);
        f82c710_mouse_enable(dev);
    }
}

static void
f82c606_update_ports(upc_t *dev, int set)
{
    uint8_t uart1_int = 0xff;
    uint8_t uart2_int = 0xff;
    uint8_t lpt1_int  = 0xff;
    int     nvr_int   = -1;

    serial_remove(dev->uart[0]);
    serial_remove(dev->uart[1]);
    lpt1_remove();
    lpt2_remove();

    nvr_at_handler(0, ((uint16_t) dev->regs[3]) << 2, dev->nvr);
    nvr_at_handler(0, 0x70, dev->nvr);

    gameport_remap(dev->gameport, 0);

    if (!set)
        return;

    switch (dev->regs[8] & 0xc0) {
        case 0x40:
            nvr_int = 3;
            break;
        case 0x80:
            uart1_int = COM2_IRQ;
            break;
        case 0xc0:
            uart2_int = COM2_IRQ;
            break;

        default:
            break;
    }

    switch (dev->regs[8] & 0x30) {
        case 0x10:
            nvr_int = 4;
            break;
        case 0x20:
            uart1_int = COM1_IRQ;
            break;
        case 0x30:
            uart2_int = COM1_IRQ;
            break;

        default:
            break;
    }

    switch (dev->regs[8] & 0x0c) {
        case 0x04:
            nvr_int = 5;
            break;
        case 0x08:
            uart1_int = 5;
            break;
        case 0x0c:
            lpt1_int = LPT2_IRQ;
            break;

        default:
            break;
    }

    switch (dev->regs[8] & 0x03) {
        case 0x01:
            nvr_int = 7;
            break;
        case 0x02:
            uart2_int = 7;
            break;
        case 0x03:
            lpt1_int = LPT1_IRQ;
            break;

        default:
            break;
    }

    if (dev->regs[0] & 1) {
        gameport_remap(dev->gameport, ((uint16_t) dev->regs[7]) << 2);
        f82c710_log("Game port at %04X\n", ((uint16_t) dev->regs[7]) << 2);
    }

    if (dev->regs[0] & 2) {
        serial_setup(dev->uart[0], ((uint16_t) dev->regs[4]) << 2, uart1_int);
        f82c710_log("UART 1 at %04X, IRQ %i\n", ((uint16_t) dev->regs[4]) << 2, uart1_int);
    }

    if (dev->regs[0] & 4) {
        serial_setup(dev->uart[1], ((uint16_t) dev->regs[5]) << 2, uart2_int);
        f82c710_log("UART 2 at %04X, IRQ %i\n", ((uint16_t) dev->regs[5]) << 2, uart2_int);
    }

    if (dev->regs[0] & 8) {
        lpt1_init(((uint16_t) dev->regs[6]) << 2);
        lpt1_irq(lpt1_int);
        f82c710_log("LPT1 at %04X, IRQ %i\n", ((uint16_t) dev->regs[6]) << 2, lpt1_int);
    }

    nvr_at_handler(1, ((uint16_t) dev->regs[3]) << 2, dev->nvr);
    nvr_irq_set(nvr_int, dev->nvr);
    f82c710_log("RTC at %04X, IRQ %i\n", ((uint16_t) dev->regs[3]) << 2, nvr_int);
}

static uint8_t
f82c710_config_read(uint16_t port, void *priv)
{
    upc_t  *dev  = (upc_t *) priv;
    uint8_t temp = 0xff;

    if (dev->configuration_mode) {
        if (port == dev->cri_addr) {
            temp = dev->cri;
        } else if (port == dev->cap_addr) {
            if (dev->cri == 0xf)
                temp = dev->cri_addr / 4;
            else
                temp = dev->regs[dev->cri];
        }
    }

    return temp;
}

static void
f82c710_config_write(uint16_t port, uint8_t val, void *priv)
{
    upc_t *dev                       = (upc_t *) priv;
    int    configuration_state_event = 0;

    switch (port) {
        case 0x2fa:
            if ((dev->configuration_state == 0) && (val != 0x00) && (val != 0xff)) {
                configuration_state_event = 1;
                dev->last_write           = val;
            } else if (dev->configuration_state == 4) {
                if ((val | dev->last_write) == 0xff) {
                    dev->cri_addr           = ((uint16_t) dev->last_write) << 2;
                    dev->cap_addr           = dev->cri_addr + 1;
                    dev->configuration_mode = 1;
                    if (dev->local == 606)
                        f82c606_update_ports(dev, 0);
                    else if (dev->local >= 710)
                        f82c710_update_ports(dev, 0);
                    /* TODO: is the value of cri reset here or when exiting configuration mode? */
                    io_sethandler(dev->cri_addr, 0x0002, f82c710_config_read, NULL, NULL, f82c710_config_write, NULL, NULL, dev);
                } else
                    dev->configuration_mode = 0;
            }
            break;
        case 0x3fa:
            if ((dev->configuration_state == 1) && ((val | dev->last_write) == 0xff))
                configuration_state_event = 1;
            else if ((dev->configuration_state == 2) && (val == 0x36))
                configuration_state_event = 1;
            else if (dev->configuration_state == 3) {
                dev->last_write           = val;
                configuration_state_event = 1;
            }
            break;
        default:
            break;
    }

    if (dev->configuration_mode) {
        if (port == dev->cri_addr) {
            dev->cri = val & 0xf;
        } else if (port == dev->cap_addr) {
            if (dev->cri == 0xf) {
                dev->configuration_mode = 0;
                io_removehandler(dev->cri_addr, 0x0002, f82c710_config_read, NULL, NULL, f82c710_config_write, NULL, NULL, dev);
                /* TODO: any benefit in updating at each register write instead of when exiting config mode? */
                if (dev->local == 606)
                    f82c606_update_ports(dev, 1);
                else if (dev->local >= 710)
                    f82c710_update_ports(dev, 1);
            } else
                dev->regs[dev->cri] = val;
        }
    }

    /* TODO: is the state only reset when accessing 0x2fa and 0x3fa wrongly? */
    if ((port == 0x2fa || port == 0x3fa) && configuration_state_event)
        dev->configuration_state++;
    else
        dev->configuration_state = 0;
}

static void
f82c710_reset(void *priv)
{
    upc_t *dev = (upc_t *) priv;

    /* Set power-on defaults. */
    if (dev->local == 606) {
        dev->regs[0] = 0x00; /* Enable */
        dev->regs[1] = 0x00; /* Configuration Register */
        dev->regs[2] = 0x00; /* Ext Baud Rate Select */
        dev->regs[3] = 0xb0; /* RTC Base */
        dev->regs[4] = 0xfe; /* UART1 Base */
        dev->regs[5] = 0xbe; /* UART2 Base */
        dev->regs[6] = 0x9e; /* Parallel Base */
        dev->regs[7] = 0x80; /* Game Base */
        dev->regs[8] = 0xec; /* Interrupt Select */
    } else if (dev->local >= 710) {
        dev->regs[0]  = 0x0c;
        dev->regs[1]  = 0x00;
        dev->regs[2]  = 0x00;
        dev->regs[3]  = 0x00;
        dev->regs[4]  = 0xfe;
        dev->regs[5]  = 0x00;
        dev->regs[6]  = 0x9e;
        dev->regs[7]  = 0x00;
        dev->regs[8]  = 0x00;
        dev->regs[9]  = 0xb0;
        dev->regs[10] = 0x00;
        dev->regs[11] = 0x00;
        dev->regs[12] = 0xa0;
        dev->regs[13] = 0x00;
        dev->regs[14] = 0x00;
    }

    if (dev->local == 606)
        f82c606_update_ports(dev, 1);
    else if (dev->local >= 710)
        f82c710_update_ports(dev, 1);
}

static void
f82c710_close(void *priv)
{
    upc_t *dev = (upc_t *) priv;

    free(dev);
}

static void *
f82c710_init(const device_t *info)
{
    upc_t *dev = (upc_t *) malloc(sizeof(upc_t));
    memset(dev, 0, sizeof(upc_t));
    dev->local = info->local;

    if (dev->local == 606) {
        dev->nvr      = device_add(&at_nvr_old_device);
        dev->gameport = gameport_add(&gameport_sio_device);
    } else if (dev->local == 710) {
        dev->fdc = device_add(&fdc_at_device);
        dev->mouse_irq = -1;
    } else if (dev->local == 1710) {
        dev->fdc = device_add(&fdc_at_actlow_device);
        /* Mouse port is at IRQ2 on the PC5086 */
        dev->mouse_irq = 2;
    }

    dev->uart[0] = device_add_inst(&ns16450_device, 1);
    dev->uart[1] = device_add_inst(&ns16450_device, 2);
    /*
     * Set up the onboard PS2 mouse, if we have selected the internal mouse
     */
    if (mouse_type == MOUSE_TYPE_INTERNAL) {
        dev->mouse = device_add(&mouse_ps2_onboard_device);
        mouse_set_poll(mouse_ps2_poll, dev->mouse);
        timer_add(&dev->mouse_delay_timer, f82c710_mouse_poll, dev, 1);
    }

    io_sethandler(0x02fa, 0x0001, NULL, NULL, NULL, f82c710_config_write, NULL, NULL, dev);
    io_sethandler(0x03fa, 0x0001, NULL, NULL, NULL, f82c710_config_write, NULL, NULL, dev);

    f82c710_reset(dev);

    return dev;
}

const device_t f82c606_device = {
    .name          = "82C606 CHIPSpak Multifunction Controller",
    .internal_name = "f82c606",
    .flags         = 0,
    .local         = 606,
    .init          = f82c710_init,
    .close         = f82c710_close,
    .reset         = f82c710_reset,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t f82c710_device = {
    .name          = "F82C710 UPC Super I/O",
    .internal_name = "f82c710",
    .flags         = 0,
    .local         = 710,
    .init          = f82c710_init,
    .close         = f82c710_close,
    .reset         = f82c710_reset,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t pc5086_sio_device = {
    .name          = "Amstrad PC5086 F82C710 UPC Super I/O",
    .internal_name = "pc5086sio",
    .flags         = 0,
    .local         = 1710,
    .init          = f82c710_init,
    .close         = f82c710_close,
    .reset         = f82c710_reset,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

/****************** PS/2 mouse port ********************/
void *
f82c710_ps2_dev_init()
{
    atkbc_dev_t *dev;

    dev = (atkbc_dev_t *) malloc(sizeof(atkbc_dev_t));
    memset(dev, 0x00, sizeof(atkbc_dev_t));

    dev->port = (kbc_at_port_t *) malloc(sizeof(kbc_at_port_t));
    memset(dev->port, 0x00, sizeof(kbc_at_port_t));
    dev->port->out_new = -1;
    dev->port->priv = dev;
    dev->port->poll = kbc_at_dev_poll;

    /* Return our private data to the I/O layer. */
    return dev;
}

static uint8_t 
f82c710_mouse_read(uint16_t port, void *priv)
{
    upc_t *dev = (upc_t *)priv;
    uint8_t temp = 0xff;
    if (port == dev->mstat_addr)
    {
        temp = dev->mouse_status;
    }

    if (port == dev->mdata_addr && (dev->mouse_status & UPC_MOUSE_RX_FULL))
    {
        temp = dev->mouse->port->out_new;
        dev->mouse->port->out_new = -1;
        dev->mouse_status &= ~UPC_MOUSE_RX_FULL;
        dev->mouse_status |= UPC_MOUSE_DEV_IDLE;
        if((dev->mouse_status & UPC_MOUSE_INTS_ON) && (dev->mouse_irq != -1))
            picintc(1 << dev->mouse_irq);
        // pclog("%04X:%04X UPC mouse READ: %04X, %02X\n", CS, cpu_state.pc, port, temp);
    }

    f82c710_log("[%04X:%04X] mouse READ: %04X, %02X\n", CS, cpu_state.pc, port, temp);
    return temp;
}

static void 
f82c710_mouse_write(uint16_t port, uint8_t val, void *priv)
{
    f82c710_log("[%04X:%04X] mouse WRITE: %04X, %02X\n", CS, cpu_state.pc, port, val);

    upc_t *dev = (upc_t *)priv;
    if (port == dev->mstat_addr) {
        /* write status bits
         * DEV_IDLE, TX_IDLE, RX_FULL and ERROR_FLAG bits are unchanged
         */
        dev->mouse_status = (val & 0xD8) | (dev->mouse_status & 0x27);
        if (dev->mouse_status & (UPC_MOUSE_CLEAR | UPC_MOUSE_RESET))
        {
            /* if CLEAR or RESET bit is set, clear mouse queue */
            kbc_at_dev_reset(dev->mouse, 0);
            dev->mouse_status &= ~UPC_MOUSE_RX_FULL;
            dev->mouse_status |= UPC_MOUSE_DEV_IDLE | UPC_MOUSE_TX_IDLE;
            if(dev->mouse_irq != -1)
                picintc(1 << dev->mouse_irq);
        }
    }

    if (port == dev->mdata_addr) {
        if ((dev->mouse_status & UPC_MOUSE_TX_IDLE) && (dev->mouse_status & UPC_MOUSE_ENABLE)) {
            dev->mouse->port->dat = val;
            dev->mouse->port->wantcmd = 1;            
            dev->mouse_status &= ~UPC_MOUSE_TX_IDLE;
        }
    }
    f82c710_log("[%04X:%04X] mouse WRITE: %04X, %02X\n", CS, cpu_state.pc, port, val);    
}

static void 
f82c710_mouse_disable(upc_t *dev)
{
    io_removehandler(dev->mdata_addr, 0x0002, f82c710_mouse_read, NULL, NULL, f82c710_mouse_write, NULL, NULL, dev);
}

static void 
f82c710_mouse_enable(upc_t *dev)
{
    io_sethandler(dev->mdata_addr, 0x0002, f82c710_mouse_read, NULL, NULL, f82c710_mouse_write, NULL, NULL, dev);
}

static void
f82c710_mouse_poll(void *priv)
{
    upc_t *dev = (upc_t *)priv;
    

    if(dev->mouse == NULL)
        return;
    
    /* On-board PS2 mouse is present, advance timer and poll pending commands */
    timer_advance_u64(&dev->mouse_delay_timer, (100ULL * TIMER_USEC));
    
    if(dev->mouse_status & UPC_MOUSE_ENABLE) {
        /* mouse is enabled: poll pending commands */
        dev->mouse->port->poll(dev->mouse);
        /* set mouse status flags */
        if(dev->mouse->port->wantcmd) {
            dev->mouse_status &= ~UPC_MOUSE_TX_IDLE;
        } else {
            dev->mouse_status |= UPC_MOUSE_TX_IDLE;
        }
        if(dev->mouse->port->out_new == -1) {
            dev->mouse_status &= ~UPC_MOUSE_RX_FULL;        
            dev->mouse_status |= UPC_MOUSE_DEV_IDLE;                            
        } else {
            dev->mouse_status |= UPC_MOUSE_RX_FULL; 
            dev->mouse_status &= ~UPC_MOUSE_DEV_IDLE;
            if((dev->mouse_status & UPC_MOUSE_INTS_ON) && (dev->mouse_irq != -1))
                picint(1 << dev->mouse_irq);
        }
    }
}

