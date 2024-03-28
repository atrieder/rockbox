/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2002 by Alan Korr & Nick Robinson
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "button.h"
#include "config.h"
#include "cpu.h"
#include "system.h"
#include "kernel.h"
#include "lcd.h"
#include "serial.h"
#include "iap.h"

#if defined(IPOD_ACCESSORY_PROTOCOL)
static int autobaud = 0;
volatile unsigned long * base_RBR, * base_THR, * base_LCR, * base_LSR, * base_DLL;
#if defined(IPOD_COLOR) || defined(IPOD_4G) || defined(IPOD_MINI) || defined(IPOD_MINI2G)
/* These iPods actually have two serial ports; the dock connector and the
 * accessory port next to the headphone jack. We can set up an additional route
 * to cover both ports.
 * NOTE: This might also apply to the 3G iPod since it also has two connectors.
 */
#define MULTI_SER_PORT
volatile unsigned long * alt_base_RBR, * alt_base_THR, * alt_base_LCR, * alt_base_LSR, * alt_base_DLL;
/* Assume only one of the ports is being used at any given time. It is unlikely
 * for someone to plug the iPod into a dock/radio remote AND also want to use
 * the iPod remote or some other accessory in the top port.
 * When checking for TX/RX readiness, we can switch between the two ports based
 * on whichever is ready first.
 */
volatile unsigned long * active_base_RBR, * active_base_THR, * active_base_LSR;
#endif

static void set_bitrate(unsigned int rate)
{
    unsigned int divisor;

    divisor = 24000000L / rate / 16;
    *base_LCR = 0x80; /* Divisor latch enable */
    *base_DLL = (divisor >> 0) & 0xFF;
    *base_LCR = 0x03; /* Divisor latch disable, 8-N-1 */

    #if defined(IPOD_COLOR) || defined(IPOD_4G) || defined(IPOD_MINI) || defined(IPOD_MINI2G)
    /* Copy same values as above since they seem to work for the other port.
     * This assumes both ports are expected to use the same bitrate.
     */
    *alt_base_LCR = 0x80; /* Divisor latch enable */
    *alt_base_DLL = (divisor >> 0) & 0xFF;
    *alt_base_LCR = 0x03; /* Divisor latch disable, 8-N-1 */
    #endif
}

void serial_setup (void)
{
    int tmp;

#if defined(IPOD_COLOR) || defined(IPOD_4G) || defined(IPOD_MINI) || defined(IPOD_MINI2G)
    int alt_tmp;

    /* Route the Tx/Rx pins.  4G Ipod, MINI & MINI2G ser1, dock connector */
    GPIO_CLEAR_BITWISE(GPIOD_ENABLE, 0x6);
    GPIO_CLEAR_BITWISE(GPIOD_OUTPUT_EN, 0x6);

    outl(0x70000018, inl(0x70000018) & ~0xc00);

    base_RBR = &SER1_RBR;
    base_THR = &SER1_THR;
    base_LCR = &SER1_LCR;
    base_LSR = &SER1_LSR;
    base_DLL = &SER1_DLL;

    DEV_EN |= DEV_SER1;
    CPU_HI_INT_DIS = SER1_MASK;

    DEV_RS |= DEV_SER1;
    sleep(1);
    DEV_RS &= ~DEV_SER1;

    SER1_LCR = 0x80; /* Divisor latch enable */
    SER1_DLM = 0x00;
    SER1_LCR = 0x03; /* Divisor latch disable, 8-N-1 */
    SER1_IER = 0x01;

    SER1_FCR = 0x07; /* Tx+Rx FIFO reset and FIFO enable */

    CPU_INT_EN = HI_MASK;
    CPU_HI_INT_EN = SER1_MASK;
    tmp = SER1_RBR;

    /* Also Route the Tx/Rx pins. 4G Ipod, MINI & MINI2G ser0, top connector */
    GPIO_CLEAR_BITWISE(GPIOC_INT_EN, 0x8);
    GPIO_CLEAR_BITWISE(GPIOC_INT_LEV, 0x8);
    GPIOC_INT_CLR = 0x8;

    alt_base_RBR = &SER0_RBR;
    alt_base_THR = &SER0_THR;
    alt_base_LCR = &SER0_LCR;
    alt_base_LSR = &SER0_LSR;
    alt_base_DLL = &SER0_DLL;

    DEV_EN |= DEV_SER0;
    CPU_HI_INT_DIS = SER0_MASK;

    DEV_RS |= DEV_SER0;
    sleep(1);
    DEV_RS &= ~DEV_SER0;

    SER0_LCR = 0x80; /* Divisor latch enable */
    SER0_DLM = 0x00;
    SER0_LCR = 0x03; /* Divisor latch disable, 8-N-1 */
    SER0_IER = 0x01;

    SER0_FCR = 0x07; /* Tx+Rx FIFO reset and FIFO enable */

    CPU_INT_EN = HI_MASK;
    CPU_HI_INT_EN = SER0_MASK;
    alt_tmp = SER0_RBR;

#elif defined(IPOD_NANO) || defined(IPOD_VIDEO)
    /* Route the Tx/Rx pins.  5G Ipod */
    (*(volatile unsigned long *)(0x7000008C)) &= ~0x0C;
    GPO32_ENABLE &= ~0x0C;

    base_RBR = &SER0_RBR;
    base_THR = &SER0_THR;
    base_LCR = &SER0_LCR;
    base_LSR = &SER0_LSR;
    base_DLL = &SER0_DLL;

    DEV_EN = DEV_EN | DEV_SER0;
    CPU_HI_INT_DIS = SER0_MASK;

    DEV_RS |= DEV_SER0;
    sleep(1);
    DEV_RS &= ~DEV_SER0;

    SER0_LCR = 0x80; /* Divisor latch enable */
    SER0_DLM = 0x00;
    SER0_LCR = 0x03; /* Divisor latch disable, 8-N-1 */
    SER0_IER = 0x01;

    SER0_FCR = 0x07; /* Tx+Rx FIFO reset and FIFO enable */

    CPU_INT_EN = HI_MASK;
    CPU_HI_INT_EN = SER0_MASK;
    tmp = SER0_RBR;

#else

    /* Default Route the Tx/Rx pins.  4G Ipod, ser0, top connector */

    GPIO_CLEAR_BITWISE(GPIOC_INT_EN, 0x8);
    GPIO_CLEAR_BITWISE(GPIOC_INT_LEV, 0x8);
    GPIOC_INT_CLR = 0x8;

    base_RBR = &SER0_RBR;
    base_THR = &SER0_THR;
    base_LCR = &SER0_LCR;
    base_LSR = &SER0_LSR;
    base_DLL = &SER0_DLL;

    DEV_EN |= DEV_SER0;
    CPU_HI_INT_DIS = SER0_MASK;

    DEV_RS |= DEV_SER0;
    sleep(1);
    DEV_RS &= ~DEV_SER0;

    SER0_LCR = 0x80; /* Divisor latch enable */
    SER0_DLM = 0x00;
    SER0_LCR = 0x03; /* Divisor latch disable, 8-N-1 */
    SER0_IER = 0x01;

    SER0_FCR = 0x07; /* Tx+Rx FIFO reset and FIFO enable */

    CPU_INT_EN = HI_MASK;
    CPU_HI_INT_EN = SER0_MASK;
    tmp = SER0_RBR;

#endif

    (void)tmp;
    #if defined(IPOD_COLOR) || defined(IPOD_4G) || defined(IPOD_MINI) || defined(IPOD_MINI2G)
    (void)alt_tmp;
    #endif

    serial_bitrate(0);
}

void serial_bitrate(int rate)
{
    if(rate == 0)
    {
        autobaud = 2;
        set_bitrate(115200);
    }
    else
    {
        autobaud = 0;
        set_bitrate(rate);
    }
}

int tx_rdy(void)
{
    #if defined(MULTI_SER_PORT)
    /* Try either port, and if one is ready, just use that one and ignore the
     * other.
     */
    if (*alt_base_LSR & 0x20)
    {
        active_base_LSR = alt_base_LSR;
        active_base_RBR = alt_base_RBR;
        active_base_THR = alt_base_THR;
        return 1;
    }
    else if (*base_LSR & 0x20) {
        active_base_LSR = base_LSR;
        active_base_RBR = base_RBR;
        active_base_THR = base_THR;
        return 1;
    }
    else {
        return 0;
    }
    #else
    if((*base_LSR & 0x20))
        return 1;
    else
        return 0;
    #endif
}

static int rx_rdy(void)
{
    #if defined(MULTI_SER_PORT)
    /* Try either port, and if one is ready, just use that one and ignore the
     * other.
     */
    if (*alt_base_LSR & 0x1)
    {
        active_base_LSR = alt_base_LSR;
        active_base_RBR = alt_base_RBR;
        active_base_THR = alt_base_THR;
        return 1;
    } else if (*base_LSR & 0x1)
    {
        active_base_LSR = base_LSR;
        active_base_RBR = base_RBR;
        active_base_THR = base_THR;
        return 1;
    }
    else {
        return 0;
    }
    #else
    if((*base_LSR & 0x1))
        return 1;
    else
        return 0;
    #endif
}

void tx_writec(unsigned char c)
{
    #if defined(MULTI_SER_PORT)
    *active_base_THR =(int) c;
    #else
    *base_THR =(int) c;
    #endif
}

static unsigned char rx_readc(void)
{
    #if defined(MULTI_SER_PORT)
    return (*active_base_RBR & 0xFF);
    #else
    return (*base_RBR & 0xFF);
    #endif
}

void SERIAL_ISR(void)
{
    static int badbaud = 0;
    static bool newpkt = true;
    char temp;

    while(rx_rdy())
    {
        temp = rx_readc();
        if (newpkt && autobaud > 0)
        {
            if (autobaud == 1)
            {
                switch (temp)
                {
                    case 0xFF:
                    case 0x55:
                        break;
                    case 0xFC:
                        set_bitrate(19200);
                        temp = 0xFF;
                        break;
                    case 0xE0:
                        set_bitrate(9600);
                        temp = 0xFF;
                        break;
                    default:
                        badbaud++;
                        if (badbaud >= 6) /* Switch baud detection mode */
                        {
                            autobaud = 2;
                            set_bitrate(115200);
                            badbaud = 0;
                        } else {
                            set_bitrate(57600);
                        }
                        continue;
                }
            } else {
                switch (temp)
                {
                    case 0xFF:
                    case 0x55:
                        break;
                    case 0xFE:
                        set_bitrate(57600);
                        temp = 0xFF;
                        break;
                    case 0xFC:
                        set_bitrate(38400);
                        temp = 0xFF;
                        break;
                    case 0xE0:
                        set_bitrate(19200);
                        temp = 0xFF;
                        break;
                    default:
                        badbaud++;
                        if (badbaud >= 6) /* Switch baud detection */
                        {
                            autobaud = 1;
                            set_bitrate(57600);
                            badbaud = 0;
                        } else {
                            set_bitrate(115200);
                        }
                        continue;
                }
            }
        }
        bool pkt = iap_getc(temp);
        if(newpkt && !pkt)
            autobaud = 0; /* Found good baud */
        newpkt = pkt;
    }
}
#endif
