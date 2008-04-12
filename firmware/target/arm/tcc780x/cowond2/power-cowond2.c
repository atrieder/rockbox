/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 Dave Chapman
 *
 * All files in this archive are subject to the GNU General Public License.
 * See the file COPYING in the source tree root for full license agreement.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#include "cpu.h"
#include "kernel.h"
#include "system.h"
#include "power.h"
#include "pcf50606.h"
#include "cpu.h"

#ifndef SIMULATOR

void power_init(void)
{
    unsigned char data[3]; /* 0 = INT1, 1 = INT2, 2 = INT3 */

    /* Clear pending interrupts from pcf50606 */
    pcf50606_read_multiple(0x02, data, 3);
    
    /* Set outputs as per OF - further investigation required. */
    pcf50606_write(PCF5060X_DCDEC1,  0xe4);
    pcf50606_write(PCF5060X_IOREGC,  0xf5);
    pcf50606_write(PCF5060X_D1REGC1, 0xf5);
    pcf50606_write(PCF5060X_D2REGC1, 0xe9);
    pcf50606_write(PCF5060X_D3REGC1, 0xf8); /* WM8985 3.3v */
    pcf50606_write(PCF5060X_DCUDC1,  0xe7);
    pcf50606_write(PCF5060X_LPREGC1, 0x0);
    pcf50606_write(PCF5060X_LPREGC2, 0x2);

#ifndef BOOTLOADER
    IEN |= EXT3_IRQ_MASK;   /* Unmask EXT3 */
#endif
}

void ide_power_enable(bool on)
{
    (void)on;
}

bool ide_powered(void)
{
    return true;
}

void power_off(void)
{
    /* Forcibly cut power to SoC & peripherals by putting the PCF to sleep */
    pcf50606_write(PCF5060X_OOCC1, GOSTDBY | CHGWAK | EXTONWAK);
}

#ifndef BOOTLOADER
void EXT3(void)
{
    unsigned char data[3]; /* 0 = INT1, 1 = INT2, 2 = INT3 */

    /* Clear pending interrupts from pcf50606 */
    int fiq_status = disable_fiq_save();
    pcf50606_read_multiple(0x02, data, 3);

    if (data[0] & 0x04)
    {
        /* ONKEY1S: reset timeout as we're using SW poweroff */
        pcf50606_write(0x08, pcf50606_read(0x08) | 0x02); /* OOCC1: TOTRST=1 */
    }

    if (data[2] & 0x08)
    {
        /* TODO: Touchscreen pen down event, do something about it */
    }

    restore_fiq(fiq_status);
}
#endif

#if CONFIG_CHARGING
bool charger_inserted(void)
{
    return (GPIOC & (1<<26)) ? false:true;
}
#endif

#else /* SIMULATOR */

bool charger_inserted(void)
{
    return false;
}

void charger_enable(bool on)
{
    (void)on;
}

void power_off(void)
{
}

void ide_power_enable(bool on)
{
   (void)on;
}

#endif /* SIMULATOR */
