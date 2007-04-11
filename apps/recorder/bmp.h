/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2002 by Daniel Stenberg
 *
 * All files in this archive are subject to the GNU General Public License.
 * See the file COPYING in the source tree root for full license agreement.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#ifndef _BMP_H_
#define _BMP_H_

#include "config.h"
#include "lcd.h"

/*********************************************************************
 * read_bmp_file()
 *
 * Reads a 8bit BMP file and puts the data in a 1-pixel-per-byte
 * array. 
 * Returns < 0 for error, or number of bytes used from the bitmap buffer
 *
 **********************************************/
int read_bmp_file(char* filename,
                  struct bitmap *bm,
                  int maxsize,
                  int format);
#endif
