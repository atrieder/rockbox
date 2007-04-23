/*
 * This config file is for the Sandisk Sansa e200
 */
#define TARGET_TREE /* this target is using the target tree system */

/* For Rolo and boot loader */
#define MODEL_NUMBER 16
#define MODEL_NAME   "Sandisk Sansa e200"

/* define this if you have recording possibility */
/*#define HAVE_RECORDING*/ /* TODO: add support for this */

/* define this if you have a bitmap LCD display */
#define HAVE_LCD_BITMAP

/* define this if you have a colour LCD */
#define HAVE_LCD_COLOR

/* define this if you have LCD enable function */
#define HAVE_LCD_ENABLE

#define HAVE_BACKLIGHT_BRIGHTNESS
/* Main LCD backlight brightness range and defaults */
#define MIN_BRIGHTNESS_SETTING           1
#define MAX_BRIGHTNESS_SETTING           12
#define DEFAULT_BRIGHTNESS_SETTING       6

/* define this if you have a light associated with the buttons */
#define HAVE_BUTTON_LIGHT

/* define this if you have access to the quickscreen */
#define HAVE_QUICKSCREEN

/* define this if you would like tagcache to build on this target */
#define HAVE_TAGCACHE

/* LCD dimensions */
#define LCD_WIDTH  176
#define LCD_HEIGHT 220
#define LCD_DEPTH  16   /* 65536 colours */
#define LCD_PIXELFORMAT RGB565 /* rgb565 */

/* #define IRAM_LCDFRAMEBUFFER IDATA_ATTR *//* put the lcd frame buffer in IRAM */

#define CONFIG_KEYPAD SANSA_E200_PAD

/* Define this if you do software codec */
#define CONFIG_CODEC SWCODEC
/* There is no hardware tone control */
#define HAVE_SW_TONE_CONTROLS
/* The PP5024 has a built-in AustriaMicrosystems AS3514 */
#define HAVE_AS3514

/* define this if you have a real-time clock */
#ifndef BOOTLOADER
#define CONFIG_RTC RTC_AS3514
#endif

/* Define this if you have a software controlled poweroff */
#define HAVE_SW_POWEROFF

/* Some Sansa E200s seem to be FAT16 formatted */
#define HAVE_FAT16SUPPORT

/* The number of bytes reserved for loadable codecs */
#define CODEC_SIZE 0x80000

/* The number of bytes reserved for loadable plugins */
#define PLUGIN_BUFFER_SIZE 0x80000

#define AB_REPEAT_ENABLE 1

/* FM Tuner */
/*#define CONFIG_TUNER TEA5767
#define CONFIG_TUNER_XTAL  32768 *//* TODO: what is this? */

/* Define this for LCD backlight available */
#define HAVE_BACKLIGHT

#define BATTERY_CAPACITY_DEFAULT 750    /* default battery capacity */

#ifndef SIMULATOR

/* Define this if you have a PortalPlayer PP5024 */
#define CONFIG_CPU PP5024

/* Define this if you want to use the PP5020 i2c interface */
#define CONFIG_I2C I2C_PP5020

/* Type of mobile power */
#define CONFIG_BATTERY BATT_LIION750
#define BATTERY_CAPACITY_MIN 750        /* min. capacity selectable */
#define BATTERY_CAPACITY_MAX 750        /* max. capacity selectable */
#define BATTERY_CAPACITY_INC 0          /* capacity increment */
#define BATTERY_TYPES_COUNT  1          /* only one type */
#define BATTERY_SCALE_FACTOR 5005       /* ADC should read 0x3ff=5.12V */

/* Hardware controlled charging? FIXME */
#define CONFIG_CHARGING CHARGING_SIMPLE

/* define this if the hardware can be powered off while charging */
/* Sansa can't be powered off while charging */
/* #define HAVE_POWEROFF_WHILE_CHARGING */

/* The start address index for ROM builds */
#define ROM_START 0x00000000

/* Define this to the CPU frequency */
#define CPU_FREQ      75000000

/* Type of LCD TODO: hopefully the same as the x5 but check this*/
#define CONFIG_LCD LCD_X5

/* Offset ( in the firmware file's header ) to the file CRC and data. These are
   only used when loading the old format rockbox.e200 file */
#define FIRMWARE_OFFSET_FILE_CRC    0x0
#define FIRMWARE_OFFSET_FILE_DATA   0x8

/* #define USB_IPODSTYLE */

/* USB On-the-go */
#define CONFIG_USBOTG USBOTG_ARC

/* define this if the unit can be powered or charged via USB */
#define HAVE_USB_POWER

/* Virtual LED (icon) */
#define CONFIG_LED LED_VIRTUAL

/* Define this if you have adjustable CPU frequency */
/*#define HAVE_ADJUSTABLE_CPU_FREQ*/

#define BOOTFILE_EXT    "mi4"
#define BOOTFILE        "rockbox." BOOTFILE_EXT
#define OLD_BOOTFILE    "rockbox.e200"
#define BOOTDIR "/.rockbox"

#define ICODE_ATTR_TREMOR_NOT_MDCT

#endif
