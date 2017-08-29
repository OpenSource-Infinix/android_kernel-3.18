/*

SiI8348 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/
#if !defined(SII_HAL_H)
#define SII_HAL_H
#include <linux/kernel.h>

#ifdef __cplusplus 
extern "C" { 
#endif

#ifndef	FALSE
#define	FALSE	false
#endif

#ifndef	TRUE
#define	TRUE	true
#endif

#define MHL_PRODUCT_NUM 8348
#define MHL_DRIVER_NAME "sii8348drv"
#define MHL_DEVICE_NAME "sii-8348"

#define CONFIG_DEBUG_DRIVER
#define RCP_INPUTDEV_SUPPORT
#define ENABLE_GEN2
#define MHL2_ENHANCED_MODE_SUPPORT
#define MEDIA_DATA_TUNNEL_SUPPORT
#define ENABLE_EDID_INFO_PRINT
#define ENABLE_EDID_DEBUG_PRINT
///#define ENABLE_DUMP_INFOFRAME

#ifdef __cplusplus
}
#endif  

#endif 
