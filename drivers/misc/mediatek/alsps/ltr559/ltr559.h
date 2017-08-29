/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for ltr559 als/ps sensor chip.
 */



#ifndef __LTR559_H__
#define __LTR559_H__

#include <linux/ioctl.h>

#define LTR559_I2C_SLAVE_ADDR 0x46//0x46 // 0x23 << 1

extern int ltr559_CMM_PPCOUNT_VALUE;
extern int ltr559_CMM_CONTROL_VALUE;
extern int ZOOM_TIME;


/*REG address*/

#define APS_RW_ALS_CONTR		0x80 // ALS operation mode control SW reset
#define APS_RW_PS_CONTR			0x81 // PS operation mode control
#define APS_RW_PS_LED			0x82 // PS LED setting
#define APS_RW_PS_N_PULSES		0x83 // PS number of pulses
#define APS_RW_PS_MEAS_RATE		0x84 // PS measurement rate in active mode
#define APS_RW_ALS_MEAS_RATE		0x85 // ALS measurement rate in active mode
#define APS_RO_PART_ID			0x86 // Part Number ID and Revision ID
#define APS_RO_MANUFAC_ID		0x87 // Manufacturer ID
#define APS_RO_ALS_DATA_CH1_0		0x88 // ALS measurement CH1 data, lower byte
#define APS_RO_ALS_DATA_CH1_1		0x89 // ALS measurement CH1 data, upper byte
#define APS_RO_ALS_DATA_CH0_0		0x8A // ALS measurement CH0 data, lower byte
#define APS_RO_ALS_DATA_CH0_1		0x8B // ALS measurement CH0 data, upper byte
#define APS_RO_ALS_PS_STATUS		0x8C // ALS and PS new data status
#define APS_RO_PS_DATA_0		0x8D // PS measurement data, lower byte
#define APS_RO_PS_DATA_1		0x8E // PS measurement data, upper byte
#define APS_RW_INTERRUPT		0x8F // Interrupt settings
#define APS_RW_PS_THRES_UP_0		0x90 // PS interrupt upper threshold, lower byte
#define APS_RW_PS_THRES_UP_1		0x91 // PS interrupt upper threshold, upper byte
#define APS_RW_PS_THRES_LOW_0		0x92 // PS interrupt lower threshold, lower byte
#define APS_RW_PS_THRES_LOW_1		0x93 // PS interrupt lower threshold, upper byte
#define APS_RW_ALS_THRES_UP_0		0x97 // ALS interrupt upper threshold, lower byte
#define APS_RW_ALS_THRES_UP_1		0x98 // ALS interrupt upper threshold, upper byte
#define APS_RW_ALS_THRES_LOW_0		0x99 // ALS interrupt lower threshold, lower byte
#define APS_RW_ALS_THRES_LOW_1		0x9A // ALS interrupt lower threshold, upper byte
#define APS_RW_INTERRUPT_PERSIST	0x9E // ALS / PS Interrupt persist setting

/* Basic Operating Modes */
// FIXME:
// We should not enable the als and ps by default

// #define MODE_ALS_ON_Range1		0x0B
// #define MODE_ALS_ON_Range2		0x03
#define MODE_ALS_ON_Range1		(0x1 << 3)
#define MODE_ALS_ON_Range2		(0x0 << 3)
#define MODE_ALS_StdBy			0x00

// #define MODE_PS_ON_Gain1		0x03
// #define MODE_PS_ON_Gain4		0x07
// #define MODE_PS_ON_Gain8		0x0B
// #define MODE_PS_ON_Gain16		0x0F
#define MODE_PS_ON_Gain1		(0x0 << 2)
#define MODE_PS_ON_Gain4		(0x1 << 2)
#define MODE_PS_ON_Gain8		(0x2 << 2)
#define MODE_PS_ON_Gain16		(0x3 << 2)
#define MODE_PS_StdBy			0x00

#define PS_RANGE1			1
#define PS_RANGE2			2
#define PS_RANGE4			4
#define PS_RANGE8			8

#define ALS_RANGE1_320			1
#define ALS_RANGE2_64K			2

/* Power On response time in ms */
#define PON_DELAY			600
#define WAKEUP_DELAY			10


#define ltr559_SUCCESS						0
#define ltr559_ERR_I2C						-1
#define ltr559_ERR_STATUS					-3
#define ltr559_ERR_SETUP_FAILURE				-4
#define ltr559_ERR_GETGSENSORDATA			-5
#define ltr559_ERR_IDENTIFICATION			-6



#endif


