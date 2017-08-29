/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_WAKEUP_TRIAL	60
#define TPD_WAKEUP_DELAY	100



#define TPD_DELAY		(2*HZ/100)

/*focaltech register*/
#define FT_GESTRUE_MODE_SWITCH_REG 0xD0
#define FT_GESTRUE_GETID_REG 0xD3

#define TPD_RES_X		800
#define TPD_RES_Y		1280


#define TPD_CALIBRATION_MATRIX_ROTATION_NORMAL  {-4096, 0, 800*4096, 0, -4096, 1280*4096, 0, 0}
#define TPD_CALIBRATION_MATRIX_ROTATION_FACTORY {-4096, 0, 800*4096, 0, -4096, 1280*4096, 0, 0}




#define FTP_DEBUG_ON                   0
#define FTP_ERROR(fmt,arg...)          printk(KERN_CRIT"<FTP-ERR>"fmt"\n", ##arg)
#define FTP_INFO(fmt,arg...)           printk(KERN_CRIT"<FTP-INF>"fmt"\n", ##arg)

#if FTP_DEBUG_ON
#define FTP_DEBUG(fmt,arg...)          do{\
					 printk(KERN_CRIT"<FTP-DBG>"fmt"\n", ##arg);\
				       }while(0)
#else
#define FTP_DEBUG(fmt,arg...)
#endif


typedef void (*GES_CBFUNC)(u8);
/*****************************************************************************
 * ENUM
 ****************************************************************************/
enum GTP_WORK_STATE {
	GTP_UNKNOWN = 0,
	GTP_NORMAL,
	GTP_DOZE,
	GTP_SLEEP,
};

enum TOUCH_DOZE_T1 {
	DOZE_INPOCKET = 0,
	DOZE_NOT_INPOCKET = 1,
};

enum TOUCH_DOZE_T2 {
	DOZE_DISABLE = 0,
	DOZE_ENABLE = 1,
};

enum TOUCH_WAKE_T {
	TOUCH_WAKE_BY_NONE,
	TOUCH_WAKE_BY_INT,
	TOUCH_WAKE_BY_IPI,
	TOUCH_WAKE_BY_SWITCH
};

/*****************************************************************************
 * STRUCTURE
 ****************************************************************************/
struct Touch_SmartWake_ID {
	u8 id;
	GES_CBFUNC cb;
};

/* #define TPD_HAVE_TREMBLE_ELIMINATION */


extern struct tpd_device *tpd;
extern unsigned int tpd_rst_gpio_number;
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
#ifdef CONFIG_CUST_FTS_APK_DEBUG
extern int ft_rw_iic_drv_init(struct i2c_client *client);
extern void  ft_rw_iic_drv_exit(void);
int ft5x0x_create_apk_debug_channel(struct i2c_client * client);

#endif

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
extern u8 *tpd_i2c_dma_va;
extern dma_addr_t tpd_i2c_dma_pa;
extern int tpd_auto_upgrade(struct i2c_client *client);
#endif
#endif /* TOUCHPANEL_H__ */
