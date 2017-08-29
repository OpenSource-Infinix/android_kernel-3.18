/*****************************************************************************
 *
 * Filename:
 * ---------
 *   catc24c16.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   LukeHu
 *
 *============================================================================*/
#ifndef __S5K2P8_EEPROM_H
#define __S5K2P8_EEPROM_H
#define BYTE               unsigned char


#define S5K3L8_OTP_DEVICE_ID        0x50


#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_RED_ADDR      0x0210
#define GAIN_GREEN2_ADDR   0x0214
/**********************define for addr ********************/
#define S5K3L8_OTP_SUCESS 0x0000
#define S5K3L8_OTP_FIRST_PIXEL 0x0002
#define S5K3L8_OTP_MIRROR_FLIP 0x0003
#define S5K3L8_OTP_FLAG 0x0011

#define I2C_SPEED 100
extern void kdSetI2CSpeed(u32 i2cSpeed);
static struct i2c_client * g_pstI2Cclient = NULL;

#endif /* __CAM_CAL_H */

