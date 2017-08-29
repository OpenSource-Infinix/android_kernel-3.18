/*Transsion Top Secret*/
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
 *   John Wei (MTK07407)
 *
 *============================================================================*/
#ifndef __S5K3H7YX_H
#define __S5K3H7YX_H
typedef unsigned short  kal_uint16;
typedef unsigned char BYTE;
typedef unsigned char kal_uint8;
#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define S5K3H7YX_EEPROM_DEVICE_ID							0x20//0xA0/*slave id of imx135*/


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para);
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr);
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para);
static bool s5k3h7_start_read_otp(BYTE zone);
static void s5k3h7_stop_read_otp(void);
int s5k3h7_check_otp(int index);
int read_otp_af(int index, u32 addr, BYTE *data,int lenth);
int s5k3h7_read_otp_af(u32 addr, BYTE* data,u32 length);

#endif/* __CAM_CAL_H */

