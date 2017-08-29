/*Transsion Top Secret*/
/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k3h7_otp.h"
/* #include <asm/system.h>  // for SMP */
#include <linux/dma-mapping.h>

//#define  CONFIG_COMPAT
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


/* #define CAM_CALGETDLT_DEBUG */
//#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define PFX "s5k3h7yx"

#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALDB(x, ...)
#define CAM_CALERR(x, ...)
#define CAM_CALINF(x, ...)
#endif
#define PAGE_SIZE_ 256
#define BUFF_SIZE 8

static DEFINE_SPINLOCK(g_CAM_CALLock); /* for SMP */
#define CAM_CAL_I2C_BUSNUM 0
BYTE afotpdata[3]={0};
int flag_afotpread = 0;
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 /* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
/*******************************************************************************
*
********************************************************************************/
/* A0 for page0 A2 for page 2 and so on for 8 pages */

/* 81 is used for V4L driver */
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER, 0);
static struct cdev *g_pCAM_CAL_CharDrv;
/* static spinlock_t g_CAM_CALLock; */
/* spin_lock(&g_CAM_CALLock); */
/* spin_unlock(&g_CAM_CALLock); */

static struct class *CAM_CAL_class;
static atomic_t g_CAM_CALatomic;
/* static DEFINE_SPINLOCK(kdcam_cal_drv_lock); */
/* spin_lock(&kdcam_cal_drv_lock); */
/* spin_unlock(&kdcam_cal_drv_lock); */

#if 1
/***************************songqi add otp for AF******************************/
static bool AWBOtpRead = false;
static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, S5K3H7YX_EEPROM_DEVICE_ID);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,S5K3H7YX_EEPROM_DEVICE_ID);
    return get_byte;
}
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, S5K3H7YX_EEPROM_DEVICE_ID);
}
/*************************************************************************************************
* Function    :  start_read_otp for s5k3h7
* Description :  before read otp , set the reading block setting  
* Parameters  :  zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
                 1, reading block setting ok 
**************************************************************************************************/
static bool s5k3h7_start_read_otp(BYTE zone)
{
  	BYTE val = 0;
	char i = 0;

  	write_cmos_sensor(0xFCFC, 0xD000);
	write_cmos_sensor_8(0x0A02, zone);//Select the page to write by writing to 0xD0000A02 0x01~0x0C
	write_cmos_sensor_8(0x0A00, 0x01);//Enter read mode by writing 01h to 0xD0000A00

	for(i = 0;i <= 100;i++)
	{
		val = read_cmos_sensor_8(0x0A01);
		if(0x01 == val)
			break;
		mdelay(2);
	}
	CAM_CALDB("[S5K3H7Y] otp start i = %d\n",i);
	if(100 == i)
	{
		write_cmos_sensor_8(0x0A00, 0x00);
		return false;
	}
	
	return true;
}

static void s5k3h7_stop_read_otp(void)
{
	char i = 0;
	BYTE val = 0x00;

	for(i = 0;i <= 100;i++)
	{
		val = read_cmos_sensor_8(0x0A01);
		if(0x01 == val)
			break;
		mdelay(2);
	}
	CAM_CALDB("[S5K3H7Y] otp stop i = %d\n",i);
	write_cmos_sensor_8(0x0A00, 0x00);   //Reset the NVM interface by writing 00h to 0xD0000A00

	return;
}


// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int s5k3h7_check_otp(int index)
{
	int flag = 0;
	BYTE zone = 0x01;
	
	if(!s5k3h7_start_read_otp(zone)) //page 1
	{
		CAM_CALDB("[S5K3H7Y]Start read Page %d Fail!\n", zone);
		return 0;
	}
	
	if(index==3)
	{
		flag=read_cmos_sensor_8(0x0A2C);
		CAM_CALDB("[s5k3h7_check_otp] group3 flag=%x\n",flag); 
	} 
	else if(index==2)
	{
		flag=read_cmos_sensor_8(0x0A24);
		CAM_CALDB("[s5k3h7_check_otp] group2 flag=%x\n",flag);	  
	}  
	else if(index==1)
	{
		flag=read_cmos_sensor_8(0x0A1C);
		CAM_CALDB("[s5k3h7_check_otp] group1 flag=%x\n",flag);	  
	}
  
	s5k3h7_stop_read_otp(); //reset NVM
  
	if(flag==0x55)
	{
		return true;
	}
	else
	{
	  	return false;
	}
}

int read_otp_af(int index, u32 addr, BYTE *data,int lenth)
{
	BYTE zone = 0x01;
	BYTE *buf = data;

	if(!s5k3h7_start_read_otp(zone)) //page 1
	{
		CAM_CALDB("[S5K3H7] Start read Page %d Fail!\n",zone);
		return false;
	}
	
	switch(index)
	{
  		case 3:
			 buf[0] = read_cmos_sensor_8(0x0A30);
    			buf[1]=read_cmos_sensor_8(0x0A2D);
			buf[2]=read_cmos_sensor_8(0x0A2E);
			AWBOtpRead = true;
			break;
  		case 2:
			buf[0] = read_cmos_sensor_8(0x0A28);
    			buf[1]=read_cmos_sensor_8(0x0A25);
			buf[2]=read_cmos_sensor_8(0x0A26);
			AWBOtpRead = true;			
			break;
  		case 1:
			buf[0] = read_cmos_sensor_8(0x0A20);
    			buf[1]=read_cmos_sensor_8(0x0A1D);
			buf[2]=read_cmos_sensor_8(0x0A1E);
			AWBOtpRead = true;			
			break;
		default:
			AWBOtpRead = false;
			CAM_CALDB("[S5K3H7Y] otp read AWB gain value fail!\n");
			break;
	}

  	CAM_CALDB("[S5K3H7Y] otp index = %d,af_high=0x%x,af_macro_low=0x%x,af_inf_low=0x%x\n",index,buf[0],buf[1],buf[2]);
	afotpdata[0]= buf[0];
	afotpdata[1]= buf[1];
	afotpdata[2]= buf[2];
	flag_afotpread = 1;
	s5k3h7_stop_read_otp(); //reset NVM
  
  	return true;
}

int s5k3h7_read_otp_af(u32 addr, BYTE* data,u32 length)
{
	int otp_index = 1;
	int i;	
	int temp;
	if(addr == 0x07)
	{
		*data = 0x08;
		CAM_CALDB("[cam_cal]check layout\n");
	}
	else
	{
		if(flag_afotpread !=1)
		{
			for(i=1;i<=3;i++) 
			{
				temp = s5k3h7_check_otp(i);
				if (temp) 
				{
					otp_index = i;
					break;
				}
			}
			//CAM_CALDB("[cam_cal]i = %d\n",i);
			if (i>3) 
			{
				// no valid wb OTP data
				CAM_CALDB("[cam_cal] no valid wb OTP data\n");
				return 1;
			}			
			read_otp_af(otp_index,addr,data,length);
		}
		else
		{
			data[0] = afotpdata[0];
			data[1] = afotpdata[1];
			data[2] = afotpdata[2];
			CAM_CALDB("[cam_cal]af data = %d,%d,%d\n",data[0],data[1],data[2]);
		}
		
	}
	return 0;
}
/***************************************end******************************************/
#endif

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	//compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->u4Offset);
	err |= put_user(i, &data32->u4Offset);
	err |= get_user(i, &data->u4Length);
	err |= put_user(i, &data32->u4Length);
	/* Assume pointer is not change */
#if 0
	err |= get_user(p, &data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->u4Offset);
	err |= put_user(i, &data->u4Offset);
	err |= get_user(i, &data32->u4Length);
	err |= put_user(i, &data->u4Length);
	err |= get_user(p, &data32->pu1Params);
	err |= put_user(compat_ptr(p), &data->pu1Params);

	return err;
}

static long s5k3h7yx_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	int err;

	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;

	CAM_CALDB("[CAMERA SENSOR] s5k3h7yx_Ioctl_Compat,%p %p %x ioc size %d\n",
	filp->f_op , filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {

	case COMPAT_CAM_CALIOC_G_READ: {
		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_cal_info_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ, (unsigned long)data);
		err = compat_put_cal_info_struct(data32, data);


		if (err != 0)
			CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	default:
		return -ENOIOCTLCMD;
	}
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
			 struct file *a_pstFile,
			 unsigned int a_u4Command,
			 unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
	struct file *file,
	unsigned int a_u4Command,
	unsigned long a_u4Param
)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pu1Params = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;

	CAM_CALDB("[S5K3H7CAM_CAL] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			CAM_CALDB(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/* get input structure address */
				kfree(pBuff);
				CAM_CALDB("[S5K3H7CAM_CAL] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
	pu1Params = kmalloc(ptempbuf->u4Length, GFP_KERNEL);

	if (NULL == pu1Params) {
		kfree(pBuff);
		CAM_CALDB("ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	//CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);
	if (copy_from_user((u8 *)pu1Params , (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pu1Params);
		CAM_CALDB("[S5K3H7CAM_CAL] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
	case CAM_CALIOC_S_WRITE:
		CAM_CALDB("[S5K3H7CAM_CAL] Write CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif

#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
		CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;
	case CAM_CALIOC_G_READ:
		CAM_CALDB("[S5K3H7CAM_CAL] Read CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
	//	CAM_CALDB("[CAM_CAL] ptempbuf->u4Offset 0x%x,length = %d\n", ptempbuf->u4Offset,ptempbuf->u4Length);
		/* CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%p\n", pu1Params); */
		s5k3h7_read_otp_af(ptempbuf->u4Offset, pu1Params,ptempbuf->u4Length);

#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
		CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;

	default:
		CAM_CALDB("[S5K3H7CAM_CAL] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		CAM_CALDB("[cam_cal] copy to user:%d, %d\n", *pu1Params, ptempbuf->u4Length);

		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pu1Params);
			CAM_CALDB("[S5K3H7CAM_CAL] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pu1Params);

	return i4RetValue;
}


static u32 g_u4Opened;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	//CAM_CALDB("[S5K3H7CAM_CAL] CAM_CAL_Open\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S5K3H7CAM_CAL] Opened, return -EBUSY\n");
		return -EBUSY;
	} /*else {*//*LukeHu--150720=For check patch*/
	if (!g_u4Opened) {/*LukeHu--150720=For check patch*/
		g_u4Opened = 1;
		atomic_set(&g_CAM_CALatomic, 0);
	}
	spin_unlock(&g_CAM_CALLock);
	mdelay(2);
	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/* .ioctl = CAM_CAL_Ioctl */
#ifdef CONFIG_COMPAT
	.compat_ioctl = s5k3h7yx_Ioctl_Compat,
#endif
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
static inline int RegisterCAM_CALCharDrv(void)
{
	struct device *CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[S5K3H7CAM_CAL] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME)) {
		CAM_CALDB("[S5K3H7CAM_CAL] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/* Allocate driver */
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (NULL == g_pCAM_CAL_CharDrv) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[S5K3H7CAM_CAL] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/* Attatch file operation. */
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALDB("[S5K3H7CAM_CAL] Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	printk("songqi: create class\n");
	CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);

		CAM_CALDB("Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
	CAM_CALDB(" create device success\n");

	return 0;
}

static inline void UnregisterCAM_CALCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}


/* //////////////////////////////////////////////////////////////////// */
static int CAM_CAL_probe(struct platform_device *pdev)
{
	/* Register char driver */
	int i4RetValue = 0;
	i4RetValue = RegisterCAM_CALCharDrv();
	if (i4RetValue) {
		CAM_CALDB("[S5K3H7CAM_CAL] register char device failed!\n");
		return i4RetValue;
	}
	return 0;
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe              = CAM_CAL_probe,
	.remove     = CAM_CAL_remove,
	.driver             = {
		.name   = CAM_CAL_DRVNAME,
		.owner  = THIS_MODULE,
	}
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init CAM_CAL_i2C_init(void)
{
	if (platform_driver_register(&g_stCAM_CAL_Driver)) {
		CAM_CALDB("failed to register S5K3H7CAM_CAL driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALDB("failed to register S5K3H7CAM_CAL device\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);


MODULE_DESCRIPTION("CAMERA_CAM_CAL driver");
MODULE_AUTHOR("Jackie Su <jackie.su@Mediatek.com>");
MODULE_LICENSE("GPL");

