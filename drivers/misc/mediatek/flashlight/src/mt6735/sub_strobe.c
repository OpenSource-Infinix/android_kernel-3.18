/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include "mt_gpio.h"//miaowei add
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"

/*miaowei--151224=modify start*/
#define GPIO_CAMERA_SUB_FLASH_EN_PIN   (55 | 0x80000000)
#define GPIO_CAMERA_SUB_FLASH_MODE_PIN    (82 | 0x80000000)
/*miaowei--151224=modify end*/

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

/*miaowei add it */
static int g_duty=-1;

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}
/*add end*/

int sub_FL_Init(void)
{
	mt_set_gpio_mode(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ZERO); 
	return 0;
}

int sub_FL_Enable(void)
{
	 PK_DBG(" sub_FL_Enable g_duty=%d\n", g_duty);
	if (g_duty > 0)
	{
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ONE);

		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ZERO); 
		udelay(3);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ONE);
		udelay(3);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ZERO); 
		udelay(3);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ONE);
		udelay(3);
	}
	else 
	{
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ZERO);
	}

return 0;

}

int sub_FL_Disable(void)
{
/*miaowei--151224=modify start*/
	mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ZERO); 
	mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ZERO);
return 0;
/*miaowei--151224=modify end*/
}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
/*miaowei add it */
	switch(cmd)
	{
		case FLASH_IOC_SET_ONOFF :
			PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
			if(arg==1)
			{
				sub_FL_Enable();
			}
			else
			{
				sub_FL_Disable();
			}
    	break;
		case FLASH_IOC_SET_DUTY :
                        PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
    			FL_dim_duty(arg);
    			break;
			
		
		default :
			PK_DBG(" No such command \n");
			i4RetValue = -EPERM;
    	break;
    }
/*add end*/
    return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
	sub_FL_Init();//add by miaowei
	PK_DBG("sub dummy open");
	return 0;
}

static int sub_strobe_release(void *pArg)
{
	sub_FL_Disable();//add by miaowei
	PK_DBG("sub dummy release");
	return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}


static struct class *sub_torch_class = NULL;
static struct device *sub_torch_dev = NULL;
static unsigned int  rgt_sub_torch_level=0xff;
static ssize_t sub_torch_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	PK_DBG("[zk sub_torch_level_show]get sub_torch value is:%d \n",rgt_sub_torch_level);
	return sprintf(buf, "%u\n", rgt_sub_torch_level);
}

static ssize_t sub_torch_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
		
{
	int value = simple_strtoul(buf, NULL, 0);
    	rgt_sub_torch_level=value;
	mt_set_gpio_mode(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ZERO);

	//Mode select pin
	mt_set_gpio_mode(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ZERO); 

    	if (9 == value)//flash mode,enable flashlight
    	{  	
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ONE);		
    	} 
    	else if(1 == value)//movie mode,enable flashlight
    	{
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ZERO);
     	}
     	else//disable flashlight
     	{
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_MODE_PIN,GPIO_OUT_ZERO); 
		mt_set_gpio_out(GPIO_CAMERA_SUB_FLASH_EN_PIN,GPIO_OUT_ZERO);
     	}
	return size;
}



static DEVICE_ATTR(sub_torch_level, 0664, sub_torch_level_show, sub_torch_level_store); 
// system/class/sub_torch/sub_torch/
static int __init rgt_sub_torch_level_init(void)
{
	//int ret = 0;
    	sub_torch_class = class_create(THIS_MODULE, "sub_torch");

    	if (IS_ERR(sub_torch_class)) {
        	//PK_ERR("[rgt_sub_torch_level_init] Unable to create class, err = %d\n", (int)PTR_ERR(sub_torch_class));
       		return 0;
    	}

    	sub_torch_dev = device_create(sub_torch_class, NULL, 0, 0,"sub_torch");

    	if(NULL == sub_torch_dev){
  		//PK_ERR("[rgt_sub_torch_level_init] device_create fail\n");
       		return 0;
    	}

 	device_create_file(sub_torch_dev, &dev_attr_sub_torch_level); 

    	PK_DBG("[rgt_sub_torch_level_init] Done\n");

    	return 0;
}

static void __exit rgt_sub_torch_level_exit(void)
{
       device_remove_file(sub_torch_dev, &dev_attr_sub_torch_level); 
       device_unregister(sub_torch_dev);
	if(sub_torch_class!=NULL)
	       class_destroy(sub_torch_class);
}

 
module_init(rgt_sub_torch_level_init);
module_exit(rgt_sub_torch_level_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xzk>");
MODULE_DESCRIPTION("sub_torch value");
