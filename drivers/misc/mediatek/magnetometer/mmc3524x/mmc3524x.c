
/* mmc3524x.c - mmc3524x compass driver
 *
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



#define MEMSIC_SINGLE_POWER 0

/*-------------------------MT6516&MT6573 define-------------------------------*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include "cust_mag.h"
#include "mmc3524x.h"
#include "mag.h"

/*----------------------------------------------------------------------------*/
#define DEBUG 0
#define MMC3524X_DEV_NAME         "mmc3524x"
#define DRIVER_VERSION          "1.0.0"
/*----------------------------------------------------------------------------*/
#define MMC3524X_DEBUG		1
#define MMC3524X_DEBUG_MSG	1
#define MMC3524X_DEBUG_FUNC	1
#define MMC3524X_DEBUG_DATA	1
#define MAX_FAILURE_COUNT	3
#define MMC3524X_RETRY_COUNT	3
#define MMC3524X_DEFAULT_DELAY	100
//#define MMC3524X_BUFSIZE  0x20
//modify stack overfow added by peixuan.qiu
#define MMC3524X_BUFSIZE sizeof(struct hwm_sensor_data)


#if MMC3524X_DEBUG_MSG
#define MMCDBG(format, ...)	printk(KERN_INFO "mmc3524x " format "\n", ## __VA_ARGS__)
#else
#define MMCDBG(format, ...)
#endif

#if MMC3524X_DEBUG_FUNC
#define MMCFUNC(func) printk(KERN_INFO "mmc3524x " func " is called\n")
#else
#define MMCFUNC(func)
#endif


extern struct mag_hw* get_cust_mag_hw(void);

static struct i2c_client *this_client = NULL;
#if 0
extern struct platform_driver m_sensor_driver ;
extern bool m_sensor_deteced ;
#endif

// calibration msensor and orientation data
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
static struct mutex sense_data_mutex;
static struct mutex read_i2c_xyz;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int mmcd_delay = MMC3524X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mmc3524x_i2c_id[] = {{MMC3524X_DEV_NAME,0},{}};
struct i2c_board_info __initdata i2c_mmc3524x={ I2C_BOARD_INFO("mmc3524x", (0x60>>1))};

/*the adapter id will be available in customization*/
//static unsigned short mmc3524x_force[] = {0x00, MMC3524x_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const mmc3524x_forces[] = { mmc3524x_force, NULL };
//static struct i2c_client_address_data mmc3524x_addr_data = { .forces = mmc3524x_forces,};
/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mmc3524x_i2c_remove(struct i2c_client *client);
//static int mmc3524x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
//static int platform_mmc3524x_probe(struct platform_device *pdev);
//static int platform_mmc3524x_remove(struct platform_device *pdev);

static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg);
static int mmc3524x_resume(struct i2c_client *client);


/*----------------------------------------------------------------------------*/
typedef enum {
	MMC_FUN_DEBUG  = 0x01,
	MMC_DATA_DEBUG = 0X02,
	MMC_HWM_DEBUG  = 0X04,
	MMC_CTR_DEBUG  = 0X08,
	MMC_I2C_DEBUG  = 0x10,
} MMC_TRC;

/* Define Delay time */
#define MMC3524X_DELAY_TM		10	/* ms */
#define MMC3524X_DELAY_SET		50	/* ms */
#define MMC3524X_DELAY_RESET	50  /* ms */
#define MMC3524X_DELAY_RST	50	/*ms*/
#define MMC3524X_DELAY_STDN		1	/* ms */

#define MMC3524X_RESET_INTV		250
static u32 read_idx = 0;
#define READMD	0

/*----------------------------------------------------------------------------*/
struct mmc3524x_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

static int  mmc3524x_local_init(void);
static int mmc3524x_remove(void);

static int mmc3524x_init_flag = 0; // 0<==>OK -1 <==> fail

struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

static struct mag_init_info mmc3524x_init_info = {
	 .name = "mmc3524x",
	 .init = mmc3524x_local_init,
	 .uninit = mmc3524x_remove,
};

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
		{.compatible = "mediatek,msensor"},
		{},
};
#endif
static struct i2c_driver mmc3524x_i2c_driver = {
    .driver = {
     //   .owner = THIS_MODULE,
        	.name  = MMC3524X_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = mag_of_match,
#endif
    },
	.probe      = mmc3524x_i2c_probe,
	.remove     = mmc3524x_i2c_remove,
	//.detect     = mmc3524x_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = mmc3524x_suspend,
	.resume     = mmc3524x_resume,
#endif
	.id_table = mmc3524x_i2c_id,
	//.address_data = &mmc3524x_addr_data,
};

/*----------------------------------------------------------------------------*/
/*
 struct platform_driver mmc3524x_platform_driver = {
	.probe      = platform_mmc3524x_probe,
	.remove     = platform_mmc3524x_remove,
	.driver     = {
		.name  = "msensor",
		//.owner = THIS_MODULE,
	}
};
*/

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
#if 0
static int mmc3524x_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	u8 addr = MMC3524X_REG_CTRL;
	//struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);

	if(hwmsen_read_byte(client, addr, databuf))
	{
		printk("mmc3524x: read power ctl register err and retry!\n");
		if(hwmsen_read_byte(client, addr, databuf))
	    {
		   printk("mmc3524x: read power ctl register retry err!\n");
		   return -1;
	    }
	}

	databuf[0] &= ~MMC3524X_CTRL_TM;

	if(enable)
	{
		databuf[0] |= MMC3524X_CTRL_TM;
	}
	else
	{
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = MMC3524X_REG_CTRL;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		printk("mmc3524x: set power mode failed!\n");
		return -1;
	}
	else
	{
		printk("mmc3524x: set power mode ok %x!\n", databuf[1]);
	}

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static void mmc3524x_power(struct mag_hw *hw, unsigned int on)
{
#if 0
	static unsigned int power_on = 0;

	if(hw->power_id != MT65XX_POWER_NONE)
	{
		MMCDBG("power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			MMCDBG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "mmc3524x"))
			{
				printk(KERN_ERR "power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "mmc3524x"))
			{
				printk(KERN_ERR "power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif
}

static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif

	/* Caller should check parameter validity.*/
	if((rxData == NULL) || (length < 1))
	{
		return -EINVAL;
	}

	for(loop_i = 0; loop_i < MMC3524X_RETRY_COUNT; loop_i++)
	{
		this_client->addr = ((this_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;
		if(i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01)))
		{
			break;
		}
		printk("I2C_RxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC3524X_RETRY_COUNT)
	{
		printk(KERN_ERR "%s retry over %d\n", __func__, MMC3524X_RETRY_COUNT);
		return -EIO;
	}
#if DEBUG
	if(atomic_read(&data->trace) & MMC_I2C_DEBUG)
	{
		printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for(i = 0; i < length; i++)
		{
			printk(KERN_INFO " %02x", rxData[i]);
		}
	    printk(KERN_INFO "\n");
	}
#endif
	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < MMC3524X_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(this_client, (const char*)txData, length) > 0)
		{
			break;
		}
		printk("I2C_TxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC3524X_RETRY_COUNT)
	{
		printk(KERN_ERR "%s retry over %d\n", __func__, MMC3524X_RETRY_COUNT);
		return -EIO;
	}
#if DEBUG
	if(atomic_read(&data->trace) & MMC_I2C_DEBUG)
	{
		printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
		{
			printk(KERN_INFO " %02x", txData[i + 1]);
		}
		printk(KERN_INFO "\n");
	}
#endif
	return 0;
}

// Daemon application save the data
static int ECS_SaveData(int buf[CALIBRATION_DATA_SIZE])
{
#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

#if DEBUG
	if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
	{
		MMCDBG("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	}
#endif

	return 0;
}

static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0,0,0,0,0,0};
	//ktime_t expires;
	//int wait_n=0;
	//int MD_times = 0;
	static int last_data[3];
	struct timespec time1, time2, time3;
#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);
#endif
//set_current_state(TASK_INTERRUPTIBLE);
time1 = current_kernel_time();

	if(size < 3)
	{
		return -1;
	}
	mutex_lock(&read_i2c_xyz);
	time2 = current_kernel_time();

	if (!(read_idx % MMC3524X_RESET_INTV))
	{
#if MEMSIC_SINGLE_POWER
		/* Reset Sensor Periodly SET */
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_REFILL;
		/* not check return value here, assume it always OK */
		I2C_TxData(data, 2);
		/* wait external capacitor charging done for next RM */
		msleep(MMC3524X_DELAY_SET);
#endif
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_SET;
		I2C_TxData(data, 2);
		msleep(1);
		data[0] = MMC3524X_REG_CTRL;
		data[1] = 0;
		I2C_TxData(data, 2);
		msleep(1);

	}

	time3 = current_kernel_time();
	/* send TM cmd before read */
	data[0] = MMC3524X_REG_CTRL;
	data[1] = MMC3524X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
	msleep(MMC3524X_DELAY_TM);

#if READMD
	/* Read MD */
	data[0] = MMC3524X_REG_DS;
	I2C_RxData(data, 1);
	while (!(data[0] & 0x01)) {
		msleep(1);
		/* Read MD again*/
		data[0] = MMC3524X_REG_DS;
		I2C_RxData(data, 1);
		if (data[0] & 0x01) break;
		MD_times++;
		if (MD_times > 3) {
			printk("TM not work!!");
			mutex_unlock(&read_i2c_xyz);
			return -EFAULT;
		}
	}
#endif
	read_idx++;
	data[0] = MMC3524X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}
	vec[0] = data[1] << 8 | data[0];
	vec[1] = data[3] << 8 | data[2];
	vec[2] = data[5] << 8 | data[4];
	vec[2] = 65536 - vec[2] ;
#if DEBUG
	if(atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
	{
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
	}
#endif
	mutex_unlock(&read_i2c_xyz);
	last_data[0] = vec[0];
	last_data[1] = vec[1];
	last_data[2] = vec[2];
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err = 0;
	err = ECS_ReadXYZData(data, 3);
	if(err !=0 )
	{
		printk(KERN_ERR "MMC3524X_IOC_TM failed\n");
		return -1;
	}

	// sensitivity 2048 count = 1 Guass = 100uT
	data[0] = (data[0] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	data[1] = (data[1] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	data[2] = (data[2] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;

	return err;
}
static int ECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}


/*----------------------------------------------------------------------------*/
static int mmc3524x_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= MMC3524X_BUFSIZE -1))
	{
		return -1;
	}
	if(!this_client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "mmc3524x Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3524X_BUFSIZE];
	mmc3524x_ReadChipInfo(strbuf, MMC3524X_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[MMC3524X_BUFSIZE];

	ECS_GetRawData(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[MMC3524X_BUFSIZE];
	tmp[0] = sensor_data[0] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[1] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[2] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if(data->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		printk(KERN_ERR "mmc3524x_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		printk(KERN_ERR "mmc3524x_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		printk(KERN_ERR "invalid content\n");
	}

	return count;
}
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3524X_BUFSIZE];
	sprintf(strbuf, "mmc3524xd");
	return sprintf(buf, "%s", strbuf);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *mmc3524x_attr_list[] = {
    	&driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
};

/*----------------------------------------------------------------------------*/
static int mmc3524x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, mmc3524x_attr_list[idx]);
		if(err)
		{
			printk(KERN_ERR "driver_create_file (%s) = %d\n", mmc3524x_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static int mmc3524x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mmc3524x_attr_list[idx]);
	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int mmc3524x_open(struct inode *inode, struct file *file)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if(atomic_read(&obj->trace) & MMC_CTR_DEBUG)
	{
		MMCDBG("Open device node:mmc3524x\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_release(struct inode *inode, struct file *file)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & MMC_CTR_DEBUG)
	{
		MMCDBG("Release device node:mmc3524x\n");
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int mmc3524x_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
static long mmc3524x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)

{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[MMC3524X_BUFSIZE];				/* for chip information */

	int value[CALIBRATION_DATA_SIZE];			/* for SET_YPR */
	int delay;				/* for GET_DELAY */
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);
	struct hwm_sensor_data* osensor_data;
	uint32_t enable;
	unsigned char reg_addr;
	unsigned char reg_value;
	//read_reg_str reg_str;
	//int data_reg[3] = {0};
	switch (cmd)
	{
		case MMC31XX_IOC_TM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_TM;
			if (I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_TM failed\n");
				return -EFAULT;
			}
			/* wait TM done for coming data read */
			//msleep(MMC3524X_DELAY_TM);
			break;

		case MMC31XX_IOC_SET:
		case MMC31XX_IOC_RM:
#if MEMSIC_SINGLE_POWER
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3524X_DELAY_SET);
#endif
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_SET;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = 0;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_RESET:
		case MMC31XX_IOC_RRM:
#if MEMSIC_SINGLE_POWER
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3524X_DELAY_RESET);
#endif
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_RESET;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = 0;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_READ:
			data[0] = MMC3524X_REG_DATA;
			if(I2C_RxData(data, 6) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_READ failed\n");
				return -EFAULT;
			}
		//	vec[0] = data[0] << 8 | data[1];
		//	vec[1] = data[2] << 8 | data[3];
		//	vec[2] = data[4] << 8 | data[5];
			vec[0] = data[1] << 8 | data[0];
			vec[1] = data[3] << 8 | data[2];
			vec[2] = data[5] << 8 | data[4];
#if DEBUG
			if(atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
			{
				printk("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
			}
#endif
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3524x_IOC_READ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MMC31XX_IOC_READXYZ:
			ECS_ReadXYZData(vec, 3);
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3524x_IOC_READXYZ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_DELAY:
			delay = mmcd_delay;
			if(copy_to_user(argp, &delay, sizeof(delay)))
			{
				printk(KERN_ERR "copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_SET_YPR:
			if(argp == NULL)
			{
				MMCDBG("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, argp, sizeof(value)))
			{
				MMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECOMPASS_IOC_GET_OPEN_STATUS:
			status = ECS_GetOpenStatus();
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = atomic_read(&o_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;


		case MSENSOR_IOCTL_READ_CHIPINFO:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}

			mmc3524x_ReadChipInfo(buff, MMC3524X_BUFSIZE);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			ECS_GetRawData(vec);
			sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_LAYOUT:
			status = atomic_read(&clientdata->layout);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:

			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			if(copy_from_user(&enable, argp, sizeof(enable)))
			{
				MMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			else
			{
			    printk( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",enable);
				if(1 == enable)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

			}

			break;

		case MMC3524X_IOC_READ_REG:
		     printk("MMC3524X_IOC_READ_REG \n");
			if (copy_from_user(&reg_addr, argp, sizeof(reg_addr)))
				return -EFAULT;
			data[0] = reg_addr;
			if (I2C_RxData(data, 1) < 0) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}
			printk("<7>planar Read register No. 0x%0x\n", data[0]);
			reg_value = data[0];
			if (copy_to_user(argp, &reg_value, sizeof(reg_value))) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}		
			break;

		case MMC3524X_IOC_WRITE_REG:
		     printk(" MMC3524X_IOC_WRITE_REG \n");
			if (copy_from_user(&data, argp, sizeof(data)))
			return -EFAULT;
			if (I2C_TxData(data, 2) < 0) {
			//mutex_unlock(&ecompass_lock);
			return -EFAULT;
			}
			printk("<7>planar Write '0x%0x' to  register No. 0x%0x\n", data[0], data[1]);
			
		    break; 
			
		case MMC3524X_IOC_READ_REGS:
			//printk(" MMC3524X_IOC_READ_REGS = %lx\n", MMC3524X_IOC_READ_REGS);
			if (copy_from_user(&data, argp, sizeof(data)))
				return -EFAULT;
			printk("<7> planar Read %d registers from 0x%0x\n", data[1], data[0]);	
			if (I2C_RxData(data, data[1]) < 0) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}
			printk("<7> data: %x %x %x \n%x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);	
			if (copy_to_user(argp, data, sizeof(data))) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}		
			break; 

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:

			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			osensor_data = (struct hwm_sensor_data *)buff;
		    	mutex_lock(&sensor_data_mutex);

			osensor_data->values[0] = sensor_data[8] * CONVERT_O;
			osensor_data->values[1] = sensor_data[9] * CONVERT_O;
			osensor_data->values[2] = sensor_data[10] * CONVERT_O;
			osensor_data->status = sensor_data[11];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);

			//sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
				//osensor_data->values[2],osensor_data->status,osensor_data->value_divide);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}

			//printk(KERN_ERR "Do nothing in case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA!!!!!!\n");
			break;

		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			return -ENOIOCTLCMD;
			break;
		}

	return 0;
}
#ifdef CONFIG_COMPAT
static long mmc3524x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	
		void __user *arg64 = compat_ptr(arg);
	
		if(!file->f_op || !file->f_op->unlocked_ioctl)
		{
			printk(KERN_ERR "file->f_op OR file->f_op->unlocked_ioctl is null!\n");
			return -ENOTTY;
		}
	
		switch(cmd)
		{
			case COMPAT_MMC31XX_IOC_TM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_TM, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_TM is failed!\n");
				}
				break;
	
			case COMPAT_MMC31XX_IOC_SET:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_SET, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_SET is failed!\n");
				}
				break;
				
			case COMPAT_MMC31XX_IOC_RM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RM, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_RM is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC31XX_IOC_RESET:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RESET, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_RESET is failed!\n");
				}
	
			case COMPAT_MMC31XX_IOC_RRM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RRM, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_RRM is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC31XX_IOC_READ:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READ, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_READ is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC31XX_IOC_READXYZ:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READXYZ, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_READXYZ is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC3524X_IOC_READ_REG:
				ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_READ_REG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC3524X_IOC_READ_REG is failed!\n");
				}		
				break;
	
			case COMPAT_MMC3524X_IOC_WRITE_REG:
				ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_WRITE_REG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC3524X_IOC_WRITE_REG is failed!\n");
				}
	
				break; 
			case COMPAT_MMC3524X_IOC_READ_REGS:
				ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_READ_REGS, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC3524X_IOC_READ_REGS is failed!\n");
				}	
				break; 
	
			case COMPAT_ECOMPASS_IOC_GET_DELAY:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_DELAY, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_DELAY is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_SET_YPR:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_SET_YPR, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_SET_YPR is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OPEN_STATUS, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_MFLAG:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_MFLAG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_MFLAG is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_OFLAG:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OFLAG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_OFLAG is failed!\n");
				}
	
				break;
	
	
			case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_READ_CHIPINFO is failed!\n");
				}
	
				break;
	
			case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_READ_SENSORDATA is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_LAYOUT:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_LAYOUT, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_LAYOUT is failed!\n");
				}
	
				break;
	
			case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE is failed!\n");
				}
	
				break;
	
			case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
/*
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA is failed!\n");
				}
*/	
				printk(KERN_ERR "Do nothing in case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA!!!!!!\n");
				break;
	
			default:
				printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
				return -ENOIOCTLCMD;
				break;		
		}
		return ret;
}

#endif
/*----------------------------------------------------------------------------*/
static struct file_operations mmc3524x_fops = {
	//.owner = THIS_MODULE,
	.open = mmc3524x_open,
	.release = mmc3524x_release,
	.unlocked_ioctl = mmc3524x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmc3524x_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mmc3524x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "msensor",
    .fops = &mmc3524x_fops,
};
/*----------------------------------------------------------------------------*/
int mmc3524x_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* msensor_data;

#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_operate");
	}
#endif
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&m_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&m_flag, 0);
					if(atomic_read(&o_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

				// TODO: turn device into standby or normal mode
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				msensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

				msensor_data->values[0] = sensor_data[4] * CONVERT_M;
				msensor_data->values[1] = sensor_data[5] * CONVERT_M;
				msensor_data->values[2] = sensor_data[6] * CONVERT_M;
				msensor_data->status = sensor_data[7];
				msensor_data->value_divide = CONVERT_M_DIV;

				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
				{
					MMCDBG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
						msensor_data->values[0],msensor_data->values[1],msensor_data->values[2],
						msensor_data->value_divide,msensor_data->status);
				}
#endif
			}
			break;
		default:
			printk(KERN_ERR "msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int mmc3524x_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* osensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_orientation_operate");
	}
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				osensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

				osensor_data->values[0] = sensor_data[8] * CONVERT_O;
				osensor_data->values[1] = sensor_data[9] * CONVERT_O;
				osensor_data->values[2] = sensor_data[10] * CONVERT_O;
				osensor_data->status = sensor_data[11];
				osensor_data->value_divide = CONVERT_O_DIV;

				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
			{
				MMCDBG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
					osensor_data->values[0],osensor_data->values[1],osensor_data->values[2],
					osensor_data->value_divide,osensor_data->status);
			}
#endif
			}
			break;
		default:
			printk(KERN_ERR "gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

//Add begin by MEMSIC for virtual gyro
int mmc3524x_gyro_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* gysensor_data;	
#if DEBUG	
	struct i2c_client *client = this_client;  
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_gyro_operate");
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			MMCFUNC("mmc3524x_gyro_delay");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				MMCDBG(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}	
			break;

		case SENSOR_ENABLE:
			MMCFUNC("mmc3524x_gyro_enable");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				MMCDBG(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}									
				}	
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			MMCFUNC("mmc3524x_gyro_data");
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				MMCDBG(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gysensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				gysensor_data->values[0] = sensor_data[12] * CONVERT_GY;
				gysensor_data->values[1] = sensor_data[13] * CONVERT_GY;
				gysensor_data->values[2] = sensor_data[14] * CONVERT_GY;
				gysensor_data->status = sensor_data[15];
				gysensor_data->value_divide = CONVERT_GY_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
			{
				MMCDBG("Hwm get gyr-sensor data: %d, %d, %d. divide %d, status %d!\n",
					gysensor_data->values[0],gysensor_data->values[1],gysensor_data->values[2],
					gysensor_data->value_divide,gysensor_data->status);
			}	
#endif
			}
			break;
		default:
			MMCDBG(KERN_ERR "gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}



int mmc3524x_roation_vector(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	static int rv_last_data[3] = {0};
	struct hwm_sensor_data* vector_data;	
	

#if DEBUG	
	struct i2c_client *client = this_client;  
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_roation_vector");
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			MMCFUNC("mmc3524x_roation_vector\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}	
			break;

		case SENSOR_ENABLE:
			 // printk("mmc3416x_roation_vector enable\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{   
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{  
					atomic_set(&o_flag, 0);

					if(atomic_read(&m_flag) == 0)
					{
					  atomic_set(&open_flag, 0);
					}									
				}	
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			//printk("mmc3524x_roation_vector_data\n");
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vector_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				if(rv_last_data[0] == sensor_data[16] && rv_last_data[1] == sensor_data[17] && rv_last_data[2] == sensor_data[18])
				{
					sensor_data[16] += 1;
				}
				vector_data->values[0] = sensor_data[16] * CONVERT_VEC;
				vector_data->values[1] = sensor_data[17] * CONVERT_VEC;
				vector_data->values[2] = sensor_data[18] * CONVERT_VEC;
				vector_data->status = sensor_data[19];
				vector_data->value_divide = CONVERT_VECTOR_DIV;
				
				rv_last_data[0] = sensor_data[16];
				rv_last_data[1] = sensor_data[17];
				rv_last_data[2] = sensor_data[18];
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			//if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
			{
				printk("Hwm get rat-sensor data: %d, %d, %d. divide %d, status %d!\n",
					vector_data->values[0],vector_data->values[1],vector_data->values[2],
					vector_data->value_divide,vector_data->status);
			}	
#endif
			}
			break;
		default:
			printk(KERN_ERR "rat operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
int mmc3524x_gravity(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vector_data;	
	

#if DEBUG	
	struct i2c_client *client = this_client;  
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_gravity");
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			MMCFUNC("mmc3524x_gravity\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}	
			break;

		case SENSOR_ENABLE:
			 // printk("mmc3524x_gravity enable\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{   
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{  
					atomic_set(&o_flag, 0);

					if(atomic_read(&m_flag) == 0)
					{
					  atomic_set(&open_flag, 0);
					}									
				}	
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			//printk("mmc3524x_gravity\n");
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vector_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vector_data->values[0] = sensor_data[20] * GRAVITY_FACTOR / 8192;
				vector_data->values[1] = sensor_data[21] * GRAVITY_FACTOR / 8192;
				vector_data->values[2] = sensor_data[22] * GRAVITY_FACTOR / 8192;
				vector_data->status = sensor_data[23];
				vector_data->value_divide = 1000;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			//if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
			{
				printk("Hwm get grv-sensor data: %d, %d, %d. divide %d, status %d!\n",
					vector_data->values[0],vector_data->values[1],vector_data->values[2],
					vector_data->value_divide,vector_data->status);
			}	
#endif
			}
			break;
		default:
			printk(KERN_ERR "grv operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
int mmc3524x_linear_acc(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vector_data;	
	

#if DEBUG	
	struct i2c_client *client = this_client;  
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_linear_acc");
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			MMCFUNC("mmc3524x_linear_acc\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}	
			break;

		case SENSOR_ENABLE:
			 // printk("mmc3524x_linear_acc enable\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{   
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{  
					atomic_set(&o_flag, 0);

					if(atomic_read(&m_flag) == 0)
					{
					  atomic_set(&open_flag, 0);
					}									
				}	
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			//printk("mmc3524x_linear_acc\n");
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vector_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vector_data->values[0] = sensor_data[24] * GRAVITY_FACTOR / 8192; 
				vector_data->values[1] = sensor_data[25] * GRAVITY_FACTOR / 8192;
				vector_data->values[2] = sensor_data[26] * GRAVITY_FACTOR / 8192;
				vector_data->status = sensor_data[27];
				vector_data->value_divide = 1000;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			//if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
			{
				printk("Hwm get lineacc-sensor data: %d, %d, %d. divide %d, status %d!\n",
					vector_data->values[0],vector_data->values[1],vector_data->values[2],
					vector_data->value_divide,vector_data->status);
			}	
#endif
			}
			break;
		default:
			printk(KERN_ERR "lineacc operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

//Add end by MEMSIC for virtual gyro
/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);


	if(msg.event == PM_EVENT_SUSPEND)
	{
		mmc3524x_power(obj->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_resume(struct i2c_client *client)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);


	mmc3524x_power(obj->hw, 1);


	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mmc3524x_early_suspend(struct early_suspend *h)
{
	struct mmc3524x_i2c_data *obj = container_of(h, struct mmc3524x_i2c_data, early_drv);

	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void mmc3524x_late_resume(struct early_suspend *h)
{
	struct mmc3524x_i2c_data *obj = container_of(h, struct mmc3524x_i2c_data, early_drv);


	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

	mmc3524x_power(obj->hw, 1);

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
//static int mmc3524x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
//{
//	strcpy(info->type, MMC3524X_DEV_NAME);
//	return 0;
//}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int mmc3524x_m_enable(int en)
{
    int value = en;
	printk("mmc3524x_m_enable++\n");
	
	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	wake_up(&open_wq);
	printk("mmc3524x_m_enable--\n");
	return 0;
}

static int mmc3524x_m_set_delay(u64 ns)
{
    int value=0;
	printk("mmc3524x_m_set_delay++\n");
	value = (int)ns/1000/1000;
	if(value <= 20)
	{
		mmcd_delay = 20;
	}
	mmcd_delay = value;
	printk("mmc3524x_m_set_delay--\n");
	return 0;
}
static int mmc3524x_m_open_report_data(int open)
{
	return 0;
}
 
static int mmc3524x_o_enable(int en)
{
  	int value =en;
	printk("mmc3524x_o_enable++\n");
	if(value == 1)
	{
		atomic_set(&o_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&o_flag, 0);
		if(atomic_read(&m_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}									
	}	
	wake_up(&open_wq);
	printk("mmc3524x_o_enable--\n");
	return 0;
}

static int mmc3524x_o_set_delay(u64 ns)
{
	int value=0;
	printk("mmc3524x_o_set_delay++\n");
	value = (int)ns/1000/1000;
	if(value <= 20)
	{
		mmcd_delay = 20;
	}
	mmcd_delay = value;
	printk("mmc3524x_o_set_delay--\n");
	return 0;
				
}
static int mmc3524x_o_open_report_data(int open)
{
	return 0;
}

static int mmc3524x_o_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);
	
	*x = sensor_data[8] * CONVERT_O;
	*y = sensor_data[9] * CONVERT_O;
	*z = sensor_data[10] * CONVERT_O;
	*status = sensor_data[11];
		
	mutex_unlock(&sensor_data_mutex);	
	
	return 0;
}

static int mmc3524x_m_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);
	
	*x = sensor_data[4] * CONVERT_M;
	*y = sensor_data[5] * CONVERT_M;
	*z = sensor_data[6] * CONVERT_M;
	*status = sensor_data[7];	
	mutex_unlock(&sensor_data_mutex);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int MMC_CheckDevice(void)
{
	char buffer[2];
	int ret;

	printk("[M_Sensor]MMC check device id");
	/* Set measure mode */
	#if 1
	buffer[0] = MMC3524X_REG_PRODUCTID_1;
	#endif

	/* Read data */
	ret = I2C_RxData(buffer, 1);
	printk("[M_Sensor]MMC check device id = %x\n", buffer[0]);
	printk("[M_Sensor]ret = %d\n", ret);
	if (ret < 0)
		return ret;

	/* Check read data */
	if (buffer[0] != 0x08)
		return -ENXIO;

	return 0;
}

static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	
	struct i2c_client *new_client;
	struct mmc3524x_i2c_data *data;
	//char tmp[2];
	int err = 0;
	struct mag_control_path ctl={0};
	struct mag_data_path mag_data={0};
	//char product_id[2] = {0,0};
	//Add begin by MEMSIC for gyro
	struct hwmsen_object sobj_gy, sobj_vec, sobj_grv, sobj_linacc;
	//Add end by MEMSIC for gyro

    printk("[M_Sensor]%s!\n", __func__);
/*********************************************************/   
//	if(!(data = kmalloc(sizeof(struct mmc3524x_i2c_data), GFP_KERNEL)))
//	{
//		err = -ENOMEM;
//		goto exit;
//	}
//	memset(data, 0, sizeof(struct mmc3524x_i2c_data));
//
//	data->hw = get_cust_mag_hw();
//
//	atomic_set(&data->layout, data->hw->direction);
//	atomic_set(&data->trace, 0);
//
//	mutex_init(&sensor_data_mutex);
//	mutex_init(&read_i2c_xyz);
//
//	init_waitqueue_head(&data_ready_wq);
//	init_waitqueue_head(&open_wq);
//
//	data->client = client;
//	new_client = data->client;
//	i2c_set_clientdata(new_client, data);
//
//	this_client = new_client;
//	//this_client->timing=400;
//        msleep(10);
//
//	/* send TM cmd to mag sensor first of all */
//	tmp[0] = MMC3524X_REG_CTRL;
//	tmp[1] = MMC3524X_CTRL_TM;
//	if(I2C_TxData(tmp, 2) < 0)
//	{
//		printk(KERN_ERR "mmc3524x_device set TM cmd failed\n");
//		goto exit_kfree;
//	}
//
//	printk("%s: ++++\n", __func__);
//	
//	//Memsic Bruce compat 2015.05.12
//	product_id[0] = MMC3524X_REG_PRODUCTID_1;
//	if(I2C_RxData(product_id, 1) < 0)
//	{
//		printk("[Memsic][Bruce][mmc3524x] read id fail\n");
//		//read again
//		I2C_RxData(product_id, 1);
//	}
//	
//	printk("[Memsic][Bruce][mmc3524x] product_id[0] = %d\n",product_id[0]);
//	if(product_id[0] != 0x8)
//	{
//		goto exit_kfree;
//	}
//	//Memsic end
//
//	tmp[0] = MMC3524X_REG_CTRL;
//	tmp[1] = MMC3524X_CTRL_SET;
//	if (I2C_TxData(tmp, 2) < 0) {
//	}
//	msleep(1);
//	tmp[0] = MMC3524X_REG_CTRL;
//	tmp[1] = 0;
//	if (I2C_TxData(tmp, 2) < 0) {
//	}
//	msleep(MMC3524X_DELAY_SET);
//
//	tmp[0] = MMC3524X_REG_BITS;
//	tmp[1] = MMC3524X_BITS_SLOW_16;
//	if (I2C_TxData(tmp, 2) < 0) {
//	}
//	msleep(MMC3524X_DELAY_TM);
//
//	tmp[0] = MMC3524X_REG_CTRL;
//	tmp[1] = MMC3524X_CTRL_TM;
//	if (I2C_TxData(tmp, 2) < 0) {
//	}
//	msleep(MMC3524X_DELAY_TM);
/******************************************************/
	data = kzalloc(sizeof(struct mmc3524x_i2c_data), GFP_KERNEL);
	printk("[Msensor] %s: after kzalloc\n", __func__);
	if (!data) {
		printk("%s: data is null!!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct mmc3524x_i2c_data));
	
	data->hw = hw;
	if (hwmsen_get_convert(data->hw->direction, &data->cvt)) {
        	printk("[Msensor] MMC3524x invalid direction: %d\n", data->hw->direction);
        	goto exit_kfree;
	}
	
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);
	
	mutex_init(&sense_data_mutex);
	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);
	printk("[Msensor] %s: after mutex_init\n", __func__);
	
	/* init_waitqueue_head(&data_ready_wq); */
	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);
	
	printk("[Msensor] %s: after init_waitqueue_head\n", __func__);
	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);
	printk("[Msensor] %s: after i2c_set_clientdata\n", __func__);
	this_client = new_client;
	
	/* Check connection */
	err = MMC_CheckDevice();
	printk("[Msensor] %s: after MMC_CheckDevice err=%d\n", __func__, err);
	if (err < 0) {
		printk("[Msensor] MMC3524X mmc3524x_probe: check device connect error\n");
		goto exit_init_failed;
	}

	err = misc_register(&mmc3524x_device);
	printk("[Msensor] %s: after misc_register err=%d\n", __func__, err);
	if (err) {
		printk("[Msensor] MMC3524X_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.m_enable = mmc3524x_m_enable;
	ctl.m_set_delay = mmc3524x_m_set_delay;
	ctl.m_open_report_data = mmc3524x_m_open_report_data;
	ctl.o_enable = mmc3524x_o_enable;
	ctl.o_set_delay = mmc3524x_o_set_delay;
	ctl.o_open_report_data = mmc3524x_o_open_report_data;
	ctl.is_report_input_direct =false;
	ctl.is_support_batch = data->hw->is_batch_supported;

	err = mag_register_control_path(&ctl);
	printk("[Msensor] %s: after mag_register_control_path err=%d\n", __func__, err);
	if (err) {
		printk("[Msensor] register mag control path err\n");
		goto exit_kfree;
	}

	mag_data.div_m = CONVERT_M_DIV;
	mag_data.div_o = CONVERT_O_DIV;
	mag_data.get_data_o = mmc3524x_o_get_data;
	mag_data.get_data_m = mmc3524x_m_get_data;

	err = mag_register_data_path(&mag_data);
	printk("[Msensor] %s: after mag_register_data_path err=%d\n", __func__, err);
	if (err) {
		MAG_ERR("register data control path err\n");
		goto exit_kfree;
	}
	err = mmc3524x_create_attr(&(mmc3524x_init_info.platform_diver_addr->driver));
	printk("[Msensor] %s: after mmc3524x_create_attr err=%d\n", __func__, err);
	if(err)
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}
/*
	ctl.is_use_common_factory = false;
	ctl.m_enable = mmc3524x_m_enable;
	ctl.m_set_delay  = mmc3524x_m_set_delay;
	ctl.m_open_report_data = mmc3524x_m_open_report_data;
	ctl.o_enable = mmc3524x_o_enable;
	ctl.o_set_delay  = mmc3524x_o_set_delay;
	ctl.o_open_report_data = mmc3524x_o_open_report_data;
	ctl.is_report_input_direct = false;
	
	err = mag_register_control_path(&ctl);
	printk("[Msensor] %s: after mag_register_control_path err=%d\n", __func__, err);
	if(err)
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}

	mag_data.div_m = CONVERT_M_DIV;
	mag_data.div_o = CONVERT_O_DIV;
	mag_data.get_data_o = mmc3524x_o_get_data;
	mag_data.get_data_m = mmc3524x_m_get_data;
	
	err = mag_register_data_path(&mag_data);
	printk("[Msensor] %s: after mag_register_data_path err=%d\n", __func__, err);
	if(err)
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}
*/
//Add begin by MEMSIC for gyro

	sobj_gy.self = data;
	sobj_gy.polling = 1;
	sobj_gy.sensor_operate = mmc3524x_gyro_operate;
	if((err = hwmsen_attach(ID_GYROSCOPE, &sobj_gy)))
	{
		MMCDBG(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}
	
	sobj_vec.self = data;
	sobj_vec.polling = 1;
	sobj_vec.sensor_operate = mmc3524x_roation_vector;
	if((err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_vec)))
	{
	 printk(KERN_ERR "	vector attach fail = %d\n", err);
	 goto exit_kfree;
	}
	sobj_grv.self = data;
	sobj_grv.polling = 1;
	sobj_grv.sensor_operate = mmc3524x_gravity;
	if((err = hwmsen_attach(ID_GRAVITY, &sobj_grv)))
	{
	 printk(KERN_ERR "	gravity attach fail = %d\n", err);
	 goto exit_kfree;
	}
	sobj_linacc.self = data;
	sobj_linacc.polling = 1;
	sobj_linacc.sensor_operate = mmc3524x_linear_acc;
	if((err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_linacc)))
	{
	 printk(KERN_ERR "	linear acc attach fail = %d\n", err);
	 goto exit_kfree;
	}

//Add end by MEMSIC for gyro
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = mmc3524x_early_suspend,
	data->early_drv.resume   = mmc3524x_late_resume,
	register_early_suspend(&data->early_drv);
#endif

	MMCDBG("%s: OK\n", __func__);
//	m_sensor_deteced = true;
	printk("mmc3524X IIC probe successful !");
	mmc3524x_init_flag = 1;
	return 0;

	exit_sysfs_create_group_failed:
	exit_misc_device_register_failed:
	exit_init_failed:
	exit_kfree:
	kfree(data);
	exit:
	printk(KERN_ERR "%s: err = %d\n", __func__, err);
//	m_sensor_deteced = false;
	mmc3524x_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_remove(struct i2c_client *client)
{
	int err;	
	
    if((err = mmc3524x_delete_attr(&(mmc3524x_init_info.platform_diver_addr->driver))))
	{
		printk(KERN_ERR "mmc3524x_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&mmc3524x_device);
	return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int platform_mmc3524x_probe(struct platform_device *pdev)
{
	printk("[M_Sensor]%s!\n", __func__);
	struct mag_hw *hw = get_cust_mag_hw();

	mmc3524x_power(hw, 1);
	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&mmc3524x_i2c_driver))
	{
		printk(KERN_ERR "add driver error\n");
		return -1;
	}
	return 0;
}
static int platform_mmc3524x_remove(struct platform_device *pdev)
{
	struct mag_hw *hw = get_cust_mag_hw();

	mmc3524x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc3524x_i2c_driver);
	return 0;
}

*/
static int	mmc3524x_local_init(void)
{
	printk("[M_Sensor] %s\n", __func__);
	
	//struct mag_hw *hw = get_cust_mag_hw();
	//printk("fwq loccal init+++\n");

	mmc3524x_power(hw, 1);
	if(i2c_add_driver(&mmc3524x_i2c_driver))
	{
		MMCDBG("add driver error\n");
		return -1;
	}
	if(-1 == mmc3524x_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}

static int mmc3524x_remove(void)
{
//    struct mag_hw *hw = get_cust_mag_hw();
	mmc3524x_power(hw, 0);    
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&mmc3524x_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init mmc3524x_init(void)
{
	
//	struct mag_hw *hw = get_cust_mag_hw();
//
//	i2c_register_board_info(hw->i2c_num, &i2c_mmc3524x, 1);
	
	/*if(platform_driver_register(&mmc3524x_platform_driver))
	{
		printk("failed to register driver!\n");
		return -ENODEV;
	}*/
	const char *name = "mediatek,mmc3524x";
	printk("[M_Sensor]%s!\n", __func__);
	hw = get_mag_dts_func(name, hw);
	if(!hw)
		printk("[M_Sensor]get dts info fail\n");
	
	mag_driver_add(&mmc3524x_init_info);
	
	return 0;
}



/*----------------------------------------------------------------------------*/
static void __exit mmc3524x_exit(void)
{
	//platform_driver_unregister(&mmc3524x_platform_driver);
	printk("[M_Sensor]mmc3524x_exit!!!\n");
}
/*----------------------------------------------------------------------------*/
module_init(mmc3524x_init);
module_exit(mmc3524x_exit);

MODULE_AUTHOR("WD");
MODULE_DESCRIPTION("mmc3524x compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

