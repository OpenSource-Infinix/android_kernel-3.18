#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mach/mt_charging.h>
//fmy add start
#include <mach/gpio_const.h>
#include <linux/kthread.h>
//fmy add end
#include <mt-plat/charging.h>
#include <mt-plat/mt_gpio.h>
#include "sm5414.h"

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define GPIO_SM5414_CHGEN_PIN	(62 | 0x80000000)
#define GPIO_SM5414_SHDN_PIN	(80 | 0x80000000)

#ifdef CONFIG_OF
#else

#define sm5414_SLAVE_ADDR_WRITE   0x92
#define sm5414_SLAVE_ADDR_READ    0x93

#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define sm5414_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define sm5414_BUSNUM 1
#endif

#endif
static struct i2c_client *new_client;
static const struct i2c_device_id sm5414_i2c_id[] = { {"sm5414", 0}, {} };

//fmy add start
extern unsigned int mt_gpio_to_irq(unsigned int gpio);
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int sm5414_flag;

 int sm5414_irq = 0;
 int sm5414_int_number;
#define GPIO_SM5414_EINT_PIN GPIO4
#define SM5414_EINT_PIN (GPIO_SM5414_EINT_PIN & ~(0x80000000))

#define SM5414_REGISTER_NUM 0x0E 
unsigned char sm5414_register[SM5414_REGISTER_NUM] = { 0 };

static const struct of_device_id sm5414_of_match[] = {
	{ .compatible = "mediatek,sm5414", },
	{},
};
//fmy add end

kal_bool chargin_hw_init_done = false;
static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);


/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char sm5414_reg[SM5414_REG_NUM] = { 0 };

static DEFINE_MUTEX(sm5414_i2c_access);

int g_sm5414_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write sm5414]
  *
  *********************************************************/
#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int sm5414_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&sm5414_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;
		mutex_unlock(&sm5414_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;
	mutex_unlock(&sm5414_i2c_access);

	return 1;
}

unsigned int sm5414_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&sm5414_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&sm5414_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&sm5414_i2c_access);
	return 1;
}
#else
unsigned int sm5414_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&sm5414_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sm5414_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int sm5414_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&sm5414_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sm5414_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif
/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int sm5414_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char sm5414_reg = 0;
	unsigned int ret = 0;

	ret = sm5414_read_byte(RegNum, &sm5414_reg);

	battery_log(BAT_LOG_FULL, "[sm5414_read_interface] Reg[%x]=0x%x\n", RegNum, sm5414_reg);

	sm5414_reg &= (MASK << SHIFT);
	*val = (sm5414_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[sm5414_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int sm5414_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char sm5414_reg = 0;
	unsigned int ret = 0;

	ret = sm5414_read_byte(RegNum, &sm5414_reg);
	battery_log(BAT_LOG_FULL, "[sm5414_config_interface] Reg[%x]=0x%x\n", RegNum, sm5414_reg);

	sm5414_reg &= ~(MASK << SHIFT);
	sm5414_reg |= (val << SHIFT);

	ret = sm5414_write_byte(RegNum, sm5414_reg);
	battery_log(BAT_LOG_FULL, "[sm5414_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    sm5414_reg);

	/* Check */
	/* sm5414_read_byte(RegNum, &sm5414_reg); */
	/* printk("[sm5414_config_interface] Check Reg[%x]=0x%x\n", RegNum, sm5414_reg); */

	return ret;
}

/* write one register directly */
unsigned int sm5414_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = sm5414_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
//STATUS2----------------------------------------------------
unsigned int sm5414_get_topoff_status(void)
{
    unsigned char val=0;
    unsigned int ret=0; 
 
    ret=sm5414_read_interface(     (unsigned char)(SM5414_STATUS), 
                                    (&val),
                                    (unsigned char)(SM5414_STATUS_TOPOFF_MASK),
                                    (unsigned char)(SM5414_STATUS_TOPOFF_SHIFT)
                                    );
    return val;
   // return 0;
}

//CTRL----------------------------------------------------
void sm5414_set_enboost(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_ENBOOST_MASK),
                                    (unsigned char)(SM5414_CTRL_ENBOOST_SHIFT)
                                    );
}

void sm5414_set_chgen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_CHGEN_MASK),
                                    (unsigned char)(SM5414_CTRL_CHGEN_SHIFT)
                                    );
}

void sm5414_set_suspen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_SUSPEN_MASK),
                                    (unsigned char)(SM5414_CTRL_SUSPEN_SHIFT)
                                    );
}

void sm5414_set_reset(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_RESET_MASK),
                                    (unsigned char)(SM5414_CTRL_RESET_SHIFT)
                                    );
}

void sm5414_set_encomparator(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_ENCOMPARATOR_MASK),
                                    (unsigned char)(SM5414_CTRL_ENCOMPARATOR_SHIFT)
                                    );
}

//vbusctrl----------------------------------------------------
void sm5414_set_vbuslimit(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_VBUSCTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_MASK),
                                    (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_SHIFT)
                                    );
}

//chgctrl1----------------------------------------------------
void sm5414_set_prechg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_PRECHG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_PRECHG_SHIFT)
                                    );
}
void sm5414_set_aiclen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLEN_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLEN_SHIFT)
                                    );
}
void sm5414_set_autostop(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_SHIFT)
                                    );
}
void sm5414_set_aiclth(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLTH_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLTH_SHIFT)
                                    );
}

//chgctrl2----------------------------------------------------
void sm5414_set_fastchg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL2), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL2_FASTCHG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL2_FASTCHG_SHIFT)
                                    );
}

//chgctrl3----------------------------------------------------
void sm5414_set_weakbat(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL3), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_MASK),
                                    (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_SHIFT)
                                    );
}
void sm5414_set_batreg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL3), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL3_BATREG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL3_BATREG_SHIFT)
                                    );
}

//chgctrl4----------------------------------------------------
void sm5414_set_dislimit(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL4), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_MASK),
                                    (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_SHIFT)
                                    );
}
void sm5414_set_topoff(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL4), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL4_TOPOFF_MASK),
                                    (unsigned char)(SM5414_CHGCTRL4_TOPOFF_SHIFT)
                                    );
}

//chgctrl5----------------------------------------------------
void sm5414_set_topofftimer(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL5), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_MASK),
                                    (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_SHIFT)
                                    );
}
void sm5414_set_fasttimer(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL5), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_MASK),
                                    (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_SHIFT)
                                    );
}
void sm5414_set_votg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL5), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL5_VOTG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL5_VOTG_SHIFT)
                                    );
}  

/**********************************************************
  *
  *   [External Function] 
  *
  *********************************************************/
void sm5414_chg_en(unsigned int enable)
{
    if(KAL_TRUE == enable)
    {    
        //SM : Please set nCHGEN pin to low(Charging ON)
        mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ZERO);    
    }
    else
    {    
        //SM : Please set nCHGEN pin to high(Charging OFF)
        mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ONE);    
    }
}

void sm5414_cnt_nshdn_pin(unsigned int enable)
{
    //SM : Below is nSHDN pin
    if(KAL_TRUE == enable)
    {    
    //SM : Below is nSHDN pin
        mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_SHDN_PIN,GPIO_OUT_ONE);
    }
    else
    {    
    //SM : Below is nSHDN pin
        mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN,GPIO_DIR_OUT);        
        mt_set_gpio_out(GPIO_SM5414_SHDN_PIN,GPIO_OUT_ZERO);
    }
}
void sm5414_otg_enable(unsigned int enable)
{
    if(KAL_TRUE == enable)
    {
        //SM : Before turning on OTG, system must turn off charing function.        
        //SM : Below is nCHGEN pin(Charging OFF)
        mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ONE);
        sm5414_set_enboost(ENBOOST_EN);
    }
    else
    {   
        sm5414_set_enboost(ENBOOST_DIS);
    }
}


/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void sm5414_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = sm5414_read_interface(0x0E, &val, 0xFF, 0x0);

	if (val == 0)
		g_sm5414_hw_exist = 0;
	else
		g_sm5414_hw_exist = 1;

	pr_debug("[sm5414_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n",
		 g_sm5414_hw_exist, val);
}

int is_sm5414_exist(void)
{
	pr_debug("[is_sm5414_exist] g_sm5414_hw_exist=%d\n", g_sm5414_hw_exist);

	return g_sm5414_hw_exist;
}

void sm5414_dump_register(void)
{
	int i=0;
	    
    printk("[sm5414]\n ");
    for (i=SM5414_INTMASK1;i<SM5414_REG_NUM;i++)
    {
        sm5414_read_byte(i, &sm5414_reg[i]);
        printk("[0x%x]=0x%x ", i, sm5414_reg[i]);        
    }
    printk("\n");

}

void sm5414_hw_init(void)
{
	battery_log(BAT_LOG_CRTI, "[sm5414_hw_init] After HW init\n");
	sm5414_dump_register();
}

void sm5414_reg_init(void)
{
   //INT MASK 1/2/3
    sm5414_write_byte(SM5414_INTMASK1, 0xFF);
    sm5414_write_byte(SM5414_INTMASK2, 0xEF); //Only OTG Fail
    sm5414_write_byte(SM5414_INTMASK3, 0xFF);

	sm5414_set_encomparator(ENCOMPARATOR_EN);
    sm5414_set_topoff(TOPOFF_150mA);
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	sm5414_set_batreg(BATREG_4_4_0_0_V); //VREG 4.352V
#else
	sm5414_set_batreg(BATREG_4_2_0_0_V); //VREG 4.208V
#endif 
    //sm5414_set_autostop(AUTOSTOP_DIS);

#if defined(SM5414_TOPOFF_TIMER_SUPPORT)
    sm5414_set_autostop(AUTOSTOP_EN);
    sm5414_set_topofftimer(TOPOFFTIMER_10MIN);
#else
    sm5414_set_autostop(AUTOSTOP_DIS);
#endif
}

static int sm5414_event_handler(void *unused)
{
	//sm5414 interrupt bottom function
	int sm5414_int_gpio_status = 0;
	printk("fmy [%s] line:%d \n",__FUNCTION__,__LINE__);

	do{
		//wait work queue
		wait_event_interruptible(waiter, sm5414_flag != 0);
		sm5414_flag = 0;

		//read all SM5414_INT register for reset electric signal level
		sm5414_read_byte(0, &sm5414_register[0]);		//SM5414_INT1
		sm5414_read_byte(1, &sm5414_register[1]);		//SM5414_INT2
		sm5414_read_byte(2, &sm5414_register[2]);		//SM5414_INT3

		printk("fmy [%s] INT1 : 0x%x, INT2 : 0x%x, INT3 : 0x%x\n",__FUNCTION__,sm5414_register[0],sm5414_register[1],sm5414_register[2]);

		//if it is real OTG boost fail
		if(sm5414_register[1] & SM5414_INT2_OTGFAIL){
		
			printk("fmy sm5414_reg[1] = 0x10, OTG Boost Fail\n");				

			sm5414_otg_enable(KAL_FALSE);	
			//NOW, it drpoed out OTG mode , enable CHRGEN , and reset OTG enable
			mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);
			mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ZERO);
			sm5414_set_votg(0x0);
			sm5414_otg_enable(KAL_TRUE);		
		}
		printk("fmy sm5414_irq_handler  sm5414_register[1] = 0x%x\n",sm5414_register[1] );

		msleep(80);		// sleep for detect if real short circuit

		sm5414_int_gpio_status = mt_get_gpio_in(GPIO_SM5414_EINT_PIN);		//read INT Pin status , if real short circuit disable interrupt
		printk("fmy [%s]Line:%d sm5414_int_gpio_status = %d\n",__FUNCTION__,__LINE__,sm5414_int_gpio_status);
		if(sm5414_int_gpio_status){
			irq_set_irq_type(sm5414_int_number, IRQF_TRIGGER_LOW);
			enable_irq(sm5414_int_number);
		}
	
	}while(!kthread_should_stop());
	return 0;
}

 irqreturn_t sm5414_irq_handler(void)
{
	//sm5414 interrupt top function
	printk("fmy [%s] line:%d \n",__FUNCTION__,__LINE__);
	
	disable_irq_nosync(sm5414_int_number);

	sm5414_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

 void sm5414_irq_registration(void)
{	
	int ret = 0;
	printk("fmy [%s] line:%d \n",__FUNCTION__,__LINE__);	
	printk("fmy Device Tree sm5414_irq_registration!\n");
	
	gpio_set_debounce(SM5414_EINT_PIN, 64000);
	sm5414_int_number = mt_gpio_to_irq(SM5414_EINT_PIN);
	printk("fmy SM5414 INT IRQ LINE %d, %d!!\n", SM5414_EINT_PIN, mt_gpio_to_irq(SM5414_EINT_PIN));

	ret = request_irq(sm5414_int_number,(irq_handler_t)sm5414_irq_handler, IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT, "sm5414-eint", NULL);

	if (ret > 0)
		printk("fmy SM5414  IRQ LINE NOT available!!\n");
	else
		printk("fmy SM5414  IRQ LINE available!!\n");

}

static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct task_struct *thread = NULL;

	printk("******fmy sm5414_driver_probe  ********\n");
	battery_log(BAT_LOG_CRTI, "[sm5414_driver_probe]\n");

	new_client = client;
	/* --------------------- */
	sm5414_hw_component_detect();
    
    	sm5414_reg_init();

	thread = kthread_run(sm5414_event_handler, 0, "sm5414");
  
	sm5414_dump_register();
	printk("fmy [sm5414_driver_probe] line:%d\n",__LINE__);
	/* sm5414_hw_init(); //move to charging_hw_xxx.c */
	chargin_hw_init_done = true;

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_sm5414 = 0;
static ssize_t show_sm5414_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_sm5414_access] 0x%x\n", g_reg_value_sm5414);
	return sprintf(buf, "%u\n", g_reg_value_sm5414);
}

static ssize_t store_sm5414_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	/*char *pvalue = NULL;*/
	unsigned int reg_value = 0;
	unsigned long int reg_address = 0;
	int rv;

	battery_log(BAT_LOG_CRTI, "[store_sm5414_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_sm5414_access] buf is %s and size is %zu\n", buf,
			    size);
		/*reg_address = simple_strtoul(buf, &pvalue, 16);*/
		rv = kstrtoul(buf, 0, &reg_address);
			if (rv != 0)
				return -EINVAL;
		/*ret = kstrtoul(buf, 16, reg_address); *//* This must be a null terminated string */
		if (size > 3) {
			/*NEED to check kstr*/
			/*reg_value = simple_strtoul((pvalue + 1), NULL, 16);*/
			/*ret = kstrtoul(buf + 3, 16, reg_value); */
			battery_log(BAT_LOG_CRTI,
				    "[store_sm5414_access] write sm5414 reg 0x%x with value 0x%x !\n",
				    (unsigned int) reg_address, reg_value);
			ret = sm5414_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = sm5414_read_interface(reg_address, &g_reg_value_sm5414, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
				    "[store_sm5414_access] read sm5414 reg 0x%x with value 0x%x !\n",
				    (unsigned int) reg_address, g_reg_value_sm5414);
			battery_log(BAT_LOG_CRTI,
				    "[store_sm5414_access] Please use \"cat sm5414_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(sm5414_access, 0664, show_sm5414_access, store_sm5414_access);	/* 664 */

static int sm5414_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** sm5414_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_sm5414_access);

	return 0;
}

struct platform_device sm5414_user_space_device = {
	.name = "sm5414-user",
	.id = -1,
};

static struct platform_driver sm5414_user_space_driver = {
	.probe = sm5414_user_space_probe,
	.driver = {
		   .name = "sm5414-user",
		   },
};
/*
#ifdef CONFIG_OF
static const struct of_device_id sm5414_of_match[] = {
	{.compatible = "mediatek,sm5414"},
	{},
};
#else
static struct i2c_board_info i2c_sm5414 __initdata = {
	I2C_BOARD_INFO("sm5414", (sm5414_SLAVE_ADDR_WRITE >> 1))
};
#endif*/

static struct i2c_driver sm5414_driver = {
	.driver = {
		   .name = "sm5414",
#ifdef CONFIG_OF
		   .of_match_table = sm5414_of_match,
#endif
		   },
	.probe = sm5414_driver_probe,
	.id_table = sm5414_i2c_id,
};

static int __init sm5414_init(void)
{
	int ret = 0;

	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "[sm5414_init] init start with i2c DTS");
#else
	battery_log(BAT_LOG_CRTI, "[sm5414_init] init start. ch=%d\n", sm5414_BUSNUM);
	i2c_register_board_info(sm5414_BUSNUM, &i2c_sm5414, 1);
#endif
	if (i2c_add_driver(&sm5414_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[sm5414_init] failed to register sm5414 i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[sm5414_init] Success to register sm5414 i2c driver.\n");
	}

	/* sm5414 user space access interface */
	ret = platform_device_register(&sm5414_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[sm5414_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&sm5414_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[sm5414_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit sm5414_exit(void)
{
	i2c_del_driver(&sm5414_driver);
}
module_init(sm5414_init);
module_exit(sm5414_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sm5414 Driver");
