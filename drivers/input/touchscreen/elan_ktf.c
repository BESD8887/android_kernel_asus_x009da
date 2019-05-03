/* drivers/input/touchscreen/ektf.c - ELAN EKTF verions of driver
*
* Copyright (C) 2011 Elan Microelectronics Corporation.
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
* 2014/0/28: The first release, version 0x0006
*             Integrated 2 ,5 ,and 10 fingers driver code together and
*             auto-mapping resolution.
*             Please change following parameters
*                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
*                    The packet size is 18 or 24 bytes.
*                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
*                    The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
*                 3. Please enable the ELAN_BUTTON configuraton to support button.
*		          4. For ektf3k serial, Add Re-Calibration Machanism 
*                    So, please enable the define of RE_CALIBRATION.
*                   
*								 
*/

/* The ELAN_PROTOCOL support normanl packet format */	
#define FINGER_NUM 10
#define ELAN_PROTOCOL	
//#define ELAN_BUFFER_MODE
/*[Arima_5816][bozhi_lin] enable elan touch virtual key 20150925 begin*/
#define ELAN_BUTTON
/*[Arima_5816][bozhi_lin] 20150925 end*/
//#define RE_CALIBRATION   /* Re-Calibration after system resume. */
//#define ELAN_2WIREICE
#define ELAN_POWER_SOURCE
//#define ELAN_RESUME_RST
#define DEVICE_NAME "elan_ktf" 
#define EKTF3K_FLASH
#define PROTOCOL_A    /* multi-touch protocol  */
//#define PROTOCOL_B    /* Default: PROTOCOL B */
//#define ELAN_HID_I2C	/* for hid over i2c protocol */

#include <linux/module.h>
#include <linux/input.h>
#ifdef PROTOCOL_B
#include <linux/input/mt.h>
#endif
#ifdef PROTOCOL_A
#include <linux/input.h>
#endif
#include <linux/interrupt.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>

#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/input/elan_ktf.h> 
#include <linux/kthread.h>
//#include "elan_ktf.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/krait-regulator.h>
#endif

#define PACKET_SIZE		67		/* support 10 fingers packet for nexus7 55 */
#define MAX_FINGER_SIZE		255
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT		  0x52
#define CMD_R_PKT		  0x53
#define CMD_W_PKT		  0x54
#define RESET_PKT		  0x77
#define CALIB_PKT		  0x66
#define IamLive_PKT		0x78
#define PEN_PKT			  0x71
#define SmartWake_PKT 0x88

#define HELLO_PKT		0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define ELAN_HID_PKT			0x3F
#define BUFFER_PKT		0x63
#define BUFFER55_PKT		0x66

/*[Arima_5816][bozhi_lin] upgrade elan truly touch firmware to 0x5501 and enable ESD & checksum 20160307 begin*/
#define ESD_CHECK
/*[Arima_5816][bozhi_lin] 20160307 end*/
#ifdef ESD_CHECK
int live_state=1;
#endif
int fwupd_flag  = 0; 
/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
#define GLOVEMODE //1117
/*[Arima_5816][bozhi_lin] 20151119 end*/

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#define GESTUREMODE
#if defined(GESTUREMODE)
#define GESTURE_ENABLE		(1<<6)
#define GESTURE_W_ENABLE	(1<<5)
#define GESTURE_S_ENABLE	(1<<4)
#define GESTURE_E_ENABLE	(1<<3)
#define GESTURE_C_ENABLE	(1<<2)
#define GESTURE_Z_ENABLE	(1<<1)
#define GESTURE_V_ENABLE	(1<<0)
#define GESTURE_MASK		0x3F
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#define ARIMA_KEY
/*[Arima_5816][bozhi_lin] 20160112 end*/

/*[Arima_5830][bozhi_lin] add touch gesture key code follow arima design 20160413 begin*/
#define ARIMA_DOUBLE_CLICK
/*[Arima_5830][bozhi_lin] 20160413 end*/

// Reset pin need to be modified by customer
#define SYSTEM_RESET_PIN_SR 12	// nexus7_grouper TEGRA_GPIO_PH6: 62, nexus7_flo 31

//Add these Define
#define IAP_PORTION            	
#define PAGERETRY  30
#define IAPRESTART 5

#define ELAN_VTG_MIN_UV		2700000
#define ELAN_VTG_MAX_UV		3300000
#define ELAN_ACTIVE_LOAD_UA	15000
#define ELAN_LPM_LOAD_UA	10

#define ELAN_I2C_VTG_MIN_UV	1800000
#define ELAN_I2C_VTG_MAX_UV	1800000
#define ELAN_I2C_LOAD_UA	10000
#define ELAN_I2C_LPM_LOAD_UA	10

// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)
#define IOCTL_PTEST  _IOR(ELAN_IOCTLID, 20, int)

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)
/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
#define POWER_BATTERY 0
#define POWER_USB     1
#define POWER_CHARGER 2
/*[Arima_5816][bozhi_lin] 20151231 end*/
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10


#define SYSFS_MAX_LEN 100
static unsigned int gPrint_point = 0;
static int debug = DEBUG_INFO;
#define touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
		printk("[ektf]:" __VA_ARGS__); \
	} while (0)


uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5500 20160328 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
/*[Arima_5830][bozhi_lin] fix touch upgrade firmware from recovery mode, x y resolution is wrong 20160322 begin*/
int X_RESOLUTION=1024;	// 768
int Y_RESOLUTION=1728;	// 1344
/*[Arima_5830][bozhi_lin] 20160322 end*/
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
int X_RESOLUTION=832;	// 768
int Y_RESOLUTION=1408;	// 1344
/*[Arima_5833][bozhi_lin] 20160328 end*/
#else
int X_RESOLUTION=1344;	// nexus7 1280 1344
int Y_RESOLUTION=2240;	// nexus7 2112 2240
#endif
int FW_ID=0x00;
int work_lock=0x00;
int power_lock=0x00;
int circuit_ver=0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr=0x10;
/*++++i2c transfer end+++++++*/
struct mutex ktf_mutex;
//int button_state = 0; //1117
bool KEY_BACK_STATE = false;
bool KEY_HOME_STATE = false;
bool KEY_MENU_STATE = false;

#ifdef IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x10, 0x20, 0x21};/*I2C devices address*/  
int is_OldBootCode = 0; // 0:new 1:old
/*[Arima_5816][bozhi_lin] fix converity bug 20151123 begin*/
static unsigned char firmware[500*132];//firmware[52800];
/*[Arima_5816][bozhi_lin] 20151123 end*/

/*[Arima_5816][bozhi_lin] upgrade elan truly touch firmware to 0x5501 and enable ESD & checksum 20160307 begin*/
#define CHECKSUM_ENABLE
/*[Arima_5816][bozhi_lin] 20160307 end*/

/*[Arima_5816][bozhi_lin] re-write touch upgrade firmware mechanism 20151124 begin*/
#define FILE_UPGRADE
/*[Arima_5816][bozhi_lin] 20151124 end*/

#if defined(FILE_UPGRADE)
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5500 20160328 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5510 20160808 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5509 20160615 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5508 20160608 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5507 20160520 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5506 20160513 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5505 20160510 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5504 20160401 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5503 20160321 begin*/
/*[Arima_5830][bozhi_lin] upgrade elan truly touch firmware to 0x5502 20160311 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan truly touch firmware to 0x5501 and enable ESD & checksum 20160307 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan truly touch firmware to 0x5500 20160218 begin*/
#define TRULY_FW_FILENAME  "CTP_2150_5830_0x5510.ekt"
/*[Arima_5816][bozhi_lin] 20160218 end*/
/*[Arima_5816][bozhi_lin] 20160307 end*/
/*[Arima_5830][bozhi_lin] 20160311 end*/
/*[Arima_5830][bozhi_lin] 20160321 end*/
/*[Arima_5830][bozhi_lin] 20160401 end*/
/*[Arima_5830][bozhi_lin] 20160510 end*/
/*[Arima_5830][bozhi_lin] 20160513 end*/
/*[Arima_5830][bozhi_lin] 20160520 end*/
/*[Arima_5830][bozhi_lin] 20160608 end*/
/*[Arima_5830][bozhi_lin] 20160615 end*/
/*[Arima_5830][bozhi_lin] 20160808 end*/
#define HOLITECH_FW_FILENAME  ""
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
#define TRULY_FW_FILENAME  ""
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5505 20160826 begin*/
/*[Arima_5833][bozhi_lin] add touch 2nd source detect 20160818 begin*/
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5504 20160720 begin*/
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5503 20160617 begin*/
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5502 20160525 begin*/
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5501 20160516 begin*/
#define HOLITECH_FW_FILENAME    "CTP_2150_5833_3050_5505.ekt"
#define HOLITECH_2_FW_FILENAME  "CTP_2150_5833_3050_5505.ekt"
/*[Arima_5833][bozhi_lin] 20160516 end*/
/*[Arima_5833][bozhi_lin] 20160525 end*/
/*[Arima_5833][bozhi_lin] 20160617 end*/
/*[Arima_5833][bozhi_lin] 20160720 end*/
/*[Arima_5833][bozhi_lin] 20160818 end*/
/*[Arima_5833][bozhi_lin] 20160826 end*/
#else
#define TRULY_FW_FILENAME  ""
#define HOLITECH_FW_FILENAME  ""
#endif
/*[Arima_5833][bozhi_lin] 20160328 end*/

#define MAX_IMAGE_NAME_LEN 256
/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
//static uint8_t file_fw_data[] = {0};
/*[Arima_5816][bozhi_lin] 20160114 end*/
#else
/*[Arima_5816][bozhi_lin] touch modify upgrade firmware in kernel 20160105 begin*/
#define MAX_IMAGE_NAME_LEN 256
/*The newest firmware, if update must be changed here*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5512 20151123 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5511 20151119 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5510 20151111 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5508 20151026 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5507 20151016 begin*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
static uint8_t file_fw_data_truly[] = {
	#include "CTP_2150_0x5523.i"
};

static uint8_t file_fw_data_holitech[] = {
	#include "CTP_2150_0x5578.i"
};

static uint8_t *file_fw_data = file_fw_data_truly;
/*[Arima_5816][bozhi_lin] 20160105 end*/
/*[Arima_5816][bozhi_lin] 20151014 end*/
/*[Arima_5816][bozhi_lin] 20151016 end*/
/*[Arima_5816][bozhi_lin] 20151026 end*/
/*[Arima_5816][bozhi_lin] 20151111 end*/
/*[Arima_5816][bozhi_lin] 20151119 end*/
/*[Arima_5816][bozhi_lin] 20151123 end*/
#endif


enum
{
	PageSize		= 132,
	ACK_Fail		= 0x00,
	ACK_OK			= 0xAA,
	ACK_REWRITE		= 0x55,
};

/*[Arima_5816][bozhi_lin] touch modify upgrade firmware in kernel 20160105 begin*/
#if defined(FILE_UPGRADE)
int	PageNum		  = 0;
#else
int	PageNum		  = sizeof(file_fw_data_truly)/132; /*for ektf2xxx/3xxx serial, the page number is 249/351*/
#endif
/*[Arima_5816][bozhi_lin] 20160105 end*/


enum
{
	E_FD			= -1,
};
#endif
//#define _ENABLE_DBG_LEVEL

#ifdef _ENABLE_DBG_LEVEL
#define PROC_FS_NAME    "ektf_dbg"
#define PROC_FS_MAX_LEN 8
static struct proc_dir_entry *dbgProcFile;
#endif

struct elan_ktf_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	//struct workqueue_struct *elan_wq;
	struct work_struct work;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	int elan_is_suspend;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	int irq_gpio;
	int reset_gpio;
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
	int id_gpio;
/*[Arima_5816][bozhi_lin] 20151117 end*/
	bool i2c_pull_up;
	// Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	// For Firmare Update 
	struct miscdevice firmware;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct wake_lock wakelock;
/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
	bool wakeup_dclick;
	int wakeup_gesture_type;
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/
/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
#if defined(GLOVEMODE)
	bool glove_enable;
#endif
/*[Arima_5816][bozhi_lin] 20151119 end*/
/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
	bool smartwindow_enable;
	int powersupply_type;
/*[Arima_5816][bozhi_lin] 20151231 end*/
#if defined(ESD_CHECK)
	struct delayed_work check_work; //0430
#endif
/*[Arima_5816][bozhi_lin] upgrade elan holitech touch firmware to 0x5571 and add gpio check 20151124 begin*/
	unsigned char image_name[MAX_IMAGE_NAME_LEN];
/*[Arima_5816][bozhi_lin] 20151124 end*/
};

static struct elan_ktf_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf_ts_calibrate(struct i2c_client *client);
static int elan_ktf_ts_resume(struct i2c_client *client);
static int elan_ktf_ts_suspend(struct i2c_client *client, pm_message_t mesg);
void elan_ktf_ts_hw_reset(struct i2c_client *client);
static int __hello_packet_handler(struct i2c_client *client);

#ifdef IAP_PORTION
static int FW_Update(void *x);
/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
#if defined(FILE_UPGRADE)

#else
static int Update_FW_in_Driver(void *x);
#endif
/*[Arima_5816][bozhi_lin] 20160114 end*/
#if defined(CHECKSUM_ENABLE)
static int getCheckSUM(struct i2c_client *client);
#endif
#endif
#if defined(CHECKSUM_ENABLE)
int PreTest(struct i2c_client *client);
#endif

#ifdef ELAN_2WIREICE
int elan_TWO_WIRE_ICE( struct i2c_client *client);
#endif

static int __elan_ktf_ts_poll(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(ts->irq_gpio);
		if(status==0) break;
		
		touch_debug(DEBUG_MESSAGES, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(50);
	} while (status == 1 && retry > 0);

	touch_debug(DEBUG_INFO, "[elan]%s: poll interrupt status %s\n",
	__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf_ts_poll(struct i2c_client *client)
{
	return __elan_ktf_ts_poll(client);
}

/************************************
* Restet TP 
*************************************/
void elan_ktf_ts_hw_reset(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	gpio_direction_output(ts->reset_gpio, 0);
	msleep(20);
	gpio_direction_output(ts->reset_gpio, 1);
	msleep(10);
}

void read_test(void)
{
	int pos=0;
	struct file *firmware_fp;
	mm_segment_t oldfs;
	oldfs=get_fs();
	set_fs(KERNEL_DS);
	firmware_fp = filp_open("/data/local/tmp/ME571_IP_10.ekt", O_RDONLY, S_IRUSR |S_IRGRP);
	if(PTR_ERR(firmware_fp) == -ENOENT){
		touch_debug(DEBUG_ERROR, "open file error\n");
		return;
	}
	PageNum=0;
	firmware_fp->f_pos = 0;
	for(pos = 0; pos < 500*132; pos += 132,PageNum++){
		if(firmware_fp->f_op->read(firmware_fp, firmware + pos,
					132, &firmware_fp->f_pos) != 132){
			break;
		}
	}
	touch_debug(DEBUG_INFO, "%s: PageNUM %d, Ver %x %x, ID %x %x\n",__func__,PageNum,firmware[34058],firmware[34059],firmware[34586],firmware[34587]);
	set_fs(oldfs);
	return;
}

// For Firmware Update 
int elan_iap_open(struct inode *inode, struct file *filp){ 
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_open\n");
	if (private_ts == NULL)  touch_debug(DEBUG_ERROR,"private_ts is NULL\n");
	
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
	int ret;
	char *tmp;
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_write\n");
#if 0
	/*++++i2c transfer start+++++++*/    	
	struct i2c_adapter *adap = private_ts->client->adapter;    	
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/	
#endif
	if (count > 8192)
	count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	
	if (tmp == NULL)
	return -ENOMEM;

	if (copy_from_user(tmp, buff, count)) {
		return -EFAULT;
	}

	/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	msg.flags = 0x00;// 0x00
	msg.len = count;
	msg.buf = (char *)tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
	
	ret = i2c_master_send(private_ts->client, tmp, count);
#endif	
	/*++++i2c transfer end+++++++*/

	//if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
	kfree(tmp);
	//return ret;
	return (ret == 1) ? count : ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
	char *tmp;
	int ret;  
	long rc;
	touch_debug(DEBUG_MESSAGES,"[ELAN]into elan_iap_read\n");
#if 0
	/*++++i2c transfer start+++++++*/
	struct i2c_adapter *adap = private_ts->client->adapter;
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/
#endif

	if (count > 8192)
	count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
	return -ENOMEM;
	/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	//msg.flags |= I2C_M_RD;
	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
	ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
	/*++++i2c transfer end+++++++*/
	if (ret >= 0)
	rc = copy_to_user(buff, tmp, count);
	
	kfree(tmp);

	//return ret;
	return (ret == 1) ? count : ret;
	
}

static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;

	touch_debug(DEBUG_MESSAGES,"[ELAN]into elan_iap_ioctl\n");
	touch_debug(DEBUG_MESSAGES,"cmd value %x\n",cmd);
	
	switch (cmd) {        
	case IOCTL_I2C_SLAVE: 
		private_ts->client->addr = (int __user)arg;
		//file_fops_addr = 0x10;
		break;   
	case IOCTL_MAJOR_FW_VER:            
		break;        
	case IOCTL_MINOR_FW_VER:            
		break;        
	case IOCTL_RESET:
		// modify
		elan_ktf_ts_hw_reset(private_ts->client);
		break;
	case IOCTL_IAP_MODE_LOCK:
		if(work_lock==0)
		{
			#ifdef ESD_CHECK 
			cancel_delayed_work_sync(&private_ts->check_work); 
			#endif
			work_lock=1;
			disable_irq(private_ts->client->irq);
			flush_work(&private_ts->work);
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if(work_lock==1)
		{			
			work_lock=0;
			enable_irq(private_ts->client->irq);
			#ifdef ESD_CHECK
			schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500)); //1218	
			#endif
		}
		break;
	case IOCTL_CHECK_RECOVERY_MODE:
		return RECOVERY;
		break;
	case IOCTL_FW_VER:
/*[Arima_5830][bozhi_lin] fix touch firmware version will show 0x0000 after do aats 20160503 begin*/
		#ifdef ESD_CHECK
		live_state = 1;
		#endif
		disable_irq(private_ts->client->irq);
		__fw_packet_handler(private_ts->client);
		enable_irq(private_ts->client->irq);
/*[Arima_5830][bozhi_lin] 20160503 end*/
		return FW_VERSION;
		break;
	case IOCTL_X_RESOLUTION:
/*[Arima_5830][bozhi_lin] fix touch firmware version will show 0x0000 after do aats 20160503 begin*/
		#ifdef ESD_CHECK
		live_state = 1;
		#endif
		disable_irq(private_ts->client->irq);
		__fw_packet_handler(private_ts->client);
		enable_irq(private_ts->client->irq);
/*[Arima_5830][bozhi_lin] 20160503 end*/
		return X_RESOLUTION;
		break;
	case IOCTL_Y_RESOLUTION:
/*[Arima_5830][bozhi_lin] fix touch firmware version will show 0x0000 after do aats 20160503 begin*/
		#ifdef ESD_CHECK
		live_state = 1;
		#endif
		disable_irq(private_ts->client->irq);
		__fw_packet_handler(private_ts->client);
		enable_irq(private_ts->client->irq);
/*[Arima_5830][bozhi_lin] 20160503 end*/
		return Y_RESOLUTION;
		break;
	case IOCTL_FW_ID:
/*[Arima_5830][bozhi_lin] fix touch firmware version will show 0x0000 after do aats 20160503 begin*/
		#ifdef ESD_CHECK
		live_state = 1;
		#endif
		disable_irq(private_ts->client->irq);
		__fw_packet_handler(private_ts->client);
		enable_irq(private_ts->client->irq);
/*[Arima_5830][bozhi_lin] 20160503 end*/
		return FW_ID;
		break;
	case IOCTL_ROUGH_CALIBRATE:
		return elan_ktf_ts_calibrate(private_ts->client);
	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->irq_gpio), ip);
		break;	
	case IOCTL_RESUME:
		//elan_ktf_ts_resume(private_ts->client);
		break;	
	case IOCTL_POWER_LOCK:
		power_lock=1;
		break;
	case IOCTL_POWER_UNLOCK:
		power_lock=0;
		break;
#ifdef IAP_PORTION		
	case IOCTL_GET_UPDATE_PROGREE:
		update_progree=(int __user)arg;
		break; 
	case IOCTL_FW_UPDATE:
		read_test();
		//Update_FW_in_Driver(0);
		FW_Update(0);
		break;
#endif
#ifdef ELAN_2WIREICE
	case IOCTL_2WIREICE:
		elan_TWO_WIRE_ICE(private_ts->client);
		break;		
#endif
	case IOCTL_PTEST:
#if defined(CHECKSUM_ENABLE)
		PreTest(private_ts->client);
#endif
		break;
	case IOCTL_CIRCUIT_CHECK:
		return circuit_ver;
		break;
	default:      
		touch_debug(DEBUG_ERROR,"[elan] Un-known IOCTL Command %d\n", cmd);   
		break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
	.open =         elan_iap_open,    
	.write =        elan_iap_write,    
	.read = 	elan_iap_read,    
	.release =	elan_iap_release,    
/*[Arima_5833][bozhi_lin] touch with different ioctl in 32bit build 20160525 begin*/
/*[Arima_5830][bozhi_lin] fix touch upgrade firmware manually fail cause by ioctl 20160219 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
	.compat_ioctl=elan_iap_ioctl,
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
	.unlocked_ioctl=elan_iap_ioctl, 
#else
	.compat_ioctl=elan_iap_ioctl,
#endif
/*[Arima_5830][bozhi_lin] 20160219 end*/
/*[Arima_5833][bozhi_lin] 20160525 end*/
};



#ifdef IAP_PORTION
int HID_EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a}; 
	uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34}; 
	uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10}; 
	uint8_t buff[67] = {0};


	len = i2c_master_send(private_ts->client, flash_key,  37);
	if (len != 37) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

	mdelay(20);

        len = i2c_master_send(private_ts->client, isp_cmd,  37);
	if (len != 37) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);


	mdelay(20);
 	len = i2c_master_send(private_ts->client, check_addr,  sizeof(check_addr));
	if (len != sizeof(check_addr)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Check Address fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", check_addr[7], check_addr[8], check_addr[9], check_addr[10]);
	
	mdelay(20);	

	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}
	else {
printk("[Check Addr]: ");
for (j=0; j<37; j++)
	printk("%x ", buff[j]);
printk("\n");
	
	}
	
	return 0;
}
int EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50}; //{0x45, 0x49, 0x41, 0x50};
	len = i2c_master_send(private_ts->client, isp_cmd,  4);
	if (len != 4) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

//1105
int CheckIapMode(void)
{
	char buff[4] = { 0 }, len = 0;
  
	len = i2c_master_recv( private_ts->client, buff, sizeof( buff ));
	if( sizeof( buff ) != len )
	{
		printk("[ELAN] CheckIapMode ERROR: read data error,len = %d\r\n", len );
		return -1;
	}
	else
	{
		if(( buff[0] == 0x55 ) && ( buff[1] == 0xAA ) && ( buff[2] == 0x33 ) && ( buff[3] == 0xCC ))
		{
			printk("[ELAN] CheckIapMode is 55 AA 33 CC\n");
			return 0;
		}
		else
		{
			printk("[ELAN] Mode = 0x%02X 0x%02X 0x%02X 0x%02X\r\n", buff[0], buff[1], buff[2], buff[3] );
			printk("[ELAN] ERROR: CheckIapMode error\n");
			return -1;
		}
	}
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
	if (len != byte) 
	{
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read page error, read error. len=%d\r\n", __func__, len);
		return -1;
	}

	return 0;
}

int WritePage(const u8 * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage,  byte);
	if (len != byte) 
	{
		touch_debug(DEBUG_ERROR,"[ELAN] %s: write page error, write error. len=%d\r\n", __func__, len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int rc = 0;

	uint8_t buff[67] = {0};
#ifdef ELAN_HID_I2C	
	rc = elan_ktf_ts_poll(client);
#endif
	rc = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (rc != sizeof(buff)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: Read ACK Data error. rc=%d\r\n", __func__, rc);
		return -1;
	}

	touch_debug(DEBUG_MESSAGES, "[ELAN] %s: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",__func__,buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9],buff[10],buff[11]);
#ifndef ELAN_HID_I2C
	if (buff[0] == 0xaa && buff[1] == 0xaa) 
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;
#endif	
	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent,page_tatol=351,percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i=0; i<((page)/10); i++) {
		str[i] = '#';
		str[i+1] = '\0';
	}
	/*
	page_tatol=page+PageNum*(ic_num-j);
	percent = ((100*page)/(PageNum));
	percent_tatol = ((100*page_tatol)/(PageNum*ic_num));
*/
	percent = ((100*page)/(PageNum));
	if ((page) == (PageNum))
	percent = 100;

	if ((page_tatol) == (PageNum*ic_num))
	percent_tatol = 100;		

	touch_debug(DEBUG_INFO, "\rprogress %s| %d %d", str, percent,page);
	
	if (page == (PageNum))
		touch_debug(DEBUG_INFO, "\n");

}

/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
#if defined(FILE_UPGRADE)

#else
static int Update_FW_in_Driver(void *x)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0, iapmodecnt=0, checksumcnt=0; // For IAP_RESTART
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
	disable_irq(private_ts->client->irq);
/*[Arima_5816][bozhi_lin] 20151014 end*/
	mutex_lock(&ktf_mutex);
	touch_debug(DEBUG_INFO, "[ELAN] %s: Update FW\n", __func__);
	
IAP_RESTART:	
	fwupd_flag = 1;
	curIndex=0;
	data=I2C_DATA[0];//Master
	touch_debug(DEBUG_INFO, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	#ifdef ESD_CHECK
	live_state = 1;
	#endif
	fwupd_flag = 1;
	work_lock = 1 ;
	power_lock = 1;
	
	if(RECOVERY != 0x80)
	{
		touch_debug(DEBUG_MESSAGES, "[ELAN] Firmware upgrade normal mode !\n");
	} else
		touch_debug(DEBUG_MESSAGES, "[ELAN] Firmware upgrade recovery mode !\n");
	
	elan_ktf_ts_hw_reset(private_ts->client);
	mdelay(20); 
	res = EnterISPMode(private_ts->client);	 //enter ISP mode
	if (res <0){
		touch_debug(DEBUG_ERROR,"[ELAN] EnterISPMode Fail\n");
		goto IAP_FAIL;
	}
	mdelay(10);
	
	res = CheckIapMode(); 
	if (res < 0){
		iapmodecnt++;
		if (iapmodecnt < 3){
			goto IAP_RESTART;
		}else{
			touch_debug(DEBUG_ERROR,"[ELAN] Firmware upgrade CheckIAP Fail 3 times\n");
			goto IAP_FAIL; //1106
		}
	}
  
	/* Send Dummy Byte	*/
	touch_debug(DEBUG_INFO,"[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
		printk("[ELAN] dummy error code = %d\n",res);
		touch_debug(DEBUG_ERROR,"[ELAN] dummy error code = %d\n",res);
		goto IAP_FAIL;
	}
	mdelay(10);
	/* Start IAP*/
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
		PAGE_REWRITE:
#if 1 // 8byte mode
		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{		
				//			printk("[ELAN] byte %d\n",byte_count);	
				//			printk("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 8;
				res = WritePage(szBuff, 8);
			}
			else
			{
				//			printk("byte %d\n",byte_count);
				//			printk("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 4;
				res = WritePage(szBuff, 4); 
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif 
#if 0 // 132byte mode		
		//szBuff = file_fw_data + curIndex;
		szBuff = firmware + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
		#if 1 //if use old bootcode
		if(iPage==PageNum)
		{
			mdelay(600); 			 
		}
		else
		{
			mdelay(50); 			 
		}	
		#endif
		
		res = GetAckData(private_ts->client);
		if (ACK_OK != res) 
		{
			mdelay(50); 
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			//1105 --start
			if ( res == ACK_REWRITE )
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
					touch_debug(DEBUG_ERROR, "[ELAN] %dth page ReWrite %d times fails!\n", iPage, PAGERETRY);
					goto IAP_FAIL;
				}
				else
				{
					touch_debug(DEBUG_ERROR,"[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					curIndex = curIndex - PageSize;
					goto PAGE_REWRITE;
				}
			}else{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5)
				{
					touch_debug(DEBUG_ERROR,"[ELAN] IAP 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					goto IAP_FAIL;
				}
				else
				{
					touch_debug(DEBUG_ERROR,"[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
			//1105
		}
		else
		{       
			print_progress(iPage,ic_num,i);
		}
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)
	
	res= __hello_packet_handler(private_ts->client);
	if (res == 0x80){
		touch_debug(DEBUG_ERROR, "[ELAN] Update Firmware Failed, Recover = 0x80!\n");
		checksumcnt++;
		if(checksumcnt<3)
			goto IAP_RESTART;
	} else {
		touch_debug(DEBUG_ERROR, "[ELAN] Update ALL Firmware successfully! %x\n", res);
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
		// update firmware version
		__fw_packet_handler(private_ts->client);
		res = getCheckSUM(private_ts->client);
		if(res == -1 && checksumcnt<3){
			checksumcnt++;
			printk("[elan] Update_FW_One: getCheckSUM error, goto IAP.\n");
			goto IAP_RESTART;
		}
	}
  
	enable_irq(private_ts->client->irq);
	mutex_unlock(&ktf_mutex);
	fwupd_flag = 0;
	power_lock = 0;
	work_lock = 0 ;
	return 0;
/*[Arima_5816][bozhi_lin] 20151014 end*/

IAP_FAIL:	
	enable_irq(private_ts->client->irq);
	mutex_unlock(&ktf_mutex);
	fwupd_flag = 0;
	power_lock = 0; 
	work_lock = 0;  
	
	return -1;
}
#endif
/*[Arima_5816][bozhi_lin] 20160114 end*/

#endif
// End Firmware Update

/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
//Glove Mode 1117
#if defined(GLOVEMODE)
int EnterGloveMode(void)
{
	int len = 0;
	uint8_t enter_glove[4] = {0x54, 0x57, 0x02, 0x01}; 

	len = i2c_master_send(private_ts->client, enter_glove,  4);
	if (len != 4) { //1118
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: enter_glove fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] enter_glove write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", enter_glove[0], enter_glove[1], enter_glove[2], enter_glove[3]);
	return 0;
}

int ExitGloveMode(void)
{
	int len = 0;
	uint8_t exit_glove[4] = {0x54, 0x57, 0x00, 0x01};

	len = i2c_master_send(private_ts->client, exit_glove,  4);
	if (len != 4) { //1118
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: exit_glove fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] exit_glove write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", exit_glove[0], exit_glove[1], exit_glove[2], exit_glove[3]);
	return 0;
}
#endif
//Glove Mode 1117
/*[Arima_5816][bozhi_lin] 20151119 end*/

// Star 2wireIAP which used I2C to simulate JTAG function
#ifdef ELAN_2WIREICE
static uint8_t file_bin_data[] = {
	//#include "2wireice.i"
};

int write_ice_status=0;
int shift_out_16(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbf,0xff};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}
int tms_reset(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0xff,0xff,0xff,0xbf};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int mode_gen(struct i2c_client *client){
	int res;
	int retry = 5;
	uint8_t buff[] = {0xff,0xff,0xff,0x31,0xb7,0xb7,0x7b,0xb7,0x7b,0x7b,0xb7,0x7b,0xf3,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xf1};
	uint8_t buff_1[] = {0x2a,0x6a,0xa6,0xa6,0x6e};
	char mode_buff[2]={0};
	do {
		res = i2c_master_send(client, buff,  sizeof(buff));
		if (res != sizeof(buff)) {
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: mode_gen write buff error, write  error. res=%d\r\n", res);
		}
		else{
			touch_debug(DEBUG_MESSAGES,"[ELAN] mode_gen write buff successfully.\r\n");
			break;
		}
		mdelay(20);
		retry -=1;
	} while(retry);
	res = i2c_master_recv(client, mode_buff, sizeof(mode_buff));
	if (res != sizeof(mode_buff)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: mode_gen read data error, write  error. res=%d\r\n", res);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] mode gen read successfully(a6 59)! buff[0]=0x%x  buff[1]=0x%x \r\n", mode_buff[0], mode_buff[1]);

	res = i2c_master_send(client, buff_1,  sizeof(buff_1));
	if (res != sizeof(buff_1)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: mode_gen write buff_1 error. res=%d\r\n", res);
		return -1;
	}
	return res;
}

int word_scan_out(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26,0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int long_word_scan_out(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26,0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}


int bit_manipulation(int TDI, int TMS, int TCK, int TDO,int TDI_1, int TMS_1, int TCK_1, int TDO_1){
	int res; 
	res= ((TDI<<3 |TMS<<2 |TCK |TDO)<<4) |(TDI_1<<3 |TMS_1<<2 |TCK_1 |TDO_1);
	return res;
}

int ins_write(struct i2c_client *client, uint8_t buf){
	int res=0;
	int length=13;
	uint8_t write_buf[7]={0};
	int TDI_bit[13]={0};
	int TMS_bit[13]={0};
	int i=0;
	uint8_t buf_rev=0;
	int TDI=0, TMS=0, TCK=0,TDO=0;
	int bit_tdi, bit_tms;
	int len;
	
	for(i=0;i<8;i++) 
	{
		buf_rev = buf_rev | (((buf >> i) & 0x01) << (7-i));
	}
	
	
	TDI = (0x7<<10) | buf_rev <<2 |0x00;
	TMS = 0x1007;
	TCK=0x2;
	TDO=1;
	
	for ( len=0; len<=length-1; len++){
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		TDI_bit[length-1-len] =bit_tdi;
		TMS_bit[length-1-len] = bit_tms;
		TDI = TDI >>1;
		TMS = TMS >>1;
	}

	for (len=0;len<=length-1;len=len+2){
		if (len == length-1 && len%2 ==0)
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0); 	
		else
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 	
		write_buf[len/2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}


int word_scan_in(struct i2c_client *client, uint16_t buf){
	int res=0;
	uint8_t write_buf[10]={0};
	int TDI_bit[20]={0};
	int TMS_bit[20]={0};
	
	
	int TDI =  buf <<2 |0x00;
	int  TMS = 0x7;
	int  TCK=0x2;
	int TDO=1;
	
	int bit_tdi, bit_tms;
	int len;
	
	for ( len=0; len<=19; len++){    //length =20
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		
		TDI_bit[19-len] =bit_tdi;
		TMS_bit[19-len] = bit_tms;
		TDI = TDI >>1;
		TMS = TMS >>1;
	}

	for (len=0;len<=19;len=len+2){
		if (len == 19 && len%2 ==0)
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0,0,0,0); 
		else
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 
		write_buf[len/2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

int long_word_scan_in(struct i2c_client *client, int buf_1, int buf_2){
	uint8_t write_buf[18]={0};
	uint8_t TDI_bit[36]={0};
	uint8_t TMS_bit[36]={0};

	int TDI_1 = buf_1;
	int TDI_2 = (buf_2<<2) |0x00;
	int TMS = 0x7;
	int TCK=0x2;
	int TDO=1;
	
	int bit_tdi, bit_tms;
	int len=0;
	int res=0;


	for ( len=0; len<=35; len++){    //length =36

		if(len<18)
		{
			bit_tdi = TDI_2 & 0x1;
		}
		else
		{
			bit_tdi = TDI_1 & 0x1;
		}
		bit_tms = TMS & 0x1;
		
		TDI_bit[35-len] =bit_tdi;
		TMS_bit[35-len] = bit_tms;
		if(len<18)
		{
			TDI_2 = TDI_2 >>1;
		}
		else
		{
			TDI_1 = TDI_1 >>1;
		}
		TMS = TMS >>1;
		bit_tdi=0;
		bit_tms=0;
	}


	for (len=0;len<=35;len=len+2){
		if (len == 35 && len%2 ==0)
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0,0,0,1);
		else
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO);
		write_buf[len/2] = res;
	}
	
	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

uint16_t trimtable[8]={0};

int Read_SFR(struct i2c_client *client, int open){
	uint8_t voltage_recv[2]={0};
	
	int count, ret;
	//uint16_t address_1[8]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007};
	
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);  
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	//  0
	ins_write(client, 0x6f);  //IO Write
	long_word_scan_in(client, 0x007f, 0x9002);  //TM=2h
	ins_write(client, 0x68);  //Program Memory Sequential Read
	word_scan_in(client, 0x0000);  //set Address 0x0000
	shift_out_16(client);   //move data to I2C buf
	
	mdelay(10);
	count = 0;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  1
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0001);
	shift_out_16(client); 

	mdelay(1);
	count=1;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  2
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0002);
	shift_out_16(client); 

	mdelay(1);
	count=2;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  3
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0003);
	shift_out_16(client); 

	mdelay(1);
	count=3;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  4
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0004);
	shift_out_16(client); 

	mdelay(1);
	count=4;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  5
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0005);
	shift_out_16(client); 

	mdelay(1);
	count=5;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}

	
	//  6
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0006);
	shift_out_16(client); 

	mdelay(1);
	count=6;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  7
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0007);
	shift_out_16(client); 

	mdelay(1);
	count=7;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	if (open == 1)
	trimtable[count]=voltage_recv[0]<<8 |  (voltage_recv[1] & 0xbf);
	else
	trimtable[count]=voltage_recv[0]<<8 | (voltage_recv[1] | 0x40);
	touch_debug(DEBUG_ERROR,"[ELAN] Open_High_Voltage recv  voltage_recv buff[0]=%x buff[1]=%x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);


	
	/*	
	for (count =0; count <8; count++){

	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, address_1[count]);
		shift_out_16(client); 

	mdelay(10);
	//count=6;
		ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	printk("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 

	}
	
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

*/	
	return 0;
}

int Write_SFR_2k(struct i2c_client *client, int open){

	//set page 1
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x0001, 0x0100);
	if(open==1)
	{
		//set HV enable
		touch_debug(DEBUG_MESSAGES,"%s set HV enable\n",__func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc041);
	}
	else
	{
		//set HV disable
		touch_debug(DEBUG_MESSAGES,"%s set HV disable\n",__func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc040);
	}
	return 0;
}

int Write_SFR(struct i2c_client *client){

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9001);


	ins_write(client, 0x66);  // Program Memory Write
	long_word_scan_in(client, 0x0000, trimtable[0]);
	ins_write(client, 0xfd);  //Set up the initial addr for sequential access
	word_scan_in(client,0x7f);
	
	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0001, trimtable[1]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0002, trimtable[2]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0003, trimtable[3]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0004, trimtable[4]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0005, trimtable[5]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);
	
	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0006, trimtable[6]);	
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0007, trimtable[7]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x7f, 0x8000);	   
	/*
	for (count=0;count<8;count++){
			ins_write(client, 0x66);
		long_word_scan_in(client, 0x0000+count, trimtable[count]);
		
	}
	*/

	return 0;
}

int Enter_Mode(struct i2c_client *client){
	mode_gen(client);
	tms_reset(client);
	ins_write(client,0xfc); //system reset
	tms_reset(client);
	return 0;
}
int Open_High_Voltage(struct i2c_client *client, int open){
#ifdef EKTF3K_FLASH
	Read_SFR(client, open);
	Write_SFR(client);
	Read_SFR(client, open);

#endif
	Write_SFR_2k(client, open);
	return 0;

}

int Mass_Erase(struct i2c_client *client){
	char mass_buff[4]={0};
	char mass_buff_1[2]={0};
	int ret, finish=0, i=0;
	touch_debug(DEBUG_MESSAGES,"[Elan] Mass_Erase!!!!\n");
	ins_write(client,0x01); //id code read
	mdelay(2);
	long_word_scan_out(client);

	ret = i2c_master_recv(client, mass_buff, sizeof(mass_buff));
	touch_debug(DEBUG_MESSAGES,"[elan] Mass_Erase mass_buff=%x %x %x %x(c0 08 01 00)\n", mass_buff[0],mass_buff[1],mass_buff[2],mass_buff[3]);  //id: c0 08 01 00
	/* / add for test
	ins_write(client, 0xf3);
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	printk("[elan] Mass_Erase mass_buff_1=%x %x(a0 00)\n", mass_buff_1[0],mass_buff_1[1]);  // a0 00 : stop
//add for test

	//read low->high 5th bit
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);

// add for test
	ins_write(client, 0xf3);
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	printk("[elan] Mass_Erase (II) mass_buff_1=%x %x(40 00)\n", mass_buff_1[0],mass_buff_1[1]);  // 40 00
//add for test
	mdelay(10); //for malata
	*/
	
	ins_write(client,0x6f);  //IO Write
	/*add by herman*/
	long_word_scan_in(client,0x007e,0x0020);

	long_word_scan_in(client,0x007f,0x4000);//orig 4000
	long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x8000);
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x9040);
	ins_write(client,0x66); //Program data Write
	long_word_scan_in(client, 0x0000,0x8765);//change by herman
	ins_write(client,0x6f);  //IO Write
	long_word_scan_in(client, 0x007f,0x8000);	//clear flash control PROG

	ins_write(client,0xf3);
	
	while (finish==0){
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));			
		if (ret != sizeof(mass_buff_1)) {
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: read data error. res=%d\r\n", ret);
			return -1;
		}
		else
		{
			finish = (mass_buff_1[1] >> 4 ) & 0x01;
			touch_debug(DEBUG_MESSAGES,"[ELAN] mass_buff_1[0]=%x, mass_buff_1[1]=%x (80 10)!!!!!!!!!! finish=%d \n", mass_buff_1[0], mass_buff_1[1], finish);  //80 10: OK, 80 00: fail
		}
		if (mass_buff_1[1]!= I2C_DATA[0] && finish!=1 && i<100) {  
			mdelay(100);
			//printk("[elan] mass_buff_1[1] >>4  !=1\n");
			i++;
			if (i == 50) {
				touch_debug(DEBUG_ERROR,"[elan] Mass_Erase fail ! \n");
				//return -1;  //for test
			}
		}
		
	}

	return 0;
}

int Reset_ICE(struct i2c_client *client){
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int res;
	touch_debug(DEBUG_INFO,"[Elan] Reset ICE!!!!\n");
	ins_write(client, 0x94);
	ins_write(client, 0xd4);
	ins_write(client, 0x20);
	client->addr = I2C_DATA[0];////Modify address before 2-wire
	elan_ktf_ts_hw_reset(client); 
	mdelay(250);
	res = __hello_packet_handler(client);
	
	return 0;
}

int normal_write_func(struct i2c_client *client, int j, uint8_t *szBuff){
	//char buff_check=0;
	uint16_t szbuff=0, szbuff_1=0;
	uint16_t sendbuff=0;
	int write_byte, iw;
	
	ins_write(client,0xfd);
	word_scan_in(client, j*64); 
	
	ins_write(client,0x65);  //Program data sequential write

	write_byte =64;

	for(iw=0;iw<write_byte;iw++){ 
		szbuff = *szBuff;
		szbuff_1 = *(szBuff+1);
		sendbuff = szbuff_1 <<8 |szbuff;
		touch_debug(DEBUG_MESSAGES,"[elan]  Write Page sendbuff=0x%04x @@@\n", sendbuff);
		//mdelay(1);
		word_scan_in(client, sendbuff); //data????   buff_read_data
		szBuff+=2;
		
	}
	return 0;
}

int fastmode_write_func(struct i2c_client *client, int j, uint8_t *szBuff){
	uint8_t szfwbuff=0, szfwbuff_1=0;
	uint8_t sendfwbuff[130]={0};
	uint8_t tmpbuff;
	int i=0, len=0;
	private_ts->client->addr = 0x76;

	sendfwbuff[0] = (j*64)>>8;
	tmpbuff = ((j*64)<< 8) >> 8;
	sendfwbuff[1] = tmpbuff;
	//printk("fastmode_write_func, sendfwbuff[0]=0x%x, sendfwbuff[1]=0x%x\n", sendfwbuff[0], sendfwbuff[1]);

	for (i=2;i < 129; i=i+2) {      //  1 Page = 64 word, 1 word=2Byte
		
		szfwbuff = *szBuff;
		szfwbuff_1 = *(szBuff+1);
		sendfwbuff[i] = szfwbuff_1;
		sendfwbuff[i+1] = szfwbuff;
		szBuff+=2;
		//printk("[elan] sendfwbuff[%d]=0x%x, sendfwbuff[%d]=0x%x\n", i, sendfwbuff[i], i+1, sendfwbuff[i+1]);
	}

	
	len = i2c_master_send(private_ts->client, sendfwbuff,  130);
	if (len != 130) {  //address+data(128)
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: fastmode write page error, write error. len=%d, Page %d\r\n", len, j);
		return -1;
	}
	//printk("fastmode_write_func, send len=%d (130), Page %d --\n", len, j);

	private_ts->client->addr = 0x77;
	
	return 0;
}


int ektSize;
int lastpage_byte;
int lastpage_flag=0;
int Write_Page(struct i2c_client *client, int j, uint8_t *szBuff){
	int len, finish=0;
	char buff_read_data[2];
	int i=0;
	
	ins_write(client,0x6f);   //IO Write
	//long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x8000);
	long_word_scan_in(client,0x007f,0x9400);

	ins_write(client,0x66);    //Program Data Write
	//long_word_scan_in(client,0x0000,0x5a5a);
	long_word_scan_in(client, j*64,0x0000);
	//printk("[elan] j*64=0x%x @@ \n", j*64);
	//long_word_scan_in(client, j*64,0x5a5a);  //set ALE
	
	//normal_write_func(client, j, szBuff); ////////////choose one : normal / fast mode
	fastmode_write_func(client, j, szBuff); //////////
	
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x9000);

	//ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x8000);

	ins_write(client, 0xf3);  //Debug Reg Read
	
	while (finish==0){
		word_scan_out(client);
		len=i2c_master_recv(client, buff_read_data, sizeof(buff_read_data));
		if (len != sizeof(buff_read_data)) {
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Write_Page read buff_read_data error, len=%d\r\n", len);
			return E_FD;
		}
		else
		{
			finish = (buff_read_data[1] >> 4 ) & 0x01;
			//printk("[ELAN] read data successfully! buff[0]=0x%x  buff[1]=0x%x  finish=%d\r\n", buff_read_data[0], buff_read_data[1], finish);  //80 10: ok
		}
		if (finish!=1) {  
			mdelay(10);
			//printk("[elan] Write_Page finish !=1\n");
			i++;
			if (i==50){ 
				touch_debug(DEBUG_ERROR,"[elan] Write_Page finish !=1, Page=%d\n", j);
				write_ice_status=1;
				return -1;
			}
		}

	}
	return 0;
}

int fastmode_read_func(struct i2c_client *client, int j, uint8_t *szBuff){
	uint8_t szfrbuff=0, szfrbuff_1=0;
	uint8_t sendfrbuff[2]={0};
	uint8_t recvfrbuff[130]={0};
	uint16_t tmpbuff;
	int i=0, len=0, retry=0;

	
	ins_write(client,0x67);
	
	private_ts->client->addr = 0x76;

	sendfrbuff[0] = (j*64)>>8;
	tmpbuff = ((j*64)<< 8) >> 8;
	sendfrbuff[1] = tmpbuff;
	//printk("fastmode_write_func, sendfrbuff[0]=0x%x, sendfrbuff[1]=0x%x\n", sendfrbuff[0], sendfrbuff[1]);
	len = i2c_master_send(private_ts->client, sendfrbuff,  sizeof(sendfrbuff));

	len = i2c_master_recv(private_ts->client, recvfrbuff,  sizeof(recvfrbuff));
	//printk("fastmode_read_func, recv len=%d (128)\n", len);
	
	for (i=2;i < 129;i=i+2){ 
		szfrbuff=*szBuff;
		szfrbuff_1=*(szBuff+1);
		szBuff+=2;
		if (recvfrbuff[i] != szfrbuff_1 || recvfrbuff[i+1] != szfrbuff)  
		{
			touch_debug(DEBUG_ERROR,"[elan] @@@@Read Page Compare Fail. recvfrbuff[%d]=%x, recvfrbuff[i+1]=%x, szfrbuff_1=%x, szfrbuff=%x, ,j =%d@@@@@@@@@@@@@@@@\n\n", i,recvfrbuff[i], recvfrbuff[i+1], szfrbuff_1, szfrbuff, j);
			write_ice_status=1;
			retry=1;
		}
		break;//for test
	}

	private_ts->client->addr = 0x77;
	if(retry==1)
	{
		return -1;
	}
	else
	return 0;
}


int normal_read_func(struct i2c_client *client, int j,  uint8_t *szBuff){
	char read_buff[2];
	int m, len, read_byte;
	uint16_t szbuff=0, szbuff_1=0;
	
	ins_write(client,0xfd);
	
	//printk("[elan] Read_Page, j*64=0x%x\n", j*64);
	word_scan_in(client, j*64);
	ins_write(client,0x67);

	word_scan_out(client);

	read_byte=64;
	//for(m=0;m<64;m++){
	for(m=0;m<read_byte;m++){
		// compare......
		word_scan_out(client);
		len=i2c_master_recv(client, read_buff, sizeof(read_buff));

		szbuff=*szBuff;
		szbuff_1=*(szBuff+1);
		szBuff+=2;
		touch_debug(DEBUG_MESSAGES,"[elan] Read Page: byte=%x%x, szbuff=%x%x \n", read_buff[0], read_buff[1],szbuff, szbuff_1);

		if (read_buff[0] != szbuff_1 || read_buff[1] != szbuff) 
		{
			touch_debug(DEBUG_ERROR,"[elan] @@@@@@@@@@Read Page Compare Fail. j =%d. m=%d.@@@@@@@@@@@@@@@@\n\n", j, m);
			write_ice_status=1;
		}
	}
	return 0;
}


int Read_Page(struct i2c_client *client, int j,  uint8_t *szBuff){
	int res=0;
	ins_write(client,0x6f);
	//long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x9000);
	ins_write(client,0x68);

	//mdelay(10); //for malata
	//normal_read_func(client, j,  szBuff); ////////////////choose one: normal / fastmode
	fastmode_read_func(client, j,  szBuff);

	//Clear Flashce
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x0000);
	if(res==-1)
	{
		return -1;
	}
	return 0;
	
}



int TWO_WIRE_ICE(struct i2c_client *client){
	int i;

	uint8_t *szBuff = NULL;
	//char szBuff[128]={0};
	int curIndex = 0;
	int PageSize=128;
	int res;
	//int ektSize;
	//test
	write_ice_status=0;
	ektSize = sizeof(file_bin_data) /PageSize;
	client->addr = 0x77;////Modify address before 2-wire

	touch_debug(DEBUG_INFO,"[Elan] ektSize=%d ,modify address = %x\n ", ektSize, client->addr);

	i = Enter_Mode(client);
	i = Open_High_Voltage(client, 1);     
	if (i == -1)
	{
		touch_debug(DEBUG_ERROR,"[Elan] Open High Voltage fail\n");
		return -1;
	}
	//return 0;

	i = Mass_Erase(client);  //mark temp
	if (i == -1)  {
		touch_debug(DEBUG_ERROR,"[Elan] Mass Erase fail\n");
		return -1;
	}


	//for fastmode
	ins_write(client,0x6f);
	long_word_scan_in(client, 0x007e, 0x0036);
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007e, 0x0023);	//add by herman


	// client->addr = 0x76;////Modify address before 2-wire 
	touch_debug(DEBUG_INFO,"[Elan-test] client->addr =%2x\n", client->addr);    
	//for fastmode
	for (i =0 ; i<ektSize; i++)
	{
		szBuff = file_bin_data + curIndex; 
		curIndex =  curIndex + PageSize; 	
		//	printk("[Elan] Write_Page %d........................wait\n ", i);	

		res=Write_Page(client, i, szBuff);
		if (res == -1) 
		{
			touch_debug(DEBUG_ERROR,"[Elan] Write_Page %d fail\n ", i);
			break;
		}
		//printk("[Elan] Read_Page %d........................wait\n ", i);
		mdelay(1);
		Read_Page(client,i, szBuff);
		//printk("[Elan] Finish  %d  Page!!!!!!!.........wait\n ", i);	
	}


	if(write_ice_status==0)
	{
		touch_debug(DEBUG_INFO,"[elan] Update_FW_Boot Finish!!! \n");
	}
	else
	{
		touch_debug(DEBUG_INFO,"[elan] Update_FW_Boot fail!!! \n");
	}

	i = Open_High_Voltage(client, 0);     
	if (i == -1) return -1; //test

	Reset_ICE(client);

	return 0;	
}

int elan_TWO_WIRE_ICE( struct i2c_client *client) // for driver internal 2-wire ice
{
	work_lock=1;
	disable_irq(private_ts->client->irq);
	//wake_lock(&private_ts->wakelock);
	TWO_WIRE_ICE(client);
	work_lock=0;
	enable_irq(private_ts->client->irq);
	//wake_unlock(&private_ts->wakelock);
	return 0;
}
// End 2WireICE
#endif


int CheckISPstatus(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t checkstatus[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x18}; 
	uint8_t buff[67] = {0};

	len = i2c_master_send(private_ts->client, checkstatus, sizeof(checkstatus));
	if (len != sizeof(checkstatus)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n", checkstatus[0], checkstatus[1], checkstatus[2], checkstatus[3], checkstatus[4], checkstatus[5]);
	
	mdelay(10);	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}
	else {
printk("[Check status]: ");
for (j=0; j<37; j++)
	printk("%x ", buff[j]);
printk("\n");
		if (buff[6] == 0x88)	return 0x88; /* return recovery mode 0x88 */
	}
	
	return 0;
}

int RecoveryISP(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a}; 
//	uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34}; 
	uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10}; 
	uint8_t buff[67] = {0};


	len = i2c_master_send(private_ts->client, flash_key,  37);
	if (len != 37) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

	mdelay(20);
/*
        len = i2c_master_send(private_ts->client, isp_cmd,  37);
	if (len != 37) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);
*/

	mdelay(20);
 	len = i2c_master_send(private_ts->client, check_addr,  sizeof(check_addr));
	if (len != sizeof(check_addr)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Check Address fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", check_addr[7], check_addr[8], check_addr[9], check_addr[10]);
	
	mdelay(20);	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}
	else {
printk("[Check Addr]: ");
for (j=0; j<37; j++)
	printk("%x ", buff[j]);
printk("\n");
	
	}

	return 0;
}

int SendEndCmd(struct i2c_client *client)
{
	int len = 0;
	uint8_t send_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x1A}; 

	len = i2c_master_send(private_ts->client, send_cmd, sizeof(send_cmd));
	if (len != sizeof(send_cmd)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Send Cmd fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n", send_cmd[0], send_cmd[1], send_cmd[2], send_cmd[3], send_cmd[4], send_cmd[5]);
	
	
	return 0;
}

/*[Arima_5816][bozhi_lin] fix converity bug 20151123 begin*/
#if 0
static int HID_FW_Update(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0; /* rewriteCnt = 0; rewriteCnt for PAGE_REWRITE */
	int j = 0; // i=0;
	int write_times = 142;
//	int write_bytes = 12;
	uint8_t data;
//	int restartCnt = 0; // For IAP_RESTART
	int byte_count;
	const u8 *szBuff = NULL;
	u8 write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x21, 0x00, 0x00, 0x28};
	u8 cmd_iap_write[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x22};
	int curIndex = 0;
	int offset = 0;
	// 0x54, 0x00, 0x12, 0x34
	int rc, fw_size;
	
	/* Star Request Firmware */
	const u8 *fw_data;
	const struct firmware *p_fw_entry;
	
	touch_debug(DEBUG_INFO, "Request_firmware name = %s\n", private_ts->image_name);
	rc = request_firmware(&p_fw_entry, private_ts->image_name, &client->dev);
	if (rc != 0) {
		touch_debug(DEBUG_ERROR,"rc=%d, Request_firmware fail\n", rc);
		return -1;
	} else
	PageNum=473;
	touch_debug(DEBUG_INFO,"Firmware Size=%zu, PageNum=%d\n", p_fw_entry->size,PageNum);

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;                                                
	/* End Request Firmware */                             
	
	touch_debug(DEBUG_INFO, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);

	data=I2C_DATA[0];//Master
	touch_debug(DEBUG_INFO, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	elan_ktf_ts_hw_reset();
	mdelay(200); 
	
	res = CheckISPstatus(private_ts->client);
	mdelay(20);
	if (res == 0x88) { /* 0x88 recovery mode  */
        	elan_ktf_ts_hw_reset();
		mdelay(200); 
		printk("[elan hid iap] Recovery mode\n");
		res = RecoveryISP(private_ts->client);
	}
	else {
		printk("[elan hid iap] Normal mode\n");
		res = HID_EnterISPMode(private_ts->client);   //enter HID ISP mode
	}

	mdelay(50);

	// Start HID IAP
	for( iPage = 1; iPage <= 473; iPage +=30 )
	{
		offset=0;
		if (iPage == 451 ) {
			write_times = 109;
			//write_bytes = 18;
		}
		else {	
			write_times = 142;
			//write_bytes = 12;
		}
		mdelay(5);
		for(byte_count=1; byte_count <= write_times; byte_count++)
		{
			mdelay(5);
			if(byte_count != write_times)
			{
				szBuff = fw_data + curIndex;
				write_buf[8] = 28;
				write_buf[7] = offset & 0x00ff;
				write_buf[6] = offset >> 8;
				offset +=28;
				curIndex =  curIndex + 28;
				for (j=0; j<28; j++)
					write_buf[j+9]=szBuff[j];
/*
if (iPage == 451) {
printk("[hid_iap] P=%d i=%d: ", iPage,byte_count);
for (j=0; j<37; j++)
	printk("%x ", write_buf[j]);
printk("\n");
} */
				res = WritePage(write_buf, 37);
			}
			else
			{

				szBuff = fw_data + curIndex;
				write_buf[8] = 12;
				write_buf[7] = offset & 0x00ff;
				write_buf[6] = offset >> 8;
				curIndex =  curIndex + 12;
				for (j=0; j<12; j++)
					write_buf[j+9]=szBuff[j];

printk("[elan hid iap packet] iPage=%d i=%d times=%d data ", iPage,byte_count, write_times);
for (j=0; j<37; j++)
	printk("%x ", write_buf[j]);
printk("\n");

				res = WritePage(write_buf, 37);
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)


	   mdelay(10);
/*
printk("[elan iap write] cmd ");
for (j=0; j<37; j++)
	printk("%x ", cmd_iap_write[j]);
printk("\n");
*/
	   res = WritePage(cmd_iap_write, 37);
	   mdelay(200);
	   res = GetAckData(private_ts->client);
	   mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)



	res = SendEndCmd(private_ts->client);

	mdelay(200);
	elan_ktf_ts_hw_reset();
	mdelay(200);
	res = elan_ktf_ts_calibrate(private_ts->client); 
	mdelay(100);
	
	touch_debug(DEBUG_INFO,"[ELAN] Update Firmware successfully!\n");

	return res;
}
#endif
/*[Arima_5816][bozhi_lin] 20151123 end*/

#if defined(CHECKSUM_ENABLE)
int PreTest(struct i2c_client *client)
{
	uint16_t sumdata=0;
	uint16_t byte1, byte2, combined;
	uint16_t precs=0;
	int PageNum=0;
	const u8 *fw_data;
	int rc, fw_size;
	int j, i;
	int pt=0;
	const struct firmware *p_fw_entry;
	
	touch_debug(DEBUG_INFO, "request_firmware name = %s\n", private_ts->image_name);

	rc = request_firmware(&p_fw_entry, private_ts->image_name, &client->dev);
	if (rc != 0) {
		touch_debug(DEBUG_ERROR,"rc=%d, request_firmware fail\n", rc);
		/*[Arima_5816][bozhi_lin] fix coverity bug 20151201 begin*/
		return -1;
		/*[Arima_5816][bozhi_lin] 20151201 end*/
	} else {
		touch_debug(DEBUG_INFO,"Firmware Size=%zu\n", p_fw_entry->size);
	}

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;      
	
	PageNum=fw_size/132;
	
	touch_debug(DEBUG_INFO,"Firmware Size=%d, PageNum=%d\n", fw_size, PageNum);
	touch_debug(DEBUG_INFO,"0x%x 0x%x\n", fw_data[0], fw_data[1]); 
		
	for (i= 1; i<PageNum-1; i++)  
	{
		for (j = 0; j < 132 ;j=j+2) //132 BYTE
		{  
			pt = i*132 + j;
			if (j > 1 && j < 130)
			{
				byte1= fw_data[pt];
				byte2= fw_data[pt+1];
				combined = (byte2 << 8 | byte1);
				//printk("%x ", combined);
			  
				if (i == 243 && j == 22){
					precs = combined;
					//printk("[%x] ", precs);
				}else{
					sumdata = sumdata + combined;
				}
			} 
		}
		//printk("\n");   
		//printk("i=%d, precs:0x%x, sum:0x%x\n", i, precs, sumdata);
	}  
	
	touch_debug(DEBUG_INFO,"sumdata=0x%x, precs=0x%x\n", sumdata, precs);
	if (sumdata != precs){
		touch_debug(DEBUG_INFO,"PreTest check Fail\n");
		goto PT_FAIL;
	}
	touch_debug(DEBUG_INFO,"PreTest check OK\n");

	/*[Arima_5816][bozhi_lin] fix coverity bug 20151201 begin*/
	release_firmware(p_fw_entry);	
	/*[Arima_5816][bozhi_lin] 20151201 end*/
	return 0;	
	
PT_FAIL:
	/*[Arima_5816][bozhi_lin] fix coverity bug 20151201 begin*/
	release_firmware(p_fw_entry);
	/*[Arima_5816][bozhi_lin] 20151201 end*/
	return -1;	
}
#endif

/*[Arima_5816][bozhi_lin] touch modify upgrade firmware in kernel 20160105 begin*/
/*[Arima_5816][bozhi_lin] re-write touch upgrade firmware mechanism 20151124 begin*/
#if defined(FILE_UPGRADE)
static int FW_Update_Check(void *x) {
	const struct firmware *p_fw_entry = NULL;
	const u8 *fw_data;
	int rc, fw_size;
	int New_FW_ID;	
	int New_FW_VER;	

	touch_debug(DEBUG_INFO, "request_firmware name = %s\n",private_ts->image_name);
	rc = request_firmware(&p_fw_entry, private_ts->image_name, &private_ts->client->dev);
	if (rc != 0) {
		touch_debug(DEBUG_INFO,"rc=%d, request_firmware fail\n", rc);
		New_FW_ID = 0x0;
		New_FW_VER = 0x0;
	} else {
		fw_data = p_fw_entry->data;
		fw_size = p_fw_entry->size;
		PageNum = fw_size/132;

		touch_debug(DEBUG_INFO,"Firmware Size=%zu, PageNum=%d\n", p_fw_entry->size, PageNum);

		printk("[ELAN]  [7d64]=0x%02x,  [7d65]=0x%02x, [7d66]=0x%02x, [7d67]=0x%02x\n",  fw_data[0x7d64],fw_data[0x7d65],fw_data[0x7d66],fw_data[0x7d67]);
		New_FW_ID = fw_data[0x7d67]<<8  | fw_data[0x7d66];
		New_FW_VER = fw_data[0x7d65]<<8  | fw_data[0x7d64];
	}
	printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);
	printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);

	if (RECOVERY == 0x80){
		FW_Update(0);
	} else {
/*[Arima_5833][bozhi_lin] upgrade elan truly touch firmware to 0x5500 20160328 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
		if (New_FW_ID == 0x3046) {
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
		if (New_FW_ID == 0x3050) {
#else
		if (1) {
#endif
/*[Arima_5833][bozhi_lin] 20160328 end*/
			if (New_FW_ID == FW_ID){
				if (New_FW_VER > FW_VERSION){
					FW_Update(0);
				}
			} else {
				printk("FW_ID is different, Force upgrade!");
				FW_Update(0);
			}
		} else {
			printk("No support build-in FW_ID = 0x%x", New_FW_ID);
		}
	}

	if (fwupd_flag == 1) {  
		printk("[ELAN]fwupd_flag = 1: execute Update FW Now, return");
	} else {
#if defined(CHECKSUM_ENABLE)
		rc = getCheckSUM(private_ts->client);
		if(rc == -1){
			printk("[ELAN]probe: getCheckSUM error, goto IAP.\n");
			FW_Update(0);
		}
#endif
	}

	if (p_fw_entry)
		release_firmware(p_fw_entry);

	return 0;
}
#endif

static int FW_Update(void *x)
//static int FW_Update(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0, iapmodecnt=0, checksumcnt=0; // For IAP_RESTART
	int byte_count;
	const u8 *szBuff = NULL;
	int curIndex = 0;
	int rc, fw_size;
	
	/* Star Request Firmware */
	const u8 *fw_data;
	const struct firmware *p_fw_entry;
	
	touch_debug(DEBUG_INFO, "request_firmware name = %s\n", private_ts->image_name);
	rc = request_firmware(&p_fw_entry, private_ts->image_name, &private_ts->client->dev);
	if (rc != 0) {
		touch_debug(DEBUG_INFO,"rc=%d, request_firmware fail\n", rc);
		/*[Arima_5816][bozhi_lin] fix coverity bug 20151201 begin*/
		return -1;
		/*[Arima_5816][bozhi_lin] 20151201 end*/
	}

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;                                                
	PageNum = fw_size/132;
    touch_debug(DEBUG_INFO,"Firmware Size=%d, PageNum=%d\n", fw_size, PageNum);	
	
#if defined(CHECKSUM_ENABLE)
	res = PreTest(private_ts->client);
	if (res == -1){
		return -1;
	}
#endif
	
	/* End Request Firmware */                             
	disable_irq(private_ts->client->irq);
	mutex_lock(&ktf_mutex);
IAP_RESTART:

	data=I2C_DATA[0];//Master
	fwupd_flag = 1;
	curIndex = 0;
	work_lock = 1;
	power_lock = 1;
	touch_debug(DEBUG_INFO, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	#ifdef ESD_CHECK
	live_state = 1;
    #endif
	
	if(RECOVERY != 0x80)
	{
		touch_debug(DEBUG_INFO,"[ELAN] Firmware upgrade normal mode !\n");
	} else{
		touch_debug(DEBUG_INFO,"[ELAN] Firmware upgrade recovery mode !\n");
	}
	elan_ktf_ts_hw_reset(private_ts->client);
	mdelay(20);
		res = EnterISPMode(private_ts->client);   //enter ISP mode
	if (res <0){
		touch_debug(DEBUG_ERROR,"[ELAN] EnterISPMode Fail\n");
		goto IAP_FAIL;
	}
	mdelay(10);

	//1106 -- start
	res = CheckIapMode(); 
	if (res < 0){
		iapmodecnt++;
		if (iapmodecnt < 3){
			goto IAP_RESTART;
		}else{
			touch_debug(DEBUG_ERROR,"[ELAN] Firmware upgrade CheckIAP Fail 3 times\n");
			goto IAP_FAIL; //1106
		}
	}
	//1106--end

	touch_debug(DEBUG_INFO,"[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
		printk("[ELAN] dummy error code = %d\n",res);
		touch_debug(DEBUG_ERROR,"[ELAN] dummy error code = %d\n",res);
		goto IAP_FAIL;
	}
	mdelay(10);
	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ )
	{
		PAGE_REWRITE:

		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{
				szBuff = fw_data + curIndex;
				curIndex =  curIndex + 8;
				res = WritePage(szBuff, 8);
			}
			else
			{

				szBuff = fw_data + curIndex;
				curIndex =  curIndex + 4;
				res = WritePage(szBuff, 4);
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
		if(iPage==PageNum)
		{
			mdelay(600);
		}
		else
		{
			mdelay(50);
		}
		res = GetAckData(private_ts->client);

		if (ACK_OK != res)
		{
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE )
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
					touch_debug(DEBUG_ERROR,"[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					goto IAP_FAIL;
				}
				else
				{
					touch_debug(DEBUG_ERROR,"[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 3)
				{
					touch_debug(DEBUG_ERROR,"[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					goto IAP_FAIL;
				}
				else
				{
					printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					touch_debug(DEBUG_ERROR,"[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		}
		else
		{
			rewriteCnt=0;
			print_progress(iPage,ic_num,i);
		}

	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	mdelay(100);

	res= __hello_packet_handler(private_ts->client);
	if (res == 0x80){
		touch_debug(DEBUG_ERROR, "[ELAN] Update Firmware Failed, Recover = 0x80!\n");
		checksumcnt++;
		if(checksumcnt<3)
			goto IAP_RESTART;
	} else {
		__fw_packet_handler(private_ts->client);
#if defined(CHECKSUM_ENABLE)
		res = getCheckSUM(private_ts->client);
		if(res == -1 && checksumcnt<3){
			checksumcnt++;
			printk("[elan] Update_FW_One: getCheckSUM error, goto IAP.\n");
			goto IAP_RESTART;
		}
#endif
	}
	
	enable_irq(private_ts->client->irq);
	mutex_unlock(&ktf_mutex);
	fwupd_flag = 0;
	power_lock = 0; 
	work_lock = 0;
	if (p_fw_entry)
		release_firmware(p_fw_entry);
	return 0;
	
IAP_FAIL:	
	enable_irq(private_ts->client->irq);
	mutex_unlock(&ktf_mutex);
	fwupd_flag = 0;
	power_lock = 0; 
	work_lock = 0;  
	/*[Arima_5816][bozhi_lin] fix coverity bug 20151201 begin*/
	release_firmware(p_fw_entry);
	/*[Arima_5816][bozhi_lin] 20151201 end*/
	return -1;
}

static ssize_t set_debug_mesg(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int level[SYSFS_MAX_LEN];
	sscanf(buf,"%x",&level[0]);
	debug=level[0];
	//touch_debug(DEBUG_INFO, "debug level %d, size %d\n", debug,count);
	return count;
}

static ssize_t show_debug_mesg(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Debug level %d \n", debug);
}

static ssize_t show_gpio_int(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->irq_gpio));
}

static ssize_t show_reset(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	elan_ktf_ts_hw_reset(private_ts->client);

	return sprintf(buf, "Reset Touch Screen Controller \n");
}

static ssize_t show_enable_irq(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//	struct i2c_client *client = to_i2c_client(dev);
	//	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Enable IRQ \n");
}

static ssize_t show_disable_irq(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//	struct i2c_client *client = to_i2c_client(dev);
	//	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
	return sprintf(buf, "Disable IRQ \n");
}

static ssize_t show_calibrate(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = elan_ktf_ts_calibrate(client);
	return sprintf(buf, "%s\n",
	(ret == 0) ? " [Elan] Calibrate Finish" : "[Elan] Calibrate Fail");
}

/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
#if defined(FILE_UPGRADE)

#else
static ssize_t show_fw_update_in_driver(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;

	ret = Update_FW_in_Driver(0);
	
	return sprintf(buf, "[Elan] Update Firmware in driver used fw_data.i\n");
}
#endif
/*[Arima_5816][bozhi_lin] 20160114 end*/

/*[Arima_5816][bozhi_lin] fix converity bug 20151123 begin*/
#if 0
static ssize_t show_hid_fw_update(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
	ret = HID_FW_Update(client, 0);
	
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
	
	return sprintf(buf, "HID Update Firmware\n");
}
#endif
/*[Arima_5816][bozhi_lin] 20151123 end*/

static ssize_t show_fw_update(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
//	struct i2c_client *client = to_i2c_client(dev);

	ret = FW_Update(0);
	
	return sprintf(buf, "Update Firmware\n");
}

#ifdef ELAN_2WIREICE
static ssize_t show_2wire(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
	ret = TWO_WIRE_ICE(client);
	
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
	
	return sprintf(buf, "Update Firmware by 2wire JTAG\n");
}
#endif

static ssize_t show_fw_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "0x%x\n", ts->fw_ver);
}

static ssize_t show_fw_id_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "0x%x\n", ts->fw_id);
}

static ssize_t show_bc_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "Bootcode:%d\n"
	"IAP:%d\n", ts->fw_ver, ts->fw_id);
}

static ssize_t show_drv_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "Elan driver version 0x0006");
}

static ssize_t show_iap_mode(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n", 
	(ts->fw_ver == 0) ? "Recovery" : "Normal");
}


static DEVICE_ATTR(debug_mesg, S_IRUGO | S_IWUGO, show_debug_mesg, set_debug_mesg);
static DEVICE_ATTR(gpio_int, S_IRUGO, show_gpio_int, NULL);
static DEVICE_ATTR(reset, S_IRUGO, show_reset, NULL);
static DEVICE_ATTR(enable_irq, S_IRUGO, show_enable_irq, NULL);
static DEVICE_ATTR(disable_irq, S_IRUGO, show_disable_irq, NULL);
static DEVICE_ATTR(calibrate, S_IRUGO, show_calibrate, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, show_fw_version_value, NULL);
static DEVICE_ATTR(fw_id, S_IRUGO, show_fw_id_value, NULL);
static DEVICE_ATTR(bc_version, S_IRUGO, show_bc_version_value, NULL);
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);
/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
#if defined(FILE_UPGRADE)

#else
static DEVICE_ATTR(fw_update_in_driver, S_IRUGO, show_fw_update_in_driver, NULL);
#endif
/*[Arima_5816][bozhi_lin] 20160114 end*/
static DEVICE_ATTR(fw_update, S_IRUGO, show_fw_update, NULL);
/*[Arima_5816][bozhi_lin] fix converity bug 20151123 begin*/
#if 0
static DEVICE_ATTR(hid_fw_update, S_IRUGO, show_hid_fw_update, NULL);
#endif
/*[Arima_5816][bozhi_lin] 20151123 end*/
#ifdef ELAN_2WIREICE
static DEVICE_ATTR(2wire, S_IRUGO, show_2wire, NULL);
#endif
static DEVICE_ATTR(iap_mode, S_IRUGO, show_iap_mode, NULL);

static struct attribute *elan_attributes[] = {
	&dev_attr_debug_mesg.attr,
	&dev_attr_gpio_int.attr,
	&dev_attr_reset.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_id.attr,
	&dev_attr_bc_version.attr,
	&dev_attr_drv_version.attr,
/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
#if defined(FILE_UPGRADE)

#else
	&dev_attr_fw_update_in_driver.attr,
#endif
/*[Arima_5816][bozhi_lin] 20160114 end*/
	&dev_attr_fw_update.attr,
/*[Arima_5816][bozhi_lin] fix converity bug 20151123 begin*/
#if 0
	&dev_attr_hid_fw_update.attr,
#endif
/*[Arima_5816][bozhi_lin] 20151123 end*/
	#ifdef ELAN_2WIREICE
	&dev_attr_2wire.attr,
	#endif
	&dev_attr_iap_mode.attr,
	NULL
};

static struct attribute_group elan_attribute_group = {
	.name = DEVICE_NAME,
	.attrs = elan_attributes,
};


// Start sysfs
static ssize_t elan_ktf_gpio_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->irq_gpio);
	touch_debug(DEBUG_MESSAGES, "GPIO_TP_INT_N=%d\n", ts->irq_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf_gpio_show, NULL);

static ssize_t elan_ktf_vendor_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;
	
/*[Arima_5830][bozhi_lin] correct touch vendor information 20160615 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
	sprintf(buf, "%s_0x%4.4x", "Truly", ts->fw_ver);
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
/*[Arima_5833][bozhi_lin] add touch 2nd source detect 20160818 begin*/
	if (gpio_get_value(ts->id_gpio)) {
		sprintf(buf, "%s_0x%4.4x", "Holitech2", ts->fw_ver);
	} else {
		sprintf(buf, "%s_0x%4.4x", "Holitech", ts->fw_ver);
	}
/*[Arima_5833][bozhi_lin] 20160818 end*/
#else
	if (gpio_get_value(ts->id_gpio)) {
		sprintf(buf, "%s_0x%4.4x", "Truly", ts->fw_ver);
	} else {
		sprintf(buf, "%s_0x%4.4x", "Truly", ts->fw_ver);
	}
#endif
/*[Arima_5830][bozhi_lin] 20160615 end*/

	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		touch_debug(DEBUG_ERROR,"[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}	

// end sysfs



static int elan_ktf_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t w_size,  size_t r_size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
	return -EINVAL;

	if ((i2c_master_send(client, cmd, w_size)) != w_size) {
		dev_err(&client->dev,
		"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf_ts_poll(client);
	if (rc < 0)
	printk("%s: poll is hight\n",__func__);

	if(r_size <= 0) r_size=w_size;
	
	if (i2c_master_recv(client, buf, r_size) != r_size)	return -EINVAL;

	return 0;
}

#if defined(CHECKSUM_ENABLE)
static int getCheckSUM(struct i2c_client *client){
	uint8_t sendCS[]={0x53, 0x70, 0x00, 0x01};
	uint8_t recvCS[4]={0};
	uint8_t sendEP[]={0x53, 0xF4, 0x00, 0x01};
	uint8_t recvEP[4]={0};
	int res;
	char CheckSum[2]={0};

	disable_irq(client->irq);

	res = elan_ktf_ts_get_data(client, sendCS, recvCS, 4, 4);
	if (res < 0){
		printk("%s elan_ktf_ts_get_data sendCS fail\n", __func__);
	}
	printk("[ELAN]recv CS! %x %x %x %x\n", recvCS[0] , recvCS[1] , recvCS[2] , recvCS[3] ); 
 
	if(recvCS[0]==0 && recvCS[1]==0 && recvCS[2]==0 && recvCS[3]==0){
		printk("get_data recvCS fail, return.\n");
		return -1;
	}
 
	res = elan_ktf_ts_get_data(client, sendEP, recvEP, 4, 4);
	if (res < 0){
		printk("%s elan_ktf_ts_get_data sendEP fail\n", __func__);
	}
	printk("[ELAN]recv EP! %x %x %x %x\n", recvEP [0] , recvEP [1] , recvEP [2] , recvEP [3] ); 
	
	if(recvEP[0]==0 && recvEP[1]==0 && recvEP[2]==0 && recvEP[3]==0){
		printk("get_data recvEP fail, return.\n");
		return -1;
	}
  
	enable_irq(client->irq);
  
	CheckSum[0]= (( recvEP[1] & 0x0F) << 4) |((recvEP[2] & 0xF0) >> 4);
	CheckSum[1]= (( recvEP[2] & 0x0F) << 4) |((recvEP[3] & 0xF0) >> 4);

	printk("[ELAN]compare checksum! EP %x %x, CheckSum %x %x\n", CheckSum[0], CheckSum[1], recvCS[2] , recvCS[3]);

	if (!(recvCS[2] == CheckSum[0] && recvCS[3] == CheckSum[1]))
	{
		printk("[ELAN]compare checksum fail\n"); 	
		return -1;
	}
    
	return 0;
}
#endif

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf_ts_poll(client);
	if (rc < 0) {
		printk( "[elan] %s: Int poll failed!\n", __func__);
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	if (rc != 8) {
		printk( "[elan] %s: i2c_master_recv error\n", __func__);
		return -1;
	}
	printk("[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
		RECOVERY=0x80;
		return RECOVERY; 
	}
	
	/*Some Elan init don't need Re-Calibration */
	//mdelay(300);
	rc = elan_ktf_ts_poll(client);
	if (rc < 0) {
		printk( "[elan] %s: Int poll failed!\n", __func__);
	}
	rc = i2c_master_recv(client, buf_recv, 8);
	if (rc != 8) {
		printk( "[elan] %s: i2c_master_recv error\n", __func__);
		return -1;
	}
	printk("[elan] %s: Try Re-Calibration packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	
	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
#if 1
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
#else	
	uint8_t info_buff[] = { 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00 }; /*Get IC info*/
	uint8_t info_buff_resp[17] = { 0 };
#endif
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};
	// Firmware version
	rc = elan_ktf_ts_get_data(client, cmd, buf_recv, 4,4);
	if (rc < 0)
	{
		printk("Get Firmware version error\n");
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;
	// Firmware ID
	rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4,4);
	if (rc < 0)
	{
		printk("Get Firmware ID error\n");
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;
	// Bootcode version
	rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv, 4,4);
	if (rc < 0)
	{
		printk("Get Bootcode version error\n");
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;

#if 1
// X Resolution
	rc = elan_ktf_ts_get_data(client, cmd_x, buf_recv, 4, 4);
	if (rc < 0)
	{
		printk("Get X Resolution error\n");
	}
	ts->x_resolution = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	X_RESOLUTION = ts->x_resolution;
// Y Resolution	
	rc = elan_ktf_ts_get_data(client, cmd_y, buf_recv, 4, 4);
	if (rc < 0)
	{
		printk("Get Y Resolution error\n");
	}
	ts->y_resolution = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	Y_RESOLUTION = ts->y_resolution;
#else
	/*Get XY info*/
	rc = elan_ktf_ts_get_data(client, info_buff, info_buff_resp, sizeof(info_buff), sizeof(info_buff_resp));
	if (rc < 0)
	{
		printk("Get XY info error\n");
	}
	ts->x_resolution = (info_buff_resp[2] + info_buff_resp[6]
	+ info_buff_resp[10] + info_buff_resp[14] - 1)*64;
//	X_RESOLUTION = ts->x_resolution;

	ts->y_resolution = (info_buff_resp[3] + info_buff_resp[7]
	+ info_buff_resp[11] + info_buff_resp[15] - 1)*64;
//	Y_RESOLUTION = ts->y_resolution;
#endif
	
	printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n",
	__func__, ts->fw_ver);
	printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n",
	__func__, ts->fw_id);
	printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n",
	__func__, ts->bc_ver);
	printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
	__func__, X_RESOLUTION, Y_RESOLUTION);
	
	return 0;
}

static inline int elan_ktf_pen_parse_xy(uint8_t *data,
uint16_t *x, uint16_t *y, uint16_t *p)
{
	*x = *y = *p = 0;

	*x = data[3];
	*x <<= 8;
	*x |= data[2];

	*y = data[5];
	*y <<= 8;
	*y |= data[4];

	*p = data[7];
	*p <<= 8;
	*p |= data[6];

	return 0;
}

static inline int elan_ktf_ts_parse_xy(uint8_t *data,
uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf_ts_setup(struct i2c_client *client)
{
	int rc;
	
	rc = __hello_packet_handler(client);

	if (rc != 0x80 && rc != -1){
		rc = __fw_packet_handler(client);
		if (rc < 0)
		printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
		dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);
		//Check for FW_VERSION, if 0x0000 means FW update fail!
		if ( FW_VERSION == 0x00)
		{
			rc = 0x80;
			printk("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION);
		}
	}
	return rc;
}

static int elan_ktf_ts_calibrate(struct i2c_client *client){

#ifdef ELAN_HID_I2C
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0xc0, 0xe1, 0x5a};
	uint8_t cal_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0x29, 0x00, 0x01};

	dev_info(&client->dev, "[elan] %s: Flash Key cmd\n", __func__);
	if ((i2c_master_send(client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
	dev_info(&client->dev,
	"[elan] %s: Calibration cmd: %02x, %02x, %02x, %02x\n", __func__,
	cal_cmd[7], cal_cmd[8], cal_cmd[9], cal_cmd[10]);
	if ((i2c_master_send(client, cal_cmd, sizeof(cal_cmd))) != sizeof(cal_cmd)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

#else	
/*[Arima_5816][bozhi_lin] touch send re-calibraion command in gesture mode 20160108 begin*/
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x01, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
	dev_dbg(&client->dev,
	"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	cmd[0], cmd[1], cmd[2], cmd[3]);
/*[Arima_5816][bozhi_lin] 20160108 end*/

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
#endif
	return 0;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status=0;

#if 0
static int elan_ktf_ts_hw_reset(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	touch_debug(DEBUG_INFO, "[ELAN] Start HW reset!\n");
	gpio_direction_output(ts->rst_gpio, 0);
	usleep_range(1000,1500);
	gpio_direction_output(ts->rst_gpio, 1);
	msleep(5);
	return 0;
}

static int elan_ktf_ts_set_power_source(struct i2c_client *client, u8 state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
	int length = 0;

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
	/*0x52 0x40 0x00 0x01  =>    Battery Mode
	0x52 0x41 0x00 0x01  =>   AC Adapter Mode
	0x52 0x42 0x00 0x01 =>    USB Mode */
	cmd[1] |= state & 0x0F;

	dev_dbg(&client->dev,
	"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	cmd[0], cmd[1], cmd[2], cmd[3]);
	
	down(&pSem);
	length = i2c_master_send(client, cmd, sizeof(cmd));
	up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}



static void update_power_source(){
	unsigned power_source = now_usb_cable_status;
	if(private_ts == NULL || work_lock) return;

	if(private_ts->abs_x_max == ELAN_X_MAX) //TF 700T device
	return; // do nothing for TF700T;
	
	touch_debug(DEBUG_INFO, "Update power source to %d\n", power_source);
	switch(power_source){
	case USB_NO_Cable:
		elan_ktf_ts_set_power_source(private_ts->client, 0);
		break;
	case USB_Cable:
		elan_ktf_ts_set_power_source(private_ts->client, 1);
		break;
	case USB_AC_Adapter:
		elan_ktf_ts_set_power_source(private_ts->client, 2);
	}
}
#endif

void touch_callback(unsigned cable_status){ 
	now_usb_cable_status = cable_status;
	//update_power_source();
}
#endif

static int elan_ktf_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{

	int rc;
	if (buf == NULL)
	return -EINVAL;

	memset(buf, 0, bytes_to_recv);

	/* The ELAN_PROTOCOL support normanl packet format */	
#ifdef ELAN_PROTOCOL		
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	//printk("[elan] Elan protocol rc = %d \n", rc);
	if (rc != bytes_to_recv) {
		dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
		return -1;
	}

#else 
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	if (rc != 8)
	{
		printk("[elan] Read the first package error.\n");
		mdelay(30);
		return -1;
	}
	printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	mdelay(1);
	
	if (buf[0] == 0x6D){    //for five finger
		rc = i2c_master_recv(client, buf+ 8, 8);	
		if (rc != 8)
		{
			printk("[elan] Read the second package error.\n");
			mdelay(30);
			return -1;
		}		      
		printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
		rc = i2c_master_recv(client, buf+ 16, 2);
		if (rc != 2)
		{
			printk("[elan] Read the third package error.\n");
			mdelay(30);
			return -1;
		}		      
		mdelay(1);
		printk("[elan_debug] %x %x \n", buf[16], buf[17]);
	}
#endif
	//printk("[elan_debug] end ts_work\n");
	return rc;
}

#ifdef PROTOCOL_B
/* Protocol B  */
static int mTouchStatus[FINGER_NUM] = {0};  /* finger_num=10 */
void force_release_pos(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int i;
	for (i=0; i < FINGER_NUM; i++) {
		if (mTouchStatus[i] == 0) continue;
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}

	input_sync(ts->input_dev);
}

static inline int elan_ktf_hid_parse_xy(uint8_t *data,
uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[6]);
	*x <<= 8;
	*x |= data[5];

	*y = (data[10]);
	*y <<= 8;
	*y |= data[9];

	return 0;
}

static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x =0, y =0,touch_size, pressure_size;
	uint16_t fbits=0;
	uint8_t i, num;
	uint16_t active = 0; 
	uint8_t idx, btn_idx;
	int finger_num;
	int finger_id;
	static uint8_t size_index[10] = {35, 35, 36, 36, 37, 37, 38, 38, 39, 39};
	int pen_hover = 0;
	int pen_down = 0;
	uint16_t p = 0;

	/* for 10 fingers */
	if (buf[0] == TEN_FINGERS_PKT){
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx=3;
		btn_idx=33;
	}
	/* for 5 fingers  */
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
	}else{
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
		idx=1;
		btn_idx=7;
	}

	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:

		for(i = 0; i < finger_num; i++){
			active = fbits & 0x1;
			if(active || mTouchStatus[i]){
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
				if(active){
					elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
					//y=Y_RESOLUTION -y;
					touch_size = ((i & 0x01) ? buf[size_index[i]] : (buf[size_index[i]] >> 4)) & 0x0F;
					pressure_size = touch_size << 4; // shift left touch size value to 4 bits for max pressure value 255   	      	
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
					input_report_abs(idev, ABS_MT_PRESSURE, 100);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					if(unlikely(gPrint_point)) 
					touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size);
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		if (num == 0){
			//printk("[ELAN] ALL Finger Up\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case PEN_PKT:
		
		pen_hover = buf[1] & 0x1;
		pen_down = buf[1] & 0x03;
		input_mt_slot(ts->input_dev, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, pen_hover);
		if (pen_hover){
			elan_ktf_pen_parse_xy(&buf[0], &x, &y, &p);
			//y=Y_RESOLUTION -y;
			if (pen_down == 0x01) {  /* report hover function  */
				input_report_abs(idev, ABS_MT_PRESSURE, 0);
				input_report_abs(idev, ABS_MT_DISTANCE, 15);
				touch_debug(DEBUG_INFO, "[elan pen] Hover DISTANCE=15 \n");
			}
			else {
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 20);
				input_report_abs(idev, ABS_MT_PRESSURE, p);
				touch_debug(DEBUG_INFO, "[elan pen] PEN PRESSURE=%d \n", p);
			}
			input_report_abs(idev, ABS_MT_POSITION_X, x);
			input_report_abs(idev, ABS_MT_POSITION_Y, y);
		}
		if(unlikely(gPrint_point)) {
			touch_debug(DEBUG_INFO, "[elan pen] %x %x %x %x %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
			touch_debug(DEBUG_INFO, "[elan] x=%d y=%d p=%d \n", x, y, p);
		}
		if (pen_down == 0){
			//printk("[ELAN] ALL Finger Up\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case ELAN_HID_PKT:
		finger_num = buf[62];
		if (finger_num > 5)	finger_num = 5;   /* support 5 fingers    */ 
		idx=3;
		num = 5;
		for(i = 0; i < finger_num; i++){						
			if ((buf[idx]&0x03) == 0x00)	active = 0;   /* 0x03: finger down, 0x00 finger up  */
			else	active = 1;

			if ((buf[idx] & 0x03) == 0) num --;
			finger_id = (buf[idx] & 0xfc) >> 2;
			input_mt_slot(ts->input_dev, finger_id);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
			if(active){
				elan_ktf_hid_parse_xy(&buf[idx], &x, &y);
				//y = Y_RESOLUTION - y;
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(idev, ABS_MT_PRESSURE, 100);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
				touch_debug(DEBUG_INFO, "[elan hid] i=%d finger_id=%d x=%d y=%d Finger NO.=%d \n", i, finger_id, x, y, finger_num);
			}
			mTouchStatus[i] = active;
			idx += 11;
		}
		if (num == 0){  
			printk("[ELAN] Release ALL Finger\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			force_release_pos(client);
		}
		input_sync(idev);
		break ;
	case IamLive_PKT://0512
		touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
		break;
		
	case SmartWake_PKT: 
		{
			 printk("[elan] Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			 switch (buf[1]){
			 	
			 case 0x00: //e
			      break;
			      			 	
			 case 0x01: //Z
			      break;
			      
			 case 0x03: //v
			      break;
			
			 case 0x05: //w
			      break;
			      
			 case 0x07: //c
			      break;		
			      
			 case 0x08: //s
			      break;			            	
			 }
			 
    }
    break;
	default:
		dev_err(&client->dev,
		"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	} // end switch

	return;
}

#endif

#ifdef PROTOCOL_A
/* Protocol A  */
static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx, btn_idx;
	int finger_num;
	
	/* for 10 fingers	*/
	if (buf[0] == TEN_FINGERS_PKT){
		finger_num = 10;
		num = buf[2] & 0x0f; 
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
		idx=3;
		btn_idx=33;
	}
	/* for 5 fingers	*/
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
		finger_num = 5;
		num = buf[1] & 0x07; 
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
	}else{
		/* for 2 fingers */    
		finger_num = 2;
		num = buf[7] & 0x03;		// for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1;	// for elan old 5A protocol the finger ID is 0x06
		idx=1;
		btn_idx=7;
	}
	
	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:	
	case TEN_FINGERS_PKT:
		//input_report_key(idev, BTN_TOUCH, 1);
		if (num == 0) {
			input_report_key(idev, BTN_TOUCH, 0);
#ifdef ELAN_BUTTON
			//printk("[elan_debug] buf[btn_idx]=0x%x, button_state=0x%x\n", buf[btn_idx], button_state);
/*[Arima_5816][bozhi_lin] upgrade elan truly touch firmware to 0x5501 and enable ESD & checksum 20160307 begin*/
/*[Arima_5830][bozhi_lin] fix touch back and recent are reversed 20160202 begin*/
			if (buf[btn_idx] == 0x21) 
/*[Arima_5830][bozhi_lin] 20160202 end*/
/*[Arima_5816][bozhi_lin] 20160307 end*/
			{
				KEY_BACK_STATE = true;//1117
				input_report_key(idev, KEY_BACK, 1);
			} 
			else if (buf[btn_idx] == 0x41)
			{
				KEY_HOME_STATE = true; //1117
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
/*[Arima_5816][bozhi_lin] fix touch home key not work 20151007 begin*/
#if defined(ARIMA_KEY)
				input_report_key(idev, KEY_HOMEPAGE, 1);
#else						
				input_report_key(idev, KEY_HOME, 1);
#endif
/*[Arima_5816][bozhi_lin] 20151007 end*/
/*[Arima_5816][bozhi_lin] 20160112 end*/
			} 
/*[Arima_5816][bozhi_lin] upgrade elan truly touch firmware to 0x5501 and enable ESD & checksum 20160307 begin*/
/*[Arima_5830][bozhi_lin] fix touch back and recent are reversed 20160202 begin*/
			else if (buf[btn_idx] == 0x81)
/*[Arima_5830][bozhi_lin] 20160202 end*/
/*[Arima_5816][bozhi_lin] 20160307 end*/
			{
				KEY_MENU_STATE = true; //1117
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
/*[Arima_5816][bozhi_lin] enable elan touch virtual key 20150925 begin*/
#if defined(ARIMA_KEY)
				input_report_key(idev, KEY_APPSELECT, 1); //recent key
#else
				input_report_key(idev, KEY_MENU, 1);
#endif
/*[Arima_5816][bozhi_lin] 20150925 end*/
/*[Arima_5816][bozhi_lin] 20160112 end*/
			} else {  //1118 -- start
					if (KEY_BACK_STATE || KEY_HOME_STATE || KEY_MENU_STATE) {
						if (KEY_BACK_STATE) {
							input_report_key(idev, KEY_BACK, 0);
							KEY_BACK_STATE = false;
						}
						if (KEY_HOME_STATE) {
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
/*[Arima_5816][bozhi_lin] fix touch home key not work 20151007 begin*/
#if defined(ARIMA_KEY)
							input_report_key(idev, KEY_HOMEPAGE, 0);
#else						
							input_report_key(idev, KEY_HOME, 0);
#endif
/*[Arima_5816][bozhi_lin] 20151007 end*/
/*[Arima_5816][bozhi_lin] 20160112 end*/
							KEY_HOME_STATE = false;
						}
						if (KEY_MENU_STATE) {
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
/*[Arima_5816][bozhi_lin] enable elan touch virtual key 20150925 begin*/
#if defined(ARIMA_KEY)						
							input_report_key(idev, KEY_APPSELECT, 0); //recent key
#else
							input_report_key(idev, KEY_MENU, 0);
#endif
/*[Arima_5816][bozhi_lin] 20150925 end*/
/*[Arima_5816][bozhi_lin] 20160112 end*/
							KEY_MENU_STATE = false;
						}
					} else {
						dev_dbg(&client->dev, "no press\n");
						input_mt_sync(idev);

					}
			}
			//1118-start
			
#endif	
		} else {			
			dev_dbg(&client->dev, "[elan] %d fingers\n", num);      
			input_report_key(idev, BTN_TOUCH, 1);			
			for (i = 0; i < finger_num; i++) {	
				if ((fbits & 0x01)) {
					elan_ktf_ts_parse_xy(&buf[idx], &x, &y);  		 
					/*[Arima_5830][bozhi_lin] fix touch right and left are reversed 20160202 begin*/
					//x = X_RESOLUTION-x;
					/*[Arima_5830][bozhi_lin] 20160202 end*/
					//y = Y_RESOLUTION-y;			     
					//printk("[elan_debug] %s, x=%x, y=%x\n",__func__, x , y);
					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
					input_report_abs(idev, ABS_MT_PRESSURE, 80);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					input_mt_sync(idev);
					reported++;
					if(unlikely(gPrint_point)) 
					touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d \n", i, x, y);
				} // end if finger status
				fbits = fbits >> 1;
				idx += 3;

			} // end for
		}
		if (reported)
		input_sync(idev);
		else {
			input_mt_sync(idev);
			input_sync(idev);
		}

		break;
	case IamLive_PKT:
		//touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
		break;
	case CALIB_PKT:
		//touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
		break;
/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
	case SmartWake_PKT:
		{
			 //printk("[elan] Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
	 
			 switch (buf[1]){
			
			 case 0x00: //e
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#if defined(ARIMA_KEY)
			      input_report_key(idev, KEY_GESTURE_E, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_E, 0);
			      input_sync(idev);
#else
			      input_report_key(idev, KEY_E, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_E, 0);
			      input_sync(idev);
#endif
/*[Arima_5816][bozhi_lin] 20160112 end*/
				  printk("[elan] KEY_GESTURE_E, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;
			      			 	
			 case 0x01: //Z
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#if defined(ARIMA_KEY)
			      input_report_key(idev, KEY_GESTURE_Z, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_Z, 0);
			      input_sync(idev);
#else
			      input_report_key(idev, KEY_Z, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_Z, 0);
			      input_sync(idev);
#endif
/*[Arima_5816][bozhi_lin] 20160112 end*/
				  printk("[elan] KEY_GESTURE_Z, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;
			
			 case 0x03: //v
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#if defined(ARIMA_KEY)
			      input_report_key(idev, KEY_GESTURE_V, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_V, 0);
			      input_sync(idev);
#else
			      input_report_key(idev, KEY_V, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_V, 0);
			      input_sync(idev);
#endif
/*[Arima_5816][bozhi_lin] 20160112 end*/
				  printk("[elan] KEY_GESTURE_V, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;
			
			 case 0x05: //w
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#if defined(ARIMA_KEY) 
			      input_report_key(idev, KEY_GESTURE_W, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_W, 0);
			      input_sync(idev);

#else
			      input_report_key(idev, KEY_W, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_W, 0);
			      input_sync(idev);
#endif
/*[Arima_5816][bozhi_lin] 20160112 end*/
				  printk("[elan] KEY_GESTURE_W, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;
			      
			 case 0x07: //c
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#if defined(ARIMA_KEY) 
			      input_report_key(idev, KEY_GESTURE_C, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_C, 0);
			      input_sync(idev);
#else
			      input_report_key(idev, KEY_C, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_C, 0);
			      input_sync(idev);
#endif
/*[Arima_5816][bozhi_lin] 20160112 end*/
				  printk("[elan] KEY_GESTURE_C, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;		
			      
			 case 0x08: //s
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
#if defined(ARIMA_KEY) 
			      input_report_key(idev, KEY_GESTURE_S, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_S, 0);
			      input_sync(idev);
#else
			      input_report_key(idev, KEY_S, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_S, 0);
			      input_sync(idev);
#endif
/*[Arima_5816][bozhi_lin] 20160112 end*/
				  printk("[elan] KEY_GESTURE_S, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;	
				  
			 case 0x0f: //double tap
/*[Arima_5830][bozhi_lin] add touch gesture key code follow arima design 20160413 begin*/
#if defined(ARIMA_DOUBLE_CLICK)
			      input_report_key(idev, KEY_GESTURE_WAKE, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_GESTURE_WAKE, 0);
			      input_sync(idev);
#else
			      input_report_key(idev, KEY_POWER, 1);
			      input_sync(idev);
			      input_report_key(idev, KEY_POWER, 0);
			      input_sync(idev);
#endif
/*[Arima_5830][bozhi_lin] 20160413 end*/
				  printk("[elan] DOUBLE_CLICK, Smart Wakeup %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
			      break;			            	

/*[Arima_5816][bozhi_lin] re-send gesture mode command when receive not support gesture key 20151120 begin*/
			default:
				{
					int   len = 0;
					uint8_t gesture_cmd[] = { CMD_W_PKT, 0x40, 0x01, 0x01 };
					printk("[B]%s(%d): not support gesture key, re-send enter gesture command again\n", __func__, __LINE__);
					len = i2c_master_send(ts->client, gesture_cmd, sizeof( gesture_cmd ));
					if( len != sizeof( gesture_cmd ))
					{
						printk("[ELAN] ERROR: enter gesture mode fail! len=%d\n", len);
					}
					else {
						printk("[ELAN] enter gesture mode successfully! cmd = [%02X, %02X, %02X, %02X]\n", gesture_cmd[0], gesture_cmd[1], gesture_cmd[2], gesture_cmd[3]);
					}
				}
				break;
/*[Arima_5816][bozhi_lin] 20151120 end*/
			 }
			 
    	}
    	break;
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/
	default:
		dev_err(&client->dev,
		"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	} // end switch

	return;
}
#endif

static irqreturn_t elan_ktf_ts_irq_handler(int irq, void *dev_id)
{
	int rc;
	struct elan_ktf_ts_data *ts = dev_id;
#ifdef ELAN_BUFFER_MODE
	uint8_t buf[4+PACKET_SIZE] = { 0 };
	uint8_t buf1[PACKET_SIZE] = { 0 };
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif
/*[Arima_5816][bozhi_lin] fix touch gesture not work in suspend state 20151012 begin*/
#if defined(GESTUREMODE)
	struct device *dev = ts->input_dev->dev.parent;
#endif
/*[Arima_5816][bozhi_lin] 20151012 end*/
	
/*[Arima_5816][bozhi_lin] fix touch gesture not work in suspend state 20151231 begin*/
#if defined(GESTUREMODE)
	if (dev->power.is_suspended) {
		if ((ts->wakeup_dclick) || (ts->wakeup_gesture_type)) {
			printk("[B]%s(%d): dev->power.is_suspended=%d, sleep 1 ms\n", __func__, __LINE__, dev->power.is_suspended);
			msleep(1);
			if (dev->power.is_suspended) {
				touch_debug(DEBUG_TRACE,"[B]%s(%d): dev->power.is_suspended=%d, return\n", __func__, __LINE__, dev->power.is_suspended);
				return IRQ_HANDLED;
			} else {
				touch_debug(DEBUG_TRACE,"[B]%s(%d): dev->power.is_suspended=%d, continue\n", __func__, __LINE__, dev->power.is_suspended);
			}
		}
	}
#endif
/*[Arima_5816][bozhi_lin] 20151231 end*/
	
	if (gpio_get_value(ts->irq_gpio))
	{
		printk("[elan] Detected the jitter on INT pin");
		return IRQ_HANDLED;
	}
	
#ifdef ESD_CHECK
	live_state=1;
#endif	
#ifdef ELAN_BUFFER_MODE
	rc = elan_ktf_ts_recv_data(ts->client, buf,4+PACKET_SIZE);
	if (rc < 0)
	{
		printk("[elan] Received the packet Error.\n");
		return IRQ_HANDLED;
	}
#else
	rc = elan_ktf_ts_recv_data(ts->client, buf, PACKET_SIZE);
	if (rc < 0)
	{
		printk("[elan] Received the packet Error.\n");
		return IRQ_HANDLED;
	}
#endif
	touch_debug(DEBUG_TRACE,"%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x ....., %2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[17]);

#ifndef ELAN_BUFFER_MODE
	elan_ktf_ts_report_data(ts->client, buf);
#else
	elan_ktf_ts_report_data(ts->client, buf+4);

	// Second package
	if (((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3))) {
		rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
		if (rc < 0){
			return IRQ_HANDLED;
		}
		elan_ktf_ts_report_data(ts->client, buf1);
		// Final package
		if (buf[1] == 3) {
			rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if (rc < 0){
				return IRQ_HANDLED;
			}
			elan_ktf_ts_report_data(ts->client, buf1);
		}
	}
#endif

	return IRQ_HANDLED;
}

static int elan_ktf_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_threaded_irq(client->irq, NULL, elan_ktf_ts_irq_handler,
	IRQF_TRIGGER_LOW /*| IRQF_TRIGGER_FALLING*/ | IRQF_ONESHOT,
	client->name, ts);
	if (err) {
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
		__func__, client->irq);
	}
	
	return err;
}

#ifdef _ENABLE_DBG_LEVEL
static int ektf_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data )
{
	int ret;

	touch_debug(DEBUG_MESSAGES, "call proc_read\n");

	if(offset > 0)  /* we have finished to read, return 0 */
	ret  = 0;
	else
	ret = sprintf(buffer, "Debug Level: Release Date: %s\n","2011/10/05");

	return ret;
}


static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0;
	int i, ret = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int command;

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN )
	procfs_buffer_size = PROC_FS_MAX_LEN+1;

	if( copy_from_user(procfs_buf, buffer, procfs_buffer_size) )
	{
		touch_debug(DEBUG_ERROR, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for(i=0; i<procfs_buffer_size-1; i++)
	{
		if( procfs_buf[i]>='0' && procfs_buf[i]<='9' )
		command |= (procfs_buf[i]-'0');
		else if( procfs_buf[i]>='A' && procfs_buf[i]<='F' )
		command |= (procfs_buf[i]-'A'+10);
		else if( procfs_buf[i]>='a' && procfs_buf[i]<='f' )
		command |= (procfs_buf[i]-'a'+10);

		if(i!=procfs_buffer_size-2)
		command <<= 4;
	}

	command = command&0xFFFFFFFF;
	switch(command){
	case 0xF1:
		gPrint_point = 1;
		break;
	case 0xF2:
		gPrint_point = 0;
		break;
	case 0xFF:
		ret = elan_ktf_ts_calibrate(private_ts->client);
		break;
	}
	touch_debug(DEBUG_INFO, "Run command: 0x%08X  result:%d\n", command, ret);

	return count; // procfs_buffer_size;
}
#endif // #ifdef _ENABLE_DBG_LEV

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct elan_ktf_ts_data *elan_dev_data =
	container_of(self, struct elan_ktf_ts_data, fb_notif);
	printk("%s fb notifier callback\n",__func__);
/*[Arima_5830][bozhi_lin] fine tune touch suspend time to get gesture event 20160707 begin*/
	if (evdata && evdata->data && elan_dev_data && private_ts->client) {
		if (event == FB_EARLY_EVENT_BLANK) {
			touch_debug(DEBUG_TRACE, "FB_EARLY_EVENT_BLANK\n");
			blank = evdata->data;
			if (*blank == FB_BLANK_POWERDOWN) {
				touch_debug(DEBUG_TRACE, "suspend\n");
				elan_ktf_ts_suspend(elan_dev_data->client, PMSG_SUSPEND);
			}
		} else if (event == FB_EVENT_BLANK) {
			touch_debug(DEBUG_TRACE, "FB_EVENT_BLANK\n");
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
			{
				touch_debug(DEBUG_TRACE, "resume\n");
				elan_ktf_ts_resume(elan_dev_data->client);
			}
			else if (*blank == FB_BLANK_POWERDOWN)
			{
				touch_debug(DEBUG_TRACE, "suspend\n");
				elan_ktf_ts_suspend(elan_dev_data->client, PMSG_SUSPEND);
			}
		}
	}
/*[Arima_5830][bozhi_lin] 20160707 end*/

	return 0;
}
#endif

static int elan_ktf_ts_regulator_configure(struct elan_ktf_ts_data
						*elan_data, bool on)
{
	int retval;

	if (on == false)
		goto hw_shutdown;

	elan_data->vdd = regulator_get(&elan_data->client->dev,
					"vdd");
	if (IS_ERR(elan_data->vdd)) {
		dev_err(&elan_data->client->dev,
				"%s: Failed to get vdd regulator\n",
				__func__);
		return PTR_ERR(elan_data->vdd);
	}

	if (regulator_count_voltages(elan_data->vdd) > 0) {
		retval = regulator_set_voltage(elan_data->vdd,
			ELAN_VTG_MIN_UV, ELAN_VTG_MAX_UV);
		if (retval) {
			dev_err(&elan_data->client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd;
		}
	}

	if (elan_data->i2c_pull_up) {
		elan_data->vcc_i2c = regulator_get(&elan_data->client->dev,
						"vcc_i2c");
		if (IS_ERR(elan_data->vcc_i2c)) {
			dev_err(&elan_data->client->dev,
					"%s: Failed to get i2c regulator\n",
					__func__);
			retval = PTR_ERR(elan_data->vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(elan_data->vcc_i2c) > 0) {
			retval = regulator_set_voltage(elan_data->vcc_i2c,
				ELAN_I2C_VTG_MIN_UV, ELAN_I2C_VTG_MAX_UV);
			if (retval) {
				dev_err(&elan_data->client->dev,
					"reg set i2c vtg failed retval =%d\n",
					retval);
			goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (elan_data->i2c_pull_up)
		regulator_put(elan_data->vcc_i2c);
err_get_vtg_i2c:
	if (regulator_count_voltages(elan_data->vdd) > 0)
		regulator_set_voltage(elan_data->vdd, 0,
			ELAN_VTG_MAX_UV);
err_set_vtg_vdd:
	regulator_put(elan_data->vdd);
	return retval;

hw_shutdown:
	if (regulator_count_voltages(elan_data->vdd) > 0)
		regulator_set_voltage(elan_data->vdd, 0,
			ELAN_VTG_MAX_UV);
	regulator_put(elan_data->vdd);
	if (elan_data->i2c_pull_up) {
		if (regulator_count_voltages(elan_data->vcc_i2c) > 0)
			regulator_set_voltage(elan_data->vcc_i2c, 0,
					ELAN_I2C_VTG_MAX_UV);
		regulator_put(elan_data->vcc_i2c);
	}
	return 0;
};

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int elan_ktf_ts_power_on(struct elan_ktf_ts_data *elan_data,
					bool on) {
	int retval;

	if (on == false)
		goto power_off;

	retval = reg_set_optimum_mode_check(elan_data->vdd,
		ELAN_ACTIVE_LOAD_UA);
	if (retval < 0) {
		dev_err(&elan_data->client->dev,
			"Regulator vdd set_opt failed rc=%d\n",
			retval);
		return retval;
	}

	retval = regulator_enable(elan_data->vdd);
	if (retval) {
		dev_err(&elan_data->client->dev,
			"Regulator vdd enable failed rc=%d\n",
			retval);
		goto error_reg_en_vdd;
	}

	if (elan_data->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(elan_data->vcc_i2c,
			ELAN_I2C_LOAD_UA);
		if (retval < 0) {
			dev_err(&elan_data->client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n",
				retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(elan_data->vcc_i2c);
		if (retval) {
			dev_err(&elan_data->client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n",
				retval);
			goto error_reg_en_vcc_i2c;
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	if (elan_data->i2c_pull_up)
		reg_set_optimum_mode_check(elan_data->vdd, 0);
error_reg_opt_i2c:
	regulator_disable(elan_data->vdd);
error_reg_en_vdd:
	reg_set_optimum_mode_check(elan_data->vdd, 0);
	return retval;

power_off:
	reg_set_optimum_mode_check(elan_data->vdd, 0);
	regulator_disable(elan_data->vdd);
	if (elan_data->i2c_pull_up) {
		reg_set_optimum_mode_check(elan_data->vcc_i2c, 0);
		regulator_disable(elan_data->vcc_i2c);
	}
	return 0;
}

static int elan_ktf_ts_parse_dt(struct device *dev,
				struct elan_ktf_i2c_platform_data *elan_pdata)
{
	struct device_node *np = dev->of_node;

	elan_pdata->i2c_pull_up = of_property_read_bool(np,
			"elan,i2c-pull-up");

	/* reset, irq gpio info */
	elan_pdata->reset_gpio = of_get_named_gpio_flags(np,
			"elan,reset-gpio", 0, &elan_pdata->reset_flags);
	elan_pdata->irq_gpio = of_get_named_gpio_flags(np,
			"elan,irq-gpio", 0, &elan_pdata->irq_flags);
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
	elan_pdata->id_gpio = of_get_named_gpio_flags(np,
			"elan,id-gpio", 0, &elan_pdata->id_flags);
/*[Arima_5816][bozhi_lin] 20151117 end*/

	return 0;
}

/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
/*[Arima_5830][bozhi_lin] disable touch cover mode 20160510 begin*/
#if 0
static int elan_ktf_smartwindow_switch(bool enable)
{
	uint8_t cmd_smartwindow_enable[]		= {0x54, 0x25, 0x01, 0x01};
	uint8_t cmd_smartwindow_disable[]	= {0x54, 0x25, 0x00, 0x01};

	//printk("[B]%s(%d): enable=%d\n", __func__, __LINE__, enable);
	if (enable){
		if ((i2c_master_send(private_ts->client, cmd_smartwindow_enable, sizeof(cmd_smartwindow_enable))) != sizeof(cmd_smartwindow_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_smartwindow_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_smartwindow_disable, sizeof(cmd_smartwindow_disable))) != sizeof(cmd_smartwindow_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_smartwindow_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}	
	}

	return 0;
}
#endif
/*[Arima_5830][bozhi_lin] 20160510 end*/

static int elan_ktf_smartwindow_query(void)
{
	uint8_t cmd_smartwindow_query[]		= {0x53, 0x25, 0x00, 0x01};
	uint8_t resp_smartwindow_query[4]  = {0};
	int ret = -1;
	
#ifdef ESD_CHECK
	flush_work(&private_ts->work);  
	cancel_delayed_work_sync(&private_ts->check_work); 
#endif
	disable_irq(private_ts->client->irq);
	
	if ((i2c_master_send(private_ts->client, cmd_smartwindow_query, sizeof(cmd_smartwindow_query))) != sizeof(cmd_smartwindow_query)) {
		printk("[B]%s(%d): i2c_master_send cmd_smartwindow_query failed\n", __func__, __LINE__);
		goto Err_SmartWindow;
	}

	elan_ktf_ts_poll(private_ts->client);//0104

	if ((i2c_master_recv(private_ts->client, resp_smartwindow_query, sizeof(resp_smartwindow_query))) != sizeof(resp_smartwindow_query)) {
		printk("[B]%s(%d): i2c_master_recv resp_smartwindow_query failed\n", __func__, __LINE__);
		goto Err_SmartWindow;
	}
	enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
    schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));	
#endif
	
	if (resp_smartwindow_query[2] == 0x01){
		printk("SmartWindow ON\n");
		ret = 1;
	}  
	if (resp_smartwindow_query[2] == 0x00){
		printk("SmartWindow OFF\n");
		ret = 0;
	}
  
	return ret;
  
Err_SmartWindow: 
	enable_irq(private_ts->client->irq); 
#ifdef ESD_CHECK
    schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));	
#endif
	return -1;
}

static int elan_ktf_powersupply_switch(int supplymode)
{
	uint8_t cmd_power_battery[]	= {0x54, 0x24, 0x00, 0x01};
	uint8_t cmd_power_usb[]	    = {0x54, 0x24, 0x01, 0x01};
	uint8_t cmd_power_charger[]	= {0x54, 0x24, 0x02, 0x01};
  
	//printk("[B]%s(%d): enable=%d\n", __func__, __LINE__, enable);
	switch (supplymode) {
		case POWER_BATTERY:
			if ((i2c_master_send(private_ts->client, cmd_power_battery, sizeof(cmd_power_battery))) != sizeof(cmd_power_battery)) {
				printk("[B]%s(%d): i2c_master_send cmd_power_battery failed\n", __func__, __LINE__);
				return -EINVAL;
			}
			break;
		
		case POWER_USB:
			if ((i2c_master_send(private_ts->client, cmd_power_usb, sizeof(cmd_power_usb))) != sizeof(cmd_power_usb)) {
				printk("[B]%s(%d): i2c_master_send cmd_power_usb failed\n", __func__, __LINE__);
				return -EINVAL;
			}
			break;
		
		case POWER_CHARGER:
			if ((i2c_master_send(private_ts->client, cmd_power_charger, sizeof(cmd_power_charger))) != sizeof(cmd_power_charger)) {
				printk("[B]%s(%d): i2c_master_send cmd_power_charger failed\n", __func__, __LINE__);
				return -EINVAL;
			}
			break;

		default:
			printk("[B]%s(%d): supplymode not in case.\n", __func__, __LINE__);
			break;
	}

	return 0;
}

static int elan_ktf_powersupply_query(void)
{
	uint8_t cmd_powersupply_query[]		= {0x53, 0x24, 0x00, 0x01};
	uint8_t resp_powersupply_query[4]  = {0};
	int ret = -1;

#ifdef ESD_CHECK
	flush_work(&private_ts->work);  
	cancel_delayed_work_sync(&private_ts->check_work); 
#endif
	disable_irq(private_ts->client->irq);
	
	if ((i2c_master_send(private_ts->client, cmd_powersupply_query, sizeof(cmd_powersupply_query))) != sizeof(cmd_powersupply_query)) {
		printk("[B]%s(%d): i2c_master_send cmd_powersupply_query failed\n", __func__, __LINE__);
		goto Err_Power;
	}

	elan_ktf_ts_poll(private_ts->client);//0104

	if ((i2c_master_recv(private_ts->client, resp_powersupply_query, sizeof(resp_powersupply_query))) != sizeof(resp_powersupply_query)) {
		printk("[B]%s(%d): i2c_master_recv resp_powersupply_query failed\n", __func__, __LINE__);
		goto Err_Power;
	}
  
	enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
    schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));	
#endif  

	if (resp_powersupply_query[2] == 0x00){
		printk("powersupply Battery\n");
		ret = POWER_BATTERY;
	}  
	if (resp_powersupply_query[2] == 0x01){
		printk("powersupply USB\n");
		ret = POWER_USB;
	}
	if (resp_powersupply_query[2] == 0x02){
		printk("powersupply Charge\n");
		ret = POWER_CHARGER;
	}

	return ret;
  
Err_Power:  
	enable_irq(private_ts->client->irq);  
#ifdef ESD_CHECK
    schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));	
#endif
	return -1;
}
/*[Arima_5816][bozhi_lin] 20151231 end*/

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
/*[Arima_5816][bozhi_lin] touch re-send gesture command in suspend function 20160104 begin*/
static int elan_ktf_dclick_switch(bool enable)
{
	uint8_t cmd_dclick_enable[]		= {CMD_W_PKT, 0x30, 0xF1, 0x01};
	uint8_t cmd_dclick_disable[]	= {CMD_W_PKT, 0x30, 0xF0, 0x01};

	//printk("[B]%s(%d): enable=%d\n", __func__, __LINE__, enable);
	if (enable){
		if ((i2c_master_send(private_ts->client, cmd_dclick_enable, sizeof(cmd_dclick_enable))) != sizeof(cmd_dclick_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_dclick_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_dclick_disable, sizeof(cmd_dclick_disable))) != sizeof(cmd_dclick_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_dclick_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}	
	}

	return 0;
}
/*[Arima_5816][bozhi_lin] 20160104 end*/

static ssize_t elan_ktf_wakeup_dclick_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->wakeup_dclick);
}

static ssize_t elan_ktf_wakeup_dclick_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	//printk("[B]%s(%d): buf= %s, size=%d\n", __func__, __LINE__,	buf, size);
	
	rc = kstrtoul(buf, 10, &val);
	if (rc != 0) {
		data->wakeup_dclick = false;
		return rc;
	}

	mutex_lock(&data->input_dev->mutex);
	
	if(val) {
		data->wakeup_dclick = true;
	} else {
		data->wakeup_dclick = false;	
	}

	//printk("[B]%s(%d): data->wakeup_dclick= %d\n", __func__, __LINE__,	data->wakeup_dclick);
	
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(wakeup_dclick, 0664, elan_ktf_wakeup_dclick_show,
				elan_ktf_wakeup_dclick_store);

/*[Arima_5816][bozhi_lin] touch re-send gesture command in suspend function 20160104 begin*/
static int elan_ktf_gesture_mode_switch(int set_mode)
{
	uint8_t cmd_gesture_w_enable[]	= {CMD_W_PKT, 0x30, 0x51, 0x01};
	uint8_t cmd_gesture_w_disable[]	= {CMD_W_PKT, 0x30, 0x50, 0x01};
	uint8_t cmd_gesture_s_enable[]	= {CMD_W_PKT, 0x30, 0x81, 0x01};
	uint8_t cmd_gesture_s_disable[]	= {CMD_W_PKT, 0x30, 0x80, 0x01};
	uint8_t cmd_gesture_e_enable[]	= {CMD_W_PKT, 0x30, 0x01, 0x01};
	uint8_t cmd_gesture_e_disable[]	= {CMD_W_PKT, 0x30, 0x00, 0x01};
	uint8_t cmd_gesture_c_enable[]	= {CMD_W_PKT, 0x30, 0x71, 0x01};
	uint8_t cmd_gesture_c_disable[]	= {CMD_W_PKT, 0x30, 0x70, 0x01};
	uint8_t cmd_gesture_z_enable[]	= {CMD_W_PKT, 0x30, 0x11, 0x01};
	uint8_t cmd_gesture_z_disable[]	= {CMD_W_PKT, 0x30, 0x10, 0x01};
	uint8_t cmd_gesture_v_enable[]	= {CMD_W_PKT, 0x30, 0x31, 0x01};
	uint8_t cmd_gesture_v_disable[]	= {CMD_W_PKT, 0x30, 0x30, 0x01};

	//printk("[B]%s(%d): set_mode=0x%x\n", __func__, __LINE__, set_mode);

	if (set_mode & GESTURE_W_ENABLE){
		if ((i2c_master_send(private_ts->client, cmd_gesture_w_enable, sizeof(cmd_gesture_w_enable))) != sizeof(cmd_gesture_w_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_w_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_gesture_w_disable, sizeof(cmd_gesture_w_disable))) != sizeof(cmd_gesture_w_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_w_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	if (set_mode & GESTURE_S_ENABLE){
		if ((i2c_master_send(private_ts->client, cmd_gesture_s_enable, sizeof(cmd_gesture_s_enable))) != sizeof(cmd_gesture_s_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_s_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_gesture_s_disable, sizeof(cmd_gesture_s_disable))) != sizeof(cmd_gesture_s_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_s_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	
	if (set_mode & GESTURE_E_ENABLE){
		if ((i2c_master_send(private_ts->client, cmd_gesture_e_enable, sizeof(cmd_gesture_e_enable))) != sizeof(cmd_gesture_e_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_e_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_gesture_e_disable, sizeof(cmd_gesture_e_disable))) != sizeof(cmd_gesture_e_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_e_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	
	if (set_mode & GESTURE_C_ENABLE){
		if ((i2c_master_send(private_ts->client, cmd_gesture_c_enable, sizeof(cmd_gesture_c_enable))) != sizeof(cmd_gesture_c_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_c_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_gesture_c_disable, sizeof(cmd_gesture_c_disable))) != sizeof(cmd_gesture_c_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_c_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	
	if (set_mode & GESTURE_Z_ENABLE){
		if ((i2c_master_send(private_ts->client, cmd_gesture_z_enable, sizeof(cmd_gesture_z_enable))) != sizeof(cmd_gesture_z_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_z_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_gesture_z_disable, sizeof(cmd_gesture_z_disable))) != sizeof(cmd_gesture_z_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_z_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	
	if (set_mode & GESTURE_V_ENABLE){
		if ((i2c_master_send(private_ts->client, cmd_gesture_v_enable, sizeof(cmd_gesture_v_enable))) != sizeof(cmd_gesture_v_enable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_v_enable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(private_ts->client, cmd_gesture_v_disable, sizeof(cmd_gesture_v_disable))) != sizeof(cmd_gesture_v_disable)) {
			printk("[B]%s(%d): i2c_master_send cmd_gesture_v_disable failed\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	return 0;
}
/*[Arima_5816][bozhi_lin] 20160104 end*/

static ssize_t elan_ktf_wakeup_gesture_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 8, "0x%x\n", data->wakeup_gesture_type);
}

static ssize_t elan_ktf_wakeup_gesture_type_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	int set_gesture_type;
	int rc;

	if (size > 8)
		return -EINVAL;

	//printk("[B]%s(%d): buf= %s, size=%d\n", __func__, __LINE__,	buf, size);

	rc = kstrtoint(buf, 2, &set_gesture_type);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	
	/*[Arima_5830][bozhi_lin] fix touch gesture map property error 20160503 begin*/
	if (set_gesture_type & GESTURE_ENABLE) {
		data->wakeup_gesture_type = set_gesture_type & GESTURE_MASK;
	} else {
		data->wakeup_gesture_type = 0;
	}
	/*[Arima_5830][bozhi_lin] 20160503 end*/
	//printk("[B]%s(%d): data->wakeup_gesture_type= 0x%x\n", __func__, __LINE__, data->wakeup_gesture_type);
	
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(wakeup_gesture_type, 0664, elan_ktf_wakeup_gesture_type_show,
				elan_ktf_wakeup_gesture_type_store);
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
//1117 Glove Mode
#if defined(GLOVEMODE)
static ssize_t elan_ktf_set_glove_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->glove_enable);
}

static ssize_t elan_ktf_set_glove_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	int set_glove;
	int rc;

	if (size > 2)
		return -EINVAL;

	//printk("[B]%s(%d): buf= %s, size=%d\n", __func__, __LINE__,	buf, size);

	rc = kstrtoint(buf, 10, &set_glove);
	if (rc != 0) {
		data->glove_enable = false;
		return rc;
	}

	mutex_lock(&data->input_dev->mutex);

	if (set_glove == 0){
		ExitGloveMode();
		data->glove_enable = false;
	}else if (set_glove == 1){
		EnterGloveMode();  
		data->glove_enable = true;
	}else{
		printk("[B]%s(%d): set_glove value error. value=%d\n", __func__, __LINE__,	set_glove);
	}

	printk("[B]%s(%d): data->glove_enable=%d\n", __func__, __LINE__, data->glove_enable);

	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(set_glove, 0664, elan_ktf_set_glove_show,
				elan_ktf_set_glove_store);
#endif
//1117
/*[Arima_5816][bozhi_lin] 20151119 end*/

/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
static ssize_t elan_ktf_set_powersupply_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	int res = elan_ktf_powersupply_query();
	int buff_size;

	if (res == POWER_CHARGER) {
		data->powersupply_type = POWER_CHARGER;
		buff_size = sprintf(buf, "%s", "AC");
	}
	else if (res == POWER_USB) {
		data->powersupply_type = POWER_USB;
		buff_size = sprintf(buf, "%s", "USB");
	}
	else if (res == POWER_BATTERY) {
		data->powersupply_type = POWER_BATTERY;
		buff_size = sprintf(buf, "%s", "Battery");
	}
	else {
		buff_size = sprintf(buf, "%s", "ERROR");
	}
	return buff_size;
}

static ssize_t elan_ktf_set_powersupply_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	int set_powersupply;
	int rc;

	if (size > 2)
		return -EINVAL;

	//printk("[B]%s(%d): buf= %s, size=%d\n", __func__, __LINE__,	buf, size);

	rc = kstrtoint(buf, 10, &set_powersupply);
	if (rc != 0) {
		printk("%s error\n", __func__);
		return rc;
	}

	mutex_lock(&data->input_dev->mutex);

	if (set_powersupply == 0){
		elan_ktf_powersupply_switch(POWER_BATTERY);
		data->powersupply_type = POWER_BATTERY;
	}else if (set_powersupply == 1){
		elan_ktf_powersupply_switch(POWER_USB);
		data->powersupply_type = POWER_USB;
	}else if (set_powersupply == 2){
		elan_ktf_powersupply_switch(POWER_CHARGER);
		data->powersupply_type = POWER_CHARGER;
	}else{
		printk("[B]%s(%d): set_powersupply value error. value=%d\n", __func__, __LINE__,	set_powersupply);
	}

	printk("[B]%s(%d): data->powersupply_type=%d\n", __func__, __LINE__, data->powersupply_type);

	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(set_powersupply, 0664, elan_ktf_set_powersupply_show,
				elan_ktf_set_powersupply_store);

static ssize_t elan_ktf_set_smartwindow_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	int res = elan_ktf_smartwindow_query();
	int buff_size;

	if (res == 1) {
		data->smartwindow_enable = true;
		buff_size = sprintf(buf, "%s", "ON");
	}
	else if (res == 0) {
		data->smartwindow_enable = false;
		buff_size = sprintf(buf, "%s", "OFF");
	}
	else {
		buff_size = sprintf(buf, "%s", "ERROR");
	}
	return buff_size;
}

static ssize_t elan_ktf_set_smartwindow_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct elan_ktf_ts_data *data = dev_get_drvdata(dev);
	int set_smartwindow;
	int rc;

	if (size > 2)
		return -EINVAL;

	//printk("[B]%s(%d): buf= %s, size=%d\n", __func__, __LINE__,	buf, size);

	rc = kstrtoint(buf, 10, &set_smartwindow);
	if (rc != 0) {
		printk("%s error\n", __func__);
		return rc;
	}

	mutex_lock(&data->input_dev->mutex);

/*[Arima_5830][bozhi_lin] disable touch cover mode 20160510 begin*/
#if 1
	data->smartwindow_enable = false;
#else
	if (set_smartwindow == 1){
		elan_ktf_smartwindow_switch(true);
		data->smartwindow_enable = true;
	}else if (set_smartwindow == 0){
		elan_ktf_smartwindow_switch(false);
		data->smartwindow_enable = false;
	}else{
		printk("[B]%s(%d): set_smartwindow value error. value=%d\n", __func__, __LINE__, set_smartwindow);
	}
#endif
/*[Arima_5830][bozhi_lin] 20160510 end*/

	printk("[B]%s(%d): set smartwindow %d\n", __func__, __LINE__, data->smartwindow_enable);

	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(set_smartwindow, 0664, elan_ktf_set_smartwindow_show,
				elan_ktf_set_smartwindow_store);
/*[Arima_5816][bozhi_lin] 20151231 end*/

#ifdef ESD_CHECK 
static void elan_ktf_ts_check_work_func(struct work_struct *work)
{	
	int res = 0;
		
	//disable_irq(private_ts->client->irq);//1102
	flush_work(&private_ts->work);	//1102

	if (fwupd_flag == 1)
	{
		schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
		return;		
	}

	if(live_state==1)
	{
		live_state =0;
		schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
		//enable_irq(private_ts->client->irq);
		return;
	}
		
	printk(KERN_EMERG "%s, chip may crash, we need to reset it \n",__func__);	
	
	elan_ktf_ts_hw_reset(private_ts->client);	
	
	disable_irq(private_ts->client->irq); //1102
	res = __hello_packet_handler(private_ts->client);		

	if (res != 0) 
	{
		printk(KERN_INFO "Receive hello package fail\n");
	} 

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);
}
#endif

static int elan_ktf_ts_probe(struct i2c_client *client,
const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf_i2c_platform_data *pdata;
	struct elan_ktf_ts_data *ts;
/*[Arima_5816][bozhi_lin] re-write touch upgrade firmware mechanism 20151124 begin*/
#if defined(FILE_UPGRADE)

#else
	int New_FW_ID;	
	int New_FW_VER;	
#endif
/*[Arima_5816][bozhi_lin] 20151124 end*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
	struct task_struct *fw_update_thread;
/*[Arima_5816][bozhi_lin] 20151014 end*/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "[elan] %s: allocate elan_ktf_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ktf_mutex);
	// james: maybe remove	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(*pdata),
			GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = elan_ktf_ts_parse_dt(&client->dev, pdata);
		if (err)
			return err;
	} else {
		pdata = client->dev.platform_data;
	}
	if (likely(pdata != NULL)) {
		ts->irq_gpio= pdata->irq_gpio;
		ts->reset_gpio = pdata->reset_gpio;
		ts->i2c_pull_up = pdata->i2c_pull_up;
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
		ts->id_gpio = pdata->id_gpio;
/*[Arima_5816][bozhi_lin] 20151117 end*/
	}

	err = elan_ktf_ts_regulator_configure(ts, true);
	if (err < 0) {
		dev_err(&client->dev, "Failed to configure regulators\n");
		goto err_reg_configure;
	}

	err = elan_ktf_ts_power_on(ts, true);
	if (err < 0) {
		dev_err(&client->dev, "Failed to power on\n");
		goto err_power_device;
	}

	if (gpio_is_valid(ts->irq_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(ts->irq_gpio, "elan_irq_gpio");
		if (err) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						ts->irq_gpio);
			goto err_irq_gpio_req;
		}
		err = gpio_direction_input(ts->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				ts->irq_gpio);
			goto err_irq_gpio_dir;
		}
		client->irq = gpio_to_irq(ts->irq_gpio);
	} else {
		dev_err(&client->dev, "irq gpio not provided\n");
		goto err_irq_gpio_req;
	}

	if (gpio_is_valid(ts->reset_gpio)) {
		/* configure touchscreen reset out gpio */
		err = gpio_request(ts->reset_gpio, "elan_reset_gpio");
		if (err) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						ts->reset_gpio);
			goto err_reset_gpio_req;
		}

		err = gpio_direction_output(ts->reset_gpio, 1);
		if (err) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				ts->reset_gpio);
			goto err_reset_gpio_dir;
		}
	} else {
		dev_err(&client->dev, "reset gpio not provided\n");
		goto err_reset_gpio_req;
	}

/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
	if (gpio_is_valid(ts->id_gpio)) {
		/* configure touchscreen id gpio */
		err = gpio_request(ts->id_gpio, "elan_id_gpio");
		if (err) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						ts->id_gpio);
			goto err_id_gpio_req;
		}

		err = gpio_direction_input(ts->id_gpio);
		if (err) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				ts->id_gpio);
			goto err_id_gpio_dir;
		}
	} else {
		dev_err(&client->dev, "reset gpio not provided\n");
		goto err_id_gpio_req;
	}

/*[Arima_5833][bozhi_lin] add touch 2nd source detect 20160818 begin*/
#if (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5830_SR && defined(CONFIG_BSP_HW_SKU_5830))
	pr_info("[B]%s(%d): Truly\n", __func__, __LINE__);
	#if defined(FILE_UPGRADE)
	strncpy(ts->image_name, TRULY_FW_FILENAME, MAX_IMAGE_NAME_LEN);
	#else
	file_fw_data = file_fw_data_truly;
	#endif
#elif (CONFIG_BSP_HW_V_CURRENT >= CONFIG_BSP_HW_V_5833_ER1 && defined(CONFIG_BSP_HW_SKU_5833))
	if (gpio_get_value(ts->id_gpio)) {
		pr_info("[B]%s(%d): Holitech2\n", __func__, __LINE__);
	#if defined(FILE_UPGRADE)
		strncpy(ts->image_name, HOLITECH_2_FW_FILENAME, MAX_IMAGE_NAME_LEN);
	#else
		file_fw_data = file_fw_data_truly;
	#endif
	} else {
		pr_info("[B]%s(%d): Holitech\n", __func__, __LINE__);
	#if defined(FILE_UPGRADE)
		strncpy(ts->image_name, HOLITECH_FW_FILENAME, MAX_IMAGE_NAME_LEN);
	#else
		file_fw_data = file_fw_data_holitech;
	#endif
	}
#else
	if (gpio_get_value(ts->id_gpio)) {
		pr_info("[B]%s(%d): Truly\n", __func__, __LINE__);
#if defined(FILE_UPGRADE)
		strncpy(ts->image_name, TRULY_FW_FILENAME, MAX_IMAGE_NAME_LEN);
#else
		file_fw_data = file_fw_data_truly;
#endif
	} else {
		pr_info("[B]%s(%d): Truly\n", __func__, __LINE__);
#if defined(FILE_UPGRADE)
		strncpy(ts->image_name, TRULY_FW_FILENAME, MAX_IMAGE_NAME_LEN);
#else
		file_fw_data = file_fw_data_holitech;
#endif
	}
#endif
/*[Arima_5833][bozhi_lin] 20160818 end*/
/*[Arima_5816][bozhi_lin] 20151117 end*/

	elan_ktf_ts_hw_reset(client);
	fw_err = elan_ktf_ts_setup(client);
	if (fw_err < 0) {
		printk(KERN_INFO "No Elan chip inside\n");
		/*[Arima_5816][bozhi_lin] probe return failed when touch not connected 20151123 begin*/
		err = -ENODEV; 
		goto err_input_dev_alloc_failed;
		/*[Arima_5816][bozhi_lin] 20151123 end*/
	}

	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "elan-touchscreen");
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";     
/*[Arima_5816][bozhi_lin] fix touch gesture not work in suspend state 20151012 begin*/
#if defined(GESTUREMODE)
	ts->input_dev->dev.parent = &client->dev;
#endif
/*[Arima_5816][bozhi_lin] 20151012 end*/

#ifdef PROTOCOL_A
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
#endif
#ifdef ELAN_BUTTON
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
/*[Arima_5816][bozhi_lin] fix touch home key not work 20151007 begin*/
/*[Arima_5816][bozhi_lin] enable elan touch virtual key 20150925 begin*/
#if defined(ARIMA_KEY)
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit); //recent key
	set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
#else
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif
/*[Arima_5816][bozhi_lin] 20150925 end*/
/*[Arima_5816][bozhi_lin] 20151007 end*/
/*[Arima_5816][bozhi_lin] 20160112 end*/
#endif
/*[Arima_5816][bozhi_lin] touch add arima key define 20160112 begin*/
/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
#if defined(ARIMA_KEY)
	set_bit(KEY_GESTURE_C, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_E, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_S, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_V, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_W, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_Z, ts->input_dev->keybit);
	set_bit(KEY_POWER, ts->input_dev->keybit);
#else
	set_bit(KEY_C, ts->input_dev->keybit);
	set_bit(KEY_E, ts->input_dev->keybit);
	set_bit(KEY_S, ts->input_dev->keybit);
	set_bit(KEY_V, ts->input_dev->keybit);
	set_bit(KEY_W, ts->input_dev->keybit);
	set_bit(KEY_Z, ts->input_dev->keybit);
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/
/*[Arima_5816][bozhi_lin] 20160112 end*/

/*[Arima_5830][bozhi_lin] add touch gesture key code follow arima design 20160413 begin*/
#if defined(ARIMA_DOUBLE_CLICK)
	set_bit(KEY_GESTURE_WAKE, ts->input_dev->keybit);
#endif
/*[Arima_5830][bozhi_lin] 20160413 end*/

#ifdef PROTOCOL_B
	input_mt_init_slots(ts->input_dev, FINGER_NUM);
#endif

#ifdef PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_X, 0,  X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0,  Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);	
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_DISTANCE, 0, MAX_FINGER_SIZE, 0, 0);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
		"[elan]%s: unable to register %s input device\n",
		__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf_ts_register_interrupt(ts->client);

	private_ts = ts;

	elan_ktf_touch_sysfs_init();

	dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
	ts->input_dev->name);

	// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&ts->firmware) < 0)
	printk("[ELAN]misc_register failed!!");
	else
	printk("[ELAN]misc_register finished!!");
	// End Firmware Update	

	/* register sysfs */
	if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
	dev_err(&client->dev, "sysfs create group error\n");

#ifdef IAP_PORTION
	if(1)
	{
		printk("[ELAN]misc_register finished!!");

/*[Arima_5816][bozhi_lin] re-write touch upgrade firmware mechanism 20151124 begin*/
#if defined(FILE_UPGRADE)
		fw_update_thread = kthread_run(FW_Update_Check, NULL, "elan_update");
		if(IS_ERR(fw_update_thread))
		{
			printk("[elan]  failed to create kernel thread\n");
		}
#else
		/* FW ID & FW VER*/
#if 0  /* For ektf21xx and ektf20xx  */
		printk("[ELAN]  [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[31696],file_fw_data[31697],file_fw_data[31698],file_fw_data[31699]);
		New_FW_ID = file_fw_data[31699]<<8  | file_fw_data[31698] ;	       
		New_FW_VER = file_fw_data[31697]<<8  | file_fw_data[31696] ;
#endif
		
#if 0   /* for ektf31xx 2 wire ice ex: 2wireice -b xx.bin */
		printk(" [7c16]=0x%02x,  [7c17]=0x%02x, [7c18]=0x%02x, [7c19]=0x%02x\n",  file_fw_data[31766],file_fw_data[31767],file_fw_data[31768],file_fw_data[31769]);
		New_FW_ID = file_fw_data[31769]<<8  | file_fw_data[31768] ;	       
		New_FW_VER = file_fw_data[31767]<<8  | file_fw_data[31766] ;
#endif	
#if 0  /* for L2500 */	
		printk(" [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[0x7bd0],file_fw_data[0x7bd1],file_fw_data[0x7bd2],file_fw_data[0x7bd3]);
		New_FW_ID = file_fw_data[0x7bd3]<<8  | file_fw_data[0x7bd2] ;	       
		New_FW_VER = file_fw_data[0x7bd1]<<8  | file_fw_data[0x7bd0] ;
#endif		
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
#if 0
		/* for ektf31xx iap ekt file   */	
		printk(" [7bd8]=0x%02x,  [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n",  file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
		New_FW_ID = file_fw_data[31707]<<8  | file_fw_data[31708] ;	       
		New_FW_VER = file_fw_data[31705]<<8  | file_fw_data[31704] ;
#endif
/*[Arima_5816][bozhi_lin] 20151014 end*/
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
#if 1
		printk("[ELAN]  [7d64]=0x%02x,  [7d65]=0x%02x, [7d66]=0x%02x, [7d67]=0x%02x\n",  file_fw_data[0x7d64],file_fw_data[0x7d65],file_fw_data[0x7d66],file_fw_data[0x7d67]);
		New_FW_ID = file_fw_data[0x7d67]<<8  | file_fw_data[0x7d66] ;	       
		New_FW_VER = file_fw_data[0x7d65]<<8  | file_fw_data[0x7d64] ;
#endif
/*[Arima_5816][bozhi_lin] 20151014 end*/
		printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);   	       
		printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);  

		/* for firmware auto-upgrade */      
/*[Arima_5816][bozhi_lin] upgrade elan touch firmware to 0x5506 and enable firmware upgrade 20151014 begin*/
		if (RECOVERY == 0x80){
			//fwupd_flag = 1; //1105
/*[Arima_5816][bozhi_lin] touch modify upgrade firmware in kernel 20160105 begin*/
#if defined(FILE_UPGRADE)
			fw_update_thread = kthread_run(FW_Update, NULL, "elan_update");
#else
			fw_update_thread = kthread_run(Update_FW_in_Driver, NULL, "elan_update");
#endif
/*[Arima_5816][bozhi_lin] 20160105 end*/
			if(IS_ERR(fw_update_thread))
			{
				printk("[elan]  failed to create kernel thread\n");
			}
		} else {
			if (New_FW_ID == FW_ID){
				if (New_FW_VER > FW_VERSION){
/*[Arima_5816][bozhi_lin] touch modify upgrade firmware in kernel 20160105 begin*/
#if defined(FILE_UPGRADE)
					fw_update_thread = kthread_run(FW_Update, NULL, "elan_update");
#else
					fw_update_thread = kthread_run(Update_FW_in_Driver, NULL, "elan_update");
#endif
/*[Arima_5816][bozhi_lin] 20160105 end*/
					if(IS_ERR(fw_update_thread))
					{
						printk("[elan]  failed to create kernel thread\n");
					}
				}
			} else {
					printk("FW_ID is different!");		
			}
		}

		if (fwupd_flag == 1) {  
			printk("[ELAN]fwupd_flag = 1: execute Update FW Now, return");
		} else {
			err = getCheckSUM(client);
			if(err == -1){
				printk("[ELAN]probe: getCheckSUM error, goto IAP.\n");
//				fw_update_thread = kthread_run(Update_FW_in_Driver, NULL, "elan_update");
				fw_update_thread = kthread_run(FW_Update, NULL, "elan_update");
				if(IS_ERR(fw_update_thread))
				{
					  printk("[elan]  failed to create kernel thread\n");
				}
			}
		}

/*[Arima_5816][bozhi_lin] 20151014 end*/
#endif
/*[Arima_5816][bozhi_lin] 20151124 end*/

	}
#endif	

#ifdef ESD_CHECK
	INIT_DELAYED_WORK(&ts->check_work, elan_ktf_ts_check_work_func);	// reset if check hang
	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
#endif 

#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = create_proc_entry(PROC_FS_NAME, 0600, NULL);
	if (dbgProcFile == NULL)
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		touch_debug(DEBUG_INFO, " Could not initialize /proc/%s\n", PROC_FS_NAME);
	}
	else
	{
		dbgProcFile->read_proc = ektf_proc_read;
		dbgProcFile->write_proc = ektf_proc_write;
		touch_debug(DEBUG_INFO, " /proc/%s created\n", PROC_FS_NAME);
	}
#endif // #ifdef _ENABLE_DBG_LEVEL
#if defined(CONFIG_FB)
	private_ts->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&private_ts->fb_notif);
	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",	err);

#endif

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
	err = device_create_file(&client->dev, &dev_attr_wakeup_dclick);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto err_input_register_device_failed;
	}
	
	err = device_create_file(&client->dev, &dev_attr_wakeup_gesture_type);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_wakeup_dclick_sys;
	}
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
#if defined(GLOVEMODE) //1117
	err = device_create_file(&client->dev, &dev_attr_set_glove);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_wakeup_gesture_type_sys;
	}
#endif
/*[Arima_5816][bozhi_lin] 20151119 end*/

/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
	err = device_create_file(&client->dev, &dev_attr_set_smartwindow);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_set_glove_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_set_powersupply);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_set_smartwindow_sys;
	}
/*[Arima_5816][bozhi_lin] 20151231 end*/

/*[Arima_5816][bozhi_lin] fix touch gesture not work in suspend state 20151012 begin*/
#if defined(GESTUREMODE)
	device_init_wakeup(&client->dev, 1);
#endif
/*[Arima_5816][bozhi_lin] 20151012 end*/

	return 0;

/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
free_set_smartwindow_sys:
	device_remove_file(&client->dev, &dev_attr_set_smartwindow);

free_set_glove_sys:
	device_remove_file(&client->dev, &dev_attr_set_glove);
/*[Arima_5816][bozhi_lin] 20151231 end*/

/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
#if defined(GLOVEMODE) //1117
free_wakeup_gesture_type_sys:
	device_remove_file(&client->dev, &dev_attr_wakeup_gesture_type);
#endif
/*[Arima_5816][bozhi_lin] 20151119 end*/

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
free_wakeup_dclick_sys:
	device_remove_file(&client->dev, &dev_attr_wakeup_dclick);
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

err_input_register_device_failed:
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif /* CONFIG_FB */
	if (ts->input_dev)
	input_free_device(ts->input_dev);

/*[Arima_5816][bozhi_lin] probe return failed when touch not connected 20151123 begin*/
#ifdef _ENABLE_DBG_LEVEL
	if (dbgProcFile)
		remove_proc_entry(PROC_FS_NAME, NULL);
#endif
/*[Arima_5816][bozhi_lin] 20151123 end*/

err_input_dev_alloc_failed: 
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
err_id_gpio_dir:
	if (gpio_is_valid(ts->id_gpio))
		gpio_free(ts->id_gpio);
/*[Arima_5816][bozhi_lin] 20151117 end*/
err_reset_gpio_dir:
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
err_id_gpio_req:
/*[Arima_5816][bozhi_lin] 20151117 end*/
err_reset_gpio_req:
err_irq_gpio_req:
	elan_ktf_ts_power_on(ts, false);
err_power_device:
	elan_ktf_ts_regulator_configure(ts, false);
err_reg_configure:
	input_free_device(ts->input_dev);
	ts->input_dev = NULL;
	kfree(ts);


err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf_ts_remove(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif /* CONFIG_FB */

	//unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);

	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
	if (gpio_is_valid(ts->id_gpio))
		gpio_free(ts->id_gpio);
/*[Arima_5816][bozhi_lin] 20151117 end*/

	elan_ktf_ts_power_on(ts, false);
	elan_ktf_ts_regulator_configure(ts, false);
	
	kfree(ts);

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
	device_remove_file(&client->dev, &dev_attr_wakeup_dclick);
	device_remove_file(&client->dev, &dev_attr_wakeup_gesture_type);
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

/*[Arima_5816][bozhi_lin] implement touch glove mode 20151119 begin*/
#if defined(GLOVEMODE) //1117
	device_remove_file(&client->dev, &dev_attr_set_glove);
#endif
/*[Arima_5816][bozhi_lin] 20151119 end*/

/*[Arima_5816][bozhi_lin] touch add smartwindow and powersupply file node for Asus cover and charger noise 20151231 begin*/
	device_remove_file(&client->dev, &dev_attr_set_smartwindow);
	device_remove_file(&client->dev, &dev_attr_set_powersupply);
/*[Arima_5816][bozhi_lin] 20151231 end*/

	return 0;
}

static int elan_ktf_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
	"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf_ts_get_data(client, cmd, buf, 4, 4);
	if (rc)
	return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
	power_state == PWR_STATE_DEEP_SLEEP ?
	"Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
	int rc = 0;
	int   len = 0;
	uint8_t gesture_cmd[] = { CMD_W_PKT, 0x40, 0x01, 0x01 };
#else
	int rc = 0;
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

#if defined(CONFIG_FB)
	if (ts->elan_is_suspend) {
		dev_dbg(&client->dev, "Already in suspend state.\n");
		return 0;
	}
#endif

	if(power_lock==0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		printk(KERN_INFO "[elan] %s: enter\n", __func__);

		#ifdef ESD_CHECK  //1102 --start
		flush_work(&ts->work);  
		cancel_delayed_work_sync(&ts->check_work); 
		#endif //1102 --end

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
		if ((ts->wakeup_dclick) || (ts->wakeup_gesture_type)) {
			pr_info("[B]%s(%d): ts->wakeup_dclick=%d, ts->wakeup_gesture_type=0x%x", __func__, __LINE__, ts->wakeup_dclick, ts->wakeup_gesture_type);
			rc = elan_ktf_dclick_switch(ts->wakeup_dclick);
			if (rc != 0) {
				printk("[B]%s(%d): elan_ktf_dclick_switch enable failed, ts->wakeup_dclick=%d\n", __func__, __LINE__,	ts->wakeup_dclick);
			}

			rc = elan_ktf_gesture_mode_switch(ts->wakeup_gesture_type);
			if (rc != 0) {
				printk("[B]%s(%d): elan_ktf_gesture_mode_switch enable failed, ts->wakeup_gesture_type=%d\n", __func__, __LINE__,	ts->wakeup_gesture_type);
			}
/*[Arima_5816][bozhi_lin] fix touch gesture not work in suspend state 20151012 begin*/
			if (device_may_wakeup(&client->dev))
				enable_irq_wake(client->irq);
/*[Arima_5816][bozhi_lin] 20151012 end*/
		}
		else {
			disable_irq(client->irq);
		}
#else
		disable_irq(client->irq);
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
		if ((ts->wakeup_dclick) || (ts->wakeup_gesture_type)) {
			printk("[elan] TP enter into guesture mode\n");
			len = i2c_master_send(client, gesture_cmd, sizeof( gesture_cmd ));
			if( len != sizeof( gesture_cmd ))
			{
				printk("[ELAN] ERROR: enter gesture mode fail! len=%d\n", len);
				return 0;
			}
				else {
				printk("[ELAN] enter gesture mode successfully! cmd = [%02X, %02X, %02X, %02X]\n", gesture_cmd[0], gesture_cmd[1], gesture_cmd[2], gesture_cmd[3]);
			}
		}
		else {
			rc = elan_ktf_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
		}
#else
		rc = elan_ktf_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/
	}
#if defined(CONFIG_FB)
	ts->elan_is_suspend = 1;
#endif
	return 0;
}

static int elan_ktf_ts_resume(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int res = 0;
/*[Arima_5816][bozhi_lin] touch use command to wake instead of reset pin 20151215 begin*/
#if defined(GESTUREMODE)
	int   len = 0;
	uint8_t gesture_cmd_disable[] = { CMD_W_PKT, 0x40, 0x00, 0x01 };
#endif
/*[Arima_5816][bozhi_lin] 20151215 end*/

#ifdef RE_CALIBRATION
	uint8_t buf_recv[4] = { 0 };
#endif

#if defined(CONFIG_FB)
	if (!ts->elan_is_suspend) {
		dev_dbg(&client->dev, "Already in awake state.\n");
		return 0;
	}
#endif

	if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		printk(KERN_INFO "[elan] %s: enter\n", __func__);

/*[Arima_5816][bozhi_lin] touch disable irq after resume in gesture mode to avoid receive hello packet 20151117 begin*/
#if defined(GESTUREMODE)
		if ((ts->wakeup_dclick) || (ts->wakeup_gesture_type)) {
			disable_irq(client->irq);	
		}
#endif
/*[Arima_5816][bozhi_lin] 20151117 end*/

/*[Arima_5816][bozhi_lin] touch use command to wake instead of reset pin 20151215 begin*/
#if defined(GESTUREMODE)
		if ((ts->wakeup_dclick) || (ts->wakeup_gesture_type)) {
			len = i2c_master_send(client, gesture_cmd_disable, sizeof( gesture_cmd_disable ));
			if( len != sizeof( gesture_cmd_disable ))
			{
				printk("[ELAN] ERROR: exit gesture mode fail! len=%d\n", len);
			}
			else {
				printk("[ELAN] enter exit mode successfully! cmd = [%02X, %02X, %02X, %02X]\n", gesture_cmd_disable[0], gesture_cmd_disable[1], gesture_cmd_disable[2], gesture_cmd_disable[3]);
			}
			/*[Arima_5816][bozhi_lin] touch send re-calibraion command in gesture mode 20160108 begin*/
			res = elan_ktf_ts_calibrate(client);
			/*[Arima_5816][bozhi_lin] 20160108 end*/
			/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
			if (res != 0) {
				printk("[ELAN] elan_ktf_ts_calibrate fail! res=%d\n", res);
			}
			/*[Arima_5816][bozhi_lin] 20160114 end*/
		}
		else {
			res = elan_ktf_ts_set_power_state(client, PWR_STATE_NORMAL);
			/*[Arima_5816][bozhi_lin] fix coverity bug 20160114 begin*/
			if (res != 0) {
				printk("[ELAN] elan_ktf_ts_set_power_state fail! res=%d\n", res);
			}
			/*[Arima_5816][bozhi_lin] 20160114 end*/
		}
		res = elan_ktf_ts_get_power_state(client);
		if (res != PWR_STATE_NORMAL) {
			printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n",	__func__, res);
			printk("[elan] %s: used reset to resume touch panel\n", __func__);
			elan_ktf_ts_hw_reset(client);

			mdelay(100);
			res = __hello_packet_handler(private_ts->client);
			if (res != 0) {
				printk("[elan] %s, fw_packet_handler fail, res = %d", __func__, res);
			}
			/*[Arima_5816][bozhi_lin] re-enable glove mode when glove mode is enabled before suspend 20151211 begin*/
			#if defined(GLOVEMODE)
			if(ts->glove_enable) {
				EnterGloveMode();  
			}
			#endif
			/*[Arima_5816][bozhi_lin] 20151211 end*/
		}
#else
		printk("[elan] %s: Used Reset instead of command to resume touch panel\n", __func__);

		elan_ktf_ts_hw_reset(client);

		mdelay(100);
		res = __hello_packet_handler(private_ts->client);
		if (res != 0) {
			printk("[elan] %s, fw_packet_handler fail, res = %d", __func__, res);
		}
#endif
/*[Arima_5816][bozhi_lin] 20151215 end*/

/*[Arima_5816][bozhi_lin] enable elan touch gesture and double-click mode 20151006 begin*/
#if defined(GESTUREMODE)
		if ((ts->wakeup_dclick) || (ts->wakeup_gesture_type)) {
			pr_info("[B]%s(%d): ts->wakeup_dclick=%d, ts->wakeup_gesture_type=0x%x", __func__, __LINE__, ts->wakeup_dclick, ts->wakeup_gesture_type);		
/*[Arima_5816][bozhi_lin] fix touch gesture not work in suspend state 20151012 begin*/
			if (device_may_wakeup(&client->dev))
				disable_irq_wake(client->irq);
/*[Arima_5816][bozhi_lin] 20151012 end*/
/*[Arima_5816][bozhi_lin] touch disable irq after resume in gesture mode to avoid receive hello packet 20151117 begin*/
			enable_irq(client->irq);
/*[Arima_5816][bozhi_lin] 20151117 end*/
		}
		else {
			enable_irq(client->irq);
		}		
#else
		enable_irq(client->irq);
#endif
/*[Arima_5816][bozhi_lin] 20151006 end*/
#ifdef ESD_CHECK
    schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));	
#endif
	}

#if defined(CONFIG_FB)	
	ts->elan_is_suspend = 0;
#endif

	return 0;
}

#if defined(CONFIG_FB)
static const struct dev_pm_ops elan_ktf_ts_dev_pm_ops = {
};
#else
static const struct dev_pm_ops elan_ktf_ts_dev_pm_ops = {
	.suspend = elan_ktf_ts_suspend,
	.resume = elan_ktf_ts_resume,
};
#endif

static const struct i2c_device_id elan_ktf_ts_id[] = {
	{ ELAN_KTF_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id elan_ktf_ts_match_table[] = {
	{ .compatible = "elan,ktf2k_ts",},
	{ },
};
#else
#define elan_ktf_ts_match_table NULL
#endif
static struct i2c_driver ektf_ts_driver = {
	.probe		= elan_ktf_ts_probe,
	.remove		= elan_ktf_ts_remove,
	.id_table	= elan_ktf_ts_id,
	.driver		= {
		.name = ELAN_KTF_NAME,
		.of_match_table = elan_ktf_ts_match_table,
		.owner    = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm = &elan_ktf_ts_dev_pm_ops,
#endif
	},
};

static int __init elan_ktf_ts_init(void)
{
	printk(KERN_INFO "[elan] %s driver version 0x0005: Integrated 2, 5, and 10 fingers together and auto-mapping resolution\n", __func__);
	return i2c_add_driver(&ektf_ts_driver);
}

static void __exit elan_ktf_ts_exit(void)
{
	i2c_del_driver(&ektf_ts_driver);
	return;
}

module_init(elan_ktf_ts_init);
module_exit(elan_ktf_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");



