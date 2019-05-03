#ifndef _LINUX_ELAN_KTF_H
#define _LINUX_ELAN_KTF_H

#if 1
#define ELAN_X_MAX      768
#define ELAN_Y_MAX      1344
#else
#define ELAN_X_MAX      1280
#define ELAN_Y_MAX      2112
#endif

#define L2500_ADDR			0x7bd0
#define EKTF2100_ADDR		0x7bd0
#define EKTF2200_ADDR		0x7bd0
#define EKTF3100_ADDR		0x7c16
#define FW_ADDR					L2500_ADDR



#define ELAN_KTF_NAME "elan_ktf"

struct elan_ktf_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	bool i2c_pull_up;
	int irq_gpio;
	u32 irq_flags;
	u32 reset_flags;
	int reset_gpio;
/*[Arima_5816][bozhi_lin] touch add gpio check main or 2nd source 20151117 begin*/
	u32 id_flags;
	int id_gpio;
/*[Arima_5816][bozhi_lin] 20151117 end*/
	int mode_check_gpio;
	int (*power)(int on);
};

#endif /* _LINUX_ELAN_KTF_H */
