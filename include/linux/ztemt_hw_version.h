#ifndef __ZTEMT_HW_VERSION_H__
#define __ZTEMT_HW_VERSION_H__

#ifdef CONFIG_ZTEMT_HW_VERSION_NX601J
typedef enum
{
	NX601J_HW_A,
	NX601J_HW_B,
	NX601J_HW_C,
	NX601J_HW_D,
	NX601J_HW_E,
	NX601J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX504J
typedef enum
{
	NX504J_HW_A,
	NX504J_HW_B,
	NX504J_HW_C,
	NX504J_HW_D,
	NX504J_HW_E,
	NX504J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX505J
typedef enum
{
	NX505J_HW_A,
	NX505J_HW_B,
	NX505J_HW_C,
	NX505J_HW_D,
	NX505J_HW_E,
	NX505J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX506J
typedef enum
{
	NX506J_HW_A,
	NX506J_HW_B,
	NX506J_HW_C,
	NX506J_HW_D,
	NX506J_HW_E,
	NX506J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX507J
typedef enum
{
	NX507J_HW_A,
	NX507J_HW_B,
	NX507J_HW_C,
	NX507J_HW_D,
	NX507J_HW_E,
	NX507J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX508J
typedef enum
{
	NX508J_HW_A,
	NX508J_HW_B,
	NX508J_HW_C,
	NX508J_HW_D,
	NX508J_HW_E,
	NX508J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX509J
typedef enum
{
	NX509J_HW_A,
	NX509J_HW_B,
	NX509J_HW_C,
	NX509J_HW_D,
	NX509J_HW_E,
	NX509J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX510J
typedef enum
{
	NX510J_HW_A,
	NX510J_HW_B,
	NX510J_HW_C,
	NX510J_HW_D,
	NX510J_HW_E,
	NX510J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#else
typedef enum
{
	HW_A,
	HW_B,
	HW_C,
	HW_D,
	HW_E,
	HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#endif

#ifdef CONFIG_ZTEMT_HW_VERSION_NX510J
enum ztemt_gpio_status {
 ZTE_GPIO_PULL_DOWN = 0,//gpio pull down
 ZTE_GPIO_FLOAT,//gpio float
 ZTE_GPIO_PULL_UP,//gpio pull up
 ZTE_GPIO_UNKNOWN,
};

struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	hw_version_type hw_type;
	int gpio_A;
	int gpio_B;
	int gpio_C;
	char hw_ver[20];
	char hw_wifi[20];

};

#elif defined CONFIG_ZTEMT_HW_VERSION_NX508J
enum ztemt_gpio_status {
 ZTE_GPIO_PULL_DOWN = 0,//gpio pull down
 ZTE_GPIO_FLOAT,//gpio float
 ZTE_GPIO_PULL_UP,//gpio pull up
 ZTE_GPIO_UNKNOWN,
};

struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	hw_version_type hw_type;
	int gpio_A;
	int gpio_B;
	int gpio_C;
	char hw_ver[20];
	char hw_wifi[20];
};

#elif defined CONFIG_ZTEMT_HW_VERSION_NX507J
struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	int low_mv_2;
	int high_mv_2;
	hw_version_type hw_type;
	char hw_ver[20];
	char hw_sc[20];
};
#else
struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	hw_version_type hw_type;
	char hw_ver[20];
};
#endif

#endif
