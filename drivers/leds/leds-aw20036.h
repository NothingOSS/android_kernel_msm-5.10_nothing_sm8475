#ifndef __AW20036_H__
#define __AW20036_H__

/******************************************************
 *
 *Load config function
 *This driver will use load firmware if AW20036_BIN_CONFIG be defined
 *****************************************************/
/*#define  AW20036_BIN_CONFIG*/
#define  AW20036_ARRAY_CONFIG
/******************************************************
 *
 * Register List
 *
 *****************************************************/
#define REG_CHIPID            0x00
#define REG_WORK_MODE         0x01
#define REG_SWRST             0x02
#define REG_GCCR              0x03
#define REG_FCS               0x04
#define REG_CLKSYS            0x05
#define REG_FLTCFG1           0x09
#define REG_FLTCFG2           0x0A
#define REG_ISRFLT            0x0B
#define REG_LEDON0            0x31
#define REG_LEDON1            0x32
#define REG_LEDON2            0x33
#define REG_LEDON3            0x34
#define REG_LEDON4            0x35
#define REG_LEDON5            0x36
#define REG_PATE              0x43
#define REG_FADEH0            0x44
#define REG_FADEH1            0x45
#define REG_FADEH2            0x46
#define REG_FADEL0            0x47
#define REG_FADEL1            0x48
#define REG_FADEL2            0x49
#define REG_PAT0T0            0x4A
#define REG_PAT0T1            0x4B
#define REG_PAT0T2            0x4C
#define REG_PAT0T3            0x4D
#define REG_PAT1T0            0x4E
#define REG_PAT1T1            0x4F
#define REG_PAT1T2            0x50
#define REG_PAT1T3            0x51
#define REG_PAT2T0            0x52
#define REG_PAT2T1            0x53
#define REG_PAT2T2            0x54
#define REG_PAT2T3            0x55
#define REG_PAT0CFG           0x56
#define REG_PAT1CFG           0x57
#define REG_PAT2CFG           0x58
#define REG_PATGO             0x59
#define REG_SIZE              0x80
#define REG_PAGE              0xF0

/******************************************************
 *
 * Register Write/Read Access
 *
 *****************************************************/
#define REG_NONE_ACCESS                 0
#define REG_RD_ACCESS                   (1 << 0)
#define REG_WR_ACCESS                   (1 << 1)
#define AW20036_REG_MAX                 0x100

const unsigned char aw20036_reg_access[AW20036_REG_MAX] = {
	[REG_CHIPID] = REG_RD_ACCESS,
	[REG_WORK_MODE] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_SWRST] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_GCCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FCS] = REG_WR_ACCESS,
	[REG_CLKSYS] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FLTCFG1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FLTCFG2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_ISRFLT] = REG_RD_ACCESS,
	[REG_LEDON0] = REG_WR_ACCESS,
	[REG_LEDON1] = REG_WR_ACCESS,
	[REG_LEDON2] = REG_WR_ACCESS,
	[REG_LEDON3] = REG_WR_ACCESS,
	[REG_LEDON4] = REG_WR_ACCESS,
	[REG_LEDON5] = REG_WR_ACCESS,
	[REG_PATE] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FADEH0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FADEH1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FADEH2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FADEL0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FADEL1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FADEL2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0CFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1CFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2CFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PATGO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_SIZE] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAGE] = REG_RD_ACCESS | REG_WR_ACCESS,
};

/******************************************************
 *
 * Register Detail
 *
 *****************************************************/
#define BIT_GCR_CHIPEN_MASK             (~(1<<0))
#define BIT_GCR_CHIPEN_ENABLE           (0<<7)
#define BIT_GCR_CHIPEN_DISABLE          (1<<7)

#define BIT_INTEN_FUNCMPE_MASK          (~(1<<0))
#define BIT_INTEN_FUNCMPE_ENABLE        (1<<0)
#define BIT_INTEN_FUNCMPE_DISABLE       (0<<0)

#define BIT_INTST_FUNCMPE               (1<<0)

#define BIT_IMAX_MASK                   (~(15<<4))
#define BIT_ALLON_MASK                   (~(1<<3))
#define BIT_GCR_ALLON_DISABLE           (0<<3)
#define BIT_GCR_ALLON_ENABLE            (1<<3)
#define AW20036_DBGCTR_DIM 0x3F
#define AW20036_DBGCTR_FDAD 0xFF

#define BIT_PAT0_MASK		(~ (1<<0))
#define BIT_PAT0_ENABLE		(1<<0)
#define BIT_PAT1_MASK		(~ (1<<1))
#define BIT_PAT1_ENABLE		(1<<1)
#define BIT_PAT2_MASK		(~ (1<<2))
#define BIT_PAT2_ENABLE		(1<<2)
/*********************************************************
 *
 * pag num
 *
 ********************************************************/
#define AW20036_REG_NUM_PAG1       36
#define AW20036_REG_NUM_PAG2       36
#define AW20036_REG_NUM_PAG3       36
#define AW20036_REG_NUM_PAG4       72
#define AW20036_REG_NUM_PAG5       72

/*********************************************************
 *
 * chip info
 *
 ********************************************************/
#define AW20036_RSTR       0x01
#define AW20036_CHIPID      0x18

enum aw20036_flags {
	AW20036_FLAG_NONE = 0,
	AW20036_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw20036_reg_page {
	AW20036_REG_PAGE0 = 0xC0,
	AW20036_REG_PAGE1 = 0xC1,
	AW20036_REG_PAGE2 = 0xC2,
	AW20036_REG_PAGE3 = 0xC3,
	AW20036_REG_PAGE4 = 0xC4,
	AW20036_REG_PAGE5 = 0xC5,
};

enum aw20036_dbgctr {
	AW20036_DBGCTR_NORMAL = 0,
	AW20036_DBGCTR_IRAM = 1,
	AW20036_DBGCTR_SFR = 2,
	AW20036_DBGCTR_FLASH = 3,
	AW20036_DBGCTR_MAX = 4,
};

enum aw20036_imax {
	AW20036_IMAX_10mA = 0x00,
	AW20036_IMAX_20mA = 0X01,
	AW20036_IMAX_30mA = 0x02,
	AW20036_IMAX_40mA = 0x03,
	AW20036_IMAX_60mA = 0x04,
	AW20036_IMAX_80mA = 0x05,
	AW20036_IMAX_120mA = 0x06,
	AW20036_IMAX_160mA = 0x07,
	AW20036_IMAX_3P3mA = 0x08,
	AW20036_IMAX_6P7mA = 0x09,
	AW20036_IMAX_10P0mA = 0x0A,
	AW20036_IMAX_13P3mA = 0x0B,
	AW20036_IMAX_20P0mA = 0X0C,
	AW20036_IMAX_26P7mA = 0x0D,
	AW20036_IMAX_40P0mA = 0x0E,
	AW20036_IMAX_53P3mA = 0x0F,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct aw20036_container {
	unsigned int len;
	unsigned int version;
	unsigned int bist;
	unsigned int key;
	unsigned char data[];
};

struct aw20036 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct workqueue_struct *leds_workqueue;
	struct work_struct vip_notification_work;
	struct work_struct leds_effect_work;
	struct work_struct brightness_work;
	struct work_struct fw_work;
	struct work_struct cfg_work;
#ifdef AWINIC_FW_UPDATE_DELAY
	struct hrtimer fw_timer;
#endif
	struct mutex cfg_lock;

	int reset_gpio;
#ifdef CONFIG_OF
	struct device_node *irq_node;
	int irq_gpio;
#endif

	unsigned char flags;
	unsigned char chipid;
	unsigned char fw_update;
	unsigned char fw_flags;

	unsigned int imax;
	unsigned int fw_version;
	unsigned int operating_mode;
	unsigned int suspend;
	unsigned int vip_notification;
	unsigned int vip_notification_id;
	unsigned int factory_test;
	unsigned char always_on;

	unsigned char effect;
	unsigned char cfg;

	unsigned int rgbcolor;

	struct completion completion;
	atomic_t  breath_config;

	struct mmap_buf_format *start_buf;
	struct mmap_buf_format *curr_buf;

	unsigned char stream_mode;
	unsigned char sec_num;

	atomic_t hw_init;
};

typedef struct aw20036_cfg {
	unsigned char *p;
	unsigned int count;
} AW20036_CFG;

struct aw20036_breath_group {
	unsigned char * pat0;
	unsigned char * pat1;
	unsigned char * pat2;
};

#define LED_STRIPS_STREAM_MODE  _IOW('x', 0x50,  unsigned long)
#define LED_STRIPS_STOP_MODE _IOW('x', 0x51,  unsigned long)
#define LED_STRIPS_ALWAYS_ON _IOW('x', 0x52,  unsigned long)
#define LED_STRIPS_FREQ_SET _IOW('x', 0x53,  unsigned long)

enum {
	MMAP_BUF_DATA_VALID = 0x55,
	MMAP_BUF_DATA_FINISHED = 0xAA,
	MMAP_BUF_DATA_INVALID = 0xFF,
};

#define LED_MMAP_PAGE_ORDER		(1)
#define LED_MMAP_BUF_SUM			(8)
#define LED_MMAP_BUF_SIZE			(500)

#pragma pack(4)
struct mmap_buf_format {
	uint8_t status;
	uint8_t bit;
	uint16_t length;

	struct mmap_buf_format *kernel_next;
	struct mmap_buf_format *user_next;
	uint8_t brightness;
	uint16_t data[LED_MMAP_BUF_SIZE];
}; /* 1024 byte */
#pragma pack()
#endif
