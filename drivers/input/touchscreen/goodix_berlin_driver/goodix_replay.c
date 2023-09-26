#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include "goodix_ts_core.h"
#include "goodix_replay_type.h"

#define MAX_FRAME_PACK_SIZE     4096
#define TX_CHANNEL_MAX          52
#define RX_CHANNEL_MAX          75

#define MUTUAL_TYPE_EN      BIT(8)
#define SELF_TYPE_EN        BIT(9)

#define LE16_TO_CPUP(x) le16_to_cpup((__le16 *)(x))

struct replay_package {
	uint32_t size;
	uint32_t offset;
	uint8_t *data;
};

typedef struct {
	uint32_t type;
	uint32_t len;
} replay_item_head_t;

#pragma pack(1)
struct fw_attr {
	uint16_t sample_frame_cnt;
	uint8_t fw_mode;
	uint8_t noise_level;
	uint16_t scan_rate;
	uint32_t status;
	uint64_t noise_state;
	uint8_t tick100ms;
	uint8_t skip_status;
	uint16_t res;
	uint8_t attr_buf[64];
};
#pragma pack()

struct kobject *parent;
static struct goodix_ts_core *cd;
static struct goodix_ic_info *ic_info;
static struct replay_package frame_pkg;
static struct mutex rep_mutex;
static bool store_rep_head;

static int append_replay_item(struct replay_package *pkg, uint32_t type,
	uint32_t len, const uint8_t *data)
{
	if (pkg->offset + sizeof(replay_item_head_t) + len > pkg->size) {
		ts_err("no enough memory, %d", pkg->size);
		return -1;
	}
	memcpy(pkg->data + pkg->offset, &type, sizeof(type));
	pkg->offset += sizeof(type);

	memcpy(pkg->data + pkg->offset, &len, sizeof(len));
	pkg->offset += sizeof(len);

	memcpy(pkg->data + pkg->offset, data, len);
	pkg->offset += len;
	return 0;
}

static struct replay_package *get_packed_frame(void)
{
	if (frame_pkg.offset > 0 && frame_pkg.data)
		return &frame_pkg;
	return NULL;
}

static void fill_replay_head(void)
{
	uint16_t type_group = GROUP_AFECAPS;
	uint16_t tx = ic_info->parm.drv_num;
	uint16_t rx = ic_info->parm.sen_num;
	u8 val = 0;

	append_replay_item(&frame_pkg, MISC_GROUP_BEGIN, MISC_GROUP_BEGIN_SIZE, (uint8_t *)&type_group);
	append_replay_item(&frame_pkg, AFECAPS_TX_NUM, AFECAPS_TX_NUM_SIZE, (uint8_t *)&tx);
	append_replay_item(&frame_pkg, AFECAPS_RX_NUM, AFECAPS_RX_NUM_SIZE, (uint8_t *)&rx);
	append_replay_item(&frame_pkg, AFECAPS_BUTTON_NUM, AFECAPS_BUTTON_NUM_SIZE, (uint8_t *)&ic_info->parm.button_num);
	append_replay_item(&frame_pkg, AFECAPS_FORCE_NUM, AFECAPS_FORCE_NUM_SIZE, (uint8_t *)&ic_info->parm.force_num);
	append_replay_item(&frame_pkg, AFECAPS_X_RESOLUTION, AFECAPS_X_RESOLUTION_SIZE, (uint8_t *)&ic_info->misc.screen_real_max_x);
	append_replay_item(&frame_pkg, AFECAPS_Y_RESOLUTION, AFECAPS_Y_RESOLUTION_SIZE, (uint8_t *)&ic_info->misc.screen_real_max_y);
	append_replay_item(&frame_pkg, AFECAPS_X_REVERSAL, AFECAPS_X_REVERSAL_SIZE, (uint8_t *)&val);
	append_replay_item(&frame_pkg, AFECAPS_Y_REVERSAL, AFECAPS_Y_REVERSAL_SIZE, (uint8_t *)&val);
	val = 1;
	append_replay_item(&frame_pkg, AFECAPS_XY_SWAP, AFECAPS_XY_SWAP_SIZE, (uint8_t *)&val);
	append_replay_item(&frame_pkg, MISC_GROUP_END, MISC_GROUP_END_SIZE, (uint8_t *)&type_group);
}

int fill_replay_data(u8 *buf)
{
	uint8_t *frame_ptr = buf;
	struct frame_head *frame_head;
	uint16_t type_group;
	uint64_t time_stamp = 0;
	struct fw_attr *fw_attr;
	uint8_t *fw_log;
	static uint8_t pkg_buf[MAX_FRAME_PACK_SIZE];
	static uint16_t mutual_duration, mutual_freq_a, mutual_freq_b;
	static uint8_t mutual_freq_a_index;
	static uint16_t self_tx_duration, self_rx_duration, self_tx_freq, self_rx_freq;
	static uint16_t mutual_adc[3], self_adc[3];
	static uint16_t max_com_noise[8];
	static uint16_t max_lcd_noise[8];
	static uint16_t im_raw_data[TX_CHANNEL_MAX * RX_CHANNEL_MAX];
	static uint16_t im_adc[3];

	if (checksum_cmp(frame_ptr, ic_info->misc.frame_data_head_len, CHECKSUM_MODE_U8_LE)) {
		ts_debug("frame head checksum error");
		return -1;
	}

	frame_head = (struct frame_head *)frame_ptr;
	if (checksum_cmp(frame_ptr, frame_head->cur_frame_len, CHECKSUM_MODE_U16_LE)) {
		ts_debug("frame body checksum error");
        return -1;
	}

	ts_debug("cur_frame_len:%d", frame_head->cur_frame_len);

	frame_pkg.size = sizeof(pkg_buf);
	frame_pkg.offset = 0;
	frame_pkg.data = pkg_buf;

	mutex_lock(&rep_mutex);

	if (store_rep_head) {
		fill_replay_head();
		goto exit;
	}

	type_group = GROUP_RUNNING;
	append_replay_item(&frame_pkg, MISC_GROUP_BEGIN, MISC_GROUP_BEGIN_SIZE, (uint8_t *)&type_group);
	type_group = GROUP_FRAME;
	append_replay_item(&frame_pkg, MISC_GROUP_BEGIN, MISC_GROUP_BEGIN_SIZE, (uint8_t *)&type_group);

	append_replay_item(&frame_pkg, FRAME_PROC_TIME_STAMP, FRAME_PROC_TIME_STAMP_SIZE, (uint8_t *)&time_stamp);

	/*======fw attr info========*/
	frame_ptr += ic_info->misc.frame_data_head_len;
	/* for reduce memcpy time we use pointer force transfer */
	fw_attr = (struct fw_attr *)frame_ptr;
	append_replay_item(&frame_pkg, FRAME_FW_MODE, FRAME_FW_MODE_SIZE, (uint8_t *)&fw_attr->fw_mode);
	append_replay_item(&frame_pkg, FRAME_NOISE_STATE, FRAME_NOISE_STATE_SIZE, (uint8_t *)&fw_attr->noise_state);
	append_replay_item(&frame_pkg, FRAME_TICK_100MS, FRAME_TICK_100MS_SIZE, (uint8_t *)&fw_attr->tick100ms);
	append_replay_item(&frame_pkg, FRAME_FRAME_STATUS, FRAME_FRAME_STATUS_SIZE, (uint8_t *)&fw_attr->skip_status);
	append_replay_item(&frame_pkg, FRAME_ATTR_BUF, sizeof(fw_attr->attr_buf), fw_attr->attr_buf);

	/*======log info========*/
	frame_ptr += ic_info->misc.fw_attr_len;
	fw_log = frame_ptr;
	append_replay_item(&frame_pkg, FRAME_FRAME_LOG, ic_info->misc.fw_log_len, fw_log);

	/*======frame body info========*/
	frame_ptr += ic_info->misc.fw_log_len;
	append_replay_item(&frame_pkg, FRAME_FRAME_INDEX, FRAME_FRAME_INDEX_SIZE, (uint8_t *)&frame_head->frame_index);
	append_replay_item(&frame_pkg, FRAME_SCAN_RATE, FRAME_SCAN_RATE_SIZE, (uint8_t *)&fw_attr->scan_rate);
	append_replay_item(&frame_pkg, FRAME_STATUS, FRAME_STATUS_SIZE, (uint8_t *)&fw_attr->status);

	if (frame_head->data_en & MUTUAL_TYPE_EN) {
		mutual_duration = LE16_TO_CPUP(frame_ptr);
		mutual_freq_a = LE16_TO_CPUP(frame_ptr + 2);
		mutual_freq_b = LE16_TO_CPUP(frame_ptr + 4);
		mutual_freq_a_index = frame_ptr[6];
		mutual_adc[0] = LE16_TO_CPUP(frame_ptr + (ic_info->misc.mutual_struct_len - 6));
		mutual_adc[1] = LE16_TO_CPUP(frame_ptr + (ic_info->misc.mutual_struct_len - 4));
		mutual_adc[2] = LE16_TO_CPUP(frame_ptr + (ic_info->misc.mutual_struct_len - 2));
		append_replay_item(&frame_pkg, FRAME_MUTUAL_DURATION, FRAME_MUTUAL_DURATION_SIZE, (uint8_t *)&mutual_duration);
		append_replay_item(&frame_pkg, FRAME_MUTUAL_FREQ_A, FRAME_MUTUAL_FREQ_A_SIZE, (uint8_t *)&mutual_freq_a);
		append_replay_item(&frame_pkg, FRAME_MUTUAL_FREQ_B, FRAME_MUTUAL_FREQ_B_SIZE, (uint8_t *)&mutual_freq_b);
		append_replay_item(&frame_pkg, FRAME_MUTUAL_FREQ_A_INDEX, 1, (uint8_t *)&mutual_freq_a_index);
		append_replay_item(&frame_pkg, FRAME_MUTUAL_ADC, FRAME_MUTUAL_ADC_SIZE * 3, (uint8_t *)&mutual_adc[0]);
		append_replay_item(&frame_pkg, FRAME_MUTUAL_DATA,
		ic_info->parm.drv_num * ic_info->parm.sen_num * FRAME_MUTUAL_DATA_SIZE, frame_ptr + 8);
		frame_ptr += ic_info->misc.mutual_struct_len;
	}

	if (frame_head->data_en & SELF_TYPE_EN) {
		self_tx_duration = LE16_TO_CPUP(frame_ptr);
		self_rx_duration = LE16_TO_CPUP(frame_ptr + 2);
		self_tx_freq = LE16_TO_CPUP(frame_ptr + 4);
		self_rx_freq = LE16_TO_CPUP(frame_ptr + 6);
		self_adc[0] = LE16_TO_CPUP(frame_ptr + (ic_info->misc.self_struct_len - 6));
		self_adc[1] = LE16_TO_CPUP(frame_ptr + (ic_info->misc.self_struct_len - 4));
		self_adc[2] = LE16_TO_CPUP(frame_ptr + (ic_info->misc.self_struct_len - 2));
		append_replay_item(&frame_pkg, FRAME_SELF_TX_DURATION, FRAME_SELF_TX_DURATION_SIZE, (uint8_t *)&self_tx_duration);
		append_replay_item(&frame_pkg, FRAME_SELF_RX_DURATION, FRAME_SELF_RX_DURATION_SIZE, (uint8_t *)&self_rx_duration);
		append_replay_item(&frame_pkg, FRAME_SELF_TX_FREQ, FRAME_SELF_TX_FREQ_SIZE, (uint8_t *)&self_tx_freq);
		append_replay_item(&frame_pkg, FRAME_SELF_RX_FREQ, FRAME_SELF_RX_FREQ_SIZE, (uint8_t *)&self_rx_freq);
		append_replay_item(&frame_pkg, FRAME_SELF_ADC, FRAME_SELF_ADC_SIZE * 3, (uint8_t *)&self_adc[0]);
		append_replay_item(&frame_pkg, FRAME_SELF_DATA,
		(ic_info->parm.sen_num + ic_info->parm.drv_num) * FRAME_SELF_DATA_SIZE, frame_ptr + 10);
		frame_ptr += ic_info->misc.self_struct_len;
	}

	memcpy((uint8_t *)max_com_noise, frame_ptr, sizeof(max_com_noise));
	append_replay_item(&frame_pkg, FRAME_MAX_COM_NOISE, sizeof(max_com_noise), (uint8_t *)max_com_noise);
	memcpy((uint8_t *)max_lcd_noise, frame_ptr + sizeof(max_com_noise), sizeof(max_lcd_noise));
	append_replay_item(&frame_pkg, FRAME_MAX_LCD_NOISE, sizeof(max_lcd_noise), (uint8_t *)max_lcd_noise);
	memcpy((uint8_t *)im_raw_data, frame_ptr + sizeof(max_com_noise) + sizeof(max_lcd_noise), ic_info->misc.im_rawdata_len);
	append_replay_item(&frame_pkg, FRAME_IM_DATA, ic_info->misc.im_rawdata_len, (uint8_t *)im_raw_data);
	memcpy((uint8_t *)im_adc, frame_ptr + sizeof(max_com_noise) + sizeof(max_lcd_noise) + ic_info->misc.im_rawdata_len, sizeof(im_adc));
	append_replay_item(&frame_pkg, FRAME_IM_ADC, sizeof(im_adc), (uint8_t *)im_adc);

	//end frame
	type_group = GROUP_FRAME;
	append_replay_item(&frame_pkg, MISC_GROUP_END, MISC_GROUP_END_SIZE, (uint8_t *)&type_group);
	type_group = GROUP_RUNNING;
	append_replay_item(&frame_pkg, MISC_GROUP_END, MISC_GROUP_END_SIZE, (uint8_t *)&type_group);

exit:
	mutex_unlock(&rep_mutex);
	sysfs_notify(parent, NULL, "replay_data");
	return 0;
}

static ssize_t enable_replay_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", (atomic_read(&cd->enable_replay)) ? "enable" : "disable");
}

static ssize_t enable_replay_type_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	u8 enable_cmd[] = {0x00, 0x00, 0x05, 0x90, 0x01, 0x96, 0x00};
	u8 disable_cmd[] = {0x00, 0x00, 0x05, 0x90, 0x00, 0x95, 0x00};

	store_rep_head = true;

	if (buf[0] == 0 || buf[0] == '0') {
		cd->hw_ops->write(cd, ic_info->misc.cmd_addr, disable_cmd, sizeof(disable_cmd));
		atomic_set(&cd->enable_replay, 0);
	} else {
		cd->hw_ops->write(cd, ic_info->misc.cmd_addr, enable_cmd, sizeof(enable_cmd));
		atomic_set(&cd->enable_replay, 1);
	}
	return count;
}

static ssize_t replay_data_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct replay_package *pkg;
	ssize_t len = 0;

	if (atomic_read(&cd->enable_replay) == 0)
	return 0;

	mutex_lock(&rep_mutex);
	pkg = get_packed_frame();
	if (pkg) {
		memcpy(buf, pkg->data, pkg->offset);
		len = pkg->offset;
		store_rep_head = false;
	}
	mutex_unlock(&rep_mutex);
	return len;
}

static DEVICE_ATTR(enable_replay, 0664, enable_replay_type_show, enable_replay_type_store);
static DEVICE_ATTR(replay_data, 0444, replay_data_type_show, NULL);

static struct attribute *replay_attrs[] = {
	&dev_attr_enable_replay.attr,
	&dev_attr_replay_data.attr,
	NULL,
};

const static struct attribute_group replay_sysfs_group = {
	.attrs = replay_attrs,
};

int replay_module_init(struct goodix_ts_core *core_data)
{
	int ret = 0;
	parent = &core_data->pdev->dev.kobj;

	ret = sysfs_create_group(parent, &replay_sysfs_group);
	if (ret) {
		ts_err("failed create replay sysfs files");
		goto err_out;
	}

	cd = core_data;
	ic_info = &core_data->ic_info;
	atomic_set(&core_data->enable_replay, 0);
	mutex_init(&rep_mutex);
	ts_info("replay module init success");
	return 0;

err_out:
	ts_err("replay module init failed!");
	return ret;
}

void replay_module_exit(void)
{
	ts_info("replay module exit");

	sysfs_remove_group(&cd->pdev->dev.kobj, &replay_sysfs_group);
	mutex_destroy(&rep_mutex);
}