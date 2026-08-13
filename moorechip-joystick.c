#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <uapi/linux/sched/types.h>
#include <linux/firmware.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/workqueue.h>
#include <drm/drm_panel.h>

#define USB_VENDOR_ID_MICROSOFT 0x045e
#define USB_DEVICE_ID_MICROSOFT_XBOX_360_PAD 0x028e

#define MOORECHIP_MAX_STICK_MAG		32767
#define MOORECHIP_MAX_TRIGGER_MAG	255
#define MOORECHIP_STICK_FUZZ		250
#define MOORECHIP_STICK_FLAT		500

#define MOORECHIP_MAGIC 0x3D5AD3A5

enum moorechip_type {
	MOORECHIP_TYPE_CMD = 1,
	MOORECHIP_TYPE_STATUS,
	MOORECHIP_TYPE_UPGRADE_DATA,
};

enum moorechip_cmd {
	MOORECHIP_CMD_NOP = 1, // returns 1
	MOORECHIP_CMD_GET_VERSION,

	// 3 and 4 are not supported in the firmware
	MOORECHIP_CMD_SET_PARAMETER = 5,
	MOORECHIP_CMD_GET_APP_SIZE,
	MOORECHIP_CMD_SET_PROCESSING_STATE,
	MOORECHIP_CMD_TEST_DATA,
	MOORECHIP_CMD_SET_TRANSMIT_STATE,

	// TODO: add calibration commands starting at 0xA0
	MOORECHIP_CMD_SET_LEFT_STICK_AXIS_SWAP = 0xB0,

	MOORECHIP_CMD_UPGRADE_START = 0xE9,
	MOORECHIP_CMD_UPGRADE_STATE_REPORT,
	MOORECHIP_CMD_UPGRADE_SET_PARAMETER,
	MOORECHIP_CMD_UPGRADE_SUCCESS,
	MOORECHIP_CMD_UPGRADE_SAVE_LAST_FRAME,
	MOORECHIP_CMD_UPGRADE_SET_APP_FLAG
};

struct moorechip_header {
	u32 magic;
	u8 seq;
	u8 type;
	u16 datalen;
} __packed;

struct moorechip_command {
	u8 cmd;
} __packed;

struct moorechip_parameter {
	u8 cmd;
	u8 data1;
	u32 debounce;
	u32 frame_rate;
} __packed;

struct moorechip_enable {
	u8 cmd;
	u8 enable;
} __packed;

struct moorechip_upgrade_parameter {
	u8 cmd;
	u32 size;
	u32 crc;
} __packed;

enum moorechip_keys {
	MOORECHIP_BTN_UP = (1 << 0),
	MOORECHIP_BTN_DOWN = (1 << 1),
	MOORECHIP_BTN_LEFT = (1 << 2),
	MOORECHIP_BTN_RIGHT = (1 << 3),
	MOORECHIP_BTN_X = (1 << 4),
	MOORECHIP_BTN_Y = (1 << 5),
	MOORECHIP_BTN_A = (1 << 6),
	MOORECHIP_BTN_B = (1 << 7),
	MOORECHIP_BTN_TL = (1 << 8),
	MOORECHIP_BTN_TR = (1 << 9),
	MOORECHIP_BTN_SELECT = (1 << 10),
	MOORECHIP_BTN_START = (1 << 11),
	MOORECHIP_BTN_THUMBL = (1 << 12),
	MOORECHIP_BTN_THUMBR = (1 << 13),
	MOORECHIP_BTN_HOME = (1 << 14),
	MOORECHIP_BTN_BACK = (1 << 15)
};

enum moorechip_ignore_extra {
	MOORECHIP_IGNORE_LEFT_TRIGGER = (1 << 16),
	MOORECHIP_IGNORE_RIGHT_TRIGGER = (1 << 17),
	MOORECHIP_IGNORE_LEFT_STICK = (1 << 18),
	MOORECHIP_IGNORE_RIGHT_STICK = (1 << 19),
};

enum moorechip_trigger_mode {
	MOORECHIP_TRIGGER_MODE_DIGITAL = (1 << 0),
	MOORECHIP_TRIGGER_MODE_ANALOG = (1 << 1),
	MOORECHIP_TRIGGER_MODE_BOTH = MOORECHIP_TRIGGER_MODE_DIGITAL | MOORECHIP_TRIGGER_MODE_ANALOG
};

struct moorechip_key_data {
	u16 keys;
	u16 left_trigger;
	u16 right_trigger;
	s16 left_joystick_x;
	s16 left_joystick_y;
	s16 right_joystick_x;
	s16 right_joystick_y;
} __packed;

struct moorechip_abs_calib {
	int min;
	int max;
	int center;
	int deadzone;
};

struct moorechip_stick_calib {
	struct moorechip_abs_calib x;
	struct moorechip_abs_calib y;
};

struct moorechip_trigger_calib {
	int min;
	int max;
};

struct moorechip_driver {
	struct serdev_device *serdev;
	struct input_dev *input;
	struct regulator *vdd_reg;
	struct regulator *levelshifter_reg;
	int boot_gpio;
	int reset_gpio;
	bool layout_xbox;
	enum moorechip_trigger_mode trigger_mode;
	u8 seq;
	struct moorechip_key_data last_keys;
	struct moorechip_stick_calib calib_stick_left;
	struct moorechip_stick_calib calib_stick_right;
	struct moorechip_trigger_calib calib_trigger_left;
	struct moorechip_trigger_calib calib_trigger_right;
	struct class *class;
	u32 ignore_mask;
	char firmware_version[58];
	const struct firmware *fw;
	u32 fw_sent;
	struct gpio_desc *m0_key;
	int m0_irq;
	u32 m0_code;
	struct gpio_desc *m1_key;
	int m1_irq;
	u32 m1_code;
	struct mutex lock;
	void *notifier_cookie;
	void *notifier_cookie_sec;
	bool active;
	bool panel_on[2];
	bool system_suspended;
	bool fw_recheck;
	bool left_stick_axis_swap;
	struct work_struct power_work;
};

static int moorechip_send_cmd(struct moorechip_driver *moorechip, u8 type, const void *data, u16 datalen)
{
	struct device *dev = &moorechip->serdev->dev;
	int packetlen = sizeof(struct moorechip_header) + datalen + 1;
	struct moorechip_header *hdr;
	u8 *buf;
	u8 checksum = 0;
	int rc, i;

	if (!READ_ONCE(moorechip->active))
		return -EHOSTDOWN;

	buf = kzalloc(packetlen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	hdr = (struct moorechip_header *)buf;

	hdr->magic = MOORECHIP_MAGIC;
	hdr->seq = ++moorechip->seq;
	hdr->type = type;
	hdr->datalen = datalen;

	memcpy(&buf[sizeof(struct moorechip_header)], data, datalen);

	for (i = 4; i < packetlen - 1; i++)
		checksum ^= buf[i];

	buf[packetlen - 1] = checksum;

	rc = serdev_device_write_buf(moorechip->serdev, buf, packetlen);
	if (rc < 0)
		dev_err(dev, "Failed to send serial data; rc=%d\n", rc);
	else
		rc = 0;

	kfree(buf);

	return rc;
}

static int moorechip_nop(struct moorechip_driver *moorechip)
{
	struct moorechip_command cmd = {
		.cmd = MOORECHIP_CMD_NOP
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &cmd, sizeof(cmd));
}

static int moorechip_get_firmware_version(struct moorechip_driver *moorechip)
{
	struct moorechip_command cmd = {
		.cmd = MOORECHIP_CMD_GET_VERSION
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &cmd, sizeof(cmd));
}

static int moorechip_set_parameter(struct moorechip_driver *moorechip, u32 debounce, u32 frame_rate)
{
	struct moorechip_parameter param = {
		.cmd = MOORECHIP_CMD_SET_PARAMETER,
		.data1 = 1, // must be 1, hardcoded in the firmware
		.debounce = __cpu_to_be32(debounce),
		.frame_rate = __cpu_to_be32(frame_rate)
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &param, sizeof(param));
}

static int moorechip_set_input_processing_enabled(struct moorechip_driver *moorechip, bool enable)
{
	struct moorechip_enable en = {
		.cmd = MOORECHIP_CMD_SET_PROCESSING_STATE,
		.enable = enable ? 1 : 2
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &en, sizeof(en));
}

static int moorechip_set_input_transmit_enabled(struct moorechip_driver *moorechip, bool enable)
{
	struct moorechip_enable en = {
		.cmd = MOORECHIP_CMD_SET_TRANSMIT_STATE,
		.enable = enable ? 2 : 1
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &en, sizeof(en));
}

static int moorechip_set_left_stick_axis_swap(struct moorechip_driver *moorechip, bool enable)
{
	struct moorechip_enable en = {
		.cmd = MOORECHIP_CMD_SET_LEFT_STICK_AXIS_SWAP,
		.enable = enable ? 3 : 0
	};

	moorechip->left_stick_axis_swap = enable;

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &en, sizeof(en));
}

static int moorechip_upgrade_start(struct moorechip_driver *moorechip)
{
	struct moorechip_command cmd = {
		.cmd = MOORECHIP_CMD_UPGRADE_START
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &cmd, sizeof(cmd));
}

static int moorechip_upgrade_set_parameter(struct moorechip_driver *moorechip, u32 size)
{
	struct moorechip_upgrade_parameter param = {
		.cmd = MOORECHIP_CMD_UPGRADE_SET_PARAMETER,
		.size = __cpu_to_be32(size),
		.crc = __cpu_to_be32(0)};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &param, sizeof(param));
}

static int moorechip_upgrade_send_chunk(struct moorechip_driver *moorechip, const u8 *data, u32 len)
{
	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_UPGRADE_DATA, data, len);
}

static int moorechip_upgrade_save_last_frame(struct moorechip_driver *moorechip)
{
	struct moorechip_command cmd = {
		.cmd = MOORECHIP_CMD_UPGRADE_SAVE_LAST_FRAME
	};

	return moorechip_send_cmd(moorechip, MOORECHIP_TYPE_CMD, &cmd, sizeof(cmd));
}

static int moorechip_upgrade_done(struct moorechip_driver *moorechip)
{
	int ret;

	ret = moorechip_set_input_transmit_enabled(moorechip, true);
	if (ret)
		return ret;
	msleep(100);

	ret = moorechip_set_parameter(moorechip, 40, 7);
	if (ret)
		return ret;
	msleep(100);

	ret = moorechip_set_input_processing_enabled(moorechip, true);
	if (!ret)
		WRITE_ONCE(moorechip->fw_recheck, false);

	return ret;
}

static s32 moorechip_map_stick_val(struct moorechip_abs_calib *cal, s32 val)
{
	s32 center = cal->center;
	s32 min = cal->min;
	s32 max = cal->max;
	s32 dz = cal->deadzone;
	s32 new_val;

	if (dz < 0 || min >= center || max <= center)
		return 0;

	if (center - min <= dz || max - center <= dz)
		return 0;

	if (abs(val - center) <= dz)
		return 0;

	if (val > center) {
		new_val = (val - center - dz) * -MOORECHIP_MAX_STICK_MAG;
		new_val /= (max - center - dz);
	} else {
		new_val = (center - val - dz) * MOORECHIP_MAX_STICK_MAG;
		new_val /= (center - min - dz);
	}
	new_val = clamp(new_val, (s32)-MOORECHIP_MAX_STICK_MAG, (s32)MOORECHIP_MAX_STICK_MAG);
	return new_val;
}

static s32 moorechip_map_trigger_val(struct moorechip_trigger_calib *cal, s32 val)
{
	s32 min = cal->min;
	s32 max = cal->max;

	if (max <= min)
		return 0;

	val = clamp(val, min, max);

	return (max - val) * MOORECHIP_MAX_TRIGGER_MAG / (max - min);
}

static int moorechip_joystick_receive_buf(struct serdev_device *serdev,
				     const unsigned char *buf, size_t len)
{
	struct moorechip_driver *moorechip = serdev_device_get_drvdata(serdev);
	struct device *dev = &moorechip->serdev->dev;
	struct moorechip_header *hdr = (struct moorechip_header *) buf;
	u8 checksum = 0;
	int packetlen, i;

	if (!READ_ONCE(moorechip->active))
		return len;

	if (len < sizeof(struct moorechip_header) + 1) {
		dev_dbg(dev, "Too small packet!\n");
		return 0;
	}

	if (hdr->magic != MOORECHIP_MAGIC) {
		dev_err(dev, "Invalid magic!\n");
		return sizeof(hdr->magic); // return sizeof(hdr->magic) here, to flush out the buffer
	}

	packetlen = sizeof(struct moorechip_header) + hdr->datalen + 1;

	if (len < packetlen) {
		dev_dbg(dev, "packet len mismatch!\n");
		return 0;
	}

	for (i = 4; i < packetlen - 1; i++)
		checksum ^= buf[i];

	if (buf[packetlen - 1] != checksum) {
		dev_err(dev, "Invalid checksum!\n");
		return packetlen; // return packetlen here, to flush out the buffer
	}

	switch (hdr->type) {
		case MOORECHIP_TYPE_CMD:
		{
			const u8 *data = &buf[sizeof(struct moorechip_header)];
			switch (data[0]) {
				case MOORECHIP_CMD_NOP:
				{
					if (hdr->datalen != 1)
						dev_err(dev, "Unexpected nop response!\n");

					// We sould only receive a NOP when the fw upgrade is done
					if (moorechip->reset_gpio >= 0) {
						gpio_direction_output(moorechip->reset_gpio, 0);
						msleep(100);
						gpio_direction_output(moorechip->reset_gpio, 1);
						msleep(100);
					} else {
						int rc;

						regulator_disable(moorechip->vdd_reg);
						msleep(100);

						rc = regulator_enable(moorechip->vdd_reg);
						if (rc)
						{
							dev_err(dev, "Failed to enable joystick vdd regulator\n");
							return rc;
						}
						msleep(100);
					}

					moorechip_upgrade_done(moorechip);
					break;
				}
				case MOORECHIP_CMD_GET_VERSION:
				{
					int rc;
					const struct firmware *ver;
					if (hdr->datalen > sizeof(moorechip->firmware_version)) {
						dev_err(dev, "Unexpected version response!\n");
						break;
					}
					memset(moorechip->firmware_version, 0, sizeof(moorechip->firmware_version));
					memcpy(moorechip->firmware_version, &data[1], hdr->datalen - 1);
					dev_info(dev, "Firmware version: %s\n", moorechip->firmware_version);

					// check if we are in bootloader
					if (moorechip->firmware_version[0] != 'B') {
						rc = request_firmware(&ver, "mcuapp_firmware.txt", dev);
						if (rc < 0) {
							dev_err(dev, "Failed to get firmware version: %d\n", rc);
							moorechip_upgrade_done(moorechip);
							break;
						}

						if (!strncmp(ver->data, &moorechip->firmware_version[4], ver->size)) {
							dev_dbg(dev, "Version matches.\n");
							release_firmware(ver);
							moorechip_upgrade_done(moorechip);
							break;
						}
						release_firmware(ver);

						dev_info(dev, "Version mismatch. Upgrading the firmware.\n");
					}

					moorechip_upgrade_start(moorechip);

					rc = request_firmware(&moorechip->fw, "mcuapp_firmware.bin", dev);
					if (rc < 0) {
						dev_err(dev, "Failed to get firmware: %d\n", rc);
						moorechip_upgrade_done(moorechip);
						break;
					}
					moorechip->fw_sent = 0;

					break;
				}
				case MOORECHIP_CMD_SET_PARAMETER:
				{
					if (hdr->datalen != 1) {
						dev_err(dev, "Unexpected set parameter response!\n");
						break;
					}
					dev_dbg(dev, "ACK: set parameter\n");
					break;
				}
				case MOORECHIP_CMD_GET_APP_SIZE:
				{
					if (hdr->datalen != 3) {
						dev_err(dev, "Unexpected get app size response!\n");
						break;
					}
					dev_dbg(dev, "APP SIZE: 0x%x\n", *(u16 *)&data[1]);
					break;
				}
				case MOORECHIP_CMD_SET_PROCESSING_STATE:
				{
					dev_dbg(dev, "ACK: set parameter\n");
					break;
				}
				case MOORECHIP_CMD_TEST_DATA:
				{
					dev_dbg(dev, "test data response\n");
					break;
				}
				case MOORECHIP_CMD_SET_TRANSMIT_STATE:
				{
					dev_dbg(dev, "ACK: set parameter\n");
					break;
				}
				case MOORECHIP_CMD_UPGRADE_START:
				{
					dev_dbg(dev, "ACK: upgrade start\n");
					break;
				}
				case MOORECHIP_CMD_UPGRADE_STATE_REPORT:
				{
					dev_dbg(dev, "ACK: upgrade state report\n");
					if (moorechip->fw)
						moorechip_upgrade_set_parameter(moorechip, moorechip->fw->size);
					break;
				}
				case MOORECHIP_CMD_UPGRADE_SET_PARAMETER:
				{
					dev_dbg(dev, "ACK: upgrade set parameter\n");
					fallthrough;
				}
				case MOORECHIP_CMD_UPGRADE_SUCCESS:
				{
					dev_dbg(dev, "ACK: upgrade success\n");

					if (moorechip->fw) {
						if (moorechip->fw_sent != moorechip->fw->size) {
							int len = moorechip->fw->size - moorechip->fw_sent;
							if (len > 256)
								len = 256;

							moorechip_upgrade_send_chunk(moorechip, &moorechip->fw->data[moorechip->fw_sent], len);
							moorechip->fw_sent += len;
						} else {
							release_firmware(moorechip->fw);
							moorechip->fw = NULL;
							moorechip_upgrade_save_last_frame(moorechip);
						}
					}
					break;
				}
				case MOORECHIP_CMD_UPGRADE_SAVE_LAST_FRAME:
				{
					dev_dbg(dev, "ACK: upgrade save last frame\n");
					break;
				}
				case MOORECHIP_CMD_UPGRADE_SET_APP_FLAG:
				{
					dev_dbg(dev, "ACK: upgrade set app flag\n");

					moorechip_nop(moorechip);
					break;
				}
			}
			break;
		}
		case MOORECHIP_TYPE_STATUS:
		{
			struct moorechip_key_data *last_keys = &moorechip->last_keys;
			const struct moorechip_key_data *key_data = (const struct moorechip_key_data *) &buf[sizeof(struct moorechip_header)];
			u16 changed_keys;
			u16 keys;

			if (hdr->datalen != sizeof(*key_data)) {
				dev_err(dev, "Unexpected packet length for status!\n");
				return packetlen; // return packetlen here, to flush out the buffer
			}

			keys = key_data->keys;
			changed_keys = (last_keys->keys ^ keys) & ~moorechip->ignore_mask;

			if (changed_keys & MOORECHIP_BTN_UP)
				input_report_abs(moorechip->input, ABS_HAT0Y, -(!!(keys & MOORECHIP_BTN_UP)));
			if (changed_keys & MOORECHIP_BTN_DOWN)
				input_report_abs(moorechip->input, ABS_HAT0Y, !!(keys & MOORECHIP_BTN_DOWN));
			if (changed_keys & MOORECHIP_BTN_LEFT)
				input_report_abs(moorechip->input, ABS_HAT0X, -(!!(keys & MOORECHIP_BTN_LEFT)));
			if (changed_keys & MOORECHIP_BTN_RIGHT)
				input_report_abs(moorechip->input, ABS_HAT0X, !!(keys & MOORECHIP_BTN_RIGHT));
			if (changed_keys & MOORECHIP_BTN_X)
				input_report_key(moorechip->input, moorechip->layout_xbox ? BTN_WEST : BTN_NORTH, !!(keys & MOORECHIP_BTN_X));
			if (changed_keys & MOORECHIP_BTN_Y)
				input_report_key(moorechip->input, moorechip->layout_xbox ? BTN_NORTH : BTN_WEST, !!(keys & MOORECHIP_BTN_Y));
			if (changed_keys & MOORECHIP_BTN_A)
				input_report_key(moorechip->input, moorechip->layout_xbox ? BTN_EAST: BTN_SOUTH, !!(keys & MOORECHIP_BTN_A));
			if (changed_keys & MOORECHIP_BTN_B)
				input_report_key(moorechip->input, moorechip->layout_xbox ? BTN_SOUTH: BTN_EAST, !!(keys & MOORECHIP_BTN_B));
			if (changed_keys & MOORECHIP_BTN_TL)
				input_report_key(moorechip->input, BTN_TL, !!(keys & MOORECHIP_BTN_TL));
			if (changed_keys & MOORECHIP_BTN_TR)
				input_report_key(moorechip->input, BTN_TR, !!(keys & MOORECHIP_BTN_TR));
			if (changed_keys & MOORECHIP_BTN_SELECT)
				input_report_key(moorechip->input, BTN_SELECT, !!(keys & MOORECHIP_BTN_SELECT));
			if (changed_keys & MOORECHIP_BTN_START)
				input_report_key(moorechip->input, BTN_START, !!(keys & MOORECHIP_BTN_START));
			if (changed_keys & MOORECHIP_BTN_THUMBL)
				input_report_key(moorechip->input, BTN_THUMBL, !!(keys & MOORECHIP_BTN_THUMBL));
			if (changed_keys & MOORECHIP_BTN_THUMBR)
				input_report_key(moorechip->input, BTN_THUMBR, !!(keys & MOORECHIP_BTN_THUMBR));
			if (changed_keys & MOORECHIP_BTN_HOME)
				input_report_key(moorechip->input, KEY_HOME, !!(keys & MOORECHIP_BTN_HOME));
			if (changed_keys & MOORECHIP_BTN_BACK)
				input_report_key(moorechip->input, KEY_BACK, !!(keys & MOORECHIP_BTN_BACK));

			if (!(moorechip->ignore_mask & MOORECHIP_IGNORE_LEFT_TRIGGER)) {
				if (moorechip->trigger_mode & MOORECHIP_TRIGGER_MODE_DIGITAL) {
					bool last_active = moorechip_map_trigger_val(&moorechip->calib_trigger_left, last_keys->left_trigger) >
						MOORECHIP_MAX_TRIGGER_MAG / 2;
					bool now_active = moorechip_map_trigger_val(&moorechip->calib_trigger_left, key_data->left_trigger) >
						MOORECHIP_MAX_TRIGGER_MAG / 2;
					if (last_active != now_active)
						input_report_key(moorechip->input, BTN_TL2, now_active);
				}
				if (moorechip->trigger_mode & MOORECHIP_TRIGGER_MODE_ANALOG) {
					if (last_keys->left_trigger != key_data->left_trigger)
						input_report_abs(moorechip->input, ABS_Z,
							moorechip_map_trigger_val(&moorechip->calib_trigger_left, key_data->left_trigger));
				}
			}

			if (!(moorechip->ignore_mask & MOORECHIP_IGNORE_RIGHT_TRIGGER)) {
				if (moorechip->trigger_mode & MOORECHIP_TRIGGER_MODE_DIGITAL) {
					bool last_active = moorechip_map_trigger_val(&moorechip->calib_trigger_right, last_keys->right_trigger) >
						MOORECHIP_MAX_TRIGGER_MAG / 2;
					bool now_active = moorechip_map_trigger_val(&moorechip->calib_trigger_right, key_data->right_trigger) >
						MOORECHIP_MAX_TRIGGER_MAG / 2;
					if (last_active != now_active)
						input_report_key(moorechip->input, BTN_TR2, now_active);
				}
				if (moorechip->trigger_mode & MOORECHIP_TRIGGER_MODE_ANALOG) {
					if (last_keys->right_trigger != key_data->right_trigger)
						input_report_abs(moorechip->input, ABS_RZ,
							moorechip_map_trigger_val(&moorechip->calib_trigger_right, key_data->right_trigger));
				}
			}

			if (!(moorechip->ignore_mask & MOORECHIP_IGNORE_LEFT_STICK)) {
				if (last_keys->left_joystick_x != key_data->left_joystick_x)
					input_report_abs(moorechip->input, ABS_X, moorechip_map_stick_val(&moorechip->calib_stick_left.x, key_data->left_joystick_x));
				if (last_keys->left_joystick_y != key_data->left_joystick_y)
					input_report_abs(moorechip->input, ABS_Y, moorechip_map_stick_val(&moorechip->calib_stick_left.y, key_data->left_joystick_y));
			}

			if (!(moorechip->ignore_mask & MOORECHIP_IGNORE_RIGHT_STICK)) {
				if (last_keys->right_joystick_x != key_data->right_joystick_x)
					input_report_abs(moorechip->input, ABS_RX, moorechip_map_stick_val(&moorechip->calib_stick_right.x, key_data->right_joystick_x));
				if (last_keys->right_joystick_y != key_data->right_joystick_y)
					input_report_abs(moorechip->input, ABS_RY, moorechip_map_stick_val(&moorechip->calib_stick_right.y, key_data->right_joystick_y));
			}

			input_sync(moorechip->input);

			memcpy(last_keys, key_data, sizeof(*key_data));
		}
	}

	return packetlen;
}

static irqreturn_t moorechip_gpio_isr(int irq, void *dev_id)
{
	struct moorechip_driver *moorechip = dev_id;
	struct gpio_desc *key;
	u32 code;

	if (!moorechip->input)
		return IRQ_HANDLED;

	if (irq == moorechip->m0_irq) {
		code = moorechip->m0_code;
		key = moorechip->m0_key;
	} else {
		code = moorechip->m1_code;
		key = moorechip->m1_key;
	}

	if (code == KEY_RESERVED)
		return IRQ_HANDLED;

	if (code == BTN_WEST)
		code = moorechip->layout_xbox ? BTN_WEST : BTN_NORTH;
	else if (code == BTN_NORTH)
		code = moorechip->layout_xbox ? BTN_NORTH : BTN_WEST;
	else if (code == BTN_SOUTH)
		code = moorechip->layout_xbox ? BTN_EAST : BTN_SOUTH;
	else if (code == BTN_EAST)
		code = moorechip->layout_xbox ? BTN_SOUTH : BTN_EAST;

	switch (code) {
	case BTN_DPAD_LEFT:
		input_report_abs(moorechip->input, ABS_HAT0X, -gpiod_get_value_cansleep(key));
		break;

	case BTN_DPAD_RIGHT:
		input_report_abs(moorechip->input, ABS_HAT0X, gpiod_get_value_cansleep(key));
		break;

	case BTN_DPAD_UP:
		input_report_abs(moorechip->input, ABS_HAT0Y, -gpiod_get_value_cansleep(key));
		break;

	case BTN_DPAD_DOWN:
		input_report_abs(moorechip->input, ABS_HAT0Y, gpiod_get_value_cansleep(key));
		break;

	default:
		input_report_key(moorechip->input, code, gpiod_get_value_cansleep(key));
	}

	input_sync(moorechip->input);

	return IRQ_HANDLED;
}

static struct serdev_device_ops moorechip_joystick_ops = {
	.receive_buf = moorechip_joystick_receive_buf,
};

static int moorechip_joystick_register_input(struct moorechip_driver *moorechip)
{
	moorechip->input = devm_input_allocate_device(&moorechip->serdev->dev);
	if (!moorechip->input)
		return -ENOMEM;
	moorechip->input->id.bustype = BUS_VIRTUAL;
	moorechip->input->id.vendor = USB_VENDOR_ID_MICROSOFT;
	moorechip->input->id.product = USB_DEVICE_ID_MICROSOFT_XBOX_360_PAD;
	moorechip->input->id.version = 0x1234;
	moorechip->input->name = "Xbox Wireless Controller";
	input_set_drvdata(moorechip->input, moorechip);

	input_set_abs_params(moorechip->input, ABS_HAT0X, -1, 1, 0, 0);
	input_set_abs_params(moorechip->input, ABS_HAT0Y, -1, 1, 0, 0);
	input_set_capability(moorechip->input, EV_KEY, BTN_NORTH);
	input_set_capability(moorechip->input, EV_KEY, BTN_WEST);
	input_set_capability(moorechip->input, EV_KEY, BTN_EAST);
	input_set_capability(moorechip->input, EV_KEY, BTN_SOUTH);
	input_set_capability(moorechip->input, EV_KEY, BTN_TL);
	input_set_capability(moorechip->input, EV_KEY, BTN_TR);
	input_set_capability(moorechip->input, EV_KEY, BTN_SELECT);
	input_set_capability(moorechip->input, EV_KEY, BTN_START);
	input_set_capability(moorechip->input, EV_KEY, BTN_THUMBL);
	input_set_capability(moorechip->input, EV_KEY, BTN_THUMBR);
	input_set_capability(moorechip->input, EV_KEY, KEY_HOME);
	input_set_capability(moorechip->input, EV_KEY, KEY_BACK);
	input_set_capability(moorechip->input, EV_KEY, BTN_TL2);
	input_set_capability(moorechip->input, EV_KEY, BTN_TR2);

	// Additional mappable options
	input_set_capability(moorechip->input, EV_KEY, BTN_C);
	input_set_capability(moorechip->input, EV_KEY, BTN_Z);
	input_set_capability(moorechip->input, EV_KEY, BTN_MODE);
	input_set_capability(moorechip->input, EV_KEY, KEY_ASSISTANT);
	input_set_capability(moorechip->input, EV_KEY, KEY_APPSELECT);
	input_set_capability(moorechip->input, EV_KEY, BTN_DND);
	input_set_capability(moorechip->input, EV_KEY, KEY_BRIGHTNESSDOWN);
	input_set_capability(moorechip->input, EV_KEY, KEY_BRIGHTNESSUP);
	input_set_capability(moorechip->input, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(moorechip->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(moorechip->input, EV_KEY, KEY_SYSRQ);
	input_set_capability(moorechip->input, EV_KEY, KEY_POWER);

	if (moorechip->trigger_mode & MOORECHIP_TRIGGER_MODE_ANALOG) {
		input_set_abs_params(moorechip->input, ABS_Z,
				     0, MOORECHIP_MAX_TRIGGER_MAG, 0, 0);
		input_set_abs_params(moorechip->input, ABS_RZ,
				     0, MOORECHIP_MAX_TRIGGER_MAG, 0, 0);
	}

	input_set_abs_params(moorechip->input, ABS_X,
			     -MOORECHIP_MAX_STICK_MAG, MOORECHIP_MAX_STICK_MAG,
			     MOORECHIP_STICK_FUZZ, MOORECHIP_STICK_FLAT);
	input_set_abs_params(moorechip->input, ABS_Y,
			     -MOORECHIP_MAX_STICK_MAG, MOORECHIP_MAX_STICK_MAG,
			     MOORECHIP_STICK_FUZZ, MOORECHIP_STICK_FLAT);

	input_set_abs_params(moorechip->input, ABS_RX,
			     -MOORECHIP_MAX_STICK_MAG, MOORECHIP_MAX_STICK_MAG,
			     MOORECHIP_STICK_FUZZ, MOORECHIP_STICK_FLAT);
	input_set_abs_params(moorechip->input, ABS_RY,
			     -MOORECHIP_MAX_STICK_MAG, MOORECHIP_MAX_STICK_MAG,
			     MOORECHIP_STICK_FUZZ, MOORECHIP_STICK_FLAT);

	return input_register_device(moorechip->input);
}

static void moorechip_joystick_power_off_locked(
		struct moorechip_driver *moorechip)
{
	struct device *dev = &moorechip->serdev->dev;
	int ret;

	if (!moorechip->active)
		return;

	WRITE_ONCE(moorechip->active, false);
	serdev_device_close(moorechip->serdev);
	if (moorechip->reset_gpio >= 0)
		gpio_direction_output(moorechip->reset_gpio, 0);
	if (moorechip->boot_gpio >= 0)
		gpio_direction_output(moorechip->boot_gpio, 0);

	if (moorechip->fw) {
		release_firmware(moorechip->fw);
		moorechip->fw = NULL;
		moorechip->fw_sent = 0;
		WRITE_ONCE(moorechip->fw_recheck, true);
	}

	ret = regulator_disable(moorechip->levelshifter_reg);
	if (ret)
		dev_warn(dev,
			 "Failed to disable joystick levelshifter regulator: %d\n",
			 ret);
	ret = regulator_disable(moorechip->vdd_reg);
	if (ret)
		dev_warn(dev, "Failed to disable joystick vdd regulator: %d\n",
			 ret);
}

static int moorechip_joystick_power_on_locked(
		struct moorechip_driver *moorechip)
{
	struct device *dev = &moorechip->serdev->dev;
	int ret;

	if (moorechip->active ||
	    (!moorechip->panel_on[0] && !moorechip->panel_on[1]) ||
	    moorechip->system_suspended)
		return 0;

	if (moorechip->boot_gpio >= 0)
		gpio_direction_output(moorechip->boot_gpio, 0);
	if (moorechip->reset_gpio >= 0)
		gpio_direction_output(moorechip->reset_gpio, 0);

	ret = regulator_enable(moorechip->vdd_reg);
	if (ret) {
		dev_err(dev, "Failed to enable joystick vdd regulator: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(moorechip->levelshifter_reg);
	if (ret) {
		dev_err(dev,
			"Failed to enable joystick levelshifter regulator: %d\n",
			ret);
		goto err_disable_vdd;
	}

	ret = serdev_device_open(moorechip->serdev);
	if (ret) {
		dev_err(dev, "Failed to open serdev device: %d\n", ret);
		goto err_disable_levelshifter;
	}
	serdev_device_set_flow_control(moorechip->serdev, false);
	serdev_device_set_baudrate(moorechip->serdev, 115200);
	WRITE_ONCE(moorechip->active, true);

	msleep(40);
	if (moorechip->reset_gpio >= 0) {
		gpio_direction_output(moorechip->reset_gpio, 1);
		msleep(100);
	}

	ret = moorechip_set_input_transmit_enabled(moorechip, true);
	if (ret)
		goto err_close;
	msleep(100);

	if (READ_ONCE(moorechip->fw_recheck)) {
		ret = moorechip_get_firmware_version(moorechip);
	} else {
		ret = moorechip_set_parameter(moorechip, 40, 7);
		if (!ret) {
			msleep(100);
			ret = moorechip_set_input_processing_enabled(moorechip,
							     true);
		}
	}
	if (ret)
		goto err_close;

	if (moorechip->left_stick_axis_swap) {
		moorechip_set_left_stick_axis_swap(moorechip, true);
	}

	return 0;

err_close:
	WRITE_ONCE(moorechip->active, false);
	serdev_device_close(moorechip->serdev);
	if (moorechip->reset_gpio >= 0)
		gpio_direction_output(moorechip->reset_gpio, 0);
err_disable_levelshifter:
	regulator_disable(moorechip->levelshifter_reg);
err_disable_vdd:
	regulator_disable(moorechip->vdd_reg);
	return ret;
}

static void moorechip_power_work(struct work_struct *work)
{
	struct moorechip_driver *moorechip =
		container_of(work, struct moorechip_driver, power_work);
	int ret;

	mutex_lock(&moorechip->lock);
	ret = moorechip_joystick_power_on_locked(moorechip);
	if (ret)
		dev_err(&moorechip->serdev->dev,
			"Failed to resume joystick: %d\n", ret);
	mutex_unlock(&moorechip->lock);
}

static void moorechip_panel_event_notifier_callback(
		enum panel_event_notifier_tag tag,
		struct panel_event_notification *notification, void *data)
{
	struct moorechip_driver *moorechip = data;
	int panel_idx;

	if (!notification)
		return;

	switch (tag) {
	case PANEL_EVENT_NOTIFICATION_PRIMARY:
		panel_idx = 0;
		break;
	case PANEL_EVENT_NOTIFICATION_SECONDARY:
		panel_idx = 1;
		break;
	default:
		return;
	}

	mutex_lock(&moorechip->lock);
	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
	case DRM_PANEL_EVENT_BLANK_LP:
		if (!notification->notif_data.early_trigger)
			break;
		moorechip->panel_on[panel_idx] = false;
		if (!moorechip->panel_on[0] && !moorechip->panel_on[1]) {
			cancel_work(&moorechip->power_work);
			moorechip_joystick_power_off_locked(moorechip);
		}
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		if (notification->notif_data.early_trigger)
			break;
		moorechip->panel_on[panel_idx] = true;
		schedule_work(&moorechip->power_work);
		break;
	default:
		break;
	}
	mutex_unlock(&moorechip->lock);
}

static int moorechip_register_panel_notifier_one(
		struct moorechip_driver *moorechip, const char *property,
		enum panel_event_notifier_tag tag,
		enum panel_event_notifier_client client, void **cookie,
		bool required)
{
	struct device *dev = &moorechip->serdev->dev;
	struct device_node *panel_node;
	struct drm_panel *panel;
	int ret;

	panel_node = of_parse_phandle(dev->of_node, property, 0);
	if (!panel_node) {
		if (!required)
			return 0;
		return dev_err_probe(dev, -ENODEV, "Missing %s phandle\n",
					     property);
	}

	panel = of_drm_find_panel(panel_node);
	of_node_put(panel_node);
	if (IS_ERR(panel))
		return dev_err_probe(dev, PTR_ERR(panel),
				     "Failed to find joystick %s\n", property);

	*cookie = panel_event_notifier_register(
		tag, client, panel,
		moorechip_panel_event_notifier_callback, moorechip);
	if (IS_ERR(*cookie)) {
		ret = PTR_ERR(*cookie);
		*cookie = NULL;
		return dev_err_probe(dev, ret,
				     "Failed to register %s notifier\n", property);
	}

	return 0;
}

static void moorechip_unregister_panel_notifiers(
		struct moorechip_driver *moorechip)
{
	if (moorechip->notifier_cookie_sec) {
		panel_event_notifier_unregister(moorechip->notifier_cookie_sec);
		moorechip->notifier_cookie_sec = NULL;
	}

	if (moorechip->notifier_cookie) {
		panel_event_notifier_unregister(moorechip->notifier_cookie);
		moorechip->notifier_cookie = NULL;
	}
}

static int moorechip_register_panel_notifier(struct moorechip_driver *moorechip)
{
	int ret;

	ret = moorechip_register_panel_notifier_one(moorechip, "panel",
		PANEL_EVENT_NOTIFICATION_PRIMARY,
		PANEL_EVENT_NOTIFIER_CLIENT_JOYSTICK,
		&moorechip->notifier_cookie, true);
	if (ret)
		return ret;

	ret = moorechip_register_panel_notifier_one(moorechip,
		"secondary-panel", PANEL_EVENT_NOTIFICATION_SECONDARY,
		PANEL_EVENT_NOTIFIER_CLIENT_JOYSTICK_SEC,
		&moorechip->notifier_cookie_sec, false);
	if (ret)
		dev_warn(&moorechip->serdev->dev,
			 "Secondary panel notifier unavailable, continuing primary-only: %d\n",
			 ret);

	return 0;
}

#define MOORECHIP_CALIB_NUM_FIELDS 20

static bool moorechip_abs_calib_valid(const struct moorechip_abs_calib *cal)
{
	if (cal->deadzone < 0)
		return false;

	if (cal->min >= cal->center || cal->max <= cal->center)
		return false;

	return cal->center - cal->min > cal->deadzone &&
	       cal->max - cal->center > cal->deadzone;
}

static bool moorechip_stick_calib_valid(const struct moorechip_stick_calib *cal)
{
	return moorechip_abs_calib_valid(&cal->x) &&
	       moorechip_abs_calib_valid(&cal->y);
}

static bool moorechip_trigger_calib_valid(const struct moorechip_trigger_calib *cal)
{
	return cal->max > cal->min;
}

static ssize_t set_calibration(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	struct moorechip_stick_calib stick_left, stick_right;
	struct moorechip_trigger_calib trigger_left, trigger_right;
	int vals[MOORECHIP_CALIB_NUM_FIELDS];
	char *copy, *next, *token;
	int i;

	copy = kstrdup(buf, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	next = copy;
	for (i = 0; i < MOORECHIP_CALIB_NUM_FIELDS; i++) {
		token = strsep(&next, ":");
		if (!token || kstrtoint(token, 10, &vals[i])) {
			kfree(copy);
			return -EINVAL;
		}
	}
	if (next) {
		kfree(copy);
		return -EINVAL;
	}
	kfree(copy);

	stick_left.x.min = vals[0];
	stick_left.x.max = vals[1];
	stick_left.x.center = vals[2];
	stick_left.x.deadzone = vals[3];
	stick_left.y.min = vals[4];
	stick_left.y.max = vals[5];
	stick_left.y.center = vals[6];
	stick_left.y.deadzone = vals[7];
	stick_right.x.min = vals[8];
	stick_right.x.max = vals[9];
	stick_right.x.center = vals[10];
	stick_right.x.deadzone = vals[11];
	stick_right.y.min = vals[12];
	stick_right.y.max = vals[13];
	stick_right.y.center = vals[14];
	stick_right.y.deadzone = vals[15];
	trigger_left.min = vals[16];
	trigger_left.max = vals[17];
	trigger_right.min = vals[18];
	trigger_right.max = vals[19];

	if (!moorechip_stick_calib_valid(&stick_left) ||
	    !moorechip_stick_calib_valid(&stick_right) ||
	    !moorechip_trigger_calib_valid(&trigger_left) ||
	    !moorechip_trigger_calib_valid(&trigger_right))
		return -EINVAL;

	moorechip->calib_stick_left = stick_left;
	moorechip->calib_stick_right = stick_right;
	moorechip->calib_trigger_left = trigger_left;
	moorechip->calib_trigger_right = trigger_right;

	return count;
}

static ssize_t get_calibration(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	return sysfs_emit(buf,
			"%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\n",
			moorechip->calib_stick_left.x.min,
			moorechip->calib_stick_left.x.max,
			moorechip->calib_stick_left.x.center,
			moorechip->calib_stick_left.x.deadzone,
			moorechip->calib_stick_left.y.min,
			moorechip->calib_stick_left.y.max,
			moorechip->calib_stick_left.y.center,
			moorechip->calib_stick_left.y.deadzone,
			moorechip->calib_stick_right.x.min,
			moorechip->calib_stick_right.x.max,
			moorechip->calib_stick_right.x.center,
			moorechip->calib_stick_right.x.deadzone,
			moorechip->calib_stick_right.y.min,
			moorechip->calib_stick_right.y.max,
			moorechip->calib_stick_right.y.center,
			moorechip->calib_stick_right.y.deadzone,
			moorechip->calib_trigger_left.min,
			moorechip->calib_trigger_left.max,
			moorechip->calib_trigger_right.min,
			moorechip->calib_trigger_right.max);
}
static DEVICE_ATTR(calibration, 0644, get_calibration, set_calibration);

static ssize_t get_raw(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	struct moorechip_key_data *last_keys = &moorechip->last_keys;

	return sysfs_emit(buf, "%04x:%d:%d:%d:%d:%d:%d\n",
			last_keys->keys,
			last_keys->left_joystick_x, last_keys->left_joystick_y,
			last_keys->right_joystick_x, last_keys->right_joystick_y,
			last_keys->left_trigger, last_keys->right_trigger);
}
static DEVICE_ATTR(raw, 0444, get_raw, NULL);

static ssize_t set_layout(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	if (sysfs_streq("xbox", buf))
		moorechip->layout_xbox = true;
	else if (sysfs_streq("nintendo", buf))
		moorechip->layout_xbox = false;
	else
		return -EINVAL;

	return count;
}

static ssize_t get_layout(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	if (moorechip->layout_xbox)
		return sysfs_emit(buf, "xbox\n");
	else
		return sysfs_emit(buf, "nintendo\n");
}
static DEVICE_ATTR(layout, 0644, get_layout, set_layout);

static ssize_t set_triggers(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	if (sysfs_streq("digital", buf))
		moorechip->trigger_mode = MOORECHIP_TRIGGER_MODE_DIGITAL;
	else if (sysfs_streq("analog", buf))
		moorechip->trigger_mode = MOORECHIP_TRIGGER_MODE_ANALOG;
	else if (sysfs_streq("both", buf))
		moorechip->trigger_mode = MOORECHIP_TRIGGER_MODE_BOTH;
	else
		return -EINVAL;

	return count;
}

static ssize_t get_triggers(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	switch (moorechip->trigger_mode) {
	case MOORECHIP_TRIGGER_MODE_DIGITAL:
		return sysfs_emit(buf, "digital\n");
	case MOORECHIP_TRIGGER_MODE_ANALOG:
		return sysfs_emit(buf, "analog\n");
	case MOORECHIP_TRIGGER_MODE_BOTH:
		return sysfs_emit(buf, "both\n");
	}
	return sysfs_emit(buf, "unknown\n");
}
static DEVICE_ATTR(triggers, 0644, get_triggers, set_triggers);

static ssize_t set_ignore_mask(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	if (kstrtou32(buf, 16, &moorechip->ignore_mask))
		return -EINVAL;

	return count;
}

static ssize_t get_ignore_mask(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%x\n", moorechip->ignore_mask);
}
static DEVICE_ATTR(ignore_mask, 0644, get_ignore_mask, set_ignore_mask);

static ssize_t get_firmware_version(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", moorechip->firmware_version);
}
static DEVICE_ATTR(firmware_version, 0444, get_firmware_version, NULL);

static const struct {
	const char *name;
	u32 code;
} moorechip_function_map[] = {
	{ "none",   KEY_RESERVED },
	{ "home",   KEY_HOME },
	{ "select", BTN_SELECT },
	{ "start",  BTN_START },
	{ "back",   KEY_BACK },
	{ "a",      BTN_SOUTH },
	{ "b",      BTN_EAST },
	{ "x",      BTN_WEST },
	{ "y",      BTN_NORTH },
	{ "l1",     BTN_TL },
	{ "l2",     BTN_TL2 },
	{ "l3",     BTN_THUMBL },
	{ "r1",     BTN_TR },
	{ "r2",     BTN_TR2 },
	{ "r3",     BTN_THUMBR },
	{ "down",   BTN_DPAD_DOWN },
	{ "up",     BTN_DPAD_UP },
	{ "left",   BTN_DPAD_LEFT },
	{ "right",  BTN_DPAD_RIGHT },

	// Additional mappable options
	{ "c",               BTN_C },
	{ "z",               BTN_Z },
	{ "mode",            BTN_MODE },
	{ "assist",          KEY_ASSISTANT },
	{ "app_switch",      KEY_APPSELECT },
	{ "dnd",             BTN_DND },
	{ "brightness_down", KEY_BRIGHTNESSDOWN },
	{ "brightness_up",   KEY_BRIGHTNESSUP },
	{ "volume_down",     KEY_VOLUMEDOWN },
	{ "volume_up",       KEY_VOLUMEUP },
	{ "screenshot",      KEY_SYSRQ },
	{ "power",           KEY_POWER },
};

static ssize_t moorechip_set_function(const char *buf, size_t count, u32 *code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(moorechip_function_map); i++) {
		if (sysfs_streq(moorechip_function_map[i].name, buf)) {
			*code = moorechip_function_map[i].code;
			return count;
		}
	}

	return -EINVAL;
}

static ssize_t moorechip_get_function(char *buf, u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(moorechip_function_map); i++) {
		if (moorechip_function_map[i].code == code)
			return sysfs_emit(buf, "%s\n", moorechip_function_map[i].name);
	}

	return -EINVAL;
}

static ssize_t set_m0_function(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	return moorechip_set_function(buf, count, &moorechip->m0_code);
}

static ssize_t get_m0_function(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	return moorechip_get_function(buf, moorechip->m0_code);
}
static DEVICE_ATTR(m0_function, 0644, get_m0_function, set_m0_function);

static ssize_t set_m1_function(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	return moorechip_set_function(buf, count, &moorechip->m1_code);
}

static ssize_t get_m1_function(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	return moorechip_get_function(buf, moorechip->m1_code);
}
static DEVICE_ATTR(m1_function, 0644, get_m1_function, set_m1_function);

static ssize_t set_left_stick_axis_swap(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&moorechip->lock);
	ret = moorechip_set_left_stick_axis_swap(moorechip, enable);
	mutex_unlock(&moorechip->lock);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR(left_stick_axis_swap, 0644, NULL, set_left_stick_axis_swap);

static int moorechip_joystick_probe(struct serdev_device *serdev)
{
	struct moorechip_driver *moorechip;
	struct device *dev = &serdev->dev;
	struct device *joystick;
	int ret = 0;

	moorechip = devm_kzalloc(dev, sizeof(*moorechip), GFP_KERNEL);
	if (!moorechip)
		return -ENOMEM;

	moorechip->serdev = serdev;
	mutex_init(&moorechip->lock);
	INIT_WORK(&moorechip->power_work, moorechip_power_work);
	moorechip->panel_on[0] = true;
	moorechip->panel_on[1] = false;
	moorechip->fw_recheck = true;
	moorechip->seq = 0;
	memset(&moorechip->last_keys, 0, sizeof(moorechip->last_keys));
	moorechip->calib_stick_left.x.min = -1350;
	moorechip->calib_stick_left.x.max = 1350;
	moorechip->calib_stick_left.x.center = 0;
	moorechip->calib_stick_left.y.min = -1350;
	moorechip->calib_stick_left.y.max = 1350;
	moorechip->calib_stick_left.y.center = 0;
	moorechip->calib_stick_right.x.min = -1350;
	moorechip->calib_stick_right.x.max = 1350;
	moorechip->calib_stick_right.x.center = 0;
	moorechip->calib_stick_right.y.min = -1350;
	moorechip->calib_stick_right.y.max = 1350;
	moorechip->calib_stick_right.y.center = 0;
	moorechip->calib_trigger_left.min = 0;
	moorechip->calib_trigger_left.max = 1900;
	moorechip->calib_trigger_right.min = 0;
	moorechip->calib_trigger_right.max = 1900;
	moorechip->trigger_mode = MOORECHIP_TRIGGER_MODE_BOTH;
	moorechip->ignore_mask = 0;
	moorechip->fw = NULL;
	moorechip->m0_code = KEY_RESERVED;
	moorechip->m1_code = KEY_RESERVED;

	moorechip->vdd_reg = devm_regulator_get(dev, "vdd");
	if (IS_ERR(moorechip->vdd_reg)) {
		ret = PTR_ERR(moorechip->vdd_reg);
		dev_err(dev, "Failed to get joystick vdd regulator\n");
		return ret;
	}

	moorechip->levelshifter_reg = devm_regulator_get(dev, "levelshifter");
	if (IS_ERR(moorechip->levelshifter_reg)) {
		ret = PTR_ERR(moorechip->levelshifter_reg);
		dev_err(dev, "Failed to get joystick levelshifter regulator\n");
		return ret;
	}

	moorechip->boot_gpio = of_get_named_gpio(dev->of_node, "moorechip,boot-gpio", 0);
	if (moorechip->boot_gpio >= 0) {
		ret = devm_gpio_request(dev, moorechip->boot_gpio, "joystick-boot");
		if (ret) {
			dev_err(dev, "Failed to request gpio joystick-boot\n");
			return ret;
		}
	}

	moorechip->reset_gpio = of_get_named_gpio(dev->of_node, "moorechip,reset-gpio", 0);
	if (moorechip->reset_gpio >= 0) {
		ret = devm_gpio_request(dev, moorechip->reset_gpio, "joystick-reset");
		if (ret) {
			dev_err(dev, "Failed to request gpio joystick-reset\n");
			return ret;
		}
	}

	serdev_device_set_drvdata(serdev, moorechip);
	serdev_device_set_client_ops(serdev, &moorechip_joystick_ops);
	moorechip->input = NULL;

	if (of_property_read_bool(dev->of_node, "moorechip,layout-xbox")) {
		moorechip->layout_xbox = true;
	} else if (of_property_read_bool(dev->of_node, "moorechip,layout-nintendo")) {
		moorechip->layout_xbox = false;
	} else {
		dev_err(dev, "Layout not defined!\n");
		return -EINVAL;
	}

	moorechip->m0_key = devm_gpiod_get(dev, "m0_key", GPIOD_IN);
	if (PTR_ERR(moorechip->m0_key) == -ENOENT) {
		moorechip->m0_key = NULL;
	} else if (IS_ERR(moorechip->m0_key)) {
		ret = PTR_ERR(moorechip->m0_key);
		dev_err(dev, "failed to get m0 key: %d\n", ret);
		goto err;
	} else {
		ret = gpiod_to_irq(moorechip->m0_key);
		if (ret < 0) {
			dev_err(dev, "Unable to get irq number for m0 key: %d\n", ret);
			goto err;
		}
		moorechip->m0_irq = ret;

		ret = devm_request_any_context_irq(dev, moorechip->m0_irq,
			moorechip_gpio_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"m0_key", moorechip);
		if (ret < 0) {
			dev_err(dev, "Unable to claim m0 key irq: %d\n", ret);
			goto err;
		}
	}

	moorechip->m1_key = devm_gpiod_get(dev, "m1_key", GPIOD_IN);
	if (PTR_ERR(moorechip->m1_key) == -ENOENT) {
		moorechip->m1_key = NULL;
	} else if (IS_ERR(moorechip->m1_key)) {
		ret = PTR_ERR(moorechip->m1_key);
		dev_err(dev, "failed to get m1 key: %d\n", ret);
		goto err;
	} else {
		ret = gpiod_to_irq(moorechip->m1_key);
		if (ret < 0) {
			dev_err(dev, "Unable to get irq number for m1 key: %d\n", ret);
			goto err;
		}
		moorechip->m1_irq = ret;

		ret = devm_request_any_context_irq(dev, moorechip->m1_irq,
			moorechip_gpio_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"m1_key", moorechip);
		if (ret < 0) {
			dev_err(dev, "Unable to claim m1 key irq: %d\n", ret);
			goto err;
		}
	}

	ret = moorechip_joystick_register_input(moorechip);
	if (ret)
		goto err;

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
	moorechip->class = class_create("moorechip-joystick");
#else
	moorechip->class = class_create(THIS_MODULE, "moorechip-joystick");
#endif
	if (IS_ERR(moorechip->class)) {
		ret = PTR_ERR(moorechip->class);
		moorechip->class = NULL;
		goto err;
	}

	joystick = device_create(moorechip->class, NULL, 0, NULL, "joystick");
	if (IS_ERR(joystick)) {
		ret = PTR_ERR(joystick);
		goto err_destroy_class;
	}
	dev_set_drvdata(joystick, moorechip);
	device_create_file(joystick, &dev_attr_calibration);
	device_create_file(joystick, &dev_attr_raw);
	device_create_file(joystick, &dev_attr_layout);
	device_create_file(joystick, &dev_attr_triggers);
	device_create_file(joystick, &dev_attr_ignore_mask);
	device_create_file(joystick, &dev_attr_firmware_version);
	if (moorechip->m0_key)
		device_create_file(joystick, &dev_attr_m0_function);
	if (moorechip->m1_key)
		device_create_file(joystick, &dev_attr_m1_function);
	device_create_file(joystick, &dev_attr_left_stick_axis_swap);

	ret = moorechip_register_panel_notifier(moorechip);
	if (ret)
		goto err_destroy_class_device;
	moorechip->panel_on[1] = !!moorechip->notifier_cookie_sec;

	mutex_lock(&moorechip->lock);
	ret = moorechip_joystick_power_on_locked(moorechip);
	mutex_unlock(&moorechip->lock);
	if (ret)
		goto err_unregister_notifier;

	return 0;

err_unregister_notifier:
	moorechip_unregister_panel_notifiers(moorechip);
err_destroy_class_device:
	device_destroy(moorechip->class, 0);
err_destroy_class:
	class_destroy(moorechip->class);
	moorechip->class = NULL;

err:
	return ret;
}

static void moorechip_joystick_remove(struct serdev_device *serdev)
{
	struct moorechip_driver *moorechip = serdev_device_get_drvdata(serdev);

	cancel_work_sync(&moorechip->power_work);

	mutex_lock(&moorechip->lock);
	moorechip->panel_on[0] = false;
	moorechip->panel_on[1] = false;
	moorechip->system_suspended = true;
	moorechip_joystick_power_off_locked(moorechip);
	mutex_unlock(&moorechip->lock);

	moorechip_unregister_panel_notifiers(moorechip);

	if (moorechip->class) {
		device_destroy(moorechip->class, 0);
		class_destroy(moorechip->class);
	}
}

static int __maybe_unused moorechip_joystick_suspend(struct device *dev)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);

	cancel_work_sync(&moorechip->power_work);

	mutex_lock(&moorechip->lock);
	moorechip->system_suspended = true;
	moorechip_joystick_power_off_locked(moorechip);
	mutex_unlock(&moorechip->lock);

	return 0;
}

static int __maybe_unused moorechip_joystick_resume(struct device *dev)
{
	struct moorechip_driver *moorechip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&moorechip->lock);
	moorechip->system_suspended = false;
	ret = moorechip_joystick_power_on_locked(moorechip);
	mutex_unlock(&moorechip->lock);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id moorechip_joystick_of_match[] = {
	{ .compatible = "moorechip,joystick" },
	{},
};
MODULE_DEVICE_TABLE(of, moorechip_joystick_of_match);
#endif

static const struct dev_pm_ops moorechip_joystick_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(moorechip_joystick_suspend, moorechip_joystick_resume)
};

static struct serdev_device_driver moorechip_joystick_driver = {
	.probe = moorechip_joystick_probe,
	.remove = moorechip_joystick_remove,
	.driver = {
		.name = "moorechip-joystick",
		.of_match_table = of_match_ptr(moorechip_joystick_of_match),
		.pm = &moorechip_joystick_pm_ops,
	},
};

int __init moorechip_joystick_init(void)
{
	return serdev_device_driver_register(&moorechip_joystick_driver);
}

void __exit moorechip_joystick_deinit(void)
{
	serdev_device_driver_unregister(&moorechip_joystick_driver);
}

module_init(moorechip_joystick_init);
module_exit(moorechip_joystick_deinit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Balázs Triszka <info@balika011.hu>");
MODULE_DESCRIPTION("Driver for Moorechip controller over UART");
