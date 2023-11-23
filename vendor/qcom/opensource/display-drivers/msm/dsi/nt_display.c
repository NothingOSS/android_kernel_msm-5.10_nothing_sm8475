#include "sde_connector.h"
#include "dsi_display.h"
#include "dsi_panel.h"

#include <linux/list.h>
#include <linux/of.h>
#include <linux/err.h>

const char *cmd_set_sfm_map[DSI_CMD_SFM_MAX] = {
	"set-frame-command",
	"disable-skip-frame-command",
	"get-reg2f-value-command",
	"get-reg6d-value-command",
	"switch-te-not-follow-source-command",
	"switch-te-follow-source-command",
	"switch-30hz-dimming-command",
	"switch-10hz-dimming-command",
	"switch-1hz-dimming-command",
	"switch-60hz-no-dimming-command",
	"switch-30hz-no-dimming-command",
	"switch-24hz-no-dimming-command",
	"switch-10hz-no-dimming-command",
	"switch-1hz-no-dimming-command",
};

extern struct dsi_panel *nt_panel;
extern struct sde_connector *panel_feature_sde_conn;

int nt_display_alloc_sfm_cmd_packets(struct dsi_display_refresh_rate_cmd_set *cmd,
					u32 packet_count)
{
	u32 size;

	size = packet_count * sizeof(*cmd->cmds);
	cmd->cmds = kzalloc(size, GFP_KERNEL);
	if (!cmd->cmds)
		return -ENOMEM;

	cmd->count = packet_count;
	return 0;
}

int nt_display_parse_switch_cmds(struct dsi_panel *panel)
{
	struct dsi_parser_utils *nt_utils = &panel->utils;
	int i, rc = 0;

	for (i = DSI_CMD_SET_FRAME; i < DSI_CMD_SFM_MAX; i++) {
		const char *string = NULL;

		if (i < DSI_CMD_30HZ_DIMMING) {
			nt_display_alloc_sfm_cmd_packets(&panel->nt_cmd_sets[i], COMMAND_LENGTH_1_BYTE_SEND);
		} else {
			nt_display_alloc_sfm_cmd_packets(&panel->nt_cmd_sets[i], COMMAND_LENGTH_2_BYTE_SEND);
		}

		rc = nt_utils->read_string(nt_utils->data, cmd_set_sfm_map[i], &string);
		if (rc)
			SDE_ERROR("sfm failed to parse set %d\n", i);
		strcpy(panel->nt_cmd_sets[i].cmds, string);
	}

	return rc;
}

int set_refresh_rate(struct sde_connector *sde_conn, int index)
{
	struct dsi_panel *panel = nt_panel;
	int skip_flag = 0;
	int err_flag = 0;

	switch (index)
	{
	case 0:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_30HZ_DIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 1:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_10HZ_DIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 2:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_1HZ_DIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 7:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_TE_FOLLOW_SOURCE].cmds, COMMAND_LENGTH_1_BYTE_SEND);
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_60HZ_NODIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 8:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_TE_NOTFOLLOW_SOURCE].cmds, COMMAND_LENGTH_1_BYTE_SEND);
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_30HZ_NODIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 9:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_24HZ_NODIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 10:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_TE_NOTFOLLOW_SOURCE].cmds, COMMAND_LENGTH_1_BYTE_SEND);
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_10HZ_NODIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 11:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_1HZ_NODIMMING].cmds, COMMAND_LENGTH_2_BYTE_SEND);
		break;
	case 16:
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_DISABLE_SKIP_FRAME_MODE].cmds, COMMAND_LENGTH_1_BYTE_SEND);
		skip_flag = 1;
		break;
	default:
		SDE_ERROR("set refresh rate failed, invalid index: %d\n", index);
		err_flag = -1;
		break;
	}

	if (!skip_flag && !err_flag) {
		nt_tx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_SET_FRAME].cmds, COMMAND_LENGTH_1_BYTE_SEND);
	}

	return err_flag;
}

int get_refresh_rate(struct sde_connector *sde_conn)
{
	struct dsi_panel *panel = nt_panel;
	int reg_2f;
	int reg_6d[2];
	int rc;
	int i, index = 0;
	unsigned long refresh_rate = 0;

	rc = nt_rx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_GET_REG2F].cmds, COMMAND_LENGTH_GET_VALUE);
	if (rc) {
		SDE_ERROR("read refresh_rate failed: 2f, cmds: %s\n", panel->nt_cmd_sets[DSI_CMD_GET_REG2F].cmds);
	}
	reg_2f = sde_conn->cmd_rx_buf[0];

	if (reg_2f != 0x00 && reg_2f != 0x01) {
		rc = nt_rx_cmd(sde_conn, panel->nt_cmd_sets[DSI_CMD_GET_REG6D].cmds, COMMAND_LENGTH_GET_VALUE);
		if (rc) {
			SDE_ERROR("read refresh_rate failed: 6d, cmds: %s\n", panel->nt_cmd_sets[DSI_CMD_GET_REG6D].cmds);
		}
		for (i = 0; i < sde_conn->rx_len; i++)
		{
			reg_6d[i] = sde_conn->cmd_rx_buf[i];
		}
	}

	switch (reg_2f) {
	case 0x00:
		refresh_rate = 120;
		index = 16;
		break;
	case 0x01:
		refresh_rate = 90;
		index = 16;
		break;
	case 0x30:
		if(reg_6d[1]) {
			switch (reg_6d[0]) {
			case 0x02:
				refresh_rate = 30;
				index = 0;
				break;
			case 0x04:
				refresh_rate = 10;
				index = 1;
				break;
			case 0x05:
				refresh_rate = 1;
				index = 2;
				break;
			default:
				index = -1;
				SDE_ERROR("unknown 6d value: %d\n", reg_6d);
				break;
			}
		} else {
			switch (reg_6d[0]) {
			case 0x00:
				refresh_rate = 60;
				index = 7;
				break;
			case 0x06:
				refresh_rate = 30;
				index = 8;
				break;
			case 0x03:
				refresh_rate = 24;
				index = 9;
				break;
			case 0x07:
				refresh_rate = 10;
				index = 10;
				break;
			case 0x05:
				refresh_rate = 1;
				index = 11;
				break;
			default:
				index = -1;
				SDE_ERROR("unknown 6d value: %d\n", reg_6d);
				break;
			}
		}
		break;
	default:
		SDE_ERROR("invalid data\n");
		index = -1;
		break;
	}
	return index;
}

