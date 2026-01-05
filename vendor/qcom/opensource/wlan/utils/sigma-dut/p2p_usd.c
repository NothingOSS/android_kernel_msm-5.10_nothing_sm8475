/*
 * Sigma Control API DUT (USD functionality)
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc.
 * All Rights Reserved.
 * Licensed under the Clear BSD license. See README for more details.
 */

#include "sigma_dut.h"
#include <sys/stat.h>
#include "wpa_ctrl.h"
#include "wpa_helpers.h"


enum sigma_cmd_result sigma_usd_publish(struct sigma_dut *dut,
					struct sigma_conn *conn,
					struct sigma_cmd *cmd)
{
	const char *program = get_param(cmd, "Prog");
	const char *ifname = get_param(cmd, "interface");
	const char *service_name = get_param(cmd, "ServiceName");
	const char *ssi = get_param(cmd, "ServSpecificInfoPayload");
	const char *serv_proto_type = get_param(cmd, "ServProtoType");
	const char *oper_chan = get_param(cmd, "AdvertiseChannel");
	const char *bootstrapmethod = get_param(cmd, "PairingBootstrapMethod");
	char buf[3000];
	int i, res;
	size_t len;

	if (!ifname)
		ifname = get_station_ifname(dut);

	if (bootstrapmethod) {
		if (wpa_command(ifname, "P2P_SET pairing_setup 1") < 0) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Failed to set pairing setup");
			return STATUS_SENT;
		}

		if (wpa_command(ifname, "P2P_SET pairing_cache 1") < 0) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Failed to set pairing cache");
			return STATUS_SENT;
		}
	}

	if (!service_name)
		return ERROR_SEND_STATUS;

	res = snprintf(buf, sizeof(buf), "NAN_PUBLISH service_name=%s ttl=100",
		       service_name);
	if (res < 0 || res >= sizeof(buf))
		return ERROR_SEND_STATUS;
	len = res;

	if (program && strcasecmp(program, "P2P") == 0) {
		res = snprintf(buf + len, sizeof(buf) - len, " p2p=1");
		if (res < 0 || res >= sizeof(buf) - len)
			return ERROR_SEND_STATUS;
		len += res;
	}

	if (ssi) {
		size_t ssi_len = atoi(ssi);
		char ssi_hex[2048];

		if (ssi_len * 2 + 1 > sizeof(ssi_hex))
			return ERROR_SEND_STATUS;
		for (i = 0; i < ssi_len; i++) {
			res = snprintf(ssi_hex + i * 2, sizeof(ssi_hex) - i * 2,
				       "%02x", i);
			if (res < 0 || res >= sizeof(ssi_hex) - i * 2)
				return ERROR_SEND_STATUS;
		}
		ssi_hex[ssi_len * 2] = '\0';
		res = snprintf(buf + len, sizeof(buf) - len, " ssi=%s",
			       ssi_hex);
		if (res < 0 || res >= sizeof(buf) - len)
			return ERROR_SEND_STATUS;
		len += res;
	}

	if (serv_proto_type) {
		res = snprintf(buf + len, sizeof(buf) - len,
			       " srv_proto_type=%s", serv_proto_type);
		if (res < 0 || res >= sizeof(buf) - len)
			return ERROR_SEND_STATUS;
		len += res;
	}

	if (oper_chan) {
		int chan, freq;

		chan = atoi(oper_chan);
		freq = channel_to_freq(dut, chan);
		res = snprintf(buf + len, sizeof(buf) - len, " freq=%d", freq);
	} else {
		res = snprintf(buf + len, sizeof(buf) - len, " freq=2437");
	}
	if (res < 0 || res >= sizeof(buf) - len)
		return ERROR_SEND_STATUS;
	len += res;

	if (wpa_command(ifname, buf)) {
		send_resp(dut, conn, SIGMA_ERROR, "Unable to USD publish");
		return STATUS_SENT_ERROR;
	}

	return SUCCESS_SEND_STATUS;
}


enum sigma_cmd_result sigma_usd_subscribe(struct sigma_dut *dut,
					  struct sigma_conn *conn,
					  struct sigma_cmd *cmd)
{
	const char *program = get_param(cmd, "Prog");
	const char *ifname = get_param(cmd, "interface");
	const char *service_name = get_param(cmd, "ServiceName");
	const char *ssi = get_param(cmd, "ServSpecificInfoPayload");
	const char *serv_proto_type = get_param(cmd, "ServProtoType");
	const char *default_chan = get_param(cmd, "DefaultPublishChannel");
	const char *chan_list = get_param(cmd, "PublishChannelList");
	const char *bootstrapmethod = get_param(cmd, "PairingBootstrapMethod");
	char buf[3000];
	int i, res;
	size_t len;

	if (!ifname)
		ifname = get_station_ifname(dut);

	if (bootstrapmethod) {
		if (wpa_command(ifname, "P2P_SET pairing_setup 1") < 0) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Failed to set pairing setup");
			return STATUS_SENT;
		}

		if (wpa_command(ifname, "P2P_SET pairing_cache 1") < 0) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Failed to set pairing cache");
			return STATUS_SENT;
		}
	}

	if (!service_name)
		return ERROR_SEND_STATUS;

	res = snprintf(buf, sizeof(buf),
		       "NAN_SUBSCRIBE service_name=%s ttl=100",
		       service_name);
	if (res < 0 || res >= sizeof(buf))
		return ERROR_SEND_STATUS;
	len = res;

	if (program && strcasecmp(program, "P2P") == 0) {
		res = snprintf(buf + len, sizeof(buf) - len, " p2p=1");
		if (res < 0 || res >= sizeof(buf) - len)
			return ERROR_SEND_STATUS;
		len += res;
	}

	if (ssi) {
		size_t ssi_len = atoi(ssi);
		char ssi_hex[2048];

		if (ssi_len * 2 + 1 > sizeof(ssi_hex))
			return ERROR_SEND_STATUS;
		for (i = 0; i < ssi_len; i++) {
			res = snprintf(ssi_hex + i * 2, sizeof(ssi_hex) - i * 2,
					"%02x", i);
			if (res < 0 || res >= sizeof(ssi_hex) - i * 2)
				return ERROR_SEND_STATUS;
		}
		ssi_hex[ssi_len * 2] = '\0';
		res = snprintf(buf + len, sizeof(buf) - len, " ssi=%s",
			       ssi_hex);
		if (res < 0 || res >= sizeof(buf) - len)
			return ERROR_SEND_STATUS;
		len += res;
	}

	if (serv_proto_type) {
		res = snprintf(buf + len, sizeof(buf) - len,
			       " srv_proto_type=%s", serv_proto_type);
		if (res < 0 || res >= sizeof(buf) - len)
			return ERROR_SEND_STATUS;
		len += res;
	}

	if (default_chan) {
		int chan, freq;

		chan = atoi(default_chan);
		freq = channel_to_freq(dut, chan);
		res = snprintf(buf + len, sizeof(buf) - len, " freq=%d", freq);
	} else {
		res = snprintf(buf + len, sizeof(buf) - len, " freq=2437");
	}
	if (res < 0 || res >= sizeof(buf) - len)
		return ERROR_SEND_STATUS;
	len += res;

	if (chan_list) {
		char freq_list[200], *token;
		size_t flen = 0;
		char *ch_list = strdup(chan_list);
		char *saveptr = NULL;

		if (!ch_list)
			return ERROR_SEND_STATUS;

		freq_list[0] = '\0';
		token = strtok_r(ch_list, " ", &saveptr);
		while (token && strlen(freq_list) < sizeof(freq_list) - 10) {
			int chan, freq;

			chan = atoi(token);
			freq = channel_to_freq(dut, chan);
			res = snprintf(freq_list, sizeof(freq_list) - flen,
				       "%d,", freq);
			if (res < 0 || res >= sizeof(freq_list) - flen)
				return ERROR_SEND_STATUS;
			flen += res;
			token = strtok_r(NULL, " ", &saveptr);
		}
		free(ch_list);
		if (flen > 0)
			freq_list[flen - 1] = '\0';

		res = snprintf(buf + len, sizeof(buf) - len, " freq_list=%s",
			       freq_list);
	} else {
		res = snprintf(buf + len, sizeof(buf) - len, " freq_list=5180");
	}
	if (res < 0 || res >= sizeof(buf) - len)
		return ERROR_SEND_STATUS;
	len += res;

	if (wpa_command(ifname, buf)) {
		send_resp(dut, conn, SIGMA_ERROR, "Unable to USD subscribe");
		return STATUS_SENT_ERROR;
	}

	return SUCCESS_SEND_STATUS;
}


enum sigma_cmd_result usd_cmd_sta_exec_action(struct sigma_dut *dut,
					      struct sigma_conn *conn,
					      struct sigma_cmd *cmd)
{
	int ret;
	u8 len;
	char buf[200];
	const char *ifname = get_param(cmd, "interface");
	const char *method_type = get_param(cmd, "MethodType");
	const char *pairing_bootstrap_method;
	int bootstrap = 0;

	if (!ifname)
		ifname = get_station_ifname(dut);

	if (!dut->usd_enabled) {
		send_resp(dut, conn, SIGMA_ERROR, "errorCode,USD not enabled");
		return STATUS_SENT;
	}

	pairing_bootstrap_method = get_param(cmd, "PairingBootstrapMethod");
	if (pairing_bootstrap_method) {
		if (strcasecmp(pairing_bootstrap_method, "NonZero") == 0) {
			/* opportunistic */
			bootstrap = BIT(0);
			/* Display pincode and passphrase */
			bootstrap |= BIT(1) | BIT(2);
			/* Keypad pincode and passphrase */
			bootstrap |= BIT(5) | BIT(6);
		} else {
			bootstrap = atoi(pairing_bootstrap_method);
		}

		len = snprintf(buf, sizeof(buf),
			       "P2P_SET supported_bootstrapmethods %d",
			       bootstrap);
		if (len < 0 || len >= sizeof(buf))
			return ERROR_SEND_STATUS;
		sigma_dut_print(dut, DUT_MSG_INFO, "PP2_SET to wpa_command: %s",
				buf);
		if (wpa_command(ifname, buf)) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "errorCode,Unable to set pairing bootstrap method");
			return STATUS_SENT_ERROR;
		}
	}

	if (!method_type) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Missing USD Type");
		return STATUS_SENT_ERROR;
	}

	if (strcasecmp(method_type, "ADVERTISE") == 0) {
		ret = sigma_usd_publish(dut, conn, cmd);
		if (ret < 0)
			return ret;
		send_resp(dut, conn, SIGMA_COMPLETE, "NULL");
		return STATUS_SENT;
	}

	if (strcasecmp(method_type, "SEEK") == 0) {
		ret = sigma_usd_subscribe(dut, conn, cmd);
		if (ret < 0)
			return ret;
		send_resp(dut, conn, SIGMA_COMPLETE, "NULL");
		return STATUS_SENT;
	}

	send_resp(dut, conn, SIGMA_ERROR,
		  "errorCode,Unsupported USD MethodType");
	return STATUS_SENT;
}
