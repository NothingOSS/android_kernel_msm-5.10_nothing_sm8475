/*
 * Sigma Control API DUT (FTM LOC functionality)
 * Copyright (c) 2016, Qualcomm Atheros, Inc.
 * Copyright (c) 2019, The Linux Foundation
 * All Rights Reserved.
 * Licensed under the Clear BSD license. See README for more details.
 */

#include "sigma_dut.h"
#include <sys/stat.h>
#include <regex.h>
#include "wpa_helpers.h"
#include "wpa_ctrl.h"

static const char LOC_XML_FILE_PATH[] = "./data/sigma-dut-target.xml";
static const char LOC_11AZ_CONFIG_XML_FILE_PATH[] = "./data/lowi11az.xml";

static const char LOC_LOWI_TEST_DISCOVERY[] = "lowi_test -a -b 2 -n 1";
static const char LOC_LOWI_TEST_RANGING[] =
"lowi_test -r ./data/sigma-dut-target.xml -n 1";
static const char LOC_LOWI_TEST_NEIGHBOR_RPT_REQ[] = "lowi_test -nrr";
static const char LOC_LOWI_TEST_ANQP_REQ[] = "lowi_test -anqp -mac ";
static const char LOC_LOWI_TEST_11AZ_CONFIG[] =
"lowi_test -11az ./data/lowi11az.xml";
static const char WPA_INTERWORKING_ENABLE[] =
"SET interworking 1";
static const char WPA_INTERWORKING_DISABLE[] =
"SET interworking 0";
static const char WPA_RM_ENABLE[] =
"VENDOR 1374 74 08000400BD000000";
static const char WPA_RM_DISABLE[] =
"VENDOR 1374 74 0800040000000000";
static const char WPA_ADDRESS_3_ENABLE[] =
"SET gas_address3 1";
static const char WPA_ADDRESS_3_DISABLE[] =
"SET gas_address3 0";

#ifndef ETH_ALEN
#define ETH_ALEN 6
#endif

#define LOC_MAX_RM_FLAGS 10
#define LOC_RM_FLAG_VAL_ARRAY 2

enum lowi_tst_cmd {
	LOWI_TST_RANGING = 0,
	LOWI_TST_NEIGHBOR_REPORT_REQ = 1,
	LOWI_TST_ANQP_REQ = 2,
};

struct capi_loc_cmd {
	unsigned int chan;
	unsigned int burstExp;
	unsigned int burstDur;
	unsigned int minDeltaFtm;
	unsigned int ptsf;
	unsigned int asap;
	unsigned int ftmsPerBurst;
	unsigned int fmtbw;
	unsigned int burstPeriod;
	unsigned int locCivic;
	unsigned int lci;
	unsigned int ntb;
	unsigned int tb;
	unsigned int ftm_bw_rtt;
	unsigned int freq;
};


static int loc_write_xml_file(struct sigma_dut *dut, const char *dst_mac_str,
			      struct capi_loc_cmd *loc_cmd)
{
	FILE *xml;
	unsigned int band, bw, preamble, primary_ch, center_freq;

	xml = fopen(LOC_XML_FILE_PATH, "w");
	if (!xml) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Unable to create the XML file", __func__);
		return -1;
	}

	/* Using this following defaults:
	 * default value of band 1
	 */
	band = 1;

#define FMT_BW_NO_PREF 0
#define FMT_BW_HT_20 9
#define FMT_BW_VHT_20 10
#define FMT_BW_HT_40 11
#define FMT_BW_VHT_40 12
#define FMT_BW_VHT_80 13

#define LOC_BW_20 0
#define LOC_BW_40 1
#define LOC_BW_80 2
#define LOC_PREAMBLE_HT 1
#define LOC_PREAMBLE_VHT 2
	switch (loc_cmd->fmtbw) {
	case FMT_BW_NO_PREF:
	case FMT_BW_HT_20:
		bw = LOC_BW_20;
		preamble = LOC_PREAMBLE_HT;
		primary_ch = 36;
		center_freq = 5180;
		break;
	case FMT_BW_VHT_20:
		bw = LOC_BW_20;
		preamble = LOC_PREAMBLE_VHT;
		primary_ch = 36;
		center_freq = 5180;
		break;
	case FMT_BW_HT_40:
		bw = LOC_BW_40;
		preamble = LOC_PREAMBLE_HT;
		primary_ch = 36;
		center_freq = 5190;
		break;
	case FMT_BW_VHT_40:
		bw = LOC_BW_40;
		preamble = LOC_PREAMBLE_VHT;
		primary_ch = 36;
		center_freq = 5190;
		break;
	case FMT_BW_VHT_80:
		bw = LOC_BW_80;
		preamble = LOC_PREAMBLE_VHT;
		primary_ch = 36;
		center_freq = 5210;
		break;
	default:
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Bad Format/BW received", __func__);
		fclose(xml);
		return -1;
	}

#define LOC_CAPI_DEFAULT_FTMS_PER_BURST 5
#define LOC_CAPI_DEFAULT_BURST_DUR 10
	fprintf(xml, "<body>\n");
	fprintf(xml, "  <ranging>\n");
	fprintf(xml, "    <ap>\n");
	fprintf(xml, "    <band>%u</band>\n", band);
	fprintf(xml, "    <rttType>3</rttType>\n");
	fprintf(xml, "    <numFrames>%u</numFrames>\n",
		LOC_CAPI_DEFAULT_FTMS_PER_BURST);
	fprintf(xml, "    <bw>%u</bw>\n", bw);
	fprintf(xml, "    <preamble>%u</preamble>\n", preamble);
	fprintf(xml, "    <asap>%u</asap>\n", loc_cmd->asap);
	fprintf(xml, "    <lci>%u</lci>\n", loc_cmd->lci);
	fprintf(xml, "    <civic>%u</civic>\n", loc_cmd->locCivic);
	fprintf(xml, "    <burstsexp>%u</burstsexp>\n", loc_cmd->burstExp);
	fprintf(xml, "    <burstduration>%u</burstduration>\n",
		LOC_CAPI_DEFAULT_BURST_DUR);
	fprintf(xml, "    <burstperiod>%u</burstperiod>\n", 0);
	/* Use parameters from LOWI cache */
	fprintf(xml, "    <paramControl>%u</paramControl>\n", 0);
	fprintf(xml, "    <ptsftimer>%u</ptsftimer>\n", 0);
	fprintf(xml, "    <center_freq1>%u</center_freq1>\n", center_freq);
	fprintf(xml, "    <center_freq2>0</center_freq2>\n");
	fprintf(xml, "    <ch>%u</ch>\n", primary_ch);
	fprintf(xml, "    <mac>%s</mac>\n", dst_mac_str);
	fprintf(xml, "    </ap>\n");
	fprintf(xml, "  </ranging>\n");
	fprintf(xml, "  <summary>\n");
	fprintf(xml, "    <mac>%s</mac>\n", dst_mac_str);
	fprintf(xml, "  </summary>\n");
	fprintf(xml, "</body>\n");

	fclose(xml);
	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Successfully created XML file", __func__);
	return 0;
}


static int pass_request_to_ltest(struct sigma_dut *dut, enum lowi_tst_cmd cmd,
				 const char *params)
{
#define MAX_ANQP_CMND_SIZE 256
	int ret;
	const char *cmd_str;
	char lowi_anqp_query[MAX_ANQP_CMND_SIZE];

	switch (cmd) {
	case LOWI_TST_RANGING:
		cmd_str = LOC_LOWI_TEST_RANGING;
		break;
	case LOWI_TST_NEIGHBOR_REPORT_REQ:
		cmd_str = LOC_LOWI_TEST_NEIGHBOR_RPT_REQ;
		break;
	case LOWI_TST_ANQP_REQ:
		if (!params) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - No Destination Mac provided for ANQP Query",
					__func__);
			return -1;
		}

		sigma_dut_print(dut, DUT_MSG_INFO,
				"%s - Destination Mac provided for ANQP Query: %s",
				__func__, params);

		snprintf(lowi_anqp_query, MAX_ANQP_CMND_SIZE, "%s%s",
			 LOC_LOWI_TEST_ANQP_REQ, params);
		cmd_str = lowi_anqp_query;
		break;
	default:
		cmd_str = LOC_LOWI_TEST_DISCOVERY;
		break;
	}

	sigma_dut_print(dut, DUT_MSG_INFO, "%s - 1 - Running command: %s",
			__func__, LOC_LOWI_TEST_DISCOVERY);
	ret = system(LOC_LOWI_TEST_DISCOVERY);
	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Finished Performing Discovery Scan through LOWI_test: ret: %d",
			__func__, ret);
	sleep(1);
	sigma_dut_print(dut, DUT_MSG_INFO, "%s - 2 - Running command: %s",
			__func__, cmd_str);
	ret = system(cmd_str);
	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Finished Performing command: %s, got ret: %d",
			__func__, cmd_str, ret);

	return ret;
}


int loc_cmd_sta_exec_action(struct sigma_dut *dut, struct sigma_conn *conn,
			    struct sigma_cmd *cmd)
{
	const char *params = NULL;
	enum lowi_tst_cmd cmnd = LOWI_TST_RANGING;
	const char *program = get_param(cmd, "prog");
	const char *loc_op = get_param(cmd, "Trigger");
	const char *interface = get_param(cmd, "interface");

	const char *destMacStr = get_param(cmd, "destmac");
	const char *burstExp = get_param(cmd, "burstsexponent");
	const char *asap = get_param(cmd, "ASAP");
	const char *fmtbw = get_param(cmd, "formatbwftm");
	const char *locCivic = get_param(cmd, "askforloccivic");
	const char *lci = get_param(cmd, "askforlci");
	struct capi_loc_cmd loc_cmd;

	memset(&loc_cmd, 0, sizeof(loc_cmd));

	if (!loc_op) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - No Operation! - Aborting", __func__);
		return -1;
	}

	cmnd = strcasecmp(loc_op, "ANQPQuery") == 0 ?
		LOWI_TST_ANQP_REQ : LOWI_TST_RANGING;
	sigma_dut_print(dut, DUT_MSG_INFO, "%s - Going to perform: %s",
			__func__, loc_op);

	if (!program) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - No Program in Command! - Aborting",
				__func__);
		return -1;
	}

	if (!interface) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOC CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Incomplete Loc CAPI command - missing interface");
		return 0;
	}

	if (!destMacStr) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOC CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Incomplete Loc CAPI command - missing MAC");
		return 0;
	}

	if (cmnd == LOWI_TST_RANGING) {
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - LOWI_TST_RANGING",
				__func__);
		if (!burstExp) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - Incomplete command in LOC CAPI request",
					__func__);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Incomplete Loc CAPI command - missing Burst Exp");
			return 0;
		}

		if (!asap) {
			sigma_dut_print(dut, DUT_MSG_INFO,
					"%s - Incomplete command in LOC CAPI request",
					__func__);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Incomplete Loc CAPI command - missing ASAP");
			return 0;
		}

		if (!fmtbw) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - Incomplete command in LOC CAPI request",
					__func__);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Incomplete Loc CAPI command - missing Format & BW");
			return 0;
		}

		if (!locCivic) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - Incomplete command in LOC CAPI request",
					__func__);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Incomplete Loc CAPI command - missing Location Civic");
			return 0;
		}

		if (!lci) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - Incomplete command in LOC CAPI request",
					__func__);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Incomplete Loc CAPI command - missing LCI");
			return 0;
		}

		if (strcasecmp(program, "loc") != 0) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - Unsupported Program: %s",
					__func__, program);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Unsupported program");
			return 0;
		}

		if (strcasecmp(interface, "wlan0") != 0) {
			sigma_dut_print(dut, DUT_MSG_INFO,
					"%s - Unsupported Interface Type: %s",
					__func__, interface);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Unsupported Interface Type");
			return 0;
		}

		sscanf(burstExp, "%u", &loc_cmd.burstExp);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - burstExp: %u",
				__func__, loc_cmd.burstExp);
		sscanf(asap, "%u", &loc_cmd.asap);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - asap: %u",
				__func__, loc_cmd.asap);
		sscanf(fmtbw, "%u", &loc_cmd.fmtbw);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - fmtbw: %u",
				__func__, loc_cmd.fmtbw);
		sscanf(locCivic, "%u", &loc_cmd.locCivic);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - locCivic: %u",
				__func__, loc_cmd.locCivic);
		sscanf(lci, "%u", &loc_cmd.lci);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - lci: %u",
				__func__, loc_cmd.lci);

		if (loc_write_xml_file(dut, destMacStr, &loc_cmd) < 0) {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - Failed to write to XML file because of bad command",
					__func__);
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrMsg,Bad CAPI command");
			return 0;
		}
	} else {
		/* ANQP Query */
		sigma_dut_print(dut, DUT_MSG_INFO,
				"%s - LOWI_TST_ANQP_REQ", __func__);
		params = destMacStr;
	}

	if (pass_request_to_ltest(dut, cmnd, params) < 0) {
		/* Loc operation been failed. */
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Failed to initiate Loc command",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Failed to initiate Loc command");
		return 0;
	}

	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Succeeded to initiate Loc command", __func__);
	send_resp(dut, conn, SIGMA_COMPLETE, NULL);
	return 0;
}


int loc_cmd_sta_send_frame(struct sigma_dut *dut, struct sigma_conn *conn,
			   struct sigma_cmd *cmd)
{
	const char *address3Cmnd = WPA_ADDRESS_3_DISABLE;
	enum lowi_tst_cmd cmnd = LOWI_TST_NEIGHBOR_REPORT_REQ; /* Default */
	/* Mandatory arguments */
	const char *interface = get_param(cmd, "interface");
	const char *program = get_param(cmd, "program");
	const char *destMacStr = get_param(cmd, "destmac");
	const char *frameName = get_param(cmd, "FrameName");

	/* Optional Arguments */
	const char *locCivic = get_param(cmd, "askforloccivic");
	const char *lci = get_param(cmd, "askforlci");
	const char *fqdn = get_param(cmd, "AskForPublicIdentifierURI-FQDN");
	const char *address3 = get_param(cmd, "address3");

	const char *params = NULL;

	if (!program) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - No Program in Command! - Aborting",
				__func__);
		return -1;
	}

	if (!interface) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOC CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR, NULL);
		return 0;
	}

	if (!frameName) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOC CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR, NULL);
		return 0;
	}

	if (strcasecmp(frameName, "AnqpQuery") == 0 && !destMacStr) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOC CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR, NULL);
		return 0;
	}

	if (!locCivic)
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Command missing LocCivic", __func__);
	if (!lci)
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Command missing LCI", __func__);
	if (!fqdn)
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Command missing FQDN", __func__);
	if (!address3) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Command missing address3", __func__);
	} else {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "%s - address3: %s",
				__func__, address3);
		if (strcasecmp(address3, "FF:FF:FF:FF:FF:FF") == 0)
			address3Cmnd = WPA_ADDRESS_3_ENABLE;
		else
			address3Cmnd = WPA_ADDRESS_3_DISABLE;
	}

	if (strcasecmp(program, "loc") != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Unsupported Program: %s", __func__,
				program);
		send_resp(dut, conn, SIGMA_ERROR, NULL);
		return 0;
	}

	sigma_dut_print(dut, DUT_MSG_INFO, "%s - Triggering Frame: %s",
			__func__, frameName);
	if (strcasecmp(frameName, "AnqpQuery") == 0) {
		cmnd = LOWI_TST_ANQP_REQ;
		params = destMacStr;
	} else {
		cmnd = LOWI_TST_NEIGHBOR_REPORT_REQ;
	}

	if (cmnd == LOWI_TST_ANQP_REQ) {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "%s - Executing command %s",
				__func__, address3Cmnd);
		if (wpa_command(get_station_ifname(dut), address3Cmnd) < 0) {
			send_resp(dut, conn, SIGMA_ERROR, NULL);
			return -1;
		}
	}
	if (pass_request_to_ltest(dut, cmnd, params) < 0) {
		/* Loc operation has failed. */
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Failed to initiate Loc command",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR, NULL);
		return 0;
	}

	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Succeeded to initiate Loc command", __func__);
	send_resp(dut, conn, SIGMA_COMPLETE, NULL);
	return 0;
}


enum e_rm_parse_states {
	LOC_LOOKING_FOR_BIT = 0,
	LOC_LOOKING_FOR_VAL,
	LOC_MAX
};


void parse_rm_bits(struct sigma_dut *dut, const char *rmFlags,
		   char rmBitFlags[LOC_MAX_RM_FLAGS][LOC_RM_FLAG_VAL_ARRAY])
{

	unsigned int bitPos = 0;
	unsigned int bitVal = 0;
	unsigned int idx = 0;
	unsigned int i = 0;
	enum e_rm_parse_states rmParseStates = LOC_LOOKING_FOR_BIT;
	char temp = '\0';

	if (!rmFlags) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - NULL pointer for rmFlags - Aborting", __func__);
		return;
	}

	sigma_dut_print(dut, DUT_MSG_INFO, "%s - rmFlags: %s",
			__func__, rmFlags);
	while (*rmFlags != '\0' && idx < LOC_MAX_RM_FLAGS) {
		temp = *rmFlags;
		rmFlags++;
		switch (rmParseStates) {
		case LOC_LOOKING_FOR_BIT:
			if (temp >= '0' && temp <= '9') {
				/* Parse Digit for bit Position */
				bitPos = (bitPos * 10) + (temp - '0');
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_BIT - parsing: %c, bitPos: %u",
						__func__, temp, bitPos);
			} else if (temp == ':') {
				/* move to Parsing Bit Value */
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_BIT - processing: %c, bitPos: %u",
						__func__, temp, bitPos);
				rmBitFlags[idx][0] = bitPos;
				rmParseStates = LOC_LOOKING_FOR_VAL;
			} else if (temp == ';') {
				/* End of Bit-Value Pair, reset and look for New Bit Position */
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_BIT - processing: %c",
						__func__, temp);
				rmBitFlags[idx][0] = bitPos;
				/* rmBitFlags[idx][1] = bitVal; */
				bitPos = 0;
				bitVal = 0;
				idx++;
			} else { /* Ignore */
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_BIT - ignoring: %c",
						__func__, temp);
			}
			break;
		case LOC_LOOKING_FOR_VAL:
			if (temp == '0' || temp == '1') {
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_VAL - processing: %c",
						__func__, temp);
				bitVal = temp - '0';
				rmBitFlags[idx][1] = bitVal;
			} else if (temp == ';') {
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_VAL - processing: %c, bitPos: %u, bitVal: %u",
						__func__, temp, bitPos, bitVal);
				/* rmBitFlags[idx][0] = bitPos; */
				/* rmBitFlags[idx][1] = bitVal; */
				bitPos = 0;
				bitVal = 0;
				idx++;
				rmParseStates = LOC_LOOKING_FOR_BIT;
			} else { /* Ignore */
				sigma_dut_print(dut, DUT_MSG_INFO,
						"%s - LOC_LOOKING_FOR_VAL - ignoring: %c",
						__func__, temp);
			}
			break;
		default: /* Ignore */
			sigma_dut_print(dut, DUT_MSG_INFO,
					"%s - default - ignoring: %c",
					__func__, temp);
			break;
		}
	}

	for (i = 0; i < LOC_MAX_RM_FLAGS; i++) {
		sigma_dut_print(dut, DUT_MSG_INFO,
				"%s - Bit Pos: %u : Bit Val: %u",
				__func__, rmBitFlags[i][0],
				rmBitFlags[i][1]);
	}
}


int loc_cmd_sta_preset_testparameters(struct sigma_dut *dut,
				      struct sigma_conn *conn,
				      struct sigma_cmd *cmd)
{
	const char *rmFTMRFlagStr = get_param(cmd, "RMEnabledCapBitmap");
	const char *interworkingEn = get_param(cmd, "Interworking");
	unsigned int rmFTMRFlag = 0;
	unsigned int i, interworking = 0;
	char rmBitFlags[LOC_MAX_RM_FLAGS][LOC_RM_FLAG_VAL_ARRAY];

	sigma_dut_print(dut, DUT_MSG_INFO, "%s", __func__);

	memset(rmBitFlags, 0, sizeof(rmBitFlags));

	sigma_dut_print(dut, DUT_MSG_INFO, "%s - 1", __func__);
	/*
	 * This function is used to configure the RM capability bits and
	 * the Interworking bit only.
	 * If these parameters are not present just returning COMPLETE
	 * because all other parameters are ignored.
	 */
	if (!rmFTMRFlagStr && !interworkingEn) {
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - 2", __func__);
		sigma_dut_print(dut, DUT_MSG_ERROR, "%s - Did not get %s",
				__func__, "RMEnabledCapBitmap");
		send_resp(dut, conn, SIGMA_COMPLETE, NULL);
		return 0;
	}

	if (rmFTMRFlagStr) {
		rmFTMRFlag = 25; /* Default invalid */
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - rmFTMRFlagStr: %s",
				__func__, rmFTMRFlagStr);
		parse_rm_bits(dut, rmFTMRFlagStr, rmBitFlags);
		for (i = 0; i < LOC_MAX_RM_FLAGS; i++) {
			if (rmBitFlags[i][0] == 34)
				rmFTMRFlag = rmBitFlags[i][1];
		}
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - rmFTMRFlag %u",
				__func__, rmFTMRFlag);
		if (rmFTMRFlag == 0) { /* Disable RM - FTMRR capability */
			sigma_dut_print(dut, DUT_MSG_INFO,
					"%s - Disabling RM - FTMRR",
					__func__);
			if (wpa_command(get_station_ifname(dut),
					WPA_RM_DISABLE) < 0) {
				send_resp(dut, conn, SIGMA_ERROR, NULL);
				return -1;
			}
		} else if (rmFTMRFlag == 1) { /* Enable RM - FTMRR capability */
			sigma_dut_print(dut, DUT_MSG_INFO,
					"%s - Enabling RM - FTMRR",
					__func__);
			if (wpa_command(get_station_ifname(dut),
					WPA_RM_ENABLE) < 0) {
				send_resp(dut, conn, SIGMA_ERROR, NULL);
				return 0;
			}
		} else {
			sigma_dut_print(dut, DUT_MSG_ERROR,
					"%s - No Setting for - FTMRR",
					__func__);
		}
		sigma_dut_print(dut, DUT_MSG_INFO,
				"%s - Succeeded in Enabling/Disabling RM Capability for FTMRR",
				__func__);
	}

	if (interworkingEn) {
		sscanf(interworkingEn, "%u", &interworking);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - interworking: %u",
				__func__, interworking);
		if (interworking)
			wpa_command(get_station_ifname(dut),
				    WPA_INTERWORKING_ENABLE);
		else
			wpa_command(get_station_ifname(dut),
				    WPA_INTERWORKING_DISABLE);
	}

	send_resp(dut, conn, SIGMA_COMPLETE, NULL);
	return 0;
}


static enum sigma_cmd_result
lowi_cmd_sta_reset_ptksa_cache(struct sigma_dut *dut, struct sigma_conn *conn,
			       struct sigma_cmd *cmd)
{
	const char *intf = get_param(cmd, "Interface");
	char buf[1024], bssid[18], req[200], *pos;

	memset(req, 0, sizeof(req));
	memset(buf, 0, sizeof(buf));
	if (wpa_command_resp(intf, "PTKSA_CACHE_LIST", buf, sizeof(buf)) < 0 ||
	    strncmp(buf, "UNKNOWN COMMAND", 15) == 0) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrorCode,PTKSA_CACHE_LIST not supported");
		return STATUS_SENT_ERROR;
	}
	pos = buf;
	while (pos) {
		pos = strchr(pos, '\n');
		if (!pos)
			break;
		pos = strchr(pos, ' ');
		if (!pos)
			break;
		pos++;

		strlcpy(bssid, pos, sizeof(bssid));
		snprintf(req, sizeof(req), "PASN_DEAUTH bssid=%s", bssid);
		wpa_command(intf, req);
	}

	return STATUS_SENT;
}


int lowi_cmd_sta_reset_default(struct sigma_dut *dut, struct sigma_conn *conn,
				struct sigma_cmd *cmd)
{
#ifdef ANDROID_WIFI_HAL
	if (wifi_hal_initialize(dut)) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - wifihal init failed for - LOC",
				__func__);
		return -1;
	}
#endif /* ANDROID_WIFI_HAL */

	dut->i2rlmrpolicy = LOC_FORCE_FTM_I2R_LMR_POLICY;
	lowi_cmd_sta_reset_ptksa_cache(dut, conn, cmd);

	return 0;
}


static int loc_r2_set_11az_config(struct sigma_dut *dut, const char *dst_mac,
				  struct capi_loc_cmd *loc_cmd)
{
	FILE *xml;

	xml = fopen(LOC_11AZ_CONFIG_XML_FILE_PATH, "w");
	if (!xml) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Unable to create the XML file", __func__);
		return -1;
	}

	fprintf(xml, "<body>\n");
	fprintf(xml, "  <config_11az>\n");
	fprintf(xml, "    <i2rlmrpolicy>%u</i2rlmrpolicy>\n",
		dut->i2rlmrpolicy);
	fprintf(xml, "  </config_11az>\n");
	fprintf(xml, "</body>\n");

	fclose(xml);
	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Successfully created XML file", __func__);

	return system(LOC_LOWI_TEST_11AZ_CONFIG);
}


static int loc_r2_write_xml_file(struct sigma_dut *dut, const char *dst_mac,
				 struct capi_loc_cmd *loc_cmd)
{
	FILE *xml;
	unsigned int bw, preamble;
	unsigned int tmrtype = 0;
	unsigned int is_rtt_bg_node = 0;

	xml = fopen(LOC_XML_FILE_PATH, "w");
	if (!xml) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Unable to create the XML file", __func__);
		return -1;
	}

#define LOC_R2_BW_VHT_20  0
#define LOC_R2_BW_VHT_40  1
#define LOC_R2_BW_VHT_80  2
#define LOC_R2_BW_VHT_160 3

#define LOC_R2_PREAMBLE_HT 1
#define LOC_R2_PREAMBLE_VHT 2
#define LOC_R2_PREAMBLE_HE 3

#define LOC_R2_BW_20  0
#define LOC_R2_BW_40  1
#define LOC_R2_BW_80  2
#define LOC_R2_BW_160 3

	preamble = LOC_R2_PREAMBLE_HE;

	switch (loc_cmd->ftm_bw_rtt) {
	case LOC_R2_BW_VHT_20:
		bw = LOC_R2_BW_20;
		break;
	case LOC_R2_BW_VHT_40:
		bw = LOC_R2_BW_40;
		break;
	case LOC_R2_BW_VHT_80:
		bw = LOC_R2_BW_80;
		break;
	case LOC_R2_BW_VHT_160:
		bw = LOC_R2_BW_160;
		break;
	default:
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Bad Format/BW received", __func__);
		fclose(xml);
		return -1;
	}

#define LOC_R2_CAPI_DEFAULT_FTMS_PER_BURST 25
#define LOC_R2_CAPI_DEFAULT_BURST_DUR 10

	if (loc_cmd->ntb) {
		tmrtype = 1;
	} else if (loc_cmd->tb) {
		tmrtype = 2;
		is_rtt_bg_node = 1;
	}
	fprintf(xml, "<body>\n");
	fprintf(xml, "  <ranging>\n");
	fprintf(xml, "    <ap>\n");
	fprintf(xml, "    <rttType>3</rttType>\n");
	fprintf(xml, "    <numFrames>%u</numFrames>\n",
		LOC_R2_CAPI_DEFAULT_FTMS_PER_BURST);
	fprintf(xml, "    <bw>%u</bw>\n", bw);
	fprintf(xml, "    <preamble>%u</preamble>\n", preamble);
	fprintf(xml, "    <phymode>-1</phymode>\n");
	fprintf(xml, "    <asap>%u</asap>\n", loc_cmd->asap);
	fprintf(xml, "    <lci>%u</lci>\n", loc_cmd->lci);
	fprintf(xml, "    <civic>%u</civic>\n", loc_cmd->locCivic);
	/* Use parameters from LOWI cache */
	fprintf(xml, "    <paramControl>%u</paramControl>\n", 0);
	fprintf(xml, "    <ptsftimer>%u</ptsftimer>\n", 0);
	fprintf(xml, "    <frequency>%u</frequency>\n", loc_cmd->freq);
	fprintf(xml, "    <tmrtype>%u</tmrtype>\n", tmrtype);
	fprintf(xml, "    <sectype>%u</sectype>\n", 1);
	fprintf(xml, "    <tmrMinDelta>%u</tmrMinDelta>\n", 2500);
	fprintf(xml, "    <tmrMaxDelta>%u</tmrMaxDelta>\n", 300);
	fprintf(xml, "    <I2RFbPolicy>%u</I2RFbPolicy>\n", dut->i2rlmr_iftmr);
	fprintf(xml, "    <isRttBgNode>%u</isRttBgNode>\n", is_rtt_bg_node);
	fprintf(xml, "    <tbPeriodicity>%u</tbPeriodicity>\n", 4);
	fprintf(xml, "    <tbDuration>%u</tbDuration>\n", 80);
	fprintf(xml, "    <tbMaxSessionExpiry>%u</tbMaxSessionExpiry>\n", 5);
	fprintf(xml, "    <mac>%s</mac>\n", dst_mac);
	fprintf(xml, "    </ap>\n");
	fprintf(xml, "  </ranging>\n");
	fprintf(xml, "  <summary>\n");
	fprintf(xml, "    <mac>%s</mac>\n", dst_mac);
	fprintf(xml, "  </summary>\n");
	fprintf(xml, "</body>\n");

	dut->i2rlmr_iftmr = 0;

	fclose(xml);
	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Successfully created XML file", __func__);
	return 0;
}


static int loc_r2_get_bss_frequency(struct sigma_dut *dut,
				    struct sigma_conn *conn,
				    struct sigma_cmd *cmd)
{
	const char *intf = get_param(cmd, "Interface");
	const char *bssid;
	char buf[4096], *pos;
	int freq, res;
	struct wpa_ctrl *ctrl;

	bssid = get_param(cmd, "destmac");
	if (!bssid) {
		send_resp(dut, conn, SIGMA_INVALID,
			  "errorCode,destmac argument is missing");
		return 0;
	}

	ctrl = open_wpa_mon(intf);
	if (!ctrl) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"Failed to open wpa_supplicant monitor connection");
		return 0;
	}

	if (wpa_command(intf, "SCAN TYPE=ONLY")) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Could not start scan");
		wpa_ctrl_detach(ctrl);
		wpa_ctrl_close(ctrl);
		return 0;
	}

	res = get_wpa_cli_event(dut, ctrl, "CTRL-EVENT-SCAN-RESULTS",
				buf, sizeof(buf));

	wpa_ctrl_detach(ctrl);
	wpa_ctrl_close(ctrl);

	if (res < 0) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Scan did not complete");
		return 0;
	}

	snprintf(buf, sizeof(buf), "BSS %s", bssid);
	if (wpa_command_resp(intf, buf, buf, sizeof(buf)) < 0 ||
	    strncmp(buf, "id=", 3) != 0) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Specified BSSID not found");
		return 0;
	}

	pos = strstr(buf, "\nfreq=");
	if (!pos) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Channel not found");
		return 0;
	}
	freq = atoi(pos + 6);
	return freq;
}


int loc_r2_cmd_sta_exec_action(struct sigma_dut *dut, struct sigma_conn *conn,
			       struct sigma_cmd *cmd)
{
	const char *params = NULL;
	const char *program = get_param(cmd, "prog");
	const char *loc_op = get_param(cmd, "trigger");
	const char *interface = get_param(cmd, "interface");
	const char *dest_mac = get_param(cmd, "destmac");
	const char *ntb = get_param(cmd, "NTB");
	const char *tb = get_param(cmd, "TB");
	const char *ftm_bw_rtt = get_param(cmd, "FormatBWRanging");
	struct capi_loc_cmd loc_cmd;

	memset(&loc_cmd, 0, sizeof(loc_cmd));

	if (!loc_op) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - No Operation! - Aborting", __func__);
		return -1;
	}

	if (!program || strcasecmp(program, "LOCR2") != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - No LOCR2 Program in Command! - Aborting",
				__func__);
		return -1;
	}

	if (!interface || strcasecmp(interface, "wlan0") != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOCR2 CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Missing wlan0 Interface Type");
		return 0;
	}

	if (!dest_mac) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOCR2 CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Incomplete LOCR2 CAPI command - missing MAC");
		return 0;
	}

	if (!ftm_bw_rtt) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOCR2 CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Incomplete Loc CAPI command - missing Format & BW");
		return 0;
	}

	if (!ntb && !tb) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Incomplete command in LOCR2 CAPI request",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Incomplete Loc CAPI command - missing TMR Type(NTB or TB)");
		return 0;
	}

	if (ntb) {
		sscanf(ntb, "%u", &loc_cmd.ntb);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - ntb: %u",
				__func__, loc_cmd.ntb);
	} else {
		sscanf(tb, "%u", &loc_cmd.tb);
		sigma_dut_print(dut, DUT_MSG_INFO, "%s - tb: %u",
				__func__, loc_cmd.tb);
	}

	sscanf(ftm_bw_rtt, "%u", &loc_cmd.ftm_bw_rtt);
	sigma_dut_print(dut, DUT_MSG_INFO, "%s - ftmbw: %u",
			__func__, loc_cmd.ftm_bw_rtt);

	loc_cmd.freq = loc_r2_get_bss_frequency(dut, conn, cmd);
	sigma_dut_print(dut, DUT_MSG_INFO, "%s - freq: %u",
			__func__, loc_cmd.freq);

	if (loc_r2_write_xml_file(dut, dest_mac, &loc_cmd) < 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Failed to write to XML file because of bad command",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Bad CAPI command");
		return 0;
	}

	if (loc_r2_set_11az_config(dut, dest_mac, &loc_cmd) < 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Failed to set 11az config command",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Bad CAPI command");
		return 0;
	}
	sleep(1);

	if (pass_request_to_ltest(dut, LOWI_TST_RANGING, params) < 0) {
		/* Loc operation been failed. */
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"%s - Failed to initiate Loc command",
				__func__);
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrMsg,Failed to initiate Loc command");
		return 0;
	}

	sigma_dut_print(dut, DUT_MSG_INFO,
			"%s - Succeeded to initiate LOCR2 command", __func__);
	send_resp(dut, conn, SIGMA_COMPLETE, NULL);
	return 0;
}
