/*
 * Sigma Control API DUT (station/AP)
 * Copyright (c) 2010, Atheros Communications, Inc.
 * Copyright (c) 2011-2013, 2016-2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2018-2021, The Linux Foundation
 * All Rights Reserved.
 * Licensed under the Clear BSD license. See README for more details.
 */

#include "sigma_dut.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <ifaddrs.h>
#include <netdb.h>

#include "wpa_helpers.h"

#ifdef ANDROID
#define SHELL "/system/bin/sh"
#else /* ANDROID */
#define SHELL "/bin/sh"
#endif /* ANDROID */


static enum sigma_cmd_result cmd_traffic_send_ping(struct sigma_dut *dut,
						   struct sigma_conn *conn,
						   struct sigma_cmd *cmd)
{
	const char *dst, *val;
	int size, dur, pkts;
	int id;
	char resp[100];
	float interval;
	double rate;
	FILE *f;
	char buf[100];
	int type = 1;
	int dscp = 0, use_dscp = 0;
	char extra[100], int_arg[100], intf_arg[100], ip_dst[100], ping[100];
	struct in6_addr ip6_addr;
	bool broadcast = false;
	const char *iface;

	val = get_param(cmd, "Type");
	if (!val)
		val = get_param(cmd, "IPType");
	if (val)
		type = atoi(val);
	if (type != 1 && type != 2) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrorCode,Unsupported address type");
		return STATUS_SENT;
	}

	if (type == 2 && dut->program == PROGRAM_P2P)
		iface = get_p2p_group_ifname(dut, get_main_ifname(dut));
	else
		iface = get_station_ifname(dut);

	dst = get_param(cmd, "destination");
	if (dst == NULL || (type == 1 && !is_ip_addr(dst)) ||
	    (type == 2 && !is_ipv6_addr(dst)))
		return INVALID_SEND_STATUS;
	if (dut->ndp_enable && type == 2) {
		snprintf(ip_dst, sizeof(ip_dst), "%s%%nan0", dst);
		dst = ip_dst;
	} else if (type == 2) {
		if (inet_pton(AF_INET6, dst, &ip6_addr) <= 0) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Unsupported address type");
			return STATUS_SENT;
		}

		if (IN6_IS_ADDR_LINKLOCAL(&ip6_addr)) {
			snprintf(ip_dst, sizeof(ip_dst), "%s%%%s", dst, iface);
			dst = ip_dst;
		}
	}

	val = get_param(cmd, "frameSize");
	if (val == NULL)
		return INVALID_SEND_STATUS;
	size = atoi(val);
	if (type != 2 && strcmp(dst, BROADCAST_ADDR) == 0) {
		if (size > 1472) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Unsupported broadcast ping frame size");
			return STATUS_SENT;
		}
		broadcast = true;
		snprintf(buf, sizeof(buf),
			 "ip route add 255.255.255.255 dev %s",
			 get_station_ifname(dut));
		run_system(dut, buf);
	}

	val = get_param(cmd, "frameRate");
	if (val == NULL)
		return INVALID_SEND_STATUS;
	rate = atof(val);
	if (rate <= 0)
		return INVALID_SEND_STATUS;

	val = get_param(cmd, "duration");
	if (val == NULL)
		return INVALID_SEND_STATUS;
	dur = atoi(val);
	if (dur <= 0 || dur > 3600)
		dur = 3600;

	pkts = dur * rate;
	interval = (float) 1 / rate;
	if (interval > 100000)
		return INVALID_SEND_STATUS;

	val = get_param(cmd, "DSCP");
	if (val) {
		dscp = atoi(val);
		if (dscp < 0 || dscp > 63) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Invalid DSCP value");
			return STATUS_SENT;
		}
		use_dscp = 1;
	}

	id = dut->next_streamid++;
	snprintf(buf, sizeof(buf), "%s/sigma_dut-ping.%d",
		 dut->sigma_tmpdir, id);
	unlink(buf);
	snprintf(buf, sizeof(buf), "%s/sigma_dut-ping-pid.%d",
		 dut->sigma_tmpdir, id);
	unlink(buf);

	sigma_dut_print(dut, DUT_MSG_DEBUG, "Send ping: pkts=%d interval=%f "
			"streamid=%d",
			pkts, interval, id);

	f = fopen(concat_sigma_tmpdir(dut, "/sigma_dut-ping.sh", ping,
				      sizeof(ping)), "w");
	if (f == NULL)
		return ERROR_SEND_STATUS;

	extra[0] = '\0';
	if (use_dscp) {
		snprintf(extra, sizeof(extra), " -Q 0x%02x",
			 dscp << 2);
	}

	int_arg[0] = '\0';
	if (rate != 1)
		snprintf(int_arg, sizeof(int_arg), " -i %f", interval);
	if (!dut->ndp_enable && type == 2)
		snprintf(intf_arg, sizeof(intf_arg), " -I %s", iface);
	else
		intf_arg[0] = '\0';
	fprintf(f, "#!" SHELL "\n"
		"ping%s%s -c %d%s -s %d%s -q%s %s > %s"
		"/sigma_dut-ping.%d &\n"
		"echo $! > %s/sigma_dut-ping-pid.%d\n",
		type == 2 ? "6" : "", broadcast ? " -b" : "",
		pkts, int_arg, size, extra,
		intf_arg, dst, dut->sigma_tmpdir, id, dut->sigma_tmpdir, id);

	fclose(f);
	if (chmod(concat_sigma_tmpdir(dut, "/sigma_dut-ping.sh", ping,
				      sizeof(ping)),
		  S_IRUSR | S_IWUSR | S_IXUSR) < 0)
		return ERROR_SEND_STATUS;

	if (system(concat_sigma_tmpdir(dut, "/sigma_dut-ping.sh", ping,
				       sizeof(ping))) != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "Failed to start ping");
		return ERROR_SEND_STATUS;
	}

	unlink(concat_sigma_tmpdir(dut, "/sigma_dut-ping.sh", ping,
				   sizeof(ping)));

	snprintf(resp, sizeof(resp), "streamID,%d", id);
	send_resp(dut, conn, SIGMA_COMPLETE, resp);
	return STATUS_SENT;
}


static enum sigma_cmd_result cmd_traffic_stop_ping(struct sigma_dut *dut,
						   struct sigma_conn *conn,
						   struct sigma_cmd *cmd)
{
	const char *val;
	int id, pid;
	FILE *f;
	char buf[100];
	int res_found = 0, sent = 0, received = 0;

	val = get_param(cmd, "streamID");
	if (val == NULL)
		return INVALID_SEND_STATUS;
	id = atoi(val);

	snprintf(buf, sizeof(buf), "%s/sigma_dut-ping-pid.%d",
		 dut->sigma_tmpdir, id);
	f = fopen(buf, "r");
	if (f == NULL) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrorCode,Unknown streamID");
		return STATUS_SENT;
	}
	if (fscanf(f, "%d", &pid) != 1 || pid <= 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "No PID for ping process");
		fclose(f);
		unlink(buf);
		return ERROR_SEND_STATUS;
	}

	fclose(f);
	unlink(buf);

	sigma_dut_print(dut, DUT_MSG_DEBUG, "Ping process pid %d", pid);
	if (kill(pid, SIGINT) < 0 && errno != ESRCH) {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "kill failed: %s",
				strerror(errno));
	}
	usleep(250000);

	snprintf(buf, sizeof(buf), "%s/sigma_dut-ping.%d",
		 dut->sigma_tmpdir, id);
	f = fopen(buf, "r");
	if (f == NULL) {
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"No ping result file found");
		send_resp(dut, conn, SIGMA_COMPLETE, "sent,0,replies,0");
		return STATUS_SENT;
	}

	while (fgets(buf, sizeof(buf), f)) {
		char *pos;

		pos = strstr(buf, " packets transmitted");
		if (pos) {
			pos--;
			while (pos > buf && isdigit(pos[-1]))
				pos--;
			sent = atoi(pos);
			res_found = 1;
		}

		pos = strstr(buf, " packets received");
		if (pos == NULL)
			pos = strstr(buf, " received");
		if (pos) {
			pos--;
			while (pos > buf && isdigit(pos[-1]))
				pos--;
			received = atoi(pos);
			res_found = 1;
		}
	}
	fclose(f);
	snprintf(buf, sizeof(buf), "%s/sigma_dut-ping.%d",
		 dut->sigma_tmpdir, id);
	unlink(buf);

	if (!res_found) {
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"No ping results found");
		send_resp(dut, conn, SIGMA_COMPLETE, "sent,0,replies,0");
		return STATUS_SENT;
	}

	snprintf(buf, sizeof(buf), "sent,%d,replies,%d", sent, received);
	send_resp(dut, conn, SIGMA_COMPLETE, buf);
	return STATUS_SENT;
}


int get_ip_addr(const char *ifname, int ipv6, char *buf, size_t len)
{
	struct ifaddrs *ifa, *ifa_tmp;
	bool non_ll_addr_found = false;

	if (getifaddrs(&ifa) == -1)
		return -1;

	for (ifa_tmp = ifa; ifa_tmp; ifa_tmp = ifa_tmp->ifa_next) {
		if (!ifa_tmp->ifa_addr ||
		    strcasecmp(ifname, ifa_tmp->ifa_name) != 0)
			continue;

		if (!ipv6 && ifa_tmp->ifa_addr->sa_family == AF_INET) {
			struct sockaddr_in *in;

			in = (struct sockaddr_in *) ifa_tmp->ifa_addr;
			if (!inet_ntop(AF_INET, &in->sin_addr, buf, len))
				return -1;
			return 0;
		}

		if (ipv6 && ifa_tmp->ifa_addr->sa_family == AF_INET6) {
			struct sockaddr_in6 *in6;

			in6 = (struct sockaddr_in6 *) ifa_tmp->ifa_addr;

			/* get link local address if available */
			if (IN6_IS_ADDR_LINKLOCAL(&in6->sin6_addr)) {
				if (!inet_ntop(AF_INET6, &in6->sin6_addr, buf,
					       len))
					return -1;
				return 0;
			}

			if (!non_ll_addr_found &&
			    inet_ntop(AF_INET6, &in6->sin6_addr, buf, len))
				non_ll_addr_found = true;
		}
	}

	return non_ll_addr_found ? 0 : -1;
}


static int get_dscp_from_policy_table(struct sigma_dut *dut, int ip_version,
				      const char *domain_name,
				      const char *src_ip, int dst_port,
				      int src_port)
{
	struct dscp_policy_data *policy;
	char *suffix;
	int dscp = -1, max_score = 0, max_suffix_length = 0;
	int score, suffix_length;

	for (policy = dut->dscp_policy_table; policy; policy = policy->next) {
		if (strlen(policy->domain_name) == 0)
			continue;

		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"Found policy with domain name %s, ipver %d start_port %d, end_port %d, dst_port %d src_port %d, src_ip %s, dscp %d",
				policy->domain_name, policy->ip_version,
				policy->start_port, policy->end_port,
				policy->dst_port, policy->src_port,
				policy->src_ip, policy->dscp);
		/*
		 * Discard if suffix is not found or suffix is not in the end of
		 * the complete domain name.
		 */
		suffix = strstr(domain_name, policy->domain_name);
		if (!suffix)
			continue;

		suffix_length = strlen(suffix);
		if (suffix_length != strlen(policy->domain_name))
			continue;

		/* Calculate granularity score */
		score = 0;
		if (policy->ip_version) {
			if (ip_version == policy->ip_version)
				score++;
			else
				continue;
		}

		if (policy->start_port && policy->end_port) {
			if (dst_port >= policy->start_port &&
			    dst_port <= policy->end_port)
				score++;
			else
				continue;
		}

		if (policy->dst_port) {
			if (dst_port == policy->dst_port)
				score++;
			else
				continue;
		}

		if (policy->src_port) {
			if (src_port == policy->src_port)
				score++;
			else
				continue;
		}

		if (strlen(policy->src_ip)) {
			if (!strcmp(src_ip, policy->src_ip))
				score++;
			else
				continue;
		}

		if (score > max_score) {
			max_score = score;
			max_suffix_length = suffix_length;
			dscp = policy->dscp;
		} else if (score == max_score &&
			   suffix_length > max_suffix_length) {
			max_suffix_length = suffix_length;
			dscp = policy->dscp;
		}
	}

	if (dscp == -1)
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"Policy not found for %s", domain_name);
	else
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"DSCP for %s is %d", domain_name, dscp);

	return dscp;
}


static enum sigma_cmd_result cmd_traffic_start_iperf(struct sigma_dut *dut,
						     struct sigma_conn *conn,
						     struct sigma_cmd *cmd)
{
	const char *val, *dst, *domain_name = NULL;
	const char *iptype;
	int duration, dst_port = 0, src_port = 0;
	const char *proto;
	char buf[256];
	const char *ifname;
	char port_str[20], iperf[100], src_ip[100];
	FILE *f;
	int server, ipv6 = 0;
	char *pos;
	int dscp, reverse = 0, rate;
	char tos[20], client_port_str[100], bitrate[30], burst_size_str[50];
	struct hostent *host_addr;
	char ip_addr[INET6_ADDRSTRLEN];
	bool iperf_v2 = false;
	char iperf_result_file[50], iperf_pid_file[50], latency_str[50];
	int res;
	char iperf_cmd[300];

	val = get_param(cmd, "iperfversion");
	if (val && atoi(val) == 2)
		iperf_v2 = true;

	val = get_param(cmd, "mode");
	if (!val) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Missing mode parameter");
		return STATUS_SENT;
	}
	server = strcasecmp(val, "server") == 0;

	iptype = "";
	val = get_param(cmd, "iptype");
	if (val) {
		if (strcasecmp(val, "ipv6") == 0 ||
		    strcasecmp(val, "version6") == 0) {
			iptype = "-6";
			ipv6 = 1;
		} else {
			iptype = "-4";
			ipv6 = 0;
		}
	}

	port_str[0] = '\0';
	val = get_param(cmd, "port");
	if (val) {
		dst_port = atoi(val);
		snprintf(port_str, sizeof(port_str), "-p %d", dst_port);
	}

	rate = 1024 * 1024 * 1024; /* default rate: 1 Gbps */
	bitrate[0] = '\0';
	val = get_param(cmd, "bitrate");
	if (val) {
		int ret = snprintf(bitrate, sizeof(bitrate), " -b %s", val);
		size_t len;
		char rate_factor;

		if (ret < 0 || ret >= sizeof(bitrate))
			return ERROR_SEND_STATUS;

		rate = atoi(val);
		len = strlen(val);
		rate_factor = len > 0 ? val[len - 1] : 0;
		if (rate_factor == 'G')
			rate *= 1024 * 1024 * 1024;
		else if (rate_factor == 'M')
			rate *= 1024 * 1024;
		else if (rate_factor == 'K')
			rate *= 1024;
	}

	burst_size_str[0] = '\0';
	val = get_param(cmd, "burstsize");
	if (!iperf_v2 && val) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Invalid Iperf3 option BurstSize");
		return ERROR_SEND_STATUS;
	}
	if (val && atoi(val) > 0) {
		int fps, ret;

		fps = rate / (atoi(val) * 8);
		/* Use --isochronous to allow lower burst size. */
		ret = snprintf(burst_size_str, sizeof(burst_size_str),
			       " --isochronous=%d:%d,0 -w 2M", fps, rate);
		if (ret < 0 || ret >= sizeof(burst_size_str))
			return ERROR_SEND_STATUS;

		/* Isochronous and bitrate don't get configured together */
		bitrate[0] = '\0';
	}

	dst = get_param(cmd, "destination");
	pos = dst ? strchr(dst, '%') : NULL;
	if (pos) {
		*pos++ = '\0';
		ifname = pos;
	} else if (dut->ndpe || dut->program == PROGRAM_NAN) {
		ifname = "nan0";
	} else {
		ifname = get_station_ifname(dut);
	}

	if (dst && !ipv6 && !is_ip_addr(dst)) {
		domain_name = dst;
		host_addr = gethostbyname2(dst, AF_INET);
		if (!host_addr) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "errorCode,Invalid IPv4 address/domain name");
			return STATUS_SENT;
		}
		dst = inet_ntop(AF_INET, host_addr->h_addr_list[0], ip_addr,
				sizeof(ip_addr));
	} else if (dst && ipv6 && !is_ipv6_addr(dst)) {
		domain_name = dst;
		host_addr = gethostbyname2(dst, AF_INET6);
		if (!host_addr) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "errorCode,Invalid IPv6 address/domain name");
			return STATUS_SENT;
		}
		dst = inet_ntop(AF_INET6, host_addr->h_addr_list[0], ip_addr,
				sizeof(ip_addr));
	}

	if (!server && !dst) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Invalid destination address");
		return STATUS_SENT;
	}

	val = get_param(cmd, "duration");
	if (val)
		duration = atoi(val);
	else
		duration = 0;

	client_port_str[0] = '\0';
	src_ip[0] = '\0';
	val = get_param(cmd, "clientport");
	if (val) {
		src_port = atoi(val);
		if (get_ip_addr(ifname, ipv6, src_ip, sizeof(src_ip))) {
			send_resp(dut, conn, SIGMA_ERROR,
				"errorCode,Cannot get own IP address");
			return STATUS_SENT;
		}

		if (ipv6)
			snprintf(buf, sizeof(buf), "%s%%%s", src_ip, ifname);
		else
			snprintf(buf, sizeof(buf), "%s", src_ip);

		res = snprintf(client_port_str, sizeof(client_port_str),
			       " -B %s%s%d", buf,
			       iperf_v2 ? ":" : " --cport ",
			       src_port);
		if (res < 0 || res >= sizeof(client_port_str))
			return ERROR_SEND_STATUS;
	}

	val = get_param(cmd, "reverse");
	if (val)
		reverse = atoi(val);

	latency_str[0] = '\0';
	val = get_param(cmd, "latency");
	if (!iperf_v2 && val) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "ErrorCode, Iperf3 doesn't support latency params");
		return STATUS_SENT_ERROR;
	}
	if (val && atoi(val)) {
		const char *lower_ci, *upper_ci;

		val = get_param(cmd, "LowerCI");
		lower_ci = val ? val : "95";

		val = get_param(cmd, "UpperCI");
		upper_ci = val ? val : "99.9";

		if (server)
			res = snprintf(latency_str, sizeof(latency_str),
				       " --realtime --histograms=1m,1000000,%s,%s ",
				       lower_ci, upper_ci);
		else
			res = snprintf(latency_str, sizeof(latency_str),
				       " --realtime --trip-times ");
		if (res < 0 || res >= sizeof(latency_str))
			return ERROR_SEND_STATUS;
	}

	tos[0] = '\0';
	val = get_param(cmd, "DSCP");
	if (val) {
		dscp = atoi(val);
		if (dscp < 0 || dscp > 63) {
			send_resp(dut, conn, SIGMA_ERROR,
				  "ErrorCode,Invalid DSCP value");
			return STATUS_SENT_ERROR;
		}
		snprintf(tos, sizeof(tos), " -S 0x%02x", dscp << 2);
	}
	if (domain_name) {
		dscp = get_dscp_from_policy_table(dut, ipv6 ? IPV6 : IPV4,
						  domain_name, src_ip, dst_port,
						  src_port);
		if (dscp != -1)
			snprintf(tos, sizeof(tos), " -S 0x%02x", dscp << 2);
	}

	res = snprintf(iperf_result_file, sizeof(iperf_result_file),
		       "/sigma_dut-iperf-res-%d-%d", server, dst_port);
	if (res < 0 || res >= sizeof(iperf_result_file))
		return ERROR_SEND_STATUS;

	res = snprintf(iperf_pid_file, sizeof(iperf_pid_file),
		       "/sigma_dut-iperf-pid-%d-%d", server, dst_port);
	if (res < 0 || res >= sizeof(iperf_pid_file))
		return ERROR_SEND_STATUS;

	unlink(concat_sigma_tmpdir(dut, iperf_result_file, iperf,
				   sizeof(iperf)));
	unlink(concat_sigma_tmpdir(dut, iperf_pid_file, iperf,
				   sizeof(iperf)));

	/* open IPv6 multicast socket using iperf v2 */
	if (ipv6 && dst && (strncmp(dst, "ff", 2) == 0))
		iperf_v2 = true;

	if (iperf_v2)
		iptype = ipv6 ? "-V" : "";

	proto = "";
	val = get_param(cmd, "transproto");
	/* proto -u is not applicable for iperf3 server */
	if (val && strcasecmp(val, "udp") == 0  && !(!iperf_v2 && server))
		proto = "-u";

	if (server) {
		/* write server side command to shell file */
		if (ipv6 && dst && (strncmp(dst, "ff", 2) == 0)) {
			/* open IPv6 multicast server socket using iperf */
			snprintf(buf, sizeof(buf), "-B %s%%%s", dst, ifname);
		} else if (dst) {
			/* open IPv4 multicast server socket using iperf3 */
			snprintf(buf, sizeof(buf), "-B %s", dst);
		} else {
			buf[0] = '\0';
		}

		res = snprintf(iperf_cmd, sizeof(iperf_cmd),
			"iperf%s -s %s %s %s %s -i 1 %s > %s%s &\n",
			iperf_v2 ? "" : "3", port_str, iptype, proto,
			buf, latency_str, dut->sigma_tmpdir, iperf_result_file);
	} else {
		/* write client side command to shell file */
		if (!dst)
			return INVALID_SEND_STATUS;
		if (ipv6)
			snprintf(buf, sizeof(buf), "%s%%%s", dst, ifname);
		else
			snprintf(buf, sizeof(buf), "%s", dst);

		res = snprintf(iperf_cmd, sizeof(iperf_cmd),
			"iperf%s -c %s -t %d %s %s%s %s%s%s%s%s -i 1 %s > %s%s &\n",
			iperf_v2 ? "" : "3",
			buf, duration, iptype, proto, bitrate, port_str,
			client_port_str, tos, reverse ? " -R" : "",
			burst_size_str, latency_str, dut->sigma_tmpdir,
			iperf_result_file);
	}
	if (res < 0 || res >= sizeof(iperf_cmd))
		return ERROR_SEND_STATUS;

	f = fopen(concat_sigma_tmpdir(dut, "/sigma_dut-iperf.sh", iperf,
				      sizeof(iperf)), "w");
	if (!f) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Can not write sigma_dut-iperf.sh");
		return STATUS_SENT;
	}

	fprintf(f, "#!" SHELL "\n%s"
		"echo $! > %s%s\n",
		iperf_cmd, dut->sigma_tmpdir, iperf_pid_file);

	fclose(f);

	if (chmod(concat_sigma_tmpdir(dut, "/sigma_dut-iperf.sh", iperf,
				      sizeof(iperf)),
		  S_IRUSR | S_IWUSR | S_IXUSR) < 0) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Can not chmod sigma_dut-iperf.sh");
		return STATUS_SENT;
	}

	sigma_dut_print(dut, DUT_MSG_DEBUG, "Starting iperf. cmd: %s",
			iperf_cmd);
	if (system(concat_sigma_tmpdir(dut, "/sigma_dut-iperf.sh", iperf,
				       sizeof(iperf))) != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "Failed to start iperf");
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Failed to run sigma_dut-iperf.sh");
		return STATUS_SENT;
	}

	unlink(concat_sigma_tmpdir(dut, "/sigma_dut-iperf.sh", iperf,
				   sizeof(iperf)));
	return SUCCESS_SEND_STATUS;
}


static enum sigma_cmd_result cmd_traffic_stop_iperf(struct sigma_dut *dut,
						    struct sigma_conn *conn,
						    struct sigma_cmd *cmd)
{
	int pid;
	FILE *f;
	char buf[1024], summary_buf[1024], iperf[100], histogram_buf[1024];
	float bandwidth, totalbytes, factor;
	char *pos;
	long l_bandwidth, l_totalbytes;
	const char *val;
	const size_t max_fname = 50;
	char iperf_result_file[max_fname], iperf_pid_file[max_fname];
	DIR *dir;
	struct dirent *entry;
	int res, server;
	bool latency_expected = false;
	char *latency;

	val = get_param(cmd, "mode");
	server = val && strcasecmp(val, "server") == 0;

	val = get_param(cmd, "port");
	if (val) {
		int dst_port;

		dst_port = atoi(val);
		res = snprintf(iperf_result_file, sizeof(iperf_result_file),
			       "/sigma_dut-iperf-res-%d-%d", server, dst_port);
		if (res < 0 || res >= sizeof(iperf_result_file))
			return ERROR_SEND_STATUS;

		res = snprintf(iperf_pid_file, sizeof(iperf_pid_file),
			       "/sigma_dut-iperf-pid-%d-%d", server, dst_port);
		if (res < 0 || res >= sizeof(iperf_pid_file))
			return ERROR_SEND_STATUS;
	} else {
		/* Assume single instance and find the file using opendir(). */
		iperf_result_file[0] = '\0';
		iperf_pid_file[0] = '\0';

		dir = opendir(dut->sigma_tmpdir);
		if (!dir)
			return ERROR_SEND_STATUS;

		while ((entry = readdir(dir))) {
			if (strncmp(entry->d_name, "sigma_dut-iperf-res-",
				    20) == 0)
				res = snprintf(iperf_result_file, max_fname,
					       "/%s", entry->d_name);

			else if (strncmp(entry->d_name, "sigma_dut-iperf-pid-",
					 20) == 0)
				res = snprintf(iperf_pid_file, max_fname,
					       "/%s", entry->d_name);
			else
				continue;

			if (res < 0 || res >= max_fname)
				return ERROR_SEND_STATUS;
		}
		closedir(dir);

		if (!strlen(iperf_result_file) || !strlen(iperf_pid_file)) {
			sigma_dut_print(dut, DUT_MSG_INFO,
					"Could not find iperf result or PID file");
			return ERROR_SEND_STATUS;
		}
	}

	f = fopen(concat_sigma_tmpdir(dut, iperf_pid_file, iperf,
				      sizeof(iperf)), "r");
	if (!f) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,PID file does not exist");
		return STATUS_SENT;
	}
	if (fscanf(f, "%d", &pid) != 1 || pid <= 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "No PID for iperf process");
		fclose(f);
		unlink(concat_sigma_tmpdir(dut, iperf_pid_file, iperf,
					   sizeof(iperf)));
		return ERROR_SEND_STATUS;
	}

	fclose(f);
	unlink(concat_sigma_tmpdir(dut, iperf_pid_file, iperf,
				   sizeof(iperf)));

	if (kill(pid, SIGINT) < 0 && errno != ESRCH) {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "kill failed: %s",
				strerror(errno));
	}
	usleep(250000);

	/* parse iperf output which is stored in sigma_dut-iperf-res* */
	summary_buf[0] = '\0';
	histogram_buf[0] = '\0';
	f = fopen(concat_sigma_tmpdir(dut, iperf_result_file, iperf,
				      sizeof(iperf)), "r");
	if (!f) {
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"No iperf result file found");
		send_resp(dut, conn, SIGMA_COMPLETE,
			  "bandwidth,0,totalbytes,0,latency,0");
		return STATUS_SENT;
	}

	/* find the last line which has the received bytes summary */
	while (fgets(buf, sizeof(buf), f)) {
		char *pos;

		pos = strchr(buf, '\n');
		if (pos)
			*pos = '\0';
		sigma_dut_print(dut, DUT_MSG_DEBUG, "iperf: %s", buf);
		pos = strstr(buf, "out-of-order");
		if (pos)
			continue;

		pos = strstr(buf, " sec  ");
		if (pos)
			strlcpy(summary_buf, buf, sizeof(summary_buf));

		pos = strstr(buf, "clients should use --trip-times");
		if (pos)
			latency_expected = true;

		if (latency_expected) {
			pos = strstr(buf, "%=");
			if (pos)
				strlcpy(histogram_buf, buf,
					sizeof(histogram_buf));
		}
	}

	fclose(f);
	unlink(concat_sigma_tmpdir(dut, iperf_result_file, iperf,
				   sizeof(iperf)));

	res = snprintf(buf, sizeof(buf), "bandwidth,0,totalbytes,0%s",
		       latency_expected ? ",latency,0" : "");
	if (res < 0 || res >= sizeof(buf))
		return ERROR_SEND_STATUS;

	pos = strstr(summary_buf, "Bytes");
	if (!pos || pos == summary_buf) {
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"Can not parse iperf results: Bytes");
		send_resp(dut, conn, SIGMA_COMPLETE, buf);
		return STATUS_SENT;
	}

	if (pos[-1] == 'G')
		factor = 1024 * 1024 * 1024;
	else if (pos[-1] == 'M')
		factor = 1024 * 1024;
	else if (pos[-1] == 'K')
		factor = 1024;
	else
		factor = 1;

	if (pos) {
		pos -= 2;
		while (pos > summary_buf && (pos[-1] != ' '))
			pos--;
		totalbytes = atof(pos);
	} else
		totalbytes = 0;
	l_totalbytes = totalbytes * factor;

	pos = strstr(summary_buf, "bits/sec");
	if (!pos || pos == summary_buf) {
		sigma_dut_print(dut, DUT_MSG_DEBUG,
				"Can not parse iperf results: bits/sec");
		send_resp(dut, conn, SIGMA_COMPLETE, buf);
		return STATUS_SENT;
	}

	if (pos[-1] == 'G')
		factor = 1024 * 1024 * 1024 / 8;
	else if (pos[-1] == 'M')
		factor = 1024 * 1024 / 8;
	else if (pos[-1] == 'K')
		factor = 1024 / 8;
	else
		factor = 1 / 8;

	if (pos && pos - summary_buf > 2) {
		pos -= 2;
		while (pos > summary_buf && (pos[-1] != ' '))
			pos--;
		bandwidth = atof(pos);
	} else
		bandwidth = 0;
	l_bandwidth = bandwidth * factor;

	if (latency_expected) {
		pos = strstr(histogram_buf, "%=");
		if (!pos || pos == histogram_buf) {
			/* Not a fatal error. */
			sigma_dut_print(dut, DUT_MSG_DEBUG,
					"Skip iperf results for latency.");
			latency = "NA";
		} else {
			latency = pos + 2;
			pos = strstr(latency, ",");
			if (!pos || pos == latency) {
				sigma_dut_print(dut, DUT_MSG_DEBUG,
						"Can not parse iperf results: latency");
				send_resp(dut, conn, SIGMA_COMPLETE, buf);
				return STATUS_SENT_ERROR;
			}
			pos[0] = '\0';
		}
		res = snprintf(buf, sizeof(buf),
			       "bandwidth,%lu,totalbytes,%lu,latency,%s",
			       l_bandwidth, l_totalbytes, latency);
	} else {
		res = snprintf(buf, sizeof(buf),
			       "bandwidth,%lu,totalbytes,%lu",
			       l_bandwidth, l_totalbytes);
	}
	if (res < 0 || res >= sizeof(buf))
		return ERROR_SEND_STATUS;

	send_resp(dut, conn, SIGMA_COMPLETE, buf);
	return STATUS_SENT;
}


void traffic_register_cmds(void)
{
	sigma_dut_reg_cmd("traffic_send_ping", NULL, cmd_traffic_send_ping);
	sigma_dut_reg_cmd("traffic_stop_ping", NULL, cmd_traffic_stop_ping);
	sigma_dut_reg_cmd("traffic_start_iperf", NULL, cmd_traffic_start_iperf);
	sigma_dut_reg_cmd("traffic_stop_iperf", NULL, cmd_traffic_stop_iperf);
}
