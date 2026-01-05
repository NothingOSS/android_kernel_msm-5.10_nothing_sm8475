/*
 * Sigma Control API DUT (DNS service discovery functionality)
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc.
 * All Rights Reserved.
 * Licensed under the Clear BSD license. See README for more details.
 */

#include "sigma_dut.h"
#include <dlfcn.h>

static void * get_dl_sym(struct sigma_dut *dut, void *handle,
			 const char *symbol)
{
	void *sym = dlsym(handle, symbol);

	if (!sym)
		sigma_dut_print(dut, DUT_MSG_ERROR, "Could not resolve %s",
				symbol);

	return sym;
}

int mdnssd_init(struct sigma_dut *dut)
{
	dut->mdnssd_so = dlopen("/vendor/lib64/libmdnssd.so", RTLD_NOW);

	if (!dut->mdnssd_so) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"Could not open libmdnssd.so");
		return -1;
	}

	dut->mdnssd.service_create_connection = get_dl_sym(
		dut, dut->mdnssd_so, "DNSServiceCreateConnection");
	dut->mdnssd.service_socket_fd = get_dl_sym(dut, dut->mdnssd_so,
						   "DNSServiceRefSockFD");
	dut->mdnssd.service_process_result = get_dl_sym(
		dut, dut->mdnssd_so, "DNSServiceProcessResult");
	dut->mdnssd.service_register = get_dl_sym(dut, dut->mdnssd_so,
						  "DNSServiceRegister");
	dut->mdnssd.service_deallocate = get_dl_sym(dut, dut->mdnssd_so,
						    "DNSServiceRefDeallocate");
	dut->mdnssd.service_browse = get_dl_sym(dut, dut->mdnssd_so,
						"DNSServiceBrowse");
	dut->mdnssd.service_resolve = get_dl_sym(dut, dut->mdnssd_so,
						 "DNSServiceResolve");
	dut->mdnssd.get_addr_info = get_dl_sym(dut, dut->mdnssd_so,
					       "DNSServiceGetAddrInfo");
	dut->mdnssd.txt_create = get_dl_sym(dut, dut->mdnssd_so,
					    "TXTRecordCreate");
	dut->mdnssd.txt_set_value = get_dl_sym(dut, dut->mdnssd_so,
					       "TXTRecordSetValue");
	dut->mdnssd.txt_deallocate = get_dl_sym(dut, dut->mdnssd_so,
						"TXTRecordDeallocate");
	dut->mdnssd.txt_contains_key = get_dl_sym(dut, dut->mdnssd_so,
						  "TXTRecordContainsKey");
	dut->mdnssd.txt_get_value  = get_dl_sym(dut, dut->mdnssd_so,
						"TXTRecordGetValuePtr");
	dut->mdnssd.txt_get_length = get_dl_sym(dut, dut->mdnssd_so,
						"TXTRecordGetLength");
	dut->mdnssd.txt_get_bytes = get_dl_sym(dut, dut->mdnssd_so,
					       "TXTRecordGetBytesPtr");

	if (!dut->mdnssd.service_create_connection ||
	    !dut->mdnssd.service_socket_fd ||
	    !dut->mdnssd.service_process_result ||
	    !dut->mdnssd.service_register ||
	    !dut->mdnssd.service_deallocate ||
	    !dut->mdnssd.service_browse ||
	    !dut->mdnssd.service_resolve ||
	    !dut->mdnssd.get_addr_info ||
	    !dut->mdnssd.txt_create ||
	    !dut->mdnssd.txt_set_value ||
	    !dut->mdnssd.txt_deallocate ||
	    !dut->mdnssd.txt_contains_key ||
	    !dut->mdnssd.txt_get_value ||
	    !dut->mdnssd.txt_get_length ||
	    !dut->mdnssd.txt_get_bytes) {
		sigma_dut_print(dut, DUT_MSG_ERROR,
				"Could not resolve needed symbol from libmdnssd.so");
		memset(&dut->mdnssd, 0, sizeof(dut->mdnssd));
		dlclose(dut->mdnssd_so);
		dut->mdnssd_so = NULL;
		return -1;
	}

	sigma_dut_print(dut, DUT_MSG_INFO, "Successfully loaded libmdnssd.so");

	return 0;
}
