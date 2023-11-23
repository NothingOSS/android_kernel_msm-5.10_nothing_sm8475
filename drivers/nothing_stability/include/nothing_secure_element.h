typedef enum {
	SECURE_OFF,
	SECURE_DEV,
	SECURE_PROD,
	SECURE_UNKNOWN,
} secure_type;

struct secure_data {
	struct device *dev;
	uint32_t oem_sec_boot_addr;
};
