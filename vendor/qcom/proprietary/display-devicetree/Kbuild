ifeq ($(CONFIG_TARGET), neo_le)
dtbo-$(CONFIG_ARCH_NEO) += display/neo-sde.dtbo \
                display/neo-sde-display-idp-overlay.dtbo \
                display/neo-sde-display-qxr-overlay.dtbo \
                display/foreseer-sde-display-qxr-overlay.dtbo
endif

ifeq ($(CONFIG_TARGET), neo_la)
dtbo-$(CONFIG_ARCH_NEO) += display/neo-sde.dtbo \
                display/neo_la-sde-no-display-overlay.dtbo \
                display/neo_luna-v2-sde-display-idp-overlay.dtbo \
		display/neo_luna-v2-sde-display-sg-idp-overlay.dtbo
endif

ifneq ($(CONFIG_ARCH_QTI_VM), y)
dtbo-$(CONFIG_ARCH_WAIPIO) += display/waipio-sde.dtbo \
		display/waipio-sde-display-mtp-overlay.dtbo \
		display/waipio-sde-display-cphy-mtp-overlay.dtbo \
		display/waipio-sde-display-mtp-68disp-overlay.dtbo \
		display/waipio-sde-display-cdp-overlay.dtbo \
		display/waipio-sde-display-qrd-overlay.dtbo \
		display/waipio-sde-display-hdk-overlay.dtbo \
		display/waipio-sde-display-waipio-lemur-cdp-overlay.dtbo \
		display/waipio-sde-display-waipio-lemur-mtp-overlay.dtbo \
		display/waipio-sde-display-rumi-overlay.dtbo
else
dtbo-$(CONFIG_ARCH_WAIPIO) += display/trustedvm-waipio-sde-display-mtp-overlay.dtbo \
		display/trustedvm-waipio-sde-display-cdp-overlay.dtbo \
		display/trustedvm-waipio-sde-display-rumi-overlay.dtbo \
		display/trustedvm-waipio-sde-display-qrd-overlay.dtbo
endif

ifneq ($(CONFIG_ARCH_QTI_VM), y)
dtbo-$(CONFIG_ARCH_CAPE) += display/cape-sde.dtbo \
		display/cape-sde-display-atp-overlay.dtbo \
		display/cape-sde-display-cdp-overlay.dtbo \
		display/cape-sde-display-cdp-qhd-overlay.dtbo \
		display/cape-sde-display-mtp-overlay.dtbo \
		display/cape-sde-display-mtp-120fps-overlay.dtbo \
		display/cape-sde-display-mtp-nodisplay-overlay.dtbo \
		display/cape-sde-display-qrd-overlay.dtbo
else
dtbo-$(CONFIG_ARCH_CAPE) += display/trustedvm-cape-sde-display-qrd-overlay.dtbo \
		display/trustedvm-cape-sde-display-cdp-overlay.dtbo \
		display/trustedvm-cape-sde-display-cdp-qhd-overlay.dtbo \
		display/trustedvm-cape-sde-display-mtp-overlay.dtbo
endif

ifneq ($(CONFIG_ARCH_QTI_VM), y)
dtbo-$(CONFIG_ARCH_DIWALI) += display/diwali-sde.dtbo \
		display/diwali-sde-display-atp-overlay.dtbo \
		display/diwali-sde-display-idp-overlay.dtbo \
		display/diwali-sde-display-idp-amoled-overlay.dtbo \
		display/diwali-sde-display-qrd-overlay.dtbo \
		display/diwali-sde-display-rumi-overlay.dtbo
else
dtbo-$(CONFIG_ARCH_DIWALI) += display/trustedvm-diwali-sde-display-idp-overlay.dtbo \
		display/trustedvm-diwali-sde-display-qrd-overlay.dtbo
endif

ifneq ($(CONFIG_ARCH_QTI_VM), y)
dtbo-$(CONFIG_ARCH_PARROT) += display/parrot-sde.dtbo \
		display/parrot-sde-display-atp-overlay.dtbo \
		display/parrot-sde-display-idp-overlay.dtbo \
		display/parrot-sde-display-idp-amoled-overlay.dtbo \
		display/parrot-sde-display-rumi-overlay.dtbo \
		display/parrot-sde-display-qrd-overlay.dtbo
else
dtbo-$(CONFIG_ARCH_PARROT) += display/trustedvm-parrot-sde-display-idp-overlay.dtbo
endif

ifneq ($(CONFIG_ARCH_QTI_VM), y)
dtbo-$(CONFIG_ARCH_RAVELIN) += display/ravelin-sde.dtbo \
		display/ravelin-sde-display-atp-overlay.dtbo \
		display/ravelin-sde-display-idp-overlay.dtbo \
		display/ravelin-sde-display-idp-amoled-overlay.dtbo \
		display/ravelin-sde-display-rumi-overlay.dtbo \
		display/ravelin-sde-display-qrd-overlay.dtbo
else
dtbo-$(CONFIG_ARCH_RAVELIN) += display/trustedvm-ravelin-sde-display-idp-overlay.dtbo \
		display/trustedvm-ravelin-sde-display-idp-amoled-overlay.dtbo
endif

always-y    := $(dtb-y) $(dtbo-y)
subdir-y    := $(dts-dirs)
clean-files    := *.dtb *.dtbo
