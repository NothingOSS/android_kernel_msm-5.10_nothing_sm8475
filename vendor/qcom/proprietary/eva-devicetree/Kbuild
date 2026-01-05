ifeq ($(CONFIG_ARCH_WAIPIO), y)
dtbo-y += waipio-eva.dtbo
endif

ifeq ($(CONFIG_ARCH_CAPE), y)
dtbo-y += ukee-eva.dtbo
dtbo-y += cape-eva.dtbo
endif

always-y	:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
