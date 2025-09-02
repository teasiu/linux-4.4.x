ifeq ($(CONFIG_SYNO_LSP_RTD129X), y)
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-ds418.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1293-synology-ds418j.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-ds218play.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-ds118.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-ds218.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-eds19.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-rs819.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-ds220j.dtb
dtb-$(CONFIG_ARCH_RTD129X) += rtd-1296-synology-ds420j.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
endif # CONFIG_SYNO_LSP_RTD129X
ifeq ($(CONFIG_SYNO_LSP_RTD1619), y)
dts-dirs += rtd16xx

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= rtd16xx/*.dtb *.dtb
endif # CONFIG_SYNO_LSP_RTD1619
