# 顶层 Makefile for synology-build
# 依赖于已 source ./env.sh 设置好 KERNEL_SRC、ARCH、CROSS_COMPILE 等变量

.PHONY: all defconfig menuconfig build clean

all: build

defconfig:
	@echo "Using default config"
	@cd $(KERNEL_SRC) && make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) defconfig

menuconfig:
	@cd $(KERNEL_SRC) && make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) menuconfig

build:
	@echo "Start compiling kernel at $(KERNEL_SRC)"
	@cd $(KERNEL_SRC) && make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) Image dtbs modules -j$$(nproc)

clean:
	@cd $(KERNEL_SRC) && make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

mrproper:
	@cd $(KERNEL_SRC) && make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) mrproper

help:
	@echo "Usage:"
	@echo "  make defconfig   # 生成默认配置"
	@echo "  make menuconfig  # 配置内核"
	@echo "  make build       # 编译内核、dtb、模块"
	@echo "  make clean       # 清理编译文件"
	@echo "  make mrproper    # 深度清理"