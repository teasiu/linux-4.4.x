#!/bin/bash

# 获取当前脚本绝对路径（即synology-build目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 工具链主目录
export TOOLCHAIN_ROOT="${SCRIPT_DIR}/toolchain/aarch64-unknown-linux-gnu"

# 交叉编译器前缀
export CROSS_COMPILE="aarch64-unknown-linux-gnu-"

# 工具链 bin 目录加入 PATH
export PATH="${TOOLCHAIN_ROOT}/bin:$PATH"

# SYSROOT 路径
export SYSROOT="${TOOLCHAIN_ROOT}/aarch64-unknown-linux-gnu/sysroot"

# 内核源码路径
export KERNEL_SRC="${SCRIPT_DIR}/kernel/linux-4.4.x"

# 编译架构
export ARCH=arm64

# 可选：显示当前 GCC 工具链版本
if command -v "${TOOLCHAIN_ROOT}/bin/${CROSS_COMPILE}gcc" >/dev/null 2>&1; then
    echo "GCC version: $("${TOOLCHAIN_ROOT}/bin/${CROSS_COMPILE}gcc" --version | head -n 1)"
else
    echo "Warning: ${TOOLCHAIN_ROOT}/bin/${CROSS_COMPILE}gcc not found!"
fi

echo "环境变量已设置："
echo "  工具链路径: $TOOLCHAIN_ROOT"
echo "  编译器前缀: $CROSS_COMPILE"
echo "  SYSROOT: $SYSROOT"
echo "  PATH: $PATH"
echo "  内核源码: $KERNEL_SRC"
echo "  ARCH: $ARCH"