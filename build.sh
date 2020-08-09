#!/bin/bash
args="-j$(nproc --all) \
O=out \
ARCH=arm64 \
SUBARCH=arm64 \
CLANG_TRIPLE=aarch64-linux-gnu- \
CROSS_COMPILE=${HOME}/clang-proton/bin/aarch64-linux-gnu- \
CC=${HOME}/clang-proton/bin/clang \
CROSS_COMPILE_ARM32=${HOME}/clang-proton/bin/arm-linux-gnueabi-"
make ${args} oneplus5_defconfig
make ${args}
