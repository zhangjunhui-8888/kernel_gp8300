#!/bin/bash
export INSTALL_MOD_PATH=arch/gpt/support/initramfs
if [ "$1" = ""  ]
then
        echo "===null===="
else
        CONFIG_FILE=$1
fi

echo ===============================================
echo ==== build kernel file $CONFIG_FILE ==========
echo ===============================================
make ARCH=gpt clean 
export PATH=$PATH:./scripts/dtc/
make ARCH=gpt -j32
make ARCH=gpt modules_install 
./fit/mkimage -f ./fit/kernel_fdt.its gpt_gp8300_dvb.itb
echo ===============================================
echo === kernel build finished!!! ===============
echo ===============================================


