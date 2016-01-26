#########################################################################
# File Name: run_make.sh
# Author: Dmaker
# mail: dongfangyiyang@yeah.net
# Created Time: Tue 22 Dec 2015 08:44:48 AM CST
#########################################################################
#!/bin/bash
make LOADADDR=0x10008000 && rm -f /mnt/hgfs/deploy_files/zImage; cp -af ./arch/arm/boot/zImage /mnt/hgfs/deploy_files/ &&ls -l ./arch/arm/boot/zImage

cp -af ./arch/arm/boot/dts/imx6ul-14x14-evk.dtb /mnt/hgfs/deploy_files/ && cp -af ./drivers/mxc/magc/imx_magc.ko /mnt/hgfs/deploy_files/
