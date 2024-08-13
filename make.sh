#!/bin/bash

export PATH=/root/bin/arm/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin:$PATH
export ARCH=arm64
export CROSS_COMPILE=arm-linux-

DESTDIR="/dev/shm/"

corenum=`grep -c processor /proc/cpuinfo`
to_build_modules=false
cur_dir=$(cd "$(dirname "$0")"; pwd)

[ "$1" = "modules" ] && to_build_modules=true

build_wifi_drv() {
    export KERNELDIR="$cur_dir"
    dir="3rdparty/mwifiex/mxm_wifiex/wlan_src"

    kernelver=`make kernelversion`

    cd $dir
    [ $? != 0 ] && exit 1
    make -j$corenum
    [ $? != 0 ] && exit 1

    for d in $DESTDIR; do
        ! [ -d "$d" ] && continue
        dir="$d/lib/modules/$kernelver/kernel/drivers/net/wireless"
        ! [ -d "$dir" ] && continue
        ! [ -d "$dir/nxp" ] && mkdir $dir/nxp
        rsync -avz *.ko $dir/nxp
        [ $? != 0 ] && exit 1

        depmod -b $d $kernelver
        [ $? != 0 ] && exit 1
    done
    make distclean >/dev/null 2>&1
    cd - >/dev/null
}

build_imx8mp() {
    SRC0DTB="arch/arm64/boot/dts/freescale/emtop-imx8mp-baseboard-hdmi.dtb"
    DST0DTB="emtop-imx8mp-baseboard-hdmi.dtb"
    SRC1DTB="arch/arm64/boot/dts/freescale/emtop-imx8mp-baseboard-mipi-dsi.dtb"
    DST1DTB="emtop-imx8mp-baseboard-mipi-dsi.dtb"
    SRC2DTB="arch/arm64/boot/dts/freescale/emtop-imx8mp-baseboard-lvds.dtb"
    DST2DTB="emtop-imx8mp-baseboard-lvds.dtb"
    SRC3DTB="arch/arm64/boot/dts/freescale/emtop-imx8mp-baseboard-mp.dtb"
    DST3DTB="emtop-imx8mp-baseboard-mp.dtb"
    SRCKER="arch/arm64/boot/Image"
    DSTKER="Image"

    SRCDTBS=($SRC0DTB $SRC1DTB $SRC2DTB $SRC3DTB)
    DSTDTBS=($DST0DTB $DST1DTB $DST2DTB $DST3DTB)

    for i in ${SRCDTBS[@]};do DTBS+=($(basename $i)); done

    if ! [ -f ".config" ]; then
        make emtop_imx8mp_baseboard_defconfig
        [ $? != 0 ] && exit 1
    fi

    if [ "$to_build_modules" = true ]; then
        make dtbs Image modules -j$corenum
        [ $? != 0 ] && exit 1
        make INSTALL_MOD_PATH=/dev/shm/ modules_install
        [ $? != 0 ] && exit 1

        build_wifi_drv
    else
        make dtbs Image -j$corenum
        [ $? != 0 ] && exit 1
    fi
    for d in $DESTDIR; do
        ! [ -d "$d" ] && continue
        for ((i = 0; i < ${#SRCDTBS[@]}; i++)) {
            echo "Info: COPY ${SRCDTBS[$i]} -> ${d}/${DSTDTBS[$i]}"
            cp -f ${SRCDTBS[$i]} ${d}/${DSTDTBS[$i]}
        }
        echo "Info: COPY ${SRCKER} ->  ${d}/${DSTKER}"
        cp -f ${SRCKER} ${d}/${DSTKER}
    done
}

# main entry
build_imx8mp
