#!/bin/sh

# Set BUILDROOT_BASE to the base of the buildroot directory. Make sure
# that the toolchain has been built.
if [ -n $BUILDROOT_BASE]
then
	BUILDROOT_BASE=../buildroot
fi

# If you run make menuconfig, be sure to copy .config to arch/arm/configs/rmds-camera_defconfig
# or this next line will overwrite it.
make ARCH=arm leopardboard_defconfig

# Build the uImage. The CodeSourcery tools need to be in the path
make -j8 ARCH=arm CROSS_COMPILE=$BUILDROOT_BASE/output/host/usr/bin/arm-none-linux-gnueabi- uImage
