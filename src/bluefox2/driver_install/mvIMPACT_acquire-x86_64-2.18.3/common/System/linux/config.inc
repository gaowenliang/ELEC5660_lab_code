#! /bin/make include file
# -*- sh -*-
# global configuration settings for all impact Makefiles

## defaults

ifeq ($(TARGET),)
	TARGET=ppc603e
endif

##--- (cross/native) compiler homes -----------------------------

GCC_$(HOSTTYPE)Xx86	= /usr
GCC_$(HOSTTYPE)Xx86_64	= /usr
GCC_$(HOSTTYPE)Xppc603e	= /opt/xgcc3.2/ppc603e
GCC_$(HOSTTYPE)Xcygwin	= /opt/xgcc3.2/ppc603e
GCC_$(HOSTTYPE)Xppc_6xx	= /opt/eldk/usr
GCC_$(HOSTTYPE)Xppc	= /usr
GCC_$(HOSTTYPE)Xoeppc	= /usr/local/angstrom/powerpc
GCC_$(HOSTTYPE)Xarmv7l	= /usr

# alternative ARM toolchains
#GCC_$(HOSTTYPE)Xarm	= /usr/local/arm/2.95.3
#Gentoo toolchain
#GCC_$(HOSTTYPE)Xarm	= /usr
# scratchbox
#GCC_$(HOSTTYPE)Xarm	= /data/mvbf/skidata/scratchbox/compilers/arm-linux-gcc3.4.cs-glibc2.3
#embeddedarm
#GCC_$(HOSTTYPE)Xarm	= /data/mvbf/assembly/usr/local/opt/crosstool/arm-linux/gcc-3.3.4-glibc-2.3.2
# IDEXX Toolchain
GCC_$(HOSTTYPE)Xarm	= /usr/local/swdevel/tools

# mvBlueLYNX-X
# If OE_HOME is defined, use toolchain from OpenEmbedded,
# otherwise presume that the SDK is installed.
ifeq ($(TARGET),armv7a)
	ifneq ($(OE_HOME),)
		SYSTYPE=$(shell uname -m)
		GCC_$(HOSTTYPE)Xarmv7a = $(OE_HOME)/tmp/sysroots/$(SYSTYPE)-linux/usr/armv7a/
	else
		ifneq ($(SDK_PATH),)
			GCC_$(HOSTTYPE)Xarmv7a = $(SDK_PATH)
		else
$(warning "Need to have OE_HOME or SDK_PATH defined to find GCC toolchain")
		endif
	endif
endif

ifeq ($(TARGET),armv6zk)
	ifneq ($(SDK_PATH),)
		GCC_$(HOSTTYPE)Xarmv6zk = $(SDK_PATH)
	else
$(warning "Need to have SDK_PATH defined to find GCC toolchain")
	endif
endif

ifeq ($(TARGET),armv7ahf)
	ifneq ($(SDK_PATH),)
		GCC_$(HOSTTYPE)Xarmv7ahf = $(SDK_PATH)
	else
		GCC_$(HOSTTYPE)Xarmv7ahf = /usr/local/oecore-i686/sysroots/i686-angstromsdk-linux/usr
	endif
endif

ifeq ($(TARGET),armhf)
	GCC_$(HOSTTYPE)Xarmhf = /usr
endif

ifeq ($(TARGET),armsf)
	GCC_$(HOSTTYPE)Xarmsf = /usr
endif

ifeq ($(TARGET),arm64)
	GCC_$(HOSTTYPE)Xarm64 = /usr
endif

###
