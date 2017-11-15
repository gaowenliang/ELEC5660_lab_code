#! /bin/make
##############################################
# Makefile for the 'mvDeviceConfigure' sample,
##############################################
#-------------------------------------------
DEST:=$(shell basename `pwd`)

#-------------------------------------------
ifndef MVIMPACT_ACQUIRE_SOURCE_DIR
    ifndef MVIMPACT_ACQUIRE_DIR
        ROOT_PATH=../..
    else
        ROOT_PATH=$(MVIMPACT_ACQUIRE_DIR)/apps
    endif
else
    ROOT_PATH=$(MVIMPACT_ACQUIRE_SOURCE_DIR)/apps
endif

#-------------------------------------------
BINARY_NAME=mvDeviceConfigure
MODULE_NAME=DeviceConfigureFrame

#-------------------------------------------
OOBJ = \
	DebugFileParser.o \
	DeviceHandler.o \
	DeviceHandlerBlueDevice.o \
	DeviceHandlerBlueFOX.o \
	DeviceHandlerHYPERION.o \
	DeviceListCtrl.o \
	Epcs.o \
	KernelDriverUpdate.o \
	LogOutputConfigurationDlg.o \
	LogOutputHandlerDlg.o \
	PackageDescriptionParser.o \
	Timeout.o

#-------------------------------------------
vpath %.cpp ..:../linux:../../Common/FirmwareUpdate_mvHYPERION

#-----------------------------------------
LOC_LIB_PATH=$(WORK)/lib/$(TARGET)

#-------------------------------------------
# 'WX_CONF_TOOL' will be set in Makefile.samp.inc
#-------------------------------------------
WXRELEASE=$(shell $(WX_CONF_TOOL) --release)
WXLIBFLAGS=$(shell $(WX_CONF_TOOL) --libs)
WXCXXFLAGS=$(shell $(WX_CONF_TOOL) --cxxflags)

#-------------------------------------------
USR_INCLS =				\

#-------------------------------------------
USR_LIBS = \
	-lexpat \
	$(ADD_LIBS)

#-------------------------------------------
LDOPT_1 =				\
	$(WXLIBFLAGS)			\

ifneq ($(DEST),ppc603e)
LOCALFLAGS=				\
	-Wno-variadic-macros -Wno-strict-aliasing
endif

#-------------------------------------------
# Local typedefs warning since gcc version 4.8 (wxWidgets code)
GCC_MAJOR          := $(shell g++ -dumpversion 2>&1 | cut -d'.' -f1)
GCC_MINOR          := $(shell g++ -dumpversion 2>&1 | cut -d'.' -f2)
NOLOCALTYPEDEFWARNING := $(shell if [ $(GCC_MAJOR) -ge 4 ] && [ $(GCC_MINOR) -ge 8 ]; then		\
						 echo "-Wno-unused-local-typedefs";			\
						 else										\
						 echo "";									\
						 fi )
#-------------------------------------------
CPPFLAGS_1 = $(NOLOCALTYPEDEFWARNING) 	\
	-pedantic -Wno-long-long            \
	$(WXCXXFLAGS)			            \
	$(LOCALFLAGS)

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc

#-------------------------------------------
