#! /bin/make
##############################################
# common make definitions for samples
##############################################
#-----------------------------------------
TARGET:=$(shell basename `pwd`)
OSTYPE := $(shell uname | cut -d _ -f 1 | tr [:upper:] [:lower:])
HOSTTYPE := $(shell uname -m)

#-------------------------------------------
WORK=$(ROOT_PATH)/..
COMMON_PATH=$(WORK)/common
SYSTEM_INC_PATH=$(COMMON_PATH)/System
SYSTEM_PATH=$(SYSTEM_INC_PATH)/linux

#-------------------------------------------
include $(SYSTEM_PATH)/Makefile.all.inc

#-------------------------------------------
SOURCES= $(MODULE_NAME).cpp
OBJECTS= $(MODULE_NAME).o
HEADERS= *.h

ifeq ($(BINARY_NAME),)
	BINARY_NAME=$(MODULE_NAME)
endif

#-------------------------------------------
#-------------------------------------------
## ATENTION!!
# Feature in the glibc!
# *** glibc detected *** free(): invalid pointer: 0xXXXXXXXX ***
# You can control it on a per-run basis by setting the MALLOC_CHECK_ environment variable.
# If you explicitly set this variable it performs even more strict tests than normal.
# 0 -- Do not generate an error message, and do not kill the program
# 1 -- Generate an error message, but do not kill the program
# 2 -- Do not generate an error message, but kill the program
# 3 -- Generate an error message and kill the program
# need you need to: export MALLOC_CHECK_=1
#-------------------------------------------
#-------------------------------------------

#-------------------------------------------
############################################
# Entries for the different target-types
#===========================================
############################################
# libs
#===========================================
IMPACTLIB_PATH=$(mvIMPACT)/lib/$(IPL_LIB_TRGT)

#-------------------------------------------
LOCDEFINES=						\

#-------------------------------------------
INCLFILES=						\
	$(GLOBINCFILES)				\
	-I$(WORK)					\
	$(USR_INCLS)				\

#---------------------------------------------
ifeq ($(HOSTTYPE),x86_64)
	ifeq ($(TARGET),x86)
		# building 32 bit on 64 bit system
		COMP_OPTIONS_32BIT=-m32
	endif
endif

#-------------------------------------------
CPPFLAGS=					\
	$(CPPFLAGS_2)			\
	$(COMP_OPTIONS_32BIT)	\
	$(COMP_OPTIONS_64BIT)	\
    $(LOCDEFINES)			\
    $(INCLFILES)

#-------------------------------------------
ifeq ($(NO_MV_DEVICE_LIBS),)
# You should only need to link to the device manager but you also need libmvPropHandling when cross-compiling
MV_LIBS +=											\
	-Wl,-unresolved-symbols=ignore-in-shared-libs	\
	-L$(WORK)/lib/$(TARGET)							\
	-lmvDeviceManager								\
	-lmvPropHandling
endif

#-------------------------------------------
LDFLAGS = 				\
	$(MV_LIBS)			\
	$(LDOPT_1)			\
	$(ARCH_LDFLAGS)			\
	$(USR_LIBS)			\
	$(COMP_OPTIONS_32BIT)		\
	$(COMP_OPTIONS_64BIT)		\
	-lm				\
	-lpthread			\

#-----------------------------------------
# You may need to adapt this path for your system.
# If $WX_CONF_TOOL is set by the environment then it is just used.
# Otherwise, by default we use "which" except on a mvBL or if a special purpose-built wxWidgets exists.
#-----------------------------------------
ifeq ($(strip $(WX_CONF_TOOL)),)
	ifeq ($(TARGET),ppc603e)
		WX_CONF_PATH=$(GCCPATH)/bin/
		WX_CONF_TOOL=$(shell if [ -x $(WX_CONF_PATH)wx-config ]; then echo $(WX_CONF_PATH)wx-config; fi )
		ADD_LIBS=							\
			-lz -lX11 -lXext -lpng -ljpeg
	else ifeq ($(TARGET),armv7a) # mvBL-X
		ifneq ($(OE_HOME),)
		# if OE_HOME is defined, use OE tools
			#WX_CONF_TOOL=$(OE_HOME)/tmp/sysroots/$(shell uname -m)-linux/usr/armv7a/bin/wx-config
			WX_CONF_TOOL=$(GCCPATH)/bin/wx-config
		else
			ifneq ($(SDK_PATH),)
				BLX_SDK=$(SDK_PATH)
				WX_CONF_TOOL=$(BLX_SDK)/arm-angstrom-linux-gnueabi/usr/bin/wx-config
			endif
		endif
	else
		# check for specially-built static wxWidgets
		WX_SPECIAL_CONF_TOOL:=$(shell ls /opt/wx/2.6/$(TARGET)/bin/wx-config 2>/dev/null)
		ifneq ($(WX_SPECIAL_CONF_TOOL),)
			WX_CONF_TOOL:=$(WX_SPECIAL_CONF_TOOL)
		else
			# use the system's wxWidgets
			WX_CONF_TOOL:=$(shell which wx-config-3.0 2>/dev/null)
			ifeq ($(WX_CONF_TOOL),)
				WX_CONF_TOOL:=$(shell which wx-config-2.8 2>/dev/null)
				ifeq ($(WX_CONF_TOOL),)
					WX_CONF_TOOL:=$(shell which wx-config-2.6 2>/dev/null)
					ifeq ($(WX_CONF_TOOL),)
						WX_CONF_TOOL:=$(shell which wx-config 2>/dev/null)
					endif
				endif
			endif
		endif
		ADD_LIBS=
	endif
endif

#-----------------------------------------
# You may need to adapt this path for your system.
# By default uses "which" except when crosscompiling for 32 bit on 64-bit systems
#-----------------------------------------
ifeq ($(strip $(FLTK_CONF_TOOL)),)
	ifeq ($(TARGET),ppc603e)
		FLTK_CONF_PATH=$(GCCPATH)/bin/
		FLTK_CONF_TOOL=$(shell if [ -x $(FLTK_CONF_PATH)fltk-config ]; then echo $(FLTK_CONF_PATH)fltk-config; fi )
	else
		ifeq ($(filter $(TARGET),i686 x86),$(TARGET))
			FLTK_CONF_TOOL=$(shell which fltk-config 2>/dev/null)
		else
			ifeq ($(HOSTTYPE),x86_64)
				ifeq ($(TARGET),x86)
					FLTK_CONF_TOOL=$(WORK)/tools/fltk32/bin/fltk-config
				else
					FLTK_CONF_TOOL=$(shell which fltk-config 2>/dev/null)
				endif
			endif
		endif
	endif
endif

ifeq ($(strip $(WX_CONF_TOOL)),)
	ifeq ($(MODULE_NAME),$(filter $(MODULE_NAME),PropViewFrame DeviceConfigureFrame IPConfigureFrame MainFrame))
		MODULE_NAME=mvNoConfTool
	endif
endif
ifeq ($(strip $(FLTK_CONF_TOOL)),)
	ifeq ($(MODULE_NAME),$(filter $(MODULE_NAME),AVILinux GigEVisionTestClient LiveSnapFLTK SynchronousCaptureMultipleInputsFLTK))
		MODULE_NAME=mvNoConfTool
	endif
else
	FLTK_LDFLAGS=$(shell $(FLTK_CONF_TOOL) --ldflags)
	FLTK_CPPFLAGS=$(shell $(FLTK_CONF_TOOL) --cxxflags)
endif
ifeq (,$(SDL_CONF_TOOL))
	ifeq ($(MODULE_NAME),$(filter $(MODULE_NAME),LiveSnapSDL))
		MODULE_NAME=mvNoConfTool
	endif
endif

############################################
#===========================================
.PHONY: run strip dbg clean_mod new pack all $(MODULE_NAME) $(MODULE_NAME) extract_ar_objects ipk

all:	$(MODULE_NAME)

#-------------------------------------------
ifeq ($(HAVE_GCC),yes)
# check if the compiler is installed
#-------------------------------------------

#-------------------------------------------
ifeq ($(MODULE_NAME),mvNoConfTool)
# might be deleted since CONF_TOOL missing
#-------------------------------------------
$(MODULE_NAME):
	@echo "*** Error: 'MODULE_NAME' set to >$(MODULE_NAME)<!!"
	@echo "*** Error: Do you need a '*_CONF_TOOL'? e.g. WX, FLTK, SDL, etc.!!"
else

#-------------------------------------------
$(MODULE_NAME): $(OBJECTS) $(OOBJ) extract_ar_objects
	$(CXX) $(CPPFLAGS) -o $(BINARY_NAME) $(OBJECTS) $(OOBJ) $(LLOCLIB) $(LDFLAGS) $(USER_AR_OBJ_LIST) -ldl $(PROFOPT)

#-------------------------------------------
strip: all
	ls -lh $(BINARY_NAME)
	$(TOOLS)strip $(BINARY_NAME)
	ls -lh $(BINARY_NAME)

endif
#-------
else
#-------
ifeq ($(WRONG_HOST),yes)
$(MODULE_NAME) strip:
	@echo "*** SKIPPED: Wrong HostType ($(HOSTTYPE)) for target: $(TARGET)!!"
else
$(MODULE_NAME) strip:
	@echo "*** WARNING: No compiler: >>$(CC_TOOL_PATH)gcc<< found!!"
endif
endif

#-------------------------------------------
clean_mod:
	rm -f $(OBJECTS) $(OOBJ) $(MODULE_NAME) $(BINARY_NAME)

#-------------------------------------------

ifeq ($(OSTYPE),cygwin)
.cpp.o:
	@ echo "Compiling $*.cpp"
	-$(CXX) -c $(CPPFLAGS) $(INCLUDES) -o $@ $< 2>gccErrors.dat
	@ gccerr gccErrors.dat
endif
