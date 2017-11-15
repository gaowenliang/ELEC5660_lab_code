#! /bin/make
############################################
# Makefile.inc for generating libraries.
############################################
TARGET?=$(shell basename `pwd`)
OSTYPE := $(shell uname | cut -d _ -f 1 | tr [:upper:] [:lower:])
HOSTTYPE := $(shell uname -m)

#-------------------------------------------
USER_AR_OBJ_LIST=

#-------------------------------------------
include $(SYSTEM_PATH)/Makefile.all.inc

#-------------------------------------------
ifeq ($(TARGET),ppc603e)
	ifeq ($(OSTYPE),cygwin)
		LIB_OUT=$(OSTYPE)
	else
		LIB_OUT=$(TARGET)
	endif
else
	ifeq ($(TARGET),arm)
		LIB_OUT=$(TARGET)
	else
		ifeq ($(TARGET),x86_64)
			LIB_OUT=$(TARGET)
		else
			LIB_OUT=$(TARGET)
		endif
	endif
endif

LIBPATH=$(WORK)/lib/$(LIB_OUT)

#-------------------------------------------
STATIC_LIB=$(LIB_NAME).a

DYNAMIC_LIB=$(LIB_NAME).so
VERSION=$(MAJOR).$(MINOR).$(BUILD)

#-------------------------------------------
INCLFILES=						\
	$(GLOBINCFILES)				\
	-I$(WORK)					\
	-I..

#-------------------------------------------
LOCDEFINES=							\

#---------------------------------------------
# building 32 bit on 64 bit system
ifeq ($(HOSTTYPE),x86_64)
	ifeq ($(TARGET),x86)
		COMP_OPTIONS_32BIT=-m32
	endif
endif

#-------------------------------------------
CPPFLAGS= 							\
	$(CPPFLAGS_2)					\
	$(COMP_OPTIONS_32BIT)			\
	$(COMP_OPTIONS_64BIT)			\
    $(LOCDEFINES)					\
    $(INCLFILES)					\

#	-D_DEBUG

#-------------------------------------------
LDOPT = 							\
	$(LDOPT_1)						\
	$(COMP_OPTIONS_32BIT)			\
	$(COMP_OPTIONS_64BIT)			\

#		--verbose

#===========================================
.PHONY: all strip clean_stat clean_dyn $(STATIC_LIB) $(DYNAMIC_LIB).$(MAJOR) ltarget extract_ar_objects

#-------------------------------------------
all: $(STATIC_LIB) $(DYNAMIC_LIB).$(MAJOR)

#-------------------------------------------
ifeq ($(HAVE_GCC),yes)
# check if the compiler is installed
#-------------------------------------------

ifeq ($(DO_STATIC_LIBS),y)
$(STATIC_LIB):	ltarget extract_ar_objects $(TARGET) $(OBJECTS) $(LIBPATH)
	$(AR) rcs $(LIBPATH)/$(STATIC_LIB) $(EXTERN_OBJ) $(USER_AR_OBJ_LIST) $(OBJECTS)

#-------
#else
#-------
#$(STATIC_LIB):
#	@echo "No static libs made! Set 'DO_STATIC_LIBS' to 'y' if this is incorrect!"

endif

##	$(CTYPE) -shared -Wl,--version-script,$(LIBPATH)/driver.vscript,-soname,$(DYNAMIC_LIB)			\
#-------------------------------------------
ifeq ($(DO_DYNAMIC_LIBS),y)
$(DYNAMIC_LIB).$(MAJOR):	ltarget $(TARGET) $(OBJECTS) $(LIBPATH)
	$(CTYPE) -shared -Wl,-soname,$(DYNAMIC_LIB)$(RPATH_OPTIONS)						\
		-o $(LIBPATH)/$(DYNAMIC_LIB).$(VERSION) $(EXTERN_OBJ) $(OBJECTS) $(USE_LN_LIBS) -lc $(LDOPT)
	@( cd $(LIBPATH);											\
		rm -f $(DYNAMIC_LIB) $(DYNAMIC_LIB).$(MAJOR) ;							\
		ln -s $(DYNAMIC_LIB).$(VERSION) $(DYNAMIC_LIB).$(MAJOR) ;					\
		ln -s $(DYNAMIC_LIB).$(MAJOR) $(DYNAMIC_LIB) ;							\
	)

strip: all
	ls -lh $(LIBPATH)/$(DYNAMIC_LIB).$(VERSION)
	$(TOOLS)strip $(LIBPATH)/$(DYNAMIC_LIB).$(VERSION)
	ls -lh $(LIBPATH)/$(DYNAMIC_LIB).$(VERSION)
#-------
#else
#-------
#$(DYNAMIC_LIB).$(MAJOR):
#	@echo "No dynamic libs made! Set 'DO_DYNAMIC_LIBS' to 'y' if this is incorrect!"

strip: all

endif

#-------
else
# if the compiler is not installed print the message!
#-------
ifeq ($(WRONG_HOST),yes)
$(STATIC_LIB) $(DYNAMIC_LIB).$(MAJOR) strip:
	@echo "*** SKIPPED: Wrong HostType ($(HOSTTYPE)) for target: $(TARGET)!!"
else
$(STATIC_LIB) $(DYNAMIC_LIB).$(MAJOR) strip:
	@echo "*** WARNING: No compiler: >>$(CC_TOOL_PATH)gcc<< found!!"
endif
#-------------------------------------------
endif
#-------------------------------------------

#-------------------------------------------
clean_mod: clean_stat clean_dyn

#-------------------------------------------
clean_stat:
	@rm -f $(OBJECTS) $(LIBPATH)/$(STATIC_LIB)

#-------------------------------------------
clean_dyn:
	@rm -f $(OBJECTS) $(LIBPATH)/$(DYNAMIC_LIB)*

#-------------------------------------------
ltarget:
	@mkdir -p $(LIBPATH)

#-------------------------------------------
linfo:
	@echo "WORK          =" $(WORK)
	@echo "TOOLS         =" $(TOOLS)
	@echo "LIBPATH       =" $(LIBPATH)
	@echo "LIB_NAME      =" $(LIB_NAME)
	@echo "STATIC_LIB    =" $(STATIC_LIB)
	@echo "DYNAMIC_LIB   =" $(DYNAMIC_LIB)
	@echo "CTYPE         =" $(CTYPE)
	@echo "INCLFILES     =" $(INCLFILES)
	@echo "VERSION       =" $(VERSION)
	@echo "WX_CONF_PATH  =" $(WX_CONF_PATH)
	@echo "WX_CONF_TOOL  =" $(WX_CONF_TOOL)
	@echo "WXLIBFLAGS    =" $(WXLIBFLAGS)
	@echo "WXCPPFLAGS    =" $(WXCPPFLAGS)

#-------------------------------------------
ifeq ($(USER_AR_LIB_LIST),)
extract_ar_objects:

else
extract_ar_objects: extract_ar
#USER_AR_OBJ_LIST=$(shell xargs ls tmp/*)
USER_AR_OBJ_LIST=$(shell find tmp/ -name "*.o" | xargs ls )

extract_ar:
	@set -x
	@mkdir -p tmp
	@cd tmp
	@for i in $(USER_AR_LIB_LIST);				                \
		do 								\
			cd tmp;							\
			if [ -f ../$$i ];					\
			then							\
                                $(TOOLS)$(AR) x ../$$i;                         \
			else							\
				if [ -f $$i ];					\
				then						\
					$(TOOLS)$(AR) x $$i;		        \
				else						\
					echo "**** no archive: '$$i' found!!";	\
				fi;						\
			fi;							\
			rm -f *_*_.SYMDEF;					\
			cd ..;							\
		done;

endif

#-------------------------------------------
#-------------------------------------------
# make ipkg package
#-------------------------------------------
IPK_VERSION=$(MAJOR).$(MINOR).$(BUILD).$(GPV_BUILD2)
IPKG_PACKAGE=$(IPKGPACK_NAME)_$(IPK_VERSION)_$(TARGET).ipk

ifeq ($(TARGET),ppc_6xx)
	IPKG_BUILD=ipkg-build-c
	IPKG_FOLDER_SUFFIX=-$(TARGET)
	ROOT_IPKG_PATH=$(IPKG_PATH)/flash
	IPKG_LIB_PATH=$(ROOT_IPKG_PATH)/MatrixVision/lib
else
	IPKG_BUILD=ipkg-build
	ROOT_IPKG_PATH=$(IPKG_PATH)/usr
	IPKG_LIB_PATH=$(ROOT_IPKG_PATH)/lib
endif
IPKG_FOLDER=ipkg$(IPKG_FOLDER_SUFFIX)
IPKG_PATH=$(ROOT_PATH)/$(IPKG_FOLDER)

#-------------------------------------------
ipk: $(IPKG_PACKAGE)

#-------------------------------------------
ipkglink: stripipkglibs
	@(																		\
		if [ -d $(IPKG_PATH) ]; 											\
		then																\
			rm -rf $(ROOT_IPKG_PATH);										\
			mkdir -p $(IPKG_LIB_PATH);										\
			echo "Copy files....";											\
			cd $(LIBPATH);													\
			for libfiles in $(LIB_LIST);									\
			do																\
				LIBFILES=`ls $$libfiles*`;									\
				for libfile in $$LIBFILES;									\
				do															\
						cd - > /dev/null 2>&1;								\
						cp -av $(LIBPATH)/$$libfile $(IPKG_LIB_PATH)/. ;	\
						cd $(LIBPATH);										\
				done;														\
			done;															\
			cd -> /dev/null 2>&1;											\
		fi																	\
	)

#-------------------------------------------
stripipkglibs:
	@(											\
		cd $(LIBPATH);							\
		echo "Strip files in `pwd` ....";		\
		for libfiles in $(LIB_LIST);			\
		do										\
			LIBFILES=`ls $$libfiles*`;			\
			for libfile in $$LIBFILES;			\
			do									\
				if [ ! -L $$libfile ];			\
				then							\
					ls -lh $$libfile;			\
					$(TOOLS)strip $$libfile;	\
					ls -lh $$libfile;			\
				fi;								\
			done;								\
		done;									\
		cd - > /dev/null 2>&1;					\
	)

#-------------------------------------------
$(IPKG_PACKAGE): ipkglink
	@(   \
		if [ -d $(IPKG_PATH) ]; 									\
		then														\
			cd $(ROOT_PATH);										\
			rm -if $(IPKG_PACKAGE);									\
			sed -e "s/!!version!!/$(IPK_VERSION)/" -e "s/!!architecture!!/$(TARGET)/" < ipkg.control.$(TARGET).scriptable > $(IPKG_FOLDER)/CONTROL/control; \
			echo "Changing ownership of files in $(IPKG_FOLDER) to 'root'"; \
			if [ -f ~/root_passwd ];								\
			then													\
				cat $$HOME/root_passwd | sudo -S -p "%p login: " chown -R root:root $(IPKG_FOLDER); \
			else 													\
				sudo -p "%p login: " chown -R root:root $(IPKG_FOLDER); 			\
			fi;														\
			echo "Executing: $(IPKG_BUILD) $(IPKG_FOLDER)";			\
			$(IPKG_BUILD) $(IPKG_FOLDER); 							\
			echo "Changing ownership of files in $(IPKG_FOLDER) back to '$$(whoami)'"; \
			if [ -f ~/root_passwd ];								\
			then													\
				cat $$HOME/root_passwd | sudo -S -p "%p login2: " chown -R $$(whoami):$$(whoami) $(IPKG_FOLDER); \
			else 													\
				sudo -p "%p login: " chown -R $$(whoami):$$(whoami) $(IPKG_FOLDER);	\
			fi;														\
			cd -> /dev/null 2>&1;									\
		fi  														\
	)

#===========================================
