#! /bin/make
# -*- sh -*-
OSTYPE := $(shell uname | cut -d _ -f 1 | tr [:upper:] [:lower:])
HOSTTYPE := $(shell uname -m)

#-------------------------------------------
ifeq ($(OSTYPE),cygwin)
	SUBDIRS=$(OSTYPE)
else
	ifeq ($(HOSTTYPE),i686)
		NATIVE=x86
	else
		ifeq ($(HOSTTYPE), aarch64)
			NATIVE=arm64
		else
			NATIVE=$(HOSTTYPE)
			ifeq ($(NATIVE),x86_64)
				EXTRA_TARGET=x86
			endif
		endif
	endif

	SUBDIRS = $(NATIVE) ppc603e $(EXTRA_TARGET) arm armv7a
endif

#-------------------------------------------
.PHONY:	$(SUBDIRS) all build new info ipk cygwin clean

#-------------------------------------------
all info ipk:
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir ; $(MAKE) $(MAKEOPTS) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
native:
	@mkdir -p $(NATIVE)
	$(MAKE) $(MAKEOPTS) -C $(NATIVE) -f ../Makefile.inc all || exit $$?

#-------------------------------------------
ppc x86 x86_64 cygwin arm armv7a:
	@mkdir -p $@
	$(MAKE) $(MAKEOPTS) -C $@ -f ../Makefile.inc all || exit $$?

#-------------------------------------------
clean:
	@rm -rf $(SUBDIRS)

#-------------------------------------------
build new: clean
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir; $(MAKE) $(MAKEOPTS) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
ppc_6xx ppc603e:
	@echo target platform $@ is unsupported
