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

	SUBDIRS = $(NATIVE) ppc603e ppc_6xx arm armv7a $(EXTRA_TARGET) armv6zk armv7ahf armhf armsf arm64
endif

#-------------------------------------------
.PHONY: $(SUBDIRS) all build new info ipk cygwin clean armv7a armv6zk armv7ahf armhf armsf arm64

#-------------------------------------------
all info ipk:
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir ; $(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
native:
	@mkdir -p $(NATIVE)
	$(MAKE) -C $(NATIVE) -f ../Makefile.inc all || exit $$?

#-------------------------------------------
ppc603e ppc ppc_6xx x86 x86_64 arm armv7a cygwin armv6zk armv7ahf armhf armsf arm64:
	@mkdir -p $@
	$(MAKE) -C $@ -f ../Makefile.inc all || exit $$?

#-------------------------------------------
clean:
	@rm -rf $(SUBDIRS)

#-------------------------------------------
build new: clean
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir; $(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
strip:
	@for dir in $(SUBDIRS);										\
	do															\
		if test -d $$dir;										\
		then													\
			$(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?;	\
		fi;														\
	done

#-------------------------------------------
