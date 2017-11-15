#! /bin/make
OSTYPE := $(shell uname | cut -d _ -f 1 | tr [:upper:] [:lower:])
HOSTTYPE := $(shell uname -m)

#-------------------------------------------
ifeq ($(OSTYPE),cygwin)
	SUBDIRS=$(OSTYPE)
else
	ifeq ($(HOSTTYPE),i686)
		NATIVE=x86
	else
		NATIVE=$(HOSTTYPE)
		ifeq ($(NATIVE),x86_64)
			EXTRA_TARGET=x86
		endif
	endif

	SUBDIRS = $(NATIVE) ppc603e arm $(EXTRA_TARGET) armv6zk
endif


#-------------------------------------------
.PHONY:	$(SUBDIRS) all build new info ipk cygwin clean armv6zk

#-------------------------------------------
all info ipk:
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir ; $(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
native:
	@mkdir -p $(NATIVE)
	$(MAKE) -C $(NATIVE) -f ../Makefile.inc all || exit $$?

#-------------------------------------------
ppc603e ppc x86 x86_64 arm cygwin armv6zk:
	@mkdir -p $@
	$(MAKE) -C $@ -f ../Makefile.inc all || exit $$?

#-------------------------------------------
clean:
	@rm -rf $(SUBDIRS)

#-------------------------------------------
build new: clean
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir; $(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
