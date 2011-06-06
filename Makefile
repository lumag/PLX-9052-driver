# This should be defined if the modules from pcmcia-cs are used.
# Leave it blank if you have PCMCIA support in the kernel.
PCMCIA_PATH =

KERNEL_VERSION = $(shell uname -r)
KERNEL_PATH = /lib/modules/$(KERNEL_VERSION)/build

MODULE_DIR_TOP = /lib/modules/$(KERNEL_VERSION)
MODULE_DIR_PCMCIA = $(MODULE_DIR_TOP)/pcmcia

# if Rules.make exists in the kernel tree, we assume 2.4 style modules
# if it doesn't assume 2.6 style
OLDMAKE = $(wildcard $(KERNEL_PATH)/Rules.make)
ifeq (,$(OLDMAKE))
ifneq (,$(PCMCIA_PATH))
$(error Can't cope with 2.6 and pcmcia-cs together)
endif
endif

MODULES = plx9052.o

SRCS = plx9052.c plx9052-24.c plx9052-26.c
TAR = tar
DEPMOD = /sbin/depmod
OUR_DIR = $(shell pwd)

ifndef VERSION
VERSION = $(shell sed -n -e 's/^ *Version \([^ ]*\)\..*/\1/p' $(OUR_DIR)/plx9052-26.c)
export VERSION
endif

DISTFILES = $(SRCS) Makefile readme.plx9052 NEWS
DISTNAME = plx9052-$(VERSION)
TARBALL = $(DISTNAME).tar.gz

MODVERDIR=$(OUR_DIR)/.tmp_versions

all: modules

dist: $(DIST_FILES)
	rm -rf $(DISTNAME)
	mkdir -p $(DISTNAME)
	for f in $(DISTFILES); do \
	  cp -f $$f $(DISTNAME)/$$f || exit 1; \
	done
	for f in `find patches -type f -name '*.diff'`; do \
	  dir=`dirname $$f`; \
	  mkdir -p $(DISTNAME)/$$dir; \
	  cp -f $$f $(DISTNAME)/$$dir; \
	done
	tar czf $(DISTNAME).tar.gz $(DISTNAME)

clean:
	rm -f core *.o *~ a.out *.d
	rm -f *.s *.i
	rm -f *.ko *.mod.c *.mod .*.cmd
	rm -rf $(MODVERDIR)

ifeq (,$(OLDMAKE))
# 2.6 style modules, get the kernel makefiles to do the work

obj-m := $(MODULES)

modules:
	mkdir -p .tmp_versions
	-cp $(KERNEL_PATH)/.tmp_versions/*.mod $(MODVERDIR)
	$(MAKE) -C $(KERNEL_PATH) SUBDIRS=$(OUR_DIR) MODVERDIR=$(MODVERDIR) modules

install: all
	mkdir -p $(MODULE_DIR_PCMCIA)
	install -m 644 -o 0 -g 0 $(MODULES:%.o=%.ko) $(MODULE_DIR_PCMCIA)
	$(DEPMOD) -ae

else
# 2.4 style modules
KERNEL_HEADERS = -I$(KERNEL_PATH)/include
ifdef PCMCIA_PATH
PCMCIA_HEADERS = -I$(PCMCIA_PATH)/include
endif

CPPFLAGS = -D__KERNEL__ -DPCMCIA_DEBUG=1 \
	-DMODULE -DEXPORT_SYMTAB \
	$(PCMCIA_HEADERS) $(KERNEL_HEADERS)
CFLAGS = -O2 -Wall -Wstrict-prototypes -fno-strict-aliasing -fno-common \
	-pipe $(EXTRACFLAGS)

MODVER = $(shell if cat $(KERNEL_PATH)/include/linux/autoconf.h 2>/dev/null | \
grep -q '^[[:space:]]*\#define[[:space:]]*CONFIG_MODVERSIONS[[:space:]]*1'; \
then echo 1; else echo 0; fi)

ifeq ($(MODVER),1)
MFLAG = -DMODVERSIONS -include $(KERNEL_PATH)/include/linux/modversions.h
endif

modules: $(MODULES)

install: all
	mkdir -p $(MODULE_DIR_PCMCIA)
	install -m 644 -o 0 -g 0 $(MODULES) $(MODULE_DIR_PCMCIA)
	$(DEPMOD) -ae

%.o: %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) $(MFLAG) -c $<

%.s: %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) -S $<

%.i: %.c
	$(CC) -MD $(CPPFLAGS) -E $< -o $@

-include $(SRCS:%.c=%.d)

endif
