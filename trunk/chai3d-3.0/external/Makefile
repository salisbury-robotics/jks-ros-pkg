#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2012-01-19 04:12:20 -0800 (Thu, 19 Jan 2012) $
#  $Rev: 703 $


# import settings
TOP_DIR = ..
include $(TOP_DIR)/Makefile.common

SUBDIRS = lib3ds

# add external dependencies if required
ifeq ($(USE_EXTERNAL_LIBPNG), no)
SUBDIRS += libpng
endif
ifeq ($(USE_EXTERNAL_LIBJPEG), no)
SUBDIRS += libjpeg
endif
ifeq ($(USE_EXTERNAL_GIFLIB), no)
SUBDIRS += giflib
endif

# rules
all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
