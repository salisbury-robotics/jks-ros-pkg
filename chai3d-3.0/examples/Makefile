#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2012-01-16 20:30:26 -0800 (Mon, 16 Jan 2012) $
#  $Rev: 685 $


# import settings
TOP_DIR = ..
include $(TOP_DIR)/Makefile.common

# demo groups
SUBDIRS  = CLI \
	         GLUT

# rules
all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
