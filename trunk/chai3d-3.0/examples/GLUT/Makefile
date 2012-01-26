#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2012-01-19 04:12:20 -0800 (Thu, 19 Jan 2012) $
#  $Rev: 703 $


# import settings
TOP_DIR = ../..
include $(TOP_DIR)/Makefile.common

# GLUT demos
SUBDIRS  = 01-devices \
	       10-oring \
	       11-effects \
	       12-polygons \
	       13-primitives \
	       20-map \
	       21-object \
	       22-chrome \
	       23-tooth \
	       25-cubic

# rules
all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	-rm -f core *~ *.bak #*
