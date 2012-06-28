#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2012, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2012-05-07 16:50:14 -0700 (Mon, 07 May 2012) $
#  $Rev: 836 $


# global path
TOP_DIR = .

# commong settings
include $(TOP_DIR)/Makefile.common

# directories
OBJ_ROOT  = $(TOP_DIR)/obj
OBJ_DIR   = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)
SRC_DIR   = $(TOP_DIR)/src
VPATH     = $(dir $(wildcard $(SRC_DIR)/* $(SRC_DIR)/*/* $(SRC_DIR)/*/*/*))

# additional libraries
ifeq ($(USE_DHD),yes)
CXXFLAGS += -I$(DHD_EXT)/include
INTLIBS  += $(DHD_EXT)/lib/$(OS)-$(ARCH)$(GCC_VER)/libdhd.a
endif

# main sources
SOURCES   = $(shell find $(SRC_DIR) -name '*.cpp')

# include dependencies
INCLUDES  = $(shell find $(SRC_DIR) -name '*.h')

# objects
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))

# temporary static lib (to combine with internal dependencies)
TMP_LIB   = $(OBJ_DIR)/$(LIB_NAME).tmp.a
INTERNAL  = internal

# optional dependencies to include in static library (see Makefile.config)
ifeq ($(USE_EXTERNAL_LIBPNG), no)
PNG_DIR   = $(INTERNAL)/libpng
CXXFLAGS += -I$(PNG_DIR)/include
INTLIBS  += $(PNG_DIR)/lib/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)/libpng.a
endif
ifeq ($(USE_EXTERNAL_LIBJPEG), no)
JPG_DIR   = $(INTERNAL)/libjpeg
CXXFLAGS += -I$(JPG_DIR)/include
INTLIBS  += $(JPG_DIR)/lib/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)/libjpeg.a
endif
ifeq ($(USE_EXTERNAL_GIFLIB), no)
GIF_DIR   = $(INTERNAL)/giflib
CXXFLAGS += -I$(GIF_DIR)/include
INTLIBS  += $(GIF_DIR)/lib/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)/libgif.a
endif
ifeq ($(USE_EXTERNAL_LIB3DS), no)
3DS_DIR   = $(INTERNAL)/lib3ds
CXXFLAGS += -I$(3DS_DIR)/include
INTLIBS  += $(3DS_DIR)/lib/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)/lib3ds.a
endif

# examples subdirectories
SUBDIRS  = examples

# build rules

all: $(SUBDIRS)

$(SUBDIRS): lib | $(BIN_DIR)
	$(MAKE) -C $@

lib: $(LIB_TARGET)

$(LIB_TARGET): $(TMP_LIB) $(INTLIBS)
	$(TOP_DIR)/libmerge.sh $(AR) $(LIB_TARGET) $?

$(TMP_LIB): $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $?

$(OBJECTS): $(INCLUDES) | $(LIB_DIR) $(OBJ_DIR)

$(INTLIBS): $(INTERNAL)

$(INTERNAL):
	$(MAKE) -C $@

$(BIN_DIR) $(LIB_DIR) $(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: $(INTERNAL) $(SUBDIRS)

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	@for T in $(INTERNAL); do make -C $$T $@; done
	-rm -f $(OBJECTS) $(LIB_TARGET) $(TMP_LIB) *~
	-rm -rf $(LIB_DIR) $(OBJ_DIR)
