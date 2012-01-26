#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2011-10-27 02:43:57 -0700 (Thu, 27 Oct 2011) $
#  $Rev: 657 $


# project layout
TOP_DIR = ../../..
include $(TOP_DIR)/Makefile.common

# local configuration
SRC_DIR   = .
HDR_DIR   = .
OBJ_DIR   = ./obj/$(OS)-$(ARCH)$(GCC_VER)/$(CFG)
PROG      = $(shell pwd | awk 'BEGIN { FS="/" } { print $$NF }') 
SOURCES   = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDES  = $(wildcard $(HDR_DIR)/*.h)
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
OUTPUT    = $(BIN_DIR)/$(PROG)

all: $(OUTPUT)

$(OBJECTS): $(INCLUDES)

$(OUTPUT): $(OBJ_DIR) $(LIB_TARGET) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -I$(HDR_DIR) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(OUTPUT)

$(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OUTPUT) $(OBJECTS) *~ TAGS core *.bak #*#
	-rm -rf $(OBJ_DIR)
