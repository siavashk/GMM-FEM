CC=g++
LD=g++

# DEBUG = 0
# PROFILE = 0

BUILDDIR = ./build
SRCDIR = ./src
TET_DIR = ./source
INCLUDEDIR = $(TET_DIR)
BINDIR = ./bin

CPPFLAGS=-I$(INCLUDEDIR) --std=c++11 -DPTR_EXCEEDS_LONG -DTETLIBRARY
ifdef DEBUG
CPPFLAGS:=$(CPPFLAGS) -g -O0
else
CPPFLAGS:=$(CPPFLAGS) -O3
endif

LDFLAGS=
ifdef PROFILE
CPPFLAGS:=$(CPPFLAGS) -pg
LDFLAGS:=$(LDFLAGS) -pg
endif

# Library extension
ifndef LIB_EXT
LIB_EXT:=dll
endif

# Matlab folders
ifndef MATLAB_ROOT
MATLAB_ROOT:="C:/Program Files/MATLAB/R2013b"
# MATLAB_ROOT:="/Applications/MATLAB_R2013b.app"
endif
ifndef MATLAB_BINDIR
MATLAB_BINDIR:="$(MATLAB_ROOT)/bin/win64"
# MATLAB_BINDIR:="$(MATLAB_ROOT)/bin/maci64"
endif
ifndef MEX_EXT
MEX_EXT:=mexw64
# MEX_EXT:=mexmaci64
endif
ifndef MATLAB_INCLUDEDIR
MATLAB_INCLUDEDIR:="$(MATLAB_ROOT)/extern/include"
endif

MEX_CPPFLAGS:=$(CPPFLAGS) -DMX_COMPAT_32 -DMATLAB_MEX_FILE -I"$(MATLAB_INCLUDEDIR)" -Wall
MEX_LDFLAGS:= $(LDFLAGS) -shared -L"$(MATLAB_BINDIR)" -lstdc++ -lmex -lmx -lmat

MEX_SOURCES = $(SRCDIR)/tetgen_mex.cpp
MEX_OBJECTS = $(BUILDDIR)/tetgen.mexo
MEX = $(BINDIR)/tetgen.$(MEX_EXT)

MFILES := $(BINDIR)/tetgen.m
SOURCES := $(TET_DIR)/predicates.cxx $(TET_DIR)/tetgen.cxx 
OBJECTS := $(BUILDDIR)/predicates.o $(BUILDDIR)/tetgen.o

MKDIR_CMD=mkdir -p $(@D)

default: all

cleanup:
	rm -rf $(OBJECTS) $(MEX_OBJECTS)

clean: cleanup
	rm -rf $(MEX) $(MFILES)

# do not delete intermediates
.SECONDARY:

vars:
	@echo "MEX_SOURCES: $(MEX_SOURCES)"
	@echo "MEX_OBJECTS: $(MEX_OBJECTS)"
	@echo "MEX_CPPFLAGS: $(MEX_CPPFLAGS)"
	@echo "MEX_LDFLAGS: $(MEX_LDFLAGS)"
	@echo "MEX: $(MEX)"
	@echo "MFILES: $(MFILES)"
	@echo "SOURCES: $(SOURCES)"
	@echo "OBJECTS: $(OBJECTS)"
	

all: mex

mex: $(MEX) $(MFILES)

$(BUILDDIR)/%.o: $(TET_DIR)/%.cxx
	@$(MKDIR_CMD)
	@echo Compiling $@...
	@$(CC) $(CPPFLAGS) -o $@ -c $<

$(BUILDDIR)/%.mexo: $(SRCDIR)/%_mex.cpp
	@$(MKDIR_CMD)
	@echo Compiling $@...
	@$(CC) $(MEX_CPPFLAGS) -o $@ -c $<

$(BINDIR)/%.$(MEX_EXT): $(BUILDDIR)/%.mexo $(OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	@$(LD) -o $@ $(MEX_LDFLAGS) $(OBJECTS) $<
	@echo Complete!
	
$(BINDIR)/%.m: $(SRCDIR)/%.m
	@$(MKDIR_CMD)
	@echo Copying $@ ...
	@cp $< $@

