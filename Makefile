CC=g++
LD=g++

DEBUG = 0
PROFILE = 0

BUILDDIR = ./build
SRCDIR = ./src
INCLUDEDIR = ./src
BINDIR = ./bin

CPPFLAGS=-I$(INCLUDEDIR) -fPIC --std=c++11
ifdef DEBUG
CPPFLAGS:=$(CPPFLAGS) -g -O0 -DMAS_DEBUG
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

MEX_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_mex.cpp')
MEX_OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.mexo, $(MEX_SOURCES))
MEX = $(patsubst $(SRCDIR)/%_mex.cpp, $(BINDIR)/%.$(MEX_EXT), $(MEX_SOURCES))

CMD_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_cmd.cpp')
CMD_OBJECTS =  $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(CMD_SOURCES))
CMD =  $(patsubst $(SRCDIR)/%_cmd.cpp, $(BINDIR)/%, $(CMD_SOURCES))

SOURCES := $(shell find $(SRCDIR)/ -type f -name '*.cpp')
SOURCES := $(filter-out $(MEX_SOURCES) $(CMD_SOURCES),$(SOURCES))
OBJECTS := $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(SOURCES))
LIB := $(BINDIR)/maslib.$(LIB_EXT)

M_FILES := $(shell find $(SRCDIR)/ -type f -name '*.m')
M_FILES_OUT := $(patsubst $(SRCDIR)/%.m, $(BINDIR)/%.m, $(M_FILES))

HEADERS := $(shell find $(INCLUDEDIR)/ -type f -name '*.h')
# $(notdir $(MEX_SOURCES:.cpp=.$(MEX_EXT))))

MKDIR_CMD=mkdir -p $(@D)

default: all

cleanup:
	rm -rf $(OBJECTS) $(MEX_OBJECTS) $(CMD_OBJECTS)

clean: cleanup
	# $(M_FILES_OUT)
	rm -rf $(CMD) $(MEX) $(LIB) 

# do not delete intermediates
.SECONDARY:

vars:
	@echo "CMD_SOURCES: $(CMD_SOURCES)"
	@echo "CMD_OBJECTS: $(CMD_OBJECTS)"
	@echo "CMD: $(CMD)"
	@echo "MEX_OBJECTS: $(MEX_OBJECTS)"
	@echo "MEX_CPPFLAGS: $(MEX_CPPFLAGS)"
	@echo "MEX_LDFLAGS: $(MEX_LDFLAGS)"
	@echo "MEX: $(MEX)"
	

all: cmd lib mex

doc:
	@doxygen doc/maslib.doxyfile

mex: $(MEX) $(M_FILES_OUT)

cmd: $(CMD)

lib: $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@$(MKDIR_CMD)
	@echo Compiling $@...
	$(CC) $(CPPFLAGS) -o $@ -c $<

$(BUILDDIR)/%.mexo: $(SRCDIR)/%.cpp
	@$(MKDIR_CMD)
	@echo Compiling $@...
	@$(CC) $(MEX_CPPFLAGS) -o $@ -c $<

$(BINDIR)/%.$(MEX_EXT): $(BUILDDIR)/%_mex.mexo $(OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	$(LD) -o $@ $(MEX_LDFLAGS) $(OBJECTS) $<
	@echo Complete!
	
$(BINDIR)/%.m: $(SRCDIR)/%.m
	@$(MKDIR_CMD)
	@echo Copying $@ ...
	@cp $< $@
	
# Command-line programs
$(BINDIR)/%: $(BUILDDIR)/%_cmd.o $(OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	$(LD) -o $@ $(LDFLAGS) $(OBJECTS) $<
	@echo Complete!
	
$(LIB) : $(OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	# ar rcs $@ $(OBJECTS)
	# ranlib $@
	$(LD) -o $@ $(LDFLAGS) -fPIC -shared $(OBJECTS)
