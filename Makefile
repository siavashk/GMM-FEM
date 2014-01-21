CC=g++
LD=g++

DEBUG = 1

BUILDDIR = ./build
SRCDIR = ./src
INCLUDEDIR = ./src
BINDIR = ./bin

CPPFLAGS=-I$(INCLUDEDIR) --std=c++11
ifdef DEBUG
CPPFLAGS:=$(CPPFLAGS) -g -O0 -DMAS_DEBUG 
else
CPPFLAGS:=$(CPPFLAGS) -O3
endif

# Library extension
ifndef LIB_EXT
LIB_EXT:=dll
endif

# Matlab folders
ifndef MATLAB_ROOT
MATLAB_ROOT:="C:/Program Files/MATLAB/R2012b"
endif
ifndef MATLAB_BINDIR
MATLAB_BINDIR:="$(MATLAB_ROOT)/bin/win64"
endif
ifndef MEX_EXT
MEX_EXT:=mexw64
endif
ifndef MATLAB_INCLUDEDIR
MATLAB_INCLUDEDIR:="$(MATLAB_ROOT)/extern/include"
endif
MEX_CPPFLAGS:=$(CPPFLAGS) -DMX_COMPAT_32 -DMATLAB_MEX_FILE -I"$(MATLAB_INCLUDEDIR)" -Wall
MEX_LDFLAGS:= -shared -L"$(MATLAB_BINDIR)" -lstdc++ -lmex -lmx -lmat

MEXLIB_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_mex_shared.cpp')
MEXLIB_OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.mexlibo, $(MEXLIB_SOURCES))
MEXLIB = $(patsubst $(SRCDIR)/%.cpp, $(BINDIR)/%.$(LIB_EXT), $(MEXLIB_SOURCES))
MEXLIB_LDFLAGS = $(patsubst %.$(LIB_EXT), -l%, $(notdir $(MEXLIB)))
MEXLIB_LDFLAGS := $(addprefix -L, $(dir $(MEXLIB))) $(MEXLIB_LDFLAGS)
MEX_LDFLAGS := $(MEXLIB_LDFLAGS) $(MEX_LDFLAGS) 

MEX_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_mex.cpp')
MEX_OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.mexo, $(MEX_SOURCES))
MEX = $(patsubst $(SRCDIR)/%_mex.cpp, $(BINDIR)/%.$(MEX_EXT), $(MEX_SOURCES))

CMD_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_cmd.cpp')
CMD_OBJECTS =  $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(CMD_SOURCES))
CMD =  $(patsubst $(SRCDIR)/%_cmd.cpp, $(BINDIR)/%, $(CMD_SOURCES))

SOURCES := $(shell find $(SRCDIR)/ -type f -name '*.cpp')
SOURCES := $(filter-out $(MEXLIB_SOURCES) $(MEX_SOURCES) $(CMD_SOURCES),$(SOURCES))
OBJECTS := $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(SOURCES))
LIB := $(BINDIR)/maslib.$(LIB_EXT)


HEADERS := $(shell find $(INCLUDEDIR)/ -type f -name '*.h')
# $(notdir $(MEX_SOURCES:.cpp=.$(MEX_EXT))))

MKDIR_CMD=mkdir -p $(@D)

default: all

cleanup:
	rm -rf $(OBJECTS) $(MEX_OBJECTS) $(CMD_OBJECTS) $(MEXLIB_OBJECTS)

clean: cleanup
	rm -rf $(CMD) $(MEX) $(LIB) $(MEX_LIB)

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
	@echo "MEXLIB_OBJECTS: $(MEXLIB_OBJECTS)"
	@echo "MEXLIB: $(MEXLIB)"
	@echo "MEXLIB_LDFLAGS: $(MEXLIB_LDFLAGS)"
	

all: cmd mex lib

mex: $(MEXLIB) $(MEX)

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
	
$(BUILDDIR)/%.mexlibo: $(SRCDIR)/%.cpp
	@$(MKDIR_CMD)
	@echo Compiling $@...
	@$(CC) $(CPPFLAGS) -o $@ -c $<
	
$(BINDIR)/%mex_shared.$(LIB_EXT): $(BUILDDIR)/%mex_shared.mexlibo
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	$(LD) -o $@ $(LDFLAGS) -fPIC -shared $<
	
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
