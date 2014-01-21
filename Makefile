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


FLAC_FILES = $(shell find flac/ -type f -name '*.flac')
MP3_FILES = $(patsubst flac/%.flac, mp3/%.mp3, $(FLAC_FILES))

MEX_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_mex.cpp')
MEX_OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.mexo, $(MEX_SOURCES))
MEX = $(patsubst $(SRCDIR)/%_mex.cpp, $(BINDIR)/%.$(MEX_EXT), $(MEX_SOURCES))

CMD_SOURCES = $(shell find $(SRCDIR)/ -type f -name '*_cmd.cpp')
CMD_OBJECTS =  $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(CMD_SOURCES))
CMD =  $(patsubst $(SRCDIR)/%_cmd.cpp, $(BINDIR)/%, $(CMD_SOURCES))

SOURCES := $(shell find $(SRCDIR)/ -type f -name '*.cpp')
SOURCES := $(filter-out $(MEX_SOURCES) $(CMD_SOURCES),$(SOURCES))
OBJECTS := $(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(SOURCES))
LIB := $(BINDIR)/maslib.a


HEADERS := $(shell find $(INCLUDEDIR)/ -type f -name '*.h')
# $(notdir $(MEX_SOURCES:.cpp=.$(MEX_EXT))))

MKDIR_CMD=mkdir -p $(@D)

default: all

cleanup:
	rm -rf $(OBJECTS) $(MEX_OBJECTS) $(CMD_OBJECTS)

clean: cleanup
	rm -rf $(CMD) $(MEX) $(LIB)

# do not delete intermediates
.SECONDARY:

vars:
	@echo "CMD_SOURCES: $(CMD_SOURCES)"
	@echo "CMD_OBJECTS: $(CMD_OBJECTS)"
	@echo "CMD: $(CMD)"
	@echo "MEX_OBJECTS: $(MEX_OBJECTS)"
	@echo "MEX_CPPFLAGS: $(MEX_CPPFLAGS)"
	@echo "MEX: $(MEX)"

all: cmd mex lib

mex: $(MEX)

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
	
# Command-line programs
$(BINDIR)/%: $(BUILDDIR)/%_cmd.o $(OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	$(LD) -o $@ $(LDFLAGS) $(OBJECTS) $<
	@echo Complete!
	
$(LIB) : $(OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	ar rcs $@ $(OBJECTS)
	ranlib $@
