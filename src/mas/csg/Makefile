CC=g++
LD=g++

DEBUG = 1

BUILDDIR = ./build
SRCDIR = ./src
INCLUDEDIR = ./include
OUTDIR = .

CPPFLAGS=-I$(INCLUDEDIR) --std=c++11
ifdef DEBUG
CPPFLAGS:=$(CPPFLAGS) -g -O0 -DCSG_DEBUG 
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

CMD_EXE = $(OUTDIR)/csg.exe
TEST_EXE = $(OUTDIR)/csg_test.exe

MEX_SOURCES = $(wildcard $(SRCDIR)/*_mex.cpp)
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
SOURCES := $(filter-out $(MEX_SOURCES),$(SOURCES))
HEADERS := $(wildcard $(INCLUDEDIR)/*.h)
OBJECTS := $(addprefix $(BUILDDIR)/,$(notdir $(SOURCES:.cpp=.o)))

CMD_OBJECTS := $(filter-out $(BUILDDIR)/csg_test.o,$(OBJECTS))
TEST_OBJECTS := $(filter-out $(BUILDDIR)/csg_cmd.o,$(OBJECTS))
MEX_CSG_OBJECTS := $(filter-out $(BUILDDIR)/csg_cmd.o $(BUILDDIR)/csg_test.o $(BUILDDIR)/csg_bsp.o, $(OBJECTS))
MEX_OBJECTS = $(addprefix $(BUILDDIR)/,$(notdir $(MEX_SOURCES:.cpp=.mo)))
MEX_MEX = $(addprefix $(OUTDIR)/,$(notdir $(MEX_SOURCES:.cpp=.$(MEX_EXT))))

MKDIR_CMD=mkdir -p $(@D)

default: all

cleanup:
	rm -rf $(OBJECTS) $(MEX_OBJECTS)

clean: cleanup
	rm -rf $(CMD_EXE) $(TEST_EXE) $(MEX_MEX)

# do not delete intermediates
.SECONDARY:

vars:
	@echo "CMD: $(CMD_OBJECTS)"
	@echo "TEST: $(TEST_OBJECTS)"
	@echo "MEX_CPPFLAGS: $(MEX_CPPFLAGS)"
	@echo "MEX: $(MEX_OBJECTS)"

all: $(CMD_EXE) $(TEST_EXE) $(MEX_MEX)

mex: $(MEX_MEX)

test: $(TEST_EXE)

cmd: $(CMD_EXE)

$(TEST_EXE): $(TEST_OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	@$(LD) $(LDFLAGS) -o $@ $(TEST_OBJECTS)
	@echo Complete!

$(CMD_EXE): $(CMD_OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	@$(LD) $(LDFLAGS) -o $@ $(CMD_OBJECTS)
	@echo Complete!

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@$(MKDIR_CMD)
	@echo Compiling $@...
	@$(CC) $(CPPFLAGS) -o $@ -c $<

$(BUILDDIR)/%.mo: $(SRCDIR)/%.cpp
	@$(MKDIR_CMD)
	@echo Compiling $@...
	@$(CC) $(MEX_CPPFLAGS) -o $@ -c $<

$(OUTDIR)/%.$(MEX_EXT): $(BUILDDIR)/%.mo $(MEX_CSG_OBJECTS)
	@$(MKDIR_CMD)
	@echo Assembling $@ ...
	@$(LD) -o $@ $(MEX_LDFLAGS) $(MEX_CSG_OBJECTS) $<
	@echo Complete!

