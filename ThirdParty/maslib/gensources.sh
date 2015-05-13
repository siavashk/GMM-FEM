#!/bin/bash
# Run from a bash shell to generate list of source files for Makefiles 
# and CMake

makefile=MakefileSources.mk
cmakefile=CMakeSources.txt

SRCDIR=./src
COMMONDIR=$SRCDIR/common
TESTDIR=$SRCDIR/test
MEXDIR=$SRCDIR/mex
CMDDIR=$SRCDIR/cmd

COMMON_SOURCES=`find $COMMONDIR/ -type f -name '*.cxx' | sort | sed "s|$COMMONDIR|\$\(COMMONDIR\)|g"`
MEX_SOURCES=`find $MEXDIR/ -type f -name '*_mex.cxx' | sort | sed "s|$MEXDIR|\$\(MEXDIR\)|g"`
CMD_SOURCES=`find $CMDDIR/ -type f -name '*_cmd.cxx' | sort | sed "s|$CMDDIR|\$\(CMDDIR\)|g"`
TEST_SOURCES=`find $TESTDIR/ -type f -name '*.cxx' | sort | sed "s|$TESTDIR|\$\(TESTDIR\)|g"`
M_SOURCES=`find $MEXDIR/ -type f -name '*.m' | sort | sed "s|$MEXDIR|\$\(MEXDIR\)|g"`

echo "COMMON_SOURCES := \\" > $makefile
echo "`echo "$COMMON_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $makefile
echo "" >> $makefile
echo "MEX_SOURCES := \\" >> $makefile
echo "`echo "$MEX_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $makefile
echo "" >> $makefile
echo "CMD_SOURCES := \\" >> $makefile
echo "`echo "$CMD_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $makefile
echo "" >> $makefile
echo "TEST_SOURCES := \\" >> $makefile
echo "`echo "$TEST_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $makefile
echo "" >> $makefile
echo "M_SOURCES := \\" >> $makefile
echo "`echo "$M_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $makefile

echo "set (COMMON_SOURCES " > $cmakefile
echo "`echo "$COMMON_SOURCES" | sed 's|^$(\(.*DIR\))\(.*\)|${\1}\2|g' | sed 's|^|        |g'`" >> $cmakefile
echo "    )" >> $cmakefile
echo "" >> $cmakefile
echo "set (MEX_SOURCES " >> $cmakefile
echo "`echo "$MEX_SOURCES" | sed 's|^$(\(.*DIR\))\(.*\)|${\1}\2|g' | sed 's|^|        |g'`" >> $cmakefile
echo "    )" >> $cmakefile
echo "" >> $cmakefile
echo "set (CMD_SOURCES " >> $cmakefile
echo "`echo "$CMD_SOURCES" | sed 's|^$(\(.*DIR\))\(.*\)|${\1}\2|g' | sed 's|^|        |g'`" >> $cmakefile
echo "    )" >> $cmakefile
echo "" >> $cmakefile
echo "set (TEST_SOURCES " >> $cmakefile
echo "`echo "$TEST_SOURCES" | sed 's|^$(\(.*DIR\))\(.*\)|${\1}\2|g' | sed 's|^|        |g'`" >> $cmakefile
echo "    )" >> $cmakefile
echo "" >> $cmakefile
echo "set (M_SOURCES " >> $cmakefile
echo "`echo "$M_SOURCES" | sed 's|^$(\(.*DIR\))\(.*\)|${\1}\2|g' | sed 's|^|        |g'`" >> $cmakefile
echo "    )" >> $cmakefile
