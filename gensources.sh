#!/bin/bash
# Run from a bash shell to generate list of source files for Makefile

filename=Makefile.sources

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

echo "COMMON_SOURCES := \\" > $filename
echo "`echo "$COMMON_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $filename
echo "" >> $filename
echo "MEX_SOURCES := \\" >> $filename
echo "`echo "$MEX_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $filename
echo "" >> $filename
echo "CMD_SOURCES := \\" >> $filename
echo "`echo "$CMD_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $filename
echo "" >> $filename
echo "TEST_SOURCES := \\" >> $filename
echo "`echo "$TEST_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $filename
echo "" >> $filename
echo "M_SOURCES := \\" >> $filename
echo "`echo "$M_SOURCES" | sed '$ ! s|$| \\\|g' | sed 's|^|    |g'`" >> $filename

#echo "MEX_SOURCES := $MEX_SOURCES"
#echo ""
#echo "CMD_SOURCES := $CMD_SOURCES"
#echo ""
#echo "TEST_SOURCES := $TEST_SOURCES"
#echo ""
#echo "M_SOURCES := $M_SOURCES"

