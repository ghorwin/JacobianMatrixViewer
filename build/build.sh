#!/bin/bash

# Build script for building application and all dependend libraries

# Command line options:
#   [reldeb|release|debug]		build type
#   [2 [1..n]]					cpu count
#   [verbose]					enable cmake to call verbose makefiles

# path export for mac, adjust as needed
export PATH=~/Qt/5.11.3/gcc_64/bin:~/Qt/5.11.3/clang_64/bin:$PATH

# for MacOS, brew install of Qt 5 ("brew install qt5")
export CMAKE_PREFIX_PATH=/usr/local/opt/qt5/

CMAKELISTSDIR=$(pwd)/..
BUILDDIR="bb"

# set defaults
CMAKE_BUILD_TYPE=" -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo"
MAKE_CPUCOUNT="8"
BUILD_DIR_SUFFIX="gcc"
BUILD_TYPE_SUFFIX="debug"
COMPILER=""

# parse parameters, except gprof and threadchecker
for var in "$@"
do

	if [[ $var = *[[:digit:]]* ]];
	then
		MAKE_CPUCOUNT=$var
		echo "Using $MAKE_CPUCOUNT CPUs for compilation"
    fi

	if [[ $var = "debug"  ]];
	then
		CMAKE_BUILD_TYPE=" -DCMAKE_BUILD_TYPE:STRING=Debug"
		BUILD_TYPE_SUFFIX="debug"
		echo "Debug build..."
	fi

	if [[ $var = "release"  ]];
	then
		CMAKE_BUILD_TYPE=" -DCMAKE_BUILD_TYPE:STRING=Release"
		BUILD_TYPE_SUFFIX="release"
		echo "Release build..."
	fi

	if [[ $var = "reldeb"  ]];
	then
		CMAKE_BUILD_TYPE=" -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo"
		BUILD_TYPE_SUFFIX="reldeb"
		echo "RelWithDebInfo build..."
	fi

	if [[ $var = "verbose"  ]];
	then
		CMAKE_OPTIONS="-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON"
	  fi

done

# create build dir if not exists
BUILDDIR=$BUILDDIR-$BUILD_DIR_SUFFIX-$BUILD_TYPE_SUFFIX
if [ ! -d $BUILDDIR ]; then
    mkdir -p $BUILDDIR
fi

cd $BUILDDIR && cmake $CMAKE_OPTIONS $CMAKE_BUILD_TYPE $CMAKE_COMPILER_OPTIONS $CMAKELISTSDIR && make -j$MAKE_CPUCOUNT &&
cd $CMAKELISTSDIR &&
# now we are top level
mkdir -p bin/release &&
echo "*** Copying executable to bin/release ***" &&

# mac os app
if [ -d build/$BUILDDIR/JacobianMatrixViewer/JacobianMatrixViewer.app ]
then
	# MacOS
	rm -rf bin/release/JacobianMatrixViewer.app
	cp -r build/$BUILDDIR/JacobianMatrixViewer/JacobianMatrixViewer.app bin/release/JacobianMatrixViewer.app
else
	cp build/$BUILDDIR/JacobianMatrixViewer/JacobianMatrixViewer bin/release/JacobianMatrixViewer
fi

