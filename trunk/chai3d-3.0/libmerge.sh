#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2011-12-14 02:23:08 -0800 (Wed, 14 Dec 2011) $
#  $Rev: 667 $


#! /bin/sh

AR=$1
shift
TARGET=$1
shift

# merging all static libs
for lib in "$@"
do
    echo merging $lib into $TARGET
    tmpdir=.libmerge.tmp   > /dev/null 2>&1
    mkdir $tmpdir          > /dev/null 2>&1
    cd $tmpdir             > /dev/null 2>&1
    $AR -x  ../$TARGET     > /dev/null 2>&1
    $AR -x  ../$lib        > /dev/null 2>&1
    $AR -rc ../$TARGET *.o > /dev/null 2>&1
    cd ..                  > /dev/null 2>&1
    rm -rf $tmpdir         > /dev/null 2>&1
done

