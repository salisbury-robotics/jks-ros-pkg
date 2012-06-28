#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2012, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2012-05-07 16:50:14 -0700 (Mon, 07 May 2012) $
#  $Rev: 836 $


#! /bin/sh

echo
TOP=../..

# determine target
echo -n "determining target..."
if [ "$1" != "" ]
then
  ARCH=$1
else
  ARCH=`uname -m`
fi
OS=`uname -s`
case "$OS" in
"Linux")
  OS="linux"
  ARCHIVE_EXT="tar.gz"
  SED_INPLACE="-i"
  ;;
"Darwin") 
  OS="mac"
  ARCHIVE_EXT="dmg"
  SED_INPLACE="-i ''"
  ;;  
*)
  echo "*** error: unsupported operating system ($OS)"
  exit -1;;
esac
echo " $OS-$ARCH"

# retrieve version info
echo -n "retrieving version info..."
VERFILE=$TOP/src/version
SVNREV=`svnversion -c | awk 'BEGIN {FS=":"} {print $NF}'`  > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "*** error: svnversion returned an error ($?)"
  exit -1
fi 
MAJOR=`cat $VERFILE | grep MAJOR | awk 'BEGIN {FS="="} {print $2}'`
MINOR=`cat $VERFILE | grep MINOR | awk 'BEGIN {FS="="} {print $2}'`
RELEASE=`cat $VERFILE | grep RELEASE | awk 'BEGIN {FS="="} {print $2}'`
echo " $MAJOR.$MINOR.$RELEASE ($SVNREV)"

# assemble package name
RELEASEDIR=`pwd`/../../releases
PACKAGE=chai3d-$MAJOR.$MINOR.$RELEASE
ARCHIVE=$PACKAGE-$OS-$ARCH.$ARCHIVE_EXT

# start with a clean slate
echo -n "cleaning existing binaries..."
cd ../.. > /dev/null 2>&1
make ARCH=$ARCH CFG=debug clean > /dev/null 2>&1
make ARCH=$ARCH clean > /dev/null 2>&1
make ARCH=$ARCH -C modules clean > /dev/null 2>&1
rm -rf lib > /dev/null 2>&1
cd - > /dev/null 2>&1
echo " ok"

# produce debug version
echo -n "building $OS-$ARCH debug binaries..."
cd ../.. > /dev/null 2>&1
make ARCH=$ARCH CFG=debug > /dev/null 2>&1
if [ $? -ne 0 ]
then
  cd - > /dev/null 2>&1
  echo
  echo "** error: compilation failed"
  exit -1
fi
make ARCH=$ARCH CFG=debug -C modules > /dev/null 2>&1
if [ $? -ne 0 ]
then
  cd - > /dev/null 2>&1
  echo
  echo "** error: compilation failed"
  exit -1
fi
cd - > /dev/null 2>&1 
echo " ok"

# produce release version
echo -n "building $OS-$ARCH release binaries..."
cd ../.. > /dev/null 2>&1
make ARCH=$ARCH > /dev/null 2>&1
if [ $? -ne 0 ]
then
  cd - > /dev/null 2>&1
  echo
  echo "** error: compilation failed"
  exit -1
fi
make ARCH=$ARCH -C modules > /dev/null 2>&1
if [ $? -ne 0 ]
then
  cd - > /dev/null 2>&1
  echo
  echo "** error: compilation failed"
  exit -1
fi
cd - > /dev/null 2>&1
echo " ok"

# produce documentation
echo -n "building documentation..."
cd ../../doc/doxygen > /dev/null 2>&1
doxygen Doxyfile > /dev/null 2>&1
if [ $? -ne 0 ]
then
  cd - > /dev/null 2>&1
  echo  
  echo "** error: documentation failed"
  popd  
  exit -1
fi 
cd - > /dev/null 2>&1
echo " ok"

# copy release files to temporary folder
echo -n "copying distribution to temporary folder..."
mkdir -p /tmp/chai/$PACKAGE > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo
  echo "** error: copy to temporary folder failed"
  exit -1
fi
cd ../.. > /dev/null 2>&1
FOLDERS=`cat ./scripts/gcc/folders.txt` > /dev/null 2>&1
for folder in $FOLDERS
do
  mkdir -p /tmp/chai/$PACKAGE/$folder > /dev/null 2>&1
  if [ $? -ne 0 ]
  then
    cd - > /dev/null 2>&1
    echo
    echo "*** error: cannot create folder $file"
    exit -1
  fi  
done
FILES=`cat ./scripts/gcc/files.txt` > /dev/null 2>&1
for file in $FILES
do
  cp $file /tmp/chai/$PACKAGE/$file
  if [ $? -ne 0 ]
  then
    cd - > /dev/null 2>&1
    echo
    echo "*** error: cannot copy file $file"
    exit -1
  fi
done
cd - > /dev/null 2>&1
echo " ok"

# perform keyword conversion
echo -n "replacing internal version info to $MAJOR.$MINOR.$RELEASE.$REVISION..."
cd /tmp/chai/$PACKAGE > /dev/null 2>&1
for file in `grep -rl '$MAJOR' *`
do
  sed $SED_INPLACE -e's/$MAJOR/'$MAJOR'/g' $file
done
for file in `grep -rl '$MINOR' *`
do
  sed $SED_INPLACE -e's/$MINOR/'$MINOR'/g' $file
done
for file in `grep -rl '$RELEASE' *`
do
  sed $SED_INPLACE -e's/$RELEASE/'$RELEASE'/g' $file
done
for file in `grep -rl '$REVISION' *`
do
  sed $SED_INPLACE -e's/$REVISION/'$SVNREV'/g' $file
done
cd - > /dev/null 2>&1
echo " ok"

# create release archive
echo -n "archiving distribution to $ARCHIVE..."
mkdir -p $RELEASEDIR > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo
  echo "** error: cannot create $RELEASEDIR folder"
  popd
  exit -1
fi
rm -f $RELEASEDIR/$ARCHIVE > /dev/null 2>&1
cd /tmp/chai > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo
  echo "** error: cannot cd to /tmp/chai"
  popd
  exit -1
fi
case $OS in
linux)
  tar -zcvf $RELEASEDIR/$ARCHIVE $PACKAGE > /dev/null 2>&1
  ;;
mac)
  hdiutil create -ov -fs HFS+ -srcfolder . -volname $PACKAGE $RELEASEDIR/$ARCHIVE > /dev/null 2>&1
  ;;
*)
  echo "*** error: unsupported operating system ($OS)"
  ;;
esac
if [ $? -ne 0 ]
then
  cd - > /dev/null 2>&1
  echo
  echo "** error: archive creation failed"
  popd
  exit -1
fi
cd - > /dev/null 2>&1
echo " ok"

echo  
echo "$PACKAGE release for $OS-$ARCH created successfully"
echo
