#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2012-01-16 17:32:45 -0800 (Mon, 16 Jan 2012) $
#  $Rev: 684 $


#! /bin/sh

echo
TOP=.

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
CURDIR=`pwd`
PACKAGE=chai3d-$MAJOR.$MINOR.$RELEASE
ARCHIVE=$PACKAGE-$OS-$ARCH.$ARCHIVE_EXT

# start with a clean slate
echo -n "cleaning existing binaries..."
make ARCH=$ARCH CFG=debug clean > /dev/null 2>&1
make ARCH=$ARCH clean > /dev/null 2>&1
rm -rf lib > /dev/null 2>&1
echo " ok"

# produce debug version
echo -n "building $OS-$ARCH debug binaries..."
make ARCH=$ARCH CFG=debug > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo
  echo "** error: compilation failed"
  exit -1
fi 
echo " ok"

# produce release version
echo -n "building $OS-$ARCH release binaries..."
make ARCH=$ARCH > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo
  echo "** error: compilation failed"
  exit -1
fi 
echo " ok"

# copy release files to temporary folder
echo -n "copying distribution to temporary folder..."
rm -rf /tmp/chai > /dev/null 2>&1
mkdir -p /tmp/chai/$PACKAGE > /dev/null 2>&1
cp -r . /tmp/chai/$PACKAGE > /dev/null 2>&1
pushd /tmp/chai > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo
  echo "** error: copy to temporary folder failed"
  exit -1
fi
echo " ok"

# remove unwanted content
echo -n "removing unwanted content..."
rm -rf `find . -name "distrib.sh"` > /dev/null 2>&1
rm -rf `find . -name ".svn"` > /dev/null 2>&1
rm -rf `find . -name "*.tar.gz"` > /dev/null 2>&1
rm -rf `find . -name "*.dmg"` > /dev/null 2>&1
rm -rf `find . -type d -name "obj"` > /dev/null 2>&1
rm -rf `find . -mindepth 1 -name ".*"` > /dev/null 2>&1
rm -rf `find . -name "*~"` > /dev/null 2>&1
rm -rf `find . -name "libchai3d.a"` > /dev/null 2>&1
rm -rf `find . -name "libchai3d-dbg.a"` > /dev/null 2>&1
echo " ok"

# perform keyword conversion
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

# produce documentation
echo -n "building documentation..."
pushd $PACKAGE/doc/doxygen > /dev/null 2>&1
doxygen Doxyfile > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo  
  echo "** error: documentation failed"
  popd  
  exit -1
fi 
popd > /dev/null 2>&1
echo " ok"

# create release archive
echo -n "archiving distribution to $ARCHIVE..."
rm -f $CURDIR/$ARCHIVE > /dev/null 2>&1
case $OS in
linux)
  tar -zcvf $CURDIR/$ARCHIVE $PACKAGE > /dev/null 2>&1
  ;;
mac)
  hdiutil create -ov -fs HFS+ -srcfolder . -volname $PACKAGE $CURDIR/$ARCHIVE > /dev/null 2>&1
  ;;
*)
  echo "*** error: unsupported operating system ($OS)"
  ;;
esac
if [ $? -ne 0 ]
then
  echo
  echo "** error: archive creation failed"
  popd
  exit -1
fi
popd > /dev/null 2>&1
echo " ok"

echo  
echo "$PACKAGE release for $OS-$ARCH created successfully"
echo
