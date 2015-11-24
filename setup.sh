#!/bin/bash
set -e -x
echo Installing armarkerdetector filter development environment
echo Version 2015-03-03
echo @author Markus Ylikerala, VTT, http://www.vtt.fi/

#######################################
# Tweak the TARGET variable if needed
# default is ~/nubomedia 
#######################################
USER_HOME=$(eval echo ~${SUDO_USER})
TARGET=${USER_HOME}/nubomedia 
mkdir -p $TARGET

#Alvar
cd $TARGET
wget -nd http://ssi.vtt.fi/ar-markerdetector-binaries/alvar-2.0.0-sdk-linux64-gcc44.tar.gz
gzip -d alvar-2.0.0-sdk-linux64-gcc44.tar.gz
tar xf alvar-2.0.0-sdk-linux64-gcc44.tar
sudo cp $TARGET/alvar-2.0.0-sdk-linux64-gcc44/bin/libalvar200.so /usr/local/lib/

#Irrlicht
sudo apt-get install subversion -y
svn checkout --revision 5029 svn://svn.code.sf.net/p/irrlicht/code/trunk/ irrlicht-code 

sudo apt-get install build-essential -y
sudo apt-get install mesa-common-dev -y
sudo apt-get install libxxf86vm-dev -y
sudo apt-get install libglu1-mesa-dev -y
sudo apt-get install xinit -y

VERSION_MAJOR="1"
VERSION_MINOR="9"
VERSION_RELEASE="0"
VERSION=$VERSION_MAJOR.$VERSION_MINOR.$VERSION_RELEASE
SHARED_LIB="libIrrlicht.so"
SHARED_FULLNAME=$SHARED_LIB.$VERSION
SONAME=$SHARED_LIB.$VERSION_MAJOR.$VERSION_MINOR
INSTALL_DIR="/usr/local/lib"
LIB_PATH=$TARGET/vtt-armarkerdetector/misc
sudo rm -rf $INSTALL_DIR/../include/irrlicht
sudo mkdir -p $INSTALL_DIR/../include/irrlicht
sudo cp $TARGET/irrlicht-code/include/*.h $INSTALL_DIR/../include/irrlicht/
wget -nd http://ssi.vtt.fi/ar-markerdetector-binaries/3d/$SHARED_FULLNAME
sudo cp $SHARED_FULLNAME $INSTALL_DIR
cd $INSTALL_DIR && sudo ln -s -f $SHARED_FULLNAME $SONAME
cd $INSTALL_DIR && sudo ln -s -f $SONAME $SHARED_LIB
sudo apt-get install libsoup2.4-dev -y

cp $TARGET/vtt-alvar/alvar-2.0.0-src/src/*.h $(pwd)/ar-markerdetector/src/server/implementation/objects/
cp $TARGET/alvar-2.0.0-sdk-linux64-gcc44/include/Alvar.h $(pwd)/ar-markerdetector/src/server/implementation/objects/
cp $TARGET/irrlicht-code/include/*.h $(pwd)/ar-markerdetector/src/server/implementation/objects/
make
