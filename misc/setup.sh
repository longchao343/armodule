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
TARGET=${USER_HOME}/nubomedia2
mkdir -p $TARGET
MISCPATH=$(pwd)

#Alvar
cd $TARGET
wget -nd http://ssi.vtt.fi/ar-markerdetector-binaries/alvar-2.0.0-sdk-linux64-gcc44.tar.gz
gzip -d alvar-2.0.0-sdk-linux64-gcc44.tar.gz
tar xf alvar-2.0.0-sdk-linux64-gcc44.tar
sudo cp $TARGET/alvar-2.0.0-sdk-linux64-gcc44/bin/libalvar200.so /usr/local/lib/
sudo apt-get install libsoup2.4-dev -y

#Irrlicht
sudo apt-get install subversion -y
svn checkout --revision 5029 svn://svn.code.sf.net/p/irrlicht/code/trunk/ irrlicht-code 
sudo apt-get install build-essential -y
sudo apt-get install mesa-common-dev -y
sudo apt-get install libxxf86vm-dev -y
sudo apt-get install libglu1-mesa-dev -y
sudo apt-get install xinit -y

cp $MISCPATH/IrrCompileConfig.h $TARGET/irrlicht-code/include
cp $MISCPATH/Makefile $TARGET/irrlicht-code/source/Irrlicht/

cd $TARGET/irrlicht-code/source/Irrlicht
make -j sharedlib NDEBUG=1
sudo make install

cp $TARGET/vtt-alvar/alvar-2.0.0-src/src/*.h $MISCPATH/../ar-markerdetector/src/server/implementation/objects/
cp $TARGET/alvar-2.0.0-sdk-linux64-gcc44/include/Alvar.h $MISCPATH/../ar-markerdetector/src/server/implementation/objects/
cp $TARGET/irrlicht-code/include/*.h $MISCPATH/../ar-markerdetector/src/server/implementation/objects/
make
