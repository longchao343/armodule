Developers Guide
=========================
This documents provides information how to develope the ARModule. 

Clone repository:
```bash
sudo apt-get install git
git clone https://github.com/nubomedia-vtt/armodule.git
```

Setup the developing environment
```bash
cd armodule/misc
chmod u+x setup.sh
./setup.sh
cd ..
```

Create a folder to build the ARModule
```bash
mkdir build
cd build
```

Now you have an option to generate either Java or JavaScript interface for the Kurento Client (or naturally both if you need)

Java version
```bash
cmake .. -DGENERATE_JAVA_CLIENT_PROJECT=TRUE
```

JavaScript version
```bash
cmake .. -DGENERATE_JS_CLIENT_PROJECT=TRUE
``` 

To use the compiled results in Kurento just set into:
```bash
/etc/default/kurento-media-server-6.0
```

path to the armodule/build and ALVAR libarary, for example:
```bash
export DISPLAY=:0
NAME=$(logname)
export KURENTO_MODULES_PATH=$KURENTO_MODULES_PATH:/home/$NAME/nubomedia/armodule/build
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/home/$NAME/nubomedia/armodule/build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$NAME/nubomedia/alvar-2.0.0-sdk-linux-gcc44/bin/
```

Please for details see also 
[ARModule Demo](https://github.com/nubomedia-vtt/armoduledemos)
for details and as an example how the ARModule can be utilized.

