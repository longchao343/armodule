AR marker detector filter with 3D 
=========================

Installation instructions
-------------------------
*version 2015-09-24*

*Tested on Ubuntu Server 14*

*Tested on Ubuntu Desktop 14*


Recommend easy installation
=========================
These are the instructions to utilize the ar-markerdetector filter.

If you wish to have the most convenient installation,
you should install the following in this order:

KMS (this refers to the latest KMS version eg KMS6)

Option apt-get

Optional - ArMarkerDetectorDemo

Note that for each of these, installation instructions are given below.



Installing KMS
=========================
For installing Kurento Media Server (KMS),

please refer to 
```bash
www.kurento.org/
```

It is better to first install KMS so that you do not have to manually modify KMS config file.
But if you still install KMS afterwards ie after you have installed either Option apt-get or Option dpkg then after KMS is installed, add manually into /etc/default/kurento-media-server-6.0
```bash
export DISPLAY=:0
```


Installing with Option apt-get 
=========================
Choose this if you wish to install with apt-get the latest release version (not necessarily contains the latest features).

Execute:
```bash
echo "deb [arch=amd64] http://ssi.vtt.fi/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
sudo apt-get install ar-markerdetector
sudo ldconfig
sudo /etc/init.d/kurento-media-server-6.0 restart
```

Installing with Option dpkg
=========================
Note that this is alternative to the Option apt-get.

Choose this if you wish to install with dpkg the latest release version (not necessarily contains the latest features).

Fetch and install the debian package: [ar-markerdetector_0.0.6~rc1_amd64.deb](http://ssi.vtt.fi/ubuntu/dists/trusty/main/binary-amd64/amd64/ar-markerdetector_0.0.6~rc1_amd64.deb)
```bash
sudo dpkg -i ar-markerdetector_0.0.6~rc1_amd64.deb
sudo apt-get -f install
sudo ldconfig
sudo /etc/init.d/kurento-media-server-6.0 restart
```

Installing Optional ArMarkerDetectorDemo
=========================
These are the instructions how to utilize the ar-markerdetector filter that you have just installed.

Thus, after installation of ar-markerde done, you can test the filter with this sample application.

It is assumed that the sample application will be installed to the same host
where KMS is installed. Otherwise change the code of ar3d.

**Installing**

Fetch and execute the installation script:
```bash
wget -N http://80.96.122.50/Markus.Ylikerala/vtt-armarkerdetector/raw/master/optional.sh
chmod u+x optional.sh
./optional.sh
```

**For Ubuntu Server Users**

Run X:
```bash
xinit
```
If you're not authorized to run X, the following might help:
```bash
sudo dpkg-reconfigure x11-common
```

**For Ubuntu Desktop Users**

If you're not authorized to access X, then:
```bash
xhost +
```

Running Optional ArMarkerDetectorDemo
=========================

**On the Server Side**

To lauch the test program
```bash
cd ~/nubomedia/vtt-armarkerdetector/ar3d
mvn compile exec:java -Dexec.mainClass="fi.vtt.nubomedia.kurento.Ar3DApp"
```

**On the Client Side**

Browse with WebRTC compliant browser (eg Chrome, Firefox) 
to the server where ar3d is launched http://IP_OF_AR3DHOST:8080/
Change the IP_OF_AR3DHOST and port (8080) if needed.

You should now see AR Demo so just follow the given instructions on that page.

**Note For Ubuntu Desktop Users**

On some environments, for some reason, it has been reported that since KMSv6 
screen can get partially or totally blank. But because ssh works, it might
be better to install ssh server beforehand to gain control to the host again.


Details of the Demo
---------
Ar3D Kurento Client gets the data that is passes to the filter as json either from the browser ie javascipt or from the file system
For this demo the default json data is gotten from the browser if you execute:
```bash
mvn compile exec:java -Dexec.mainClass="fi.vtt.nubomedia.kurento.Ar3DApp"
```

But json data is gotten from the file system if executed with a valid json file eg [local.json](http://80.96.122.50/Markus.Ylikerala/vtt-armarkerdetector/blob/master/ar3d/local.json)
```bash
mvn compile exec:java -Dexec.mainClass="fi.vtt.nubomedia.kurento.Ar3DApp" -Dexec.args="local.json"
```

Ar3D Kurento Client passes data related to the augmentation via key-value pairs eg {”model”, ”/opt/cube.ply”}, {”scale”, 0.5f}.
This is described in kmd [armarkerdetector.ArMarkerdetector.kmd.json](http://80.96.122.50/Markus.Ylikerala/vtt-armarkerdetector/blob/master/ar-markerdetector/src/server/interface/armarkerdetector.ArMarkerdetector.kmd.json)
Please refer to this kmd to find out the syntax of json currently in use.

The models, images etc are loaded from the file system or via URL.

Thus, please check that the paths/URLs are correct ie that eg /opt/cube.ply is readable.

Material
=========================
**Alvar Markers**

Some Alvar markers can be found eg from [AlvarMarkers](http://ssi.vtt.fi/ar-markerdetector-binaries/demo/AlvarMarkers) 
Eg the zero marker is defined as MarkerData_0.png

**Planars**

Some planar ie images utilized as markers models can be found eg from [planars](http://80.96.122.50/Markus.Ylikerala/vtt-armarkerdetector/tree/master/planars) 

**Augmented models**

Some models can be found eg from [models](http://80.96.122.50/Markus.Ylikerala/vtt-armarkerdetector/tree/master/models) 

