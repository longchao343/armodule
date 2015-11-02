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

