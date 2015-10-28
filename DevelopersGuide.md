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

