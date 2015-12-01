/** @author Markus Ylikerälä */
 #include "Process.h"

 #include <opencv2/objdetect/objdetect.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include "unistd.h"
 #include "sys/wait.h"
 #include <GL/gl.h>
 #include <GL/glext.h>
#include "ArMarkerdetector.hpp"
 #include "MarkerDetector.h"
 #include "ArMarkerPose.hpp"
 #include "OverlayType.hpp"
 //#define USE_URL_WGET

 #define USE_URL_SOUP
 #ifdef USE_URL_SOUP
 #include <libsoup/soup.h>
 #endif

#include <sys/time.h>

#include <irrlicht.h>

#include <iostream>



#define SHOWMATRIX 0

using namespace irr;
using namespace video;
using namespace scene;
using namespace cv;
using namespace std;
using namespace kurento;
using namespace module;
using namespace armarkerdetector;

int ArProcess::counter = 0;


class OwnData {
public:
  alvar::MarkerDetector<alvar::MarkerData> *marker_detector;
  PlanarTracker *pose_tracker;
  OwnData() : marker_detector(0), pose_tracker(0) {}
};

void ArProcess::initAr(cv::Mat& mat){
  std::cout << "initAR resolution w/h: " << mat.cols << "/" << mat.rows << std::endl;   

  markerPoseFrequencyMs = 1000.0f / markerPoseFrequency;

  isArInitialized = 0;
  if(device){
    std::cout<<"TEAR 3D ENGINE" << std::endl << std::flush;
    if(smgr){
      smgr->clear();
    }
    device->drop();
    reboot = true;
  }

  std::cout<<"INIT 3D ENGINE" << std::endl << std::flush;
  video::E_DRIVER_TYPE driverType = irr::video::E_DRIVER_TYPE(irr::video::EDT_OPENGL);
//video::E_DRIVER_TYPE driverType = irr::video::E_DRIVER_TYPE(irr::video::EDT_BURNINGSVIDEO);

#if 0
SIrrlichtCreationParameters p;
	p.DriverType = driverType;
	p.WindowSize = core::dimension2d<u32>(mat.cols, mat.rows);
	p.Bits = 32;
	p.Fullscreen = false;
	p.Stencilbuffer = false;
	p.Vsync = false;
	p.EventReceiver = NULL;

	//device = irr::createDeviceEx(p);
	device = new CIrrDeviceFB(p);
	//SIrrlichtCreationParameters params;
	//IrrlichtDevice dev = null;


	//dev = new CIrrDeviceFB(params);	
#else
	//  device = irr::createDevice(driverType, core::dimension2d<u32>(mat.cols, mat.rows), 32, true, false);
  device = irr::createDevice(driverType, core::dimension2d<u32>(mat.cols, mat.rows), 32, true, false);
#endif

  if(device){
    driver = device->getVideoDriver();
    driver->setViewPort(core::rect<s32>(0, 0, mat.cols, mat.rows));
    smgr = device->getSceneManager();
    ILightSceneNode* light = smgr->addLightSceneNode( 0, core::vector3df(50.0f,50.0f,50.0f), video::SColorf(1.0f,1.0f,1.0f,1.0f), 500.0f );
    
    smgr->setAmbientLight(video::SColor(0,80,80,80));
    rt = 0;
    fixedCam = 0;
    if(driver->queryFeature(video::EVDF_RENDER_TO_TARGET)){
      rt = driver->addRenderTargetTexture(core::dimension2d<u32>(mat.cols, mat.rows), "RTT1");
      fixedCam = smgr->addCameraSceneNode(0, core::vector3df(0,0,-0.1), core::vector3df(0,0,0));

      if(rt){
	std::cout<<"RTT OK" << std::endl;  
	device->run();
	std::cout<<"IRRLICHT DEVICE IS RUNNING" << std::endl;

	driver->setRenderTarget(rt, true, true, video::SColor(0,0,0,255));   

	width = mat.cols;
	height = mat.rows;

	ardummy = 1;
	isArInitialized = 1;
	std::cout<<"READY FOR 3D" << std::endl << std::flush;
      }
    }
    else{
      std::cout<<"CANNOT RENDER TO TARGET" << std::endl << std::flush;
    }
  }
  else{
    std::cout<<"BIZARRE IRRLICH DEVICE" << std::endl;
  }
}

ArProcess::~ArProcess() {
  std::cout<<"BYE ~ArProcess() BEGIN" << std::endl << std::flush;
  if(device){
  std::cout<<"BYE 1" << std::endl << std::flush;
    device->drop();
  }

if(planarTracker){
delete planarTracker;
}

  if (owndata){
    if (static_cast<OwnData*>(owndata)->marker_detector) delete static_cast<OwnData*>(owndata)->marker_detector;

    if (static_cast<OwnData*>(owndata)->pose_tracker) delete static_cast<OwnData*>(owndata)->pose_tracker;

    delete static_cast<OwnData*>(owndata);
    owndata = 0;
  }

  //TODO 2014-12-20
  pthread_mutex_destroy(&mMutex);
  arThings.clear();
  
  std::cout<<"BYE ~ArProcess() END" << std::endl << std::flush;
}

void ArProcess::writeImage(std::string name, cv::Mat& mat){
  cv::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  imwrite(name, mat, compression_params);
}

void ArProcess::show(int isrowmajor, irr::core::CMatrix4<f32> m, std::string label){
  if(isrowmajor != 0){
    std::cout<< label << std::endl<<std::flush;
    for(int i=0, k=0; i<4; i++){
      for(int j=0; j<4; j++, k++){
	std::cout<<" " << m[k];
      }   
      std::cout<< std::endl<<std::flush;
    } 
  }
  else{
    std::cout << label << std::endl<<std::flush;
    for(int i=0; i<4; i++){
      std::cout<< " "<< m[i] << " "<< m[i+4]<< " "<< m[i+8]<< " " << m[i+12];
      std::cout<< std::endl<<std::flush;
    } 
  }
}

double ArProcess::getMillisecondsTime2(){
  timespec tick;
  clock_gettime(CLOCK_MONOTONIC, &tick);
  return (tick.tv_sec) + (tick.tv_nsec)/1000000000.0;
}


unsigned long ArProcess::getMillisecondsTime()
{
  struct timeval tv;
  if(gettimeofday(&tv, NULL) != 0){
    return 0;
  }
  return (unsigned long)((tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul));
}


void ArProcess::getTimeStamp(unsigned long *timeStamp){
  unsigned long currentTime = getMillisecondsTime();
  if(forTheVeryFirstTime == false){
    forTheVeryFirstTime = true;
    veryFirstTime = currentTime;
  }
  *timeStamp = currentTime - veryFirstTime;
}

bool ArProcess::poseEventTimeout(unsigned long timeStamp){
  if(timeStamp - oldTimer > markerPoseFrequencyMs){  
    oldTimer = timeStamp;
    return true;
  }
  return false;
}

void ArProcess::detect(cv::Mat& mat){
  //if (!owndata) return;
  if (!static_cast<OwnData*>(owndata)->marker_detector){
    return;
  }

  alvar::MarkerDetector<alvar::MarkerData> &marker_detector = *(static_cast<OwnData*>(owndata)->marker_detector);
  IplImage ipl = mat;
  IplImage *image = &ipl;

#if 1
#define MAX_FRAMESKIP 3
#if (MAX_FRAMESKIP > 0)
  // Skip up to 3 frames depending on the difference 
  // TODO: This is really complex. In reality we should skip based on latency on upper level.
  bool skip = false;
  static bool skip_prev = false;
  static unsigned long ms_frame_timestamp_prev=0;
  static double ms_framediff_with_detect=0;
  static double ms_framediff_without_detect=0;
  unsigned long ms_frame_timestamp;
  getTimeStamp(&ms_frame_timestamp);
  if (ms_frame_timestamp_prev > 0) {
    double alpha = 0.8;
    double ms_framediff = ms_frame_timestamp - ms_frame_timestamp_prev;
    if (skip_prev) ms_framediff_without_detect = alpha*ms_framediff_without_detect + (1-alpha)*ms_framediff;
    else           ms_framediff_with_detect = alpha*ms_framediff_with_detect + (1-alpha)*ms_framediff;
    double skip_frames = (ms_framediff_with_detect/(ms_framediff_without_detect+0.1)) - 1.0;
    if (skip_frames > MAX_FRAMESKIP) skip_frames = MAX_FRAMESKIP;
    //std::cout<<"skip frames: "<<skip_frames<<"\t="<<ms_framediff_with_detect<<"/"<<ms_framediff_without_detect<<"-1.0"<<std::endl;
    if (skip_frames > 1.) {
      static double skip_count=0;
      if (skip_count > skip_frames) skip_count -= skip_frames;
      else {
        skip_count += 1.0;
        skip = true;
      }
    }
  }
  skip_prev = skip;
  ms_frame_timestamp_prev = ms_frame_timestamp;
  if (skip) return;
#endif
#endif

#if SHOWMATRIX
  std::cout << std::endl <<"DETECT...:"
	    << " width(cols):" << image->width
	    << " height(rows):" << image->height
	    << " width(cols):" << mat.cols
	    << " height(rows):" << mat.rows
	    << std::endl<<std::flush;
#endif

  detectedMarkersPrev = detectedMarkers;
  std::map<int,int>::iterator iter;
  for (iter = detectedMarkers.begin(); iter != detectedMarkers.end(); iter++) {
    iter->second = 0; // Reset counters (but do not forget that this was seen previously?)
  }
  detectedMarkerData.clear();

  pthread_mutex_lock(&mMutex);

  marker_detector.Detect(image, &cam, true, (mShowDebugLevel > 0));
  if(marker_detector.markers->empty()) {
    if(planarTracker){
      planarTracker->trackPlanars(mat, detectedMarkerData);
    }
  }
  pthread_mutex_unlock(&mMutex);

  for(size_t i=0; i<marker_detector.markers->size(); i++){
    if(i >= 32){ 
      break;
    }
    alvar::MarkerData marker = (*(marker_detector.markers))[i];
    detectedMarkerData.push_back(marker);
    detectedMarkers[marker.GetId()]++;
  }
  //std::cout<<"DETECTED: "<<marker_detector.markers->size()<<" -> "<<detectedMarkerData.size()<<" -> "<<detectedMarkers.size()<<std::endl;
}


void ArProcess::solveProjectionMatrix(cv::Mat& mat){
  cam.SetSimpleCalib(mat.cols, mat.rows, 0.85);  
  cam.GetOpenglProjectionMatrix(pmatrix, mat.cols, mat.rows);
  if(planarTracker){
    planarTracker->solveProjectionMatrix(mat);
  }
}

void ArProcess::proactivate(cv::Mat& mat){
  if(width != mat.cols || height != mat.rows){
    std::cout<<"proactivate initAr" << std::endl;
    initAr(mat);
    if(isArInitialized == 0){
      std::cout<<"Bizarre initAR" << std::endl;
      return;
    }
    solveProjectionMatrix(mat);
    //std::cout << std::endl <<"***SMART RESOLUTION\t" << getSubTime()<< std::endl;
  }
  if(reboot){
    //std::cout<<"try to load content" << std::endl;
    if(isArInitialized){
      populate();
      reboot = false;
      solveProjectionMatrix(mat);
      SMART_TIMESTAMP("REBOOT", getSubTime());
    }
  }
}

void ArProcess::setArThing (const std::vector<std::shared_ptr<ArThing>> setArThing){
  std::cout<<"\n\n\nsetArThing()" << std::endl;
  pthread_mutex_lock(&mMutex);
  arThings.clear();
  //flatArThings.clear();  
  for(const auto& it : setArThing){
    ArThing *arThing = it.get();
    arThings[arThing->getMarkerId()] = it;
  }
  reboot = true;
  char tmpbase[] = "/tmp/ar_XXXXXX";
  tmpdir = mkdtemp(tmpbase);
  pthread_mutex_unlock(&mMutex);
}


void ArProcess::addFgWithAlpha(cv::Mat &bg, cv::Mat &fg) {
   if (fg.channels() < 4) {
     fg.copyTo(bg);  
     return;
   }
   std::vector<cv::Mat> splitted_bg, splitted_fg;
   cv::split(bg, splitted_bg);
   cv::split(fg, splitted_fg);
   cv::Mat mask = splitted_fg[3];
   cv::Mat invmask = ~mask;
   splitted_bg[0] = (splitted_bg[0] - mask) + (splitted_fg[0] - invmask);
   splitted_bg[1] = (splitted_bg[1] - mask) + (splitted_fg[1] - invmask);
   splitted_bg[2] = (splitted_bg[2] - mask) + (splitted_fg[2] - invmask);
   if (bg.channels() > 3) {
     splitted_bg[3] = splitted_bg[3] + splitted_fg[3];
   }
   cv::merge(splitted_bg, bg);
 }

#ifdef USE_URL_SOUP
bool ArProcess::load_from_url(const char *file_name, const char *url) {
   SoupSession *session;
   SoupMessage *msg;
   FILE *dst;
   session = soup_session_sync_new ();
   msg = soup_message_new ("GET", url);
   if(msg == NULL){
     g_object_unref (session);
     return false;
   }
   soup_session_send_message (session, msg);
   dst = fopen (file_name, "w+");
   if (dst == NULL) {
     g_object_unref (msg);
     g_object_unref (session);
     return false;
   }
   fwrite (msg->response_body->data, 1, msg->response_body->length, dst);
   fclose (dst);
   g_object_unref (msg);
   g_object_unref (session);
   return true;
   
#if 0
   SoupSession *session;
   SoupMessage *msg;
   FILE *dst;
   session = soup_session_sync_new ();
   msg = soup_message_new ("GET", url);
   soup_session_send_message (session, msg);
   dst = fopen (file_name, "w+");
   if (dst == NULL) {
     goto end;
   }
  fwrite (msg->response_body->data, 1, msg->response_body->length, dst);
  fclose (dst);
return true;
end:
  g_object_unref (msg);
  g_object_unref (session);
return false;
#endif
}
#endif

bool ArProcess::readImage(std::string uri, cv::Mat& bg) {
  //cv::Mat bg;

#ifdef USE_URL_SOUP
  bg = cv::imread(uri, CV_LOAD_IMAGE_UNCHANGED);
  if ((!bg.data)) {
    //std::string tmpfile = ("/tmp/tmp.png");
    std::string tmpfile = tmpdir + "/tmp.png";
    std::cout<<"SOUP: Loading URL image:" << uri << " to temp file: "<<tmpfile<<std::endl;
    if(load_from_url (tmpfile.c_str(), uri.c_str()) == false){
      std::cout<<"Bizarre 2D file: " << uri <<std::endl;      
      return false;
    }
    
    bg = cv::imread(tmpfile, CV_LOAD_IMAGE_UNCHANGED);
  }
#endif

#ifdef USE_URL_WGET
  std::string tmpfile("/tmp/tmp.png"); //tmpnam(NULL));
  pid_t pid = fork();
  if (pid == 0) {
    std::cout<<"WGET: Loading URL image to temp file: "<<tmpfile<<std::endl;
    //TODO: Why this does not work: http://www.fnordware.com/superpng/straight.png
    //http://www.dplkbumiputera.com/slider_image/sym/root/proc/self/cwd/usr/share/zenity/clothes/monk.png
    //http://www.dplkbumiputera.com/slider_image/sym/root/proc/self/cwd/usr/share/zenity/clothes/hawaii-shirt.png
    //execlp("/usr/bin/wget", "/usr/bin/wget", "-O", "/tmp/tmp.png", "http://www.fnordware.com/superpng/straight.png", NULL);
    //execlp("/usr/bin/wget", "/usr/bin/wget", "-O", "/tmp/tmp.png", "http://www.dplkbumiputera.com/slider_image/sym/root/proc/self/cwd/usr/share/zenity/clothes/hawaii-shirt.png", NULL);
    //execlp("/usr/bin/wget", "/usr/bin/wget", "-O", "/tmp/tmp.png", "http://www.dplkbumiputera.com/slider_image/sym/root/proc/self/cwd/usr/share/zenity/clothes/sunglasses.png", NULL);
    execlp("/usr/bin/wget", "/usr/bin/wget", "-O", tmpfile.c_str(), uri.c_str(), NULL);
    printf("\nError: Could not execute wget\n");
    //_exit(0);
    return false;
  } else if (pid > 0) {
    int status;
    waitpid(pid, &status, 0);
  }
  bg = cv::imread(tmpfile, CV_LOAD_IMAGE_UNCHANGED);
#endif

#ifdef USE_URL_AS_FILE
  bg = cv::imread(uri, CV_LOAD_IMAGE_UNCHANGED);
#endif

  if (!bg.data) {
    bg = cv::Mat(256,256,CV_8UC3);
  }
  if (bg.channels() == 3) {
    cv::cvtColor(bg, bg, CV_BGR2BGRA);
  }
  std::cout<<"BG image: "<< uri << " " << bg.cols<<"x"<<bg.rows<<" channels: "<<bg.channels()<<std::endl;

  return true;
}

ArProcess::ArProcess() : owndata(0), mShowDebugLevel(0) {
  std::cout<<"Me Create ArProcess 1" << std::endl << std::flush;
  isArInitialized = 0;
  device = NULL;
  smgr = NULL;
  reboot = false;

  char tmpbase[] = "/tmp/ar_XXXXXX";
  tmpdir = mkdtemp(tmpbase);
  std::cout<<"\n\n\n*** JESTAS ***\ntmpDir: "<< tmpdir << "#" << string(tmpdir) <<std::endl;
  //tmpdir = std::string(tmpdir).c_str();
  pthread_mutex_init(&mMutex, NULL);

  owndata = new OwnData;
  static_cast<OwnData*>(owndata)->marker_detector = new alvar::MarkerDetector<alvar::MarkerData>();

  alvar::MarkerDetector<alvar::MarkerData> &marker_detector = *(static_cast<OwnData*>(owndata)->marker_detector);
  //marker_detector.SetMarkerSize(15);
  marker_detector.SetMarkerSize(2);


#if 1
planarTracker = new PlanarTracker();
#else
planarTracker = new NullTracker();
//planarTracker = new SmartTracker();
#endif
  
  oldTimer = 0;
  markerPoseFrequency = 1;
  sequenceNum = 0;
  markerPoseFrame = 0;
  device = NULL;
  width = -1;
  height = -1;
  forTheVeryFirstTime = false;
  dummyLoad = false;
}


bool ArProcess::isNthFrame(){
  markerPoseFrame++;
  if(markerPoseFrame == markerPoseFrameFrequency){
    markerPoseFrame = 0;
    return true;
  }
  return false;
}

void ArProcess::parseAugmentable(ArThing *arThing, std::map<std::string, std::string>& strings, std::map<std::string, float>& floats){
    std::cout<< std::endl <<"MARKER:"<< arThing->getMarkerId() << std::endl<<std::flush;
    for(const auto& it : arThing->getStrings()){
      ArKvpString *arKvpString = it.get();
      strings[arKvpString->getKey()] = arKvpString->getValue();
    }    
    for(const auto& it : arThing->getFloats()){
      ArKvpFloat *arKvpFloat = it.get();
      floats[arKvpFloat->getKey()] = arKvpFloat->getValue();
    }
}
			   
bool ArProcess::splitModel(std::string modelUrl, std::string &modelName, std::string &modelSuffix){
  unsigned found = modelUrl.find_last_of("/\\");
  if(found == string::npos){
    std::cout << "bizarre model uri " << std::endl;
    return false;
  }
  //modelName = "/tmp/" + modelUrl.substr(found+1);
  modelName = modelUrl.substr(found+1);
  
#if 1
  found = modelName.find_last_of(".");
  if(found == string::npos){
    std::cout << "bizarre model suffix " << std::endl;
    return false;
  }
  modelSuffix = modelName.substr(found+1);
#endif
return true;
  //  std::cout << "Try to load model: " << modelUrl << " from cache:" << modelLocal << " suffix:" << modelSuffix << std::endl;
}


bool ArProcess::fetchModel(std::string modelUrl, std::string &modelName){
  SoupURI *tst = soup_uri_new(modelUrl.c_str());
  if(SOUP_URI_IS_VALID(tst)) {
    std::cout << "valid uri yes" << std::endl;
    modelName = tmpdir + modelName; 
    std::cout << "fetch model: " << modelUrl << " to:" << modelName << std::endl;
    if(load_from_url(modelName.c_str(), modelUrl.c_str()) == false){
      std::cout<<"Bizarre loading of 3DModel" << std::endl;
      return false;
    }
  } 
  else {
    std::cout << "valid uri no" << std::endl;
    return false;
  }
  return true;
}
  
void ArProcess::animate(irr::scene::IAnimatedMeshSceneNode* node, std::map<std::string, std::string> strings, std::string modelSuffix){
  if(modelSuffix == "md2"){
    std::string  value = strings["animate"];
    if(value.compare("run") == 0){
      node->setMD2Animation(scene::EMAT_RUN);
      std::cout<<"Got Animate: run" << std::endl;
    }
    else if(value.compare("walk") == 0){
      node->setMD2Animation(scene::EMAT_CROUCH_WALK);
      std::cout<<"Got Animate: walk" << std::endl;
    }
    else if(value.compare("attack") == 0){
      node->setMD2Animation(scene::EMAT_ATTACK);
      std::cout<<"Got Animate: attack" << std::endl;
    }
    else{
      node->setMD2Animation(scene::EMAT_STAND);
      std::cout<<"Got Animate: stand" << std::endl;
    }
  }
}

void ArProcess::erasePopulation(){
  flatsScale.clear();  
  flats.clear();  
  fixedNodes.clear();
  nodes.clear();
  planarTracker->clear();
  positions.clear();
  rotations.clear();
}

irr::scene::IAnimatedMeshSceneNode* ArProcess::loadNode(std::string modelUrl, std::string& modelSuffix){
  std::string modelName = "";
  //std::string modelSuffix = "";
  //std::string modelUrl = strings["model"];      
  
  if(splitModel(modelUrl, modelName, modelSuffix) == false){
    std::cout << "Bizarre local model skippedA:" << modelUrl << std::endl;
    return NULL;
  }
  
  IAnimatedMesh* mesh = smgr->getMesh(modelUrl.c_str());
  if(!mesh){
    if(fetchModel(modelUrl, modelName) == false){
      std::cout << "Bizarre local model skippedB:" << modelUrl << std::endl;
      return NULL;
    }      
    mesh = smgr->getMesh(modelName.c_str());
    if(!mesh){
      std::cout << "Bizarre remote model skippedC:" << modelUrl << std::endl;
      return NULL;
    }
  }

  mesh->setHardwareMappingHint(scene::EHM_STATIC);
  //  mesh->setHardwareMappingHint(scene::EHM_STREAM);
  return smgr->addAnimatedMeshSceneNode(mesh);      
}

void ArProcess::modifyNode(ArThing *arThing, std::map<std::string, std::string>& strings, std::map<std::string, float>& floats, irr::scene::IAnimatedMeshSceneNode* node, std::string modelSuffix){
  node->setVisible(false);
  float factor = 1.0f;
  if(floats.find("scale") != floats.end()){
    factor = floats["scale"];
  }
  node->setScale(core::vector3df(factor, factor, factor));
  node->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
  
  positions[arThing->getMarkerId()] = core::vector3df(0.0f, 0.0f, 0.0f);
  rotations[arThing->getMarkerId()] = core::vector3df(0.0f, 0.0f, 0.0f);
  
  if(strings.find("texture") != strings.end()){
    node->setMaterialTexture( 0, driver->getTexture(strings["texture"].c_str()));
    node->getMaterial(0).Shininess = 10.0f; 
  }
  if(strings.find("animate") != strings.end()){
    animate(node, strings, modelSuffix);
  }
}

void ArProcess::populate(){
  pthread_mutex_lock(&mMutex);

  if(tmpdir.length() == 0){
    std::cout << "Bizarre tmp path" << std::endl;
    return;
  }

  erasePopulation();

  for(std::map<int, std::shared_ptr<ArThing>>::iterator it = 
	arThings.begin(); it != arThings.end(); ++it){      
    ArThing *arThing = it->second.get();
    std::map<std::string, std::string> strings;
    std::map<std::string, float> floats;
    parseAugmentable(arThing, strings, floats);      

    std::cout << "Loading model from: >>>" << strings["model"] << "<<<" << std::endl;

    std::cout << "GO A " << std::endl;
    if(strings.find("model") == strings.end()){
    std::cout << "GO B" << std::endl;
      std::cout<<"Bizarre XDModel" << std::endl;
      continue;
    }
    std::cout << "GO C" << std::endl;

    if (strings.find("detect_planar") != strings.end()) {
      std::cout<<"got detect_planar" << std::endl;
      planarTracker->planar_image_ids[strings["detect_planar"]] = arThing->getMarkerId();
      std::cout<<"Planar image \""<<strings["detect_planar"]<<" can replace marker "<<arThing->getMarkerId()<<std::endl;
    }
    
    std::shared_ptr<OverlayType> overlayType = arThing->getOverlayType();

    if(OverlayType::type::TYPE3D == arThing->getOverlayType()->getValue()){
      std::cout << "GO 3D";

      std::string modelSuffix = "";
      irr::scene::IAnimatedMeshSceneNode* node = loadNode(strings["model"], modelSuffix);
      if(!node){
	continue;
      }
      std::cout << "GO 3D suffix: " << modelSuffix << std::endl;
      modifyNode(arThing, strings, floats, node, modelSuffix);     

      if(strings.find("detect_usefixedpose") != strings.end()){
	std::cout<<"JES FIXED POSE" << std::endl;
	float factor[3];
	if(floats.find("x") == floats.end() ||
	   floats.find("y") == floats.end() ||
	   floats.find("z") == floats.end()){
	  std::cout<<"BIZARRE FIXED POSE" << std::endl;
	  continue;
	}
	fixedNodes[arThing->getMarkerId()] = node;
	node->setVisible(true);
	node->setPosition(core::vector3df(floats["x"], floats["y"], floats["z"]));
      }
      else if(strings.find("detect_usefixedcoordinates") != strings.end()){
	std::cout<<"JES FIXED COORDS" << std::endl;
	fixedNodes[arThing->getMarkerId()] = node;
	node->setVisible(true);
      }
      else{
	//"detect_planar" and plain marker
	nodes[arThing->getMarkerId()] = node;
      }
    }
    else if(OverlayType::type::TYPE2D == arThing->getOverlayType()->getValue()){
      std::cout << "GO 2D";
      std::string image = strings["model"];
      std::string text = strings["label"];
      float factor = floats["scale"];
      std::cout<<"2DModel" << image << "#" << text << "#" << factor << std::endl << std::flush;
      cv::Mat overlay = set_overlay(image, text, factor);
      std::cout<<"got overlay" << std::endl << std::flush;
      if(overlay.cols != 0 && overlay.rows != 0){
	std::cout<<"overlay ok" << arThing->getMarkerId() << std::endl << std::flush;
	//flatArThings[arThing->getMarkerId()] = (std::shared_ptr<ArThing>)arThing;
	flats[arThing->getMarkerId()] = overlay;
	float factor = floats["scale"];
	flatsScale[arThing->getMarkerId()] = factor;
	std::cout<<"OK 2DModel" << std::endl;
      }
      else{
	std::cout<<"Bizarre 2DModel" << std::endl;
      }
    }
    else{
      std::cout<<"Bizarre XDModel" << std::endl;
    }
    std::cout << "...LOADED" << std::endl << std::flush;
  }

  if(planarTracker->planar_image_ids.size() > 0){
    std::cout<<"yes createPlanarConfiguration" << std::flush;
    planarTracker->createPlanarConfiguration(tmpdir, width, height);
    std::cout << "...PLANARED" << std::endl << std::flush;
  }

  pthread_mutex_unlock(&mMutex);
  //std::cout << std::endl <<"***SMART GOGO\t" << getSubTime()<< std::endl;
}

cv::Mat ArProcess::set_overlay(std::string overlay_image, 
			       std::string overlay_text, 
			       float scale){
//  pthread_mutex_lock(&mMutex);
  cv::Mat fg, bg;

  if (overlay_image.length() > 0) {
    if(readImage(overlay_image, bg) == false){
      //      pthread_mutex_unlock(&mMutex);
      return cv::Mat(0, 0, CV_8UC4);
    }
  }
  if (overlay_text.length() > 0) {
    int font = cv::FONT_HERSHEY_PLAIN, font_thickness = 3;
    double font_scale = 6.0;
    int baseline=0;
    cv::Size textSize = cv::getTextSize(overlay_text.c_str(), font, font_scale, font_thickness, &baseline);
    fg = cv::Mat(textSize.height+baseline+4, textSize.width+4, CV_8UC4); // TODO: CV_8UC4
    fg.setTo(cv::Scalar(255,255,255,64));
    cv::putText(fg, overlay_text.c_str(), cv::Point(2, fg.rows-(baseline/2)), font, font_scale, cv::Scalar(0,64,0,255), font_thickness);
  }
  int res = 0;
  if (bg.data) res = std::max(bg.cols, bg.rows);
  if (fg.data) res = std::max(res, std::max(fg.cols, fg.rows));

  if (res < 1){
    //    pthread_mutex_unlock(&mMutex);
    return cv::Mat(0, 0, CV_8UC4);
  }
  cv::Mat overlay = cv::Mat(res, res, CV_8UC4);
  overlay.setTo(cv::Scalar(255,255,255,0));
  if (bg.data) {
    double fx = res/std::max(bg.cols, bg.rows), fy = fx;
    cv::resize(bg, bg, cv::Size(), fx, fy);
    cv::Mat overlay_bg(overlay, cv::Rect(
      (overlay.cols-bg.cols)/2, (overlay.rows-bg.rows)/2,
      bg.cols, bg.rows));
    bg.copyTo(overlay_bg);
  }
  if (fg.data) {
    double fx = res/std::max(fg.cols, fg.rows), fy = fx;
    cv::resize(fg, fg, cv::Size(), fx, fy);
    cv::Mat overlay_fg(overlay, cv::Rect(
      (overlay.cols-fg.cols)/2, (overlay.rows-fg.rows)/2,
      fg.cols, fg.rows));
    addFgWithAlpha(overlay_fg, fg);
  }
//  pthread_mutex_unlock(&mMutex);

  return overlay;
}

void ArProcess::augment(cv::Mat& mat){
  if(augmentationEnabled && detectedMarkerData.size() > 0){
    //proactivate(mat);
    if(isArInitialized){
      IplImage img = mat;
      //std::cout << std::endl <<"***SMART AUGMENT BEGIN\t" << getSubTime()<< std::endl;
      if(augment2DModel(&img) != 0){
	//std::cout << std::endl <<"***SMART AUGMENTED 2D\t" << getSubTime()<< std::endl;
      }
      if(augment3DModel(mat) != 0){
	//std::cout << std::endl <<"***SMART AUGMENTED 3D\t" << getSubTime()<< std::endl;
      }
      //std::cout << std::endl <<"***SMART AUGMENT END\t" << getSubTime()<< std::endl;
    }
  }
}

int ArProcess::augment2DModel(IplImage *image) {
  int augmented = 0;
  for(int i=0; i<detectedMarkerData.size(); i++){
    alvar::MarkerData marker = detectedMarkerData[i];
    if(flats.find(marker.GetId()) == flats.end()){
      continue;
    }
    
    cv::Mat warped_overlay(image->height, image->width, CV_8UC4);
    warped_overlay.setTo(cv::Scalar(0,0,0,0));
    cv::Mat overlay = flats[marker.GetId()]; 
    float overlayScale = flatsScale[marker.GetId()]; 

    //std::shared_ptr<ArThing> arThing = flatArThings[marker.GetId()];

    if (overlayScale > 0.5f) {
      cv::Point2f source_points[4];
      cv::Point2f dest_points[4];
      float overlay_maxdim = std::max(overlay.rows, overlay.cols);
      overlay_maxdim /= overlayScale;
      source_points[0] = cv::Point2f(overlay.cols/2, overlay.rows/2) + cv::Point2f(-overlay_maxdim/2,overlay_maxdim/2);
      source_points[1] = cv::Point2f(overlay.cols/2, overlay.rows/2) + cv::Point2f(overlay_maxdim/2,overlay_maxdim/2);
      source_points[2] = cv::Point2f(overlay.cols/2, overlay.rows/2) + cv::Point2f(overlay_maxdim/2,-overlay_maxdim/2);
      source_points[3] = cv::Point2f(overlay.cols/2, overlay.rows/2) + cv::Point2f(-overlay_maxdim/2,-overlay_maxdim/2);
      dest_points[0] = cv::Point2f(
        marker.marker_corners_img[0].x,
        marker.marker_corners_img[0].y);
      dest_points[1] = cv::Point2f(
        marker.marker_corners_img[1].x,
        marker.marker_corners_img[1].y);
      dest_points[2] = cv::Point2f(
        marker.marker_corners_img[2].x,
        marker.marker_corners_img[2].y);
      dest_points[3] = cv::Point2f(
        marker.marker_corners_img[3].x,
        marker.marker_corners_img[3].y);

      cv::Mat transform = cv::getPerspectiveTransform(source_points, dest_points);
      cv::warpPerspective(overlay, warped_overlay, transform, cv::Size(warped_overlay.cols, warped_overlay.rows), 0, cv::BORDER_TRANSPARENT); 

      std::cout<<"GOT OK flats: " << marker.GetId() << std::endl;   
    } else {
      // TODO: Does not work
      cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
      CvMat ipltransform = transform;
      marker.pose.GetMatrix(&ipltransform);
    }
    cv::Mat frame(image);
    addFgWithAlpha(frame, warped_overlay);
    augmented++;
  }
  return augmented;
}

int ArProcess::augment3DModel(cv::Mat& mat){
  int matSize = mat.rows*mat.cols*mat.elemSize();  

#if 0
  unsigned char *buffer = new unsigned char[matSize];
  memcpy(buffer, mat.data, matSize);


  SMART_TIMESTAMP("AUGMENTED 3D CV_2_IRR_BUFFER\t", getSubTime());

  video::IImage* img;
  int augmented  = 0;
  if(mat.channels() == 4){
    img = driver->createImageFromData(irr::video::ECOLOR_FORMAT(irr::video::ECF_A8R8G8B8), 
				      core::dimension2d<u32>(mat.cols, mat.rows), buffer);
  }
  else if (mat.channels() == 3){
    img = driver->createImageFromData(irr::video::ECOLOR_FORMAT(irr::video::ECF_R8G8B8), 
				      core::dimension2d<u32>(mat.cols, mat.rows), buffer);
  }
  else{
    std::cout<<"Bizarre channels: " << mat.channels()	      
	     << std::endl<<std::flush;
    delete buffer;
    return augmented;
  }
#else


  int augmented  = 0;
  video::IImage* img = driver->createImageFromData(irr::video::ECOLOR_FORMAT(irr::video::ECF_A8R8G8B8), core::dimension2d<u32>(mat.cols, mat.rows), mat.data);

#endif

    //driver->setRenderTarget(rt, true, true, video::SColor(0,0,0,255));   

  video::ITexture* imgTxt = driver->addTexture("bg", img);	


	
  driver->beginScene(true, true, 0);
  //std::cout<<"Bizarre A" << std::endl;    
  driver->draw2DImage(imgTxt, core::position2d<s32>(0,0),
		      core::rect<s32>(0,0, mat.cols, mat.rows), 0,
		      video::SColor(255,255,255,255), true);
  //std::cout<<"Bizarre B" << std::endl;      

  SMART_TIMESTAMP("AUGMENTED 3D CV_2_IRR", getSubTime());

  if(detectedMarkerData.size() > 0){
    irr::core::CMatrix4<f32> projectionMatrix = 
      fixedCam->getProjectionMatrix();

#if SHOWMATRIX
    show(true, projectionMatrix, "projectionMatrix");
#endif

    for(int i=0; i<16; i++){
      projectionMatrix[i] = pmatrix[i];
    }
    fixedCam->setProjectionMatrix(projectionMatrix);

    std::vector<int> shown;
    int MAX = 16;
    double amatrix[MAX];    
    for(int i=0; i<detectedMarkerData.size(); i++){
      alvar::MarkerData marker = detectedMarkerData[i];
      if(nodes.find(marker.GetId()) == nodes.end()){
	//std::cout<<"Bizarre Cannot find: " << marker.GetId()	      
	//	 << std::endl<<std::flush;
	continue;
      }
      shown.push_back(marker.GetId());
      marker.pose.GetMatrixGL(amatrix);              
      
      irr::core::CMatrix4<f32> imatrix;

#if 0
      f32 mirror[] = {
	1.0, 0.0, 0.0, 0.0,
	0.0, 1.0, 0.0, 0.0,
	0.0, 0.0, 1.0, 0.0,
	0.0, 0.0, 0.0, 1.0,
      };
#else
      f32 mirror[] = {
	1.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 1.0, 0.0,
	0.0, -1.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 1.0,
      };
#endif
      irr::core::CMatrix4<f32> imirror;
      imirror.setM(mirror);
      
      for(int i=0; i<MAX; i++){
	imatrix[i] = amatrix[i];
      }
      imatrix = imatrix * imirror;

#if SHOWMATRIX
      show(true, imatrix, "PoseMatrix");
#endif      

      core::vector3df position = positions[marker.GetId()];
      core::vector3df iposition = imatrix.getTranslation();
      iposition += position;
      nodes[marker.GetId()]->setPosition(iposition);

      core::vector3df rotation = rotations[marker.GetId()];
      core::vector3df irotation = imatrix.getRotationDegrees();
      irotation += rotation;
      nodes[marker.GetId()]->setRotation(irotation);

      //nodes[marker.GetId()]->setPosition(imatrix.getTranslation());
      //nodes[marker.GetId()]->setRotation(imatrix.getRotationDegrees());
      nodes[marker.GetId()]->setVisible(true);
      augmented++;
    }

    smgr->drawAll();
    
    for(std::vector<int>::iterator it=shown.begin(); it!=shown.end(); ++it){
      nodes[*it]->setVisible(false);
    }
    SMART_TIMESTAMP("AUGMENTED 3D RTT", getSubTime());
  }
  
  driver->endScene();
  SMART_TIMESTAMP("AUGMENTED 3D IRR_2_CV", getSubTime());
  driver->removeTexture(imgTxt);
  //SMART_TIMESTAMP("AUGMENTED 3D IRR_2_CV_2", getSubTime());
  img->drop();
  //SMART_TIMESTAMP("AUGMENTED 3D IRR_2_CV_3", getSubTime());
  
#if 1
  void* imgData   = rt->lock(video::E_TEXTURE_LOCK_MODE(video::ETLM_READ_ONLY));
  if(imgData != NULL){
    memcpy(mat.data, imgData, matSize);
  }
  else{
    if(ardummy != 0){
      std::cout<<"GOAR BIZARRE imgData of Texture:" << std::endl;
    }
  }
#endif

  rt->unlock();
  //SMART_TIMESTAMP("AUGMENTED 3D IRR_2_CV_4", getSubTime());  

#if 0
  delete buffer;
#endif

  //  SMART_TIMESTAMP("AUGMENTED 3D IRR_2_CV_5", getSubTime());
  
  return augmented;
}


void ArProcess::setPoseScale(irr::scene::IAnimatedMeshSceneNode* node, float value){
  core::vector3df scale = node->getScale();
  scale *= value;
  node->setScale(scale);
}

void ArProcess::setPosePosition(int id, float value, int type){
  core::vector3df position = positions[id];
  switch(type){
  case 0:
    position.X += value;
    break;
  case 1:
    position.Y += value;
    break;
  case 2:
    position.Z += value;
    break;
  }
  positions[id] = position;  
}

void ArProcess::setPoseRotation(int id, float value, int type){
  core::vector3df rotation = rotations[id];
  switch(type){
  case 0:
    rotation.X += value;
    break;
  case 1:
    rotation.Y += value;
    break;
  case 2:
    rotation.Z += value;
    break;
  }
  rotations[id] = rotation;
}

void ArProcess::setPose (int id, int type, float value){
  if(nodes.find(id) != nodes.end()){
    irr::scene::IAnimatedMeshSceneNode* node = nodes[id];
    if(node){
      switch(type){
      case 0:
	setPoseScale(node, value);
	break;
      case 1:
	setPoseScale(node, 1.0f/value);
	break;
      case 2:
	setPosePosition(id, -1.0f*value, 0);
	break;
      case 3:
	setPosePosition(id, value, 0);
	break;
      case 4:
	setPosePosition(id, -1.0f*value, 1);
	break;
      case 5:
	setPosePosition(id, value, 1);
	break;
      case 6:
	setPosePosition(id, -1.0f*value, 2);
	break;
      case 7:
	setPosePosition(id, value, 2);
	break;
      case 8:
	setPoseRotation(id, -1.0f*value, 0);
	break;
      case 9:
	setPoseRotation(id, value, 0);
	break;
      case 10:
	setPoseRotation(id, -1.0f*value, 1);
	break;
      case 11:
	setPoseRotation(id, value, 1);
	break;
      case 12:
	setPoseRotation(id, -1.0f*value, 2);
	break;
      case 13:
	setPoseRotation(id, value, 2);
	break;
      default:
	std::cout<<"setPose for type: "<< type << " not implemented yet" << std::endl<<std::flush;
	break;
      }
    }
  }
}

void ArProcess::generateEvents(std::shared_ptr<MediaObject> mo, sigc::signal<void, MarkerPose> signalMarkerPose, sigc::signal<void, MarkerCount> signalMarkerCount, sigc::signal<void, Tick> signalTick, cv::Mat& mat){
  ++sequenceNum;
  unsigned long timeStamp = 0;
  getTimeStamp(&timeStamp);

  if(detectedMarkerData.size() > 0 &&
     isPoseEventTime(timeStamp)){
    generatePoseEvents(signalMarkerPose, mo, timeStamp, mat);
  }

  if(markerCountEnabled){
    generateCountEvents(signalMarkerCount, mo, timeStamp);
  }

  if(ticksEnabled){
    generateTickEvents(signalTick, mo, timeStamp);
  }
  ticks.clear();
}

void ArProcess::generatePoseEvents(sigc::signal<void, MarkerPose> signalMarkerPose, std::shared_ptr<MediaObject> mo, unsigned long timeStamp, cv::Mat& mat){
  int MAX = 16;
  std::vector<float> matrixModelview(MAX);
  std::vector<float> matrixProjection(MAX);
  double matrix[MAX];    
  std::vector<std::shared_ptr<ArMarkerPose>> poses;      
  for (int i=0; i<detectedMarkerData.size(); i++){
    alvar::MarkerData marker = detectedMarkerData[i];
    marker.pose.GetMatrixGL(matrix);
    matrixModelview.assign(matrix, matrix+MAX);
    matrixProjection.assign(pmatrix, pmatrix+MAX);
    
    int marker_id = marker.GetId();
    try {
      ArMarkerPose *pose = new ArMarkerPose(marker_id, matrixModelview);
      std::shared_ptr<ArMarkerPose> a(pose);
      poses.push_back(a);	
    } 
    catch (std::bad_weak_ptr &e){
      std::cout<<"WEAK PTR EXP: "<< std::endl<<std::flush;
    }
    catch(std::exception const & ex){
      std::cout<<"POSE EXP: "<< std::endl<<std::flush;
    }
  }
#if 0
  MarkerPose event(sequenceNum, (int)timeStamp, 
		   mat.cols,
		   mat.rows,
		   matrixProjection, poses, mo, MarkerPose::getName() );
#endif
  MarkerPose event(mo, MarkerPose::getName(),
		   sequenceNum, (int)timeStamp, 
		   mat.cols,
		   mat.rows,
		   matrixProjection, poses);
  signalMarkerPose(event);
  poses.clear();
}

void ArProcess::generateCountEvents(sigc::signal<void, MarkerCount> signalMarkerCount, std::shared_ptr<MediaObject> mo, unsigned long timeStamp){

  std::map<int,int>::iterator iter;  
  for (iter = detectedMarkers.begin(); iter != detectedMarkers.end(); iter++) {
    int marker_id = iter->first;
    int marker_count = iter->second;
    int marker_count_prev = detectedMarkersPrev[marker_id];
    
    if (marker_count == marker_count_prev){
      continue;
    }

    try {
#if 0
      MarkerCount event(sequenceNum, (int)timeStamp, marker_id, marker_count, marker_count-marker_count_prev,
			mo,
			MarkerCount::getName() );
#endif
      MarkerCount event(mo,
			MarkerCount::getName(), sequenceNum, (int)timeStamp, marker_id, marker_count, marker_count-marker_count_prev
			 );
      signalMarkerCount(event);
    } catch (std::bad_weak_ptr &e) {}
    catch(std::exception const & ex){
      std::cout<<"COUNT EXT: "<< std::endl<<std::flush;
    }
  }
}

void ArProcess::generateTickEvents(sigc::signal<void, Tick> signalTick, std::shared_ptr<MediaObject> mo, unsigned long timeStamp){

  std::map<int, std::string>::iterator iter;  
  for(iter = ticks.begin(); iter != ticks.end(); iter++) {
    int timestamp = iter->first;
    std::string msg = iter->second;
    try {
      Tick event(mo,
		 Tick::getName(), msg, timestamp
		 );
      signalTick(event);
    }
    catch (std::bad_weak_ptr &e) {}
    catch(std::exception const & ex){
    std::cout<<"TICK EXT: "<< std::endl<<std::flush;
    }
  }
}


