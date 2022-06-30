
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "RVLCore2.h"
#ifndef RVLLINUX
#include <Windows.h>
#endif
#include <ctime>
#include <fstream>
#include <chrono>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include "RVLVTK.h"
#include <vtkTriangle.h>		
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "MSTree.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognitionCommon.h"
#include "RVLRecognition.h"
#include "RFRecognition.h"
#include "RVLMeshNoiser.h"
#ifdef RVLRECOGNITION_DEMO_RVLTG
#include "TGraph.h"
#endif
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include <pcl/common/common.h>
#include <pcl/registration/registration.h>
#include <pcl/PolygonMesh.h>
#ifdef RVLOPENNI
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#endif
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
#include "NeighborhoodTool.h"
#include "PCLNeighborhoodTool.h"
#include "MarchingCubes.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#include "DDDetector.h"
#include "Gripper.h"
#ifdef RVL_ASTRA
#include "astraUtils.h"
#endif
#include "RVLAstra.h"
#include "ObjectDetector.h"
#ifdef RVLPSGM_CUDA_ICP
#include "ICPCUDAv1.h"
#endif
bool bVerbose = false;
#define RVL_FEATURE_TEST_SCENE_SEQUENCE
#define RVL_LOAD_SINGLE_MODEL
#define PSGM_MATCHES_SCORE_COMPARE
#define RVLPSGM_TRANSPARENCY_AND_COLLISION
#define RVLPSGM_RMSE_CALCULATION
#ifndef RVLVERSION_171125
#define RVLRECOGNITION_DEMO_CLASS_ALIGNMENT
#endif
#define RVLRECOGNITION_DEMO_FLAG_SAVE_PLY				0x00000001
#define RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION		0x00000002
#define RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_MODEL		0x00000004
#define RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_INSTANCE	0x00000008
#define RVLCAMERA_NONE		0
#define RVLCAMERA_ASTRA		1
#define RVLCAMERA_KINECT	2
#ifdef RVLPSGM_CUDA_ICP
ICPcudaV1 *pCUDAICPObjv1;
#endif

using namespace RVL;
//Detector 

RECOG::DDD::Hypothesis dddetector(){
  CRVLMem mem0;

  mem0.Create(1000000000);
	//mem0.Create(100000000000); //VIDOVIC

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);


  printf("Create mesh builder.\n");

	PCLMeshBuilder meshBuilder;

  std::string cfgFileName = "/home/andrej/RVL/RVLRecognitionDemo_Andrej_DDD_Cuboid_Detection.cfg";

	meshBuilder.Create("/home/andrej/RVL/RVLRecognitionDemo_Andrej_DDD_Cuboid_Detection.cfg", &mem0);


	int w = 640;
	int h = 480;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));


	meshBuilder.PC = PC;

  meshBuilder.camera.depthFu = meshBuilder.camera.depthFu0;
	meshBuilder.camera.depthUc = meshBuilder.camera.depthUc0;
	meshBuilder.camera.depthFv = meshBuilder.camera.depthFv0;
	meshBuilder.camera.depthVc = meshBuilder.camera.depthVc0;


  // Initialize surfel detection

  SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams((char *)cfgFileName.data());

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams((char *)cfgFileName.data());

  
  // Initialize visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();


  //Get image from file
  Mesh mesh; 

  std::string RGBFileName = "/home/andrej/RVL/02_RGB.png";
  std::string PLYFileName = "/home/andrej/RVL/02_PLY.ply";
  std::string depthFileName = "/home/andrej/RVL/02_D.png";

  LoadMesh(&meshBuilder, (char *)RGBFileName.data(), &mesh, true, (char *)PLYFileName.data(), (char *)depthFileName.data());


  //Create DDDetector
  DDDetector detector;
  detector.pMem0 = &mem0;
  detector.pMem = &mem;
  detector.Create((char *)cfgFileName.data());
  detector.SetCameraParams(meshBuilder.camera.depthFu, meshBuilder.camera.depthFv, meshBuilder.camera.depthUc, meshBuilder.camera.depthVc, meshBuilder.camera.w, meshBuilder.camera.h);
  detector.pSurfels = &surfels;
  detector.pSurfelDetector = &surfelDetector;
  detector.InitVisualizer(&visualizer);

  // Detect cuboids.

    float cuboidSize[] = { 1.0f, 1.0f, 1.0f };
    int iModel;
    for (iModel = 0; iModel < detector.models.n; iModel++)
      RVL_DELETE_ARRAY(detector.models.Element[iModel].points.Element);
    RVL_DELETE_ARRAY(detector.models.Element);
    detector.models.Element = new RECOG::DDD::Model[1];
    detector.models.n = 1;
    detector.CreateCuboidModel(cuboidSize, 0.1f, detector.models.Element);
    // LoadMesh(&meshBuilder, "/home/andrej/RVL/Plys/mesh.ply", pMesh, false);
    RECOG::DDD::Hypothesis hyp=detector.DetectCuboids(&mesh);
    printf("Size: %f %f %f\n", hyp.pose.s[0], hyp.pose.s[1], hyp.pose.s[2]);
    printf("Matrix R: \n");
    for(int i=0;i<3;i++){
      for(int j=i;j<i+3;j++)
        printf("%f ", hyp.pose.R[j]);
      printf("\n");
    }
    printf("Matrix t: \n");
    for(int i=0;i<3;i++)
      printf("%f \n", hyp.pose.t[i]);
    printf("\n");
    return hyp;
}

//Image subscriber

void imageCallback(const sensor_msgs::ImageConstPtr& msg_RGB, const sensor_msgs::ImageConstPtr& msg_D)
{
  ROS_INFO("Callback successful");
  cv::imwrite("/home/andrej/RVL/02_RGB.png", cv_bridge::toCvShare(msg_RGB, "bgr8")->image);
  cv::imwrite("/home/andrej/RVL/02_D.png", cv_bridge::toCvShare(msg_D, "16UC1")->image);
}

using namespace message_filters;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  //Subscribe to 2 topics at once and synchronize them

  message_filters::Subscriber<sensor_msgs::Image> image_RGB(nh, "/camera/color/image_raw",  1);
  message_filters::Subscriber<sensor_msgs::Image> image_D(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> img_sync(MySyncPolicy(10), image_RGB, image_D);
  img_sync.registerCallback(boost::bind(&imageCallback, _1, _2));
  cv::destroyWindow("view");

  //Init 2 publishers for R and t matrix

  ros::Publisher publisher_R = nh.advertise<std_msgs::Float64MultiArray>("matrix_R", 0);
  ros::Publisher publisher_t = nh.advertise<std_msgs::Float64MultiArray>("matrix_t", 0);
  ros::Publisher publisher_s = nh.advertise<std_msgs::Float64MultiArray>("size", 0);

  std_msgs::Float64MultiArray array_msg_R;
  array_msg_R.data.resize(9);

  std_msgs::Float64MultiArray array_msg_t;
  array_msg_t.data.resize(3);

  std_msgs::Float64MultiArray array_msg_s;
  array_msg_s.data.resize(3);

  while(ros::ok()){
    std::cout << "Press enter to capture image" << std::endl;
    std::cin.get();
    ros::spinOnce();
    sleep(2);
    cv::Mat I = cv::imread("/home/andrej/RVL/02_RGB.png", 0);
    while(I.empty()){
      std::cout << "Failed imread(): image not found" << std::endl;
      sleep(2);
      ros::spinOnce();
      I = cv::imread("/home/andrej/RVL/02_RGB.png", 0);
    }
    std::cout << "Image found! Processing image..." << std::endl;
    RECOG::DDD::Hypothesis hyp = dddetector();
    int flag=0;
    for(int i=0;i<3;i++){
      array_msg_s.data[i]=hyp.pose.s[i];
      if(hyp.pose.s[i]<0.0001)
        flag++;
    }
    for(int i=0;i<9;i++){
      array_msg_R.data[i]=hyp.pose.R[i];
    }
    for(int i=0;i<3;i++){
      array_msg_t.data[i]=hyp.pose.t[i];
    }
    if(flag==0){
    printf("Waiting for subsribers\n");
    while(true)
    {
      if(publisher_R.getNumSubscribers()>0){
        printf("Got one subscriber on matrix R\n");
        publisher_R.publish(array_msg_R);
        break;
      }
    }
    while(true)
    {
      if(publisher_t.getNumSubscribers()>0){
        printf("Got one subscriber on matrix t\n");
        publisher_t.publish(array_msg_t);
        break;
      }
    }
    while(true)
    {
      if(publisher_s.getNumSubscribers()>0){
        printf("Got one subscriber on size\n");
        publisher_s.publish(array_msg_s);
        break;
      }
    }
    }
    else{
      printf("No hypothesis generated, not publishing to topics..");
    }
    printf("\n");
  }

  return 0;
  
}
