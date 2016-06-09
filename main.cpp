/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <iostream>
#include <cstring>
#include "Leap.h"
#include "ref.cpp"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <thread>         // std::thread


using namespace Leap;
using namespace pcl ;

boost::mutex updateModelMutex;

void reconstruct();
volatile bool update ;

class SampleListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);

  private:
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr hand (new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  reconstruct();
}
pcl::PointXYZ basic_point;
pcl::PointXYZ f;
void SampleListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    update = true;

    const Frame frame = controller.frame();
    HandList hands = frame.hands();
    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        const Hand hand = *hl;

        basic_point.x = hand.palmPosition().x;
        basic_point.y = hand.palmPosition().y;
        basic_point.z = hand.palmPosition().z;
        basic_cloud_ptr->points.push_back(basic_point);

        // Get fingers
        const FingerList fingers = hand.fingers();
        for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {

            const Finger finger = *fl;
            f.x = finger.tipPosition().x;
            f.y = finger.tipPosition().y;
            f.z = finger.tipPosition().z;
            basic_point.x = finger.tipPosition().x;
            basic_point.y = finger.tipPosition().y;
            basic_point.z = finger.tipPosition().z;
            basic_cloud_ptr->points.push_back(basic_point);
      }
    }
    updateLock.unlock();
  }

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void SampleListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

void leap()
{
   // Create a sample listener and controller
  SampleListener listener;
  Controller controller;

  // Have the sample listener receive events from the controller
  controller.addListener(listener);


  // Keep this process running until Enter is pressed
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  // Remove the sample listener when done
  controller.removeListener(listener);
}

int main(int argc, char** argv) {
  std::thread first(leap) ;
  viewer = simpleVis(basic_cloud_ptr);
    viewer->setCameraPosition(189.194,1016.86, -166.963 , 0.701186 ,-0.239757, -0.671457 , 0.738823 , -0.429874 , -0.518987);
    /*
    while (!viewer->wasStopped ())
    {
        //viewer->updatePointCloud(basic_cloud_ptr);
        //viewer->removeAllPointClouds();
        //viewer->addPointCloud(basic_cloud_ptr);
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (10);
        std::vector<pcl::visualization::Camera> cam;

        //Save the position of the camera
        viewer->getCameras(cam);

        //Print recorded points on the screen:
        cout << "Cam: " << endl
        << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
        << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
        << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
        // Get lock on the boolean update and check if cloud was updated
        boost::mutex::scoped_lock updateLock(updateModelMutex);
        if(update)
        {
           // viewer->removeAllShapes();
          //  viewer->addLine(basic_point,f,"line",0);
            if(!viewer->updatePointCloud(basic_cloud_ptr))
                viewer->addPointCloud(basic_cloud_ptr);
            update = false;
        }
        updateLock.unlock();
    }
    first.join();

  return 0;
}

void reconstruct()
{
  cout << "begin passthrough filter" << endl;
  PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
  PassThrough<PointXYZ> filter;
  filter.setInputCloud(basic_cloud_ptr);
  filter.filter(*filtered);
  cout << "passthrough filter complete" << endl;
  cout << "begin normal estimation" << endl;
  NormalEstimationOMP<PointXYZ, Normal> ne;
  ne.setNumberOfThreads(8);
  ne.setInputCloud(filtered);
  ne.setRadiusSearch(100);
  Eigen::Vector4f centroid;
  compute3DCentroid(*filtered, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);


  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
  ne.compute(*cloud_normals);
  cout << "normal estimation complete" << endl;

    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

  cout << "combine points and normals" << endl;
  PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
  concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);  

  cout << "begin poisson reconstruction" << endl;
  Poisson<PointNormal> poisson;
  poisson.setDepth(3);
  poisson.setInputCloud(cloud_smoothed_normals);
  PolygonMesh mesh;
  poisson.reconstruct(mesh);

  io::savePLYFile("/Users/gautamgupta/Desktop/project/interactive2/mesh.ply", mesh);
}