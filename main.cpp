/******************************************************************************\
 @description This program is intended to be used for 3D Sketching using input from
 leap motion.
 \******************************************************************************/

#include <iostream>
#include <cstring>
#include <thread>

#include "Leap.h"
#include "ref.cpp"
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/gp3.h>

using namespace Leap;
using namespace pcl;
using namespace std;

boost::mutex updateModelMutex;
volatile bool update;

PointCloud<PointXYZ>::Ptr global_cloud_ptr(new PointCloud<PointXYZ>);

PointCloud<PointXYZ>::Ptr hand(new PointCloud<PointXYZ>);

boost::shared_ptr<visualization::PCLVisualizer> viewer(
		new visualization::PCLVisualizer("3D Viewer"));

const Controller *controller_global;

const string fingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const string boneNames[] = { "Metacarpal", "Proximal", "Middle", "Distal" };
const string stateNames[] = { "STATE_INVALID", "STATE_START", "STATE_UPDATE",
		"STATE_END" };

int flag = 0;
PointXYZ pt[5][4];
PointXYZ pt_prev[5][4];
Eigen::Affine3f transform = Eigen::Affine3f::Identity();

vector<PolygonMesh> meshes;

PointXYZ bmin;
PointXYZ bmax;
int flag_cube = 0;

class SampleListener: public Listener {
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

void SampleListener::onInit(const Controller& controller) {
	cout << "Initialized" << endl;
	controller_global = &controller;
}

void SampleListener::onConnect(const Controller& controller) {
	cout << "Connected" << endl;
	controller.enableGesture(Gesture::TYPE_CIRCLE);
	controller.enableGesture(Gesture::TYPE_KEY_TAP);
	controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
	controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
	cout << "Disconnected" << endl;
}

void SampleListener::onExit(const Controller& controller) {
	cout << "Exiting.." << endl;

	//Combine all meshes and display it
//	for(int i; i < meshes.size(); i++){
//		viewer->addPolygonMesh(meshes[i], "mesh_" + to_string(i), 0);
//	}

	exit(0);
}

/**
 * onFrame: called every frame.
 */
void SampleListener::onFrame(const Controller& controller) {

	if (flag == 0)
		return;
	boost::mutex::scoped_lock updateLock(updateModelMutex);
	update = true;

	const Frame frame = controller.frame();

	InteractionBox box = frame.interactionBox();

	bmin.x = box.center().x - box.width();
	bmin.y = box.center().y - box.height();
	bmin.z = box.center().z - box.depth();

	bmax.x = box.center().x + box.width();
	bmax.y = box.center().y + box.height();
	bmax.z = box.center().z + box.depth();

	HandList hands = frame.hands();
	for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
		const Hand hand = *hl;
		PointXYZ basic_point;
		basic_point.x = hand.palmPosition().x;
		basic_point.y = hand.palmPosition().y;
		basic_point.z = hand.palmPosition().z;

		global_cloud_ptr->points.push_back(basic_point);

		const FingerList fingers = hand.fingers();
		if (hand.isRight()) {
			for (FingerList::const_iterator fl = fingers.begin();
					fl != fingers.end(); ++fl) {

				const Finger finger = *fl;

				PointXYZ f;
				f.x = finger.tipPosition().x;
				f.y = finger.tipPosition().y;
				f.z = finger.tipPosition().z;

				global_cloud_ptr->points.push_back(f);

				string name;
				for (int b = 0; b < 4; ++b) {

					Bone::Type boneType = static_cast<Bone::Type>(b);
					Bone bone = finger.bone(boneType);
					name = fingerNames[finger.type()] + "_" + boneNames[b];

					PointXYZ next;
					next.x = bone.nextJoint().x;
					next.y = bone.nextJoint().y;
					next.z = bone.nextJoint().z;

					PointXYZ prev;
					prev.x = bone.prevJoint().x;
					prev.y = bone.prevJoint().y;
					prev.z = bone.prevJoint().z;

					pt[finger.type()][b] = next;
					pt_prev[finger.type()][b] = prev;

				}
			}
		}
	}
	updateLock.unlock();
}

void SampleListener::onFocusGained(const Controller& controller) {
	cout << "Focus Gained" << endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
	cout << "Focus Lost" << endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
	cout << "Device Changed" << endl;
	const DeviceList devices = controller.devices();

	for (int i = 0; i < devices.count(); ++i) {
		cout << "id: " << devices[i].toString() << endl;
		cout << "  isStreaming: "
				<< (devices[i].isStreaming() ? "true" : "false") << endl;
	}
}

void SampleListener::onServiceConnect(const Controller& controller) {
	cout << "Service Connected" << endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
	cout << "Service Disconnected" << endl;
}

void leap() {
	SampleListener listener;
	Controller controller;
	controller.addListener(listener);
	cout << "Press Enter to quit..." << endl;
	cin.get();
	controller.removeListener(listener);
}

PolygonMesh reconstructGP3() {

	PolygonMesh triangles;

//	local_cloud_ptr->width = (int) local_cloud_ptr->points.size();
//	local_cloud_ptr->height = 1;

	if ((int) global_cloud_ptr->points.size() < 50) {
		return triangles;
	}

	// Create a KD-Tree
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	PointCloud<PointXYZ>::Ptr mls_smoothed(new PointCloud<PointXYZ>());

	MovingLeastSquares<PointXYZ, PointXYZ> mls;

	mls.setInputCloud(global_cloud_ptr);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	mls.process(*mls_smoothed);

	// Normal estimation*
	NormalEstimationOMP<PointXYZ, Normal> n;
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	tree->setInputCloud(mls_smoothed);
	n.setInputCloud(mls_smoothed);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	PointCloud<PointNormal>::Ptr cloud_with_normals(
			new PointCloud<PointNormal>);
	concatenateFields(*mls_smoothed, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	GreedyProjectionTriangulation<PointNormal> gp3;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	return triangles;
}

PolygonMesh poission_passthrough() {

//	local_cloud_ptr->width = (int) local_cloud_ptr->points.size();
//	local_cloud_ptr->height = 1;

	PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
	PassThrough<PointXYZ> filter;
	filter.setInputCloud(global_cloud_ptr);
	filter.filter(*filtered);

	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads(8);
	ne.setInputCloud(filtered);
	ne.setRadiusSearch(0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid(*filtered, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
	ne.compute(*cloud_normals);

	for (size_t i = 0; i < cloud_normals->size(); ++i) {
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	PointCloud<PointNormal>::Ptr cloud_smoothed_normals(
			new PointCloud<PointNormal>());
	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

	PolygonMesh mesh;

	Poisson<PointNormal> poisson;
	poisson.setDepth(3);
	poisson.setInputCloud(cloud_smoothed_normals);
	poisson.reconstruct(mesh);

	return mesh;
}

PolygonMesh reconstruct() {

//	local_cloud_ptr->width = (int) local_cloud_ptr->points.size();
//	local_cloud_ptr->height = 1;

	// Create a KD-Tree
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	PointCloud<PointXYZ>::Ptr mls_smoothed(new PointCloud<PointXYZ>());

	MovingLeastSquares<PointXYZ, PointXYZ> mls;

	mls.setInputCloud(global_cloud_ptr);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	mls.process(*mls_smoothed);

	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads(8);
	ne.setInputCloud(mls_smoothed);
	ne.setRadiusSearch(100);

	Eigen::Vector4f centroid;
	compute3DCentroid(*mls_smoothed, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
	ne.compute(*cloud_normals);

	for (size_t i = 0; i < cloud_normals->size(); ++i) {
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	PointCloud<PointNormal>::Ptr cloud_smoothed_normals(
			new PointCloud<PointNormal>());
	concatenateFields(*mls_smoothed, *cloud_normals, *cloud_smoothed_normals);

	PolygonMesh mesh;

	Poisson<PointNormal> poisson;
	poisson.setDepth(3);
	poisson.setInputCloud(cloud_smoothed_normals);

	poisson.reconstruct(mesh);

	return mesh;
}

void print_cam(boost::shared_ptr<visualization::PCLVisualizer> viewer) {
	vector<visualization::Camera> cam;
	viewer->getCameras(cam);
	cout << "Cam: " << endl << " - pos: (" << cam[0].pos[0] << ", "
			<< cam[0].pos[1] << ", " << cam[0].pos[2] << ")" << endl
			<< " - view: (" << cam[0].view[0] << ", " << cam[0].view[1] << ", "
			<< cam[0].view[2] << ")" << endl << " - focal: (" << cam[0].focal[0]
			<< ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")"
			<< endl;
}

void add_shapes() {
	PointXYZ init;
	init.x = 0;
	init.y = 0;
	init.z = 0;

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 4; j++) {
			cout << fingerNames[i] + "_" + boneNames[j] << endl;
			viewer->addSphere(init, 1.0,
					string(fingerNames[i] + "_" + boneNames[j]));
			viewer->addLine(init, init,
					string(fingerNames[i] + "_" + boneNames[j]) + "l");
			pt[i][j].x = 0;
			pt[i][j].y = 0;
			pt[i][j].z = 0;
		}
	}

	flag = 1;
}

void add_interaction_box() {
	viewer->addCube(bmin.x, bmax.x, bmin.y, bmax.y, bmin.z, bmax.z, 0, 0,
			256 /*R,G,B*/, "cube");
	viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR,
			0.9, 0.1, 0.1 /*R,G,B*/, "cube", 0);
	viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_OPACITY,
			1, "cube", 0);
	viewer->setShapeRenderingProperties(
			visualization::PCL_VISUALIZER_REPRESENTATION,
			visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube", 0);
}

int main(int argc, char** argv) {
	thread first(leap);

	cout << "Visualizer Setup\n";
	add_shapes();
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem();
	viewer->initCameraParameters();
	viewer->setCameraPosition(189.194, 1016.86, -166.963, 0.701186, -0.239757,
			-0.671457, 0.738823, -0.429874, -0.518987);
	viewer->addPointCloud<PointXYZ>(global_cloud_ptr, "cloud");
	viewer->setPointCloudRenderingProperties(
			visualization::PCL_VISUALIZER_POINT_SIZE, 3); //default point cloud id=cloud

	add_interaction_box();

	cout << "============== Adding Objects complete ==============\n";

	int meshNo = 0;
	while (!viewer->wasStopped()) {
		viewer->spinOnce(10); //loop allowed to run for 10ms (frame rate)

		boost::mutex::scoped_lock updateLock(updateModelMutex); //init lock
		if (update) {
			if (!viewer->updatePointCloud(global_cloud_ptr)) {
				viewer->addPointCloud(global_cloud_ptr);
			}

			meshNo++;

			PolygonMesh mesh = reconstruct();
			meshes.push_back(mesh);
//			viewer->removeShape("mesh");
			viewer->addPolygonMesh(mesh, "mesh_" + meshNo, 0);

			global_cloud_ptr->points.clear();

			for (int i = 0; i < 5; i++) {
				for (int j = 0; j < 4; j++) {
					viewer->removeShape(
							string(fingerNames[i] + "_" + boneNames[j]));
					viewer->addSphere(pt[i][j], 3.0, 0, 0, 255,
							string(fingerNames[i] + "_" + boneNames[j])); //255, 255, 255
					viewer->removeShape(
							string(fingerNames[i] + "_" + boneNames[j] + "l"));
					viewer->addLine(pt_prev[i][j], pt[i][j], 255, 255, 0,
							string(fingerNames[i] + "_" + boneNames[j]) + "l"); //251, 223, 65
				}
			}
			update = false;
		}
		updateLock.unlock();
	}

	first.join();

	return 0;
}
