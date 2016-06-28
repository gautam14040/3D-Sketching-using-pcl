/*
 * main.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: naman
 */

#include <boost/make_shared.hpp>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>

using namespace Eigen;
using namespace std;

bool convergence_reached() {
	return false;
}


double phi(double param) {
	double value;

	///

	return value;
}

double dphi(double param) {
	double value;

	///

	return value;
}

int main(int argc, char **argv) {

	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>());

	 // Create an empty kdtree representation, and pass it to the normal estimation object.
	 // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	 new pcl::search::KdTree<pcl::PointXYZ>());

	// Load bun0.pcd -- should be available with the PCL archive in test
	pcl::io::loadPCDFile("bun0.pcd", *cloud);

	 // Create the normal estimation class, and pass the input dataset to it
	 pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	 ne.setInputCloud(cloud);
	 ne.setSearchMethod(tree);

	 // Output datasets
	 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
	 new pcl::PointCloud<pcl::Normal>);

	 // Use all neighbors in a sphere of radius 3cm
	 ne.setRadiusSearch(0.03);
	 // Compute the features
	 ne.compute(*cloud_normals);

	int max_iterations = 3;

	double  alpha, //alpha is the starting point
			fx; //implicit function

	double  f = 0,
			threshold, //threshold on range of point X
			sigma_r,
			sigma_n = 0.75,
			w,
			gradient_w;

	pcl::PointXYZ x; //point of interest
	pcl::PointXYZ p; //neighbor point

	Eigen::Vector3f p_normal; //normal of neighbor point
	Eigen::Vector3f px; //p->x vector from p to X.

	Eigen::Vector3f gradient_f; //gradient of fx

	while (f * gradient_f.squaredNorm() < threshold) {

		x = cloud->points[0]; //initial point

		for (size_t j = 0; j < cloud->points.size(); ++j) { //for all points x belonging to X

			//	x = cloud->points[j];

			for (int i = 0; i < max_iterations || convergence_reached(); ++i) {

				int sumW = 0, sumGw = 0, sumF = 0, sumGF = 0, sumN = 0;

					// K nearest neighbor search
					int K = 10;
					std::vector<int> pointIdxNKNSearch(K);
					std::vector<float> pointNKNSquaredDistance(K);

					if (tree->nearestKSearch(x, K, pointIdxNKNSearch,
							pointNKNSquaredDistance) > 0) {
						for (size_t k = 0; k < pointIdxNKNSearch.size(); ++k) {

							p = cloud->points[pointIdxNKNSearch[k]];
							p_normal = cloud_normals->points[pointIdxNKNSearch[k]].getNormalVector3fMap();

							px = x.getVector3fMap() - p.getVector3fMap();
							fx = px.dot(p_normal);

							if (i > 0) {
								alpha = exp(-1 * pow((fx - f) / sigma_r, 2))
										* exp(-1 * (p_normal - gradient_f).squaredNorm() / pow(sigma_n,2));
							} else {
								alpha = 1;
							}

							w = alpha * phi(pow(px.norm(), 2));
							gradient_w = alpha * 2 * px.norm() * dphi(pow(px.norm(), 2));

							sumW += w;
							sumGw += gradient_w;
							sumF += w * fx;
							sumGF += gradient_w * fx;
							sumN += w * p_normal.norm();
						}
				}
				f = sumF / sumW;
				gradient_f.x() = (sumGF - f * sumGw + sumN) / sumW; //gradient_f is Vector3, but RHS is double.
				gradient_f.y() = (sumGF - f * sumGw + sumN) / sumW; //gradient_f is Vector3, but RHS is double.
				gradient_f.z() = (sumGF - f * sumGw + sumN) / sumW; //gradient_f is Vector3, but RHS is double.

			}
			x.x -= f * gradient_f.x();
			x.y -= f * gradient_f.y();
			x.z -= f * gradient_f.z();
		}
	}

	 pcl::io::savePCDFile("bun0-rimls.pcd", *cloud);

	//END
}
