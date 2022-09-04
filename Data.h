#ifndef _DATA_H_
#define _DATA_H_
#include <pcl/io/pcd_io.h>
#include <conio.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <sstream>
#include <iostream>
#include <fstream>


// RANSAC
#include <pcl/console/parse.h>
#include <pcl\console\time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <Eigen\src\Core\EigenBase.h>

// Projection
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
// Smoothinh
#include <pcl/surface/mls.h>

// ICP
#include <pcl/registration/icp_nl.h>

typedef pcl::PointXYZRGB PointT;

struct POSE_DATA {
	unsigned int frame;
	Eigen::Matrix4f mat;
	POSE_DATA() {
		mat.setIdentity();
	}
};

struct LATERAL_DATA {
	double distance_L;
	double distance_R;
	int frame;
};

struct GLOBAL_SOLUTION {
	double x_L;
	double y_L;
	double x_R;
	double y_R;
	double distance_L;
	double distance_R;
	int frame;
};
struct ReferenceFrame {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	Eigen::Matrix4f Transformation;
};


// Voxel
struct MAP_DATA{
	unsigned int times = 0;
	unsigned int scanCount = 0;
	double intensity = 0;
	long unsigned int pointIndex = 0;
	double z;
};
struct MY_POINT2D{
	int x;
	int y;
	//double z;
};
struct MAP_DATA_{
	unsigned int times = 0;
	unsigned int scanCount = 0;
	double intensity = 0;
	long unsigned int pointIndex = 0;
	vector<double> z_set;
};
bool operator<(const MY_POINT2D c1, const MY_POINT2D c2);
#endif