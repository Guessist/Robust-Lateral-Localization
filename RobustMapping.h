#ifndef _ROBUSTMAPPING_H_
#define _ROBUSTMAPPING_H_


// User Defined
#include "Parameter.h"
#include "RANSAC_custom.h"
#include "Data.h"
#define LEAF 0.05
#define LIDAR_H -1.9
const double k0 = 0.9996;
const double drad = 0.01745329251994329576923690768489;

// WGS84 Model
const double eqRad = 6378137.0;
const double flat = 298.2572236;
const double EARTH_RADIUS_MAJOR = 6378137.0;
const double EARTH_RADIUS_MINOR = 6356752.3142;
using namespace std;



class RobustMapping {
private:
	bool CheckPoint;
	int START_FRAME;

	float HEIGHT_THRESHOLD;	
	double GRADIENT_THRESHOLD;
	float PRODUCT_THRESHOLD;
	float PRODUCT_THRESHOLD_;

	vector<double> Solutions[1000];
	int count;
	double FinalSolution_x;
	double FinalSolution_y;

	
	//vector<LATERAL_DATA> LateralDistances;
	GLOBAL_SOLUTION LateralSolution_G;
	LATERAL_DATA LateralSolution_L;
	pLine CurrentModel;
	string GlobalPath;
	string Genesis;	
	
	vector<POSE_DATA> GPS_TFM;				// GPS transformation matrix vector

	vector<ReferenceFrame> ReferenceMap;

	vector<pcl::PointCloud<pcl::PointXYZRGB>> CurbMap;
	pcl::PointCloud<pcl::PointXYZRGB> CurbMap_;
	Eigen::Matrix4f tempLastPose;
	Eigen::Matrix4f BackStepPose;	// Last transformation of each map building step
	Eigen::Matrix4f GlobalTrans;	// Global transformation matrix
	
	std::map<MY_POINT2D, MAP_DATA_> voxelGridMap_;

	double **GPS_Table;						// GPS pose table
	int DataSize;			// Total data size
	 double INIT_LAT;  // 위도의 초기값. 맵의 원점
	 double INIT_LON;	// 고도의 초기값. 맵의 원점
	 double INIT_ALT;	// 위도의 초기값. 맵의 원점

	ofstream POSE_LOG;		//  pose data logging
	ofstream MeasuredPOSE_LOG;		//  pose data logging
	ofstream CorrectedPOSE_LOG;		//  pose data logging
	ofstream MeasuredPOSE_LOG_forMATLAB;		//  pose data logging
	ofstream CorrectedPOSE_LOG_forMATLAB;		//  pose data logging
	ofstream GPS_DATA;		// Parsed GPS paose data logging
	ofstream PRODUCT_LOG;
	ofstream GRADIENT_LOG;
	ofstream DISTANCES_LOG;
	ofstream DISTANCES_LOG_G;
	
public:
	// 파라미터 조정
	RobustMapping() : HEIGHT_THRESHOLD(-1.65), START_FRAME(START_), GRADIENT_THRESHOLD(4), PRODUCT_THRESHOLD(0.2), PRODUCT_THRESHOLD_(0.4), CheckPoint(false)
	
	{
		cout << "Robust Mapping Stand-by.." << endl;
		Genesis = "D:\\SNU\\VI lab\\Reserch\\Velodynedata\\genesis.pcd";
		GlobalPath = GLOBAL_PATH;
		GRADIENT_LOG.open("../data/GRADIENT_LOG.txt", ios_base::out | ios_base::trunc);
		PRODUCT_LOG.open("../data/PRODUCT_LOG.txt", ios_base::out | ios_base::trunc);
		DISTANCES_LOG.open("../data/DISTANCES_LOG.txt", ios_base::out | ios_base::trunc);
		DISTANCES_LOG_G.open("../data/DISTANCES_LOG_G.txt", ios_base::out | ios_base::trunc);
		POSE_LOG.open("../data/POSE_LOG.txt", ios_base::out | ios_base::trunc);

		cout << "HEIGHT_THRESHOLD : " << this->HEIGHT_THRESHOLD << endl;
		cout << "GRADIENT_THRESHOLD : " << GRADIENT_THRESHOLD << endl;
		cout << "PRODUCT_THRESHOLD : " << this->PRODUCT_THRESHOLD << endl;
		cout << "PRODUCT_THRESHOLD_ : " << this->PRODUCT_THRESHOLD_ << endl;		
	}
	RobustMapping(double **table1, int size) :
		HEIGHT_THRESHOLD(-1.67), START_FRAME(START_), GRADIENT_THRESHOLD(2), PRODUCT_THRESHOLD(0.2), PRODUCT_THRESHOLD_(0.2)
		, GPS_Table(table1), DataSize(size), INIT_LAT(table1[1][START_]), INIT_LON(table1[2][START_]), INIT_ALT(table1[3][START_]), CheckPoint(false)
	{
		cout << "Robust Mapping Stand-by.." << endl;
		Genesis = "D:\\SNU\\VI lab\\Reserch\\Velodynedata\\genesis.pcd";
		GlobalPath = GLOBAL_PATH;
		GRADIENT_LOG.open("../data/GRADIENT_LOG.txt", ios_base::out | ios_base::trunc);
		PRODUCT_LOG.open("../data/PRODUCT_LOG.txt", ios_base::out | ios_base::trunc);
		DISTANCES_LOG.open("../data/DISTANCES_LOG.txt", ios_base::out | ios_base::trunc);
		DISTANCES_LOG_G.open("../data/DISTANCES_LOG_G.txt", ios_base::out | ios_base::trunc);
		
		POSE_LOG.open("../data/POSE_LOG.txt", ios_base::out | ios_base::trunc);
		MeasuredPOSE_LOG.open("../data/MeasuredPOSE_LOG.txt", ios_base::out | ios_base::trunc);
		CorrectedPOSE_LOG.open("../data/CorrectedPOSE_LOG.txt", ios_base::out | ios_base::trunc);
		MeasuredPOSE_LOG_forMATLAB.open("../data/MeasuredPOSE_LOG_forMATLAB.txt", ios_base::out | ios_base::trunc);
		CorrectedPOSE_LOG_forMATLAB.open("../data/CorrectedPOSE_LOG_forMATLAB.txt", ios_base::out | ios_base::trunc);

		cout << "HEIGHT_THRESHOLD : " << this->HEIGHT_THRESHOLD << endl;
		cout << "GRADIENT_THRESHOLD : " << GRADIENT_THRESHOLD << endl;
		cout << "PRODUCT_THRESHOLD : " << this->PRODUCT_THRESHOLD << endl;
		cout << "PRODUCT_THRESHOLD_ : " << this->PRODUCT_THRESHOLD_ << endl;
	}
	~RobustMapping()
	{
		GRADIENT_LOG.close();
		PRODUCT_LOG.close();
		DISTANCES_LOG.close();
		DISTANCES_LOG_G.close();

		POSE_LOG.close();
		MeasuredPOSE_LOG.close();
		MeasuredPOSE_LOG_forMATLAB.close();
		CorrectedPOSE_LOG.close();
		CorrectedPOSE_LOG_forMATLAB.close();
	
	}
	void LateralPositioning();
	// Local frame process
	void GetLateralDistances(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int);
	void GetLateralDistances(int NUM);
	void Smoothing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_int);
	void HeightCut(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
	void GradientEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
	void NormalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);	
	//void RansacFitting_Line(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
	void RansacFitting_Poly_L(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM);
	void FittingViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_L, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_R);

	// Global frame process
	bool GlobalLateralPositioning(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, int NUM);
	void NormalEstimation_G(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int);
	void RansacFitting_Poly_G(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM);  // 3차로 피팅
	void RansacFitting_Linear_G(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM);	// 1차로 피
	void Positioning(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, int);
	void calcGPS();
	void getTransformations();
	void getInverseTransformation(Eigen::Matrix4f matrix, double &x, double &y, double &z, double &roll, double &pitch, double &yaw);
	void ICPWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloudPtr, int loop, Eigen::Matrix4f &LocalMat);
	
	// General functions
	inline string PathSetting(string PATH, int num);
	inline int solve_cubic(float a, float b, float c, float d, float* xout);
	inline int solve_linear(float a, float b, float* xout);
	inline void SequentialScreenCapture(int NUM);

	
	// Sobel filter 
	void Filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
	void fillVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloudPtr, const double &leafSize, bool putPointCenter);
	void SovelFiltering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloudPtr, map<MY_POINT2D, MAP_DATA> & minvoxel, const double & leafSize, bool OriginScale);
	void MinFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloudPtr, map<MY_POINT2D, MAP_DATA> & minvoxel, const double & leafSize, bool OriginScale);
	void MaskSelection(int **MaskSobelX, int **MaskSobelY, map<MY_POINT2D, MAP_DATA> &minvoxel, int i, int j);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
		// --------------------------------------------
		// -----Open 3D viewer and add point cloud-----
		// --------------------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		//viewer->addCoordinateSystem (1.0, "global");
		viewer->initCameraParameters();
		return (viewer);
	}

	
	
};

class MyPointRepresentation : public pcl::PointRepresentation < pcl::PointXYZRGBNormal >
{
	using pcl::PointRepresentation<pcl::PointXYZRGBNormal>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 5;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const pcl::PointXYZRGBNormal &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.r;
		out[4] = p.curvature;
	}
};
#endif _ROBUSTMAPPING_H_