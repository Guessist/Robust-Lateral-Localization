#ifndef _IVN_PARSING_H_
#define _IVN_PARSING_H_
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl\point_types.h>
#include "ParameterDefine.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <math.h>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> CloudRGB;

class IVN_Parsing {
public:
	double **TotalTable;
	std::string GLOBAL_PATH_POSE = GLOBAL_PATH;
	std::string INPUT_IVN_FILE = "IVN_Data.txt";
	std::string OUTPUT_POSE_DATA = "EncoderPose.txt";
	vector<string> Lable;
	double **EncoderPoseData;
	//double **PoseData;

private:
	int Column;
	int Row;
	

public:
	IVN_Parsing() {

	}
	IVN_Parsing(int column, int row) :Column(column), Row(row)  {
		std::cout << "IVN Data Parsing Processing..." << std::endl;
		cout << endl;
		EncoderPoseData = new double*[column];
		for (int i = 0; i < column; i++){
			EncoderPoseData[i] = new double[row];
			for (int j = 0; j < row; j++) EncoderPoseData[i][j] = { 0, };
		}

	}

	~IVN_Parsing() {
		cout << "멤버변수 메모리해제 " << endl;
		for (int i = 0; i < Column; i++) delete[] EncoderPoseData[i];
		delete[] EncoderPoseData;
	};



	void GetEncoderPose_Table(double **table);		// Encoder data parsing
	void DataFromEncoder(const int Datasize);		// Encoder pose table generate
	// Pose estimation 
	void getPose_from_encoder(int RL, int RR, double pre_pose_x, double pre_pose_y, double pre_pose_heading, double* cur_pose_x, double* cur_pose_y, double* cur_pose_heading, double* heading_rate);
	


};

#endif _IVN_PARSING_H_