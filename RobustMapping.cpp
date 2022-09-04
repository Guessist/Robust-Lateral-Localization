#include"RobustMapping.h"
#include "ScreenCpature.h"
bool operator<(const MY_POINT2D c1, const MY_POINT2D c2){
	if (c1.x < c2.x) return true;
	else if (c1.x > c2.x) return false;
	if (c1.y < c2.y) return true;
	else if (c1.y > c2.y) return false;
	//if (c1.z < c2.z) return true;
	return false;
}

void
RobustMapping::LateralPositioning() {
	//pcl::visualization::CloudViewer Viewer("THIS");
	int startnum = START_;
	int startnum_ = START_ + 1 * STEP;
	int endnum = END;
	this->count = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in__(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::visualization::CloudViewer Viewer("Lateral Positioning LAST");
	for (int i = startnum; i < startnum_; i = i + STEP){
		string Path = RobustMapping::PathSetting(this->GlobalPath, i);
		pcl::io::loadPCDFile(Path, *cloud_in_);
		for each(auto j in *cloud_in_) {
			pcl::PointXYZRGB tempPo;
			tempPo.x = j.x;
			tempPo.y = j.y;
			tempPo.z = j.z;
			tempPo.r = 255;
			tempPo.g = 255;
			tempPo.b = 255;
			cloud_in__->push_back(tempPo);
		}

		ReferenceFrame tempRF;
		for each(auto i in *cloud_in__) tempRF.cloud.push_back(i);
		tempRF.Transformation = this->GPS_TFM[i].mat;
		this->ReferenceMap.push_back(tempRF);
		int tempNUM = i + STEP;

		if (tempNUM == START_ + 3 * STEP) this->CheckPoint = true;
		cout << "YES" << endl;
		cloud_in__->clear();
	}

	this->BackStepPose = this->GPS_TFM[startnum_ - STEP_].mat;
	for (int i = startnum_; i < endnum; i = i + STEP_) {
		pcl::console::TicToc time;
		time.tic();
		cloud_in->clear();
		cout << "끼아아아아" << endl;
		RobustMapping::GetLateralDistances(cloud_in, i);	// 로컬프레임 처리모듈		
		RobustMapping::GlobalLateralPositioning(cloud_in, i); 	Viewer.showCloud(cloud_in);

		RobustMapping::SequentialScreenCapture(i);

		cout << "Map Building 진행 중...(시작!)  " << i << "/" << endnum << "\t Computation time = " << time.toc() / 1000 << "초" << endl;
	}
}
void 
RobustMapping::Filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
	map<MY_POINT2D, MAP_DATA> MinVoxel;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Sobelfiltered(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel_(new pcl::PointCloud <pcl::PointXYZRGB>);
	//fillVoxelGrid(cloud_in_, cloud_voxel, 1.5, true, 2, 1, false);
	RobustMapping::fillVoxelGrid(cloud_in, cloud_voxel_, LEAF, true);// cout << " Fill Voxel Complete" << endl;
	RobustMapping::MinFilter(cloud_filtered, MinVoxel, LEAF, true); //cout << " MinFilter Complete" << endl;
	RobustMapping::SovelFiltering(cloud_Sobelfiltered, MinVoxel, LEAF, true); cout << "Sobel Filter Complete" << endl;
	cloud_in->clear();
	pcl::copyPointCloud(*cloud_Sobelfiltered, *cloud_in);
}
bool
RobustMapping::GlobalLateralPositioning(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {
	cloud_in->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subMap(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subMap2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in__(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::visualization::CloudViewer reer("VEIW");

	string Path = RobustMapping::PathSetting(this->GlobalPath, NUM);
	pcl::io::loadPCDFile(Path, *cloud_in_);
	for each(auto i in *cloud_in_) {
		pcl::PointXYZRGB tempPo;
		tempPo.x = i.x;
		tempPo.y = i.y;
		tempPo.z = i.z;
		tempPo.r = 255;
		tempPo.g = 255;
		tempPo.b = 255;
		cloud_in__->push_back(tempPo);
	}
	cloud_in_->clear();

	if (this->ReferenceMap.size() > 2) {
		//while (this->ReferenceMap.size() == 2) {
			this->ReferenceMap.erase(this->ReferenceMap.begin());
		//}
		
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < this->ReferenceMap.size(); i++) {
	//	cout << "ReferenceMap[i].cloud.size() = " << ReferenceMap[i].cloud.size() << endl;
		pcl::copyPointCloud(ReferenceMap[i].cloud, *cloud_subMap2);
		pcl::transformPointCloud(*cloud_subMap2, *cloud_out, ReferenceMap[i].Transformation);
		for each(auto it in *cloud_out) cloud_in->push_back(it);
		RobustMapping::HeightCut(cloud_subMap2);
		RobustMapping::GradientEstimation(cloud_subMap2);
		
		
		RobustMapping::NormalEstimation(cloud_subMap2); // reer.showCloud(cloud_subMap2); _getch();
		//RobustMapping::Filtering(cloud_subMap2);  //reer.showCloud(cloud_subMap2); _getch();
		for each(auto it in *cloud_subMap2) {
			double r = sqrt(pow(it.x, 2) + pow(it.y, 2));
			if (r > 2) 	cloud_temp->push_back(it);
		}
		
	//	cout << "cloud_subMap2.size() = " << cloud_subMap2->size() << endl;		
		pcl::transformPointCloud(*cloud_temp, *cloud_ref, ReferenceMap[i].Transformation);

		for each(auto j in *cloud_ref) {
			cloud_subMap->push_back(j);
		}
		cloud_subMap2->clear();
		cloud_temp->clear();
		cloud_ref->clear();
		cloud_out->clear();
	}
	//reer.showCloud(cloud_subMap); _getch();
	//RobustMapping::NormalEstimation(cloud_subMap2); // reer.showCloud(cloud_subMap2); _getch();
	//	RobustMapping::GradientEstimation(cloud_subMap); // reer.showCloud(cloud_subMap); _getch();	
	//	RobustMapping::NormalEstimation_G(cloud_subMap, NUM);  

	if (this->CurbMap_.size() != 0){
	//	for each(auto points in CurbMap_) cloud_subMap->push_back(points);
	}
	

	RobustMapping::RansacFitting_Linear_G(cloud_subMap, NUM); // reer.showCloud(cloud_subMap); _getch();
	RobustMapping::Positioning(cloud_subMap, NUM);// reer.showCloud(cloud_subMap); _getch();
	// The Result of deected curb and solutions	


	// 솔루션 찾은 결과를 띄워보는 코드
	/*cloud_in->clear();
	pcl::copyPointCloud(*cloud_subMap, *cloud_in);*/
	cloud_subMap->clear();
	// ICP 
	//cout << "ICP 시작" << endl;
	Eigen::Matrix4f CorrectedMat = this->GPS_TFM[NUM].mat;
	CorrectedMat(0, 3) = this->FinalSolution_x;
	CorrectedMat(1, 3) = this->FinalSolution_y;

	//RobustMapping::ICPWithNormals(cloud_temp, NUM, CorrectedMat);// reer.showCloud(cloud_subMap); _getch();

	ReferenceFrame tempRF;
	for each(auto i in *cloud_in__) tempRF.cloud.push_back(i);
	tempRF.Transformation = CorrectedMat;
	this->ReferenceMap.push_back(tempRF);
	this->BackStepPose = CorrectedMat;// 이번스텝 포즈 계산...
	// 맵빌딩 결과 출력

//	cout << "this->Solutions->size() : " << this->Solutions->size() << endl;
	/*for (int i = 0; i <= count; i++) {
	pcl::PointXYZRGB befPose, CorrPose;
	CorrPose.x = this->Solutions[i][0];
	CorrPose.y = this->Solutions[i][1];
	CorrPose.g = 85;
	CorrPose.r = 125;
	CorrPose.b = 255;
	befPose.x = this->Solutions[i][2];
	befPose.y = this->Solutions[i][3];
	befPose.r = 255;
	cloud_in->push_back(CorrPose);
	cloud_in->push_back(befPose);
	}*/
	this->count++;

	// 맵빌딩 결과 Pose Logging
	this->MeasuredPOSE_LOG << this->GPS_Table[0][NUM] << "	" << this->GPS_Table[1][NUM] << "	" << this->GPS_Table[2][NUM] << "	"
		<< this->GPS_Table[3][NUM] << "	" << this->GPS_Table[4][NUM] << "	" << this->GPS_Table[5][NUM] << "	" << this->GPS_Table[6][NUM] << endl;
	this->MeasuredPOSE_LOG_forMATLAB << this->GPS_Table[0][NUM] << "	" << this->GPS_Table[1][NUM] << "	" << this->GPS_Table[2][NUM] << endl;

	double tempx(0), tempy(0), tempz(0), tempyaw(0), temproll(0), temppitch(0);
	RobustMapping::getInverseTransformation(CorrectedMat, tempx, tempy, tempz, temproll, temppitch, tempyaw);
	this->CorrectedPOSE_LOG << NUM << "	" << tempx << "	" << tempy << "	" << tempz << "	" << tempyaw << "	" << temproll << "	" << temppitch << endl;

	this->CorrectedPOSE_LOG_forMATLAB << NUM << "	" << tempx << "	" << tempy << endl;


	return 1;

}

void
RobustMapping::Positioning(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {
	// Distance offset 구하기	
	double LocalDistance = this->LateralSolution_L.distance_L + this->LateralSolution_L.distance_R;
	double GlobalDistance = this->LateralSolution_G.distance_L + this->LateralSolution_G.distance_R;
	float DistanceOffset = abs((float)LocalDistance - (float)GlobalDistance);
	cout << "this->LateralSolution_L.distance_L : " << this->LateralSolution_L.distance_L << endl;
	cout << "this->LateralSolution_L.distance_R : " << this->LateralSolution_L.distance_R << endl;
	cout << "LocalDistance : " << LocalDistance << endl;
	cout << "this->LateralSolution_G.distance_L : " << this->LateralSolution_G.distance_L << endl;
	cout << "this->LateralSolution_G.distance_R : " << this->LateralSolution_G.distance_R << endl;
	cout << "GlobalDistance : " << GlobalDistance << endl;
	cout << "DistanceOffset : " << DistanceOffset << endl;// _getch();
	if (GlobalDistance > LocalDistance) {
		this->LateralSolution_L.distance_L += DistanceOffset / 2;
		this->LateralSolution_L.distance_R += DistanceOffset / 2;
	}
	else if (GlobalDistance < LocalDistance) {
		this->LateralSolution_L.distance_L -= DistanceOffset / 2;
		this->LateralSolution_L.distance_R -= DistanceOffset / 2;
	}

	// 최종 솔루션 구하기
	double xL = this->LateralSolution_G.x_L;
	double yL = this->LateralSolution_G.y_L;
	double xR = this->LateralSolution_G.x_R;
	double yR = this->LateralSolution_G.y_R;
	double LeftDist = this->LateralSolution_L.distance_L;
	double RightDist = this->LateralSolution_L.distance_R;
	double totalDist = this->LateralSolution_L.distance_L + this->LateralSolution_L.distance_R;

	this->FinalSolution_x = (LeftDist*xR + RightDist*xL) / totalDist;
	this->FinalSolution_y = (LeftDist*yR + RightDist*yL) / totalDist;
	/*this->Solutions[this->count].push_back(FinalSolution_x);
	this->Solutions[this->count].push_back(FinalSolution_y);
	this->Solutions[this->count].push_back(this->GPS_Table[1][NUM]);
	this->Solutions[this->count].push_back(this->GPS_Table[2][NUM]);*/
	pcl::PointXYZRGB Pose_IMU;
	Pose_IMU.x = this->GPS_Table[1][NUM];
	Pose_IMU.y = this->GPS_Table[2][NUM];
	Pose_IMU.z = 0;
	Pose_IMU.r = 255;

	pcl::PointXYZRGB Pose_Solution;
	Pose_Solution.x = FinalSolution_x;
	Pose_Solution.y = FinalSolution_y;
	Pose_Solution.z = 0;
	Pose_Solution.g = 85;
	Pose_Solution.r = 125;
	Pose_Solution.b = 255;
	double PoseDiff = sqrt(pow(this->GPS_Table[1][NUM] - FinalSolution_x, 2) + pow(this->GPS_Table[2][NUM] - FinalSolution_y, 2));
	cout << "Pose Difference : " << PoseDiff << endl;
	cloud_in->push_back(Pose_IMU);
	cloud_in->push_back(Pose_Solution);

}

void
RobustMapping::NormalEstimation_G(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {
	cout << "Normal 계산 시작" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals(new pcl::PointCloud<pcl::Normal>);
	int ManagePoint = NUM - STEP_;
	double HeadingAngle = this->GPS_Table[4][ManagePoint];// +M_PI / 2;

	// Global Height cut
	double xp = sin(this->GPS_Table[5][ManagePoint] + 1.5*drad)*cos(this->GPS_Table[4][ManagePoint] + 1.5*drad);
	double yp = sin(this->GPS_Table[5][ManagePoint] + 1.5*drad)*sin(this->GPS_Table[4][ManagePoint] + 1.5*drad);
	double zp = cos(this->GPS_Table[5][ManagePoint] + 1.5*drad);
	double xn = sqrt(pow(xp, 2) + pow(zp, 2))*sin(this->GPS_Table[6][ManagePoint]);
	double yn = yp;
	double zn = sqrt(pow(xp, 2) + pow(zp, 2))*cos(this->GPS_Table[6][ManagePoint]);
	/*for each(auto i in *cloud_in) {
	if ((xn*(i.x - this->GPS_Table[1][NUM - STEP]) + yn*(i.y - this->GPS_Table[2][NUM - STEP]) + zn*(i.z - this->GPS_Table[3][NUM - STEP])) < HEIGHT_THRESHOLD)
	cloud_temp1->push_back(i);
	}*/

	for each(auto i in *cloud_in) {
		if ((xn*(i.x - this->GPS_Table[1][ManagePoint]) + yn*(i.y - this->GPS_Table[2][ManagePoint]) + zn*(i.z - this->GPS_Table[3][ManagePoint])) < HEIGHT_THRESHOLD) {
			if ((i.y < tan(HeadingAngle)*(i.x - this->GPS_Table[1][ManagePoint]) + this->GPS_Table[2][ManagePoint] + 10) &&
				(i.y > tan(HeadingAngle)*(i.x - this->GPS_Table[1][ManagePoint]) + this->GPS_Table[2][ManagePoint] - 80)){
				cloud_temp2->push_back(i);
			}
		}
	}
	//	pcl::visualization::CloudViewer reer("ee"); reer.showCloud(cloud_temp2); _getch();
	// Compute the cloud_Normals of the cloud (do not worry, we will see this later).
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_temp2);
	normalEstimation.setRadiusSearch(KDTREE_SEARCH_GN);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_nor(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree_nor);
	normalEstimation.compute(*cloud_Normals);
	/*for (int i = 0; i < cloud_Normals->size(); i++) {
	if (cloud_Normals->points[i].normal_x < this->QNAN_) continue;
	cout << cloud_Normals->points[i].normal_x << ", " << cloud_Normals->points[i].normal_y << ", " << cloud_Normals->points[i].normal_z << endl;
	_getch();
	}*/
	PRODUCT_LOG << "Idx\t" << "Product(n*y)\t" << "Product(n*z)" << endl;

	for (int i = 0; i<cloud_Normals->size(); i++) {
		Eigen::Vector3f Ref1;
		Ref1[0] = xp;
		Ref1[1] = yp;
		Ref1[2] = zp;
		Eigen::Vector3f Ref2;
		Ref2[0] = xn;
		Ref2[1] = yn;
		Ref2[2] = zn;
		Eigen::Vector3f tempNormal;
		tempNormal[0] = cloud_Normals->points[i].normal_x;
		tempNormal[1] = cloud_Normals->points[i].normal_y;
		tempNormal[2] = cloud_Normals->points[i].normal_z;
		double prod1 = abs(tempNormal.transpose()*Ref1);
		double prod2 = abs(tempNormal.transpose()*Ref2);
		//cout << "Ref1 = " << Ref1 << "\nRef2 = " << Ref2 << "\ntempNormal = " << tempNormal << endl;
		//_getch();
		PRODUCT_LOG << i << "\t" << prod1 << "\t" << prod2 << endl;
		if (prod1 < PRODUCT_THRESHOLD && prod2 < PRODUCT_THRESHOLD_) {
			cloud_temp->push_back(cloud_temp2->points[i]);
		}
	}
	cout << "Normal 결과" << endl;
	/*pcl::visualization::CloudViewer vv("tet");
	vv.showCloud(cloud_temp); _getch();*/
	cloud_in->clear();
	// CurbMap 만들기를 robust하게
	//this->CurbMap.push_back(*cloud_temp);
	pcl::copyPointCloud(*cloud_temp, *cloud_in);
	cloud_temp->clear();
}


// 추가해야할 것 : 1. 이전포즈와 현재포즈 비교하여 에러 최소화 2.Robust CurbMap 생성 
void
RobustMapping::RansacFitting_Poly_G(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {
	cout << "Fitting 시작" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fitting(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Projection Process

	for each(auto i in *cloud_in) {
		pcl::PointXYZRGB tempPoint;
		tempPoint.x = i.x;
		tempPoint.y = i.y;
		tempPoint.z = 0;
		tempPoint.r = 255;
		tempPoint.g = 255;
		tempPoint.b = 255;
		cloud_projected_->push_back(tempPoint);
	}
	//pcl::transformPointCloud(*cloud_projected_, *cloud_projected, this->GPS_TFM[NUM])
	float HeadingAngle = this->GPS_Table[4][NUM] + M_PI / 2;  // 현재 라디안, 헤딩앵글 입력
	float xPoint = this->GPS_Table[1][NUM]; // 현재 y 대입
	float yPoint = this->GPS_Table[2][NUM]; // 현재 x 대입
	float Slope = tan(this->GPS_Table[4][NUM]);
	float Slope_ = tan(HeadingAngle);

	for (int i = 0; i < 100; i++) {
		double t_ = -25 + 0.5*(double)i;
		pcl::PointXYZRGB tempPoint;
		/*tempPoint.x = xPoint + t_*cos(this->GPS_Table[4][NUM-STEP]);
		tepPoint.y = yPoint + t_*sin(this->GPS_Table[4][NUM - STEP]);*/
		tempPoint.x = t_;
		tempPoint.y = tan(this->GPS_Table[4][NUM])*(t_ - xPoint) + yPoint;
		tempPoint.z = 0;
		tempPoint.g = 255;
		tempPoint.r = 255;
		cloud_fitting->push_back(tempPoint);
	}
	//cout << "cloud_projected_->size() : " << cloud_projected_->size() << endl;
	//pcl::visualization::CloudViewer tet("글로벌랜잭결과");
	//tet.showCloud(cloud_in); _getch();
	//tet.showCloud(cloud_projected_); _getch();
	for each(auto i in *cloud_projected_) {
		if (i.y - (Slope_ *(i.x - xPoint) + yPoint)< 0) cloud_left->push_back(i);	// Support vector
		else cloud_right->push_back(i);
	}

	//tet.showCloud(cloud_left);	_getch();
	//tet.showCloud(cloud_right);	_getch();
	double distance_threshold = 0.1;

	//RANSAC_custom* RANSACtest_L(new RANSAC_custom);
	RANSAC_custom RANSACtest_L;
	pLine Model_L;
	int no_data_L = cloud_left->size();
	pPoint *data_L = new pPoint[no_data_L];
	int cnt = 0;
	for each(auto i in *cloud_left) {
		data_L[cnt].x = i.x;
		data_L[cnt].y = i.y;
		cnt++;
	}
	double Cost_L = RANSACtest_L.PolyFitting_cubic(data_L, no_data_L, Model_L, distance_threshold);
	RANSACtest_L.~RANSAC_custom();

	// Right 클라우드 RANSAC fitting
	//RANSAC_custom* RANSACtest_R(new RANSAC_custom);
	RANSAC_custom RANSACtest_R;
	pLine Model_R;
	int no_data_R = cloud_right->size();
	pPoint *data_R = new pPoint[no_data_R];
	cnt = 0;
	for each(auto i in *cloud_right) {
		data_R[cnt].x = i.x;
		data_R[cnt].y = i.y;
		cnt++;
	}
	double Cost_R = RANSACtest_R.PolyFitting_cubic(data_R, no_data_R, Model_R, distance_threshold);
	RANSACtest_R.~RANSAC_custom();
	//delete RANSACtest_R;	


	for each(auto i in *cloud_left) cloud_fitting->push_back(i);
	for each(auto i in *cloud_right) cloud_fitting->push_back(i);
	double Max_X = cloud_fitting->points[0].x;
	double Min_X = cloud_fitting->points[0].x;
	for each(auto i in *cloud_fitting) {
		if (i.x >= Max_X) Max_X = i.x;
		else if (i.x <= Min_X) Min_X = i.x;
	}
	cout << "Max_X : " << Max_X << "\tMin_X : " << Min_X << endl;

	for (int i = 0; i < 400; i++) {
		double t_ = -20 + 0.1*(double)i;
		pcl::PointXYZRGB tempPoint_L, tempPoint_R;
		tempPoint_L.y = Model_L.a*pow(t_, 3) + Model_L.b*pow(t_, 2) + Model_L.c*pow(t_, 1) + Model_L.d*pow(t_, 0);
		tempPoint_L.x = t_;
		tempPoint_L.z = 0;
		tempPoint_L.r = 255;
		tempPoint_R.y = Model_R.a*pow(t_, 3) + Model_R.b*pow(t_, 2) + Model_R.c*pow(t_, 1) + Model_R.d*pow(t_, 0);
		tempPoint_R.x = t_;
		tempPoint_R.z = 0;
		tempPoint_R.r = 255;
		cloud_fitting->push_back(tempPoint_L);
		cloud_fitting->push_back(tempPoint_R);
	}


	// Heading 대비 lateral solution 구하기
	// pose 정보 - 로컬 프레임에서
	//float HeadingAngle = M_PI / 2;  // 라디안, 헤딩앵글 입력
	//float xPoint = 0; // y 대입
	//float yPoint = 0; // x 대입
	//float Slope = -sin(M_PI / 2 - HeadingAngle) / cos(M_PI / 2 - HeadingAngle);
	cout << "SLOPE : " << Slope << endl;
	float *solution_L = new float[3];
	float *solution_R = new float[3];
	int SolNum_L;
	int SolNum_R;
	//if (Slope < FLT_EPSILON_CUS) {
	//	solution_L[0] = Model_L.d;
	//	solution_R[0] = Model_R.d;
	//	SolNum_L = 1;
	//	SolNum_R = 1;
	//}
	//else { // ax^3 + bx^2 + cx + d 와 tan(GPS[4] +M_PI/2)(x - xPoint) + yPoint 의 교점
	//	SolNum_L = RobustMapping::solve_cubic(Model_L.a, Model_L.b, Model_L.c - Slope, Model_L.d - (yPoint - Slope*xPoint), solution_L);
	//	SolNum_R = RobustMapping::solve_cubic(Model_R.a, Model_R.b, Model_R.c - Slope, Model_R.d - (yPoint - Slope*xPoint), solution_R);
	//}
	// ax^3 + bx^2 + cx + d 와 tan(GPS[4] +M_PI/2)(x - xPoint) + yPoint 의 교점
	SolNum_L = RobustMapping::solve_cubic(Model_L.a, Model_L.b, Model_L.c - Slope, Model_L.d - (yPoint - Slope*xPoint), solution_L);
	cout << "-----------------교점구하기->왼쪽 ----------" << endl;
	cout << "Model_L.a : " << Model_L.a << "\nModel_L.b : " << Model_L.b << "\nModel_L.c - Slope : " << Model_L.c - Slope << "\nModel_L.d - (yPoint - Slope*xPoint) : " << Model_L.d - (yPoint - Slope*xPoint) << endl;
	SolNum_R = RobustMapping::solve_cubic(Model_R.a, Model_R.b, Model_R.c - Slope, Model_R.d - (yPoint - Slope*xPoint), solution_R);
	cout << "-----------------교점구하기->오른쪽 ----------" << endl;
	cout << "Model_R.a : " << Model_R.a << "\nModel_R.b : " << Model_R.b << "\nModel_R.c - Slope : " << Model_R.c - Slope << "\nModel_R.d - (yPoint - Slope*xPoint) : " << Model_R.d - (yPoint - Slope*xPoint) << endl;

	cout << "SolNum_L : " << SolNum_L << endl;
	cout << "SolNum_R : " << SolNum_R << endl;
	pcl::PointXYZRGB tempL;		// 솔류션 시각화를 위한
	//GLOBAL_SOLUTION tempSolution;
	this->LateralSolution_G.frame = NUM;
	if (SolNum_L == 1) {
		cout << "solution_L[0] : " << solution_L[0] << endl;
		tempL.x = solution_L[0];
		tempL.y = Slope*(solution_L[0] - xPoint) + yPoint;
		//tempL.y = Model_L.a*pow(solution_L[0], 3) + Model_L.b*pow(solution_L[0], 2) + Model_L.c * solution_L[0] + Model_L.d;
		tempL.z = 0;
		tempL.g = 255;
		this->LateralSolution_G.x_L = solution_L[0];
		this->LateralSolution_G.y_L = Slope*(solution_L[0] - xPoint) + yPoint;
		this->LateralSolution_G.distance_L = sqrt(pow(tempL.x - xPoint, 2) + pow(tempL.y - yPoint, 2));
		this->DISTANCES_LOG_G << NUM << "\t" << solution_L[0];
		cloud_fitting->push_back(tempL);
	}
	else {
		double tempSolution;
		for (int k = 0; k < SolNum_L; k++) {
			cout << "solution_L[" << k << "] : " << solution_L[k] << endl;
			if (solution_L[k] < Max_X && solution_L[k] > Min_X) {
				tempSolution = solution_L[k];
			}
		}
		cout << "tempSolution L : " << tempSolution << endl;
		tempL.x = tempSolution;
		tempL.y = Slope*(tempSolution - xPoint) + yPoint;
		//tempL.y = Model_L.a*pow(solution_L[0], 3) + Model_L.b*pow(solution_L[0], 2) + Model_L.c * solution_L[0] + Model_L.d;
		tempL.z = 0;
		tempL.g = 255;
		this->LateralSolution_G.x_L = tempSolution;
		this->LateralSolution_G.y_L = tempL.y;
		this->LateralSolution_G.distance_L = sqrt(pow(tempL.x - xPoint, 2) + pow(tempL.y - yPoint, 2));
		this->DISTANCES_LOG_G << NUM << "\t" << tempSolution;

		//cout << "솔루션 이상, 글로벌 뷰어 끄셈" << endl; _getch();
		//pcl::visualization::CloudViewer Vieww("Wiered solution result");
		for (int k = 0; k < SolNum_L; k++) {
			pcl::PointXYZRGB tempL_;
			tempL_.x = solution_L[k];
			tempL_.y = Slope*(solution_L[k] - xPoint) + yPoint;
			//tempL.y = Model_L.a*pow(solution_L[0], 3) + Model_L.b*pow(solution_L[0], 2) + Model_L.c * solution_L[0] + Model_L.d;
			tempL_.z = 0;
			tempL_.g = 100 + k * 40;
			cloud_fitting->push_back(tempL);
		}
		//Vieww.showCloud(cloud_fitting); _getch();
		//throw runtime_error("솔루션이 이상해");
	}

	pcl::PointXYZRGB tempR; // 솔류션 시각화를 위한
	if (SolNum_R == 1) {
		cout << "solution_R[0] : " << solution_R[0] << endl;
		tempR.x = solution_R[0];
		tempR.y = Slope*(solution_R[0] - xPoint) + yPoint;
		//tempR.y = Model_R.a*pow(solution_R[0], 3) + Model_R.b*pow(solution_R[0], 2) + Model_R.c * solution_R[0] + Model_R.d;
		tempR.z = 0;
		tempR.g = 255;
		this->LateralSolution_G.x_R = tempR.x;
		this->LateralSolution_G.y_R = tempR.y;
		this->LateralSolution_G.distance_R = sqrt(pow(tempR.x - xPoint, 2) + pow(tempR.y - yPoint, 2));
		this->DISTANCES_LOG_G << "\t" << solution_R[0] << "\t" << LateralSolution_G.distance_L + LateralSolution_G.distance_R << endl;

		cloud_fitting->push_back(tempR);
	}
	else {
		double tempSolution = 0;
		for (int k = 0; k < SolNum_R; k++) {
			cout << "solution_R[" << k << "] : " << solution_R[k] << endl;
			if (solution_R[k] < Max_X && solution_R[k] > Min_X) {
				tempSolution = solution_R[k];
			}
		}
		cout << "tempSolution R : " << tempSolution << endl;
		tempR.x = tempSolution;
		tempR.y = Slope*(tempSolution - xPoint) + yPoint;
		//tempL.y = Model_L.a*pow(solution_L[0], 3) + Model_L.b*pow(solution_L[0], 2) + Model_L.c * solution_L[0] + Model_L.d;
		tempR.z = 0;
		tempR.g = 255;
		this->LateralSolution_G.x_R = tempSolution;
		this->LateralSolution_G.y_R = tempR.y;
		this->LateralSolution_G.distance_R = sqrt(pow(tempR.x - xPoint, 2) + pow(tempR.y - yPoint, 2));
		this->DISTANCES_LOG_G << NUM << "\t" << tempSolution;
		//cout << "솔루션 이상, 글로벌 뷰어 끄셈" << endl; _getch();
		//pcl::visualization::CloudViewer Vieww("Wiered solution result");
		for (int k = 0; k < SolNum_R; k++) {
			pcl::PointXYZRGB tempR_;
			tempR_.x = solution_R[k];
			tempR_.y = Slope*(solution_R[k] - xPoint) + yPoint;
			//tempL.y = Model_L.a*pow(solution_L[0], 3) + Model_L.b*pow(solution_L[0], 2) + Model_L.c * solution_L[0] + Model_L.d;
			tempR_.z = 0;
			tempR_.g = 100 + k * 40;
			cloud_fitting->push_back(tempR);
		}
		//Vieww.showCloud(cloud_fitting); _getch();
		//throw runtime_error("Fuck this solution");
	}

	//////////////////////////////////
	/*********양옆 거리정보 저장******/
	//this->LateralSolution_G = tempSolution;
	//////////////////////////////////




	//pcl::visualization::CloudViewer view("Robust Mapping_fitting");
	//view.showCloud(cloud_fitting); _getch();
	cloud_in->clear();
	pcl::copyPointCloud(*cloud_fitting, *cloud_in);
	delete[] data_L;
	delete[] data_R;
	delete[] solution_L;
	delete[] solution_R;
	cloud_fitting->clear();
	cloud_projected->clear();
	cloud_left->clear();
	cloud_right->clear();
}


void
RobustMapping::RansacFitting_Linear_G(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {
	cout << "Fitting 시작" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fitting(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Projection Process

	for each(auto i in *cloud_in) {
		pcl::PointXYZRGB tempPoint;
		tempPoint.x = i.x;
		tempPoint.y = i.y;
		tempPoint.z = 0;
		tempPoint.r = 255;
		tempPoint.g = 255;
		tempPoint.b = 255;
		cloud_projected_->push_back(tempPoint);
	}
	cloud_in->clear();
	//pcl::transformPointCloud(*cloud_projected_, *cloud_projected, this->GPS_TFM[NUM])
	float HeadingAngle = this->GPS_Table[4][NUM] + M_PI / 2;  // 현재 라디안, 헤딩앵글 입력
	float xPoint = this->GPS_Table[1][NUM]; // 현재 y 대입
	float yPoint = this->GPS_Table[2][NUM]; // 현재 x 대입
	float Slope = tan(this->GPS_Table[4][NUM]);
	float Slope_ = tan(HeadingAngle);


	/*for (int i = 0; i < 100; i++) {
	double t_ = xPoint+ -25 + 0.5*(double)i;
	pcl::PointXYZRGB tempPoint;
	tempPoint.x = t_;
	tempPoint.y = tan(this->GPS_Table[4][NUM])*(t_ - xPoint) + yPoint;
	tempPoint.z = 0;
	tempPoint.g = 255;
	tempPoint.r = 255;
	cloud_fitting->push_back(tempPoint);
	}*/

	//cout << "cloud_projected_->size() : " << cloud_projected_->size() << endl;

	//tet.showCloud(cloud_in); _getch();
	//tet.showCloud(cloud_projected_); _getch();
	for each(auto i in *cloud_projected_) {
		if (i.y - (Slope_ *(i.x - xPoint) + yPoint)< 0) cloud_left->push_back(i);	// Support vector
		else cloud_right->push_back(i);
	}
	//pcl::visualization::CloudViewer tet("글로벌랜잭결과");
	//tet.showCloud(cloud_left);	_getch();
	//tet.showCloud(cloud_right);	_getch();
	
	//cout << "cloud_projected_->size() : " << cloud_projected_->size() << endl;
	//cout << "cloud_left->size() : " << cloud_left->size() << endl;
	//cout << "cloud_right->size()  : " << cloud_right->size() << endl;
	//cout << "cloud_fitting->size()   : " << cloud_fitting->size() << endl;
	double distance_threshold = 0.1;

	//RANSAC_custom* RANSACtest_L(new RANSAC_custom);
	RANSAC_custom RANSACtest_L;
	pLine_ Model_L;
	int no_data_L = cloud_left->size();
	pPoint *data_L = new pPoint[no_data_L];
	int cnt = 0;
	for each(auto i in *cloud_left) {
		data_L[cnt].x = i.x;
		data_L[cnt].y = i.y;
		cnt++;
	}
	double Cost_L = RANSACtest_L.PolyFitting_line(data_L, no_data_L, Model_L, distance_threshold);
	RANSACtest_L.~RANSAC_custom();

	// Right 클라우드 RANSAC fitting
	//RANSAC_custom* RANSACtest_R(new RANSAC_custom);
	RANSAC_custom RANSACtest_R;
	pLine_ Model_R;
	int no_data_R = cloud_right->size();
	pPoint *data_R = new pPoint[no_data_R];
	cnt = 0;
	for each(auto i in *cloud_right) {
		data_R[cnt].x = i.x;
		data_R[cnt].y = i.y;
		cnt++;
	}
	double Cost_R = RANSACtest_R.PolyFitting_line(data_R, no_data_R, Model_R, distance_threshold);
	RANSACtest_R.~RANSAC_custom();
	//delete RANSACtest_R;	


	//for each(auto i in *cloud_left) cloud_fitting->push_back(i);
	//for each(auto i in *cloud_right) cloud_fitting->push_back(i);
	pcl::copyPointCloud(*cloud_projected_, *cloud_fitting);
	cloud_left->clear();
	cloud_right->clear();
	cloud_projected_->clear();

	double Max_X = cloud_fitting->points[0].x;
	double Min_X = cloud_fitting->points[0].x;
	for each(auto i in *cloud_fitting) {
		if (i.x >= Max_X) Max_X = i.x;
		else if (i.x <= Min_X) Min_X = i.x;
	}
	cout << "Max_X : " << Max_X << "\tMin_X : " << Min_X << endl;

	//직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
	//직선의 방정식: my*(x - sx) = mx*(y - sy);
	double m_L = Model_L.my / Model_L.mx;
	double m_R = Model_R.my / Model_R.mx;

	pcl::search::KdTree<pcl::PointXYZRGB> kdtree_curb;
	kdtree_curb.setInputCloud(cloud_fitting);
	vector<int> PointIndecies;
	vector<float> SquaredDistances;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_curb(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < 400; i++) {
		double t_ = xPoint - 10 + 0.05*(double)i;
		pcl::PointXYZRGB tempPoint_L, tempPoint_R;
		tempPoint_L.y = m_L*(t_ - Model_L.sx) + Model_L.sy;
		tempPoint_L.x = t_;
		tempPoint_L.z = 0;
		tempPoint_L.r = 255;
		tempPoint_R.y = m_R*(t_ - Model_R.sx) + Model_R.sy;
		tempPoint_R.x = t_;
		tempPoint_R.z = 0;
		tempPoint_R.r = 255;
		cloud_fitting->push_back(tempPoint_L);
		cloud_fitting->push_back(tempPoint_R);

		if (kdtree_curb.radiusSearch(tempPoint_L, KDTREE_SEARCH_CURB, PointIndecies, SquaredDistances) > 0) {
			//cout << "PointIndecies.size() L : " << PointIndecies.size() << endl;
			if (PointIndecies.size()<1) {} // Do Nothing
			else if (PointIndecies.size() >= 1) {
				for each(auto i in PointIndecies) cloud_curb->push_back(cloud_fitting->points[i]);
			}
		}
		PointIndecies.clear();
		SquaredDistances.clear();
		if (kdtree_curb.radiusSearch(tempPoint_R, KDTREE_SEARCH_CURB, PointIndecies, SquaredDistances) > 0) {
			//cout << "PointIndecies.size() R : " << PointIndecies.size() << endl;
			if (PointIndecies.size()<1) {} // Do Nothing
			else if (PointIndecies.size() >= 1) {
				for each(auto i in PointIndecies) cloud_curb->push_back(cloud_fitting->points[i]);
			}
		}
	//	tet.showCloud(cloud_fitting);	_getch();
		PointIndecies.clear();
		SquaredDistances.clear();
	}
	// Curb맵 -> robustness
	//this->CurbMap.push_back(*cloud_curb);
	if (this->CurbMap_.size() != 0) this->CurbMap_.clear();
	for each(auto points in *cloud_curb) this->CurbMap_.push_back(points);
	cloud_curb->clear();

	// Heading 대비 lateral solution 구하기
	// pose 정보 - 로컬 프레임에서
	//float HeadingAngle = M_PI / 2;  // 라디안, 헤딩앵글 입력
	//float xPoint = 0; // y 대입
	//float yPoint = 0; // x 대입
	//float Slope = -sin(M_PI / 2 - HeadingAngle) / cos(M_PI / 2 - HeadingAngle);
	cout << "SLOPE : " << Slope << endl;
	float *solution_L = new float[3];
	float *solution_R = new float[3];
	int SolNum_L;
	int SolNum_R;

	Eigen::Matrix4f InitialAlign;
	this->GlobalTrans = this->GPS_TFM[NUM].mat*this->GPS_TFM[NUM - STEP_].mat.inverse();
	InitialAlign = this->GlobalTrans * this->BackStepPose;
	float xPoint_ = InitialAlign(0, 3); // 보정된 현재 x 대입
	float yPoint_ = InitialAlign(1, 3);  // 보정된 현재 y 대입
	pcl::PointXYZRGB ttt, yyy, ppp;

	ttt.x = xPoint_;
	ttt.y = yPoint_;
	ttt.z = 0;
	ttt.g = 255;

	//yyy.x = this->GPS_TFM[NUM].mat(0,3)+0.05;
	//yyy.y = this->GPS_TFM[NUM].mat(1, 3);
	//yyy.z = 0;
	//yyy.r = 255;
	//yyy.g = 255;

	ppp.x = this->FinalSolution_x;
	ppp.y = this->FinalSolution_y;
	ppp.z = 0;
	ppp.g = 125;
	ppp.r = 225;
	ppp.b = 125;
	cloud_fitting->push_back(ttt);
	//cloud_fitting->push_back(yyy);
	cloud_fitting->push_back(ppp);
	//xPoint_ = xPoint;
	//yPoint_ = yPoint;

	SolNum_L = RobustMapping::solve_linear(m_L - Slope, Model_L.sy - (yPoint_ - Slope*xPoint_) - m_L*Model_L.sx, solution_L);
	cout << "-----------------교점구하기->왼쪽 ----------" << endl;
	cout << "m_L - Slope : " << m_L - Slope << "\nModel_L.sy - (yPoint - Slope*xPoint) : " << Model_L.sy - (yPoint_ - Slope*xPoint_) << endl;
	SolNum_R = RobustMapping::solve_linear(m_R - Slope, Model_R.sy - (yPoint_ - Slope*xPoint_) - m_R*Model_R.sx, solution_R);
	cout << "-----------------교점구하기->오른쪽 ----------" << endl;
	cout << "m_R - Slope : " << m_R - Slope << "\nModel_R.sy - (yPoint - Slope*xPoint) : " << Model_R.sy - (yPoint_ - Slope*xPoint_) << endl;

	cout << "SolNum_L : " << SolNum_L << endl;
	cout << "SolNum_R : " << SolNum_R << endl;

	pcl::PointXYZRGB tempL;		// 솔류션 시각화를 위한
	//GLOBAL_SOLUTION tempSolution;
	this->LateralSolution_G.frame = NUM;
	if (SolNum_L == 1) {
		cout << "solution_L[0] : " << solution_L[0] << endl;
		tempL.x = solution_L[0];
		tempL.y = Slope*(solution_L[0] - xPoint) + yPoint;
		//tempL.y = Model_L.a*pow(solution_L[0], 3) + Model_L.b*pow(solution_L[0], 2) + Model_L.c * solution_L[0] + Model_L.d;
		tempL.z = 0;
		tempL.g = 255;
		this->LateralSolution_G.x_L = solution_L[0];
		this->LateralSolution_G.y_L = Slope*(solution_L[0] - xPoint) + yPoint;
		this->LateralSolution_G.distance_L = sqrt(pow(tempL.x - xPoint, 2) + pow(tempL.y - yPoint, 2));
		this->DISTANCES_LOG_G << NUM << "\t" << solution_L[0];
		cloud_fitting->push_back(tempL);
	}

	pcl::PointXYZRGB tempR; // 솔류션 시각화를 위한
	if (SolNum_R == 1) {
		cout << "solution_R[0] : " << solution_R[0] << endl;
		tempR.x = solution_R[0];
		tempR.y = Slope*(solution_R[0] - xPoint) + yPoint;
		//tempR.y = Model_R.a*pow(solution_R[0], 3) + Model_R.b*pow(solution_R[0], 2) + Model_R.c * solution_R[0] + Model_R.d;
		tempR.z = 0;
		tempR.g = 255;
		this->LateralSolution_G.x_R = tempR.x;
		this->LateralSolution_G.y_R = tempR.y;
		this->LateralSolution_G.distance_R = sqrt(pow(tempR.x - xPoint, 2) + pow(tempR.y - yPoint, 2));
		this->DISTANCES_LOG_G << "\t" << solution_R[0] << "\t" << LateralSolution_G.distance_L + LateralSolution_G.distance_R << endl;

		cloud_fitting->push_back(tempR);
	}

	//////////////////////////////////
	/*********양옆 거리정보 저장******/
	//this->LateralSolution_G = tempSolution;
	//////////////////////////////////


	//pcl::visualization::CloudViewer view("Robust Mapping_fitting");
	//view.showCloud(cloud_fitting); _getch();

	pcl::copyPointCloud(*cloud_fitting, *cloud_in);
	delete[] data_L;
	delete[] data_R;
	delete[] solution_L;
	delete[] solution_R;
	cloud_fitting->clear();
	cloud_projected->clear();
	cloud_left->clear();
	cloud_right->clear();
}
void
RobustMapping::GetLateralDistances(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::visualization::CloudViewer viewer("Viewer");
	string name = RobustMapping::PathSetting(this->GlobalPath, NUM);
	
	pcl::io::loadPCDFile(name, *cloud_in);
	for each(auto i in *cloud_in) {
		pcl::PointXYZRGB tempPo;
		tempPo.x = i.x;		tempPo.y = i.y;		tempPo.z = i.z;
		tempPo.r = 255;		tempPo.g = 255;		tempPo.b = 255;
		cloud_temp1->push_back(tempPo);
	}
	//viewer.showCloud(cloud_temp1); _getch();
	for each(auto i in *cloud_temp1) {
		if (i.z < HEIGHT_THRESHOLD){	// && i.y<20 && i.y>-20){
			cloud_temp2->push_back(i);
		}
	}
	cloud_in->clear();
	pcl::copyPointCloud(*cloud_temp2, *cloud_in);
	cloud_temp1->clear();
	cloud_temp2->clear();

	//viewer.showCloud(cloud_in); _getch();
	pcl::console::TicToc time;
	time.tic();
	//RobustMapping::Smoothing(cloud_in);
	RobustMapping::GradientEstimation(cloud_in);	//viewer.showCloud(cloud_in); //_getch();
	//cout << "그래디언트 러닝타임 : " << time.toc() / 1000 << endl;
	//_getch();
	RobustMapping::NormalEstimation(cloud_in);  // viewer.showCloud(cloud_in); _getch();
	//RobustMapping::RansacFitting_Line(cloud_in);
	RobustMapping::RansacFitting_Poly_L(cloud_in, NUM); // viewer.showCloud(cloud_in); _getch();

}


void
RobustMapping::GetLateralDistances(int NUM) {


	cout << NUM << " 번째 프레임 처리중.. " << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	string name = RobustMapping::PathSetting(this->GlobalPath, NUM);
	pcl::io::loadPCDFile(name, *cloud_in);



	//pcl::visualization::CloudViewer thisis("viewer");
	RobustMapping::HeightCut(cloud_in);
	RobustMapping::GradientEstimation(cloud_in); //thisis.showCloud(cloud_in); _getch();
	RobustMapping::NormalEstimation(cloud_in);	//thisis.showCloud(cloud_in); _getch();
	RobustMapping::RansacFitting_Poly_L(cloud_in, NUM); //thisis.showCloud(cloud_in); _getch();

}


void
RobustMapping::HeightCut(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
	for each(auto i in *cloud_in) {
		if (i.z < HEIGHT_THRESHOLD && i.y<20 && i.y>-20){
			cloud_temp1->push_back(i);
		}
	}
	cloud_in->clear();
	pcl::copyPointCloud(*cloud_temp1, *cloud_in);
	cloud_temp1->clear();
}

void
RobustMapping::Smoothing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
	cout << "Smoothing 시작" << endl;
	pcl::console::TicToc time;
	time.tic();
	mls.setComputeNormals(true);
	// Set parameters
	mls.setInputCloud(cloud_in);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(.30);
	// Reconstruct
	mls.process(mls_points);
	cloud_in->clear();
	pcl::copyPointCloud(mls_points, *cloud_in);

	cout << "Smoothing 끝... " << time.toc() / 1000 << "초 " << endl;
}

void
RobustMapping::NormalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
	cout << "Normal 계산 시작" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals(new pcl::PointCloud<pcl::Normal>);

	// Compute the cloud_Normals of the cloud (do not worry, we will see this later).
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_in);
	normalEstimation.setRadiusSearch(0.10);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_nor(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree_nor);
	normalEstimation.compute(*cloud_Normals);
	/*for (int i = 0; i < cloud_Normals->size(); i++) {
	if (cloud_Normals->points[i].normal_x < this->QNAN_) continue;
	cout << cloud_Normals->points[i].normal_x << ", " << cloud_Normals->points[i].normal_y << ", " << cloud_Normals->points[i].normal_z << endl;
	_getch();
	}*/
	PRODUCT_LOG << "Idx\t" << "Product(n*y)\t" << "Product(n*z)" << endl;

	for (int i = 0; i<cloud_Normals->size(); i++) {
		Eigen::Vector3f Ref1;
		Ref1[0] = 0;
		Ref1[1] = 1;
		Ref1[2] = 0;
		Eigen::Vector3f Ref2;
		Ref2[0] = 0;
		Ref2[1] = 0;
		Ref2[2] = 1;
		Eigen::Vector3f tempNormal;
		tempNormal[0] = cloud_Normals->points[i].normal_x;
		tempNormal[1] = cloud_Normals->points[i].normal_y;
		tempNormal[2] = cloud_Normals->points[i].normal_z;
		double prod1 = abs(tempNormal.transpose()*Ref1);
		double prod2 = abs(tempNormal.transpose()*Ref2);
		//cout << "Ref1 = " << Ref1 << "\nRef2 = " << Ref2 << "\ntempNormal = " << tempNormal << endl;
		//_getch();
		PRODUCT_LOG << i << "\t" << prod1 << "\t" << prod2 << endl;
		if (abs(tempNormal.transpose()*Ref1) < PRODUCT_THRESHOLD && abs(tempNormal.transpose()*Ref2) < PRODUCT_THRESHOLD_) {

			cloud_temp->push_back(cloud_in->points[i]);
		}
	}
	//pcl::visualization::CloudViewer vv("tet");
	//vv.showCloud(cloud_in);
	cloud_in->clear();
	pcl::copyPointCloud(*cloud_temp, *cloud_in);
	cloud_temp->clear();
	//_getch();
	cout << "Normal 결과" << endl;
	//vv.showCloud(cloud_in);
	//_getch();
}

void
RobustMapping::GradientEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
	cout << "Gradient 계산 시작" << endl;
	// kd-tree object.
	pcl::search::KdTree<pcl::PointXYZRGB> kdtree_grad;
	kdtree_grad.setInputCloud(cloud_in);
	vector<double> PointGradients;
	// We will find the 5 nearest neighbors of this point
	// (it does not have to be one of the cloud's, we can use any coordinate).
	std::vector<int> pointIndices;
	// This vector will store their squared distances to the search point.
	std::vector<float> squaredDistances;
	for (int i = 0; i < cloud_in->size(); i++) {
		// This vector will store the output neighbors.
		double MinMax_z[2] = { 0, };
		int MinMax_idx[2] = { 0, };
		double dx, dy, dz;
		double Gradient;
		// Now we find all neighbors within 3cm of the point
		// (inside a sphere of radius 3cm centered at the point).
		if (kdtree_grad.radiusSearch(cloud_in->points[i], KDTREE_SEARCH_GG, pointIndices, squaredDistances) > 0) {
			/*std::cout << "Neighbors within 10cm:" << std::endl;
			cout << "pointIndices.size() : " << pointIndices.size() << endl;
			_getch();*/
			MinMax_z[0] = cloud_in->points[pointIndices[0]].z;
			MinMax_z[1] = cloud_in->points[pointIndices[0]].z;

			for (int j = 0; j < pointIndices.size(); ++j) {
				if (MinMax_z[0] > cloud_in->points[pointIndices[j]].z){
					MinMax_z[0] = cloud_in->points[pointIndices[j]].z;
					MinMax_idx[0] = pointIndices[j];
				}

				if (MinMax_z[1] < cloud_in->points[pointIndices[j]].z){
					MinMax_z[1] = cloud_in->points[pointIndices[j]].z;
					MinMax_idx[1] = pointIndices[j];
				}

				//cout << "points[pointIndices[j]].z = " << cloud_in->points[pointIndices[j]].z << endl;
			}

			// 미분값 계산
			//cout << "pointIndices.size() " << pointIndices.size() << endl;
			if (pointIndices.size() > 1) {
				dx = cloud_in->points[MinMax_idx[0]].x - cloud_in->points[MinMax_idx[1]].x;
				dy = cloud_in->points[MinMax_idx[0]].y - cloud_in->points[MinMax_idx[1]].y;
				dz = MinMax_z[1] - MinMax_z[0];
				//cout << "dx = " << dx << "\tdy = " << dy << "\tdz = " << dz << endl;
				Gradient = sqrt(pow(dz / dx, 2) + pow(dz / dy, 2));
				PointGradients.push_back(Gradient);

			}
			else if (pointIndices.size() <= 1){
				Gradient = 0;
				PointGradients.push_back(Gradient);
			}
			//	cout << "Gradient = " << Gradient << endl;
		}
		pointIndices.clear();
		squaredDistances.clear();
	}
	//	cout << "여기? G1" << endl;
	//if (cloud_in->size() == this->PointGradients.size()) cout << "Gradient 계산 끝" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	GRADIENT_LOG << "Idx\tGradient" << endl;

	for (int i = 0; i<PointGradients.size(); i++) {
		//cout << "Gradient of " << cnt << "th point : " << it << endl;
		GRADIENT_LOG << i << "\t" << PointGradients[i] << endl;
		if (PointGradients[i]>GRADIENT_THRESHOLD) cloud_temp->push_back(cloud_in->points[i]);
	}
	PointGradients.clear();
	//	cout << "여기? G2" << endl;
	//pcl::visualization::CloudViewer vv("d");
	//vv.showCloud(cloud_in);
	cloud_in->clear();
	pcl::copyPointCloud(*cloud_temp, *cloud_in);
	cloud_temp->clear();
	//_getch();
	cout << "Gradient 결과" << endl;
	//vv.showCloud(cloud_in);
	//_getch();
}

void
RobustMapping::RansacFitting_Poly_L(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, int NUM) {
	cout << "Fitting 시작" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Projection Process
	// Create a set of planar coefficients with X=Y=0,Z=1
	/*pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_in);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);*/
	for each(auto i in *cloud_in) {
		pcl::PointXYZRGB tempPoint;
		tempPoint.x = i.x;
		tempPoint.y = i.y;
		tempPoint.z = 0;
		tempPoint.r = 255;
		tempPoint.g = 255;
		tempPoint.b = 255;
		cloud_projected->push_back(tempPoint);
	}

	for each(auto i in *cloud_projected) {
		if (i.x < 0) cloud_left->push_back(i);
		else cloud_right->push_back(i);
	}
	double distance_threshold = 0.1;
	// Left 클라우드 RANSAC fitting
	//int cntt = 1;
	//cout << "여기? " << cntt++ << endl;
	//_getch();
	/*RANSAC_custom RANSACtest_;
	pLine Model_;
	int no_data_ = cloud_projected->size();
	pPoint *data_ = new pPoint[no_data_];
	int cnt = 0;
	for each(auto i in *cloud_projected) {
	data_[cnt].x = i.y;
	data_[cnt].y = i.x;
	cnt++;
	}
	double Cost_ = RANSACtest_.PolyFitting(data_, no_data_, Model_, distance_threshold);
	RANSACtest_.~RANSAC_custom();
	cout << Model_.a << ", " << Model_.b << ", " << Model_.c << ", " << Model_.d << endl;

	for (int i = 0; i < 200; i++) {
	double t_ = -10 + 0.1*(double)i;
	pcl::PointXYZRGB tempPoint_L, tempPoint_R;
	tempPoint_L.x = Model_.a*pow(t_, 3) + Model_.b*pow(t_, 2) + Model_.c*pow(t_, 1) + Model_.d*pow(t_, 0);
	tempPoint_L.y = t_;
	tempPoint_L.z = 0;
	tempPoint_L.r = 255;
	cloud_projected->push_back(tempPoint_R);
	}
	pcl::visualization::CloudViewer cc("d");
	cc.showCloud(cloud_projected);
	_getch();*/

	//RANSAC_custom* RANSACtest_L(new RANSAC_custom);
	RANSAC_custom RANSACtest_L;
	pLine Model_L;
	int no_data_L = cloud_left->size();
	pPoint *data_L = new pPoint[no_data_L];
	int cnt = 0;
	for each(auto i in *cloud_left) {
		data_L[cnt].x = i.y;
		data_L[cnt].y = i.x;
		cnt++;
	}
	double Cost_L = RANSACtest_L.PolyFitting_cubic(data_L, no_data_L, Model_L, distance_threshold);
	RANSACtest_L.~RANSAC_custom();

	// Right 클라우드 RANSAC fitting
	//RANSAC_custom* RANSACtest_R(new RANSAC_custom);
	RANSAC_custom RANSACtest_R;
	pLine Model_R;
	int no_data_R = cloud_right->size();
	pPoint *data_R = new pPoint[no_data_R];
	cnt = 0;
	for each(auto i in *cloud_right) {
		data_R[cnt].x = i.y;
		data_R[cnt].y = i.x;
		cnt++;
	}
	double Cost_R = RANSACtest_R.PolyFitting_cubic(data_R, no_data_R, Model_R, distance_threshold);
	RANSACtest_R.~RANSAC_custom();
	//delete RANSACtest_R;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fitting(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < 200; i++) {
		double t_ = -10 + 0.1*(double)i;
		pcl::PointXYZRGB tempPoint_L, tempPoint_R;
		tempPoint_L.x = Model_L.a*pow(t_, 3) + Model_L.b*pow(t_, 2) + Model_L.c*pow(t_, 1) + Model_L.d*pow(t_, 0);
		tempPoint_L.y = t_;
		tempPoint_L.z = 0;
		tempPoint_L.r = 255;
		tempPoint_R.x = Model_R.a*pow(t_, 3) + Model_R.b*pow(t_, 2) + Model_R.c*pow(t_, 1) + Model_R.d*pow(t_, 0);
		tempPoint_R.y = t_;
		tempPoint_R.z = 0;
		tempPoint_R.r = 255;
		cloud_fitting->push_back(tempPoint_L);
		cloud_fitting->push_back(tempPoint_R);
	}
	for each(auto i in *cloud_left) cloud_fitting->push_back(i);
	for each(auto i in *cloud_right) cloud_fitting->push_back(i);

	// Heading 대비 lateral solution 구하기
	// pose 정보 - 로컬 프레임에서
	float HeadingAngle = M_PI / 2;  // 라디안, 헤딩앵글 입력
	float xPoint = 0; // y 대입
	float yPoint = 0; // x 대입
	float Slope = -sin(M_PI / 2 - HeadingAngle) / cos(M_PI / 2 - HeadingAngle);
	//cout << "SLOPE : " << Slope << endl;
	float *solution_L = new float[3];
	float *solution_R = new float[3];
	int SolNum_L;
	int SolNum_R;
	if (Slope < FLT_EPSILON_CUS) {
		solution_L[0] = Model_L.d;
		solution_R[0] = Model_R.d;
		SolNum_L = 1;
		SolNum_R = 1;
	}
	else {
		SolNum_L = RobustMapping::solve_cubic(Model_L.a, Model_L.b, Model_L.c - Slope, Model_L.d - (yPoint - Slope*xPoint), solution_L);
		SolNum_R = RobustMapping::solve_cubic(Model_R.a, Model_R.b, Model_R.c - Slope, Model_R.d - (yPoint - Slope*xPoint), solution_R);
	}

	cout << "SolNum_L : " << SolNum_L << endl;
	cout << "SolNum_R : " << SolNum_R << endl;
	pcl::PointXYZRGB tempL;
	LATERAL_DATA tempData;
	tempData.frame = NUM;
	if (SolNum_L == 1) {
		cout << "solution_L[0] : " << solution_L[0] << endl;
		tempL.x = solution_L[0];
		tempL.y = Slope*(solution_L[0] - xPoint) + yPoint;
		tempL.z = 0;
		tempL.g = 255;
		tempData.distance_L = sqrt(pow(tempL.x - xPoint, 2) + pow(tempL.y - yPoint, 2));
		this->DISTANCES_LOG << NUM << "\t" << solution_L[0];
	}
	else throw runtime_error("솔루션이 이상해");
	pcl::PointXYZRGB tempR;
	if (SolNum_R == 1) {
		cout << "solution_R[0] : " << solution_R[0] << endl;
		tempR.x = solution_R[0];
		tempR.y = Slope*(solution_R[0] - xPoint) + yPoint;
		tempR.z = 0;
		tempR.g = 255;
		tempData.distance_R = sqrt(pow(tempR.x - xPoint, 2) + pow(tempR.y - yPoint, 2));
		this->DISTANCES_LOG << "\t" << solution_R[0] << "\t" << tempData.distance_L + tempData.distance_R << endl;
	}
	else throw runtime_error("솔루션 이상");

	//this->LateralDistances.push_back(tempData);
	this->LateralSolution_L = tempData;

	cloud_fitting->push_back(tempL);
	cloud_fitting->push_back(tempR);
	//for each(auto i in *cloud_genesis) cloud_fitting->push_back(i);
	//pcl::visualization::CloudViewer view("Robust Mapping_fitting");
	//view.showCloud(cloud_fitting);


	cloud_in->clear();
	pcl::copyPointCloud(*cloud_fitting, *cloud_in);
	delete[] data_L;
	delete[] data_R;
	delete[] solution_L;
	delete[] solution_R;
	cloud_fitting->clear();
	cloud_projected->clear();
	cloud_left->clear();
	cloud_right->clear();
}


void
RobustMapping::FittingViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_L, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_R) {


}

void
RobustMapping::calcGPS(){

	for (int i = 0; i < this->DataSize; i++) {
		long double lat, lngd, alt;
		long double radius;
		long double x, y, z;

		lat = this->GPS_Table[1][i];
		lngd = this->GPS_Table[2][i];
		alt = this->GPS_Table[3][i];

		long double roll = drad * this->GPS_Table[6][i];
		long double pitch = drad * this->GPS_Table[5][i];
		long double yaw = -drad * this->GPS_Table[4][i];

		radius = (EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR) / std::sqrt(EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR * std::cos(lat*drad) * std::cos(lat*drad) + EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR * std::sin(lat*drad) * std::sin(lat*drad)) + alt;

		this->GPS_Table[1][i] = radius * (lngd - this->INIT_LON)*drad *cos(this->INIT_LAT*drad);  // dx 데이터
		this->GPS_Table[2][i] = radius * ((lat - this->INIT_LAT)*drad);					  // dy 데이터
		//this->GPS_Table[3][i] = alt - this->INIT_ALT;										  // dz 데이터
		this->GPS_Table[3][i] = 0;										  // dz 데이터
		this->GPS_Table[4][i] = yaw;
		this->GPS_Table[5][i] = pitch;
		this->GPS_Table[6][i] = roll;
		GPS_DATA << this->GPS_Table[0][i] << "	" << this->GPS_Table[1][i] << "		" << this->GPS_Table[2][i] << "		" << this->GPS_Table[3][i] << "		" << this->GPS_Table[4][i] << "		" << this->GPS_Table[5][i] << "		" << this->GPS_Table[6][i] << endl;
	}
	GPS_DATA.close();
}
void
RobustMapping::ICPWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloudPtr, int loop, Eigen::Matrix4f &LocalMat){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temptarget(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < this->ReferenceMap.size(); i++) {
		pcl::transformPointCloud(ReferenceMap[i].cloud, *cloud_ref, ReferenceMap[i].Transformation);

		for each(auto j in *cloud_ref) {
			temptarget->push_back(j);
		}
		cloud_ref->clear();
	}
	cout << "this->TargetFrames.size() : " << this->ReferenceMap.size() << endl;
	cout << "temptarget.size() : " << temptarget->size() << endl;
	cout << "sourceCloudPtr.size() : " << sourceCloudPtr->size() << endl;
	//_getch();


	Eigen::Matrix4f mat;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(KDTREE_SEARCH_ICP);
	//cout << "sourceCloudPtr->size()" << sourceCloudPtr->size() << endl;
	norm_est.setInputCloud(sourceCloudPtr);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*sourceCloudPtr, *points_with_normals_src);
	cout << "1 번" << endl;
	norm_est.setInputCloud(temptarget);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*temptarget, *points_with_normals_tgt);
	cout << "2 번" << endl;
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[5] = { 1.0, 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// Align
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> reg;
	// Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
	cout << "3 번" << endl;
	int cnt = 0;
	double epsilon = ICP_EPS_FROM;
	unsigned int iteration = ICP_ITR_FROM;
	for (double distance = ICP_DIST_FROM; distance >= ICP_DIST_TO; distance -= ICP_DIST_UNIT){
		reg.setInputTarget(points_with_normals_tgt);
		reg.setInputSource(points_with_normals_src);
		reg.setMaximumIterations(iteration);
		reg.setTransformationEpsilon(epsilon);
		reg.setMaxCorrespondenceDistance(distance);
		reg.align(*points_with_normals_src);
		cout << "icp 고 : " << cnt << endl;
		if (!reg.hasConverged()) {
			cout << "매칭 FAIL! Coresponding distance 늘리자" << endl;

			cnt++;
			distance = ICP_DIST_FROM + 0.5 * cnt;
			iteration = ICP_ITR_FROM;
			epsilon = ICP_EPS_FROM;
		}

		LocalMat = reg.getFinalTransformation() * LocalMat;
		cnt++;
		epsilon -= ICP_EPS_UNIT;
		iteration += ICP_ITR_UNIT;
	}

	cout << "스캔매칭 결과 mat : \n" << LocalMat << endl;
	/********** 맵 쌓기 **********/

	//this->BackStepPose = LocalMat;

	cout << "GPS Data Processing...end!" << endl;
	cout << endl;
}

void 
RobustMapping::fillVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloudPtr,
const double &leafSize, bool putPointCenter) {
	std::map<MY_POINT2D, bool> voxelGridMap_t;

	for (unsigned int i = 0; i < inputCloudPtr->size(); i++){
		int x = inputCloudPtr->points[i].x >= 0 ? (int)(inputCloudPtr->points[i].x / leafSize) : (int)(inputCloudPtr->points[i].x / leafSize - 1);
		int y = inputCloudPtr->points[i].y >= 0 ? (int)(inputCloudPtr->points[i].y / leafSize) : (int)(inputCloudPtr->points[i].y / leafSize - 1);
		//int z = inputCloudPtr->points[i].z >= 0 ? (int)(inputCloudPtr->points[i].z / leafSize) : (int)(inputCloudPtr->points[i].z / leafSize - 1);
		double z = inputCloudPtr->points[i].z / leafSize;
		if (z < -1.65 / LEAF) {

			//MY_POINT2D key = { x, y, z };
			MY_POINT2D key = { x, y };
			//map<MY_POINT2D, MAP_DATA_>::iterator iter1 = voxelGridMap_.find(key);
			//map<MY_POINT2D, bool>::iterator iter2 = find(voxelGridMap_t.begin(), voxelGridMap_t.end(), key); 
			voxelGridMap_[key].intensity = (voxelGridMap_[key].intensity * voxelGridMap_[key].scanCount + inputCloudPtr->points[i].r) / (++(voxelGridMap_[key].scanCount));
			/*if (iter1 != voxelGridMap_.end()) {
			iter1->second.z_set.push_back(z);
			}
			else*/
			voxelGridMap_[key].z_set.push_back(z);

			voxelGridMap_[key].pointIndex = i;
			//cout << "voxelGridMap_t[key] = " << voxelGridMap_t[key] << endl; //_getch(); 
			//cout << "voxelGridMap_[key].times = " << voxelGridMap_[key].times << endl;// _getch();
			if (voxelGridMap_t[key] == false){
				voxelGridMap_t[key] = true;
				voxelGridMap_[key].times++;

				if (voxelGridMap_[key].times == 1){
					//voxelGridMap_[key].pointIndex = outputCloudPtr->size();
					voxelGridMap_[key].pointIndex = outputCloudPtr->size();
					pcl::PointXYZRGB point_t;
					//point_t.intensity = inputCloudPtr->points[i].intensity;
					point_t.r = 255; point_t.g = 255; point_t.b = 255;
					//point_t.x = x; point_t.y = y; point_t.z = z;
					if (putPointCenter){
						point_t.x = ((double)x + 0.5f) * leafSize;
						point_t.y = ((double)y + 0.5f) * leafSize;
						point_t.z = ((double)z + 0.5f) * leafSize;
					}
					else{
						point_t.x = inputCloudPtr->points[i].x;
						point_t.y = inputCloudPtr->points[i].y;
						point_t.z = inputCloudPtr->points[i].z;
					}

					outputCloudPtr->push_back(point_t);
				}
			}
			if (voxelGridMap_[key].times > 1){
				voxelGridMap_[key].times++;
				pcl::PointXYZRGB point_t;
				// point_t.intensity = inputCloudPtr->points[i].intensity;
				point_t.r = 255; point_t.g = 255; point_t.b = 255;
				//point_t.x = x; point_t.y = y; point_t.z = z;
				if (putPointCenter){
					point_t.x = ((double)x + 0.5f) * leafSize;
					point_t.y = ((double)y + 0.5f) * leafSize;
					point_t.z = ((double)z + 0.5f) * leafSize;
				}
				else{
					point_t.x = inputCloudPtr->points[i].x;
					point_t.y = inputCloudPtr->points[i].y;
					point_t.z = inputCloudPtr->points[i].z;
				}

				outputCloudPtr->push_back(point_t);
			}
		}
	}	
}

void 
RobustMapping::SovelFiltering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloudPtr, map<MY_POINT2D, MAP_DATA> & minvoxel, const double & leafSize, bool OriginScale) {
	
		int MaskSobelX[3][3] = { 0, };
		int MaskSobelY[3][3] = { 0, };

		//cout << "되는중? : " << cnt++ << "\t";
		//MaskSelection(MaskSobelX, MaskSobelY, minvoxel, 0, 0); // _getch();
		int Thresh = 50 / LEAF;
		//double zMean = LIDAR_H;
		int num = 0;
		for (int i = -10 / leafSize; i < 10 / leafSize + 1; i++) {
			//cout << "되는중 i : " << i << endl;
			for (int j = -8 / leafSize; j < 8 / leafSize + 1; j++) {
				//cout << "되는중 j : " << j << "\t";
				//int cnt = 0;
				//vector<double> PointGradient;
				MaskSobelX[0][0] = -1;	MaskSobelX[0][2] = 3;
				MaskSobelX[1][0] = -2;	MaskSobelX[1][2] = 2;
				MaskSobelX[2][0] = -1;	MaskSobelX[2][2] = 1;

				MaskSobelY[0][0] = 1;	MaskSobelY[0][1] = 2;	MaskSobelY[0][2] = 1;
				MaskSobelY[2][0] = -1;	MaskSobelY[2][1] = -2;	MaskSobelY[2][2] = -1;
				double zMean = LIDAR_H / LEAF;
				double z[9], z5;

				/*MY_POINT2D key[25] = { 0, };
				for (int k = 0; k < 5; k++) {
				for (int p = 0; p < 5; p++) {
				key[k*5 + p] = { j + k - 2, i + p - 2 };
				}
				}*/
				MY_POINT2D key_t1 = { j - 1, i + 1 };
				MY_POINT2D key_t2 = { j, i + 1 };
				MY_POINT2D key_t3 = { j + 1, i + 1 };

				MY_POINT2D key_t4 = { j - 1, i };
				MY_POINT2D key_t5 = { j, i };
				MY_POINT2D key_t6 = { j + 1, i };

				MY_POINT2D key_t7 = { j - 1, i - 1 };
				MY_POINT2D key_t8 = { j, i - 1 };
				MY_POINT2D key_t9 = { j + 1, i - 1 };
				z[0] = minvoxel[key_t1].z;	z[1] = minvoxel[key_t2].z;	z[2] = minvoxel[key_t3].z;
				z[3] = minvoxel[key_t4].z;	z[4] = minvoxel[key_t5].z;	z[5] = minvoxel[key_t6].z;
				z[6] = minvoxel[key_t7].z;	z[7] = minvoxel[key_t8].z;	z[8] = minvoxel[key_t9].z;
				z5 = minvoxel[key_t5].z;
				for (int k = 0; k<9; k++) {
					if (z[k] < Thresh && z[k] > -Thresh) {
						zMean += z[k];
						num++;
					}
				}
				if (num > 0) {
					zMean = zMean / num;
					num = 0;
					//	cout << "zMean : " << zMean << endl;
				}

				if (z[0] > Thresh || z[0] < -Thresh) {
					MaskSobelX[0][0] = 0;
					MaskSobelY[0][0] = 0;
				}
				if (z[1]>Thresh || z[1] < -Thresh) {
					MaskSobelX[0][1] = 0;
					MaskSobelY[0][1] = 0;
				}
				if (z[2]>Thresh || z[2] < -Thresh) {
					MaskSobelX[0][2] = 0;
					MaskSobelY[0][2] = 0;
				}
				if (z[3]>Thresh || z[3] < -Thresh) {
					MaskSobelX[1][0] = 0;
					MaskSobelY[1][0] = 0;
				}
				if (z[4]>Thresh || z[4] < -Thresh) {
					MaskSobelX[1][1] = 0;
					MaskSobelY[1][1] = 0;
				}
				if (z[5]>Thresh || z[5] < -Thresh) {
					MaskSobelX[1][2] = 0;
					MaskSobelY[1][2] = 0;
				}
				if (z[6]>Thresh || z[6] < -Thresh) {
					MaskSobelX[2][0] = 0;
					MaskSobelY[2][0] = 0;
				}
				if (z[7]>Thresh || z[7] < -Thresh) {
					MaskSobelX[2][1] = 0;
					MaskSobelY[2][1] = 0;
				}
				if (z[8]>Thresh || z[8] < -Thresh) {
					MaskSobelX[2][2] = 0;
					MaskSobelY[2][2] = 0;
				}

				double test1 = z[0] * MaskSobelX[0][0] + z[1] * MaskSobelX[0][1] + z[2] * MaskSobelX[0][2] +
					z[3] * MaskSobelX[1][0] + z[4] * MaskSobelX[1][1] + z[5] * MaskSobelX[1][2] +
					z[6] * MaskSobelX[2][0] + z[7] * MaskSobelX[2][1] + z[8] * MaskSobelX[2][2];
				//cout << "test1 = " << test1 << endl;
				double test2 = z[0] * MaskSobelY[0][0] + z[1] * MaskSobelY[0][1] + z[2] * MaskSobelY[0][2] +
					z[3] * MaskSobelY[1][0] + z[4] * MaskSobelY[1][1] + z[5] * MaskSobelY[1][2] +
					z[6] * MaskSobelY[2][0] + z[7] * MaskSobelY[2][1] + z[8] * MaskSobelY[2][2];
				//cout << "test2 = " << test2 << endl;
				double Strength2 = fabs(test1) + fabs(test2);
				//cout << "되는중? : " << cnt++ << "\t";
				//cout << "Strength = " << Strength << endl; 
				//cout << "Strength2 = " << Strength2 << endl; //_getch();
				if (fabs(Strength2) > 140) {

					pcl::PointXYZRGB point_t;
					if (OriginScale) {
						if (z5 < Thresh && z5 > -Thresh) {
							point_t.x = ((double)key_t5.x + 0.5f) * leafSize;
							point_t.y = ((double)key_t5.y + 0.5f) * leafSize;
							point_t.z = ((double)minvoxel[key_t5].z + 0.5f) * leafSize;
							//point_t.intensity = minvoxel[key_t5].intensity;
							point_t.r = 255; point_t.g = 255; point_t.b = 255;
						}

					}
					outputCloudPtr->push_back(point_t);
				}
			}
		}
}

void 
RobustMapping::MinFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloudPtr, map<MY_POINT2D, MAP_DATA> & minvoxel, const double & leafSize, bool OriginScale) {
	std::map<MY_POINT2D, bool> voxelGridMap_t;

	for each(auto iter in voxelGridMap_) {
		MY_POINT2D tempMy;
		tempMy = iter.first;
		//if (voxelGridMap_t[tempMy] == false) {
		voxelGridMap_t[tempMy] = true;
		double Minz = *min_element(iter.second.z_set.begin(), iter.second.z_set.end());
		pcl::PointXYZRGB point_t;
		if (OriginScale){
			point_t.x = ((double)iter.first.x + 0.5f) * leafSize;
			point_t.y = ((double)iter.first.y + 0.5f) * leafSize;
			point_t.z = ((double)Minz + 0.5f) * leafSize;
			//point_t.intensity = voxelGridMap_[tempMy].intensity;
			point_t.r = 255; point_t.g = 255; point_t.b = 255;
		}
		else{
			point_t.x = iter.first.x; point_t.y = iter.first.y; point_t.z = Minz;// point_t.intensity = iter.second.intensity;
			point_t.r = 255; point_t.g = 255; point_t.b = 255;
		}
		outputCloudPtr->push_back(point_t);
		minvoxel[tempMy].pointIndex;
		minvoxel[tempMy].intensity = iter.second.intensity;
		minvoxel[tempMy].z = Minz;

		//}
	}

}

void 
RobustMapping::MaskSelection(int **MaskSobelX, int **MaskSobelY, map<MY_POINT2D, MAP_DATA> &minvoxel, int i, int j) {


}


void
RobustMapping::getInverseTransformation(Eigen::Matrix4f matrix, double &x, double &y, double &z, double &roll, double &pitch, double &yaw){
	pitch = -std::asin(matrix(2, 0));
	double cos_pitch = std::cos(yaw);
	roll = std::atan2(matrix(2, 1) / cos_pitch, matrix(2, 2) / cos_pitch);
	yaw = std::atan2(matrix(1, 0) / cos_pitch, matrix(0, 0) / cos_pitch);
	x = matrix(0, 3);
	y = matrix(1, 3);
	z = matrix(2, 3);
}
void
RobustMapping::getTransformations() {
	cout << "글로벌 좌표계 변환 행렬 구하기" << endl;

	for (int i = 0; i < this->DataSize; i++) {
		Eigen::Matrix4f matrix;
		double cos_yaw = std::cos(this->GPS_Table[4][i]);
		double sin_yaw = std::sin(this->GPS_Table[4][i]);
		double cos_pitch = std::cos(this->GPS_Table[5][i]);
		double sin_pitch = std::sin(this->GPS_Table[5][i]);
		double cos_roll = std::cos(this->GPS_Table[6][i]);
		double sin_roll = std::sin(this->GPS_Table[6][i]);
		double comb_a = cos_yaw * sin_pitch;
		double comb_b = sin_yaw * sin_pitch;
		matrix(0, 0) = cos_yaw * cos_pitch;
		matrix(0, 1) = comb_a * sin_roll - sin_yaw * cos_roll;
		matrix(0, 2) = comb_a * cos_roll + sin_yaw * sin_roll;
		matrix(1, 0) = sin_yaw * cos_pitch;
		matrix(1, 1) = comb_b * sin_roll + cos_yaw * cos_roll;
		matrix(1, 2) = comb_b * cos_roll - cos_yaw * sin_roll;
		matrix(2, 0) = -sin_pitch;
		matrix(2, 1) = cos_pitch * sin_roll;
		matrix(2, 2) = cos_pitch * cos_roll;
		matrix(3, 0) = matrix(3, 1) = matrix(3, 2) = 0;
		matrix(3, 3) = 1;

		matrix(0, 3) = this->GPS_Table[1][i];
		matrix(1, 3) = this->GPS_Table[2][i];
		matrix(2, 3) = this->GPS_Table[3][i];
		//matrix(2, 3) = 0;
		POSE_DATA tempPose;
		tempPose.frame = this->GPS_Table[0][i];
		tempPose.mat = matrix;
		this->GPS_TFM.push_back(tempPose);

	}

}

inline string
RobustMapping::PathSetting(string PATH, int num) {
	string path = PATH;
	string number;
	int num1(num);
	stringstream sst;
	sst << num1;
	sst >> number;
	string path1 = path + number + "_lidar.pcd";
	return path1;
}

//RANCAC_LINE

//void
//RobustMapping::RansacFitting_Line(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
//	cout << "Fitting 시작" << endl;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::visualization::CloudViewer vdv("Robust Mapping_fitting");
//	// Projection Process
//	// Create a set of planar coefficients with X=Y=0,Z=1
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//	coefficients->values.resize(4);
//	coefficients->values[0] = coefficients->values[1] = 0;
//	coefficients->values[2] = 1.0;
//	coefficients->values[3] = 0;
//	// Create the filtering object
//	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//	proj.setModelType(pcl::SACMODEL_PLANE);
//	proj.setInputCloud(cloud_in);
//	proj.setModelCoefficients(coefficients);
//	proj.filter(*cloud_projected);
//
//	
//	for each(auto i in *cloud_projected) {
//		if (i.x < 0) cloud_left->push_back(i);
//		else cloud_right->push_back(i);
//	}
//
//	pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_L(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB>(cloud_left));
//	std::vector<int> inliers_L;
//	Eigen::VectorXf Coefficients_L;
//	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac1(model_L, 0.3);
//	//ransac1.setDistanceThreshold (.1);																	
//	//ransac1.setProbability(0.99);
//	///ransac1.setMaxIterations(10000);
//	ransac1.computeModel();
//	ransac1.getInliers(inliers_L);
//	//cout << "inliers_L.size() : " << inliers_L.size() << endl; _getch();
//	ransac1.getModelCoefficients(Coefficients_L);
//	
//	//cout << "Coefficients : " << Coefficients << endl; _getch();
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp1_(new pcl::PointCloud<pcl::PointXYZRGB>);
//	/*	for (int i = 0; i < cloud_left->size(); i++) {
//		for each(auto j in inliers_L) {
//			if (i == j) {				
//				cloud_temp1->push_back(cloud_left->points[i]);
//			}
//		}
//	}
//	*/	
//	cout << "cloud_temp1->size() : " << cloud_temp1->size() << endl;
//	pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_left, inliers_L, *cloud_temp1);
//	for (int i = 0; i < 500; i++) {
//		double t_ = -25 + 0.1*(double)i;
//		pcl::PointXYZRGB tempPoint;
//		tempPoint.x = Coefficients_L[0] + t_*Coefficients_L[3];
//		tempPoint.y = Coefficients_L[1] + t_*Coefficients_L[4];
//		tempPoint.z = Coefficients_L[2] + t_*Coefficients_L[5];
//		cloud_projected->push_back(tempPoint);
//	}
//
//	cout << "Fitting 결과_left" << endl;
//	vdv.showCloud(cloud_left);
//	_getch();
//	//vdv.showCloud(cloud_projected);
//	//_getch();
//	vdv.showCloud(cloud_temp1);
//	_getch();
//	/*vdv.showCloud(cloud_temp1_);
//	_getch();*/
//
//
//
//	pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_R(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB>(cloud_right));
//	std::vector<int> inliers_R;
//	Eigen::VectorXf Coefficients_R;
//	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac2(model_R, 0.3);
//	//ransac2.setDistanceThreshold (.1);
//	ransac2.computeModel();
//	ransac2.getInliers(inliers_R);
//	ransac2.getModelCoefficients(Coefficients_R);
//	cout << "inliers2.size() : " << inliers_R.size() << endl;
//	//for each(auto i in inliers2) cout << i << endl;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_right, inliers_R, *cloud_temp2);
//	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	
//	for (int i = 0; i < 500; i++) {
//		double t_ = -25 + 0.1*(double)i;
//		pcl::PointXYZRGB tempPoint;
//		tempPoint.x = Coefficients_R[0] + t_*Coefficients_R[3];
//		tempPoint.y = Coefficients_R[1] + t_*Coefficients_R[4];
//		tempPoint.z = Coefficients_R[2] + t_*Coefficients_R[5];
//		cloud_projected->push_back(tempPoint);
//	}
//
//	cout << "Fitting 결과_right" << endl;	
//	vdv.showCloud(cloud_right);
//	_getch();
//	vdv.showCloud(cloud_temp2);
//	_getch();
//	vdv.showCloud(cloud_projected);
//	_getch();
//	cloud_in->clear();
//	for each(auto i in *cloud_temp1) cloud_in->push_back(i);
//	for each(auto i in *cloud_temp2) cloud_in->push_back(i);
//	vdv.showCloud(cloud_in);
//	_getch();
//}


inline int
RobustMapping::solve_cubic(float a, float b, float c, float d, float* xout) {
	static const float cos120 = -0.5f;
	static const float sin120 = 0.866025404f;
	int n = 0;
	if (fabs(d) < FLT_EPSILON_CUS)
	{
		// first solution is x = 0
		*xout = 0.0f;
		++n;
		++xout;
		// divide all terms by x, converting to quadratic equation
		d = c;
		c = b;
		b = a;
		a = 0.0f;
	}
	if (fabs(a) < FLT_EPSILON_CUS)
	{
		if (fabs(b) < FLT_EPSILON_CUS)
		{
			// linear equation
			if (fabs(c) > FLT_EPSILON_CUS)
			{
				*xout = -d / c;
				n += 1;
			}
		}
		else
		{
			// quadratic equation
			float yy = c*c - 4 * b*d;
			if (yy >= 0)
			{
				float inv2b = 1 / (2 * b);
				float y = sqrt(yy);
				xout[0] = (-c + y) * inv2b;
				xout[1] = (-c - y) * inv2b;
				n += 2;
			}
		}
	}
	else
	{
		// cubic equation
		float inva = 1 / a;
		float invaa = inva*inva;
		float bb = b*b;
		float bover3a = b*(1 / 3.0f)*inva;
		float p = (3 * a*c - bb)*(1 / 3.0f)*invaa;
		float halfq = (2 * bb*b - 9 * a*b*c + 27 * a*a*d)*(0.5f / 27)*invaa*inva;
		float yy = p*p*p / 27 + halfq*halfq;
		if (yy > FLT_EPSILON_CUS)
		{
			// sqrt is positive: one real solution
			float y = sqrt(yy);
			float uuu = -halfq + y;
			float vvv = -halfq - y;
			float www = fabs(uuu) > fabs(vvv) ? uuu : vvv;
			float w = (www < 0) ? -pow(fabs(www), 1 / 3.0f) : pow(www, 1 / 3.0f);
			*xout = w - p / (3 * w) - bover3a;
			n = 1;
		}
		else if (yy < -FLT_EPSILON_CUS)
		{
			// sqrt is negative: three real solutions
			float x = -halfq;
			float y = sqrt(-yy);
			float theta;
			float r;
			float ux;
			float uyi;
			// convert to polar form
			if (fabs(x) > FLT_EPSILON_CUS)
			{
				theta = (x > 0) ? atan(y / x) : (atan(y / x) + 3.14159625f);
				r = sqrt(x*x - yy);
			}
			else
			{
				// vertical line
				theta = 3.14159625f / 2;
				r = y;
			}
			// calc cube root
			theta /= 3.0f;
			r = pow(r, 1 / 3.0f);
			// convert to complex coordinate
			ux = cos(theta)*r;
			uyi = sin(theta)*r;
			// first solution
			xout[0] = ux + ux - bover3a;
			// second solution, rotate +120 degrees
			xout[1] = 2 * (ux*cos120 - uyi*sin120) - bover3a;
			// third solution, rotate -120 degrees
			xout[2] = 2 * (ux*cos120 + uyi*sin120) - bover3a;
			n = 3;
		}
		else
		{
			// sqrt is zero: two real solutions
			float www = -halfq;
			float w = (www < 0) ? -pow(fabs(www), 1 / 3.0f) : pow(www, 1 / 3.0f);
			// first solution           
			xout[0] = w + w - bover3a;
			// second solution, rotate +120 degrees
			xout[1] = 2 * w*cos120 - bover3a;
			n = 2;
		}
	}
	return n;
}
inline int
RobustMapping::solve_linear(float a, float b, float* xout) {
	xout[0] = -b / a;
	return 1;
}

inline void
RobustMapping::SequentialScreenCapture(int NUM) {
	stringstream sst;
	string name;
	sst << NUM;
	sst >> name;
	string filename = "../Results/result_" + name + ".jpg";
	const char *cstr = filename.c_str();
	screenCapturePart(0, 0, 1680, 1080, cstr);
}