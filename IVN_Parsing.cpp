#include "IVN_Parsing.h"
#include <stdlib.h>
#include <conio.h>
void
IVN_Parsing::GetEncoderPose_Table(double **table) {
	
	ifstream IVNData;
	IVNData.open(this->GLOBAL_PATH_POSE + this->OUTPUT_POSE_DATA);
	int cnt = 0;
	int cnt2 = 0;
	if (!IVNData.good()){ exit(-1); }

	while (!IVNData.eof())
	{
		//모든 라인을 읽어서 메모리에 넣는다.
		char *buf = new char[MAX_CHARS_PER_LINE];
		IVNData.getline(buf, MAX_CHARS_PER_LINE);

		// 블랭크를단위로 토큰을 파싱한다
		int n = 0; // loop index
		cnt++;
		//buf에 있는 토큰들의 메모리주소를 어레이에 저장한다.
		const char* token[MAX_TOKENS_PER_LINE] = {}; // 0으로 초기화한다.z

		//라인을 파싱함.
		token[0] = strtok(buf, DELIMITER2); // 첫번째 token
		if (token[0]) {

			for (n = 1; n < MAX_TOKENS_PER_LINE; n++){
				token[n] = strtok(NULL, DELIMITER2); // 연속적인 토큰
				//cout << "토큰 : " << token[n	-1] << endl;
				if (!token[n]){
					this->Column = n;
					//cout << "n : " << n << endl;
					break;
					//토큰이 더이상 없을때 끝낸다.
				}
				cnt2++;
				//Resultfile << "}, " << endl;
			}

		}
		if (cnt == 1) {
			//for (int i = 0; i < n; i++) {
			//	cout << token[i] << " ";
			//}
			//cout << endl;
			//_getch();
		} // Do Nothing
		else {
			for (int i = 0; i < n; i++) {
				this->EncoderPoseData[i][cnt - 2] = stod(token[i]);
				//cout << "IVN CTNT : " << cnt - 2 << endl;
			//	cout << this->EncoderPoseData[i][cnt - 2] << " ";
			}
			/*cout << endl;
			_getch();*/
		}

		delete buf;
	}

	for (int i = 0; i < Column; i++){
		for (int j = 0; j < Row; j++) {
			table[i][j] = this->EncoderPoseData[i][j];// -this->EncoderPoseData[i][START_FRAME];
			//cout << table[i][j] << endl;
		}
	//	_getch();
	}
	
}

void 
IVN_Parsing::DataFromEncoder(const int Datasize)
{
	
	ifstream inputfile;
	inputfile.open(GLOBAL_PATH_POSE + INPUT_IVN_FILE);
	ofstream Output_Pose;
	Output_Pose.open(GLOBAL_PATH_POSE + OUTPUT_POSE_DATA);


	int frame_num;
	char str[256];


	CloudRGB::Ptr Map(new pcl::PointCloud<PointRGB>);
	

	struct tag_edge_info{

		int cur_pose_frame;
		int pre_pose_frame;
		double dff_x;
		double dff_y;
		double dff_heading;

	};
	std::vector<tag_edge_info> edge_info;

	if (inputfile.is_open())
	{
		for (int i = 0; i < 15; i++) inputfile >> str;

		double pre_pose_x;
		double pre_pose_y;
		double pre_pose_heading;
		int pre_frame_num;
		int cur_frame_num;

		double df_x;
		double df_y;
		double df_heading;
		double yaw_rate;


		for (int j = 0; j <Datasize; j++)
		{
			int cur_pul_FL;
			int cur_pul_FR;
			int cur_pul_RL;
			int cur_pul_RR;

			int pre_pul_FL;
			int pre_pul_FR;
			int pre_pul_RL;
			int pre_pul_RR;
			double pre_yawrate;

			int diff_pul_FL = 0;
			int diff_pul_FR = 0;
			int diff_pul_RL = 0;
			int diff_pul_RR = 0;

			inputfile >> str;
			frame_num = atoi(str);
			inputfile >> str; inputfile >> str;
			yaw_rate = std::stod(str);
			for (int i = 0; i < 6; i++) inputfile >> str;

			inputfile >> str;
			cur_pul_FL = std::stod(str);
			inputfile >> str;
			cur_pul_FR = std::stod(str);
			inputfile >> str;
			cur_pul_RL = std::stod(str);
			inputfile >> str;
			cur_pul_RR = std::stod(str);

			if (j == 0) //initial value setting.
			{
				pre_pul_FL = cur_pul_FL;
				pre_pul_FR = cur_pul_FR;
				pre_pul_RL = cur_pul_RL;
				pre_pul_RR = cur_pul_RR;

				pre_pose_x = 0;
				pre_pose_y = 0;
				pre_pose_heading = -4.84*degree_to_radian;
				pre_frame_num = frame_num;
				pre_yawrate = yaw_rate;

				inputfile >> str; inputfile >> str;

				//cout << pre_frame_num << " " << pre_pose_x << " " << pre_pose_y << " " << pre_pose_heading << " " << pre_yawrate << endl;
				//Output_Pose << "VERTEX2 " << pre_frame_num << " " << pre_pose_x << " " << pre_pose_y << " " << pre_pose_heading << endl;
				Output_Pose << "frame_num" << " " << "pose_x" << " " << "pose_y" << " " << "pose_heading" << endl;
				Output_Pose << pre_frame_num << " " << pre_pose_x << " " << pre_pose_y << " " << pre_pose_heading << endl;
			}
			else
			{

				diff_pul_FL = (cur_pul_FL - pre_pul_FL + resolution) % resolution;
				diff_pul_FR = (cur_pul_FR - pre_pul_FR + resolution) % resolution;
				diff_pul_RL = (cur_pul_RL - pre_pul_RL + resolution) % resolution;
				diff_pul_RR = (cur_pul_RR - pre_pul_RR + resolution) % resolution;

				pre_pul_FL = cur_pul_FL;
				pre_pul_FR = cur_pul_FR;
				pre_pul_RL = cur_pul_RL;
				pre_pul_RR = cur_pul_RR;

				inputfile >> str; inputfile >> str;

				double cur_pose_x;
				double cur_pose_y;
				double cur_pose_heading;
				double check_heading_rate;
				cur_frame_num = frame_num;

				IVN_Parsing::getPose_from_encoder(diff_pul_RL, diff_pul_RR, pre_pose_x, pre_pose_y, pre_pose_heading, &cur_pose_x, &cur_pose_y, &cur_pose_heading, &check_heading_rate);


				df_x = cur_pose_x - pre_pose_x;
				df_y = cur_pose_y - pre_pose_y;
				df_heading = cur_pose_heading - pre_pose_heading;
				tag_edge_info edge;
				edge.cur_pose_frame = frame_num;
				edge.pre_pose_frame = pre_frame_num;
				edge.dff_x = df_x;
				edge.dff_y = df_y;
				edge.dff_heading = df_heading;
				edge_info.push_back(edge);


				//Output_Pose << "VERTEX2 " << frame_num << " " << cur_pose_x << " " << cur_pose_y << " " << cur_pose_heading << endl;
				Output_Pose << frame_num << "	" << cur_pose_x << "	" << cur_pose_y << "	" << cur_pose_heading << endl;
				//cout << frame_num << " " << cur_pose_x << " " << cur_pose_y << " " << cur_pose_heading << " " << cur_pose_heading / degree_to_radian << endl;
				pre_pose_heading = cur_pose_heading;
				pre_pose_x = cur_pose_x;
				pre_pose_y = cur_pose_y;
				pre_frame_num = cur_frame_num;
				pre_yawrate = yaw_rate; 
				
				PointRGB tempPoint;
				tempPoint.x = cur_pose_x;
				tempPoint.y = cur_pose_y;
				tempPoint.z = 0;
				if (j < 500) {
					tempPoint.r = 255;
					tempPoint.g = 255;
					tempPoint.b = 255;
				}
				else if (j >= 500) {
					tempPoint.r = 255;
					tempPoint.g = 255;
					tempPoint.b = 0;
				}
				

				Map->push_back(tempPoint);

			}
		}

	}
	/*pcl::visualization::CloudViewer viewer1("Odometry test");
	viewer1.showCloud(Map);
	_getch();*/
	inputfile.close();
	Output_Pose.close();

}

void
IVN_Parsing::getPose_from_encoder(int RL, int RR, double pre_pose_x, double pre_pose_y, double pre_pose_heading, double* cur_pose_x, double* cur_pose_y, double* cur_pose_heading, double* heading_rate)
{
	double distance_RL = encoder_resolution*RL;
	double distance_RR = encoder_resolution*RR;
	double center_distance = (distance_RL + distance_RR) / 2;
	double delta_pose_x;
	double delta_pose_y;

	double delta_heading = (-distance_RR + distance_RL) / distanceBetweenWheel;

	if (RL == RR)
	{
		delta_pose_x = center_distance*cos(pre_pose_heading);
		delta_pose_y = center_distance*sin(pre_pose_heading);


		*cur_pose_x = pre_pose_x + delta_pose_x;
		*cur_pose_y = pre_pose_y + delta_pose_y;
		*cur_pose_heading = pre_pose_heading;
	}

	else
	{
		*cur_pose_heading = pre_pose_heading + delta_heading;

		if (*cur_pose_heading > PI)
			*cur_pose_heading -= 2 * PI;
		else if (*cur_pose_heading <= -PI)
			*cur_pose_heading += 2 * PI;

		delta_pose_x = center_distance*cos(*cur_pose_heading);
		delta_pose_y = center_distance*sin(*cur_pose_heading);
		*heading_rate = delta_heading;
		*cur_pose_x = pre_pose_x + delta_pose_x;
		*cur_pose_y = pre_pose_y + delta_pose_y;



	}

}
