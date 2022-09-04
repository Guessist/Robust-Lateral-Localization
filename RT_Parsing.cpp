#include "RT_Parsing.h"
#include <stdlib.h>
#include <conio.h>

void
RT_Parsing::GetTotalTable(int &size, int &ref) {
	this->Filename = "RT_DATA_new.txt";
	ifstream RtData;
	RtData.open(this->RTdataPath + this->Filename);
	int cnt = 0;
	int cnt2 = 0;
	if (!RtData.good()){ exit(-1); }
		

	while (!RtData.eof())
	{
		//��� ������ �о �޸𸮿� �ִ´�.
		char *buf = new char[MAX_CHARS_PER_LINE];
		RtData.getline(buf, MAX_CHARS_PER_LINE);

		// ��ũ�������� ��ū�� �Ľ��Ѵ�
		int n = 0; // loop index
		cnt++;
		//buf�� �ִ� ��ū���� �޸��ּҸ� ��̿� �����Ѵ�.
		const char* token[MAX_TOKENS_PER_LINE] = {}; // 0���� �ʱ�ȭ�Ѵ�.z

		//������ �Ľ���.
		token[0] = strtok(buf, DELIMITER); // ù��° token
		if (token[0]) {

			for (n = 1; n < MAX_TOKENS_PER_LINE; n++){
				token[n] = strtok(NULL, DELIMITER); // �������� ��ū
				//cout << "��ū : " << token[n	-1] << endl;
				if (!token[n]){
					this->Column = n;
					//cout << "n : " << n << endl;
					break;
					//��ū�� ���̻� ������ ������.
				}
				cnt2++;
				//Resultfile << "}, " << endl;

			}
		}

		delete buf;
	}


	this->Row = cnt-2;
	TotalTable = new double*[this->Row];
	for (int i = 0; i < this->Row; i++) {
		TotalTable[i] = new double[this->Column];
		for (int j = 0; j < this->Column; j++) TotalTable[i][j] = { 0, };
	}
	/*
	for (int i = 0; i < this->Column; i++) {
		this->Lable[i] = "0";
	}*/
	cout << "column" << Column << endl;
	cout << "row " << this->Row << endl;
	
	ifstream RtData_;
	RtData_.open(this->RTdataPath + this->Filename);
	cnt = 0;

	if (!RtData_.good()) cout << "������ ��� ��"<<endl;
		//throw runtimeErr; // ������ ������ ������.


	while (!RtData_.eof())
	{
		//��� ������ �о �޸𸮿� �ִ´�.
		char *buf = new char[MAX_CHARS_PER_LINE];
		RtData_.getline(buf, MAX_CHARS_PER_LINE);

		// ��ũ�������� ��ū�� �Ľ��Ѵ�
		int n = 0; // loop index
		cnt++;
		//buf�� �ִ� ��ū���� �޸��ּҸ� ��̿� �����Ѵ�.
		const char* token[MAX_TOKENS_PER_LINE] = {}; // 0���� �ʱ�ȭ�Ѵ�.z

		//������ �Ľ���.
		token[0] = strtok(buf, DELIMITER); // ù��° token
		if (token[0]) {

			for (n = 1; n < MAX_TOKENS_PER_LINE; n++){
				token[n] = strtok(0, DELIMITER); // �������� ��ū

				if (!token[n]){

					break;
					//��ū�� ���̻� ������ ������.
				}
			}
		}
		
		if (cnt == 1) {
			for (int i = 0; i < n; i++) {								
				this->Lable.push_back(token[i]);
				//cout << "Lable : " << Lable[i]<<endl;				
			}
		}		
		else {
			for (int i = 0; i < n; i++) {
				float teampToken = atof(token[i]);
				//cout << "cnt : " << cnt << ", teampToken : " << teampToken << " " << endl;
				this->TotalTable[cnt - 2][i] = teampToken;
			}
		}

		delete buf;
	}

	//cout << "cnt : " << cnt << ", this->Row : " << this->Row<< " " << endl;
	/*
	cout << "this->TotalTable[i][j]";
	for (int i = 0; i < this->Row; i++) {
		for (int j = 0; j < this->Column; j++) {

			cout << this->TotalTable[i][j] << " ";
		}
		cout << endl;
	}*/

	size = this->Row;
	ref = this->ReferenceInfo;
}

void
RT_Parsing::GetGPS_Table(double **table) {
	
	this->GPSData = new double*[this->ReferenceInfo];
	for (int i = 0; i < this->ReferenceInfo; i++) {
		this->GPSData[i] = new double[this->Row];
		for (int j = 0; j < this->Row < j; j++) {
			this->GPSData[i][j] = { 0, };
		}
	}
	int latIdx = 0;
	int longIdx = 0;
	int altIdx = 0;
	int headIdx = 0;
	int pitIdx = 0;
	int rolIdx = 0;
	vector<string>::iterator iter;
	iter = find(this->Lable.begin(), this->Lable.end(), "RT_lat");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) latIdx++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RT_long");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) longIdx++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RT_speed(km/h)");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) altIdx++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RT_yaw");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) headIdx++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RT_pitch");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) pitIdx++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RT_roll");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) rolIdx++;
	//cout << latIdx << " " << longIdx << " " << altIdx << " " << headIdx << " " << pitIdx << " " << rolIdx << endl;
		
	for (int i = 0; i < this->Row; i++) {
		this->GPSData[0][i] = i;  //frame ��ȣ
		this->GPSData[1][i] = this->TotalTable[i][latIdx];
		this->GPSData[2][i] = this->TotalTable[i][longIdx];
		this->GPSData[3][i] = this->TotalTable[i][altIdx];
		this->GPSData[3][i] = 0;
		this->GPSData[4][i] = this->TotalTable[i][headIdx];
		this->GPSData[5][i] = this->TotalTable[i][pitIdx];
		this->GPSData[6][i] = this->TotalTable[i][rolIdx];

	}
	

	for (int i = 0; i < this->ReferenceInfo; i++) {  // for check
		for (int j = 0; j < this->Row; j++) {
			table[i][j] = this->GPSData[i][j];
			}
		
	}
	
	//
	//for (int i = 0; i < this->ReferenceInfo; i++) {  // for check
	//	for (int j = 0; j < this->Row; j++) {
	//		cout<< table[i][j]<<" ";
	//	}
	//	_getch();
	//	cout << endl;
	//}
	//
	for (int i = 0; i < Row; i++) delete[] TotalTable[i];
	delete[] TotalTable;
	std::cout << "RT Data Parsing Processing...end!" << std::endl;
	cout << endl;
}

