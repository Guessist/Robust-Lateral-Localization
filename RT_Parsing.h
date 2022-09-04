#ifndef _RT_PARSING_H_
#define _RT_PARSING_H_
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl\point_types.h>
#include "Parameter.h"
using namespace std;
//////////////////////////////////////// RT_Parsing Parameter ////////////////////////////////////////
const int MAX_CHARS_PER_LINE = 512000;
const int MAX_TOKENS_PER_LINE = 51200;
const char* const DELIMITER = "	";

class RT_Parsing {
public:
	double **TotalTable;
	vector<string> Lable;
	double **GPSData;
	string RTdataPath;
	string Filename;

private:
	int Column;
	int Row;
	int ReferenceInfo = 7;

public:
	RT_Parsing() {		
		
	}
	RT_Parsing(int column, int row) :Column(column), Row(row)  {
		RTdataPath = GLOBAL_PATH;
		std::cout << "RT Data Parsing Processing..." << std::endl;
		cout << endl;
	}

	~RT_Parsing() { 
		
		cout << "멤버변수 메모리해제 "<< endl;
	};
		
	void GetTotalTable(int &size, int &ref);		// RTK data parsing
	void GetGPS_Table(double **table);				// Get pose data

};


#endif _RT_PARSING_H_