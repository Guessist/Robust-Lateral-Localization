#include "RobustMapping.h"
#include "RT_Parsing.h"
int main(void)
{
	
	/*********************** RT Data Parsing ***********************/
	int DataSize = 0;																// ����� ��ü ������ ������
	int RefInfo = 0;																// ���� ����� ���� �ʿ��� ������ ������ ����
	RT_Parsing RT_DataHandling(0, 0);
	RT_DataHandling.GetTotalTable(DataSize, RefInfo);								// RTK ������ parsing 
	double **GPS_Table;
	cout << "DataSize : " << DataSize << ", RefInfo : " << RefInfo << endl;
	GPS_Table = new double*[RefInfo];
	for (int i = 0; i < RefInfo; i++) {
		GPS_Table[i] = new double[DataSize];
		for (int j = 0; j < DataSize; j++) GPS_Table[i][j] = { 0, };
	}
	RT_DataHandling.GetGPS_Table(GPS_Table);

	RobustMapping *Mapping = new RobustMapping(GPS_Table, DataSize);
	Mapping->calcGPS();
	Mapping->getTransformations();
	Mapping->LateralPositioning();
	
	_getch();
	delete Mapping;
	for (int i = 0; i < RefInfo; i++) delete[] GPS_Table[i];
	delete[] GPS_Table;
	
	
}