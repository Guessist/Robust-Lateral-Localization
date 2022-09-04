#ifndef _RANSAC_CUSTOM_H_
#define _RANSAC_CUSTOM_H_
#include <stdlib.h>
//#include <Eigen\src\Core\EigenBase.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <conio.h>

#define	TwoPi  6.28318530717958648
const double eps = 1e-14;

using namespace std;

struct pPoint {
	double x, y;
};
struct pLine_ {
	double mx, my;
	double sx, sy;
	//double a, b, c, d;
};
struct pLine {
	//double mx, my;
	//double sx, sy;
	double a, b, c, d;
};

class RANSAC_custom {
private:
	
	const int no_samples;
	float Probability;
	

	//double max_cost;

public:

	RANSAC_custom() : Probability(0.99), no_samples(4) {}
	~RANSAC_custom() {
		//delete[] inliers;
	} 

	bool FindSamples(pPoint *samples, int no_samples, pPoint *data);
	void GetSamples(pPoint *samples, int no_samples, pPoint *data, int no_data);

	void ModelEstimation_cubic(pPoint samples[], int no_samples, pLine &model);
	double GetDistance_cubic(pLine &line, pPoint &x);
	double PolyFitting_cubic(pPoint *data, int no_data, pLine &model, double distance_threshold);
	double ModelVerification_cubic(pPoint *inliers, int *no_inliers, pLine &estimated_model, pPoint *data, int no_data, double distance_threshold);

	void ModelEstimation_line(pPoint samples[], int no_samples, pLine_ &model);	
	double GetDistance_line(pLine_ &line, pPoint &x);	
	double PolyFitting_line(pPoint *data, int no_data, pLine_ &model, double distance_threshold);	
	double ModelVerification_line(pPoint *inliers, int *no_inliers, pLine_ &estimated_model, pPoint *data, int no_data, double distance_threshold);

	int FindSolution(pLine model, double *x);
};


double ransac_line_fitting(pPoint *data, int no_data, pLine &model, double distance_threshold);

#endif _RANSAC_CUSTOM_H_