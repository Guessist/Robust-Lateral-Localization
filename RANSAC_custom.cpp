#include "RANSAC_custom.h"
#include <math.h>

double
RANSAC_custom::PolyFitting_cubic(pPoint *data, int no_data, pLine &model, double distance_threshold)
{
	//const int no_samples = 4;  // �����̴ϱ� �� ���� �̴´� -> ������ ���� : �𵨿� ���� �����ϴ� ���� ������ �ٸ�.
//	int cntt = 1;
	if (no_data < no_samples) {
		return 0.;
	}

//	cout << "THIS? " << cntt++ << endl;
	pPoint *samples = new pPoint[no_samples];


	//cout << "THIS? " << cntt++ << endl;
	int no_inliers = 0;
	//inliers = new pPoint[no_data];
	//cout << "no_data " << no_data << endl;
	pPoint *inliers = new pPoint[no_data];		/// �̰� ����@!!!!

	//cout << "THIS? " << cntt++ << endl;
	pLine estimated_model;
	double max_cost = 0.;


	int max_iteration = (int)(1 + log(1. - this->Probability) / log(1. - pow(0.5, no_samples)));
//	cout << "THIS? " << cntt++ << endl;
	for (int i = 0; i<max_iteration; i++) {
		// 1. hypothesis

		// ���� �����Ϳ��� ���Ƿ� N���� ���� �����͸� ����.
		RANSAC_custom::GetSamples(samples, no_samples, data, no_data);
		
		// �� �����͸� �������� �����ͷ� ���� �� �Ķ���͸� �����Ѵ�.
		RANSAC_custom::ModelEstimation_cubic(samples, no_samples, estimated_model);
		/*cout << "����? (�𵨿���Ƽ���̼ǳ�)" << endl;
		_getch();*/
		// 2. Verification

		// ���� �����Ͱ� ������ �𵨿� �� �´��� �˻��Ѵ�.
		double cost = RANSAC_custom::ModelVerification_cubic(inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

		// ���� ������ ���� �� �´´ٸ�, �� �𵨿� ���� ��ȿ�� �����ͷ� ���ο� ���� ���Ѵ�.
		if (max_cost < cost) {
			max_cost = cost;

			RANSAC_custom::ModelEstimation_cubic(inliers, no_inliers, model);
		}
	}
	//cout << "THIS? " << cntt++ << endl;
	delete[] samples;
	delete[] inliers;

	return max_cost;
}

bool 
RANSAC_custom::FindSamples(pPoint *samples, int no_samples, pPoint *data)
{
	for (int i = 0; i<no_samples; ++i) {
		//�̹� ���÷μ� sample �迭�� �� ����Ʈ�� ���ؼ� �˻��Ѵ�.
		if (samples[i].x == data->x && samples[i].y == data->y) {  
			return true;
		}
	}
	return false;
}

void 
RANSAC_custom::GetSamples(pPoint *samples, int no_samples, pPoint *data, int no_data)
{
	// �����Ϳ��� �ߺ����� �ʰ� N���� ������ ������ ä���Ѵ�.
	for (int i = 0; i<no_samples;) {
		int j = rand() % no_data;
		// �������� ���� ���� �߿��� samples�� �̹� �ִ��� ������ �˻��ϰ�, ���ٸ� �߰��Ѵ�. (Robustness of sampling)
		if (!RANSAC_custom::FindSamples(samples, i, &data[j])) {  
			samples[i] = data[j];
			++i;
		}
	};
}

void
RANSAC_custom::ModelEstimation_cubic(pPoint samples[], int no_samples, pLine &model)
{
	// Least Square�� �� �Ķ���� ã�� : 3��
	// f(x) = |y - ax^3 + bx^2 + cx + d|^2
	/*Eigen::MatrixXf Jacobian(4, 4);	
	Jacobian(0, 0) = 0; Jacobian(0, 1) = 0; Jacobian(0, 2) = 0; Jacobian(0, 3) = 0;
	Jacobian(1, 0) = 0; Jacobian(1, 1) = 0; Jacobian(1, 2) = 0; Jacobian(1, 3) = 0;
	Jacobian(2, 0) = 0; Jacobian(2, 1) = 0; Jacobian(2, 2) = 0; Jacobian(2, 3) = 0;
	Jacobian(3, 0) = 0; Jacobian(3, 1) = 0; Jacobian(3, 2) = 0; Jacobian(3, 3) = 0;

	Eigen::VectorXf RightVal(4);
	RightVal(0) = 0; RightVal(1) = 0; RightVal(2) = 0; RightVal(3) = 0;

	
	for (int i = 0; i < no_samples; i++) {

		Jacobian(0, 0) += pow(samples[i].x, 6); Jacobian(0, 1) += pow(samples[i].x, 5); 
		Jacobian(0, 2) += pow(samples[i].x, 4); Jacobian(0, 3) += pow(samples[i].x, 3);

		Jacobian(1, 0) += pow(samples[i].x, 5); Jacobian(1, 2) += pow(samples[i].x, 4);
		Jacobian(1, 2) += pow(samples[i].x, 3); Jacobian(1, 3) += pow(samples[i].x, 2);

		Jacobian(2, 0) += pow(samples[i].x, 4); Jacobian(2, 3) += pow(samples[i].x, 3);
		Jacobian(2, 2) += pow(samples[i].x, 2); Jacobian(2, 3) += samples[i].x;

		Jacobian(3, 0) += pow(samples[i].x, 3); Jacobian(3, 1) += pow(samples[i].x, 2);
		Jacobian(3, 2) += samples[i].x; Jacobian(3, 3) += 1;

		RightVal(0, 0) += RightVal(0, 0) + pow(samples[i].x, 3)*samples[i].y;
		RightVal(1, 0) += RightVal(1, 0) + pow(samples[i].x, 2)*samples[i].y;
		RightVal(2, 0) += RightVal(2, 0) + pow(samples[i].x, 1)*samples[i].y;
		RightVal(3, 0) += RightVal(3, 0) + pow(samples[i].x, 0)*samples[i].y;

		

	}*/
	

	Eigen::MatrixXf Jacobian(no_samples, 4);	
	Eigen::VectorXf RightVal(no_samples);

	for (int i = 0; i < no_samples; i++) {		
		Jacobian(i, 0) = pow(samples[i].x, 3);
		Jacobian(i, 1) = pow(samples[i].x, 2);
		Jacobian(i, 2) = pow(samples[i].x, 1);
		Jacobian(i, 3) = pow(samples[i].x, 0);
		RightVal(i) = samples[i].y;
	}

	Eigen::VectorXf Parameter(4);
	Eigen::MatrixXf JacoMul;
	JacoMul = Jacobian.transpose()*Jacobian;
	Parameter = JacoMul.inverse()*Jacobian.transpose()*RightVal;

	
	model.a = Parameter(0, 0);
	model.b = Parameter(1, 0);
	model.c = Parameter(2, 0);
	model.d = Parameter(3, 0);	
	
}
double
RANSAC_custom::GetDistance_cubic(pLine &line, pPoint &x)
{
	// �� ��(x)�κ��� ����(line)�� ���� ������ ����(distance)�� ����Ѵ�.

	//return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx) / sqrt(line.mx*line.mx + line.my*line.my);
	return fabs(line.a*pow(x.x, 3) + line.b*pow(x.x, 2) + line.c*x.x + line.d - x.y);
}

double
RANSAC_custom::ModelVerification_cubic(pPoint *inliers, int *no_inliers, pLine &estimated_model, pPoint *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;

	double cost = 0.;

	for (int i = 0; i<no_data; i++){
		// ������ ���� ������ ���̸� ����Ѵ�.
		double distance = RANSAC_custom::GetDistance_cubic(estimated_model, data[i]);  // 3�� ��
		//double distance = RANSAC_custom::GetDistance_line(estimated_model, data[i]);	// 1�� ��

		// ������ �𵨿��� ��ȿ�� �������� ���, ��ȿ�� ������ ���տ� ���Ѵ�.
		if (distance < distance_threshold) {
			cost += 1.;

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}

	return cost;
}

double
RANSAC_custom::PolyFitting_line(pPoint *data, int no_data, pLine_ &model, double distance_threshold)
{
	//const int no_samples = 4;  // �����̴ϱ� �� ���� �̴´� -> ������ ���� : �𵨿� ���� �����ϴ� ���� ������ �ٸ�.
	//	int cntt = 1;
	if (no_data < no_samples) {
		return 0.;
	}

	//	cout << "THIS? " << cntt++ << endl;
	pPoint *samples = new pPoint[no_samples];


	//cout << "THIS? " << cntt++ << endl;
	int no_inliers = 0;
	//inliers = new pPoint[no_data];
	//cout << "no_data " << no_data << endl;
	pPoint *inliers = new pPoint[no_data];		/// �̰� ����@!!!!

	//cout << "THIS? " << cntt++ << endl;
	pLine_ estimated_model;
	double max_cost = 0.;


	int max_iteration = (int)(1 + log(1. - this->Probability) / log(1. - pow(0.5, no_samples)));
	//	cout << "THIS? " << cntt++ << endl;
	for (int i = 0; i<max_iteration; i++) {
		// 1. hypothesis

		// ���� �����Ϳ��� ���Ƿ� N���� ���� �����͸� ����.
		RANSAC_custom::GetSamples(samples, no_samples, data, no_data);

		// �� �����͸� �������� �����ͷ� ���� �� �Ķ���͸� �����Ѵ�.
		RANSAC_custom::ModelEstimation_line(samples, no_samples, estimated_model);
		/*cout << "����? (�𵨿���Ƽ���̼ǳ�)" << endl;
		_getch();*/
		// 2. Verification

		// ���� �����Ͱ� ������ �𵨿� �� �´��� �˻��Ѵ�.
		double cost = RANSAC_custom::ModelVerification_line(inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

		// ���� ������ ���� �� �´´ٸ�, �� �𵨿� ���� ��ȿ�� �����ͷ� ���ο� ���� ���Ѵ�.
		if (max_cost < cost) {
			max_cost = cost;

			RANSAC_custom::ModelEstimation_line(inliers, no_inliers, model);
		}
	}
	//cout << "THIS? " << cntt++ << endl;
	delete[] samples;
	delete[] inliers;

	return max_cost;
}
void
RANSAC_custom::ModelEstimation_line(pPoint samples[], int no_samples, pLine_ &model)
{
	
	// PCA ������� ���� ���� �Ķ���͸� �����Ѵ�.

	double sx = 0, sy = 0;
	double sxx = 0, syy = 0;
	double sxy = 0, sw = 0;

	for (int i = 0; i<no_samples; ++i)
	{
		double &x = samples[i].x;
		double &y = samples[i].y;

		sx += x;	// ������ x ��
		sy += y;	// ������ y ��
		sxx += x*x;  // ������ x ������� ��
		sxy += x*y;  // ������ xy ��
		syy += y*y; // ������ y ������� ��
		sw += 1; // ������ �� ����
	}

	//variance;
	double vxx = (sxx - sx*sx / sw) / sw;
	double vxy = (sxy - sx*sy / sw) / sw;
	double vyy = (syy - sy*sy / sw) / sw;

	//principal axis		// PCA�� covariance�� ������� ������ ����Ѵ�.
	double theta = atan2(2 * vxy, vxx - vyy) / 2;

	model.mx = cos(theta);
	model.my = sin(theta);

	//center of mass(xc, yc)
	model.sx = sx / sw;
	model.sy = sy / sw;

	//������ ������: sin(theta)*(x - sx) = cos(theta)*(y - sy);

}
double
RANSAC_custom::GetDistance_line(pLine_ &line, pPoint &x)
{
	// �� ��(x)�κ��� ����(line)�� ���� ������ ����(distance)�� ����Ѵ�.

	return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx) / sqrt(line.mx*line.mx + line.my*line.my);
	//return fabs(line.a*pow(x.x, 3) + line.b*pow(x.x, 2) + line.c*x.x + line.d - x.y);
}

double
RANSAC_custom::ModelVerification_line(pPoint *inliers, int *no_inliers, pLine_ &estimated_model, pPoint *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;

	double cost = 0.;

	for (int i = 0; i<no_data; i++){
		// ������ ���� ������ ���̸� ����Ѵ�.
		//double distance = RANSAC_custom::GetDistance_cubic(estimated_model, data[i]);  // 3�� ��
		double distance = RANSAC_custom::GetDistance_line(estimated_model, data[i]);	// 1�� ��

		// ������ �𵨿��� ��ȿ�� �������� ���, ��ȿ�� ������ ���տ� ���Ѵ�.
		if (distance < distance_threshold) {
			cost += 1.;

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}

	return cost;
}


int
RANSAC_custom::FindSolution(pLine model, double *x) {

	
	double a2 = model.a*model.a;
	double q = (a2 - 3 * model.b) / 9;
	double r = (model.a*(2 * a2 - 9 * model.b) + 27 * model.c) / 54;
		double r2 = r*r;
		double q3 = q*q*q;
		double A, B;
		if (r2<q3) {
			double t = r / sqrt(q3);
			if (t<-1) t = -1;
			if (t> 1) t = 1;
			t = acos(t);
			model.a /= 3; q = -2 * sqrt(q);
			x[0] = q*cos(t / 3) - model.a;
			x[1] = q*cos((t + TwoPi) / 3) - model.a;
			x[2] = q*cos((t - TwoPi) / 3) - model.a;
			return(3);
		}
		else {
			A = -pow(fabs(r) + sqrt(r2 - q3), 1. / 3);
			if (r<0) A = -A;
			B = A == 0 ? 0 : B = q / A;

			model.a /= 3;
			x[0] = (A + B) - model.a;
			x[1] = -0.5*(A + B) - model.a;
			x[2] = 0.5*sqrt(3.)*(A - B);
			if (fabs(x[2])<eps) { x[2] = x[1]; return(2); }
			return(1);
		}
	

}