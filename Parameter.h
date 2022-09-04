#ifndef _PARAMETER_H_
#define _PARAMETER_H_

//char GLOBAL_PATH[] = "D:\\SNU\\VI lab\\Reserch\\Velodynedata\\150914용서고속\\";
//float HEIGHT_THRESHOLD = -1.5;
//int START_FRAME = 3100;
//double GRADIENT_THRESHOLD = 20;
//float PRODUCT_THRESHOLD = 0.5;
//#define START 6680 // 직선 도로 //6680 // 커브진 도로
//#define END 6880 // 직선 도로 //6680 // 커브진 도로

// 고산터널
//#define START_ 5371 
//#define END 5591 

// 운증터널
#define START_ 6803
#define END 7103 

#define STEP 2
#define STEP_ 1
#define KDTREE_SEARCH_ICP 3
#define KDTREE_SEARCH_GG 0.13
#define KDTREE_SEARCH_GN 0.1
#define KDTREE_SEARCH_L 0.2
#define KDTREE_SEARCH_CURB 0.05
#define SOLUTION_RANGE 30
#define FLT_EPSILON_CUS 0.0001
const char GLOBAL_PATH[] = "D:\\SNU\\VI lab\\Reserch\\Velodynedata\\[WSHur]150914_DragonWest\\";

/*----- SYSTEM CONSTANT -----*/



// ICP parameters
const double ICP_TRANSFORM_EPSILON = 1e-7;
const double ICP_INIT_OFFSET = 2.006;
const double ICP_DIST_FROM = 0.5;
const double ICP_DIST_UNIT = 0.05;
const double ICP_DIST_TO = 0.05;
const double ICP_ITR_FROM = 10;
const double ICP_ITR_UNIT = 10;
const double ICP_EPS_FROM = 0.0000001;
const double ICP_EPS_UNIT = 0.00000001;
const double ICP_DEQUE_SIZE = 100;
const double ICP_SAMP_DIST = 120;

//////////////////////////////////////// IVN_Parsing Parameter////////////////////////////////////////
const char* const DELIMITER2 = "	";
const int resolution = 255;
//double distance_RL;
const double distanceBetweenWheel = 1.633;//m
const double wheel_radius = (24.26 + 24.5*0.4) / 100;//m
const double PI = 3.14159265;
const double encoder_resolution = 0.01769; // cm/pulse 1.7679
const double TWO_PI = 2 * PI;

const double degree_to_radian = 0.0174532925199432957692369076849;



#endif _PARAMETER_H_