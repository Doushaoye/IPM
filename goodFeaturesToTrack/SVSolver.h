#include <opencv2\opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;

class SVSolver
{
public:
	SVSolver();
	~SVSolver();


	static void findChessCorners(cv::Mat inputArray,cv::Size winsize,vector<cv::Point>outputArray);
	
	int SlidingWnd(cv::Mat& src, vector<cv::Point2f>& allpoint, cv::Size& wndSize, vector<cv::Mat> &Kernal);

	void SetKernal(cv::Size winsize, vector<cv::Mat> &Kernal);//卷积核构造函数

private:

	float SymmetryOperator(cv::Mat &winROI);//对称算子

	float VarianceOperator(cv::Mat &winROI);//方差算子

	

};
