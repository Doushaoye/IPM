#pragma once
#include <opencv2\opencv.hpp>
#include <math.h>
// 本类用于解决IPM问题，顺带解决自动校准俯仰角的功能
// 使用google论文的逆透视法，需要知道俯仰角，偏航角，相机矩阵和相机高度
// 调用顺序：
// 1.初始化本类
// 2.调用aotuCamlib(),校准俯仰角（自动校准，需要两条原始图中的平行线作为输入）
// 3.设计好逆透视图的尺寸和比例，调用Solve()方法运行计算，得到俯瞰图
using namespace std;


class IPMSolver //逆透视变换
{
public:


	IPMSolver();
	IPMSolver(cv::Point3f thetax,cv::Mat cameraM, float h);
	void setThetax(double thetaxX);
	~IPMSolver();
	//相机欧拉角
	cv::Point3f Thetax;

	double Fx;
	double Fy;

	float Cu; //X方向光学中心
	float Cv; //Y方向光学中心
	float H; //相机高度
	double getTanOfLine(cv::Point2f p1, cv::Point2f p2);
	float c1, c2, s1, s2; //c1=cos(cu)
	bool getTrend(std::vector<cv::Point2f> line, double &error);
	int aotuCamlib(std::vector<cv::Point2f> line);
	int Solve(cv::Mat &input, cv::Mat &output, float xSize, float ySize);
	void getInterpolationRatio(cv::Point2f P, float a[], cv::Point p[]);
	int setH(float a[]);
	int setH2(float a[]);
private:
	float getAngleFromTan(float a);
};

