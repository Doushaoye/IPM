#pragma once
#include <opencv2\opencv.hpp>
#include <math.h>
// �������ڽ��IPM���⣬˳������Զ�У׼�����ǵĹ���
// ʹ��google���ĵ���͸�ӷ�����Ҫ֪�������ǣ�ƫ���ǣ�������������߶�
// ����˳��
// 1.��ʼ������
// 2.����aotuCamlib(),У׼�����ǣ��Զ�У׼����Ҫ����ԭʼͼ�е�ƽ������Ϊ���룩
// 3.��ƺ���͸��ͼ�ĳߴ�ͱ���������Solve()�������м��㣬�õ����ͼ
using namespace std;


class IPMSolver //��͸�ӱ任
{
public:


	IPMSolver();
	IPMSolver(cv::Point3f thetax,cv::Mat cameraM, float h);
	void setThetax(double thetaxX);
	~IPMSolver();
	//���ŷ����
	cv::Point3f Thetax;

	double Fx;
	double Fy;

	float Cu; //X�����ѧ����
	float Cv; //Y�����ѧ����
	float H; //����߶�
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

