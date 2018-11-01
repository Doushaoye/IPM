#include "stdafx.h"
#include "IPMSolver.h"

//初始化赋值，参数：欧拉角，相机矩阵，相机高度
IPMSolver::IPMSolver(cv::Point3f thetax, cv::Mat cameraM ,float h)
{
	//F = f;
	Fx = cameraM.at<double>(0, 0);
	Fy = cameraM.at<double>(1, 1);

	Thetax.x = -thetax.x / 180.0*CV_PI;
	Thetax.y = thetax.y / 180.0*CV_PI;
	Thetax.z = thetax.z / 180.0*CV_PI;
	Cu = cameraM.at<double>(0, 2);
	Cv = cameraM.at<double>(1, 2);
 	H = h;

	c1 = cos(Thetax.x);
	c2 = cos(Thetax.y);
	s1 = sin(Thetax.x);
	s2 = sin(Thetax.y);
}
//对俯仰角进行重新设定
void IPMSolver::setThetax(double thetaxX)
{
	Thetax.x = thetaxX;
	c1 = cos(Thetax.x);
	s1 = sin(Thetax.x);
}

IPMSolver::~IPMSolver()
{
}

//获取双线性差值的四个点位及其权重
void IPMSolver::getInterpolationRatio(cv::Point2f P, float a[], cv::Point p[])
{
	//float a[4];
	/*cv::Point p[4];*/
	
	p[0]= cv::Point(floor(P.x), floor(P.y));
	p[1] = cv::Point(ceil(P.x), floor(P.y));
	p[2] = cv::Point(floor(P.x), ceil(P.y));
	p[3] = cv::Point(ceil(P.x), ceil(P.y));
	a[0] = (1 - (P.x - floor(P.x)))*(P.y - floor(P.y));
	a[1] = (1 - (P.y - floor(P.y)))*(1 - (P.x - floor(P.x)));
	a[2] = (1 - (P.y - floor(P.y)))* (P.x - floor(P.x));
	a[3] = (P.x - floor(P.x))*(P.y - floor(P.y));
}

//设置世界坐标系转图像坐标系矩阵
int IPMSolver::setH(float *a) 
{


	a[0] = Fx*c2 + Cu*c1*s2;
	a[1] = Cu*c1*c2 - s2*Fx;
	a[2] = -Cu*s1;
	a[3] = 0;
	a[4] = s2*(Cv*c1 - Fy*s1);
	a[5] = c2*(Cv*c1 - Fy*s1);
	a[6] = -Fy*c1 - Cv*s1;
	a[7] = 0;
	a[8] = c1*s2;
	a[9] = c1*c2;
	a[10] = -s1;
	a[11] = 0;
	a[12] = c1*s2;
	a[13] = c1*c2;
	a[14] = - s1;
	a[15] = 0;

	return 0;
}

//设置世界坐标系转图像坐标系矩阵
int IPMSolver::setH2(float *a) 
{


	a[0] = -H*c2 / Fx;
	a[1] = H*s1*s2 / Fy;
	a[2] = H*(c2*Cu / Fx - Cv*s1*s2 / Fy - c1*s2);
	a[3] = 0;
	a[4] = H*s2/Fx;
	a[5] = H*s1*c1 / Fy;
	a[6] = H*(-Cu*s2 / Fx - Cv*s1*c2 / Fy - c1*c2);
	a[7] = 0;
	a[8] = 0;
	a[9] = H*c1/Fy;
	a[10] = H*(-Cv*c1 / Fy + s1);
	a[11] = 0;
	a[12] = 0;
	a[13] = -c1 / Fy;
	a[14] = c1*Cv / Fy - s1;
	a[15] = 0;

	return 0;
}

//逆透视变换处理，参数3,4位逆透视图的单位尺度
int IPMSolver::Solve(cv::Mat &input, cv::Mat &output, float xSize, float ySize)
{
	
	if (input.channels() != 1 && output.channels() != 1)
	{
		return -1;
	}

	cv::Mat src;
	input.copyTo(src);
	//input.copyTo(output);
	float tgi[16];
	float tig[16];
	setH(tgi);
	//setH2(tig);
	cv::Mat Tgtoi(4, 4, CV_32FC1, tgi); //世界to图像矩阵
	//cv::Mat Titog(4, 4, CV_32FC1, tig); //图像to世界
	//float p_test[4] = { 1200, 719, 1, 1 }; //Pg 世界坐标系点位
	//cv::Mat p_test_m(4, 1, CV_32FC1, p_test);
	//cv::Mat p_test_W = Titog*p_test_m; 
	//p_test_W = p_test_W / p_test_W.at<float>(3, 0);
	//cout << p_test_W << endl;
	//cv::Mat p_test_IMG = Tgtoi*p_test_W;
	//p_test_IMG = p_test_IMG / p_test_IMG.at<float>(3, 0);
	//cout << p_test_IMG << endl;

	//cout << Tgtoi*Titog/149.5 << endl;
	//cout << Tgtoi << endl;

	//cout << Titog << endl;

	for (int j = 0; j < output.rows; j++)
	{
		float Yp = (output.rows - j) * ySize;
		uchar *out_p = output.ptr<uchar>(j);
		for (int i = 0; i < output.cols; i++)
		{
			
			float Xp = (i - 0.5*output.cols) * xSize;
			float Pgf[4] = { Xp, Yp, -H, 1 }; //Pg 世界坐标系点位
			cv::Mat Pg(4, 1, CV_32FC1, Pgf);
			//cout << Tgtoi*Pg << endl;
			cv::Mat Pi = Tgtoi*Pg; //计算得到图像坐标系下对应点坐标

			//cout << Pi << endl;
			/*float U,V,u, v;
			float at = atan2f(Yp , H);
			float ae = Thetax / 180 * CV_PI;
			U = tan(Thetax / 180 * CV_PI - atan2f(Yp,H));
			V = Xp / sqrt(pow(H, 2) + pow(Yp, 2)) * sqrt(1 + pow(tan(Thetax / 180 * CV_PI - atan2f(H,Yp)), 2));
			u = (1 - U / tan(Fov.y))*(src.rows - 1) / 2;
			v = (1 + V / tan(Fov.x))*(src.cols - 1) / 2;
			*/
			cv::Point2f P(Pi.at<float>(0, 0) / Pi.at<float>(3, 0), Pi.at<float>(1, 0) / Pi.at<float>(3, 0));
			cv::Point p[4]; //周围四个角
			float a[4]; //插值系数
			getInterpolationRatio(P, a, p); //计算双线性差值系数
			//cv::circle(input, P, 3, 0, -1);
			/*if (p[3].x < src.cols&&p[3].y < src.rows &&p[0].x>0 && p[0].y >0)
			{

				output.at<uchar>(j, i) = a[0] * src.at<uchar>(p[0].y, p[0].x) + a[1] * src.at<uchar>(p[1].y, p[1].x)
					+ a[2] * src.at<uchar>(p[2].y, p[2].x) + a[3] * src.at<uchar>(p[3].y, p[3].x);
			}
			else
			{
				output.at<uchar>(j, i) = 0;
			}*/
			if (P.inside(cv::Rect(0, 0, input.cols, input.rows)))
			{
				out_p[i] = a[0] * src.at<uchar>(p[0].y, p[0].x) + a[1] * src.at<uchar>(p[1].y, p[1].x)
					+ a[2] * src.at<uchar>(p[2].y, p[2].x) + a[3] * src.at<uchar>(p[3].y, p[3].x);
			}
			else
			{
				out_p[i] = 0;
			}
		}
	}
	//cout << output << endl;
	return 1;
}

//自动标定，参数为原图上的两条平行线
int IPMSolver::aotuCamlib(std::vector<cv::Point2f> line)
{
	std::vector<cv::Point2f> undistort_line;
	double low = Thetax.x - 0.1;
	double high = Thetax.x + 0.1; 
	double mid;
	//int count = 0;
	while (low < high)
	{
		mid = (low + high) / 2;
		setThetax(mid);
		float tig[16];
		setH2(tig);
		cv::Mat Titog(4, 4, CV_32FC1, tig); //图像to世界矩阵
		cout << Titog << endl;
		//count++;

		for (int i = 0; i < line.size(); i++)
		{
			float Xi = line[i].x;
			float Yi = line[i].y;
			float Pif[4] = { Xi, Yi, 1, 1 }; //Pi 图像坐标系点位
			cv::Mat Pi(4, 1, CV_32FC1, Pif);
			//std::cout << Pi << std::endl;
			cv::Mat Pg = Titog*Pi; //计算得到世界坐标系下对应点坐标
			std::cout << Pg.at<float>(2, 0) / Pg.at<float>(3, 0) << std::endl;
			cv::Point2f P(Pg.at<float>(0, 0) / Pg.at<float>(3, 0), Pg.at<float>(1, 0) / Pg.at<float>(3, 0));
			std::cout << P << std::endl;

			undistort_line.push_back(P);
		}
		double Error;
		bool Trend = getTrend(undistort_line, Error);
		if (fabs(Error) < 0.5)
		{
			break;
		}
		else
		{
			undistort_line.clear();
			if (Trend)
			{
				high = mid ;
			}
			else
			{
				low = mid ;
			}
		}
			
	}
	cout << "自动标定俯仰角：" << Thetax.x / CV_PI * 180 << endl;
	return 1;
}

//获取直线斜率
double IPMSolver::getTanOfLine(cv::Point2f p1, cv::Point2f p2)
{
	double  x_dis = p1.x - p2.x;
	if (p1.x == p2.x)
		return 0;
	return atan2f((p1.y - p2.y), x_dis)/CV_PI*180.0;
}

//获取倾斜趋势，ture为俯仰角实际偏大，false为偏小
bool IPMSolver::getTrend(std::vector<cv::Point2f> line, double &error)
{
	int numline = line.size() / 2;
	switch (numline)
	{
	case 2:
	{
		double tan1, tan2;
		tan1 = getTanOfLine(line[0],line[1]);
		tan2 = getTanOfLine(line[2], line[3]);
		error = tan1 - tan2;
		if (error<0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	case 3:
	{
		//暂时为讨论好合适的判断方法
	}
	default:
		break;
	}
}

