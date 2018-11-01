// goodFeaturesToTrack.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <vector>
#include <Windows.h>
#include "PNPSolver.h"
#include "IPMSolver.h"
#include "SVSolver.h"


const float gridSide = 6.5; //厘米
const float fov_H = 53.8; //度
const float fov_V = 31.4;
const float H = 149.5; //厘米

const float ipm_x = 5;//厘米
const float ipm_y = 5;//厘米
const float ipm_Y = 1000;//像素
const float ipm_X = 240;//像素
const int image_w = 1280;
const int image_h = 720;
cv::Mat outRoi;
cv::Mat outrealtimeRoi;

class camera_info
{
public:
	cv::Mat camera_matrix;
	cv::Mat distortion_coefficients;
};

class auto_calib
{
public:

	cv::Mat m_src;
	std::vector<cv::Point2f> m_point;
private:

}calib;

//从目录中获取相机矩阵
int getMandD(std::string path, camera_info &info)
{
	cv::FileStorage fsD, fsM;
	std::string D_path = path + "/D.yml";
	std::string M_path = path + "/M.yml";
	fsD.open(D_path, cv::FileStorage::READ);
	fsM.open(M_path, cv::FileStorage::READ);
	if (fsD.isOpened() && fsM.isOpened())
	{
		fsD["D"] >> info.distortion_coefficients;
		fsM["M"] >> info.camera_matrix;
		return 1;
	}
	else
	{
		std::cout << "相机参数未输入" << std::endl;
		return 0;
	}
	
}

//void getPlanarSurface(std::vector<cv::Point2f> imgP, cv::Mat objPM, camera_info info,cv::Mat &rvec,cv::Mat &tvec,cv::Mat &cvec)
//{
//
//	//cv::Mat Rvec;
//	cv::Mat_<float> Rvec;
//	cv::Mat_<float> Tvec;
//	//cv::Mat raux, taux;
//	///////////////////////////////////////
//	std::cout << rvec << tvec << std::endl;
//	std::cout << info.camera_matrix << std::endl;
//	std::cout << info.distortion_coefficients << std::endl;
//	//Rodrigues(rotM, rvec);
//	cv::solvePnP(objPM, cv::Mat(imgP), info.camera_matrix, info.distortion_coefficients, rvec, tvec);
//	//Rodrigues(rvec, rotM);
//
//	//cout << "rotation matrix: " << endl << rotM << endl;
//	//cout << "translation matrix: " << endl << tv[0] << " " << tv[1] << " " << tv[2] << endl;
//	rvec.convertTo(Rvec, CV_32F);    //旋转向量
//	tvec.convertTo(Tvec, CV_32F);   //平移向量
//
//	cv::Mat_<float> rotMat(3, 3);
//	cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
//	/*projectedPoints.clear();
//	projectPoints(objPM, rvec, tvec, info.camera_matrix, info.distortion_coefficients, projectedPoints);
//
//	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
//	{
//		circle(image, projectedPoints[i], 3, Scalar(255, 0, 0), -1, 8);
//	}*/
//}

//棋盘格自动检测，亚像素精确化
void getCorner(cv::Mat src, cv::Point Mouse, cv::Point2f &Corner, cv::Size rect = cv::Size(20,20))
{
	using namespace cv;
	Mat image_copy;
	src.copyTo(image_copy);
	Mat image_gray;
	cv::cvtColor(image_copy, image_gray, cv::COLOR_BGR2GRAY);
	Mat gray_ROI = image_gray(Rect(Mouse.x - rect.width / 2, Mouse.y - rect.height / 2, rect.width, rect.height));
	std::vector<cv::Point2f> corners;
	int max_corners = 61;
	double quality_level = 0.05;
	double min_distance = 10;
	int block_size = 3;
	bool use_harris = false;
	double k = 0.04;

	//角点检测
	cv::goodFeaturesToTrack(gray_ROI,
		corners,
		max_corners,
		quality_level,
		min_distance,
		cv::Mat(),
		block_size,
		use_harris,
		k);

	if (corners.size() == 0)
	{
		return;
	}
	cv::TermCriteria criteria = cv::TermCriteria(
		cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
		40,
		0.01);

	//亚像素检测
	cv::cornerSubPix(gray_ROI, corners, cv::Size(3, 3), cv::Size(-1, -1), criteria);

	Corner = Point2f(corners[0].x + Mouse.x - rect.width / 2, corners[0].y + Mouse.y - rect.height / 2);
}

//void on_mouse(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
//{
//	cv::Mat img;
//	cv::Point pre_pt;
//	cv::Point2f corner_pt;
//	(((calib_Image*)ustc)->m_src).copyTo(img);//将原始图片复制到img中  
//	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
//	{
//		
//		pre_pt = cv::Point(x, y);
//		corner_pt = pre_pt;
//		getCorner(img, pre_pt, corner_pt);
//		((calib_Image*)ustc)->m_point.push_back(corner_pt);
//		//putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, +Scalar(0, 0, 255), 1, 8);//在窗口上显示坐标  
//		//circle(img, corner_pt, 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//划圆  
//		for (int i = 0; i < ((calib_Image*)ustc)->m_point.size(); i++)
//		{
//			circle(img, ((calib_Image*)ustc)->m_point[i], 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//划圆  
//		}
//		cv::imshow("img", img);
//	}
//	if (event == CV_EVENT_RBUTTONDOWN)
//	{
//		if (((calib_Image*)ustc)->m_point.size()>0)
//			((calib_Image*)ustc)->m_point.pop_back();
//		for (int i = 0; i < ((calib_Image*)ustc)->m_point.size(); i++)
//		{
//			circle(img, ((calib_Image*)ustc)->m_point[i], 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//划圆  
//		}
//		cv::imshow("img", img);
//	}
//}

//鼠标点点测距
void on_mouse_out(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
{
	cv::Mat img;
	cv::Point pre_pt;
	cv::Point2f corner_pt;
	outRoi.copyTo(img);//将原始图片复制到img中  
	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
	{

		pre_pt = cv::Point(x, y);
		int y_dist = 0, x_dist = 0;
		y_dist = (ipm_Y - pre_pt.y)*ipm_y;
		x_dist = (ipm_X/2 - pre_pt.x)*ipm_x;
		cout << "Y距离：" << y_dist << endl;
		cout << "X距离：" << x_dist << endl;
		//corner_pt = pre_pt;
		//getCorner(img, pre_pt, corner_pt);
		//((calib_Image*)ustc)->m_point.push_back(corner_pt);
		//putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, +Scalar(0, 0, 255), 1, 8);//在窗口上显示坐标  
		//circle(img, corner_pt, 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//划圆  
		cv::circle(img, pre_pt, 4, cv::Scalar(0), CV_FILLED, CV_AA, 0);//划圆  
		cv::imshow("IPM_image", img);
	}

}

//鼠标点点连线
void on_mouse_line(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
{
	cv::Mat img;
	cv::Point pre_pt;
	cv::Point2f corner_pt;
//	img = ((auto_calib*)ustc)->m_src;//将原始图片复制到img中  
	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
	{

		pre_pt = cv::Point(x, y);
		
		if (((auto_calib*)ustc)->m_point.size() != 6)
		{
			((auto_calib*)ustc)->m_point.push_back(pre_pt);
		}	
		for (int i = 0; i < calib.m_point.size(); i++)
		{
			cv::circle(((auto_calib*)ustc)->m_src, ((auto_calib*)ustc)->m_point[i], 4, cv::Scalar(0), CV_FILLED, CV_AA, 0);//划圆  
		}
		int num = floor(calib.m_point.size() / 2);
		for (int i = 0; i < num; i++)
		{
			cv::line(((auto_calib*)ustc)->m_src, calib.m_point[i * 2], calib.m_point[i * 2 + 1], cv::Scalar(0), 3);
		}
		cv::imshow("image_gray", ((auto_calib*)ustc)->m_src);
	}

}

//cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta)
//{
//	// 计算旋转矩阵的X分量
//	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
//		1, 0, 0,
//		0, cos(theta[0]), -sin(theta[0]),
//		0, sin(theta[0]), cos(theta[0])
//		);
//
//	// 计算旋转矩阵的Y分量
//	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
//		cos(theta[1]), 0, sin(theta[1]),
//		0, 1, 0,
//		-sin(theta[1]), 0, cos(theta[1])
//		);
//
//	// 计算旋转矩阵的Z分量
//	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
//		cos(theta[2]), -sin(theta[2]), 0,
//		sin(theta[2]), cos(theta[2]), 0,
//		0, 0, 1);
//
//	// 合并 
//	cv::Mat R = R_z * R_y * R_x;
//
//	return R;
//}

void setImgP(std::vector<cv::Point2f> &imgP)
{
	//imgP.push_back(cv::Point2f(916.282837,328.988599));
	//imgP.push_back(cv::Point2f(955.690186, 329.316803));
	//imgP.push_back(cv::Point2f(994.717590, 329.345398));
	//imgP.push_back(cv::Point2f(916.532532, 368.465027));
	//imgP.push_back(cv::Point2f(955.586914, 368.615845));
	//imgP.push_back(cv::Point2f(994.784851, 368.568390));
	//imgP.push_back(cv::Point2f(955.519958, 408.263184));
	//imgP.push_back(cv::Point2f(955.590027, 447.417236));
	//imgP.push_back(cv::Point2f(955.595215, 486.777832));
	//imgP.push_back(cv::Point2f(955.625732, 526.348328));
	//imgP.push_back(cv::Point2f(955.589783, 565.472351));
	//imgP.push_back(cv::Point2f(955.650757, 604.770630));
	//imgP.push_back(cv::Point2f(955.598389, 644.516846));
	//imgP.push_back(cv::Point2f(955.569946, 683.501648));
	//imgP.push_back(cv::Point2f(955.669678, 722.931763));
	//imgP.push_back(cv::Point2f(955.639221, 762.314453));
	//imgP.push_back(cv::Point2f(955.836548, 801.171082));
	//imgP.push_back(cv::Point2f(916.716309, 840.411133));
	//imgP.push_back(cv::Point2f(955.766174, 840.597839));
	//imgP.push_back(cv::Point2f(994.931091, 840.451904));
	//imgP.push_back(cv::Point2f(916.822632, 879.373230));
	//imgP.push_back(cv::Point2f(955.766174, 879.554688));
	//imgP.push_back(cv::Point2f(995.030762, 879.554688));
}

int _tmain(int argc, _TCHAR* argv[])
{

	//****************************设置标定板世界坐标****************************************************
	std::vector<cv::Point3f>objP = {
		cv::Point3f(-1 * gridSide, -7 * gridSide, 0),
		cv::Point3f(0 * gridSide, -7 * gridSide, 0),
		cv::Point3f(1 * gridSide, -7 * gridSide, 0),
		cv::Point3f(-1 * gridSide, -6 * gridSide, 0),
		cv::Point3f(0 * gridSide, -6 * gridSide, 0),
		cv::Point3f(1 * gridSide, -6 * gridSide, 0),
		cv::Point3f(-1 * gridSide, -5 * gridSide, 0),
		cv::Point3f(0 * gridSide, -5 * gridSide, 0),
		cv::Point3f(1 * gridSide, -5 * gridSide, 0),
		cv::Point3f(-1 * gridSide, -4 * gridSide, 0),
		cv::Point3f(0 * gridSide, -4 * gridSide, 0),
		cv::Point3f(1 * gridSide, -4 * gridSide, 0),
		cv::Point3f(-1 * gridSide, -3 * gridSide, 0),
		cv::Point3f(0 * gridSide, -3 * gridSide, 0),
		cv::Point3f(1 * gridSide, -3 * gridSide, 0),
		cv::Point3f(-1 * gridSide, -2 * gridSide, 0),
		cv::Point3f(0 * gridSide, -2 * gridSide, 0),
		cv::Point3f(1 * gridSide, -2 * gridSide, 0),
		cv::Point3f(-1 * gridSide, -1 * gridSide, 0),
		cv::Point3f(0 * gridSide, -1 * gridSide, 0),
		cv::Point3f(1 * gridSide, -1 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 0 * gridSide, 0),
		cv::Point3f(0 * gridSide, 0 * gridSide, 0),
		cv::Point3f(1 * gridSide, 0 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 1 * gridSide, 0),
		cv::Point3f(0 * gridSide, 1 * gridSide, 0),
		cv::Point3f(1 * gridSide, 1 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 2 * gridSide, 0),
		cv::Point3f(0 * gridSide, 2 * gridSide, 0),
		cv::Point3f(1 * gridSide, 2 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 3 * gridSide, 0),
		cv::Point3f(0 * gridSide, 3 * gridSide, 0),
		cv::Point3f(1 * gridSide, 3 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 4 * gridSide, 0),
		cv::Point3f(0 * gridSide, 4 * gridSide, 0),
		cv::Point3f(1 * gridSide, 4 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 5 * gridSide, 0),
		cv::Point3f(0 * gridSide, 5 * gridSide, 0),
		cv::Point3f(1 * gridSide, 5 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 6 * gridSide, 0),
		cv::Point3f(0 * gridSide, 6 * gridSide, 0),
		cv::Point3f(1 * gridSide, 6 * gridSide, 0),
		cv::Point3f(-1 * gridSide, 7 * gridSide, 0),
		cv::Point3f(0 * gridSide, 7 * gridSide, 0),
		cv::Point3f(1 * gridSide, 7 * gridSide, 0),
	};
	//****************************设置标定板世界坐标****************************************************


	std::string camlib_path = "./camlib";
	camera_info info1;
	bool isGetMandD = getMandD(camlib_path, info1);
	if (!isGetMandD)
	{
		return -1;
	}
	//std::cout << info1.camera_matrix << std::endl;
	//std::cout << info1.distortion_coefficients << std::endl;
	cv::VideoCapture cam;
	
	//cv::namedWindow("frame", CV_WINDOW_NORMAL);
	cam.open(0);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, image_w);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, image_h);
	cv::Mat image_color_src;
	cv::Mat image_color;

	//****************************开启摄像头现场取景标定****************************************************
	double camera_x = info1.camera_matrix.at<double>(0, 2);
	double camera_y = info1.camera_matrix.at<double>(1, 2);
	cv::Mat newCameraMatrix = info1.camera_matrix.clone();
	newCameraMatrix.at<double>(0, 2) = 1280.0/ 2.0;
	newCameraMatrix.at<double>(1, 2) = 720.0/ 2.0;
	//cout << newCameraMatrix << endl;
	cv::Mat frame;
	while (true)
	{

		cam >> frame;
		if (frame.empty())
			continue;
		cv::undistort(frame, image_color_src, info1.camera_matrix, info1.distortion_coefficients);
		//image_color_src.copyTo(image_color);
		cv::line(image_color_src, cv::Point2f(camera_x - 1, 0), cv::Point(camera_x, image_color_src.rows), cv::Scalar(0, 0, 255), 1);
		cv::line(image_color_src, cv::Point2f(0, camera_y - 1), cv::Point(image_color_src.cols, camera_y-1), cv::Scalar(0, 0, 255), 1);
		cv::imshow("frame", image_color_src);
		if (cv::waitKey(30) == VK_ESCAPE)
		{
			image_color = frame;
		}
		else
		{
			continue;
		}


		//****************************开启摄像头现场取景标定****************************************************
		//image_color = cv::imread("8mm.jpg", cv::IMREAD_COLOR);
		//****************************读取硬盘内图片标定****************************************************
		//cv::undistort(image_color_src, image_color, info1.camera_matrix, info1.distortion_coefficients);
		cv::Mat image_g;
		cv::cvtColor(image_color, image_g, CV_BGR2GRAY);
		//cv::equalizeHist(image_g, image_g);
		//calib_Image calib;
		//image_color.copyTo(calib.m_src);
		//cv::namedWindow("img", CV_WINDOW_NORMAL);

		std::vector<cv::Point2f> imgP;
		//cv::setMouseCallback("img", on_mouse, &calib);//调用回调函数  
		//cv::Mat ROI = image_color(cv::Rect(0, 0, 1920, 540));
		bool isfind = cv::findChessboardCorners(image_g, cv::Size(3, 15), imgP, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (!isfind)
		{
			cout << "未找到角点" << endl;
			continue;
		}

		cv::cornerSubPix(image_g, imgP, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
		//cv::find4QuadCornerSubpix(image_g, imgP, cv::Size(3, 3));
		bool isdraw = false;
		cv::drawChessboardCorners(image_g, cv::Size(3, 15), imgP, isdraw);

		//SVSolver SV;
		//vector<cv::Mat> k;
		//cv::Size s(5, 5);
		//SV.SetKernal(s,k);

		//vector<cv::Point2f> p;
		//SV.SlidingWnd(image_g,p,s,k );

		cv::imshow("img", image_g);


		//setImgP(imgP);
		//****************************初始化PNP类并进行PNP计算****************************************************
		PNPSolver p4psolver;
		//if (cv::waitKey(0) == VK_SPACE)
		if (1)
		{
			//imgP = calib.m_point;
			//getPlanarSurface(imgP, objPM,info1,rvec,tvec,cvec);
			//初始化PNPSolver类

			//初始化相机参数
			p4psolver.SetCameraMatrix(info1.camera_matrix);
			//设置畸变参数
			p4psolver.SetDistortionCoefficients(info1.distortion_coefficients);
			//设置特征点的世界坐标
			p4psolver.Points3D = objP;

			cout << "test2:特征点世界坐标 = " << endl << p4psolver.Points3D << endl;
			//设置特征点的图像坐标
			p4psolver.Points2D = imgP;

			cout << "test2:图中特征点坐标 = " << endl << p4psolver.Points2D << endl;

			/*if (p4psolver.Solve(PNPSolver::METHOD::CV_P3P) == 0)
				cout << "test2:CV_P3P方法:  相机位姿→" << "Oc坐标=" << p4psolver.Position_OcInW << "    相机旋转=" << p4psolver.Theta_W2C << endl;
				if (p4psolver.Solve(PNPSolver::METHOD::CV_ITERATIVE) == 0)
				cout << "test2:CV_ITERATIVE方法:    相机位姿→" << "Oc坐标=" << p4psolver.Position_OcInW << "    相机旋转=" << p4psolver.Theta_W2C << endl;*/
			if (p4psolver.Solve(PNPSolver::METHOD::CV_EPNP) == 0)
				cout << "test2:CV_EPNP方法: 相机位姿→" << "Oc坐标=" << p4psolver.Position_OcInW << "    相机旋转=" << p4psolver.Theta_W2C << endl;
		}
		cv::FileStorage Theta("Theta.yml", cv::FileStorage::WRITE);
		Theta << "Theta_W2C" << p4psolver.Theta_W2C;
		//****************************初始化PNP类并进行PNP计算****************************************************
		//****************************初始化IPM类并进行IPM计算****************************************************
		//float thetax = p4psolver.Theta_W2C.x;
		//p4psolver.Theta_W2C.x += 0.2;
		IPMSolver ipm(p4psolver.Theta_W2C, info1.camera_matrix, H);
		cv::Mat image_gray, image_gray_src;
		image_gray_src = cv::imread("8mm.png", 0);

		//image_gray_src = frame.clone();


		//cv::cvtColor(image_gray_src, image_gray_src, CV_BGR2GRAY);
		//cv::Mat newcameramatrix = info1.camera_matrix;
		//newcameramatrix.at<double>(0, 2) = image_w / 2.0;
		//newcameramatrix.at<double>(1, 2) = image_h / 2.0;
		cv::undistort(image_gray_src, image_gray, info1.camera_matrix, info1.distortion_coefficients);
		//cv::cvtColor(image_color, image_gray, CV_BGR2GRAY);
		//****************************进行自动校准，得到校准俯仰角更新IPM类中的参数，重新进行IPM计算****************************************************
		image_gray.copyTo(calib.m_src);
		//cv::imshow("image_gray", calib.m_src);
		//cv::setMouseCallback("image_gray", on_mouse_line, &calib);//调用回调函数  


		//if (cv::waitKey(0) == VK_SPACE)
		if (1)
		{
			cv::Mat out(ipm_Y, ipm_X, CV_8UC1);

			//if (calib.m_point.size() == 4)
			if (1)
			{

				ipm.aotuCamlib(calib.m_point);
				int isolve = ipm.Solve(image_gray, out, ipm_x, ipm_y);
				//cv::namedWindow("IPM_image", CV_WINDOW_NORMAL);
				outRoi = out(cv::Rect(0, 0, ipm_X, ipm_Y));


			}
			else
			{
				std::cout << "输入平行线数量不等于2,不校正" << std::endl;
				cv::Mat out(ipm_Y, ipm_X, CV_8UC1);
				int isolve = ipm.Solve(image_gray, out, ipm_x, ipm_y);
				//cv::namedWindow("IPM_image", CV_WINDOW_NORMAL);
				outRoi = out(cv::Rect(0, 0, ipm_X, ipm_Y));

			}


			cv::imshow("IPM_image", outRoi);
			//cv::setMouseCallback("IPM_image", on_mouse_out);//调用回调函数  
		}
		/*cam.open(0);
		cam.set(CV_CAP_PROP_FRAME_WIDTH, image_w);
		cam.set(CV_CAP_PROP_FRAME_HEIGHT, image_h);
		while (true)
		{
		cv::Mat frame;
		cam >> frame;
		if (frame.empty())
		continue;
		cv::Mat outrealtime(ipm_Y, ipm_X, CV_8UC1);
		int isolve = ipm.Solve(frame, outrealtime, ipm_x, ipm_y);
		cv::namedWindow("realtime", CV_WINDOW_NORMAL);
		outrealtimeRoi = outrealtime(cv::Rect(0, 0, ipm_X, ipm_Y - 500));
		cv::imshow("realtime", outrealtimeRoi);
		if (cv::waitKey(30) == VK_ESCAPE)
		{
		cam.release();
		break;
		}

		}*/

		//****************************进行自动校准，得到校准俯仰角更新IPM类中的参数，重新进行IPM计算****************************************************
		//****************************初始化IPM类并进行IPM计算****************************************************
		cv::waitKey(10);
	}
	return 0;
}
