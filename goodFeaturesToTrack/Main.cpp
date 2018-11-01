// goodFeaturesToTrack.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <vector>
#include <Windows.h>
#include "PNPSolver.h"
#include "IPMSolver.h"
#include "SVSolver.h"


const float gridSide = 6.5; //����
const float fov_H = 53.8; //��
const float fov_V = 31.4;
const float H = 149.5; //����

const float ipm_x = 5;//����
const float ipm_y = 5;//����
const float ipm_Y = 1000;//����
const float ipm_X = 240;//����
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

//��Ŀ¼�л�ȡ�������
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
		std::cout << "�������δ����" << std::endl;
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
//	rvec.convertTo(Rvec, CV_32F);    //��ת����
//	tvec.convertTo(Tvec, CV_32F);   //ƽ������
//
//	cv::Mat_<float> rotMat(3, 3);
//	cv::Rodrigues(Rvec, rotMat);  //����solvePnP���ص�����ת�����������޵����˹�任�����ת����
//	/*projectedPoints.clear();
//	projectPoints(objPM, rvec, tvec, info.camera_matrix, info.distortion_coefficients, projectedPoints);
//
//	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
//	{
//		circle(image, projectedPoints[i], 3, Scalar(255, 0, 0), -1, 8);
//	}*/
//}

//���̸��Զ���⣬�����ؾ�ȷ��
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

	//�ǵ���
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

	//�����ؼ��
	cv::cornerSubPix(gray_ROI, corners, cv::Size(3, 3), cv::Size(-1, -1), criteria);

	Corner = Point2f(corners[0].x + Mouse.x - rect.width / 2, corners[0].y + Mouse.y - rect.height / 2);
}

//void on_mouse(int event, int x, int y, int flags, void *ustc)//event����¼����ţ�x,y������꣬flags��ק�ͼ��̲����Ĵ���  
//{
//	cv::Mat img;
//	cv::Point pre_pt;
//	cv::Point2f corner_pt;
//	(((calib_Image*)ustc)->m_src).copyTo(img);//��ԭʼͼƬ���Ƶ�img��  
//	if (event == CV_EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ  
//	{
//		
//		pre_pt = cv::Point(x, y);
//		corner_pt = pre_pt;
//		getCorner(img, pre_pt, corner_pt);
//		((calib_Image*)ustc)->m_point.push_back(corner_pt);
//		//putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, +Scalar(0, 0, 255), 1, 8);//�ڴ�������ʾ����  
//		//circle(img, corner_pt, 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//��Բ  
//		for (int i = 0; i < ((calib_Image*)ustc)->m_point.size(); i++)
//		{
//			circle(img, ((calib_Image*)ustc)->m_point[i], 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//��Բ  
//		}
//		cv::imshow("img", img);
//	}
//	if (event == CV_EVENT_RBUTTONDOWN)
//	{
//		if (((calib_Image*)ustc)->m_point.size()>0)
//			((calib_Image*)ustc)->m_point.pop_back();
//		for (int i = 0; i < ((calib_Image*)ustc)->m_point.size(); i++)
//		{
//			circle(img, ((calib_Image*)ustc)->m_point[i], 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//��Բ  
//		}
//		cv::imshow("img", img);
//	}
//}

//�������
void on_mouse_out(int event, int x, int y, int flags, void *ustc)//event����¼����ţ�x,y������꣬flags��ק�ͼ��̲����Ĵ���  
{
	cv::Mat img;
	cv::Point pre_pt;
	cv::Point2f corner_pt;
	outRoi.copyTo(img);//��ԭʼͼƬ���Ƶ�img��  
	if (event == CV_EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ  
	{

		pre_pt = cv::Point(x, y);
		int y_dist = 0, x_dist = 0;
		y_dist = (ipm_Y - pre_pt.y)*ipm_y;
		x_dist = (ipm_X/2 - pre_pt.x)*ipm_x;
		cout << "Y���룺" << y_dist << endl;
		cout << "X���룺" << x_dist << endl;
		//corner_pt = pre_pt;
		//getCorner(img, pre_pt, corner_pt);
		//((calib_Image*)ustc)->m_point.push_back(corner_pt);
		//putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, +Scalar(0, 0, 255), 1, 8);//�ڴ�������ʾ����  
		//circle(img, corner_pt, 4, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);//��Բ  
		cv::circle(img, pre_pt, 4, cv::Scalar(0), CV_FILLED, CV_AA, 0);//��Բ  
		cv::imshow("IPM_image", img);
	}

}

//���������
void on_mouse_line(int event, int x, int y, int flags, void *ustc)//event����¼����ţ�x,y������꣬flags��ק�ͼ��̲����Ĵ���  
{
	cv::Mat img;
	cv::Point pre_pt;
	cv::Point2f corner_pt;
//	img = ((auto_calib*)ustc)->m_src;//��ԭʼͼƬ���Ƶ�img��  
	if (event == CV_EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ  
	{

		pre_pt = cv::Point(x, y);
		
		if (((auto_calib*)ustc)->m_point.size() != 6)
		{
			((auto_calib*)ustc)->m_point.push_back(pre_pt);
		}	
		for (int i = 0; i < calib.m_point.size(); i++)
		{
			cv::circle(((auto_calib*)ustc)->m_src, ((auto_calib*)ustc)->m_point[i], 4, cv::Scalar(0), CV_FILLED, CV_AA, 0);//��Բ  
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
//	// ������ת�����X����
//	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
//		1, 0, 0,
//		0, cos(theta[0]), -sin(theta[0]),
//		0, sin(theta[0]), cos(theta[0])
//		);
//
//	// ������ת�����Y����
//	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
//		cos(theta[1]), 0, sin(theta[1]),
//		0, 1, 0,
//		-sin(theta[1]), 0, cos(theta[1])
//		);
//
//	// ������ת�����Z����
//	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
//		cos(theta[2]), -sin(theta[2]), 0,
//		sin(theta[2]), cos(theta[2]), 0,
//		0, 0, 1);
//
//	// �ϲ� 
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

	//****************************���ñ궨����������****************************************************
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
	//****************************���ñ궨����������****************************************************


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

	//****************************��������ͷ�ֳ�ȡ���궨****************************************************
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


		//****************************��������ͷ�ֳ�ȡ���궨****************************************************
		//image_color = cv::imread("8mm.jpg", cv::IMREAD_COLOR);
		//****************************��ȡӲ����ͼƬ�궨****************************************************
		//cv::undistort(image_color_src, image_color, info1.camera_matrix, info1.distortion_coefficients);
		cv::Mat image_g;
		cv::cvtColor(image_color, image_g, CV_BGR2GRAY);
		//cv::equalizeHist(image_g, image_g);
		//calib_Image calib;
		//image_color.copyTo(calib.m_src);
		//cv::namedWindow("img", CV_WINDOW_NORMAL);

		std::vector<cv::Point2f> imgP;
		//cv::setMouseCallback("img", on_mouse, &calib);//���ûص�����  
		//cv::Mat ROI = image_color(cv::Rect(0, 0, 1920, 540));
		bool isfind = cv::findChessboardCorners(image_g, cv::Size(3, 15), imgP, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (!isfind)
		{
			cout << "δ�ҵ��ǵ�" << endl;
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
		//****************************��ʼ��PNP�ಢ����PNP����****************************************************
		PNPSolver p4psolver;
		//if (cv::waitKey(0) == VK_SPACE)
		if (1)
		{
			//imgP = calib.m_point;
			//getPlanarSurface(imgP, objPM,info1,rvec,tvec,cvec);
			//��ʼ��PNPSolver��

			//��ʼ���������
			p4psolver.SetCameraMatrix(info1.camera_matrix);
			//���û������
			p4psolver.SetDistortionCoefficients(info1.distortion_coefficients);
			//�������������������
			p4psolver.Points3D = objP;

			cout << "test2:�������������� = " << endl << p4psolver.Points3D << endl;
			//�����������ͼ������
			p4psolver.Points2D = imgP;

			cout << "test2:ͼ������������ = " << endl << p4psolver.Points2D << endl;

			/*if (p4psolver.Solve(PNPSolver::METHOD::CV_P3P) == 0)
				cout << "test2:CV_P3P����:  ���λ�ˡ�" << "Oc����=" << p4psolver.Position_OcInW << "    �����ת=" << p4psolver.Theta_W2C << endl;
				if (p4psolver.Solve(PNPSolver::METHOD::CV_ITERATIVE) == 0)
				cout << "test2:CV_ITERATIVE����:    ���λ�ˡ�" << "Oc����=" << p4psolver.Position_OcInW << "    �����ת=" << p4psolver.Theta_W2C << endl;*/
			if (p4psolver.Solve(PNPSolver::METHOD::CV_EPNP) == 0)
				cout << "test2:CV_EPNP����: ���λ�ˡ�" << "Oc����=" << p4psolver.Position_OcInW << "    �����ת=" << p4psolver.Theta_W2C << endl;
		}
		cv::FileStorage Theta("Theta.yml", cv::FileStorage::WRITE);
		Theta << "Theta_W2C" << p4psolver.Theta_W2C;
		//****************************��ʼ��PNP�ಢ����PNP����****************************************************
		//****************************��ʼ��IPM�ಢ����IPM����****************************************************
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
		//****************************�����Զ�У׼���õ�У׼�����Ǹ���IPM���еĲ��������½���IPM����****************************************************
		image_gray.copyTo(calib.m_src);
		//cv::imshow("image_gray", calib.m_src);
		//cv::setMouseCallback("image_gray", on_mouse_line, &calib);//���ûص�����  


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
				std::cout << "����ƽ��������������2,��У��" << std::endl;
				cv::Mat out(ipm_Y, ipm_X, CV_8UC1);
				int isolve = ipm.Solve(image_gray, out, ipm_x, ipm_y);
				//cv::namedWindow("IPM_image", CV_WINDOW_NORMAL);
				outRoi = out(cv::Rect(0, 0, ipm_X, ipm_Y));

			}


			cv::imshow("IPM_image", outRoi);
			//cv::setMouseCallback("IPM_image", on_mouse_out);//���ûص�����  
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

		//****************************�����Զ�У׼���õ�У׼�����Ǹ���IPM���еĲ��������½���IPM����****************************************************
		//****************************��ʼ��IPM�ಢ����IPM����****************************************************
		cv::waitKey(10);
	}
	return 0;
}
