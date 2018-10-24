#include "stdafx.h"
#include "SVSolver.h"





SVSolver::SVSolver()
{
}

SVSolver::~SVSolver()
{
}

int SVSolver::SlidingWnd(cv::Mat& src, vector<cv::Point2f>& allpoint, cv::Size& wndSize, vector<cv::Mat> &Kernal)
{
	cv::Mat copy;
	src.copyTo(copy);
	float k = 0.05;
	int count = 0;  //记录滑动窗口的数目
	/*int x_step = cvCeil(x_percent*wndSize.width);
	int y_step = cvCeil(y_percent*wndSize.height);*/
	int x_step = 1;
	int y_step = 1;
	//利用窗口对图像进行遍历
	for (int i = 0; i < src.cols - wndSize.width-1; i += y_step)
	{
		for (int j = 0; j < src.rows - wndSize.height-1; j += x_step)
		{
			cv::Rect roi(i, j, wndSize.width, wndSize.height);
			
			cv::Mat ROI = src(roi);
			if (ROI.empty())
				return count;
			cv::Mat ROI_s;
			ROI.convertTo(ROI_s, CV_16SC1);
			float Cs = 0, Cv = 0, Csv = 0;
			cv::Scalar     mean;
			cv::Scalar     stddev;
			//cout << ROI_s << endl;
			for (int n = 0; n < Kernal.size(); n++)
			{
				//cout << Kernal[2] << endl;
				Cs += abs(ROI_s.dot(Kernal[n]));
			}
			meanStdDev(ROI, mean, stddev);
			Cv = pow(stddev.val[0],2);
			Csv = k*Cv - Cs;
			
			if (Csv > 400 && Cs<50)
			{
				cout << "对称算子：" << Cs << endl;
				cout << "方差算子：" << Cv << endl;
				cout << Csv << endl;
				allpoint.push_back(cv::Point2f((roi.x + (wndSize.width - 1) / 2), (roi.y + (wndSize.height - 1) / 2)));
				cv::circle(copy, allpoint.back(), 5, cv::Scalar(255));
				cv::imshow("src", copy);
				cv::waitKey(0);
			}
			
			//wnd.push_back(ROI);
			count++;
		}
	}
	return count;
}

void SVSolver::SetKernal(cv::Size winSize, vector<cv::Mat> &Kernal)
{
	vector<cv::Mat> kernal;
	int xsize = winSize.width;
	int ysize = winSize.height;
	int Kernal_num = (xsize*ysize - 1) / 2;
	cv::Mat K;
	K.create(ysize, xsize, CV_16SC1);
	short* inData = K.ptr<short>(0);
	short* inData2 = K.ptr<short>(ysize - 1) + (xsize - 1);
	if (K.isContinuous())
	{
		for (int i = 0; i < Kernal_num; i++)
		{
			K.setTo(0);
			*inData = 1;
			*inData2 = -1;
			inData = inData+1;
			inData2 = inData2-1;
			kernal.push_back(K.clone());
			
			cout << K << endl;
		}
	}
	Kernal = kernal;
}