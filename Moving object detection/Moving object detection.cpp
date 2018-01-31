// Moving object detection.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <cv.h>    
#include <highgui.h>      
#include <string>    
#include <iostream>    
#include <algorithm>    
#include <iterator>   
#include <stdio.h>    
#include <ctype.h>   
#include <time.h> 
#include <math.h> 
#include<windows.h>
#include <mmsystem.h>
#include <direct.h>//for mk_dir
#include <io.h>//for _acess()
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include <vector>
#include<opencv2/imgproc/types_c.h>
#include "imgproc/imgproc.hpp"
#include "video/tracking.hpp"
#define threshold_diff1 12 //设置简单帧差法阈值
#define threshold_diff2 12//设置简单帧差法阈值
using namespace cv;
using namespace std;
Mat image;

bool backprojMode = false;//表示是否要进入反向投影模式，ture表示准备进入反向投影模式  
bool selectObject = false;//代表是否在选要跟踪的初始目标，true表示正在用鼠标选择
int trackObject =0;//代表跟踪目标数目0
bool showHist = true;//是否显示直方图 
int vmin = 10, vmax = 256, smin = 30;

static void help()
{
	cout << "\nThis is a demo that shows mean-shift based tracking\n"
		"You select a color objects such as your face and it tracks it.\n"
		"This reads from video camera (0 by default, or the camera number the user enters\n"
		"Usage: \n"
		"   ./camshiftdemo [camera number]\n";

	cout << "\n\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tc - stop the tracking\n"
		"\tb - switch to/from backprojection view\n"
		"\th - show/hide object histogram\n"
		"\tp - pause video\n"
		"To initialize tracking, select the object with mouse\n";
}

const char* keys =
{
	"{@camera_number| 0 | camera number}"
};
class Package
{
private:

public:
	Rect MeanShitfMath(const Mat &OrgInImage, const Mat &PreInImage, Rect &ROIrect);
};

Rect Package::MeanShitfMath(const Mat &OrgInImage, const Mat &PreInImage, Rect &ROIrect)
{
	Mat orgimg = OrgInImage;
	Mat preimg = PreInImage;

	/*先对输入的原始图像进行感兴趣区域的采集*/
	//先获取感兴趣区域的图像
	Mat bgrROI = orgimg(ROIrect);

	//将感兴趣区域的BGR空间转换为HSV空间
	Mat hsvROI;
	cvtColor(bgrROI, hsvROI, CV_BGR2HSV);
	//获取感兴趣区域的HSV空间的S空间
	vector<Mat>hsvROIvector;
	split(hsvROI, hsvROIvector);
	//将S通道的阀值化
	threshold(hsvROIvector[1], hsvROIvector[1], 65, 255.0, THRESH_BINARY);
	//计算S通道的直方图
	float hranges[2];
	const float* ranges[1];
	int channels[1];
	MatND hsvROIhist;
	int histSize[1];
	hranges[0] = 0.0;
	hranges[1] = 180.0;
	ranges[0] = hranges;
	channels[0] = 0;
	histSize[0] = 256;
	calcHist(&hsvROIvector[1], 1, channels, Mat(), hsvROIhist, 1, histSize, ranges);

	//归一化直方图
	normalize(hsvROIhist, hsvROIhist, 1.0);

	/*在输入的第二幅图像中进行均值漂移算法*/
	Mat prehsv;
	cvtColor(preimg, prehsv, CV_BGR2HSV);
	vector<Mat>prehsvvector;
	split(prehsv, prehsvvector);
	/*threshold(prehsvvector[1], prehsvvector[1], 65, 255.0, THRESH_BINARY);*/
	//在第二幅图上获取感兴趣区域的直方图的反投影
	Mat result;
	calcBackProject(&(prehsvvector[1]), 1, channels, hsvROIhist, result, ranges, 255.0);
	threshold(result, result, 255 * (-1.0f), 255.0, THRESH_BINARY);
	bitwise_and(result, prehsvvector[1], result);

	/*rectangle(preimg, ROIrect, Scalar(0, 0, 255));*/
	//meanshift算法
	TermCriteria criteria(TermCriteria::MAX_ITER, 10, 0.01);
	meanShift(result, ROIrect, criteria);
	/*rectangle(preimg, ROIrect, Scalar(0, 255, 0));*/

	return ROIrect;
}
Package P;

Point coord;//储存初始坐标
Rect sqart;//储存矩形框的起始坐标以及长度和宽度
bool draw;
bool flag = 0;//这个标志位是用在如果要将矩形标定的部分单独显示在一个窗口时使用的
Mat frame, frame_org;
Mat dat;//感兴趣区域图像
IplImage* img = 0;
Rect ROIrect, ROIrect_org;


int main(int argc, char** argv)
{
	Mat img_src1, img_src2, img_src3;//3帧法需要3帧图片  
	Mat img_dst, gray1, gray2, gray3;
	Mat rgb, hsv1;
	Mat gray_diff1, gray_diff2;//存储2次相减的图片  
	Mat gray_diff11, gray_diff12;
	Mat gray_diff21, gray_diff22;
	Mat gray;//用来显示前景的  
	bool pause = false;
	help();
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = { 0, 180 };//hranges在后面的计算直方图函数中要用到  
	const float*phranges = hranges;
	CommandLineParser parser(argc, argv, keys);//命令解析器函数  
	int camNum = parser.get<int>(0);

	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		help();
		cout << "***Could not initialize capturing..***\n";
		cout << "Current parameter'svalue:\n";
		parser.printMessage();
		return-1;
	}
	namedWindow("Histogram", 0);
  
	
	int i, idx1, idx2;
	IplImage* silh;
	CvMemStorage *stor;
	CvSeq *cont;


	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;
	for (;;)
			{

		   if (!paused)//没有暂停 
				{
			   cap >> frame >> img_src1;;//从摄像头抓取一帧图像并输出到frame中  
			   cvtColor(img_src1, gray1, CV_BGR2GRAY);
			   waitKey(5);
			   cap >> img_src2;
			   cvtColor(img_src2, gray2, CV_BGR2GRAY); 

			   waitKey(5);
			   cap >> img_src3;
			   cvtColor(img_src3, gray3, CV_BGR2GRAY);

			   Sobel(gray1, gray1, CV_8U, 1, 0, 3, 0.4, 128);
			   Sobel(gray2, gray2, CV_8U, 1, 0, 3, 0.4, 128);
			   Sobel(gray3, gray3, CV_8U, 1, 0, 3, 0.4, 128);

			   subtract(gray2, gray1, gray_diff11);//第二帧减第一帧  
			   subtract(gray1, gray2, gray_diff12);
			   add(gray_diff11, gray_diff12, gray_diff1);
			   subtract(gray3, gray2, gray_diff21);//第三帧减第二帧  
			   subtract(gray2, gray3, gray_diff22);
			   add(gray_diff21, gray_diff22, gray_diff2);

			   for (int i = 0; i<gray_diff1.rows; i++)
			   for (int j = 0; j<gray_diff1.cols; j++)
			   {
				   if (abs(gray_diff1.at<unsigned char>(i, j)) >= threshold_diff1)//这里模板参数一定要用unsigned char，否则就一直报错  
					   gray_diff1.at<unsigned char>(i, j) = 255;            //第一次相减阈值处理  
				   else gray_diff1.at<unsigned char>(i, j) = 0;

				   if (abs(gray_diff2.at<unsigned char>(i, j)) >= threshold_diff2)//第二次相减阈值处理  
					   gray_diff2.at<unsigned char>(i, j) = 255;
				   else gray_diff2.at<unsigned char>(i, j) = 0;
			   }
			   bitwise_and(gray_diff1, gray_diff2, gray);

			   dilate(gray, gray, Mat()); 
			   erode(gray, gray, Mat());
			   Mat edges;
			   //	使用3*3内核来降噪（2*3+1=7）
			   blur(gray, edges, Size(7, 7));
			  
			   Mat element = getStructuringElement(MORPH_RECT,Size(15,15));
			   dilate(edges, edges, element);
			   erode(edges, edges, element);
			   Mat out;
			  GaussianBlur(edges, out, Size(7, 7), 0, 0);
			  blur(out, out, Size(3, 3));
			   //	进行canny边缘检测
			   Canny(out, out,50, 150, 3);
			 
			   imshow("foreground", out);
			  
			   //********************************************************************************************************
			   //提取轮廓
			   // 
			  // IplImage* img=0;
			   IplImage* dst=0;
			   int flag;
			    //cvtColor(gray, rgb, CV_GRAY2RGB);
				//imshow("123",rgb);
			   img = &IplImage(frame);//frame
			   CvSize size = cvSize(img->width, img->height);
			   IplImage*pyr = cvCreateImage(cvSize((size.width&-2)/2,(size.height&-2)/2),8,1);//创建pyr的图像指针，是降采样的金字塔图像 
			   
			   dst = &IplImage(out);
			   IplImage* silh;

			   //中值滤波
			  // cvSmooth(dst, dst, CV_MEDIAN, 3, 0, 0, 0);

			   // 向下采样，去掉噪声
			   cvPyrDown(dst, pyr, CV_GAUSSIAN_5x5);
			   cvDilate(pyr, pyr, 0, 1); // 做膨胀操作，消除目标的不连续空洞
			 cvPyrUp(pyr, dst, CV_GAUSSIAN_5x5);
			   
			 
			   CvMemStorage *stor;
			   CvSeq *cont;
			   const int CONTOUR_MAX_AERA =15000;//矩形面积
			   const int CONTOUR_MIN_AERA = 10000;
			   // 下面的程序段用来找到轮廓
			
			   // Create dynamic structure and sequence.
			   stor = cvCreateMemStorage(0);//创建内存存储器
			   cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);//创建序列用于存储对象
			   // 找到所有轮廓
			   cvFindContours(dst, stor, &cont, sizeof(CvContour),
				   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));//CV_CHAIN_APPROX_SIMPLE只保存最后一个点
			  
			   // 直接使用CONTOUR中的矩形来画轮廓
			   for (; cont; cont = cont->h_next)//CV_CHAIN_APPROX_SIMPLE
			   {
				  CvScalar color = CV_RGB(rand() & 255, rand() & 255, rand() & 255);
				   CvRect r = ((CvContour*)cont)->rect;
				   if (r.height * r.width > CONTOUR_MAX_AERA)//面积小的正方形抛弃掉
				   {
					   cvDrawContours(dst, cont, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 2, CV_FILLED, 8, cvPoint(0, 0));
					   CvRect rect = cvBoundingRect(cont, 0);
					cvRectangle(img, cvPoint(r.x, r.y),cvPoint(r.x + r.width , r.y+r.height) , CV_RGB(255, 0, 0), 8, CV_AA, 0); 
			
				   }
			
			   }
			   
			//  cvReleaseImage(&pyr);
			   //cvNamedWindow("Motion", 1);//
			   //cvShowImage("Motion", img);
			   //*********************************************************************************************************
			   Mat result;
			   
		   /*if (cvWaitKey(33) >= 0)
			   break;*/
			   if (frame.empty())
						break;
				}
		   ///***********************************************************

		 //meanShift跟踪
				Mat mat = cvarrToMat(img);
				Mat dst1;
				char c;
				int start = 0;
				//将矩形框得到矩形区域用另一个窗口显示
				if ((flag == 1) && sqart.height > 0 && sqart.width > 0)
				{
					dst1 = mat(Rect(sqart.x, sqart.y, sqart.width, sqart.height));
					namedWindow("dst");
					imshow("dst", dst1);

					ROIrect_org = Rect(sqart.x, sqart.y, sqart.width, sqart.height);
					frame_org = frame;

					start = 1;
					flag = 0;
				}

				//rectangle(mat, sqart, Scalar(0, 0, 255), 3);
				
				if (start == 1)
				{
					ROIrect = P.MeanShitfMath(frame_org, mat, ROIrect_org);//
					rectangle(mat, ROIrect, Scalar(255, 0, 0), 3);
					frame_org = mat;
					ROIrect_org = ROIrect;
				}

				imshow("Mouse", mat);

				c = waitKey(20);
				if (c == 27)
					break;




			///*****************************************************
		 
			}///////////for循环

			cvReleaseMemStorage(&stor);
		
			
	return 0;
}
