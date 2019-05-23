#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>
#include <array>
#include <stack>
#include <algorithm>
#include <numeric>

using namespace std;
using namespace cv;

class Imageprocess
{
public:
	bool findCorrespondingPointsbyORB(const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f> & points1, vector<cv::Point2f>& points2);  //ORB_找对应点;

	Mat Sobelboundary(Mat img0);                                                  //Sobel算子边缘提取;
	Mat maxEntropySegMentation(Mat inputImage);                                   //最大熵阈值分割;
	void CcaByTwoPass(const Mat & _binfilterImg, Mat & _labelImg);                //两次扫描法连通成分分析（4邻域）
	void CcaBySeedFill(const Mat& _binfilterImg, Mat & _lableImg);                //种子填充法连通成分分析（8邻域）
	void ImgReverse(const Mat &img, Mat &img_reverse);                            //二值化图像反色
	void ImgFilling(const Mat &img, Mat &img_fill);                               //空洞填充
	void LabelColor(const Mat & _labelImg, Mat & _colorImg);                      //连通域着色
	void DetectCornerHarris(const Mat & src, const Mat & colorlabel, Mat & cornershow, Mat & cornerwithimg, int threshold);             //Harris角点检测
	void DetectCornerShiTomasi(const Mat & src, const Mat & colorlabel, Mat & cornerwithimg, int minDistance, double mincorenerscore);  //Shi-Tomasi角点检测

protected:
	float caculateCurrentEntropy(Mat hist, int threshold);                        //计算当前阈值的前后景熵;
	Scalar GetRandomColor();                                                      //随机取色器

private:
};

#endif //IMAGE_H