#ifndef OPENCVDEAL_H
#define OPENCVDEAL_H
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <QImage>
#include <iostream>
#include "vibe.h"
#include <sstream>
using namespace cv;
using namespace std;
#define MAX_GRAY_VALUE 256
#define MIN_GRAY_VALUE 0
class opencvdeal
{
public:
    opencvdeal();
    ~opencvdeal();
    void circlefirst();
    void circlesecond();
    void stereofirst();
    void stereosecond();
    void process();
    void setstartrecognize(bool i);
    void setmode(unsigned int i);
    void setcirclestate(unsigned int i);
    void setstereostate(unsigned int i);
    void saveimg();
    int getfps();
    int getratio();
    unsigned int getstereostate();
    unsigned int getcirclestate();
    unsigned int getmode();
    Mat getframe();
    Mat getframe1();
    Mat getframepross();
    Mat getframe3();
    Mat getimgl1();
    Mat getimgr1();
    int* getpoint();
    int ImgStrong(Mat &img, Mat &result);
    int getarea(Mat &img);
    int otsu(cv::Mat&dst);
    int otsu(cv::Mat&dst, cv::Mat&mask);
    bool isclose(vector<Point> contour);
    void sethsv(int h1,int h2,int s1,int s2,int v1,int v2);
    void MatchingMethod( int, Mat &img ,Mat &img2);
    //bool comp(const vector<Point> &a, const  vector<Point> &b);
private:
    void circleinit();
    void stereoinit();
    VideoCapture capture3;//获得图片
    Mat frame3;//单目
    Mat frameprocess;//处理后展示
    unsigned int m_mode=0;
    unsigned int m_circle_state=0;
    unsigned int m_stereo_state=0;
    int fps;
    const int g_cannyLowThreshold = 3;
    const int c_rectr = 20;
    int br,sr;
    Mat  gray,f1;//frame:原图当作原图输出 gray作为 灰度中间变量
    ViBe_BGS Vibe_Bgs;//不知道为何只能初始化一次
    //Mat framecp;//作为最后输出
    Mat cmask;//第一次冲煮时检测出的滤杯框架位置
    Rect re;//方框位置 在第二阶段初始化cmask
    int c_size_max, c_size_min;//检测时的直径范围
    Mat lastmask, lastmask2;//范围mask
    vector<vector<Point> > contours;//检测轮廓时都会用到的，两个阶段都用到，但没有联系
    vector<Vec4i> hierarchy;
    bool count = true;
    RotatedRect ellisperectmax;
    bool startrecognize=false;
    //////////////////////////////////////*双目定位*/////////////////////////////////////////////
    VideoCapture capture,capture1;
    int numberOfDisparities ;
    //int SADWindowSize = 0, numberOfDisparities = 0;
    Rect roi1, roi2;
    Mat frame,frame1;
    ViBe_BGS Vibe_stereo1,Vibe_stereo2;
    bool count2=true;
    //StereoBM bm;//bm算法
    Size img_size;
    Mat Q;
    Mat map11, map12, map21, map22;
    int hl,hu,sl,su,vl,vu;
    Mat templ;
    Point pointl,pointr;
    int match_method;
    int tdpoint[3];
    Mat imgl1,imgr1;
    const double cx=4.4472863578796387e+002;
    const double cy=1.9008141326904297e+002;
    const double f=1.3545912000000001e+003;
    const double Tx=3.9682650000000002e+001;
    const int cof=2;
};

#endif // OPENCVDEAL_H
