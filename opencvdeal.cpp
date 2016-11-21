#include "opencvdeal.h"
using namespace cv;
using namespace std;
bool comp(const vector<Point> &a, const  vector<Point> &b)//soft排序的自定义排序函数
{
    return a.size()>b.size();
}
opencvdeal::opencvdeal()
{
    //circleinit();
}
opencvdeal::~opencvdeal(){

}
//更新程序
void opencvdeal::process(){
    //模式0:检测circle 在uibutton 切换
    if(m_mode==1){
        //先检验是否在状态1 若在则证明已经打开摄像头 变成state2 初始化大圈区域
        if(m_circle_state==0){
            cout<<"circlestate0"<<endl;
            circleinit();
        }else if(m_circle_state==1){
            cout<<"circlestate1"<<endl;
            circlefirst();
        }
        if(m_circle_state==2){
            circlesecond();
        }else if(m_circle_state==3){
            cout<<"circlestate3"<<endl;
        }
    }//模式1:双目模式 检测注水坐标
    else if(m_mode==2){
        if(m_stereo_state==0){
            stereoinit();
        }else if(m_stereo_state==1){
            stereofirst();
        }
        if(m_stereo_state==2){
            stereosecond();
            //cout<<"stereostate2"<<endl;
        }else if(m_stereo_state==3){
            cout<<"stereostate3"<<endl;
        }
    }
}
//stereo
void opencvdeal::stereoinit(){
    m_stereo_state=1;
    int cont = 0;
    match_method=0;
    cout<<"opencvdealinit()"<<endl;
    while (frame.rows < 2){
        capture.open(1);//左边
        cont = 0;
        cout << capture.get(CV_CAP_PROP_FRAME_WIDTH) << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        cout << capture.get(CV_CAP_PROP_FRAME_WIDTH) << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
        while (frame.rows < 2 && cont<5){
            capture >> frame;
            cont++;
        }
    }
    while (frame1.rows < 2){
        capture1.open(0);//右边
        cont = 0;
        cout << capture1.get(CV_CAP_PROP_FRAME_WIDTH) << capture1.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
        capture1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
        capture1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
       cout << capture1.get(CV_CAP_PROP_FRAME_WIDTH) << capture1.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
        while (frame1.rows < 2 && cont<5){
            capture1 >> frame1;
            cont++;
        }
    }
}
void opencvdeal::stereofirst(){
    capture>>frame;
    capture1>>frame1;
    count2=true;
    templ = imread("left0muban6f.jpg",0 );
    //templ = imread("left0blue.jpg",0 );//这个效果不错
    //cvtColor(frame,frame,CV_BGR2GRAY );
    //cvtColor(frame1,frame1,CV_BGR2GRAY );
    img_size=frame.size();
    Mat R, T, R1, P1, R2, P2;
    Mat M1, D1, M2, D2;
    M1=(Mat_<double>(3, 3) << 1352.18130, 0,       305.36488 ,
                              0,        1362.66396, 166.76304,
                              0,       0,       1);
    D1=(Mat_<double>(5, 1) <<   -0.02368  , -0.04913  , -0.02612  , -0.00245,  0.00000 );
    M2=(Mat_<double>(3, 3) <<  1347.15555, 0,        381.90553,
                              0,       1354.59120,223.13032,
                               0,       0,       1);
    D2=(Mat_<double>(5, 1) <<0.04663 ,  -0.21662 ,  -0.03152 ,  -0.00436,  0.00000);
    R=(Mat_<double>(3, 3) <<  0.9996 ,  -0.0107 ,   0.0253,
                               0.0117 ,   0.9992,   -0.0390,
                              -0.0249 ,   0.0393 ,   0.9989);//xiugai
    T=(Mat_<double>(3, 1) <<  -39.68265,   0.59558 , 3.30036 );//xiugai
    //得出 Q，R, T, R1, P1, R2, P2等参数  其实此处可省区 因为 上述参数不变，也可以实现设置好

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    FileStorage fs("extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    //得出映射的map
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    /*
    //bm参数
    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    //bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 11;
    bm.state->SADWindowSize =15;//SAD越大 深度图越形成块状11
    bm.state->minDisparity =0;//0
    numberOfDisparities = ((img_size.width/8) + 15) & -16;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;//10貌似越大色块越小
    bm.state->uniquenessRatio =10;//越大匹配点越少，即更精确15
    bm.state->speckleWindowSize = 500;//100
    bm.state->speckleRange = 32;//32
    bm.state->disp12MaxDiff = -1;//1
    */
    m_stereo_state=2;
}
void opencvdeal::stereosecond(){
    capture>>frame;
    capture1>>frame1;
    Mat imgl, imgr,maskl,maskr;
    //获得重新映射后的图
    remap(frame, imgl, map11, map12, INTER_LINEAR);
    remap(frame1, imgr, map21, map22, INTER_LINEAR);
    frame = imgl;
    frame1 = imgr;
    inRange(frame,Scalar(hl,sl,vl),Scalar(hu,su,vu),maskl);
    inRange(frame1,Scalar(hl,sl,vl),Scalar(hu,su,vu),maskr);
    //cout<<hl<<hu<<sl<<su<<vl<<vu<<endl;
    //Mat mask1,mask2;
    //mask1=Mat::zeros(frame.size(), CV_8UC1);
    //rectangle(mask1,roi1,Scalar(255,255,255),-1,8,0);
    //mask2=Mat::zeros(frame.size(), CV_8UC1);
    //rectangle(mask2,roi2,Scalar(255,255,255),-1,8,0);
    //mask1=mask1(roi1);
    //mask1.setTo(0);
    //mask1.size()=;
    //imshow("mask1",mask2);
    frame.copyTo(imgl1,maskl);
    frame1.copyTo(imgr1,maskr);
    imgr1=imgr1(roi2);
    imgl1=imgl1(roi1);
    cvtColor(imgl1,imgl1,CV_BGR2GRAY);
    cvtColor(imgr1,imgr1,CV_BGR2GRAY);
    if(count2){
        Vibe_stereo1.init(imgl1);
        Vibe_stereo1.processFirstFrame(imgl1);
        Vibe_stereo2.init(imgr1);
        Vibe_stereo2.processFirstFrame(imgr1);
        count2=false;
    }else {
        Mat mask1,mask2;
        Vibe_stereo1.testAndUpdate(imgl1);
        mask1 = Vibe_stereo1.getMask();
        Vibe_stereo2.testAndUpdate(imgr1);
        mask2 = Vibe_stereo2.getMask();
        //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        //进行形态学操作
        //morphologyEx(mask1, mask1, MORPH_CLOSE, element);
        //morphologyEx(mask2, mask2, MORPH_CLOSE, element);
        //imshow("mask",mask1);
        //imshow("mask2",mask2);
        Mat imgtemp1,imgtemp2;
        imgl1.copyTo(imgtemp1);
        imgr1.copyTo(imgtemp2);
        MatchingMethod(0,imgtemp1,imgtemp2);
        //Mat disp, disp8;
        //因为是用bm算法因此需要输入灰度图
        //cvtColor(imgl1,imgl,CV_BGR2GRAY );
        //cvtColor(imgr1,imgr,CV_BGR2GRAY );
        //bm(imgl, imgr, disp);
        //disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        //imshow("l",imgl);
        //imshow("r",imgr);
        //imshow("deep photo",disp8);
        //Mat xyz;
        //Mat td=(Mat_<double>(4, 1) <<   pointl.x,pointl.y,pointl.x-pointr.x,1 );//xiugai  pointl.x,pointl.y,pointl.x-pointr.x
        //reprojectImageTo3D(td, xyz, Q, true);
        //cout<< <<endl;
        //Mat xyz=Q*td;
        //把定位点还原为世界坐标 效果还行

        double X,Y,Z;//计算真实坐标里的X,Y,Z
        X=(roi1.x+pointl.x-cx)*Tx/(roi1.x+pointl.x-pointr.x-roi2.x);//因为用roi把目标范围缩小了，因此要补回 下面原因同上
        Y=(cy-pointl.y-roi1.y)*Tx/(roi1.x+pointl.x-pointr.x-roi2.x);
        Z=f*Tx/(roi1.x+pointl.x-pointr.x-roi2.x);
        tdpoint[0]=X;
        tdpoint[1]=Y;
        tdpoint[2]=Z;
        //tdpoint.rows=Y;
        //tdpoint.depth=Z;
        //cout<<"X:"<<X<<"Y:"<<Y<<"Z:"<<Z<<endl;

    }

}
void opencvdeal::MatchingMethod( int, Mat& img,Mat& img2 )
{
  /// Source image to display
  Mat result;
  //Mat img_display,img_display2;
  //img2.copyTo(img_display2);
  //img.copyTo( img_display );
  /// Create the result matrix
  int result_cols =  img2.cols - templ.cols + 1;
  int result_rows = img2.rows - templ.rows + 1;
  result.create( result_rows, result_cols, CV_32FC1 );
  /// Do the Matching and Normalize
  ///用灰度图检测比较快

  matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }
  /// Show me what you got
  pointl=Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows );
  rectangle( imgl1, matchLoc, pointl, Scalar::all(0), 2, 8, 0     );
  //rectangle( result, matchLoc, pointl, Scalar::all(0), 2, 8, 0 );
  matchTemplate( img2, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  /// Localizing the best match with minMaxLoc
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }
  /// Show me what you got
   pointr= Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows );
  rectangle( imgr1, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0     );
  //rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  //imshow( "image_window", img );
  //imshow( "image_window2", img2 );
  //imshow( "result_window", result );
  //cout<< "left:"<<"x:"<<pointl.x<<"y:"<<pointl.y<< endl;
  //cout<< "right:"<<"x:"<<pointr.x<<"y:"<<pointr.y<< endl;

}
//circle
void opencvdeal::circleinit(){
    m_circle_state=1;
    int cont = 0;
    cout<<"opencvdealinit()"<<endl;
    while (frame3.rows < 2){
        capture3.open(1);//E:/QTPROJECT/OPENCVshow/4.avi 注意这里路径的写法，如果是只写文件名 需要放在相应release/degub 的根目录
        //cap1.set(CV_CAP_PROP_FPS, 10); //desired  FPS
        //cout<<cap1.get(CV_CAP_PROP_FPS)<<endl;
        //fps=cap1.get(CV_CAP_PROP_FPS);
        //cap1.set(CV_CAP_PROP_FOURCC, 'GPJM');
        cont = 0;
        cout << capture3.get(CV_CAP_PROP_FRAME_WIDTH) << capture3.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
        cout << boolalpha << capture3.set(CV_CAP_PROP_FRAME_HEIGHT, 240) << endl;
        cout << boolalpha << capture3.set(CV_CAP_PROP_FRAME_WIDTH, 320) << endl;
        cout << capture3.get(CV_CAP_PROP_FRAME_WIDTH) << capture3.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
        while (frame3.rows < 2 && cont<5){
            capture3 >> frame3;

            cont++;
        }
    }
    if(frame3.size().width==640)
        resize(frame3,frame3,Size(640/cof,480/cof));

}
void opencvdeal::circlefirst(){
    capture3 >> frame3;
    if(frame3.size().width==640)
        resize(frame3,frame3,Size(640/cof,480/cof));
    frame3.copyTo(frameprocess);
    if(startrecognize){
        cvtColor(frame3, gray, CV_RGB2GRAY);
        GaussianBlur(gray, gray, Size(35/cof/2*2+1, 35/cof/2*2+1), 0, 0);//高斯必须为奇数35,35
        threshold(gray, gray, 0, 255, THRESH_BINARY | THRESH_OTSU);
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(12/cof, 12/cof));
        //进行形态学操作
        morphologyEx(gray, gray, MORPH_OPEN, element);
        Canny(gray, gray, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
        findContours(gray, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        //////////////////////////////////////////////////////
        cout << "Contours: " << contours.size() << "  " << ends;
        const unsigned int cmin = 200/cof;//200
        const unsigned int cmax = 4000/cof;//2000
        vector<vector<Point> >::iterator itc = contours.begin();
        while (itc != contours.end())
        {
            if ((!isclose(*itc)) || itc->size()<cmin || itc->size()>cmax)
                itc = contours.erase(itc);
            else
            {
                ++itc;
            }
        }
        cout << "Contours: " << contours.size() << "  " << ends;
        cout << endl;
        //双层 因此大于等于4 时才有两个圈 没有两个圈前不会进入步骤二
        if (contours.size() >= 4){
            itc = contours.begin();
            int i = 0;
            while (itc != contours.end())
            {
                Scalar color(40 * i, 255, 30 * i);
                drawContours(frameprocess, contours, i, color, 2);
                itc++;
                i++;
            }
            sort(contours.begin(), contours.end(), comp);//对轮廓进行排序
            c_size_max = contours[0].size();//取出最大轮廓
            ellisperectmax = fitEllipse(Mat(contours[0]));
            br=(ellisperectmax.size.height+ellisperectmax.size.width)/2;
            cout << "height:" << ellisperectmax.size.height << "width:" << ellisperectmax.size.width << "area:" << ellisperectmax.size.area() << endl;
            re = boundingRect(contours[0]);//roi感兴趣区域，其他去除
            ellipse(frameprocess, ellisperectmax, Scalar(rand() & 255, rand() & 255, rand() & 255), 8/cof);
            //imshow("frameprocess1", frameprocess);
            cmask = Mat::zeros(frame3.size(), CV_8UC1);
            cmask.setTo(0);
            ellipse(cmask, ellisperectmax, Scalar(255), -1);
            int rectoffset=15/cof;
            cmask = cmask(Rect(re.x + rectoffset, re.y + rectoffset, re.size().width -2*rectoffset, re.size().height - 2*rectoffset));
            ellisperectmax.center.x = ellisperectmax.center.x - re.x-15/cof;//消除切除后圆心的偏移
            ellisperectmax.center.y = ellisperectmax.center.y - re.y-15/cof;//同上

            lastmask=Mat::zeros(frame3.size(), CV_8UC1);
            lastmask2 = Mat::zeros(frame3.size(), CV_8UC1);
            lastmask.setTo(255);
            lastmask2.setTo(255);

            c_size_min = contours[2].size();//取出第二大的轮廓 因为是双层 所以取第三个
            RotatedRect ellispemin = fitEllipse(Mat(contours[2]));
            int esoffset=100/cof;
            ellipse(lastmask, RotatedRect(ellispemin.center, Size(ellispemin.size.width - esoffset, ellispemin.size.height - esoffset), ellispemin.angle), Scalar(0), -1);
            ellipse(lastmask2, RotatedRect(ellispemin.center, Size(ellispemin.size.width + esoffset, ellispemin.size.height + esoffset), ellispemin.angle), Scalar(0), -1);

            lastmask = lastmask(Rect(re.x + rectoffset, re.y + rectoffset, re.size().width - 2*rectoffset, re.size().height - 2*rectoffset));
            lastmask2 = lastmask2(Rect(re.x + rectoffset, re.y + rectoffset, re.size().width - 2*rectoffset, re.size().height - 2*rectoffset));
            m_circle_state=2;
        }
    }

}
void opencvdeal::circlesecond(){
    Mat  mask;
    static bool count = true;
    capture3 >> frame3;
    if(frame3.size().width==640)
        resize(frame3,frame3,Size(640/cof,480/cof));//缩小
    Mat frame4;
    frame4=frame3;
    if (!frame4.empty())
    {
        int rectoffset=15/cof;
        frame4 = frame3( Rect(re.x+rectoffset,re.y+rectoffset,re.size().width-2*rectoffset,re.size().height-2*rectoffset));//只取出中心部分，其余不要，可减少遍历时运算，加快速度
        cvtColor(frame4, gray, CV_RGB2GRAY);

        if (count)
        {
            cout << "rows:"<<frame4.rows<<"col:"<<frame4.cols<< endl;
            Vibe_Bgs.init(gray);
            Vibe_Bgs.processFirstFrame(gray);
            cout << " Training ViBe complete!" << endl;
            count = false;
        }
        else
        {
            if (mask.empty())
                mask.create(frame4.size(), frame4.type());
            //Mat mask(frame4.size(),frame4.type());
            mask = Scalar::all(0);
            //bg_model(frame4, mask, true ? 0.005 : 0);// 0.005背景更新速度  选择
            Vibe_Bgs.testAndUpdate(gray);
            mask = Vibe_Bgs.getMask();
            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
            morphologyEx(mask, mask, MORPH_CLOSE, element);//闭运算 让运动区域变大
            //imshow("move", mask);
            Mat move;
            frame4.copyTo(move,mask);
            GaussianBlur(move, move, Size(5, 5), 0, 0);
            cvtColor(move, move, CV_BGR2HSV);
            vector<Mat> colorspirt;
            split(move, colorspirt);
            //Mat move2;
            //此处需更改 现在是检测蓝色
            inRange(colorspirt[0], 75, 140, mask);
            morphologyEx(mask, mask, MORPH_OPEN, element);
            int mask_area = countNonZero(mask);//这里的mask需要改成只有注水杆区域（目前还包括粉层的扩散）
            //mask_area=getarea(mask);
            cout << "area:"<<mask_area<< endl;
            if (mask_area<500/cof)//更新背景
            {
                frame4.copyTo(f1);
                cout << "f1update"<< endl;
            }
            //imshow("mask", mask);
            //gray(Scalar(255));
            gray.setTo(255);
            Mat frame_gs;
            /*vibe运动算法去除注水区域（暂时不用）
            f1.copyTo(frame2, mask);
            mask = 255 - mask;//mask取反 此时mask为没运动区域
            frame.copyTo(frame2, mask);
            Mat frame2_gray;
            cvtColor(frame2, frame2_gray, CV_RGB2GRAY);
            GaussianBlur(frame2_gray, frame2_gray, Size(3, 3), 0, 0);
            element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
            morphologyEx(frame2_gray, frame2_gray, MORPH_OPEN, element);
            imshow("f2", frame2_gray);//消除遮挡后mat
            */
            //bg_model.getBackgroundImage(f1);
            f1.copyTo(gray, mask);
            mask = 255 - mask;//mask取反 此时mask为没运动区域
            frame4.copyTo(gray, mask);
            cvtColor(gray,gray, CV_RGB2GRAY);
            GaussianBlur(gray, gray, Size(21/cof/2*2+1, 21/cof/2*2+1), 0, 0);
            //element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
            //morphologyEx(frame2_gray, frame2_gray, MORPH_OPEN, element);
            //imshow("f2", gray);//消除遮挡后mat
            //GaussianBlur(gray, gray, Size(21, 21), 0, 0);
            //morphologyEx(gray, gray, MORPH_CLOSE, element);
            //mask = cmask&mask&lastmask;//此时mask为椭圆内没运动区域
            //imshow("ellispe", mask);
            int a = otsu(gray, cmask);//只对cmask二值化（避免四个角剩余的蓝黑色部分影响） frame2_gray是用来vibe算法去除注水杆效果
            frame_gs = gray > a;
            Mat c_gray;
            frame_gs.copyTo(c_gray, cmask);
            c_gray = (c_gray&lastmask)|lastmask2;//lastmask与lastmask2限制了粉圈范围，粉圈的二值化图黑圈大于lastmask，小于lastmask2
            //dilate (c_gray, c_gray, element);//腐蚀操作
            element = getStructuringElement(MORPH_ELLIPSE, Size(20/cof, 20/cof));
            morphologyEx(c_gray, c_gray, MORPH_CLOSE, element);
            //imshow("dst", c_gray);
            Canny(c_gray, c_gray, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
            findContours(c_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            vector<vector<Point> >::iterator itContours = contours.begin();
            for (; itContours != contours.end(); ++itContours) {

                cout << "Size: " << itContours->size() << ends;//每个轮廓包含的点数
            }
            cout << "Contours: " << contours.size() << c_size_min << endl;
            unsigned int cmin,cmax;
            if (c_size_min == 0){
                cmin = 100/cof;
                cmax = 2000/cof;
            }
            else
            {
                cmin = c_size_min-100/cof;//把上次检测的结果作为下次检测的筛选条件
                cmax =c_size_min+50/cof;//
            }

            vector<vector<Point> >::iterator itc = contours.begin();
            while (itc != contours.end())
            {
                if ((!isclose(*itc))||itc->size()<cmin || itc->size()>cmax )
                    itc = contours.erase(itc);
                else
                {
                    ++itc;
                }
            }
            cout << "Contours: " << contours.size() << "  " << ends;
            cout << endl;
            ////////////////////////////////////////////
            if (contours.size() > 0){
                itc = contours.begin();
                int i = 0;
                int j = 0;
                unsigned int max = itc->size();
                frame4.copyTo(frameprocess);
                while (itc != contours.end())
                {
                    Scalar color(40 * i, 255, 30 * i);
                    drawContours(frameprocess, contours, i, color, 2);
                    if (itc->size() > max){
                        max = itc->size();
                        j = i;
                    }
                    itc++;
                    i++;
                }
                RotatedRect rrect = fitEllipse(Mat(contours[j]));
                cout << "height:" << rrect.size.height << "width:" << rrect.size.width << "area:"<<rrect.size.area()<<endl;
                if (abs(rrect.center.x - ellisperectmax.center.x)<c_rectr&&abs(rrect.center.y - ellisperectmax.center.y)<c_rectr
                    && (rrect.size.height + rrect.size.width)<(ellisperectmax.size.height + ellisperectmax.size.width)
                    && rrect.size.area()<ellisperectmax.size.area())
                {
                    sr=(rrect.size.height+rrect.size.width)/2;
                    c_size_min = contours[j].size();
                    ellipse(frameprocess, rrect, Scalar(rand() & 255, rand() & 255, rand() & 255), 8/cof);
                    lastmask.setTo(255);
                    lastmask2.setTo(255);
                    //以这次检测出的圆去限制（补全&去除）下一个圆   范围：-10~20（待调整）
                    int roffset=10/cof;
                    ellipse(lastmask2, RotatedRect(rrect.center, Size(rrect.size.width +roffset, rrect.size.height +roffset), rrect.angle), Scalar(0), -1);
                    ellipse(lastmask, RotatedRect(rrect.center, Size(rrect.size.width -roffset, rrect.size.height - roffset), rrect.angle), Scalar(0), -1);
                    Rect re = boundingRect(Mat(contours[j]));//roi感兴趣区域，其他去除
                    //int a = otsu(gray);
                    Mat fc;
                    fc=gray(re);
                    Mat fc2;
                    lastmask2(re).copyTo(fc2);
                    fc = fc > 100;//取出单独处理  fc = fc > a;
                    fc=fc|fc2;//去除四个角的黑色部分
                    //Mat fc3;
                    //mask(re);

                    fc2 = fc2 | mask(re);//只取圆圈内作为注水杆
                    //imshow("fc2", mask(re));
                    fc = fc & fc2;//去除注水杆部分
                    fc2 = 255 - fc2;

                    int contarea=contourArea(contours[j]);
                    int blackarea =fc.rows*fc.cols- countNonZero(fc);
                    double prewhite =  (contarea - blackarea)*100/ (contarea-countNonZero(fc2));//在总面积中减去注水 杆遮掩到的面积
                    cout << prewhite<<"%"<< "  "<<contarea << "  "<<blackarea<<  endl;
                    //imshow("quanli", fc);
                    //imshow("yuantu",frame);
                    //imshow("frameprocess", frameprocess);//绘制椭圆轮廓后的图像
                }
            }
        }
    }
}
void opencvdeal::saveimg(){
    static int imgcount=0;
    string leftname;
    string rightname;
    stringstream ss;//把int 转string型
    string s;
    ss<<imgcount;
    ss>>s;
    if(m_mode==1){
        leftname="frame"+s+".jpg";
        rightname="process"+s+".jpg";
        imgcount++;
        cvtColor(frameprocess,frameprocess,CV_RGB2BGR);
        cvtColor(frame3,frame3,CV_RGB2BGR);
        imwrite(leftname,frame3);
        imwrite(rightname,frameprocess);
    }
    else if(m_mode==2)
    {
        leftname="left"+s+".jpg";
        rightname="right"+s+".jpg";
        imgcount++;
        cvtColor(frame,frame,CV_RGB2BGR);
        cvtColor(frame1,frame1,CV_RGB2BGR);
        imwrite(leftname,frame);
        imwrite(rightname,frame1);
    }

}
int opencvdeal::getratio(){
    return sr*100/br;//扩大了100倍控制最终结果在两位小数
}
Mat opencvdeal::getimgl1(){
    return imgl1;
}

Mat opencvdeal::getimgr1(){
    return imgr1;
}
float* opencvdeal::getpoint(){
    return tdpoint;
}
Mat opencvdeal::getframe3(){
    return frame3;
}
Mat opencvdeal::getframe1(){
    return frame1;
}
Mat opencvdeal::getframe(){
    return frame;
}
Mat opencvdeal::getframepross(){
    return frameprocess;
}
int opencvdeal::getfps(){
    return fps;
}
void opencvdeal::sethsv(int h1, int h2, int s1, int s2, int v1, int v2){
    hl=h1;
    hu=h2;
    sl=s1;
    su=s2;
    vl=v1;
    vu=v2;
}
void opencvdeal::setstartrecognize(bool i){
    startrecognize=i;
}
void opencvdeal::setmode(unsigned int i){
    m_mode=i;
}
void opencvdeal::setstereostate(unsigned int i){
    m_stereo_state=i;
}
void opencvdeal::setcirclestate(unsigned int i){
    m_circle_state=i;
}
unsigned int opencvdeal::getmode(){
    return m_mode;
}
unsigned int opencvdeal::getstereostate(){
    return m_stereo_state;
}
unsigned int opencvdeal::getcirclestate(){
    return m_circle_state;
}


bool opencvdeal::isclose(vector<Point> contour){//判断轮廓是否连续
    Point firstp;
    Point endp;
    firstp = contour[contour.size() / 4*3];
    endp = contour[contour.size()/4];
    //所有findcontour出来的轮廓都是闭合的，如果轮廓为曲线/直线（看上去），则是原路返回
    //所以取1/4 与3/4的两个点进行对比，如果两个点位置为同一个点则，可以判断该轮廓不是闭合曲线（视觉）
    if (abs(firstp.x - endp.x) <=50 && abs(firstp.y - endp.y) <= 50)
    {
        return false;
    }
    else
    {
        return true;
    }
}
int opencvdeal::getarea(Mat &img){
    int i = 0,j=0;
    int height = img.cols;
    int width = img.rows;
    int count=0;
    for (i = 0; i<width; i++){
        for (j = 0; j<height; j++){

            if (img.at<char>(i,j)=0)
            {
                count++;
            }
        }
    }
    return count;
}
////////////////////////////////灰度直方图增强对比度///////////////////////////////////////////////////
int opencvdeal::ImgStrong(Mat &img, Mat &result)
{
    //***************
    //p[]各个灰度级出现的概率
    //p1[]各个灰度级之前的概率和
    //各个灰度级出现的次数
    //*****************
    assert((img.cols == result.cols) && (img.rows == result.rows));
    double p[256], p1[256], num[256];
    int nheight = img.rows;
    int nwidth = img.cols;
    int total = nheight*nwidth;
    memset(p, 0, sizeof(p));
    memset(p1, 0, sizeof(p1));
    memset(num, 0, sizeof(num));
    //各个灰度级出现的次数
    for (int i = 0; i < nheight; i++)
    {
        uchar *data = img.ptr<uchar>(i);
        for (int j = 0; j < nwidth; j++)
        {
            num[data[j]]++;
        }
    }

    //各个灰度级出现的概率
    for (int i = 0; i < 256; i++)
    {
        p[i] = num[i] / total;
    }
    //各个灰度级之前的概率和
    for (int i = 0; i < 256; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            p1[i] += p[j];
        }
    }

    //直方图变换
    for (int i = 0; i < nheight; i++)
    {
        uchar *data = img.ptr<uchar>(i);
        uchar *data0 = result.ptr<uchar>(i);
        for (int j = 0; j < nwidth; j++)
        {
            data0[j] = p1[data[j]] * 255 + 0.5;
        }
    }
    return 0;
}

////////////////////////////otsu法阈值确定///////////////////////////////////////////
int opencvdeal::otsu(cv::Mat&dst){

    int i, j;
    int tmp;

    double u0, u1, w0, w1, u, uk;

    double cov;
    double maxcov = 0.0;
    int maxthread = 0;

    int hst[MAX_GRAY_VALUE] = { 0 };
    double pro_hst[MAX_GRAY_VALUE] = { 0.0 };

    int height = dst.cols;
    int width = dst.rows;

    //统计每个灰度的数量
    for (i = 0; i<width; i++){
        for (j = 0; j<height; j++){
            tmp = dst.at<uchar>(i, j);
            hst[tmp]++;
        }
    }
    //计算每个灰度级占图像中的概率
    for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
        pro_hst[i] = (double)hst[i] / (double)(width*height);

    //计算平均灰度值
    u = 0.0;
    for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
        u += i*pro_hst[i];

    double det = 0.0;
    for (i = MIN_GRAY_VALUE; i< MAX_GRAY_VALUE; i++)
        det += (i - u)*(i - u)*pro_hst[i];

    //统计前景和背景的平均灰度值，并计算类间方差

    for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++){

        w0 = 0.0; w1 = 0.0; u0 = 0.0; u1 = 0.0; uk = 0.0;

        for (j = MIN_GRAY_VALUE; j < i; j++){

            uk += j*pro_hst[j];
            w0 += pro_hst[j];

        }
        u0 = uk / w0;

        w1 = 1 - w0;
        u1 = (u - uk) / (1 - w0);
        //计算类间方差
        cov = w0*w1*(u1 - u0)*(u1 - u0);
        if (cov > maxcov)
        {
            maxcov = cov;
            maxthread = i;
        }
    }
    std::cout << maxthread << std::endl;
    return maxthread;
}

int opencvdeal::otsu(cv::Mat&dst, cv::Mat&mask){

    int i, j;
    int tmp;

    double u0, u1, w0, w1, u, uk;

    double cov;
    double maxcov = 0.0;
    int maxthread = 0;

    int hst[MAX_GRAY_VALUE] = { 0 };
    double pro_hst[MAX_GRAY_VALUE] = { 0.0 };

    int height = dst.cols;
    int width = dst.rows;
    int are = 0;
    //统计每个灰度的数量
    for (i = 0; i<width; i++){
        for (j = 0; j<height; j++){
            if (mask.at<uchar>(i, j) != 0)
            {
                are++;
                tmp = dst.at<uchar>(i, j);
                hst[tmp]++;
            }

        }
    }
    //计算每个灰度级占图像中的概率
    for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
        pro_hst[i] = (double)hst[i] / (double)(are);


    //计算平均灰度值
    u = 0.0;
    for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
        u += i*pro_hst[i];

    double det = 0.0;
    for (i = MIN_GRAY_VALUE; i< MAX_GRAY_VALUE; i++)
        det += (i - u)*(i - u)*pro_hst[i];

    //统计前景和背景的平均灰度值，并计算类间方差

    for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++){

        w0 = 0.0; w1 = 0.0; u0 = 0.0; u1 = 0.0; uk = 0.0;

        for (j = MIN_GRAY_VALUE; j < i; j++){

            uk += j*pro_hst[j];
            w0 += pro_hst[j];

        }
        u0 = uk / w0;


        w1 = 1 - w0;
        u1 = (u - uk) / (1 - w0);


        //计算类间方差
        cov = w0*w1*(u1 - u0)*(u1 - u0);




        if (cov > maxcov)
        {
            maxcov = cov;
            maxthread = i;
        }
    }
    std::cout << maxthread << std::endl;
    return maxthread;
}
