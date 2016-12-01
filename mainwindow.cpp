#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "opencvdeal.h"

using namespace cv;

QImage MainWindow::Mat2QImage(Mat image1)
{
    QImage img;
    //image1=image;
    if (image1.channels()==3) {
        cvtColor(image1, image1, CV_BGR2RGB);
        img = QImage((const unsigned char *)(image1.data), image1.cols, image1.rows,
                image1.cols*image1.channels(), QImage::Format_RGB888);
        //img=img.scaled(image1.cols/2, image1.rows/2);//调整缩放比例
        //cvtColor(image, image, CV_RGB2BGR);
    } else if (image1.channels()==1) {
        img = QImage((const unsigned char *)(image1.data), image1.cols, image1.rows,
                image1.cols*image1.channels(), QImage::Format_Indexed8);//灰度图
        //img=img.scaled(image1.cols/1.2, image1.rows/1.2);//调整缩放比例

    } else {
        img = QImage((const unsigned char *)(image1.data), image1.cols, image1.rows,
                image1.cols*image1.channels(), QImage::Format_RGB888);
    }

    return img;
}
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    this->setWindowState(Qt::WindowMaximized);
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
    ui->lblOriginal ->setText(tr("no img"));
    ui->lblProcessed->setText(tr("no img"));
    ui->SldHLower->setSliderPosition(0);
    ui->SldHUpper->setSliderPosition(255);
    ui->SldSLower->setSliderDown(0);
    ui->SldSUpper->setSliderPosition(255);
    ui->SldVLower->setSliderPosition(0);
    ui->SldVUpper->setSliderPosition(255);
    //tmrTimer->start(1000/cvdeal.getfps());
    //获得串口数目与名字
    vector<string> comstring=Serialconnect.init();
    vector<string>::iterator iterator=comstring.begin();
    ui->com->addItem(tr("none"));
    for(;iterator!=comstring.end();iterator++){
        string com=*iterator;
        ui->com->addItem(tr(com.data()));
    }
    Opoint[0]=0;
    Opoint[1]=0;
    Opoint[2]=0;
}
void MainWindow::processFrameAndUpdateGUI(){

    ui->SldHValue->clear();
    ui->SldHValue->appendPlainText(QString("(") +
                                      QString::number(ui->SldHLower->value()).rightJustified(3,' ') +
                                      QString(",") +
                                      QString::number(ui->SldHUpper->value()).rightJustified(3,' ') +
                                      QString(")"));
    ui->SldSValue->clear();
    ui->SldSValue->appendPlainText(QString("(") +
                                       QString::number(ui->SldSLower->value()).rightJustified(3,' ') +
                                       QString(",") +
                                       QString::number(ui->SldSUpper->value()).rightJustified(3,' ') +
                                       QString(")"));
    ui->SldVValue->clear();
    ui->SldVValue->appendPlainText(QString("(") +
                                     QString::number(ui->SldVLower->value()).rightJustified(3,' ') +
                                     QString(",") +
                                     QString::number(ui->SldVUpper->value()).rightJustified(3,' ') +
                                     QString(")"));
    cvdeal.sethsv(ui->SldHLower->value(),ui->SldHUpper->value(),ui->SldSLower->value(),ui->SldSUpper->value(),ui->SldVLower->value(),ui->SldVUpper->value());
    settemp=ui->SldTemp->value();
    cout<<settemp<<endl;
    if(settemp!=70)
        ui->labelsettemp->setText(QString("settemp:")+QString::number(settemp));
    else
        ui->labelsettemp->setText(QString("settemp:not set"));
    QImage qtimg1,qtimg2;
    cvdeal.process();
    //Mat 转 QImage 需要在此处转换 不然报错（不知为何） 因此需要被显示的需要提供接口
    //模式一
    //cout<<Serialconnect.receive()<<endl;
    //Serialconnect.send("hello,arduino");
    if(Serialconnect.getisstarted()){
        String str=Serialconnect.receive();
        if(str!="")
            ui->Sldreceive->appendPlainText(QString::fromStdString(str));
        if(str.length()>0&&str.length()<4){
            switch (str[0]){
            case 't':
                nowtemp=atoi(str.substr(1).c_str());
                ui->labelnowtemp->setText(QString("nowtemp:")+QString::number(nowtemp) );
            break;
            case 'o':
                Opoint[0]=point[0];
                Opoint[1]=point[1];
                Opoint[2]=point[2];
                stereosend=true;
            break;
            case 'n':
                pointsend=false;
            break;
            case 's':
                pointsend=true;
            break;
            case 'c':
                stereosend=false;
            break;
            }
        }
    }
    if(cvdeal.getmode()==1){
        qtimg1=Mat2QImage(cvdeal.getframe3());
        qtimg2=Mat2QImage(cvdeal.getframepross());
        //点了start才开始识别且计算比例
        //一开始就发送咖啡豆的信息过去 到arduino 再发送到手机
        if(recognizestart){
            int ratio=cvdeal.getratio();
            cout<<ratio<<"%"<<endl;
            ui->labelpowder->setText(QString("POWDER:")+QString::number(ratio)+QString("%"));
            //pi*R^2*h/3 h=R*sqrt(3)  so v=pi*sqrt(3)/3*R^3、
            string r;
            string sH,sS,sV,sP;
            stringstream ss;//把int 转string型
            ss<<(int)ratio;
            ss>>r;
            ss.clear();
            ss<<cvdeal.getbeanh();
            ss>>sH;
            ss.clear();
            ss<<cvdeal.getbeans();
            ss>>sS;
            ss.clear();
            ss<<cvdeal.getbeanv();
            ss>>sV;
            ss.clear();
            ss<<cvdeal.getbeanp();
            ss>>sP;
            ss.clear();
            int v=ratio*ratio*ratio*3.14*sqrt(3)/24000;// 3.14*(ratio/100*100/2)*(ratio/100*100/2)*sqrt(3)*(ratio/100*100/2)/3
            ui->labelv->setText(QString("V:")+QString::number(v));
            cout<<sH<<","<<sS<<","<<sV<<","<<sP<<endl;
            if(Serialconnect.getisstarted()){
                static int count=0;
                if(count<5)
                    count++;
                switch (count) {
                case 1:
                    Serialconnect.send("a"+sH);
                    break;
                case 2:
                    Serialconnect.send("b"+sS);
                    break;
                case 3:
                    Serialconnect.send("c"+sV);
                    break;
                case 4:
                    Serialconnect.send("d"+sP);
                    break;
                case 5:
                    Serialconnect.send("p"+r);
                    break;
                default:
                    break;
                }
            }

        }
    }
    if(cvdeal.getmode()==2){
        qtimg1=Mat2QImage(cvdeal.getimgl1());
        qtimg2=Mat2QImage(cvdeal.getimgr1());
        point=cvdeal.getpoint();
        string sx,sy,sz,sspeed,sdis;
        string s;
        stringstream ss;//把int 转string型
        float x=point[0]-Opoint[0];
        float z=point[1]-Opoint[1];
        float y=point[2]-Opoint[2];
        static float lastx,lasty,lastz;
        static float lastdis,lasttwodis,lastspeed;
        static float total_dis,total_twodis;
        static int discount,twodiscount;
        float dis;
        float twopointdis;
        float speed;
        ss<<(int)x;
        ss>>sx;
        ss.clear();
        ss<<(int)y;
        ss>>sy;
        ss.clear();
        ss<<(int)z;
        ss>>sz;
        //cout<<"x:"<<x<<"y:"<<y<<"z:"<<z<<endl;
        s="x"+sx+"y"+sy+"h"+sz;
        if(stereosend){
            add(sqrt(x*x+y*y),10,&lastdis,&discount,&total_dis);
            add(sqrt((x-lastx)*(x-lastx)+(y-lasty)*(y-lasty)),3,&lasttwodis,&twodiscount,&total_twodis);
            static int count=0;
            count++;
            if(count==10){//一秒钟更新一次 计数十次
                dis=total_dis/count;
                twopointdis=total_twodis/count;
                speed=twopointdis*1000/dis/(6.28);//twopointdis/dis=10ms角度 1s角度=*10 除/2pi得出比例 再乘100 方便传输
                total_twodis=0;
                total_dis=0;
                ss.clear();
                ss<<(int)speed;
                ss>>sspeed;
                ss.clear();
                ss<<(int)dis;
                ss>>sdis;
                count=0;
                cout<<"r:"<<sdis<<" speed:"<<(int)speed<<" h:"<<sz<<" td:"<<(int)twopointdis<<endl;
            }
        }
        if(Serialconnect.getisstarted()&&pointsend&&stereosend){
            //Serialconnect.send(s);
            //cout<<s<<endl;
            static int count=0;
            count++;
            switch (count){
            case 4:
                if(lastdis!=dis){
                    Serialconnect.send("r"+sdis);
                    lastdis=dis;
                    //cout<<sdis<<endl;
                }
                break;
            case 8:
                if(abs(speed-lastspeed)>5&&speed<100){
                    Serialconnect.send("s"+sspeed);
                    lastspeed=speed;
                    //cout<<sspeed<<endl;
                }
                break;
            case 12:
                if(z!=lastz){
                    Serialconnect.send("h"+sz);
                    //cout<<sz<<endl;
                }
                count=0;
                break;
            default:
                break;
            }
        }
        ui->label3dtext->setText(tr(s.data()));
        lastx=x;
        lasty=y;
        lastz=z;
    }
    if(cvdeal.getmode()==1||cvdeal.getmode()==2){
        ui->lblOriginal->setPixmap(QPixmap::fromImage(qtimg1).scaled(qtimg1.width(),qtimg1.height()));
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qtimg2).scaled(qtimg2.width(),qtimg2.height()));
    }

}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::on_startButton_clicked(){
    cout<<"start"<<endl;
    if(cvdeal.getmode()==1){
        recognizestart=true;
        cvdeal.setstartrecognize(recognizestart);

    }else if (cvdeal.getmode()==2){
        if(Serialconnect.getisstarted())
            Serialconnect.send("t"+settemp);
    }


}
void MainWindow::add(float m,int error,float *lastm,int *count,float *q){//n是上一次数据
    if(abs(m-*lastm)>error){
        //*q+=*lastm;
        *count++;
        if(*count>=3){
            *lastm=m;
            *count=0;
        }
    } else{
        *lastm=m;
        *count=0;
    }
    *q+=*lastm;
}
void MainWindow::on_cancelButton_clicked()
{
    cout<<"presscancel"<<endl;
    if(tmrTimer->isActive() == true){ // timer is running
        tmrTimer->stop();
        ui->lblOriginal->setText(tr("no img"));
        ui->lblProcessed->setText(tr("no img"));
        ui->cancelButton ->setText("resume stream");
        if(Serialconnect.getisstarted())
            Serialconnect.end();
    }
    else{
        if(ui->com->currentText().toStdString()!="none"){
            Serialconnect.start(ui->com->currentText().toStdString());
        }
        tmrTimer->start(100);
        ui->cancelButton->setText("pause stream");
    }
}
void MainWindow::on_circleButton_clicked(){
    cvdeal.setmode(1);
    cout<<"circlebutton"<<endl;
    if(Serialconnect.getisstarted())
        Serialconnect.send("0");
}
void MainWindow::on_stereoButton_clicked(){
    cvdeal.setmode(2);
    cout<<"stereobutton"<<endl;
    if(Serialconnect.getisstarted())
        Serialconnect.send("2");

}
void MainWindow::on_saveButton_clicked(){
    cvdeal.saveimg();
    cout<<"savebutton"<<endl;
}

