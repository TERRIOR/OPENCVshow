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
        if(str.length()>1&&str.length()<4){
            switch (str[0]){
                case 't':
                nowtemp=atoi(str.substr(1).c_str());
                ui->labelnowtemp->setText(QString("nowtemp:")+QString::number(nowtemp) );
                break;
            }
        }
    }
    if(cvdeal.getmode()==1){
        qtimg1=Mat2QImage(cvdeal.getframe3());
        qtimg2=Mat2QImage(cvdeal.getframepross());
        //点了start才开始识别且计算比例
        if(recognizestart){
            int ratio=cvdeal.getratio();
            cout<<ratio<<"%"<<endl;
            ui->labelpowder->setText(QString("POWDER:")+QString::number(ratio)+QString("%"));
            //pi*R^2*h/3 h=R*sqrt(3)  so v=pi*sqrt(3)/3*R^3
            int v=ratio*ratio*ratio*3.14*sqrt(3)/24000;// 3.14*(ratio/100*100/2)*(ratio/100*100/2)*sqrt(3)*(ratio/100*100/2)/3
            ui->labelv->setText(QString("V:")+QString::number(v));
            if(Serialconnect.getisstarted())
                Serialconnect.send("p"+ratio);
        }
    }
    if(cvdeal.getmode()==2){
        qtimg1=Mat2QImage(cvdeal.getimgl1());
        qtimg2=Mat2QImage(cvdeal.getimgr1());
        int *point;
        point=cvdeal.getpoint();
        string x,y,z;
        string s;
        stringstream ss;//把int 转string型
        ss<<point[0];
        ss>>x;
        ss.clear();
        ss<<point[1];
        ss>>y;
        ss.clear();
        ss<<point[2];
        ss>>z;
        //cout<<"x:"<<x<<"y:"<<y<<"z:"<<z<<endl;
        s="x"+x+"y"+y+"h"+z;
        if(Serialconnect.getisstarted())
            Serialconnect.send(s);
        ui->label3dtext->setText(tr(s.data()));
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

