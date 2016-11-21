#include "serialconnect.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
using namespace std;
serialconnect::serialconnect()
{

}
serialconnect::~serialconnect(){

}
vector<string> serialconnect::init(){
    vector<string> comstring;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos){
        cout<<info.portName().toStdString()<<endl;
        comstring.push_back(info.portName().toStdString());
    }
    return comstring;
}

bool serialconnect::send(string senddata){
    //requestData.append(tr(senddata.data()));
    if(isstarted){
        my_serialport.write(senddata.data());
        return true;
    }

}

string serialconnect::receive(){
    if(isstarted){
        requestData = my_serialport.readAll();
        string data;
        data=requestData.data();
        requestData.clear();
        return data;
    }

}
void serialconnect::start(string com){
    isstarted=true;
    my_serialport.setPortName(MainWindow::tr(com.data()));
    my_serialport.open(QIODevice::ReadWrite);
    my_serialport.setBaudRate(115200);
    my_serialport.setDataBits(QSerialPort::Data8);
    my_serialport.setParity(QSerialPort::NoParity);
    my_serialport.setStopBits(QSerialPort::OneStop);
    my_serialport.setFlowControl(QSerialPort::NoFlowControl);
}
void serialconnect::end(){
    my_serialport.close();
}
bool serialconnect::getisstarted(){
    return isstarted;
}
