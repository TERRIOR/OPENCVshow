#include "piserialconnect.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
using namespace std;
serialconnect::serialconnect()
{

}
serialconnect::~serialconnect(){

}
bool serialconnect::init(){
   if(wiringPiSetupSys() < 0)return false;
   else {
       return true;
   }
}

bool serialconnect::send(string senddata){
    //requestData.append(tr(senddata.data()));
    serialPrintf(fd,senddata.c_str());
    return true;
}

string serialconnect::receive(){
    string data;
    while(serialDataAvail(fd)>0){
            comdata+=char(serialGetchar(fd));
    }
    if(comdata.length()>0)
    {
            cout<<comdata<<endl;
            data=comdata;
            comdata="";
    }
    return data;
}
void serialconnect::start(){
    fd = serialOpen("/dev/ttyUSB0",9600);
    if(fd>0){
        isstarted=true;
        cout<<"opened the serial"<<endl;
    } else cout<<"unopened"<<endl;
}
void serialconnect::end(){
    //my_serialport.close();
}
bool serialconnect::getisstarted(){
    return isstarted;
}
