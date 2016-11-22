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
   if(wiringPiSetup() < 0)return false;
   else return true;
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
    fd = serialOpen("/dev/ttyACM0",9600);
}
void serialconnect::end(){
    //my_serialport.close();
}
bool serialconnect::getisstarted(){
    return isstarted;
}
