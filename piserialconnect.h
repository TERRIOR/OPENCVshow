#ifndef PISERIALCONNECT_H
#define PISERIALCONNECT_H
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
using namespace std;
class serialconnect
{
public:
    serialconnect();
    ~serialconnect();
    bool send(string st);
    string receive();
    bool init();
    void start();
    void end();
    bool getisstarted();
private:
    int fd;
    string comdata="";
    //QSerialPort my_serialport;
    //QByteArray requestData;
    bool isstarted;
};

#endif // SERIALCONNECT_H
