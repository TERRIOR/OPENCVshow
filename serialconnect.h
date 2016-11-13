#ifndef SERIALCONNECT_H
#define SERIALCONNECT_H
#include <QtSerialPort/QSerialPortInfo>
#include <QtSerialPort/QSerialPort>

#include <iostream>
using namespace std;
class serialconnect
{
public:
    serialconnect();
    ~serialconnect();
    bool send(string st);
    string receive();
    vector<string> init();
    void start(string com);
    void end();
    bool getisstarted();
private:
    QSerialPort my_serialport;
    QByteArray requestData;
    bool isstarted;
};

#endif // SERIALCONNECT_H
