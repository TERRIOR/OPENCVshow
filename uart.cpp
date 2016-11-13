/*************************************************************************
    > File Name: uart.cpp
    > Author: ma6174
    > Mail: ma6174@163.com 
    > Created Time: 2016年08月10日 星期三 12时55分58秒
 ************************************************************************/

#include <wiringPi.h>
#include <wiringSerial.h>
#include<iostream>
#include<string>
using namespace std;
int main()
{
	int fd;
	string comdata="";
	if(wiringPiSetup() < 0)return 1;
	if((fd = serialOpen("/dev/ttyACM0",9600)) < 0)return 1;
	cout<<"test start"<<endl;
	//serialPrintf(fd,"Hello World!!!\n");
	while(1){  
		serialPrintf(fd,"hi");
		while(serialDataAvail()>0){
			comdata+=char(serialGetchar(fd));
		}
		if(comdata.length()>0)
		{
			cout<<comdata<<endl;
			comdata="";
		}
		delay(1000);
	} 
	serialClose(fd);
	return 0;
}
