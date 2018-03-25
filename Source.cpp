#include<iostream>
#include<fstream>
#include<string>
#include<exception>
#include<cstdio>
#include<cmath>
#include"opencv2\core\core.hpp"
#include"opencv2\highgui\highgui.hpp"
#include"opencv2\imgproc\imgproc.hpp"
#using<system.dll>
using namespace System;
using namespace System::IO;
using namespace System::IO::Ports;
using namespace cv;
using namespace std;

#define _BLUE 0
#define _GREEN 1
#define _RED 2

typedef struct PointS
	{Point2f pointc;
	int size;
	PointS(Point2f point,int SIZE)
		{pointc=point;
		size=SIZE;
		}
	} POINT_S;

namespace
{
const string FILE_POS("C:\\Users\\rupa\\Desktop\\escapist_simulator\\pos.txt");
const string FILE_EXPORT("C:\\Users\\rupa\\Desktop\\errr\\");

char mymsg[20];

string full;
ofstream myfile;
POINT_S temp(Point2f(),0);
Mat frame;
vector<Mat> splitframe;
vector<vector<Point>> contsBorder;
vector<Vec4i> hierarchy;
Rect rec;
vector<POINT_S> BlueBot, RedBot, GreenBot;
float mindist=0; int mindistindex=0;
int j=0;
int ERRRADIUS=100;
float orient_targG,orient_mybotG,mybot_targG;
float mybot_targR;
float slopewithhead,slopewithbody;
int errorsign;
bool firstgreen=true,firstred=true,gototop=false;
int SPEEDG = 180,SPEEDR=250;
}
namespace PID
{
	int KpG=9,KdG=15,KiG=0;
	int KpR=35,KdR=30,KiR=0;
	float error=0,preverr=0,rate=0,d=0,i=0,integral=0,corr=0,poscor,negcor;
}


void callBackFunc(int x,void* data)
{
	*(int*)data = x;
}


string getstring(int value, int length)
{
	string filen;
	filen=::to_string(value);
	int LENGTH=filen.length();

	if(LENGTH<length)
	{
		for(int i=1;i<=length-LENGTH;i++)
			filen="0" + filen;
	}
	return filen;
}

void movebot(SerialPort^ sp)
{
	System::String^ str2 = gcnew System::String(full.c_str());
	sp->Write(str2);
	_sleep(1);
}

void move(SerialPort^ sp,int lspeed, int rspeed)
{
	string L_S=getstring(lspeed,3);
	string R_S=getstring(rspeed,3);
	//string B1x=getstring(BlueBot[1].pointc.x,3);
	//string B1y=getstring(BlueBot[1].pointc.y,3);
	//string B0x=getstring(BlueBot[0].pointc.x,3);
	//string B0y=getstring(BlueBot[0].pointc.y,3);
	full=L_S+"#"+R_S;//+"#"+B1x+"#"+B1y+"#"+B0x+"#"+B0y;
	try
	{
	//myfile.open (FILE_POS);
	//myfile << L_S << " " << R_S;
	//myfile.close();
	movebot(sp);

	}
	catch(exception e)
	{
	
	}
}


void deleteFiles(int index)
{
	string pathDel;
	static int lastindex=1;
	try
	{
		for(;lastindex<=index-2;lastindex++)
		{
			pathDel = FILE_EXPORT + "es_" + getstring(lastindex,4) + ".jpg";
			remove(pathDel.c_str());
		}
	}
	catch(exception e)
	{
	}
}

string getFileName()
{
	static int index=1;
	string indexS,path;
	while(true)
	{
		indexS=getstring(index,4);
		path = FILE_EXPORT + "es_" + indexS + ".jpg";
		if(!ifstream(path).good())
		{
			indexS=getstring(index-1,4);
			path = FILE_EXPORT + "es_" + indexS + ".jpg";
			break;
		}
		index++;
	}
	//deleteFiles(index);
	cout<< path<<endl;
	return path;
}

float getDist(Point2f p1, Point2f p2)
{
	return sqrt(((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y)));
}

float scaleSpeed(int leftspeed, int rightspeed)
{
	return ((2.9/255)*((leftspeed>rightspeed)?leftspeed:rightspeed));
}

float distFromLine(Point2f pointLine1,Point2f pointLine2, Point2f targetPoint)
{   
	float a=pointLine2.y-pointLine2.y;
	float b=pointLine1.x-pointLine2.x;
	float c=-b*pointLine1.y - a*pointLine1.x;
	float dist=abs(a*targetPoint.x+b*targetPoint.y+c)/sqrt(a*a+b*b);
	return dist;
}

void pid(SerialPort^ sp,float error,int Kp,int Kd, int Ki,int SPEED)
{
	PID::rate=error-PID::preverr;
	PID::d=Kd*PID::rate;
	PID::i+=error;
	PID::integral=Ki*PID::i;
	PID::corr=(Kp*error) + PID::d + PID::integral;
	PID::preverr=error;
	PID::poscor=SPEED+PID::corr;
	PID::negcor=SPEED-PID::corr;
	if(PID::poscor>255)
		PID::poscor=255;
	if (PID::poscor<0)
		PID::poscor=0;
	if(PID::negcor<0)
		PID::negcor=0;
	if(PID::negcor>255)
		PID::negcor=255;
	//cout<< PID::poscor<<" " <<PID::negcor<<endl;
	move(sp,PID::poscor,PID::negcor);

}

void main() 
{
	//float distnow[2],distlast[2]={0};
	//bool ftime = true;
	namedWindow("BLUE");
	namedWindow("RED");
	namedWindow("GREEN");
	SerialPort^ sp=gcnew SerialPort("COM12",9600);
	//sp->WriteTimeout=500;
	//sp->ReadTimeout=500;
	sp->Open();

	namedWindow("ControlG");
	namedWindow("ControlR");
	
	createTrackbar("SpeedG","ControlG",&SPEEDG,255,callBackFunc,&SPEEDG);
	createTrackbar("KpG","ControlG",&PID::KpG,100,callBackFunc,&PID::KpG);
	createTrackbar("KdG","ControlG",&PID::KdG,100,callBackFunc,&PID::KdG);
	createTrackbar("KiG","ControlG",&PID::KiG,100,callBackFunc,&PID::KiG);

	createTrackbar("SpeedR","ControlR",&SPEEDR,255,callBackFunc,&SPEEDR);
	createTrackbar("KpR","ControlR",&PID::KpR,100,callBackFunc,&PID::KpR);
	createTrackbar("KdR","ControlR",&PID::KdR,100,callBackFunc,&PID::KdR);
	createTrackbar("KiR","ControlR",&PID::KiR,100,callBackFunc,&PID::KiR);
	createTrackbar("Radius","ControlR",&ERRRADIUS,200,callBackFunc,&ERRRADIUS);
	
	while(true)
	{
		
		try
		{
			GreenBot.clear();
			BlueBot.clear();
			RedBot.clear();

			frame=imread(getFileName());
			split(frame,splitframe);
			inRange(frame,Scalar(CV_RGB(150,0,0)),Scalar(CV_RGB(255,50,50)),splitframe[_RED]);
			findContours(splitframe[_RED],contsBorder,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

			for (int i=0;i<contsBorder.size();i++)
				{
					rec = boundingRect(contsBorder[i]);
					RedBot.push_back(POINT_S(Point2f(rec.x+rec.width/2.0,rec.y+rec.height/2.0),rec.area()));
				}

			inRange(frame,Scalar(CV_RGB(0,150,0)),Scalar(CV_RGB(50,255,50)),splitframe[_GREEN]);
			findContours(splitframe[_GREEN],contsBorder,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	
			for (int i=0;i<contsBorder.size();i++)
				{
					rec = boundingRect(contsBorder[i]);
					GreenBot.push_back(POINT_S(Point2f(rec.x+rec.width/2.0,rec.y+rec.height/2.0),rec.area()));
				}

			if(contsBorder.size()<1)
				GreenBot.push_back(POINT_S(Point2f(250,250),0));

			inRange(frame,Scalar(CV_RGB(0,0,255)),Scalar(CV_RGB(50,50,255)),splitframe[_BLUE]);
			GaussianBlur(splitframe[_BLUE],splitframe[_BLUE],Size(7,7),1,1);
			findContours(splitframe[_BLUE],contsBorder,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

			for (int i=0;i<contsBorder.size();i++)
					{
						rec = boundingRect(contsBorder[i]);
						BlueBot.push_back(POINT_S(Point2f(rec.x+rec.width/2.0,rec.y+rec.height/2.0),rec.area()));
					}
	
			imshow("BLUE",splitframe[_BLUE]);
			waitKey(1);
			imshow("RED",splitframe[_RED]);
			waitKey(1);
			imshow("GREEN",splitframe[_GREEN]);
			waitKey(1);
	continue;
			try{
					if(BlueBot[0].size>BlueBot[1].size)
					{
						temp=BlueBot[1];
						BlueBot[1]=BlueBot[0];
						BlueBot[0]=temp;
					}
				}
			catch(exception e)
				{
				}

			gototop=false;

			mindist=getDist(BlueBot[1].pointc,RedBot[0].pointc);

			for(j=0;j<RedBot.size();j++)
				{
					mybot_targR = getDist(BlueBot[1].pointc,RedBot[j].pointc);
					if(mybot_targR<=mindist)
						{
							mindist=mybot_targR;
							mindistindex=j;
						}
				}

			if(mindist<ERRRADIUS)
				{
					slopewithhead=(RedBot[mindistindex].pointc.y-BlueBot[0].pointc.y)/(RedBot[mindistindex].pointc.x-BlueBot[0].pointc.x);
					slopewithbody=(RedBot[mindistindex].pointc.y-BlueBot[1].pointc.y)/(RedBot[mindistindex].pointc.x-BlueBot[1].pointc.x);
					//errorsign - right:-1 left:1
					errorsign=(slopewithhead-slopewithbody>0)?-1:1;
					PID::error=((ERRRADIUS*2)/mindist)*errorsign;
			
					if(firstred)
						{
							PID::preverr=0;
							firstred=false;
							firstgreen=true;
						}
			
					cout<<"RED!!!!!!!!! " <<mindistindex<<endl;//<<" " <<PID::error<<" "<<PID::preverr<<endl;;

					pid(sp,PID::error,PID::KpR,PID::KdR,PID::KiR,SPEEDR);

					gototop=true;
				}
			else
					firstred=true;
		
			if(gototop)
				continue;


			orient_targG = getDist(GreenBot[0].pointc,BlueBot[0].pointc);
			orient_mybotG = getDist(BlueBot[1].pointc,BlueBot[0].pointc);
			mybot_targG = getDist(BlueBot[1].pointc,GreenBot[0].pointc);

			slopewithhead=(GreenBot[0].pointc.y-BlueBot[0].pointc.y)/(GreenBot[0].pointc.x-BlueBot[0].pointc.x);
			slopewithbody=(GreenBot[0].pointc.y-BlueBot[1].pointc.y)/(GreenBot[0].pointc.x-BlueBot[1].pointc.x);

			//errorsign - right:1 left:-1
			errorsign=(slopewithhead-slopewithbody>0)?1:-1;
		
			//cout<< getDist(BlueBot[1].pointc,RedBot[0].pointc)<<" " <<getDist(BlueBot[1].pointc,RedBot[1].pointc)<<endl;
	
			/*
			while((getDist(BlueBot[1].pointc,RedBot[0].pointc)<70)||(getDist(BlueBot[1].pointc,RedBot[1].pointc)<70))
			{
				distnow[0]=distFromLine(BlueBot[0].pointc,BlueBot[1].pointc,RedBot[0].pointc);
				distnow[1]=distFromLine(BlueBot[0].pointc,BlueBot[1].pointc,RedBot[1].pointc);
				if(!(distnow[0]>distlast[0]))
					go_left(0,SPEED);
				else
					go_forward();
				if(!(distnow[1]>distlast[1]))
					go_left(0,SPEED);
				else
					go_forward();
				distlast[0]=distnow[0];
				distlast[1]=distnow[1];
			}
			*/

			//PID::error=distFromLine(BlueBot[1].pointc,GreenBot[0].pointc,BlueBot[0].pointc)*errorsign;

			PID::error=abs((orient_mybotG+orient_targG)-mybot_targG)*errorsign;

			if (firstgreen)
				{
					PID::preverr=0;
					firstgreen=false;
				}

			if(abs((orient_mybotG+orient_targG)-mybot_targG)<scaleSpeed(0,SPEEDG))
				cout << "Oriented " <<endl;// PID::error<<" "<<PID::preverr<<endl;
			else
				cout<<"Not oriented "<<endl;//PID::error<<" "<<PID::preverr<<endl;
			
			pid(sp,PID::error,PID::KpG,PID::KdG,PID::KiG,SPEEDG);
		
			waitKey(10);
			}//end of try
		catch(exception e)
			{
			}
		} //end of while
}


