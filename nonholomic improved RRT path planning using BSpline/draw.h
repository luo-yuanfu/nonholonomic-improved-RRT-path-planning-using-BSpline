#include "load_data.h"
#include <graphics.h>
#include <conio.h>


void drawPath(std::vector<node>path)
{
	/*
	for(int i=path.size()-1;i>0;i--)
	{
		
		fillcircle(int(path.at(i).conf.x*500),int(path.at(i).conf.y*500),3);
		//Sleep(500);
		if(path.at(i-1).v_dir.v<0) setcolor(YELLOW);
		line(int(path.at(i).conf.x*500),int(path.at(i).conf.y*500),int(path.at(i-1).conf.x*500),int(path.at(i-1).conf.y*500));
		setcolor(GREEN);
		
		
		
	}
	//Sleep(500);
	fillcircle(int(path.at(0).conf.x*500),int(path.at(0).conf.y*500),3);
	*/
	fillcircle(int(path.at(0).conf.x*500),int(path.at(0).conf.y*500),3);
	Sleep(300);
	for(int i=1;i<path.size();i++)
	{
		//if(path.at(i-1).v_dir.v<0) setcolor(YELLOW);
		line(int(path.at(i).conf.x*500),int(path.at(i).conf.y*500),int(path.at(i-1).conf.x*500),int(path.at(i-1).conf.y*500));
		//setcolor(GREEN);
		fillcircle(int(path.at(i).conf.x*500),int(path.at(i).conf.y*500),3);
		Sleep(300);		
		
	}
	
}

void drawObsts()
{
	
	setfillstyle(BROWN,5,NULL);
	for(int i=0;i<obsts.size();i++)
	{
		//fillpoly(2,int(obsts.at(i).x0*500),int(obsts.at(i).y0*500),int(obsts.at(i).x1*500),int(obsts.at(i).y1*500));
		//rectangle(int(obsts.at(i).x0*500),int(obsts.at(i).y0*500),int(obsts.at(i).x1*500),int(obsts.at(i).y1*500));
		bar(int(obsts.at(i).x0*500),int(obsts.at(i).y0*500),int(obsts.at(i).x1*500),int(obsts.at(i).y1*500));
	}
}

void drawMap(std::vector<node>path,double length)
{

	initgraph(500, 500);
	char *leng;
	char len[6];
	int dec, sign;
	setcolor(BROWN);
	drawObsts();

	setcolor(GREEN);
	drawPath(path);

	leng = ecvt(length, 6, &dec, &sign);
	strcpy(len,leng);
	strcat(len,"*");
	char decchar[2];
	decchar[0]=dec+48;
	decchar[1]='\0';
	strcat(len,decchar);
	outtextxy(0,480,len);
	
	getchar();

	closegraph();
}

void drawPath2(std::vector<point>path,std::vector<node> init_path)
{
	setcolor(WHITE);
	//fillcircle(int(path.at(0).x*500),int(path.at(0).y*500),3);
	//Sleep(300);
	for(int i=1;i<path.size();i++)
	{
		//if(path.at(i-1).v_dir.v<0) setcolor(YELLOW);
		line(int(path.at(i).x*500),int(path.at(i).y*500),int(path.at(i-1).x*500),int(path.at(i-1).y*500));
		//setcolor(GREEN);
		//fillcircle(int(path.at(i).x*500),int(path.at(i).y*500),3);
		//Sleep(300);		
		
	}

	
//	drawPath(init_path);
	
}


void drawMap2(std::vector<point>path,std::vector<node>init_path,double length)
{

	initgraph(500, 500);
	char *leng;
	char len[10];
	int dec, sign;
	setcolor(BROWN);
	drawObsts();

	setcolor(GREEN);
	drawPath2(path,init_path);

	leng = ecvt(length, 6, &dec, &sign);
	strcpy(len,leng);
	strcat(len,"*");
	char decchar[2];
	decchar[0]=dec+48;
	decchar[1]='\0';
	strcat(len,decchar);
	outtextxy(0,480,len);
	
	getchar();
	getchar();

	closegraph();
}