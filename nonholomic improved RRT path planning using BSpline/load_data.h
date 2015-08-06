#include "rrt.h"

extern std::vector<obstacles> obsts;


void loadData(car_conf &start_conf, car_conf &goal_conf)
{
	double x,y,theta;
	FILE *p;
	char temp[30];
	char c;
	obstacles obst;

	int test_index;
	
	char filename[100]="D:\\MyWorkspace\\c\\car-like motion planning based on RRT\\data_files\\test";
	printf("which obstacle file do you want to use? choose one:\n1.test1\n2.test2\n3.test3\n4.test4\n5.test5\n6.test6\n");
	scanf("%d",&test_index);
	char tempname[5];
	tempname[0]=test_index+48;
	tempname[1]='.';
	tempname[2]='m';
	tempname[3]='p';
	tempname[4]='\0';
	strcat(filename,tempname);

	if((p=fopen(filename,"r"))==NULL)
	{
		printf("cannot open files\n");
		exit(0);
		
	}

	fgets(temp,30,p);
	fscanf(p,"%lf %lf %lf",&start_conf.x,&start_conf.y,&start_conf.theta);
	fgetc(p);
	fgets(temp,30,p);

	fgetc(p);

	fgets(temp,30,p);
	fscanf(p,"%lf %lf %lf",&goal_conf.x,&goal_conf.y,&goal_conf.theta);
	fgetc(p);
	fgets(temp,30,p);
	c=fgetc(p);
	if(c==EOF) {fclose(p);return;}

	int carriage_num=0;
	while(carriage_num!=2)
	{
		fgets(temp,30,p);
		fscanf(p,"%lf %lf",&obst.x0,&obst.y0);
		fgetc(p);
		fscanf(p,"%lf %lf",&obst.x0,&obst.y1);
		fgetc(p);
		fscanf(p,"%lf %lf",&obst.x1,&obst.y1);
		fgetc(p);
		fscanf(p,"%lf %lf",&obst.x1,&obst.y0);
		fgetc(p);
		fgets(temp,30,p);
		if(obst.x0>obst.x1)
		{
			double temp=obst.x0;
			obst.x0=obst.x1;
			obst.x1=temp;
		}
		if(obst.y0>obst.y1)
		{
			double temp=obst.y0;
			obst.y0=obst.y1;
			obst.y1=temp;
		}
		if((c=fgetc(p))=='\n') carriage_num++;
		if(c==EOF) break;
		if((c=fgetc(p))=='\n') carriage_num++;
		else carriage_num=0;

		obsts.push_back(obst);
	}

	fclose(p);
}