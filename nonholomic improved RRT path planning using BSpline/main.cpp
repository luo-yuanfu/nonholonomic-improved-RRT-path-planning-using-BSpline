#include <stdlib.h>
#include <time.h>

#include "draw.h"



extern std::vector<obstacles> obsts;

int main()
{
	srand(time(NULL));

	car_conf start_conf,goal_conf;
	std::vector<node> path,shorter_path;
	double running_time;
	clock_t Beginning_Time;

	std::vector<point> smooth_path;
	double length3,smooth_length2,best_length=INFINITY;
	std::vector<point> best_path;

	loadData(start_conf, goal_conf);

	printf("enter the running time (second):\n");
	scanf("%lf",&running_time);
	Beginning_Time=clock();

	while((clock()-Beginning_Time)/CLOCKS_PER_SEC<running_time)
	{
		map.clear();
		path.clear();
		smooth_path.clear();

		buildRRT(start_conf, goal_conf);
		path=getPath(goal_conf);
		for(int i=5;i>=2;i--)
		{
			shorter_path.clear();
			shorter_path=shorterPath(path,i);
			path.clear();
			path=inverseVector(shorter_path);
		}

		for(int i=5;i<=8;i++)
		{
			shorter_path.clear();
			shorter_path=shorterPath(path,i);
			path.clear();
			path=inverseVector(shorter_path);
		}

		smooth_path=B_Spline(shorter_path);

		length3=newPathLength(smooth_path);

		if(length3<best_length)
		{
			best_length=length3;
			best_path=smooth_path;
		}
	}

	drawMap2(best_path,shorter_path,best_length);


	/*
	
	//printf("lenght: %lf %lf %lf",length,smooth_length1,smooth_length2);
	//drawMap(shorter_path,smooth_length2);

	*/
		
	/*
	printf("\n\n*****path nodes*****\n");
	for(int i=path.size()-1;i>=0;i--)
	{
		printf("%d %d %f %f\n",path.at(i).ID,path.at(i).father_ID,path.at(i).conf.x,path.at(i).conf.y);
	}
	*/

	/*
	printf("\n\n*****start and goal*****\n");
	printf("%lf %lf %lf\n",start_conf.x,start_conf.y,start_conf.theta);
	printf("%lf %lf %lf\n",goal_conf.x,goal_conf.y,goal_conf.theta);
	*/

	/*
	printf("\n\n*****obstacles*****\n");
	for(int i=0;i<obsts.size();i++)
	{
		printf("%lf %lf %lf %lf\n",obsts.at(i).x0,obsts.at(i).y0,obsts.at(i).x1,obsts.at(i).y1);
	}
	*/
	
	printf("press 'Enter' to end\n");
	getchar();

	return 0;
}
