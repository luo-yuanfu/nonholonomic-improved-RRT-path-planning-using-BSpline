#include "my_class.h"
#include <math.h>
#include <stdlib.h>
#include <vector>

#define PI 3.1415926
#define CAR_LENGTH 0.1 //car length
#define INFINITY 1000.0
#define OBST_MARGIN 0.007
#define CTRL_VAL_A 0.40 //if a random number in (0,1) is less than CTRL_VAL_A, make goal be the new generated node.
#define CTRL_VAL_B 500 //control the number of loops. the higher CTRL_VAL_B is, the more accurate the algorithm is
#define CTRL_VAL_C 0.50
#define RAND_TIMES 20
#define NUM_ANGLE_DIVEDED 12
#define TOLERANCE 0.06 //if the distance between one node and the goal is smaller than TOLERANCE, consider this node is goal
#define MAX_STEERING_ANGLE (PI/4.0)

#define POINT_NUM_IN_ONE_CURVE 10

std::vector<obstacles> obsts;
std::vector<node> map; //use vector to build a tree (road map)
double dt=1.0; //delta t. unit time
double v[2]={0.1, -0.1}; //velocity of the car. -0.1 means reversing

bool xor(bool a, bool b)
{
	return ((a && !b) || (!a && b));
}

bool isCCW(double x0,double y0, double x1, double y1, double x2,double y2) 
{
	vect u,v;
	u.setValue(x1-x0,y1-y0);
	v.setValue(x2-x1,y2-y1);
	return (u.x*v.y - v.x*u.y)>0;
}

bool isCoincident(double x0,double y0, double x1, double y1, double x2,double y2, double x3, double y3 ) 
{
	return xor(isCCW(x0,y0,x1,y1,x2,y2),
	isCCW(x0,y0,x1,y1,x3,y3)) &&
	xor(isCCW(x2,y2,x3,y3,x0,y0),
	isCCW(x2,y2,x3,y3,x1,y1));
}

bool edgeCollision(double x0,double y0, double x1, double y1)//used to check whether the edge of smoothed path collides with obstacles
{
	for(int i=0;i<obsts.size();i++)
	{
		if( isCoincident(x0,y0,x1,y1,obsts.at(i).x0-OBST_MARGIN,obsts.at(i).y0-OBST_MARGIN,obsts.at(i).x0-OBST_MARGIN,obsts.at(i).y1+OBST_MARGIN) ||
			isCoincident(x0,y0,x1,y1,obsts.at(i).x0-OBST_MARGIN,obsts.at(i).y1+OBST_MARGIN,obsts.at(i).x1+OBST_MARGIN,obsts.at(i).y1+OBST_MARGIN) ||
			isCoincident(x0,y0,x1,y1,obsts.at(i).x1+OBST_MARGIN,obsts.at(i).y0-OBST_MARGIN,obsts.at(i).x1+OBST_MARGIN,obsts.at(i).y1+OBST_MARGIN) ||
			isCoincident(x0,y0,x1,y1,obsts.at(i).x0-OBST_MARGIN,obsts.at(i).y0-OBST_MARGIN,obsts.at(i).x1+OBST_MARGIN,obsts.at(i).y0-OBST_MARGIN)) return true;

	}

	return false;
}

bool outOfBoundary(car_conf p)
{
	/*
	double center_x,center_y,radius,dist;
	center_x=p.x+(CAR_LENGTH/2.0)*cos(p.theta);
	center_y=p.y+(CAR_LENGTH/2.0)*sin(p.theta);
	radius=1.12*CAR_LENGTH;

	if(center_x<(-1.0)*radius || center_x>radius+1.0 
		|| center_x<(-1.0)*radius || center_x>radius+1.0)
		return true;
	*/
	if(p.x<0 || p.x>1 || p.y<0 || p.y>1) return true;
	return false;
}

bool collideWithObst(car_conf p)//check whether sample point collides with obstacles
{
	
/*	double center_x,center_y,radius,dist;
	center_x=p.x+(CAR_LENGTH/2.0)*cos(p.theta);
	center_y=p.y+(CAR_LENGTH/2.0)*sin(p.theta);
	radius=1.12*CAR_LENGTH;

	for(int i=0;i<obsts.size();i++)
	{
		obstacles obst=obsts.at(i);

		if(abs(obst.x0-center_x)<radius ||
			abs(obst.x1-center_x)<radius ||
			abs(obst.y0-center_y)<radius ||
			abs(obst.y1-center_y)<radius ) return true;

	}
*/
	bool x_in_obst;
	bool y_in_obst;

	for(int i=0;i<obsts.size();i++)
	{
		obstacles obst=obsts.at(i);
		if(obsts.at(i).x1<obsts.at(i).x0)
		{
			x_in_obst = ( (p.x>=obst.x1-OBST_MARGIN) && (p.x<=obst.x0+OBST_MARGIN) );
		}
		else
		{
			x_in_obst = ((p.x>=obst.x0-OBST_MARGIN) && (p.x<=obst.x1+OBST_MARGIN));
		}

		if(obsts.at(i).y0<obsts.at(i).y1)
		{
			y_in_obst= ((p.y>=obst.y0-OBST_MARGIN) && (p.y<=obst.y1+OBST_MARGIN));
		}
		else
		{
			y_in_obst= ((p.y>=obst.y1-OBST_MARGIN) && (p.y<=obst.y0+OBST_MARGIN));
		}

		if(x_in_obst &&  y_in_obst) return true;

	}
	return false;
}

double distance(car_conf p, car_conf q)
{
	double distance;
	double dtheta;

	dtheta=p.theta-q.theta;
	//ensure dtheta in range of (-PI,PI)
	if (dtheta>PI)
		dtheta-=2*PI;
	else if (dtheta<-PI)
		dtheta+=2*PI;

	distance = pow((p.x-q.x)*(p.x-q.x)+ (p.y-q.y)*(p.y-q.y)+ CAR_LENGTH*CAR_LENGTH*dtheta*dtheta,0.5);

	return(distance);
}


node nearestNode(car_conf q)
{

	car_conf  q_temp;
	node nearest_node;
	double d=INFINITY,d_temp;

	/*
	for (std::vector<node>::iterator it=map.begin();it!=map.end();++it)
	{
		q_temp=it->conf;
		d_temp=distance(q_temp, q);
		if (d_temp<d)
		{
			nearest_node=*it;
			d=d_temp;
		}
	}
	*/
	for(int i=0;i<map.size();i++)
	{
		q_temp=map.at(i).conf;
		d_temp=distance(q_temp, q);
		if (d_temp<d)
		{
			nearest_node=map.at(i);
			d=d_temp;
		}
	}

	return nearest_node;

}



car_conf randConf()
{
	car_conf q;

	q.x=(rand()%100)/100.0;

	q.y=(rand()%100)/100.0;

	q.theta=(rand()%100)/100.0;
	q.theta=q.theta*2*PI-PI; //so q.theta is in range (-pi, pi)

	return(q);

}


//generate a new configuration near goal region with higher possibility than those of other regions
car_conf randConfNearGoal(car_conf goal, double d) 
{
	double rand_num=(rand()%100)/100.0;

	car_conf q;

	if (rand_num<CTRL_VAL_A) //make the node we want to generate be the goal with a probabilisty of 5% (CTRL_VAL_A==0.05) 
		return (goal);

	else if (rand_num< CTRL_VAL_C)
	{ //make the node we want to generate be within the distance of 2d of goal node with a probabilisty of 45%
		q=randConf();
		while (distance(q,goal)>2*d)
			q=randConf();
		return(q);
	}

	else return(randConf()); //gnenerate a random node with a probabilisty of 50%

}


car_conf newConf(car_conf near_conf,car_steering opt_vdir)
{
	car_conf new_conf;

	new_conf.x = near_conf.x+opt_vdir.v*dt*cos(near_conf.theta)*cos(opt_vdir.phi);  
	new_conf.y = near_conf.y+opt_vdir.v*dt*sin(near_conf.theta)*cos(opt_vdir.phi);
	new_conf.theta = near_conf.theta+(opt_vdir.v*dt/CAR_LENGTH)*sin(opt_vdir.phi);

	return(new_conf);
}


//optimize the velocity and direction to make the cost of going from nearest_conf to aim_conf least. use normal extension, not greedy extension
car_steering optmVelocityDir(car_conf nearest_conf, car_conf aim_conf)
{
	double dist1, dist2, dist_temp, d=INFINITY;
	bool collision_1=false, collision_2=false;
	car_conf optm_conf, conf_new_1, conf_new_2, conf_temp;
	car_steering optm_v_dir, v_dir_1, v_dir_2, v_dir;

	double phi[3];

	//three possible optimal steering angles
	phi[0]=atan(CAR_LENGTH*(nearest_conf.theta-aim_conf.theta)/ ((nearest_conf.x-aim_conf.x)*cos(nearest_conf.theta)+(nearest_conf.y-aim_conf.y)*sin(nearest_conf.theta)));
	phi[1]=atan(CAR_LENGTH*(nearest_conf.theta-aim_conf.theta+2*PI)/ ((nearest_conf.x-aim_conf.x)*cos(nearest_conf.theta)+(nearest_conf.y-aim_conf.y)*sin(nearest_conf.theta)));
	phi[2]=atan(CAR_LENGTH*(nearest_conf.theta-aim_conf.theta-2*PI)/ ((nearest_conf.x-aim_conf.x)*cos(nearest_conf.theta)+(nearest_conf.y-aim_conf.y)*sin(nearest_conf.theta)));

	for (int i=0;i<2;i++)
	{// for two directions
		for (int j=0;j<3;j++)
		{//for three angles
			v_dir.v=v[i];
			v_dir.phi=phi[j];
			if (v_dir.phi<-MAX_STEERING_ANGLE)
				v_dir.phi=-MAX_STEERING_ANGLE;
			else if (v_dir.phi>MAX_STEERING_ANGLE)
				v_dir.phi=MAX_STEERING_ANGLE;
			conf_temp=newConf(nearest_conf,v_dir);
			dist_temp=distance(conf_temp, aim_conf);
			if(dist_temp<d)
			{
				d=dist_temp;
				optm_conf=conf_temp;
				optm_v_dir=v_dir;
			}		
		}
	}


	v_dir_1=optm_v_dir;
	v_dir_2=optm_v_dir;


	//if (outOfBoundary(optm_conf)==true || collideWithObst(optm_conf)==true)
	if (outOfBoundary(optm_conf)==true || edgeCollision(nearest_conf.x,nearest_conf.y,optm_conf.x,optm_conf.y)==true)
	{ //if the car is out of boundary or collides with obstacles, try to modify the angles in two directions(see while sentence below)
		v_dir_1.phi=v_dir_1.phi+PI/NUM_ANGLE_DIVEDED;
		v_dir_2.phi=v_dir_2.phi-PI/NUM_ANGLE_DIVEDED;
		conf_new_1=newConf(nearest_conf,v_dir_1);
		conf_new_2=newConf(nearest_conf,v_dir_2);
		collision_1=true;
		collision_2=true;
	}


	//try to adjust phi to avoid collisions:

	while(collision_1==true)
	{
		collision_1=false;
		dist1=distance(conf_new_1, aim_conf);

		//if(outOfBoundary(conf_new_1)==true || collideWithObst(conf_new_1)==true)
		if (outOfBoundary(optm_conf)==true || edgeCollision(nearest_conf.x,nearest_conf.y,conf_new_1.x,conf_new_1.y)==true)
		{
			v_dir_1.phi=v_dir_1.phi+PI/NUM_ANGLE_DIVEDED;
			conf_new_1=newConf(nearest_conf,v_dir_1);
			collision_1=true;
		}
		
		if(abs(v_dir_1.phi)>MAX_STEERING_ANGLE)
		{//if adding phi cannot get a collision-free move
			dist1=INFINITY;
			break;
		}
	}

	while(collision_2==true)
	{
		collision_2=false;
		dist2=distance(conf_new_2, aim_conf);

		//if(outOfBoundary(conf_new_2)==true || collideWithObst(conf_new_2)==true)
		if (outOfBoundary(optm_conf)==true || edgeCollision(nearest_conf.x,nearest_conf.y,conf_new_2.x,conf_new_2.y)==true)
		{
			v_dir_2.phi=v_dir_2.phi-PI/NUM_ANGLE_DIVEDED;
			conf_new_2=newConf(nearest_conf,v_dir_2);
			collision_2=true;
		}
		
		if(abs(v_dir_2.phi)>MAX_STEERING_ANGLE)
		{//if substracting phi cannot get a collision-free move
			dist2=INFINITY;
			break;
		}
	}

	if (dist1==INFINITY && dist2==INFINITY)
	{
		optm_v_dir.v=0;//don't move
		return(optm_v_dir);
	}		
	//dist2+=0.1;//fine for backing up
	if (dist1<=dist2)
		optm_v_dir=v_dir_1;
	else optm_v_dir=v_dir_2;
	return(optm_v_dir);

	

}

int withinDist(car_conf conf)
{
	car_conf  q_temp;
	double d_temp;
	int sum=0;
	for (std::vector<node>::iterator it=map.begin();it!=map.end();++it)
	{
		q_temp=it->conf;
		d_temp=distance(q_temp, conf);
		if (d_temp<dt*v[0]*4)
		{
			sum+=1;
		}
	}
	return sum;
}

bool worthExploring(car_conf parent_conf,car_conf new_conf)
{
	/*
	node nearest_node=nearestNode(new_conf);
	if(distance(nearest_node.conf,new_conf)==distance(parent_conf,new_conf)) return true;
	else if(distance(nearest_node.conf,new_conf)<distance(parent_conf,new_conf)/2.0) return false;
	else return true;
	*/
	int near_parent_num, near_new_conf_num;
	near_parent_num=withinDist(parent_conf);
	near_new_conf_num=withinDist(new_conf);

	if(near_new_conf_num<=near_parent_num*1.2) return true;

	else return false;
}

void buildRRT(car_conf start_conf, car_conf goal_conf )
{

	node node_temp;
	car_conf rand_conf, nearest_conf;
	node nearest_conf_node,new_conf_node;
	car_conf new_conf;
	car_steering new_steering;
	double dist=INFINITY;

	new_steering.setValue(0,0);
	node_temp.setValue(0,0,start_conf,new_steering);
	map.push_back(node_temp);//push start node into map as the first node
	int a1=0,a2=0,a3=0;
	

	while (dist>TOLERANCE) 
	{
		for(int i=0;i<CTRL_VAL_B; i++)
		{
			rand_conf=randConfNearGoal(goal_conf, dist);
			nearest_conf_node=nearestNode(rand_conf);
			new_steering=optmVelocityDir(nearest_conf_node.conf,rand_conf);
			if(new_steering.v==0){continue;}
					
			new_conf=newConf(nearest_conf_node.conf,new_steering); 

			if(!worthExploring(nearest_conf_node.conf,new_conf)) 
			{
				if((rand()%100)/100.0>1.0)continue;
				//a2++;
				//continue;
			} 
		
			new_conf_node.setValue(map.size(),nearest_conf_node.ID,new_conf,new_steering);
			map.push_back(new_conf_node);

		}

		nearest_conf_node=nearestNode(goal_conf);
		dist = distance(nearest_conf_node.conf, goal_conf);
	
	}

}



//std::vector<obstacles>map is a tree, i.e., no circles in map, and every node knows its father but doesn't know its children
std::vector<node> getPath(car_conf goal_conf)
{
	node nearest_goal, start_node;
	std::vector<node> path;

	nearest_goal=nearestNode(goal_conf);//here, suppose that the node nearest to goal_conf is goal_conf(cuz the distance is very samll)

	start_node = map.front();//the first node of G is start_conf, we can see that from buildRRT().

	path.push_back(nearest_goal);

	while (nearest_goal.equal(start_node)==false)
	{
		nearest_goal=map.at(nearest_goal.father_ID);
		path.push_back(nearest_goal);
	}

	path.at(0).conf.setValue(goal_conf.x,goal_conf.y,goal_conf.theta);

	return(path);
}


double pathLength(std::vector<node> path)
{
	double length=0.0;
	for(int i=path.size()-1;i>=1;i--)
	{
		length+=pow((path.at(i).conf.x-path.at(i-1).conf.x)*(path.at(i).conf.x-path.at(i-1).conf.x)+(path.at(i).conf.y-path.at(i-1).conf.y)*(path.at(i).conf.y-path.at(i-1).conf.y),0.5);
	}

	return length;
}

double euclidDistance(car_conf conf1, car_conf conf2)
{
	return pow((conf1.x-conf2.x)*(conf1.x-conf2.x)+(conf1.y-conf2.y)*(conf1.y-conf2.y),0.5);
}

int numOfNeighbors(std::vector<node> path,int k,std::vector<int> &index,int dist)
{
	int num=0;
	for(int i=k-1;i>=0 && i>=k-12;i--)
	{
		if(euclidDistance(path.at(k).conf,path.at(i).conf)<(dt*v[0])*dist)
		{
			num++;
			index.push_back(i);
		}
	}
	return num;
}

bool okToBeShorter(std::vector<node> shorter_path,std::vector<node> path,std::vector<int> index,int k)
{// |cos(new_theta)|>cos(MAX_STEERING_ANGLE)

	vect head,body,tail;
//	bool goal_node=false, start_node=false;
	double cos_head_body, cos_body_tail;

	if(index.at(k)>0) head.setValue(path.at(index.at(k)-1).conf.x-path.at(index.at(k)).conf.x,path.at(index.at(k)-1).conf.y-path.at(index.at(k)).conf.y);
	else  head.setValue(cos(path.at(0).conf.theta),sin(path.at(0).conf.theta)); //goal_node=true;

	body.setValue(path.at(index.at(k)).conf.x-shorter_path.back().conf.x,path.at(index.at(k)).conf.y-shorter_path.back().conf.y);

	if(shorter_path.size()>1) tail.setValue(shorter_path.back().conf.x-shorter_path.at(shorter_path.size()-2).conf.x,shorter_path.back().conf.y-shorter_path.at(shorter_path.size()-2).conf.y);
	else tail.setValue(cos(shorter_path.at(0).conf.theta),sin(shorter_path.at(0).conf.theta));//start_node=true;

	cos_head_body=(head.x*body.x+head.y*body.y)/(pow((head.x*head.x+head.y*head.y)*(body.x*body.x+body.y*body.y),0.5));
	cos_body_tail=(tail.x*body.x+tail.y*body.y)/(pow((tail.x*tail.x+tail.y*tail.y)*(body.x*body.x+body.y*body.y),0.5));

	if(abs(cos_head_body)>=cos(MAX_STEERING_ANGLE) && abs(cos_body_tail)>=cos(MAX_STEERING_ANGLE) 
		&& !edgeCollision(path.at(index.at(k)).conf.x,path.at(index.at(k)).conf.y,shorter_path.back().conf.x,shorter_path.back().conf.y)) return true;
	else return false;
}

void shorter(std::vector<node> &shorter_path,std::vector<node> &path,std::vector<int> index,int &i)
{
	for(int k=index.size()-1;k>=0;k--)
	{
		if(okToBeShorter(shorter_path,path,index,k))
		{
			i=index.at(k);
			path.at(i).father_ID=shorter_path.back().ID;
			i++;
			break;//the new node path.at(i) will be pushed into smooth_path in function smoothPath
		}
	}
}

std::vector<node> shorterPath(std::vector<node> path,int dist)
{
	std::vector<node> shorter_path;
	std::vector<int> index;
	for(int i=path.size()-1;i>=0;i--)
	{
		shorter_path.push_back(path.at(i));

		if(numOfNeighbors(path,i,index,dist)>=1) 
		{
			shorter(shorter_path,path,index,i);
		}

		index.clear();
	}

	return shorter_path;
}

std::vector<node> inverseVector(std::vector<node> smooth_path)
{
	std::vector<node> new_vector;
	for(int i=smooth_path.size()-1;i>=0;i--)
		new_vector.push_back(smooth_path.at(i));

	return new_vector;
}



double g02(double t)
{
	return 1.0/2.0*(t-1.0)*(t-1.0);
}

double g12(double t)
{
	return 1.0/2.0*(-2.0*t*t+2*t+1.0);
}

double g22(double t)
{
	return 1.0/2.0*t*t;
}


point pkn(double x0,double y0,double x1,double y1,double x2,double y2,double t)
{
	point interp_poi;
	interp_poi.setValue(x0*g02(t)+x1*g12(t)+x2*g22(t),y0*g02(t)+y1*g12(t)+y2*g22(t));
	return interp_poi;
}

std::vector<point> B_Spline(std::vector<node> smooth_path)
{
	std::vector<point> new_smooth_path;
	double t=0;
	point temp;
	temp.setValue(smooth_path.at(0).conf.x,smooth_path.at(0).conf.y);
	new_smooth_path.push_back(temp);

	vect body,tail;
	double cos_body_tail;

	for(int i=0;i<=smooth_path.size()-3;i++)
	{
		body.setValue(smooth_path.at(i+1).conf.x-smooth_path.at(i).conf.x,smooth_path.at(i+1).conf.y-smooth_path.at(i).conf.y);
		tail.setValue(smooth_path.at(i+2).conf.x-smooth_path.at(i+1).conf.x,smooth_path.at(i+2).conf.y-smooth_path.at(i+1).conf.y);
		cos_body_tail=(tail.x*body.x+tail.y*body.y)/(pow((tail.x*tail.x+tail.y*tail.y)*(body.x*body.x+body.y*body.y),0.5));
		if(cos_body_tail<0)
		{
			temp.setValue(smooth_path.at(i+1).conf.x,smooth_path.at(i+1).conf.y);
			new_smooth_path.push_back(temp);
			continue;
		}
		for(int j=-1;j<POINT_NUM_IN_ONE_CURVE;j++)
		{
			t=1.0/double(POINT_NUM_IN_ONE_CURVE)*double(j+1);
			new_smooth_path.push_back(pkn(smooth_path.at(i).conf.x,smooth_path.at(i).conf.y,smooth_path.at(i+1).conf.x,smooth_path.at(i+1).conf.y,smooth_path.at(i+2).conf.x,smooth_path.at(i+2).conf.y,t));
		}
		
	}
	temp.setValue(smooth_path.at(smooth_path.size()-1).conf.x,smooth_path.at(smooth_path.size()-1).conf.y);
	new_smooth_path.push_back(temp);
	return new_smooth_path;
}

double newPathLength(std::vector<point> path)
{
	double sum=0;
	for(int i=0;i<path.size()-1;i++)
		sum+=pow((path.at(i+1).x-path.at(i).x)*(path.at(i+1).x-path.at(i).x)+(path.at(i+1).y-path.at(i).y)*(path.at(i+1).y-path.at(i).y),0.5);

	return sum;

}