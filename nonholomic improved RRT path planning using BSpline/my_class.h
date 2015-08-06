class car_conf //configuratioin of car
{public:
	double x;
	double y;
	double theta;

	void setValue(double a,	double b, double c)
	{
		x=a;
		y=b;
		theta=c;
	}
};

class car_steering //drive with speed v and steering angle phi from one configuration to another
{public:
	double v;
	double phi;

	void setValue(double a,	double b)
	{
		v=a;
		phi=b;
	}
};

class node
{public:
	int ID;//its own index in the vector<node>map
	int father_ID;//its father's index in the vector<node>map
	car_conf conf;
	car_steering v_dir;//the velocity and steering angle that its previous node(i.e., its father node) uses to
						//get to the current node(i.e., child node (itself))

	void setValue(int ID, int father_ID,car_conf conf,car_steering v_dir)
	{
		this->ID=ID;
		this->father_ID=father_ID;
		this->conf=conf;
		this->v_dir=v_dir;
	}

	bool equal(node compare_node)
	{
		if(compare_node.ID==this->ID) return true;

		return false;
	}
};


class obstacles
{public:
	double x0;
	double y0;
	double x1;
	double y1;

	void setValue(double x0,double y0,double x1,double y1)
	{
		this->x0=x0;
		this->y0=y0;
		this->x1=x1;
		this->y1=y1;
	}
};

class vect//vector
{public:
	double x;
	double y;
	void setValue(double x,double y)
	{
		this->x=x;
		this->y=y;
	}
};

class point//point
{public:
	double x;
	double y;
	void setValue(double x,double y)
	{
		this->x=x;
		this->y=y;
	}
};



