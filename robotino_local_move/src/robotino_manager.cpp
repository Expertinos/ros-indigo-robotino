#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include "robotino_local_move/FullPath.h"

#include <iostream>
#include <iomanip>
#include <queue>
//#include <string>
#include <math.h>
#include <ctime>

using namespace std;

int i_ant = -1;
int xA, yA, goal;
float posX, posY, posTheta;
const int w=21; // vertical size size of the map
const int m=11; // horizontal size of the map
static int myMap[w][m] = {  {99,99,99,99,99,99,99,99,99,99,99},    //  -5,60        DEPOSITOS: D1 = 51
                            {99,99, 0, 0, 0,52, 0, 0,24,99,99},    //  -5,04        DEPOSITOS: D2 = 52
                            {99,99, 0,21, 0, 0, 0, 0, 0, 0,99},    //  -4,48
                            {99,99, 0,99, 0,99,22, 0,23,99,99},    //  -3,92        PUCK AREA TEAM 1: 41 e 43
                            {99,99, 0, 0, 0, 0, 0, 0, 0, 0,99},    //  -3,36        PUCK AREA TEAM 2: 42 e 44
                            {99,99, 0,99,18, 0,19,99, 0,99,99},    //  -2,80
                            {99,99,42,15, 0, 0, 0, 0, 0,20,99},    //  -2,24        RECYCLE MACHINE TEAM 1: 31
                            {99,99, 0,99, 0,99, 0,99, 0, 0,99},    //  -1,68        RECYCLE MACHINE TEAM 2: 32
                            {99,99,44, 0, 0,16, 0,17, 0,32,99},    //  -1,12
                            {99, 0,13,99, 0,99,14, 0, 0,99,99},    //  -0,56
                            {99, 0, 0, 0, 0, 0, 0, 0, 0, 0,99},    //  0
                            {99, 0, 1,99, 0,99, 2, 0, 0,99,99},    //  0,56
                            {99,99,43, 0, 0, 4, 0, 5, 0,31,99},    //  1,12
                            {99,99, 0,99, 0,99, 0,99, 0, 0,99},    //  1,68
                            {99,99,41, 3, 0, 0, 0, 0, 0, 8,99},    //  2,24
                            {99,99, 0,99, 6, 0, 7,99, 0,99,99},    //  2,80
                            {99,99, 0, 0, 0, 0, 0, 0, 0, 0,99},    //  3,36
                            {99,99, 0,99, 0,99,10, 0,11,99,99},    //  3,92
                            {99,99, 0, 9, 0, 0, 0, 0, 0, 0,99},    //  4,48
                            {99,99, 0, 0, 0,51, 0, 0,12,99,99},    //  5,04
                            {99,99,99,99,99,99,99,99,99,99,99}     //  5,60
                        };
static int closed_nodes_map[w][m]; // map of closed (tried-out) nodes
static int open_nodes_map[w][m]; // map of open (not-yet-tried) nodes
static int dir_map[w][m]; // map of directions
const int dir=4; // number of possible directions to go at any position
// if dir==4
static int dx[dir]={1, 0, -1, 0};
static int dy[dir]={0, 1, 0, -1};
// if dir==8
//static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
//static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority
    int prev_i;

    public:
        node(int xp, int yp, int d, int p, int pi)
            {xPos=xp; yPos=yp; level=d; priority=p; prev_i=pi;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}
        int getPrev_i() const {return prev_i;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i, int i_ant) // i: direction
        {
            if(i_ant == -1 || i == i_ant)
                level+=10;
            else
                level+=14;
        }

        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            //d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, int goal)
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    int xFinish = -1;
    int yFinish = -1;
    pqi=0;

    if(goal != 0 && goal != 99)
    {
        for(y=0;y<m;y++)
        {
            for(x=0;x<w;x++)
            {
                if(myMap[x][y]==goal)
                {
                    xFinish = x;
                    yFinish = y;
                }
            }
        }
    }

    if(xFinish == -1)
    {
        printf("Objetivo Inválido!\n");
        return "";
    }

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<w;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0, -1);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority(), pq[pqi].top().getPrev_i());

        i_ant = n0->getPrev_i();

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>w-1 || ydy<0 || ydy>m-1 || myMap[xdx][ydy]==99
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority(), i);
                m0->nextLevel(i, i_ant);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

void transformRealPos_BlockPos(float posX, float posY, int &xA, int &yA)
{
	xA = ( (posX + 5.6) / 0.56 ) + 0.5;
	yA = ( posY / 0.56 ) + 0.5;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
	posX = msg.pose.pose.position.x;
	posY = msg.pose.pose.position.y;
	posTheta = msg.pose.pose.orientation.z;
	transformRealPos_BlockPos(posX, posY, xA, yA);
}

bool genCallback(robotino_local_move::FullPath::Request &req, robotino_local_move::FullPath::Response &res)
{
	srand(time(NULL));
	// get the route
	clock_t start = clock();
	res.full_path = pathFind(xA, yA, req.goal);    // <------------------------------------- CALCULA O CAMINHO!

	printf("Start: [%i][%i]\n", xA, yA);
	printf("Goal: %i\n", req.goal);

	if(res.full_path == "")
	{
		printf("An empty route generated!\n");
		return false;
	}
	clock_t end = clock();
	double time_elapsed = double(end - start);
	printf("Time to calculate the route (ms): %f", time_elapsed);
	printf("Route: %s\n", res.full_path.c_str());

	return true;
}

int main(int argc, char **argv)
{
	posX = 0;
	posY = 0;
	posTheta = 0;

	ros::init(argc, argv, "robotino_manager");
	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);
	ros::ServiceServer service = n.advertiseService("FullPath", genCallback);
	ros::Rate loop_rate(1); // Em Hz

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return(0);
}
