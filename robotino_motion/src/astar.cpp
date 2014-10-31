// Astar.cpp
// http://en.wikipedia.org/wiki/A*
// Compiler: Dev-C++ 4.9.9.2
// FB - 201012256
#include "astar.h"

node::node()
{
	xPos = 0;
    yPos = 0;
    // total distance already travelled to reach the node
    level = 0;
    // priority=level+remaining distance estimate
    priority = 0;  // smaller: higher priority

	v_base[0] = v_base[1] = 0;
	v_aux[0] = v_aux[1] = 0;
	comando[0] = comando[1] = 0;
	res = 0;
	resx = 0;
	resy = 0;
	//map88[n8][m8] = 0;
}


const int n=17; // horizontal size of the map
const int m=17; // vertical size size of the map
static int map[n][m];
const int n8=8;
const int m8=8;
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static int dir_map[n][m]; // map of directions
//const int dir=8; // number of possible directions to go at any position
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};


// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, 
                 const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

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

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), 
                             n0->getPriority());
                m0->nextLevel(i);
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

void node::findpath(std::vector<int> vet, int xA, int yA, int xB, int yB)
{
	for (int i = 0; i < 17; i++)
	{
		for (int j = 0; j < 17; j++)
		{
			if (vet[i * 17 + j] == 0)
			{
				cout << "   ";
			}
			else
			{
				cout << " * ";
			}
		}
		cout << endl;
	}

	int movimentox = 0;
	int movimentoy = 0;	
	int movimentoxy1 = 0;
	int movimentoxy2 = 0;
	
	int t1 = 0;
	int t2 = 0;
	int t3 = 0;
	
    srand(time(NULL));
//----------------------------------------------------------------------------------------//  

	//Aqui as �reas livres s�o setados em '0'
    // create empty map 17x17
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++)
        	{
        		map[x][y]=0;
        	}
    }

    // create empty map 8x8
    for(int j=0;j<m8;j++)
    {
        for(int i=0;i<n8;i++)
        	{
        		map88[i][j]=0;
        	}
    }

//----------------------------------------------------------------------------------------//  

    // fillout the map matrix with a '+' pattern
    

    //----------------------------------------------------------------------------------------//

    	//Lendo um vetor de 17x17. Sendo o vet[i*dim + j] o vetor dado pelo mapping

	cout << " criando matrix";
    	for(int i=0;i<17;i++)
    	{
    		for(int j=0;j<17;j++)
    		{
    			map[i][j] = vet[i*17 + j];
    		}
    	}


//----------------------------------------------------------------------------------------//    
//Aqui os obst�culos s�o setados em '1'
/*    for(int x=n/8;x<n*7/8;x++)
    {
        map[x][m/2]=1;
    }
    for(int y=m/8;y<m*7/8;y++)
    {
        map[n/2][y]=1;
    }
*/

    //Aqui setamos a coordenada INICIAL e FINAL
    // randomly select start and finish locations
 /*   int xA, yA, xB, yB;
    xA = 7;
	yA = 7;
	xB = 10;
	yB = 10;
*/
//----------------------------------------------------------------------------------------//      

/*	switch(rand()%8)
    {
        case 0: xA=0;yA=0;xB=n-1;yB=m-1; break;
        case 1: xA=0;yA=m-1;xB=n-1;yB=0; break;
        case 2: xA=n/2-1;yA=m/2-1;xB=n/2+1;yB=m/2+1; break;
        case 3: xA=n/2-1;yA=m/2+1;xB=n/2+1;yB=m/2-1; break;
        case 4: xA=n/2-1;yA=0;xB=n/2+1;yB=m-1; break;
        case 5: xA=n/2+1;yA=m-1;xB=n/2-1;yB=0; break;
        case 6: xA=0;yA=m/2-1;xB=n-1;yB=m/2+1; break;
        case 7: xA=n-1;yA=m/2+1;xB=0;yB=m/2-1; break;
    }
*/

//    cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
//    cout<<"Start: "<<xA<<","<<yA<<endl;
//    cout<<"Finish: "<<xB<<","<<yB<<endl;
    // get the route
    clock_t start = clock();
	cout << "calculando uma trajetoria";
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
//    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
//    cout<<"Route:"<<endl;
//    cout<<route<<endl<<endl;

    // follow the route on the map and display it 
    if(route.length()>0)
    {
    	//Seta a coordenada inicial em 2
        int j; char c;
        int x=xA;
        int y=yA;
        map[x][y]=2;
		
		//desenha no mapa a trajet�ria
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c); 
            x=x+dx[j];
            y=y+dy[j];
            map[x][y]=3;
            
		if(j % 2 != 0)
		{
            		//insere x e y impares na fila temp
            		queue_temp.push(dx[j]);
            		queue_temp.push(dy[j]);
		}
        }
        //no fim da trajet�ria insere '10' � fila
        queue_temp.push(10);
        queue_temp.push(10);

//        printf("\nSize do queue_temp: %d\n", queue_temp.size());
        
		//a ultima c�lula da trajet�ria recebe '4' = F
		map[x][y]=4;
		//Assim finaliza-se a matriz 17x17
	//----------------------------------------------------------------------------------------//  
		//Cenvertendo uma matriz 17x17 em uma matriz 8x8
		
		for(int f=0;f<m;f++)
		{
			if(f % 2 != 0 )
			{
				for(int g=0;g<n;g++)
				{
					if(g % 2 != 0)
					{
						map88[(g-1)/2][(f-1)/2] = map[g][f];
					}		
				}
			}
		}
		


	//----------------------------------------------------------------------------------------//  

    
        // display the map with the route
        //Aqui desenha o mapa. N�o precisa modificar.
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++)
                if(map[x][y]==0)
                    cout<<".";
                else if(map[x][y]==1)
                    cout<<"O"; //obstacle
                else if(map[x][y]==2)
                    cout<<"S"; //start
                else if(map[x][y]==3)
                    cout<<"R"; //route
                else if(map[x][y]==4)
                	cout<<"F"; //finish
            cout<<endl;
        }
        printf("\n\n");
        for(int y=0;y<m8;y++)
        {
            for(int x=0;x<n8;x++)
                if(map88[x][y]==0)
                    cout<<".";
                else if(map88[x][y]==1)
                    cout<<"O"; //obstacle
                else if(map88[x][y]==2)
                    cout<<"S"; //start
                else if(map88[x][y]==3)
                    cout<<"R"; //route
                else if(map88[x][y]==4)
                	cout<<"F"; //finish
            cout<<endl;
        }
        
     //   printf("(%d,%d)", queue_astar.front(),queue_temp.front());
      //  printf("\n(%d,%d)",queue_astar.size(),queue_temp.size());
        
	}
        //Unindo movimentos na mesma coordenada

		v_base[0] = queue_temp.front(); //x
	    	queue_temp.pop();
	    	v_base[1] = queue_temp.front(); //y
	    	queue_temp.pop();

    /*      	v_aux[0] = queue_temp.front();
        	queue_temp.pop();
        	v_aux[1] = queue_temp.front();
        	queue_temp.pop();
		*/	
//			printf("\nAntes do while");
        	while(!queue_temp.empty())
        	{
        		if(v_base[0] != 10)
        		{
//	        		printf("\nentrou no while geral");
//	        		printf("\nSize do queue_temp: %d", queue_temp.size());
	        	
//	        		printf("\nSize do queue_temp: %d", queue_temp.size());
					//while((v_base[0] - v_aux[0]) != 0 && (v_base[1] - v_aux[1]) == 0) 
	        		while((v_base[1] == 0 && v_base[0] != 10)) //movimento x
	        		{
//	        			printf("\nentrou no while do movimento x");
	        			movimentox = movimentox + v_base[0];
	        			comando[0] = movimentox;
	        			comando[1] = 0; 
	        		//	if(!queue_temp.empty())	
						{	
						v_base[0] = queue_temp.front(); //x
			        	queue_temp.pop();
			        	v_base[1] = queue_temp.front(); //y
			        	queue_temp.pop();
			    		}
						t1 = 1;
//			        	printf("\nSize do queue_temp: %d", queue_temp.size());
			     }
			    	if(t1 == 1)
			    	{
//			    		printf("\nfundiu 7 movimentos");
						queue_astar.push(comando[0]);
	        			queue_astar.push(comando[1]);
	        			movimentox = 0;
	        			t1 = 0;
//	        			printf("\ncolocou (7,0) na fila astar ");
				 }
	        		
	        		//while((v_base[1] - v_aux[1]) != 0 && (v_base[0] - v_aux[0]) == 0) //movimento y
	        		
	        		while((v_base[0] == 0 && v_base[0] != 10)) //movimento y
					{
//						printf("\nentrou no while do movimento y");
						
	        			movimentoy = movimentoy + v_base[1];
	        			comando[0] = 0;
	        			comando[1] = movimentoy; 
	        		//if(!queue_temp.empty())	
					{
						v_base[0] = queue_temp.front(); //x
			        	queue_temp.pop();
			        	v_base[1] = queue_temp.front(); //y
			        	queue_temp.pop();
			    	}
						t2 = 1;
//			        	printf("\nSize do queue_temp: %d", queue_temp.size());
			    	}
			    	if(t2 == 1)
			    	{	//printf("\nfundiu movimento y");
			    		queue_astar.push(comando[0]);
	        			queue_astar.push(comando[1]);
	        			movimentoy = 0;
	        			t2 = 0;
			    	}
//			    	printf("\njogou mov y na fila");
					
	        		//while((v_base[0] - v_aux[0]) != 0 && (v_base[1] - v_aux[1]) != 0) //movimento diagonal
	        		while((v_base[0] < 0) && (v_base[1] < 0) && (v_base[0] > -100) && v_base[0] != 10) //movimento diagonal
					{
						//if(v_base[0] != 10)
						//{
						
//						printf("\nentrou no while do movimento xy");
						movimentoxy1 = movimentoxy1 + v_base[0];
						movimentoxy2 = movimentoxy2 + v_base[1];
	        			comando[0] = movimentoxy1;
	        			comando[1] = movimentoxy2; 
	        		//if(!queue_temp.empty())	
	        		{
						v_base[0] = queue_temp.front(); //x
			        	queue_temp.pop();
			        	v_base[1] = queue_temp.front(); //y
			        	queue_temp.pop();
			    	}
						t3 = 1;
//			        	printf("\nSize do queue_temp: %d", queue_temp.size());
			    	}
			    	if(t3 == 1)
			    	{
//			    		printf("\nfundiu movimento xy");
			    		queue_astar.push(comando[0]);
	        			queue_astar.push(comando[1]);
	        			movimentoxy1 = 0;
	        			movimentoxy2 = 0;
	        			t3 = 0;
	        			
//	        			printf("\njogou mov xy");
			    	}
			    	
			    	while((v_base[0] < 0) && (v_base[1] > 0) && (v_base[0] > -100) && v_base[0] != 10) //movimento diagonal
					{
						//if(v_base[0] != 10)
						//{
						
//						printf("\nentrou no while do movimento xy");
						movimentoxy1 = movimentoxy1 + v_base[0];
						movimentoxy2 = movimentoxy2 + v_base[1];
	        			comando[0] = movimentoxy1;
	        			comando[1] = movimentoxy2; 
	        		//if(!queue_temp.empty())	
	        		{
						v_base[0] = queue_temp.front(); //x
			        	queue_temp.pop();
			        	v_base[1] = queue_temp.front(); //y
			        	queue_temp.pop();
			    	}
						t3 = 1;
//			        	printf("\nSize do queue_temp: %d", queue_temp.size());
			    	}
			    	if(t3 == 1)
			    	{
//			    		printf("\nfundiu movimento xy");
			    		queue_astar.push(comando[0]);
	        			queue_astar.push(comando[1]);
	        			movimentoxy1 = 0;
						movimentoxy2 = 0;
	        			t3 = 0;
	        			
//	        			printf("\njogou mov xy");
			    	}
			    	
			    	while((v_base[0] > 0) && (v_base[1] > 0) && (v_base[0] > -100) && v_base[0] != 10) //movimento diagonal
					{
						//if(v_base[0] != 10)
						//{
						
//						printf("\nentrou no while do movimento xy");
						movimentoxy1 = movimentoxy1 + v_base[0];
						movimentoxy2 = movimentoxy2 + v_base[1];
	        			comando[0] = movimentoxy1;
	        			comando[1] = movimentoxy2; 
	        		//if(!queue_temp.empty())	
	        		{
						v_base[0] = queue_temp.front(); //x
			        	queue_temp.pop();
			        	v_base[1] = queue_temp.front(); //y
			        	queue_temp.pop();
			    	}
						t3 = 1;
//			        	printf("\nSize do queue_temp: %d", queue_temp.size());
			    	}
			    	if(t3 == 1)
			    	{
//			    		printf("\nfundiu movimento xy");
			    		queue_astar.push(comando[0]);
	        			queue_astar.push(comando[1]);
	        			movimentoxy1 = 0;
	        			movimentoxy2 = 0;
	        			t3 = 0;
	        			
//	        			printf("\njogou mov xy");
			    	}
			    	
			    	while((v_base[0] > 0) && (v_base[1] < 0) && (v_base[0] > -100) && v_base[0] != 10) //movimento diagonal
					{
						//if(v_base[0] != 10)
						//{
						
//						printf("\nentrou no while do movimento xy");
						movimentoxy1 = movimentoxy1 + v_base[0];
						movimentoxy2 = movimentoxy2 + v_base[1];
	        			comando[0] = movimentoxy1;
	        			comando[1] = movimentoxy2; 
	        		//if(!queue_temp.empty())	
	        		{
						v_base[0] = queue_temp.front(); //x
			        	queue_temp.pop();
			        	v_base[1] = queue_temp.front(); //y
			        	queue_temp.pop();
			    	}
						t3 = 1;
//			        	printf("\nSize do queue_temp: %d", queue_temp.size());
			    	}
			    	if(t3 == 1)
			    	{
//			    		printf("\nfundiu movimento xy");
			    		queue_astar.push(comando[0]);
	        			queue_astar.push(comando[1]);
	        			movimentoxy1 = 0;
	        			movimentoxy2 = 0;
	        			t3 = 0;
	        			
//	        			printf("\njogou mov xy");
			    	}
			    
//	        	printf("\nSize do queue_temp: %d", queue_temp.size());
	        	}//else{break;}
	        	
			}
	        
        	
//			printf("\nSAIU do while");
//			printf("\nSize do queue_temp: %d", queue_temp.size());
        	res = 0;
			resx = 0;
        	resy = 0;
        	
			//printf("(%d,%d)", queue_astar.front(),queue_temp.front());
 //       	printf("\n Size: %d",queue_astar.size());
 /*       	while(!queue_astar.empty())
        	{
        		printf("\n(%d,",queue_astar.front());
        		queue_astar.pop();
        		printf("%d)",queue_astar.front());
				queue_astar.pop();        		
        	}*/
 //			printf("\nSize do queue_astar: %d", queue_astar.size());
    
    //getchar(); // wait for a (Enter) keypress  
    //return(0);
}
/*
int main()
{
	node node;
	
	node.findpath();
	
	
	return 0;
}

*/

