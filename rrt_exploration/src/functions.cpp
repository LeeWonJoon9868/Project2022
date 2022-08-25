#include "functions.h"
#include "geometry_msgs/PointStamped.h"

// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);}
float rdm::randomize() { i=i+1;  srand (i);  return float(rand())/float(RAND_MAX);}


//Norm function 
float Norm(vec<float> x1,vec<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}


//sign function
float sign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}
//Nearest function
vec<float> Nearest(  vec< vec<float>  > V, vec<float>  x){

float min=Norm(V[0],x);
int min_index;
float temp;

for (int j=0;j<V.size();j++)
{
temp=Norm(V[j],x);
if (temp<=min){
min=temp;
min_index=j;}

}

return V[min_index];
}

//
vec<float> Near_avg(  vec< vec<float>  > V, vec<float> x, nav_msgs::OccupancyGrid mapsub){

char checking;
vec< vec<float> > fake_V;



	for (int j=0;j<V.size();j++){
	
		if(checking = ObstacleFree(V[j],x,mapsub)){
			fake_V.push_back(V[j]);
		}
	}
	if(!fake_V.empty()){
	return Nearest(fake_V, x);
	}

	else{
		fake_V.clear();
		for (int j=0;j<V.size();j++){
			if(V[j].child==0){
				fake_V.push_back(V[j]);
			}
		}
		
//sort
		std::sort(fake_V.begin(),fake_V.end());
		int half_num =fake_V.size()/2;
		return fake_V[half_num];


	}
}
	





//Steer function
vec<float> Steer(  vec<float> x_nearest , vec<float> x_rand, float eta){

vec<float> x_new;

if (Norm(x_nearest,x_rand)<=eta){
x_new=x_rand;
}
else{


float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

x_new.push_back(  (sign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );

if(x_rand[0]==x_nearest[0]){
x_new[0]=x_nearest[0];
x_new[1]=x_nearest[1]+eta;
}



}
return x_new;
}





//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData,vec<float> Xp){

float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;

float width=mapData.info.width;
std::vector<signed char> Data=mapData.data;

//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free
float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
int out;
out=Data[int(indx)];
return out;
}




// ObstacleFree function-------------------------------------
/*
char ObstacleFree(vec<float> xnear, vec<float> &xnew, nav_msgs::OccupancyGrid mapsub){
float rez=float(mapsub.info.resolution)*.2;
float stepz=int(ceil(Norm(xnew,xnear))/rez); 
vec<float> xi=xnear;
char  obs=0; char unk=0;
 
geometry_msgs::Point p;
for (int c=0;c<stepz;c++){
  xi=Steer(xi,xnew,rez);
  		

   if ((gridValue(mapsub,xi) ==100) || (infoGain(xi,mapsub)>5)){     
	obs=1; }
   
   if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}
  }
char out=0;
 xnew=xi;
 if (unk==1){  out=-1;}
 	
 if (obs==1){  out=0;}
 		
 if (obs!=1 && unk!=1){   out=1;}

 
 
 
 return out;
 

 }
*/

int ObstacleFree(vec<float> xnear, vec<float> xnew, nav_msgs::OccupancyGrid mapsub){
float rez=float(mapsub.info.resolution)*.2;
float stepz=int(ceil(Norm(xnew,xnear))/rez); 
vec<float> xi=xnear;
char  obs=0; char unk=0;
 
geometry_msgs::Point p;
for (int c=0;c<stepz;c++){
  xi=Steer(xi,xnew,rez);
  		

   if (gridValue(mapsub,xi) ==100){     obs=1; }
   
   if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}
  }
int out=0;
 xnew=xi;
 if (unk==1){  out=-1;}
 	
 if (obs==1){  out=0;}
 		
 if (obs!=1 && unk!=1){   out=1;} 
 
 
 return out;
 

 }
char goal_find(vec<float> xnew,nav_msgs::OccupancyGrid mapsub){
char out=0;
if(gridValue(mapsub,xnew) == 0){out=1;}
else{ out= 0;}
return out;

}
/*
int infoGain(vec< vec<float>  >V,vec<float> next_node,nav_msgs::OccupancyGrid mapData){


	int i =0,n=0;
	int r = 0.1;
	int infoGain= 0;	
	int start= 0, end= 0 ,limit =0;
	int resolution=mapData.info.resolution;
	int Xstartx=mapData.info.origin.position.x;
	int Xstarty=mapData.info.origin.position.y;
	int width=mapData.info.width;
	int index= int(( floor((next_node[1]-Xstarty)/resolution)*width)+( floor((next_node[0]-Xstartx)/resolution) ));
	int r_region=int(r/resolution);
	int init_index=index-r_region*(mapData.info.width+1);
	
	
	for (n =0 ; n < 2*r_region+1 ;n++){
		start=n*width+init_index;
		end=start+2*r_region;
		limit=((start/width)+2)*width;
		for (i =start;i<=end;i++){
		
			if ((i>=0) && (i<limit) && (i<mapData.data.size())){
				if(mapData.data[i]==100){
					infoGain+=1;

				}
			}
		}	
	}
  return infoGain; 

}
*/


 float min(float a,float b){
	if (a>b)
		return b;
	else
		return a;
}
 // find near node function--------------------------------
 /*    
 def find_near_nodes(self, new_node):
    nnode = len(self.node_list) + 1
    r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
    # if expand_dist exists, search vertices in a range no more than expand_dist
    if hasattr(self, 'expand_dis'): 
        r = min(r, self.expand_dis)
    dist_list = [(node.x - new_node.x) ** 2 +
                 (node.y - new_node.y) ** 2 for node in self.node_list]
    near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
    return near_inds
 */
vec<float> find_near_node(vec< vec<float>  > V,vec<float> x_new, float eta)
{
	float connet_circle_dist = 10.0;
	int nv = V.size()+1;
	float r = connet_circle_dist*sqrt((log(nv)/nv));
    r = min(r,eta);
    vec<float> dist_list;
    for(int i=0;i<V.size();i++){
    	//dist_list.push_back(pow((V[i]-x_new),2)+pow((V[i]-x_new),2));
    	dist_list.push_back(pow(Norm(V[i],x_new),2));
    }
    vec<float> near_inds;
    near_inds.clear();
    for(int i=0;i<dist_list.size();i++){
    	if(dist_list[i]<=pow(r,2))
    		near_inds.push_back(i);
    }

    return near_inds;
}


     
   
// chooseparent function ------------------------------------
/*
    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node
*/
/*
 def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  # safe
*/
/*points.points.clear
  def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d                                     <<d is Norm , cost?
  def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
*/
vec<float> chooseparent(vec< vec<float>  > V,vec<float> x_new , vec<float> near_inds,nav_msgs::OccupancyGrid mapsub)
{
	vec<float> near_node;
	vec<float> t_node;
	vec<float> a;
	a.clear();
	float eta = 1.0;
	if (near_inds.empty())
		return a;
	float* costs= new float[near_inds.size()];
	for(int i=0;i<near_inds.size();i++){
		near_node = V[near_inds[i]];
		//t_node =Steer(near_node,x_new,eta); // il-dan self->V
                char abc =ObstacleFree(near_node,x_new,mapsub);
		if (abc==1)
			{
				costs[i]=Norm(near_node,x_new);
			}
		else
			{
				costs[i]=1000000000.0;
			}
	}
	float* min_cost = std::min_element(costs,costs+near_inds.size());
	float find_ar = (std::find(costs,costs+near_inds.size(),*min_cost) - costs);

	 float min_ind = near_inds[find_ar];
        //x_new = Steer(V[min_ind], x_new,eta);
	//---------------------------------------------------------
       // x_new.parents = V[min_ind];
	x_new.parents= min_ind;
        x_new.cost = *min_cost;
	V[x_new.parents].child = V[x_new.parents].child +1; 
        if(x_new.cost >= 1000000000)
            return a;

    	//----------------------------------------------------------
    
    return x_new;
}


//rewire function --------------------------------------------
/*    
	def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)
                ---------------------------------------------
    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
*/
vec< vec<float>  > propagate_cost_to_leaves(vec< vec<float>  > V,vec<float> x_new )
{
	for(int i=0;i<V.size();i++){
		if(V[V[i].parents]==x_new){
			//V[i].cost = V[i].parents.cost + Norm(V[i],x_new);
			V[i].cost = x_new.cost + Norm(V[i],x_new);
			//V = propagate_cost_to_leaves(V,x_new);
	}
}
return V;
}
vec< vec<float>  > rewire(vec< vec<float>  > V,vec<float> x_new , vec<float> near_inds,nav_msgs::OccupancyGrid mapsub)
 {
 	float eta = 1.0;
	vec<float> near_node;
	vec<float> edge_node;
 	for(int i=0;i<near_inds.size();i++){
		near_node = V[near_inds[i]];
		//edge_node =Steer(x_new,near_node,eta); 
		//if (edge_node.empty())
		//	continue;
		float edge_node_cost = near_node.cost + Norm(near_node,x_new);

		
		char check_obs = ObstacleFree(near_node,x_new,mapsub); // why - 48
		
		bool improved_cost = x_new.cost + Norm(x_new,near_node) > edge_node_cost;

		if (check_obs==1&&improved_cost)
		{
	
			V[near_inds[i]].parents=V.size()-1;
			V = propagate_cost_to_leaves(V,x_new);
		}
	}
return V;
 }
vec<float> find_init_next(vec< vec<float>  >V,geometry_msgs::PointStamped exploration_goal, vec<float> init_node)
{
	vec<float> next_node;
  	vec<float> temp_node;
	int i =0;
	//next_node.clear();
	temp_node.clear();
	temp_node.push_back(exploration_goal.point.x);
	temp_node.push_back(exploration_goal.point.y);
	vec<float> target_goal= Nearest(V,temp_node);

	
	for(next_node= target_goal; init_node != V[next_node.parents]; next_node = V[next_node.parents]);
	


	return next_node;	
}

 
 

int infoGain(vec<float> next_node,nav_msgs::OccupancyGrid mapData){


	int i =0,n=0;
	int r = 0.2;
	int infoGain= 0;	
	int start= 0, end= 0 ,limit =0;
	int resolution=mapData.info.resolution;
	int Xstartx=mapData.info.origin.position.x;
	int Xstarty=mapData.info.origin.position.y;
	int width=mapData.info.width;
	int index= int(( floor((next_node[1]-Xstarty)/resolution)*width)+( floor((next_node[0]-Xstartx)/resolution) ));
	int r_region=int(r/resolution);
	int init_index=index-r_region*(mapData.info.width+1);
	
	
	for (n =0 ; n < 2*r_region+1 ;n++){
		start=n*width+init_index;
		end=start+2*r_region;
		limit=((start/width)+2)*width;
		for (i =start;i<=end;i++){
		
			if ((i>=0) && (i<limit) && (i<mapData.data.size())){
				if(mapData.data[i]==100){
					infoGain+=1;

				}
			}
		}	
	}
  return infoGain; 

}
 

/*
def informationGain2(mapData,point,r):
	infoGain=0;
	index=index_of_point(mapData,point)
	r_region=int(r/mapData.info.resolution)
	init_index=index-r_region*(mapData.info.width+1)	
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width
		for i in range(start,end+1):
			if (i>=0 and i<limit and i<len(mapData.data)):
				if(mapData.data[i]==100 and norm(array(point)-point_of_index(mapData,i))<=r):
					infoGain+=1
	return infoGain
*/

























