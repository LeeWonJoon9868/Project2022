#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>




// global variables
nav_msgs::OccupancyGrid mapData;

geometry_msgs::PointStamped clickedpoint, deletedpoint;
geometry_msgs::PointStamped exploration_goal,init_goal;
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;

rdm r; // for genrating random numbers



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData=*msg;
}

 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 

geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;

points.points.push_back(p);

}
	
void deleteCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 


deletedpoint.point.x=msg->point.x;
deletedpoint.point.y=msg->point.y;
deletedpoint.point.z=msg->point.z;


}
	




int main(int argc, char **argv)
{

  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init, length); // 32-bit int generator
// this is an example of initializing by an array
// you may use MTRand(seed) with any 32bit integer
// as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

// generate the same numbers as in the original C test program
  ros::init(argc, argv, "local_rrt_frontier_detector");
  ros::NodeHandle nh;
  
  // fetching all parameters
  float eta,init_map_x,init_map_y,range;
  std::string map_topic,base_frame_topic;
  
  std::string ns;
 ns=ros::this_node::getName();

  ros::param::param<float>(ns+"/eta", eta, 0.15);

  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/map"); 
  ros::param::param<std::string>(ns+"/robot_frame", base_frame_topic, "/base_link"); 
//---------------------------------------------------------------
ros::Subscriber sub= nh.subscribe(map_topic, 100 ,mapCallBack);
	
ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);	
/*
geometry_msgs::Point pp;  
pp.x=23.4;
pp.y=7.29;
pp.z= 0.0;
points.points.push_back(pp);
  pp.x= -8.45;
  pp.y= -7.27;
   pp.z= 0.0;
points.points.push_back(pp);
  pp.x= - 8.45;
  pp.y= -22.2;
  pp.z= 0;
points.points.push_back(pp);
  pp.x= 22.2;
  pp.y= -22.3;
  pp.z= 0.0;
points.points.push_back(pp);
pp.x= -4.45;
  pp.y= 1.15;
  pp.z= 0;
points.points.push_back(pp);
*/
ros::Subscriber deleted= nh.subscribe("/deleted", 10 ,deleteCallBack);



ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);
ros::Publisher init_pub = nh.advertise<geometry_msgs::PointStamped>("/init_points", 1);
ros::Rate rate(100); 
 
while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}



//visualizations  points and lines..
points.header.frame_id=mapData.header.frame_id;
line.header.frame_id=mapData.header.frame_id;
points.header.stamp=ros::Time(0);
line.header.stamp=ros::Time(0);
	
points.ns=line.ns = "markers";
points.id = 0;
line.id =1;


points.type = points.POINTS;
line.type=line.LINE_LIST;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points.action =points.ADD;
line.action = line.ADD;
points.pose.orientation.w =1.0;
line.pose.orientation.w = 1.0;
line.scale.x =  0.03;
line.scale.y= 0.03;
points.scale.x=0.3; 
points.scale.y=0.3; 

line.color.r =255.0/255.0;
line.color.g= 0.0/255.0;

line.color.b =0.0/255.0;
points.color.r = 255.0/255.0;
points.color.g = 0.0/255.0;
points.color.b = 0.0/255.0;
points.color.a=0.3;
line.color.a = 1.0;
points.lifetime = ros::Duration();
line.lifetime = ros::Duration();

geometry_msgs::Point p;  


while(points.points.size()<5)
{
ros::spinOnce();

pub.publish(points) ;
}






vec<float> temp1;
temp1.push_back(points.points[0].x);
temp1.push_back(points.points[0].y);
	
vec<float> temp2; 
temp2.push_back(points.points[2].x);
temp2.push_back(points.points[0].y);


	init_map_x=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

temp1.push_back(points.points[0].x);
temp1.push_back(points.points[0].y);

temp2.push_back(points.points[0].x);
temp2.push_back(points.points[2].y);

init_map_y=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

Xstartx=(points.points[0].x+points.points[2].x)*.5;
Xstarty=(points.points[0].y+points.points[2].y)*.5;





geometry_msgs::Point trans;
trans=points.points[4];
vec< vec<float>  > V; 
vec<float> xnew; 
xnew.push_back( trans.x);xnew.push_back( trans.y);  
V.push_back(xnew);

points.points.clear();
pub.publish(points) ;






vec<float> frontiers;
//int i=0;
int flag = 1;
int new_start=1;
int go_start =0;
int node_num = 4;
bool going = false;
bool rand1_unknown = false;
bool rand2_unknown = false;
int goal_flag =0;
float xr,yr,xr2,yr2;
int checking;
vec<float> x_rand,x_nearest,x_new,near_inds,next_node,init_node,temp_node,target_goal,temp_goal,real_goal,x_rand2;
real_goal.clear();
real_goal.push_back(-10.0); real_goal.push_back(0.1);
tf::TransformListener listener;
// Main loop
while (ros::ok()){

	if((goal_flag= goal_find(real_goal,mapData))){
			if(flag){
			init_goal.header.stamp=ros::Time(0);
		  	init_goal.header.frame_id=mapData.header.frame_id;
			init_goal.point.x=exploration_goal.point.x;// insert upper code
	  		init_goal.point.y=exploration_goal.point.y;
	  		init_goal.point.z=0.0;
			init_pub.publish(init_goal); // think
			flag =0;
			}
			points.points.clear();

			pub.publish(points);
			exploration_goal.header.stamp=ros::Time(0);
  				exploration_goal.header.frame_id=mapData.header.frame_id;
				exploration_goal.point.x=real_goal[0];// insert upper code
  				exploration_goal.point.y=real_goal[1];
  				exploration_goal.point.z=0.0;
  				targetspub.publish(exploration_goal);

	}
	else{
	if(!going){
		// Sample free
		x_rand.clear();
		x_rand2.clear();
		//xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
		//yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;
		if(drand()>0.09){
			xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
			yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;


			}
		else{


			xr= real_goal[0];
			yr= real_goal[1];
			}
			xr2=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
			yr2=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;

		x_rand.push_back( xr ); x_rand.push_back( yr );
                x_rand2.push_back( xr2 ); x_rand2.push_back( yr2 );
		
	
	   
		// Nearest
		x_nearest=Nearest(V,x_rand);

	
		// Steer
		if(!go_start){

		x_new=Steer(x_nearest,x_rand,5);
		}
		else{
			rand1_unknown =!goal_find(x_rand,mapData);
			rand2_unknown =!goal_find(x_rand2,mapData);
			if( (rand1_unknown && rand2_unknown) || !(rand1_unknown || rand2_unknown)){
				if(Norm(x_rand2,real_goal) > Norm(x_rand,real_goal)){
				x_new=Steer(x_nearest,x_rand,4); //9
				}
		
				else{
				x_new=Steer(x_nearest,x_rand2,4); //9
				}	
			}

			else{
				if(rand1_unknown){
				x_new=Steer(x_nearest,x_rand,4);
				}
				else{
				x_new=Steer(x_nearest,x_rand2,4);
				}

			}

		}


	/*if(!going){
		// Sample free
		x_rand.clear();
		//xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
		//yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;
		if(drand()>0.09){
			xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
			yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;

			}
		else{


			xr= real_goal[0];
			yr= real_goal[1];
			}

		x_rand.push_back( xr ); x_rand.push_back( yr );


		// Nearest
		x_nearest=Nearest(V,x_rand);

	
		// Steer
		if(!go_start){

		x_new=Steer(x_nearest,x_rand,eta);
		}
		else{
		x_new=Steer(x_nearest,x_rand,9);
		}*/

		// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
		checking=ObstacleFree(x_nearest,x_new,mapData);

		}
	else{
		checking = 0;
		}


		if ((checking==-1) ||go_start ){



				
			  	

				if(new_start==1){
					x_new[0] = -0.3;
					x_new[1] = 1.4;
					exploration_goal.header.stamp=ros::Time(0);
			  		exploration_goal.header.frame_id=mapData.header.frame_id;
					exploration_goal.point.x=x_new[0];// insert upper code
			  		exploration_goal.point.y=x_new[1];
			  		exploration_goal.point.z=0.0;
			
				  	targetspub.publish(exploration_goal);
					
				        init_node =x_new;

					init_node.parents =0;
					init_node.child =0;

					new_start = 0;
					line.points.clear();
					points.points.clear();
				  	V.clear();
				
					V.push_back(init_node);

					}
				else if(go_start==1){
					exploration_goal.header.stamp=ros::Time(0);
			  		exploration_goal.header.frame_id=mapData.header.frame_id;
					exploration_goal.point.x=real_goal[0];// insert upper code
			  		exploration_goal.point.y=real_goal[1];
			  		exploration_goal.point.z=0.0;
					go_start =0;
					temp_node.clear();

					temp_node.push_back(exploration_goal.point.x);
					temp_node.push_back(exploration_goal.point.y);
					target_goal= Near_avg(V,temp_node,mapData);

	
					points.color.r = 0.0/255.0;
					points.color.g = 255.0/255.0;
					points.color.b = 0.0/255.0;
					points.color.a=0.3;
	
					p.x=target_goal[0]; 
					p.y=target_goal[1]; 
					p.z=0.0;
			  		points.points.push_back(p);
			  		pub.publish(points) ; 
					node_num =6;
	
	//goal setting
					}
				}
		

//going end
	  else if ((checking==1)&&(new_start==0)&&(go_start==0) || going){ //new start = 0 


 		if(V.size()<=node_num){
  	       		near_inds=find_near_node(V,x_new,eta);
	        	x_new = chooseparent(V,x_new,near_inds,mapData);   //value
	        	if(x_new.empty()){
				continue;
			}
			x_new.child = 0;
	 		V.push_back(x_new);
 	

          		//p.x=x_new[0]; 
			//p.y=x_new[1]; 
			//p.z=0.0;
			//pub.publish(points);
		     	//V=rewire(V,x_new,near_inds,mapData);
/*
		 	p.x=x_new[0]; 
			p.y=x_new[1]; 
			p.z=0.0;
		 	line.points.push_back(p);
		        x_new = V[x_new.parents];
		 	p.x=x_new[0]; 
			p.y=x_new[1]; line.points.clear();
				points.points.clear();
				init_node.parents = 0;
				V.push_back(init_node);
				pub.publish(points);
			p.z=0.0;
		 	line.points.push_back(p);
*/
//erase
		
			if(V.size()>node_num){
				go_start =1;
				points.points.clear();	
			for(int t =0; t<V.size(); t++){
				p.x=V[t][0]; 
				p.y=V[t][1]; 
				p.z=0.0;	
				line.points.push_back(p);
				p.x=V[V[t].parents][0]; 
				p.y=V[V[t].parents][1]; 
				p.z=0.0;	
				line.points.push_back(p);
				pub.publish(line); 
			}
			}
//

		}//if

		else if(V.size()>node_num){
			going = true;
			x_new[0] = 1000;
			x_new[1] = 1000;

/*arrive next node or target goal */
			ros::Time ros_time;
			ros::Time ros_time_after;
			ros_time = ros::Time::now();
			ros::Duration d(10);
			while(Norm(x_new,init_node)>=0.35){
				
	
				
		     		tf::StampedTransform transform;
				int temp=0;

				while (temp==0){
				try{
				temp=1;
				listener.lookupTransform(map_topic, base_frame_topic , ros::Time(0), transform);
				}
				catch (tf::TransformException ex){
			
				temp=0;
				ros::Duration(0.1).sleep();
				}}
		
				x_new[0]=transform.getOrigin().x();
				x_new[1]=transform.getOrigin().y();
				
				ros_time_after = ros::Time::now();
				if((ros_time_after -ros_time)>d){
			
				x_new[0]=init_node[0];
				x_new[1]=init_node[1];

	
				}
				//init_pub.publish(init_goal); // think

			}
			
			init_goal.header.stamp=ros::Time(0);
		  	init_goal.header.frame_id=mapData.header.frame_id;
			init_goal.point.x=x_new[0];// insert upper code
	  		init_goal.point.y=x_new[1];
	  		init_goal.point.z=0.0;
			init_pub.publish(init_goal); // think
			
			 
//erase
//
			
			


		
			
			if(init_node!=V[target_goal.parents]){
			

				
			for(next_node= target_goal; init_node != V[next_node.parents]; next_node = V[next_node.parents]);
			

			exploration_goal.header.stamp=ros::Time(0);
			exploration_goal.header.frame_id=mapData.header.frame_id;
			exploration_goal.point.x=next_node[0];// insert upper code
			exploration_goal.point.y=next_node[1];
			exploration_goal.point.z=0.0;
			targetspub.publish(exploration_goal);

	
			
			init_node = next_node;
/*			points.color.r = 0.0/255.0;
			points.color.g = 255.0/255.0;
			points.color.b = 0.0/255.0;
			points.color.a=0.3;
	
			p.x=init_node[0];
			p.y=init_node[1];
			p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ;
*/
			ros::Duration(1).sleep();

			}
			
	     	

			else{
				//float f=3;
				//do{
				//target_goal=Steer(init_node,temp_node,f);
				//f = f-0.5;

				//}while(!goal_find(target_goal,mapData));
				//}while(!ObstacleFree(init_node,target_goal,mapData));


				//target_goal=Steer(init_node,temp_node,3);
				exploration_goal.header.stamp=ros::Time(0);
	  			exploration_goal.header.frame_id=mapData.header.frame_id;
				exploration_goal.point.x=target_goal[0];// insert upper code
	  			exploration_goal.point.y=target_goal[1];
	  			exploration_goal.point.z=0.0;
	  			targetspub.publish(exploration_goal);

			
				init_node = target_goal;


				go_start =0;
				going = false;
				V.clear();
				line.points.clear();
				points.points.clear();
				init_node.parents = 0;
				init_node.child = 0;
				V.push_back(init_node);
				pub.publish(points);
				pub.publish(line);  
	/*			points.color.r = 0.0/255.0;
			points.color.g = 0.0/255.0;
			points.color.b = 255.0/255.0;
			points.color.a=0.3;
	
			p.x=target_goal[0];
			p.y=target_goal[1];
			p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ; */
			}
/*			points.color.r = 0.0/255.0;
			points.color.g = 255.0/255.0;
			points.color.b = 0.0/255.0;
			points.color.a=0.3;
	
			pub.publish(points) ;
*/
		}//else if 

	}//checking else if

	}


pub.publish(line);  


   

ros::spinOnce();
rate.sleep();
  }return 0;}



