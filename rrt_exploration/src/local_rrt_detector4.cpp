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
#include "nav_msgs/Odometry.h"



// global variables
nav_msgs::OccupancyGrid mapData;
nav_msgs::Odometry odomData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal,init_goal;
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;

rdm r; // for genrating random numbers



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData=*msg;
}
void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
odomData=*msg;
}

 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 

geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;

points.points.push_back(p);

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

  ros::param::param<float>(ns+"/eta", eta, 0.5);
  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot_1/map"); 
  ros::param::param<std::string>(ns+"/robot_frame", base_frame_topic, "/robot_1/base_link"); 
//---------------------------------------------------------------
ros::Subscriber sub= nh.subscribe(map_topic, 100 ,mapCallBack);	
ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);	
ros::Subscriber odom_sub= nh.subscribe("/robot_1/odom", 100 ,odomCallBack);	
ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10); //1
ros::Publisher init_pub = nh.advertise<geometry_msgs::PointStamped>("/init_points", 1); //1
ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);

ros::Rate rate(100); 
 
 
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
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
int new_start=1;
int go_start =0;
bool going = false;
float xr,yr;
char checking;
vec<float> x_rand,x_nearest,x_new,near_inds,next_node,init_node,temp_node,target_goal,temp_goal;

tf::TransformListener listener;
// Main loop
while (ros::ok()){


	if(!going){
		// Sample free
		x_rand.clear();
		//xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
		//yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;
		if(drand()>0.09){
			xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
			yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;
			}
		else{
			xr= 4.56;
			yr= -2.72;
			}

		x_rand.push_back( xr ); x_rand.push_back( yr );


		// Nearest
		x_nearest=Nearest(V,x_rand);

		// Steer

		x_new=Steer(x_nearest,x_rand,eta);


		// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
		checking=ObstacleFree(x_nearest,x_new,mapData);
		
		}
	else{
		checking = 0;
		}


		if (checking==-1){

				
			  	

				if(new_start==1){
					exploration_goal.header.stamp=ros::Time(0);
			  		exploration_goal.header.frame_id=mapData.header.frame_id;
					exploration_goal.point.x=x_new[0];// insert upper code
			  		exploration_goal.point.y=x_new[1];
			  		exploration_goal.point.z=0.0;
			
				  	targetspub.publish(exploration_goal);
					
				        init_node =x_new;


					new_start = 0;
					line.points.clear();
					points.points.clear();
				  	V.clear();
				
					V.push_back(x_new);

					}
				else if(go_start==1){
					exploration_goal.header.stamp=ros::Time(0);
			  		exploration_goal.header.frame_id=mapData.header.frame_id;
					exploration_goal.point.x=x_new[0];// insert upper code
			  		exploration_goal.point.y=x_new[1];
			  		exploration_goal.point.z=0.0;
					go_start =0;
					temp_node.clear();
					temp_node.push_back(exploration_goal.point.x);
					temp_node.push_back(exploration_goal.point.y);
					target_goal= Nearest(V,temp_node);
					//if(infoGain(V,target_goal, mapData)>10) target_goal = V[target_goal.parents];
					points.color.r = 255.0/255.0;
					points.color.g = 0.0/255.0;
					points.color.b = 0.0/255.0;
					points.color.a=0.3;
	
					p.x=x_new[0]; 
					p.y=x_new[1]; 
					p.z=0.0;
			  		points.points.push_back(p);
			  		pub.publish(points) ;
	
	//goal setting
					}
				}
		

//going end
	  else if ((checking==1)&&(new_start==0)&&(go_start==0) || going){ //new start = 0 
 		if(V.size()<=10){
  	       		near_inds=find_near_node(V,x_new,eta);
	        	x_new = chooseparent(V,x_new,near_inds,mapData);   //value
	        	if(x_new.empty()){
				continue;
			}

	 		V.push_back(x_new);
          		//p.x=x_new[0]; 
			//p.y=x_new[1]; 
			//p.z=0.0;
          		//points.points.push_back(p);	 	
			//pub.publish(points);
		     	//V=rewire(V,x_new,near_inds,mapData);
/*
		 	p.x=x_new[0]; 
			p.y=x_new[1]; 
			p.z=0.0;
		 	line.points.push_back(p);
		        x_new = V[x_new.parents];
		 	p.x=x_new[0]; 
			p.y=x_new[1]; 
			p.z=0.0;
		 	line.points.push_back(p);
*/
//erase
		
			if(V.size()>10){
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

		else if(V.size()>10){
			going = true;
			x_new[0] = 1000;
			x_new[1] = 1000;
			
/*arrive next node or target goal */
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
			
			
//neareast target goal
/*
			float min=Norm(V[0],temp_node);
			int min_index;
			float temp_n;

			for (int w=0;w<6;w++)
			{
			temp_n=Norm(V[w],temp_node);
				if (temp_n<=min){
					min=temp_n;
					min_index=w;
					}
			}*/

		
			
			if(init_node!=V[target_goal.parents]){
		
				//next_node= find_init_next(V,exploration_goal,init_node);
				for(next_node= target_goal; init_node != V[next_node.parents]; next_node = V[next_node.parents]){
				if(infoGain(V,next_node, mapData)>10){
//&&(init_node != V[next_node.parents])) next_node = V[next_node.parents];

					p.x=1;
				p.y=1;
				p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ;
				}
				}

				//for(next_node= target_goal; init_node != V[next_node.parents];next_node = V[next_node.parents]);
				exploration_goal.header.stamp=ros::Time(0);
  				exploration_goal.header.frame_id=mapData.header.frame_id;
				exploration_goal.point.x=next_node[0];// insert upper code
  				exploration_goal.point.y=next_node[1];
  				exploration_goal.point.z=0.0;
  				targetspub.publish(exploration_goal);

	
				p.x=3;
				p.y=3;
				p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ;
				init_node = next_node;
				points.color.r = 0.0/255.0;
			points.color.g = 255.0/255.0;
			points.color.b = 0.0/255.0;
			points.color.a=0.3;
	
			p.x=init_node[0];
			p.y=init_node[1];
			p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ;

			ros::Duration(1).sleep();

			}
			
	     	

			else{
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
				V.push_back(init_node);
				pub.publish(points);
				pub.publish(line);  
				points.color.r = 0.0/255.0;
			points.color.g = 0.0/255.0;
			points.color.b = 255.0/255.0;
			points.color.a=0.3;
	
			p.x=target_goal[0];
			p.y=target_goal[1];
			p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ;
			}
			points.color.r = 0.0/255.0;
			points.color.g = 255.0/255.0;
			points.color.b = 0.0/255.0;
			points.color.a=0.3;
	
			pub.publish(points) ;

		}//else if 

	}//checking else if

	


pub.publish(line);  


   

ros::spinOnce();
rate.sleep();
  }return 0;}













