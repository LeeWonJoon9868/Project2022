#ifndef functions_H
#define functions_H
#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"

// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
float randomize();
};


template <typename T>
class vec : public std::vector<T>{
	 
private:
	typedef std::vector<T> base_vector;

 	
public:
    typedef typename base_vector::size_type       size_type;
    typedef typename base_vector::iterator        iterator;
    typedef typename base_vector::const_iterator  const_iterator;
    float cost ;
    float parents;
    float child;

    using base_vector::operator[];
    using base_vector::operator=;

    using base_vector::begin;
    using base_vector::clear;
    using base_vector::end;
    using base_vector::erase;
    using base_vector::push_back;
    using base_vector::reserve;
    using base_vector::resize;
    using base_vector::size;
    using base_vector::empty;
    

 
};



//Norm function prototype
float Norm( vec<float> , vec<float> );

//sign function prototype
float sign(float );

//Nearest function prototype
vec<float> Nearest(  vec< vec<float>  > , vec<float> );
vec<float> Near_avg(  vec< vec<float>  > , vec<float>  ,nav_msgs::OccupancyGrid );
//Steer function prototype
vec<float> Steer(  vec<float>, vec<float>, float );

//gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &,vec<float>);

//ObstacleFree function prototype
int ObstacleFree(vec<float> , vec<float>  , nav_msgs::OccupancyGrid);
char goal_find(vec<float>,nav_msgs::OccupancyGrid);
vec<float> find_near_node(vec< vec<float>  >,vec<float>, float );

vec<float> chooseparent(vec< vec<float>  > ,vec<float> , vec<float> ,nav_msgs::OccupancyGrid );

vec< vec<float>  > rewire(vec< vec<float>  > ,vec<float> , vec<float> ,nav_msgs::OccupancyGrid );
vec<float> find_init_next(vec< vec<float>  >V,geometry_msgs::PointStamped exploration_goal, vec<float> init_node);

int infoGain( vec<float> next_node,nav_msgs::OccupancyGrid mapData);

#endif
