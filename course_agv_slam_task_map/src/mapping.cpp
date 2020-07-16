#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace Eigen;

class mapping{

public:
    mapping(ros::NodeHandle &n);
	~mapping();
    ros::NodeHandle& n;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Publisher map_pub;
    tf::TransformListener listener;
    // transform
    tf::StampedTransform transform;
    // global grid map
    nav_msgs::OccupancyGrid grid_map;
    // some variables
    string world_frame, sensor_frame;
    int map_height, map_width;
    float map_res;
    // grid points location
    MatrixXd grid_points;
    
    // main process
    void process(sensor_msgs::LaserScan input);
};

mapping::~mapping()
{}

mapping::mapping(ros::NodeHandle& n):
    n(n)
{
    // get the params
    n.getParam("/mapping/world_frame", world_frame);
	n.getParam("/mapping/sensor_frame", sensor_frame);

	n.getParam("/mapping/map_height", map_height);
	n.getParam("/mapping/map_width", map_width);
	n.getParam("/mapping/map_res", map_res);
    
    // iniitialization
    grid_map.info.height = map_height;
    grid_map.info.width = map_width;
    grid_map.info.resolution = map_res;
    grid_map.header.frame_id = world_frame;

    // set origin of map
    grid_map.info.origin.position.x = - float(grid_map.info.width) / 2 * grid_map.info.resolution;
    grid_map.info.origin.position.y = - float(grid_map.info.height) / 2 * grid_map.info.resolution;
    grid_map.info.origin.orientation.w = 1;

    // fill with -1 / unknown in the map
    grid_map.data.assign(map_width * map_height, -1);

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map_mine", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &mapping::process, this);
}

void mapping::process(sensor_msgs::LaserScan input)
{
    cout<<"------seq:  "<<input.header.seq<<endl;

    // transformation is needed
    listener.lookupTransform(world_frame, sensor_frame,  
                                ros::Time(0), transform);

    // TODO: Please complete your mapping code

    // publish
    map_pub.publish(grid_map);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle n;

    mapping mapping_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}