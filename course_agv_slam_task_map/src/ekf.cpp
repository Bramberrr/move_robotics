#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

class ekf{

public:
    ekf(ros::NodeHandle &n);
	~ekf();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta; 
    // match threshold;
    float match_th;
    // bool
    bool isFirstScan;
    // status
    VectorXd status;
    // covariance
    MatrixXd covariance;
    // noise R
    MatrixXd noise_R;
    // noise Q
    MatrixXd noise_Q;
    // landmark num
    int landMark_num;
    // noises
    float noise_motion, noise_measure;
    // count the non-zero elements in status
    int nonZero_cnt;
    
    // init all 
    void initAll();
    // predict phase
    void predict(nav_msgs::Odometry odom);
    // update phase
    void update(visualization_msgs::MarkerArray input);
    // landMarks to XY matrix
    Eigen::MatrixXd landMarksToXY(visualization_msgs::MarkerArray input);
    // landMarks to r-phi matrix
    Vector2d cartesianToPolar(double x, double y);
    // update feature map
    void updateFeatureMap(Eigen::MatrixXd newFeatures);
    // get motion Jacobian
    MatrixXd getMotionJacobian();
    // get observation Jacobian
    MatrixXd getObservJacobian();
    // angle normalization
    double angleNorm(double angle);
    // calc 2D distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // find nearest map points
    int findNearestMap(Vector2d point);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber icpOdom_sub;
    tf::TransformBroadcaster ekf_broadcaster;
    void publishResult();
};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/ekf/robot_x", robot_x);
	n.getParam("/ekf/robot_y", robot_y);
	n.getParam("/ekf/robot_theta", robot_theta);

    n.getParam("/ekf/match_th", match_th);
    n.getParam("/ekf/landMark_num", landMark_num);
    n.getParam("/ekf/noise_motion", noise_motion);
    n.getParam("/ekf/noise_measure", noise_measure);

    this->initAll();

    isFirstScan = true;
    landMark_sub = n.subscribe("/landMarks", 1, &ekf::update, this);
    icpOdom_sub = n.subscribe("/icp_odom", 1, &ekf::predict, this);
}

void ekf::predict(nav_msgs::Odometry odom)
{
    // TODO: Please complete the predict phase or motion model
}

void ekf::update(visualization_msgs::MarkerArray input)
{   
    double time_0 = (double)ros::Time::now().toSec();

    MatrixXd landMarkFeatures = this->landMarksToXY(input);
    cout<<"-------------New LM Cnt:    "<<landMarkFeatures.cols()<<endl;

    // TODO: Please complete the update phase or observation model

    // initial
    if(isFirstScan)
    {
        this->updateFeatureMap(landMarkFeatures);
        return;
    }
    
    this->publishResult();

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}

void ekf::initAll()
{   
    // TODO: You can initial here if you need
}

Eigen::MatrixXd ekf::landMarksToXY(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

void ekf::updateFeatureMap(Eigen::MatrixXd newFeatures)
{   
    // TODO:  Please complete this function if you need
    if(isFirstScan)
    {   
        // initial the map by landmarks in first scan

        isFirstScan = false;
    }
    else
    {   
    }
}

int ekf::findNearestMap(Vector2d point)
{   
    // TODO: Please complete the NN search
}

Eigen::MatrixXd ekf::getMotionJacobian()
{
    // TODO: Please complete the Jocobian Calculation of Motion
}

Eigen::MatrixXd ekf::getObservJacobian()
{
    // TODO: Please complete the Jocobian Calculation of Observation

}

Vector2d ekf::cartesianToPolar(double x, double y)
{
    float r = std::sqrt(x*x + y*y);
    float phi = angleNorm(std::atan2(y, x));
    Vector2d r_phi(r, phi);
    return r_phi;
}

float ekf::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{   
    return std::sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]));
}

double ekf::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

void ekf::publishResult()
{
    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(status(2));

    geometry_msgs::TransformStamped ekf_trans;
    ekf_trans.header.stamp = ros::Time::now();
    ekf_trans.header.frame_id = "world_base";
    ekf_trans.child_frame_id = "ekf_slam";

    ekf_trans.transform.translation.x = status(0);
    ekf_trans.transform.translation.y = status(1);
    ekf_trans.transform.translation.z = 0.0;
    ekf_trans.transform.rotation = odom_quat;

    ekf_broadcaster.sendTransform(ekf_trans);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}