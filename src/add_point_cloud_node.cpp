#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

double _step = 0.05; 
double _radius = 0.5; 
double _length = 0.5;

Eigen::Quaterniond currentRotation_;
Eigen::Vector3d currentPosition_;
pcl::PointCloud<pcl::PointXYZ> crashSumPoints_;

void generatePointClound(const double& x, const double& y, const double& z);

void generatePointClound(Eigen::Vector3d vec);

ros::Publisher pcPub_;

// Subscribe the current pose of UAV
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos = *msg;
}

geometry_msgs::PointStamped crash_point;

void cranshHandler(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  crash_point = *msg;

  Eigen::Vector3d vec(crash_point.point.x ,crash_point.point.y ,crash_point.point.z );

  Eigen::Vector3d vec_norm;

  vec_norm = vec / vec.norm();

  vec_norm = vec_norm*_length;

  generatePointClound(vec_norm);

  ROS_INFO("Pose x:%f, y:%f, z:%f", currentPosition_.x(),
          currentPosition_.y(),currentPosition_.z());
  ROS_INFO("CrashPC x:%f, y:%f, z:%f", vec_norm[0],vec_norm[1],vec_norm[2]);

}

void odomHandler(const nav_msgs::Odometry::ConstPtr &msg){

  currentPosition_.x() = msg->pose.pose.position.x;
  currentPosition_.y() = msg->pose.pose.position.y;
  currentPosition_.z() = msg->pose.pose.position.z;

  currentRotation_.w() = msg->pose.pose.orientation.w;
  currentRotation_.x() = msg->pose.pose.orientation.x;
  currentRotation_.y() = msg->pose.pose.orientation.y;
  currentRotation_.z() = msg->pose.pose.orientation.z;

}


int main(int argc, char **argv){

  ros::init(argc, argv, "add_point_cloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ros::Subscriber crashSub = nh.subscribe("imu_detection/crash_point", 10, cranshHandler);
  // Subscribe the current pos（feedback signal）
  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
  ros::Subscriber odomSub = nh.subscribe("mavros/local_position/odom",10,odomHandler);

  pcPub_ = nh.advertise<sensor_msgs::PointCloud2>("/forbid_reg_cloud",1);

  nh_p.param<double>("step", _step, 0.05);
  nh_p.param<double>("radius", _radius, 1.0);
  nh_p.param<double>("length", _length, 1.0);

   ros::Rate loop_rate(10);

  while (ros::ok())
  {

      // for test
      // generatePointClound(1,1,1);

      // if(vec_norm.norm() !=0)
      // {
      //   generatePointClound(vec_norm);
      // }

      ROS_INFO_ONCE("Ready!");

      sensor_msgs::PointCloud2 msgPub;

      pcl::toROSMsg(crashSumPoints_,msgPub);

      msgPub.header.frame_id = "map";
      msgPub.header.stamp = ros::Time::now();

      pcPub_.publish(msgPub);

      ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;

}


void generatePointClound(const double& x, const double& y, const double& z){

  pcl::PointCloud<pcl::PointXYZ> butter;

  for(double i=x-_radius; i<= x+_radius; i+= _step){
    for(double j=y-_radius; j<= y+_radius; j+= _step){
        for(double k=z-_radius; k<= z+_radius; k+= _step){

          if(sqrt((i-x)*(i-x)+(j-y)*(j-y)+(k-z)*(k-z)) <= _radius){

            if(std::abs(x*(i-x)+y*(j-y)+z*(k-z))  <= 1e-4 ){

              pcl::PointXYZ p(i,j,k);

              butter.push_back(p);

            }
          }
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> trans;
  Eigen::Matrix4d M;
  M.block<3,3>(0,0) = currentRotation_.toRotationMatrix();
  M.col(3)[0] = currentPosition_.x();
  M.col(3)[1] = currentPosition_.y();
  M.col(3)[2] = currentPosition_.z();

  pcl::transformPointCloud(butter,trans,M);

  crashSumPoints_ += trans;
}

void generatePointClound(Eigen::Vector3d vec){
  double x= vec[0];
  double y= vec[1];
  double z= vec[2];

  generatePointClound(x,y,z);

}