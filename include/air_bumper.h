#ifndef AIR_BUMPER_H
#define AIR_BUMPER_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <vector>

#include <mavros/mavros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <air_bumper/activity_pose.h>
#include <air_bumper/confirmation.h>

#include <grad_recover.h>
#include <std_srvs/Empty.h>

using namespace std;

namespace air_bumper{
class AirBumper
{
public:
  AirBumper(ros::NodeHandle &nh);
  ~AirBumper();

  void state_cb(const mavros_msgs::State::ConstPtr &msg);
  void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void confirm_cb(const air_bumper::confirmation::ConstPtr &msg);
  void set_pos_control(double pos_x, double pos_y, double pos_z, string type);
  void crash_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void crash_recover_FSM();
  bool auto_take_off();
  void engageServiceCall();
  Eigen::Vector3d grad_recover_control(const Eigen::Vector3d &pos, double &dist);

  mavros_msgs::State current_state;
  ros::ServiceClient engage_client;

private:
  shared_ptr<GradMap> grad_map_;

  double flight_alt;
  double react_d; // Diameter of MAV
  double react_dz; // Height of MAV
  double react_w;
  int waypoint_num;
  int waypoint_index;
  int waypoint_lst;  // last waypoint for crash recovery
  double waypoints[30][3];
  bool sim_flg, grad_flg;
  int step;      // step for FSM
  int flight_mode; // 1->normal flight, 2->collision reaction

  bool crash_flag = false; // true->crash happened & still not react
  bool exec_flag = false;

  // for px4 controller
  bool confirm_flag = false;
  int action_header = 0;
  int response_header = 0;

  // Buffer the last several pos for reaction, size need to be manually set
  boost::circular_buffer<geometry_msgs::PoseStamped> pos_buffer; // 30

  vector<geometry_msgs::PoseStamped> obs_pos;

  geometry_msgs::PoseStamped reaction_pos;
  geometry_msgs::Twist vel_vector;
  geometry_msgs::PoseStamped local_pos;
  geometry_msgs::PointStamped crash_point;

  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
  air_bumper::activity_pose motion_vector;
  Eigen::Vector3d rad_crash;

  ros::Subscriber state_sub;
  ros::Subscriber local_pos_sub;
  ros::Subscriber sub_crash;
  ros::Subscriber confirm_sub;
  ros::Publisher motion_pub;
  ros::Publisher vec_pub;
  ros::Publisher react_pub;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::Time last_request;
};

}// namespace air_bumper

#endif // AIR_BUMPER_H
