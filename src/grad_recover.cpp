#include "grad_recover.h"

namespace air_bumper {

GradMap::GradMap() {}

GradMap::~GradMap() {
  delete [] _val_map;
}

void GradMap::initMap(ros::NodeHandle &nh)
{
  // load params
  nh.param<float>("/GIE_mapping/voxel_width",_voxel_width,0.1);
  nh.param<float>("/GIE_mapping/local_size_x",local_volume_size(0),10);
  nh.param<float>("/GIE_mapping/local_size_y",local_volume_size(1),10);
  nh.param<float>("/GIE_mapping/local_size_z",local_volume_size(2),3);

  cost_map_sub = nh.subscribe<air_bumper::CostMap>("cost_map", 1, &GradMap::cost_map_cb, this);
  pos_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &GradMap::pos_cb, this);
  gradPub = nh.advertise<visualization_msgs::Marker>("grad_dir", 1);

  _voxel_width_inv = 1 / _voxel_width;

  _local_size << int(local_volume_size(0) * _voxel_width_inv),
                 int(local_volume_size(1) * _voxel_width_inv),
                 int(local_volume_size(2) * _voxel_width_inv);

  _seendist_size = _local_size(0) * _local_size(1) *_local_size(2) * sizeof(SeenDist);

  _val_map = new SeenDist[_local_size(0) * _local_size(1) *_local_size(2)];
  memset(_val_map, 0, static_cast<size_t>(_seendist_size));

  _map_center = coord2pos_edt((_local_size.cast<float>() * 0.5).cast<int>());

  _map_origin << 0, 0, 0;

  rviz_flg = true;
}

void GradMap::copyEdtData(const air_bumper::CostMap::ConstPtr &msg)
{
  if (msg->x_size != _local_size(0)
      || msg->y_size != _local_size(1)
      || msg->z_size != _local_size(2))
  {
    printf("Dimension mismatch during map data copying!\n Map is not copied!\n");
    return;
  }

  memcpy(_val_map, msg->payload8.data(), static_cast<size_t>(_seendist_size));
}

void GradMap::cost_map_cb(const air_bumper::CostMap::ConstPtr &msg)
{
  // _map_origin = origin;
  // _map_origin << msg->x_origin, msg->y_origin, msg->z_origin;
  copyEdtData(msg);
  // cout<<"_edt_data size:" << _edt_data.size()<<endl;
}

void GradMap::pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (rviz_flg)
  {
    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    pos_info = pos;
    gradRvizPub(pos);
  }
}

// Note the grad is the direction away from obstacles!
double GradMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  // if (!is_inside_local_volume(pos)) {
  //   grad.setZero();
  //   ROS_WARN("Outside local volume!");
  //   return 0;
  // }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * _voxel_width * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * _voxel_width_inv;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
        // cout << values[x][y][z] << " ";
      }
      // cout << std::endl;

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * _voxel_width_inv;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * _voxel_width_inv;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= _voxel_width_inv;

  /*Simple method to debug Rviz*/
  // Eigen::Vector3i id;
  // posToIndex(_map_center, id);

  // grad(0) = (getDistance(id(0) + 1, id(1), id(2)) -
  //                getDistance(id(0) - 1, id(1), id(2))) *
  //               0.5 * _voxel_width_inv;
  // grad(1) = (getDistance(id(0), id(1) + 1, id(2)) -
  //                getDistance(id(0), id(1) - 1, id(2))) *
  //               0.5 * _voxel_width_inv;
  // grad(2) = (getDistance(id(0), id(1), id(2) + 1) -
  //                getDistance(id(0), id(1), id(2) - 1)) *
  //               0.5 * _voxel_width_inv;
  //
  // double dist = getDistance(id);

  // cout << "grad: "<<grad(0)<<" "<<grad(1)<<" "<<grad(2)<<endl;
  // cout << getDistance(id(0) + 1, id(1), id(2)) << " " << getDistance(id(0) - 1, id(1), id(2)) << endl;
  // cout << getDistance(id(0), id(1) + 1, id(2)) << " " << getDistance(id(0), id(1) - 1, id(2)) << endl;
  // cout << getDistance(id(0), id(1), id(2) + 1) << " " << getDistance(id(0), id(1), id(2) - 1) << endl;

  return dist;
}

void GradMap::gradRvizPub(const Eigen::Vector3d& pos) {
  Eigen::Vector3d grad;

  // Evaluate the EDT and its gradient at the robot's current position
  double dist = getDistWithGrad(_map_center, grad);

  grad_info = grad;

  // If the gradient is not significant, no need to move the robot
  if (grad.norm() <= 1e-3)  return;

  // Normalize the gradient to get a unit vector
  grad.normalize();

  // Pub to Rviz
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = grad.norm();  // Scale the arrow length to match the gradient magnitude
  marker.scale.y = 0.1;  // Scale the arrow width
  marker.scale.z = 0.1;  // Scale the arrow height
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.position.z = pos[2];

  // xyz -> rpy -> quaternion
  double dx = grad[0];
  double dy = grad[1];
  double dz = grad[2];
  double yaw = atan2(dy, dx);
  double pitch = atan2(sqrt(dx*dx + dy*dy), dz);
  double roll = 0.0;  // Assuming roll is 0 degrees
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = sin(yaw/2) * cos(pitch/2);
  marker.pose.orientation.w = cos(yaw/2) * cos(pitch/2);

  gradPub.publish(marker);

  // cout << "norm: "<<grad(0)<<" "<<grad(1)<<" "<<grad(2)<<endl;
  // cout << endl;
}

} // namespace air_bumper