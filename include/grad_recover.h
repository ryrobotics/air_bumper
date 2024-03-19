#ifndef GRAD_RECOVER_H
#define GRAD_RECOVER_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <ros/ros.h>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <air_bumper/CostMap.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

#include <random>

using namespace std;

namespace air_bumper {

class GradMap {
public:
  GradMap();
  ~GradMap();

  struct SeenDist
  {
      float d; // distance
      bool s;  // seen
      bool o; // used for thining algorithm
  };

  bool rviz_flg;
    
  Eigen::Vector3f  local_volume_size; // local volume size (m)
  Eigen::Vector3i _local_size;
  Eigen::Vector3d _map_center, _map_origin;
  float _voxel_width, _voxel_width_inv;

  SeenDist* _val_map; // Value map, store EDT value
  int _seendist_size;

  Eigen::Vector3d pos_info, grad_info;

  ros::Subscriber cost_map_sub;
  ros::Subscriber pos_sub;

  ros::Publisher gradPub;
  visualization_msgs::Marker gradROS;

  void initMap(ros::NodeHandle &nh);

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);

  void copyEdtData(const air_bumper::CostMap::ConstPtr &msg);
  void cost_map_cb(const air_bumper::CostMap::ConstPtr &msg);
  void pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
  void gradRvizPub(const Eigen::Vector3d& pos);

  Eigen::Vector3i pos2coord_edt(const Eigen::Vector3d & p);
  Eigen::Vector3d coord2pos_edt(const Eigen::Vector3i & c);
  int coord2idx_edt(const Eigen::Vector3i &c);
  bool is_inside_local_volume(const Eigen::Vector3i & crd);
  bool is_inside_local_volume(const Eigen::Vector3d & pos);

  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);
  double getDistance(int x, int y, int z);

  double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
};

/****************************************************************************************/

// Similar to pos2coord_edt
inline void GradMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - _map_origin(i)) * _voxel_width_inv);
}

// Similar to coord2pos_edt
inline void GradMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * _voxel_width + _map_origin(i);
}

inline double GradMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3d pos;
  indexToPos(id, pos);

  return getDistance(pos);
}

inline double GradMap::getDistance(int x, int y, int z) {
  Eigen::Vector3d pos;
  indexToPos(Eigen::Vector3i(x, y, z), pos);

  return getDistance(pos);
}

inline double GradMap::getDistance(const Eigen::Vector3d& pos) {
  if (!is_inside_local_volume(pos)) return -1;

  // Operation with GIE Mapping
  int idx_edt;
  idx_edt = coord2idx_edt(pos2coord_edt(pos));

  // cout << "idx: " << idx_edt << " EDT Center Dist: " << _val_map[idx_edt].d
  //      << " s: " << _val_map[idx_edt].s
  //      << " o: " << _val_map[idx_edt].o << endl;

  return _val_map[idx_edt].d * _voxel_width;
}

/****************************************************************************************/

Eigen::Vector3i GradMap::pos2coord_edt(const Eigen::Vector3d & p)
{
    Eigen::Vector3i output;

    output(0) = floorf( p(0)/ _voxel_width + 0.5f);
    output(1) = floorf( p(1)/ _voxel_width + 0.5f);
    output(2) = floorf( p(2)/ _voxel_width + 0.5f);
    return output;
}

Eigen::Vector3d GradMap::coord2pos_edt(const Eigen::Vector3i & c)
{
    Eigen::Vector3d output;
    output(0) = c(0) * _voxel_width;
    output(1) = c(1) * _voxel_width;
    output(2) = c(2) * _voxel_width;
    return output;
}

int GradMap::coord2idx_edt(const Eigen::Vector3i &c)
{
    return c(0) + c(1)*_local_size(0) + c(2)*_local_size(0)*_local_size(1);
}

bool GradMap::is_inside_local_volume(const Eigen::Vector3i & crd)
{
    if (crd(0)<0 || crd(0)>=_local_size(0) ||
        crd(1)<0 || crd(1)>=_local_size(1) ||
        crd(2)<0 || crd(2)>=_local_size(2))
        return false;
    return true;
}

bool GradMap::is_inside_local_volume(const Eigen::Vector3d & pos)
{
    Eigen::Vector3i crd = pos2coord_edt(pos);
    // cout << "Crd: x: " << crd(0) << " y: " << crd(1) << " z: " << crd(2) << endl;
    is_inside_local_volume(crd);
}

}  // namespace air_bumper

#endif // GRAD_RECOVER_H
