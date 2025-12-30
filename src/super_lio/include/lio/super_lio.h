

#ifndef MvLIO_HPP_
#define MvLIO_HPP_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


#include "basic/alias.h"
#include "common/ds.h"
#include "params.h"
#include "ROSWrapper.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"


namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){
    pub_processing_time_ = nh_.advertise<geometry_msgs::PoseStamped>("/lio/processing_time", 10);
  };
  ~SuperLIO(){};

  void init();
  void run();

private:
  void process();

  bool kf_init();
  bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void Observe();
  void UpdateMap();
  void Output();
  void caceData();
  void saveMap();
  void ProcessCaceMap();

  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  alignas(64) bool effect_mask_[20000] = {false};
  alignas(64) bool effect_knn_mask_[20000] = {false};
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  int pcd_index_ = -1;

  ros::NodeHandle nh_;
  ros::Publisher pub_processing_time_;
};

} // namespace END.

#endif


