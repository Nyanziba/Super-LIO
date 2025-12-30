

#ifndef MvLIO_RE_HPP_
#define MvLIO_RE_HPP_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "params.h"
#include "ROSWrapper.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"


namespace LI2Sup{

class SuperLIOReLoc{
public:
  SuperLIOReLoc(){
    nh_.setCallbackQueue(&self_queue_);
    pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/lio/global_map", 10);
    sub_init_pose_ = nh_.subscribe("/initialpose", 1, &SuperLIOReLoc::initialpose_callback, this);
    re_init_pose_ = BASIC::SE3(BASIC::SO3(BASIC::M3::Identity()), BASIC::V3(0, 0, 0));

    BASIC::V3 init_t = BASIC::V3(g_init_px, g_init_py, g_init_pz);
    Eigen::Matrix3d init_R = (Eigen::AngleAxisd(g_init_yaw / 180 * M_PI, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(g_init_pitch /180 * M_PI, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(g_init_roll / 180 * M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
                
    BASIC::M3 init_R2 = init_R.cast<BASIC::scalar>();             
      
    re_init_pose_ = BASIC::SE3(BASIC::SO3(init_R2), init_t);

  };
  ~SuperLIOReLoc(){};

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



  /// relocation related
  ros::NodeHandle nh_;
  ros::Publisher pub_map_;
  ros::Subscriber sub_init_pose_;
  sensor_msgs::PointCloud2 msg_global_map_;
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  ros::CallbackQueue self_queue_;
  void Refreshcallback(){
    self_queue_.callAvailable();
  }
  BASIC::CloudPtr init_obs_data_;
  bool flg_init_state_ = false;
  bool flg_init_reset_ = false;
  bool flg_get_init_guess_ = false;
  int  init_frame_count_ = 0;
  BASIC::SE3 re_init_pose_;


};

} // namespace END.

#endif


