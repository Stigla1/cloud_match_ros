#ifndef PC_LOCALIZER_H
#define PC_LOCALIZER_H


#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace pc_localizer
{
class PCLocalizer
{
 public:

  explicit PCLocalizer(ros::NodeHandle nh);

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void process();

  Eigen::Matrix4f randomTransform();

  void param_loader(ros::NodeHandle nh);

  Eigen::Matrix4f final_transform;
  Eigen::Matrix3f  final_transform_3;
  Eigen::Quaternionf final_transform_quat;

  bool got_transform;

  std::string base_frame_name;
  std::string object_frame_name;


 private:
  ros::Subscriber sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, model_cloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_n, model_cloud_n;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_FPFH, model_FPFH;
  ros::Publisher pub_transformed, pub_after_t, pub_drawing;
  bool debug_, ral_test;
  float m_leaf_size, t_leaf_size;
  float m_norm_search, t_norm_search, fpfh_search;
  int scp_iter, scp_samples;
  float scp_similarity, scp_distance, scp_inliers, scp_threshold;
  int icp_iter, ral_test_iterations;
  float icp_distance, icp_transform_e, icp_euclidian_fit_e, icp_threshold;

  std::string template_file, input_cloud_topic, drawing_file;

  
};
}

#endif  // PC_LOCALIZER_H