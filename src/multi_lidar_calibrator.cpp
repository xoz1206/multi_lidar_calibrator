/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * multi_lidar_calibrator.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include "multi_lidar_calibrator.h"


void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header.frame_id = parent_frame_;
  in_publisher.publish(cloud_msg);
}

void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_first_child_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_second_child_cloud_msg)
{
  pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr in_first_child_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr in_second_child_cloud(new pcl::PointCloud<PointT>);

  pcl::PointCloud<PointT>::Ptr first_child_filtered_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr second_child_filtered_cloud (new pcl::PointCloud<PointT>);

  pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
  pcl::fromROSMsg(*in_first_child_cloud_msg, *in_first_child_cloud);
  pcl::fromROSMsg(*in_second_child_cloud_msg, *in_second_child_cloud);

  parent_frame_ = in_parent_cloud_msg->header.frame_id;
  first_child_frame_ = in_first_child_cloud_msg->header.frame_id;
  second_child_frame_ = in_second_child_cloud_msg->header.frame_id;

  DownsampleCloud(in_first_child_cloud, first_child_filtered_cloud, voxel_size_);
  DownsampleCloud(in_second_child_cloud, second_child_filtered_cloud, voxel_size_);

  // Initializing Normal Distributions Transform (first_ndt).
  pcl::NormalDistributionsTransform<PointT, PointT> first_ndt;
  pcl::NormalDistributionsTransform<PointT, PointT> second_ndt;

  first_ndt.setTransformationEpsilon(ndt_epsilon_);
  first_ndt.setStepSize(ndt_step_size_);
  first_ndt.setResolution(ndt_resolution_);
  first_ndt.setMaximumIterations(ndt_iterations_);
  first_ndt.setInputSource(first_child_filtered_cloud);
  first_ndt.setInputTarget(in_parent_cloud);

  second_ndt.setTransformationEpsilon(ndt_epsilon_);
  second_ndt.setStepSize(ndt_step_size_);
  second_ndt.setResolution(ndt_resolution_);
  second_ndt.setMaximumIterations(ndt_iterations_);
  second_ndt.setInputSource(second_child_filtered_cloud);
  second_ndt.setInputTarget(in_parent_cloud);

  pcl::PointCloud<PointT>::Ptr first_output_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr second_output_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr total_output_cloud(new pcl::PointCloud<PointT>);

  Eigen::Translation3f first_init_translation(first_initial_x_, first_initial_y_, first_initial_z_);
  Eigen::AngleAxisf first_init_rotation_x(first_initial_roll_, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf first_init_rotation_y(first_initial_pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf first_init_rotation_z(first_initial_yaw_, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f second_init_translation(second_initial_x_, second_initial_y_, second_initial_z_);
  Eigen::AngleAxisf second_init_rotation_x(second_initial_roll_, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf second_init_rotation_y(second_initial_pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf second_init_rotation_z(second_initial_yaw_, Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f first_init_guess_ = (first_init_translation * first_init_rotation_z * first_init_rotation_y * first_init_rotation_x).matrix();
  Eigen::Matrix4f second_init_guess_ = (second_init_translation * second_init_rotation_z * second_init_rotation_y * second_init_rotation_x).matrix();

  if(first_current_guess_ == Eigen::Matrix4f::Identity())
  {
    first_current_guess_ = first_init_guess_;
  }

  if(second_current_guess_ == Eigen::Matrix4f::Identity())
  {
    second_current_guess_ = second_init_guess_;
  }

  first_ndt.align(*first_output_cloud, first_current_guess_);
  second_ndt.align(*second_output_cloud, second_current_guess_);

  std::cout << "first child Normal Distributions Transform converged:" << first_ndt.hasConverged ()
            << " score: " << first_ndt.getFitnessScore () << " prob:" << first_ndt.getTransformationProbability() << std::endl;

  std::cout << "second child Normal Distributions Transform converged:" << second_ndt.hasConverged ()
            << " score: " << second_ndt.getFitnessScore () << " prob:" << second_ndt.getTransformationProbability() << std::endl;

  std::cout << "first child transformation from " << first_child_frame_ << " to " << parent_frame_ << std::endl;
  std::cout << "second child transformation from " << second_child_frame_ << " to " << parent_frame_ << std::endl;

  
  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*in_first_child_cloud, *first_output_cloud, first_ndt.getFinalTransformation());
  pcl::transformPointCloud (*in_second_child_cloud, *second_output_cloud, second_ndt.getFinalTransformation());

  first_current_guess_ = first_ndt.getFinalTransformation();
  second_current_guess_ = second_ndt.getFinalTransformation();

  Eigen::Matrix3f rotation_matrix = first_current_guess_.block(0,0,3,3);
  Eigen::Vector3f translation_vector = first_current_guess_.block(0,3,3,1);
  std::cout << "This transformation can be replicated using:" << std::endl;
  std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
            << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
            << " /" << first_child_frame_ << " 10" << std::endl;

  std::cout << "Corresponding transformation matrix:" << std::endl
            << std::endl << first_current_guess_ << std::endl << std::endl;

  Eigen::Matrix3f rotation_matrix2 = second_current_guess_.block(0,0,3,3);
  Eigen::Vector3f translation_vector2 = second_current_guess_.block(0,3,3,1);
  std::cout << "This transformation can be replicated using:" << std::endl;
  std::cout << "rosrun tf static_transform_publisher " << translation_vector2.transpose()
            << " " << rotation_matrix2.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
            << " /" << second_child_frame_ << " 10" << std::endl;

  std::cout << "Corresponding transformation matrix:" << std::endl
            << std::endl << second_current_guess_ << std::endl << std::endl;

  if(first_ndt.getFitnessScore() < 0.5 && second_ndt.getFitnessScore() < 0.5)
  {
    std_msgs::Float32MultiArray first_transform_data;
    float first_array[4][4] = {
      { rotation_matrix(0,0), rotation_matrix(0,1), rotation_matrix(0,2), translation_vector[0] },
      { rotation_matrix(1,0), rotation_matrix(1,1), rotation_matrix(1,2), translation_vector[1] },
      { rotation_matrix(2,0), rotation_matrix(2,1), rotation_matrix(2,2), translation_vector[2] },
      { 0, 0, 0, 1}
    };

    std::vector<float> first_vec;

    for(int i = 0; i < 4; i++)
      for(int k = 0; k < 4; k++)
        first_vec.push_back(first_array[i][k]);

    std_msgs::Float32MultiArray second_transform_data;
    float second_array[4][4] = {
      { rotation_matrix2(0,0), rotation_matrix2(0,1), rotation_matrix2(0,2), translation_vector2[0] },
      { rotation_matrix2(1,0), rotation_matrix2(1,1), rotation_matrix2(1,2), translation_vector2[1] },
      { rotation_matrix2(2,0), rotation_matrix2(2,1), rotation_matrix2(2,2), translation_vector2[2] },
      { 0, 0, 0, 1}
    };

    std::vector<float> second_vec;

    for(int i = 0; i < 4; i++)
      for(int k = 0; k < 4; k++)
        second_vec.push_back(second_array[i][k]);

    first_transform_data.data = first_vec;
    second_transform_data.data = second_vec;

    first_transform_publisher_.publish(first_transform_data);
    second_transform_publisher_.publish(second_transform_data);

    ros::shutdown();
  }

  for(size_t i = 0; i < first_output_cloud->points.size(); i++)
    total_output_cloud->points.push_back(first_output_cloud->points[i]);
  ROS_INFO(" first size : %zu ", total_output_cloud->points.size());

  for(size_t i = 0; i < second_output_cloud->points.size(); i++)
    total_output_cloud->points.push_back(second_output_cloud->points[i]);
  ROS_INFO(" second size : %zu ", total_output_cloud->points.size());

  for(size_t i = 0; i < in_parent_cloud->points.size(); i++)
    total_output_cloud->points.push_back(in_parent_cloud->points[i]);
  ROS_INFO(" total size : %zu ", total_output_cloud->points.size());

  PublishCloud(first_calibrated_cloud_publisher_, first_output_cloud);
  PublishCloud(second_calibrated_cloud_publisher_, second_output_cloud);
  PublishCloud(total_calibrated_cloud_publisher_, total_output_cloud);

  // timer end
  //auto end = std::chrono::system_clock::now();
  //auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  //std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

}

/*void ROSMultiLidarCalibratorApp::InitialPoseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr in_initialpose)
{
  ROS_INFO("[%s] Initial Pose received.", __APP_NAME__);
  tf::Quaternion pose_quaternion(in_initialpose->pose.pose.orientation.x,
                   in_initialpose->pose.pose.orientation.y,
                   in_initialpose->pose.pose.orientation.z,
                   in_initialpose->pose.pose.orientation.w);

  //rotation
  initialpose_quaternion_ = pose_quaternion;

  //translation
  initialpose_position_.setX(in_initialpose->pose.pose.position.x);
  initialpose_position_.setY(in_initialpose->pose.pose.position.y);
  initialpose_position_.setZ(in_initialpose->pose.pose.position.z);


}*/

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
  pcl::VoxelGrid<PointT> voxelized;
  voxelized.setInputCloud(in_cloud_ptr);
  voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  voxelized.filter(*out_cloud_ptr);
}

void ROSMultiLidarCalibratorApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string points_parent_topic_str, points_first_child_topic_str, points_second_child_topic_str;
  std::string initial_pose_topic_str = "/initialpose";
  std::string first_calibrated_points_topic_str;
  std::string second_calibrated_points_topic_str;
  std::string total_calibrated_points_topic_str;

  in_private_handle.param<std::string>("first_output_topic", first_calibrated_points_topic_str, "/points_calibrated");
  ROS_INFO("[%s] first_calibrated_points_topic_str: %s",__APP_NAME__, first_calibrated_points_topic_str.c_str());

  in_private_handle.param<std::string>("second_output_topic", second_calibrated_points_topic_str, "/points_calibrated");
  ROS_INFO("[%s] second_calibrated_points_topic_str: %s",__APP_NAME__, second_calibrated_points_topic_str.c_str());
  
  in_private_handle.param<std::string>("total_output_topic", total_calibrated_points_topic_str, "/points_calibrated");
  ROS_INFO("[%s] total_calibrated_points_topic_str: %s",__APP_NAME__, total_calibrated_points_topic_str.c_str());

  in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
  ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

  in_private_handle.param<std::string>("points_first_child_src", points_first_child_topic_str, "points_raw");
  ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_first_child_topic_str.c_str());

  in_private_handle.param<std::string>("points_second_child_src", points_second_child_topic_str, "points_raw");
  ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_second_child_topic_str.c_str());

  in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
  ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

  in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
  ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

  in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
  ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

  in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
  ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

  in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
  ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

  in_private_handle.param<double>("first_x", first_initial_x_, 0.0);
  in_private_handle.param<double>("first_y", first_initial_y_, 0.0);
  in_private_handle.param<double>("first_z", first_initial_z_, 0.0);
  in_private_handle.param<double>("first_roll", first_initial_roll_, 0.0);
  in_private_handle.param<double>("first_pitch", first_initial_pitch_, 0.0);
  in_private_handle.param<double>("first_yaw", first_initial_yaw_, 0.0);

  in_private_handle.param<double>("second_x", second_initial_x_, 0.0);
  in_private_handle.param<double>("second_y", second_initial_y_, 0.0);
  in_private_handle.param<double>("second_z", second_initial_z_, 0.0);
  in_private_handle.param<double>("second_roll", second_initial_roll_, 0.0);
  in_private_handle.param<double>("second_pitch", second_initial_pitch_, 0.0);
  in_private_handle.param<double>("second_yaw", second_initial_yaw_, 0.0);

  ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
           first_initial_x_, first_initial_y_, first_initial_z_,
           first_initial_roll_, first_initial_pitch_, first_initial_yaw_);

  ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
           second_initial_x_, second_initial_y_, second_initial_z_,
           second_initial_roll_, second_initial_pitch_, second_initial_yaw_);

  //generate subscribers and synchronizer
  cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                       points_parent_topic_str, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

  cloud_first_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                          points_first_child_topic_str, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_first_child_topic_str.c_str());

  cloud_second_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                          points_second_child_topic_str, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_second_child_topic_str.c_str());

  /*initialpose_subscriber_ = node_handle_.subscribe(initial_pose_topic_str, 10,
                                                            &ROSMultiLidarCalibratorApp::InitialPoseCallback, this);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, initial_pose_topic_str.c_str());*/

  first_calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(first_calibrated_points_topic_str, 1);
  second_calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(second_calibrated_points_topic_str, 1);
  total_calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(total_calibrated_points_topic_str, 1);
  first_transform_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("/first_transform_matrix", 1, true);
  second_transform_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("/second_transform_matrix", 1, true);

  ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, first_calibrated_points_topic_str.c_str());
  ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, second_calibrated_points_topic_str.c_str());
  ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, total_calibrated_points_topic_str.c_str());

  cloud_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
                                                     *cloud_parent_subscriber_,
                                                     *cloud_first_child_subscriber_,
                                                     *cloud_second_child_subscriber_);

  cloud_synchronizer_->registerCallback(boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2, _3));

}


void ROSMultiLidarCalibratorApp::Run()
{
  ros::NodeHandle private_node_handle("~");

  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END",__APP_NAME__);
}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
  //initialpose_quaternion_ = tf::Quaternion::getIdentity();
  first_current_guess_ = Eigen::Matrix4f::Identity();
  second_current_guess_ = Eigen::Matrix4f::Identity();
}