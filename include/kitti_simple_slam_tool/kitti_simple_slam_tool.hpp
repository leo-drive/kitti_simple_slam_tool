// Copyright (c) 2024 Leo Drive Teknoloji A.Ş.

#ifndef  KITTI_SIMPLE_SLAM_TOOL__KITTI_SIMPLE_SLAM_TOOL_HPP_
#define  KITTI_SIMPLE_SLAM_TOOL__KITTI_SIMPLE_SLAM_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <iostream>


class KittiSimpleSlamTool : public rclcpp::Node {

public:

    KittiSimpleSlamTool();

    std::string save_pc_dir_;
    using gt_path_msg = nav_msgs::msg::Path;
    using pc_msg = sensor_msgs::msg::PointCloud2;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<gt_path_msg, pc_msg>;
    using ApproximateTimeSynchronizer = message_filters::Synchronizer<ApproximateTimeSyncPolicy>;
    std::shared_ptr<ApproximateTimeSynchronizer> sync_ptr_;

    message_filters::Subscriber<gt_path_msg> gt_path_;
    message_filters::Subscriber<pc_msg> pc_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr gt_path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr map_path_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    void syncCallback(const nav_msgs::msg::Path::ConstSharedPtr & in_gt_path_msg_,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_pc_msg_);

    int path_counter_ = 0;
    Eigen::Matrix4d gt_rotation_matrix_ = Eigen::Matrix4d::Identity();

    static pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;

    /// NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pc_pub_;



};

#endif  //  KITTI_SIMPLE_SLAM_TOOL__KITTI_SIMPLE_SLAM_TOOL_HPP_
