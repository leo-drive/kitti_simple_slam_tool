// Copyright (c) 2024 Leo Drive Teknoloji A.Ş.

#include <kitti_simple_slam_tool/kitti_simple_slam_tool.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>

pcl::PointCloud<pcl::PointXYZ>::Ptr KittiSimpleSlamTool::global_cloud(new pcl::PointCloud<pcl::PointXYZ>);

KittiSimpleSlamTool::KittiSimpleSlamTool()
        : Node("KittiSimpleSlamTool") {

    save_pc_dir_ = declare_parameter("save_pc_dir", "");
    RCLCPP_INFO(get_logger(), "save_pc_dir_: %s", save_pc_dir_.c_str());

    map_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("map_path",10);
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_topic", 10);

    gt_path_.subscribe(this, "/ground_truth", rclcpp::QoS{1}.get_rmw_qos_profile());
    pc_.subscribe(this, "/points_raw", rclcpp::QoS{1}.get_rmw_qos_profile());

    sync_ptr_.reset(new ApproximateTimeSynchronizer(ApproximateTimeSyncPolicy(10), gt_path_, pc_));

    sync_ptr_->registerCallback(
            std::bind(&KittiSimpleSlamTool::syncCallback, this, std::placeholders::_1,std::placeholders::_2));

    // Initialize NDT parameters
    const double NDTTransEpsilon = declare_parameter("NDTTransEpsilon", 0.01);
    const double NDTStepSize = declare_parameter("NDTStepSize", 0.1);
    const double NDTResolution = declare_parameter("NDTResolution", 3.0);

    std::cout<<"NDTTransEpsilon:    "<<NDTTransEpsilon<<std::endl;
    std::cout<<"NDTStepSize:    "<<NDTStepSize<<std::endl;
    std::cout<<"NDTResolution:    "<<NDTResolution<<std::endl;

    ndt = std::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
    ndt->setTransformationEpsilon(NDTTransEpsilon);
    ndt->setStepSize(NDTStepSize);
    ndt->setResolution(NDTResolution);

    aligned_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_pc_topic", 10);
}

void KittiSimpleSlamTool::syncCallback(const nav_msgs::msg::Path::ConstSharedPtr &in_gt_path_msg_,
                                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_pc_msg_) {

    geometry_msgs::msg::PoseStamped new_gt_pose_;
    auto path_counter_x_ = static_cast<int>(in_gt_path_msg_->poses.size());
    new_gt_pose_ = in_gt_path_msg_->poses[path_counter_x_ - 1];

    Eigen::Quaterniond quaternion(new_gt_pose_.pose.orientation.w,
                                  new_gt_pose_.pose.orientation.x,
                                  new_gt_pose_.pose.orientation.y,
                                  new_gt_pose_.pose.orientation.z);
    Eigen::Matrix3d gt_rotationMatrix_3d = quaternion.toRotationMatrix();
    gt_rotation_matrix_.block<3, 3>(0, 0) = gt_rotationMatrix_3d;
    gt_rotation_matrix_(0,3) = new_gt_pose_.pose.position.x;
    gt_rotation_matrix_(1,3) = new_gt_pose_.pose.position.y;
    gt_rotation_matrix_(2,3) = new_gt_pose_.pose.position.z;

    Eigen::Matrix4d map2baselink = gt_rotation_matrix_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*in_pc_msg_, *in_pc);

    pcl::transformPointCloud(*in_pc, *out_pc, map2baselink);

    if (path_counter_x_ > 3){
        ndt->setInputTarget(global_cloud);  //map
        ndt->setInputSource(out_pc);        //pc
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        ndt->align(*aligned_cloud);

        // If the NDT registration converged
        if (ndt->hasConverged()) {
            std::cout << "NDT has converged. Score: " << ndt->getFitnessScore() << std::endl;

        } else {
            std::cerr << "NDT has not converged." << std::endl;
        }
        // add aligned_cloud to global_cloud
        *global_cloud += *aligned_cloud;
        // Publish the aligned point cloud
        sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(*global_cloud, aligned_cloud_msg);
        aligned_cloud_msg.header.stamp = in_pc_msg_->header.stamp;
        aligned_cloud_msg.header.frame_id ="map";
        aligned_pc_pub_->publish(aligned_cloud_msg);
    }
    else{
        *global_cloud += *out_pc;

    }


    pcl::io::savePCDFileASCII(save_pc_dir_, *global_cloud);
    sensor_msgs::msg::PointCloud2 global_pc_share;
    pcl::toROSMsg(*out_pc, global_pc_share);

    global_pc_share.header.frame_id = "map";
    pc_pub_->publish(global_pc_share);


    Eigen::Matrix3d rotation_part = map2baselink.block<3, 3>(0, 0);
    Eigen::Quaterniond rotation_quaternion(rotation_part);
    Eigen::Vector3d translation_part = map2baselink.block<3, 1>(0, 3);

    auto pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    // Oluşturulan rotasyon ve pozisyonu mesaja atayın
    pose_stamped_msg->pose.orientation.w = rotation_quaternion.w();
    pose_stamped_msg->pose.orientation.x = rotation_quaternion.x();
    pose_stamped_msg->pose.orientation.y = rotation_quaternion.y();
    pose_stamped_msg->pose.orientation.z = rotation_quaternion.z();

    pose_stamped_msg->pose.position.x = translation_part.x();
    pose_stamped_msg->pose.position.y = translation_part.y();
    pose_stamped_msg->pose.position.z = translation_part.z();

    nav_msgs::msg::Path ground_truth_map_path;
    ground_truth_map_path.poses.push_back(*pose_stamped_msg);
    ground_truth_map_path.header.frame_id = "map";
    map_path_publisher_->publish(ground_truth_map_path);
    path_counter_++;
    std::cout<<"path_counter : "<<path_counter_ <<std::endl;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiSimpleSlamTool>());
    rclcpp::shutdown();
    return 0;
}
