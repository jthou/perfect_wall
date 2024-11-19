#include <iostream>
#include <vector>
#include <random>
#include <thread>

#include <ros/ros.h>  
#include <sensor_msgs/PointCloud2.h>  

#include <Eigen/Dense>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


#include <chrono>

std::vector<pcl::PointXYZ> generateRandomPoints2(int num_points)
{
    std::vector<pcl::PointXYZ> points;

    std::random_device rd;
    std::mt19937 gen(rd()); // random twist engine

    std::uniform_real_distribution<> dis(10, 5);
    std::uniform_real_distribution<> dis1(-50, 50);
    std::uniform_real_distribution<> dis2(0, 40);


    for(int i = 0 ; i < num_points ; i++)
    {
        pcl::PointXYZ point;
        point.x = dis(gen);
        point.y = dis1(gen);
        point.z = dis2(gen);
        points.push_back(point);
    }
    return points;
}


int main(int argc, char** argv) {  
    // 初始化 ROS 节点  
    ros::init(argc, argv, "perfect_wall");  
    
    // 创建节点句柄  
    ros::NodeHandle nh;  

    // 创建点云发布者  
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/aeb/fisheye_image", 1);  
    ros::Publisher pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/Omni_Perception/pointclouds", 1);

    // 设置循环频率为 7Hz  
    ros::Rate loop_rate(7);  
    auto points = generateRandomPoints2(10000);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(points.size());
    std::copy(points.begin(), points.end(), cloud->points.begin());

    // 旋转点云
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI/2.5, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud, transform);

    //平移点云
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translation() << 50, 0, 0;
    pcl::transformPointCloud(*cloud, *cloud, transform2);

    // 转换点云到 ROS 消息格式  
    sensor_msgs::PointCloud2 pcl_msg;  
    pcl::toROSMsg(*cloud, pcl_msg);  
    pcl_msg.header.frame_id = "map"; // 设置坐标系  

    while (ros::ok()) {  
        // 更新时间戳并发布点云数据  
        pcl_msg.header.stamp = ros::Time::now();  
        pcl_pub.publish(pcl_msg);  
        pcl_pub2.publish(pcl_msg);
        ROS_INFO("Publishing point cloud!");  
        ros::spinOnce();  
        loop_rate.sleep();  
    }  

    return 0;  
}  

