/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include<string>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>


#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>

#include "detector_desktop/Config.h"
#include "detector_desktop/Types.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

std::shared_ptr<message_filters::Synchronizer<sync_pol> > image_sub;
std::shared_ptr<ros::Publisher> pcl_pub;
std::shared_ptr<ros::Subscriber> image_info_sub;

std::shared_ptr<pcl::EuclideanClusterExtraction<PointT>> eucCluster;//点云 欧式聚类分割
std::shared_ptr<pcl::ExtractIndices<PointT>> indicesExtractor;//按点云索引提取点云子集
std::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> voxelFilter;//体素滤波

image_geometry::PinholeCameraModel cam_model;


Eigen::Affine3d Toptical2cam;
Eigen::Affine3d Tcam2arm;
Eigen::Affine3d Toptical2arm;

void BuildPointCloud(const cv::Mat color,const cv::Mat depth,int rows_start,int rows_end,int cols_start,int cols_end,PointCloud::Ptr &pc)
{
    //cout<<"BuildPointCloud: size:"<<color.size()<<" row:"<<rows_start<<" "<<rows_end<<" cols:"<<cols_start<<" "<<cols_end<<endl;

    for ( int i=rows_start; i<rows_end; i++ ){
        for ( int j=cols_start; j<cols_end; j++ ){
            unsigned short d = depth.ptr<unsigned short> (i)[j]; // 深度值
            if ( d<100 ) continue; // 为0表示没有测量到
            if ( d >= 2500 ) continue; // 深度太大时不稳定，去掉
            PointT p;
            p.z=float(d)/Config::kDepthFactor;
            p.x= (float(j)-Config::kCX) * p.z / Config::kFX;
            p.y= (float(i)-Config::kCY) * p.z / Config::kFY;

            p.b = color.data[ i*color.step+j*color.channels() ];
            p.g = color.data[ i*color.step+j*color.channels()+1 ];
            p.r = color.data[ i*color.step+j*color.channels()+2 ];
            pc->points.push_back(p);
        }
    }
}


void BuildPointCloudWithCameraInfo(const cv::Mat color,const cv::Mat depth,int rows_start,int rows_end,int cols_start,int cols_end,PointCloud::Ptr &pc)
{
    for ( int i=rows_start; i<rows_end; i++ ){
        for ( int j=cols_start; j<cols_end; j++ ){
            unsigned short d = depth.ptr<unsigned short> (i)[j]; // 深度值
            if ( d<100 ) continue; // 为0表示没有测量到
            if ( d >= 2500 ) continue; // 深度太大时不稳定，去掉
            PointT p;
            p.z=float(d)/Config::kDepthFactor;
            p.x=(float(j)-cam_model.cx())*p.z/cam_model.fx();
            p.y=(float(i)-cam_model.cy())*p.z/cam_model.fy();

            p.b = color.data[ i*color.step+j*color.channels() ];
            p.g = color.data[ i*color.step+j*color.channels()+1 ];
            p.r = color.data[ i*color.step+j*color.channels()+2 ];
            pc->points.push_back(p);
        }
    }
}


PointCloud::Ptr BuildOjectPointCloud(const cv::Mat &color,const cv::Mat &depth)
{
    PointCloud::Ptr cpc(new PointCloud);
    //BuildPointCloud(color,depth,0,color.rows,0,color.cols,cpc);//本线程
    BuildPointCloudWithCameraInfo(color,depth,0,color.rows,0,color.cols,cpc);//本线程
    if(cpc->size()<1000){
        return PointCloud::Ptr(new PointCloud);
    }
    PointCloud::Ptr cloud_cluster (new PointCloud);
    pcl::transformPointCloud(*cpc, *cpc, Toptical2arm);
    //体素滤波
    /*
    voxelFilter->setInputCloud( cpc );
    voxelFilter->filter( *cpc );

    //cout<<"voxelFilter:"<<cpc->size()<<endl;

    //聚类分割
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cpc);
    eucCluster->setMinClusterSize (int(cpc->size()/3));//最小簇
    eucCluster->setMaxClusterSize (cpc->size());//最大簇
    eucCluster->setSearchMethod (tree);//设置搜索时所用的搜索机制，参数 tree 指向搜索时所用的搜索对象，例如kd-tree octree 等对
    eucCluster->setInputCloud (cpc);
    eucCluster->extract (cluster_indices);

    //cout<<"cluster_indices:"<<cluster_indices.size()<<endl;
    if(cluster_indices.empty()){
        return PointCloud::Ptr(new PointCloud);
    }

    std::vector<pcl::PointIndices>::const_iterator itp = cluster_indices.begin ();
    pcl::PointIndices::Ptr pi_ptr(new pcl::PointIndices(*itp));
    indicesExtractor->setInputCloud (cpc);
    indicesExtractor->setIndices (pi_ptr);
    indicesExtractor->setNegative (false);
    indicesExtractor->filter (*cloud_cluster);


    //pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, Tcb);
    //对将点云从相机坐标系转换到世界坐标系
    //Eigen::Affine3d T=Eigen::Affine3d::Identity();
    //T.translate(-Tcb.translation());
    //T.rotate(Tcb.rotation().inverse());
    //pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, T);
*/
    /*
    auto m=Tcb.matrix();
    for(auto &p : cloud_cluster->points){
        double x=p.x,y=p.y,z=p.z;
        p.x=m(0,0)*x+m(0,1)*y+m(0,2)*z;
        p.y=m(1,0)*x+m(1,1)*y+m(1,2)*z;
        p.z=m(2,0)*x+m(2,1)*y+m(2,2)*z;
    }
     */
   // cout<<"cloud_cluster:"<<cloud_cluster->size()<<endl;

    return cpc;
}


void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
    cv_bridge::CvImageConstPtr rgb_ptr;
    try{
        rgb_ptr = cv_bridge::toCvShare(msgRGB);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr depth_ptr;
    try{
        depth_ptr = cv_bridge::toCvShare(msgD);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat color=rgb_ptr->image;
    cv::Mat depth=depth_ptr->image;

    cv::cvtColor(color,color,CV_BGR2RGB);
    PointCloud::Ptr pc=BuildOjectPointCloud(color,depth);

    cout<<pc->size()<<endl<<endl;

    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*(pc),msg_pc);
    msg_pc.header.frame_id=Config::kPointCloudMsgFrame;
    pcl_pub->publish(msg_pc);
}


void InfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cam_model.fromCameraInfo(info_msg);
    image_sub->registerCallback(boost::bind(GrabRGBD,_1,_2));
    image_info_sub->shutdown();

    cout<<"fx:"<<cam_model.fx()<<endl;
    cout<<"fy:"<<cam_model.fy()<<endl;
    cout<<"cx:"<<cam_model.cx()<<endl;
    cout<<"cy:"<<cam_model.cy()<<endl;
    cout<<"Tx:"<<cam_model.Tx()<<endl;
    cout<<"Ty:"<<cam_model.Ty()<<endl;

    ROS_INFO("相机参数获取成功");
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");//防止中文乱码
    ros::init(argc, argv, "detect_node");
    ros::start();
    ros::NodeHandle nh;

    std::string config_file;
    if(!nh.getParam("config_file",config_file)){
        config_file="/home/chen/ws/detector_ws/src/detector_desktop/config/config.yaml";
    }
    Config cfg(config_file);

    cv::cv2eigen(Config::kOpticalToCam, Toptical2cam.matrix());
    cv::cv2eigen(Config::kCamToArm, Tcam2arm.matrix());
    cv::cv2eigen(Config::kOpticalToArm, Toptical2arm.matrix());
    //Tcb=Tcb.inverse();

    eucCluster.reset(new pcl::EuclideanClusterExtraction<PointT>);
    eucCluster->setClusterTolerance (0.01); // 2cm，在欧氏空间里设置空间聚类容差 tolerance ，其 实是在近邻搜索中所使用的半径。
    indicesExtractor.reset(new pcl::ExtractIndices<PointT>);
    voxelFilter.reset(new pcl::ApproximateVoxelGrid<PointT>);
    voxelFilter->setDownsampleAllData(false);
    voxelFilter->setLeafSize(0.005,0.005,0.005);

    //接受RGB图和深度图
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, Config::kRealsense2RgbTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, Config::kRealsense2DepthTopic, 1);
    image_sub.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(10), rgb_sub,depth_sub));

    image_info_sub.reset(new ros::Subscriber(nh.subscribe(Config::kRealsense2CameraInfoTopic, 1, InfoCallback)));
    pcl_pub.reset( new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(Config::kPointCloudOutputTopic, 1)));

    cout<<"loop:"<<endl;
    int counter=0;

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();

    ros::shutdown();
    ROS_INFO_STREAM("程序结束");
    return 0;
}