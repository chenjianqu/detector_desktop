//
// Created by chen on 2021/8/9.
//

#ifndef LIBTORCH_DETECT_SEGMENT_H
#define LIBTORCH_DETECT_SEGMENT_H

#include <queue>
#include <mutex>
#include <condition_variable>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "detector_desktop/Config.h"
#include "detector_desktop/Detector.h"
#include "detector_desktop/Types.h"
#include "detector_desktop/Frame.h"


class Segmentor{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Segmentor();
    void Run();
    bool WaitForFrame(cv::Mat &vis,Instance::InstsPtr &insts,int time_out) ;
    bool WaitForFrame(Frame::Ptr &frame,int time_out) ;
    void SetDetector(Detector *detector_);
    bool TryPop(Frame::Ptr &frame);

private:
    PointCloud::Ptr BuildObjectPointCloud(const cv::Mat &color, const cv::Mat &depth, Box2D &box);
    void BuildPointCloud(const cv::Mat& color,const cv::Mat& depth,int rows_start,int rows_end,int cols_start,
                         int cols_end,PointCloud::Ptr &pc);
    void PallelBuildOjectPointCloud(const cv::Mat &color, const cv::Mat &depth, std::vector<Box2D::Ptr> &boxs,
                                    std::vector<Instance::Ptr> &insts);


    std::shared_ptr<pcl::EuclideanClusterExtraction<PointT>> euc_cluster;//点云 欧式聚类分割
    std::shared_ptr<pcl::ExtractIndices<PointT>> indices_extractor;//按点云索引提取点云子集
    std::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> voxel_filter;//体素滤波

    Detector::Ptr detector;

    std::queue<Frame::Ptr> frame_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cond;

    Eigen::Affine3d Tcb;
};




#endif //LIBTORCH_DETECT_SEGMENT_H
