//
// Created by chen on 2021/8/9.
//

#include "detector_desktop/Segmentor.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "detector_desktop/Types.h"

Segmentor::Segmentor()
{
    euc_cluster.reset(new pcl::EuclideanClusterExtraction<PointT>);
    euc_cluster->setClusterTolerance (0.01); // 2cm，在欧氏空间里设置空间聚类容差 tolerance ，其 实是在近邻搜索中所使用的半径。
    indices_extractor.reset(new pcl::ExtractIndices<PointT>);
    voxel_filter.reset(new pcl::ApproximateVoxelGrid<PointT>);
    voxel_filter->setDownsampleAllData(false);
    voxel_filter->setLeafSize(Config::kPointCloudResolution, Config::kPointCloudResolution, Config::kPointCloudResolution);
    cv::cv2eigen(Config::kOpticalToArm, Tcb.matrix());
}

void Segmentor::SetDetector(Detector *detector_) {
    detector.reset(detector_);
}

bool Segmentor::WaitForFrame(cv::Mat &vis,Instance::InstsPtr &insts,int time_out) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cond.wait_for(lock, std::chrono::milliseconds(time_out));//等待time_out
    if(frame_queue.empty())
        return false;
    vis=frame_queue.front()->color;
    *insts=frame_queue.front()->insts;
    frame_queue.pop();
    return true;
}


bool Segmentor::WaitForFrame(Frame::Ptr &frame,int time_out) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    if(queue_cond.wait_for(lock, std::chrono::milliseconds(time_out)) == std::cv_status::timeout){//等待time_out
        return false;
    }
    if(frame_queue.empty())
        return false;
    frame=frame_queue.front();
    //frameQueue.front()->clone(frame);
    frame_queue.pop();
    return true;
}

bool Segmentor::TryPop(Frame::Ptr &frame) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    if(frame_queue.empty())
        return false;
    frame=frame_queue.front();
    frame_queue.pop();
    return true;
}

void Segmentor::BuildPointCloud(const cv::Mat& color,const cv::Mat& depth,int rows_start,int rows_end,
                                int cols_start,int cols_end,PointCloud::Ptr &pc)
{
    for ( int i=rows_start; i<rows_end; i++ ){
        for ( int j=cols_start; j<cols_end; j++ ){
            unsigned short d = depth.ptr<unsigned short> (i)[j]; // 深度值
            if ( d<100 ) continue; // 为0表示没有测量到
            if ( d >= 2500 ) continue; // 深度太大时不稳定，去掉
            PointT p;
            p.z=float(d)/cfg::kDepthFactor;
            p.x= (float(j)-cfg::kCX) * p.z / cfg::kFX;
            p.y= (float(i)-cfg::kCY) * p.z / cfg::kFY;
            p.b = color.data[ i*color.step+j*color.channels() ];
            p.g = color.data[ i*color.step+j*color.channels()+1 ];
            p.r = color.data[ i*color.step+j*color.channels()+2 ];
            pc->points.push_back(p);
        }
    }
}


PointCloud::Ptr Segmentor::BuildObjectPointCloud(const cv::Mat &color, const cv::Mat &depth, Box2D &box)
{
    PointCloud::Ptr cpc(new PointCloud);
    BuildPointCloud(color,depth,box.top,box.bottom,box.left,box.right,cpc);//本线程
    /*
        //两个并行线程构建点云
        PointCloud::Ptr pc1(new PointCloud);
        int left=box.left,top=box.top,right=box.right,bottom=box.bottom;
        int half_cols=left+(right-left)/2;
        std::thread t1(&Segmentor::BuildPointCloud, this,color,depth,top,bottom,left,half_cols,std::ref(pc1));//另一个线程
        BuildPointCloud(color,depth,top,bottom,half_cols,right,cpc);//本线程
        t1.join();
        *cpc+=*pc1;
        */
    if(cpc->size()<1000){
        return PointCloud::Ptr(new PointCloud);
    }
    PointCloud::Ptr cloud_cluster (new PointCloud);
    //体素滤波
    voxel_filter->setInputCloud(cpc );
    voxel_filter->filter(*cpc );
    //聚类分割
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cpc);
    euc_cluster->setMinClusterSize (int(cpc->size() / 3));//最小簇
    euc_cluster->setMaxClusterSize (cpc->size());//最大簇
    euc_cluster->setSearchMethod (tree);//设置搜索时所用的搜索机制，参数 tree 指向搜索时所用的搜索对象，例如kd-tree octree 等对
    euc_cluster->setInputCloud (cpc);
    euc_cluster->extract (cluster_indices);
    if(cluster_indices.empty()){
        return PointCloud::Ptr(new PointCloud);
    }
    auto itp = cluster_indices.begin ();
    pcl::PointIndices::Ptr pi_ptr(new pcl::PointIndices(*itp));
    indices_extractor->setInputCloud (cpc);
    indices_extractor->setIndices (pi_ptr);
    indices_extractor->setNegative (false);
    indices_extractor->filter (*cloud_cluster);
    //对点云进行变换
    pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, Tcb);
    //对将点云从相机坐标系转换到世界坐标系
    //Eigen::Affine3d T=Eigen::Affine3d::Identity();
    //T.translate(-Tcb.translation());
    //T.rotate(Tcb.rotation().inverse());
    //pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, T);

    /*
    auto m=Tcb.matrix();
    for(auto &p : cloud_cluster->points){
        double x=p.x,y=p.y,z=p.z;
        p.x=m(0,0)*x+m(0,1)*y+m(0,2)*z;
        p.y=m(1,0)*x+m(1,1)*y+m(1,2)*z;
        p.z=m(2,0)*x+m(2,1)*y+m(2,2)*z;
    }
     */
    return cloud_cluster;
}



void Segmentor::Run()
{
    while(cfg::is_running)
    {
        Frame::Ptr frame;
        detector->WaitForResult(frame);
        TimeCounter timeCounter;
        Instance::Insts insts;
        for(auto &box : frame->boxes){
            PointCloud::Ptr cpc= BuildObjectPointCloud(frame->color, frame->depth, box);
            insts.emplace_back(Instance(cpc,box));
            //在图片上绘制框
            if(!cpc->empty()){
                auto color=cv::Scalar(255,0,0);
                box.VisBox(frame->drawedImg,color);
            }
            else{
                auto color=cv::Scalar(0,255,0);
                box.VisBox(frame->drawedImg,color);
            }
        }
        frame->insts=insts;
        frame->MergePointCloud();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if(frame_queue.size() < Config::kSegmentorQueueSize){
                frame_queue.push(frame);
            }
            queue_cond.notify_one();
        }
    }
    Debugt("Segmentor线程结束");
}


