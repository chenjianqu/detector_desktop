//
// Created by chen on 2021/8/24.
//

#ifndef DETECTOR_TYPES_H
#define DETECTOR_TYPES_H

#include <chrono>
#include <istream>

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

using namespace std;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef std::chrono::steady_clock::time_point TimeType;


class TimeCounter{
public:
    TimeCounter():t_start(std::chrono::steady_clock::now()){}
    void Start(){
        t_start=std::chrono::steady_clock::now();
    }
    float CountMs(){
        std::chrono::duration<float,std::milli> duration=std::chrono::steady_clock::now()-t_start;
        Start();
        return duration.count();
    }
private:
    std::chrono::steady_clock::time_point t_start;
};




class Box2D{
public:
    typedef std::shared_ptr<Box2D> Ptr;
    typedef std::vector<Box2D> Boxes;
    typedef std::shared_ptr<Boxes> BoxesPtr;
    Box2D()=default;

    Box2D(const int &left_,const int &top_,const int &right_,const int &bottom_,const float &score_,const int &classId_):
    left(left_),top(top_),right(right_),bottom(bottom_),score(score_),classId(classId_)
    {}

    void VisBox(cv::Mat &img,cv::Scalar &color) const{
        cv::rectangle(img, cv::Rect(left,top, (right - left), (bottom - top)), color, 2);
        cv::putText(img, class_names[classId] + ": " + cv::format("%.2f", score),
                    cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    bool IsBelong(int x,int y) const{
        return x>=left && x<right && y>top && y<bottom;
    }

    std::string GetClassLabel() const{
        return class_names[classId];
    }

    static std::vector<std::string> class_names;

    int left,top,right,bottom;
    float score;
    int classId;
};


class Instance{
public:
    typedef std::shared_ptr<Instance> Ptr;
    typedef std::vector<Instance> Insts;
    typedef std::shared_ptr<Instance::Insts> InstsPtr;

    Instance()=default;

    Instance(const PointCloud::Ptr &pc_,Box2D &box2D_)
    :box(box2D_),pc(pc_)
    {
        if(!pc->empty()){
            pcl::getMinMax3D(*pc, minPt, maxPt);
            centerPt.x=(minPt.x+maxPt.x)/2.0f;
            centerPt.y=(minPt.y+maxPt.y)/2.0f;
            centerPt.z=(minPt.z+maxPt.z)/2.0f;
        }
    }

    PointT minPt, maxPt,centerPt;//获得点云的各个轴的极值
    Box2D box;
    PointCloud::Ptr pc;

    size_t id;
};




#endif //DETECTOR_TYPES_H
