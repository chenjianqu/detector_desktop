//
// Created by chen on 2021/8/22.
//

#ifndef DETECTOR_DETECTORBASE_H
#define DETECTOR_DETECTORBASE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "detector_desktop/Types.h"

class DetectorBase{ //虚基类
public:
    typedef std::shared_ptr<DetectorBase> Ptr;
    virtual ~DetectorBase()=default;//虚析构函数使用默认函数
    virtual void DoDetect(cv::Mat img,Box2D::BoxesPtr &boxes)=0;//虚函数
};



#endif //DETECTOR_DETECTORBASE_H
