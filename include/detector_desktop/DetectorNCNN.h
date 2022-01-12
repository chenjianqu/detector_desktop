/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DETECTOR_DETECTORNCNN_H
#define DETECTOR_DETECTORNCNN_H

#include "detector_desktop/DetectorBase.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <float.h>
#include <stdio.h>
#include <vector>
#include "layer.h"
#include "net.h"

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};


class DetectorNCNN : public DetectorBase{
public:
     DetectorNCNN();
     void DoDetect(cv::Mat img,Box2D::BoxesPtr &boxes) override;
private:
    int DetectYolov5(const cv::Mat& bgr, std::vector<Object>& objects);

    std::shared_ptr<ncnn::Net> model;

    const int target_size = 640;
    const float prob_threshold = 0.25f;
    const float nms_threshold = 0.45f;
};




#endif //DETECTOR_DETECTORNCNN_H
