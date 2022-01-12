//
// Created by chen on 2021/8/30.
//

#ifndef DETECTOR_DESKTOP_VIEWER_H
#define DETECTOR_DESKTOP_VIEWER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <queue>

class Viewer{
public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer()=default;

    void Run();

private:
    std::queue<cv::Mat> detectorImgQueue;
    std::queue<cv::Mat> segmentorImgQueue;
};





#endif //DETECTOR_DESKTOP_VIEWER_H
