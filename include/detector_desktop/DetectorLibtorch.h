//
// Created by chen on 2021/8/22.
//

#ifndef DETECTOR_DETECTORLIBTORCH_H
#define DETECTOR_DETECTORLIBTORCH_H

#include <torch/script.h>
#include <torch/torch.h>
#include "detector_desktop/DetectorBase.h"
#include "detector_desktop/Config.h"

class DetectorLibtorch : public DetectorBase{
public:
    typedef std::shared_ptr<DetectorLibtorch> Ptr;
    DetectorLibtorch();
    void DoDetect(cv::Mat img,Box2D::BoxesPtr &boxes) override;
private:
    std::vector<torch::Tensor> NMS(torch::Tensor preds, float score_thresh= 0.5, float iou_thresh= 0.5);
    std::shared_ptr<torch::jit::script::Module> module;
    torch::DeviceType device_type;
};




#endif //DETECTOR_DETECTORLIBTORCH_H
