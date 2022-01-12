//
// Created by chen on 2021/8/22.
//
#include <iostream>
#include "detector_desktop/DetectorLibtorch.h"

using namespace std;

DetectorLibtorch::DetectorLibtorch()
{
    if(torch::cuda::is_available() && Config::use_gpu){
        device_type=at::kCUDA;
        Debugt("Libtorch Device Type:CUDA");
    }
    else{
        device_type=at::kCPU;
        Debugt("Libtorch Device Type:CPU");
    }
    module.reset(new torch::jit::script::Module);
    *module = torch::jit::load(Config::kModelPath);//模型加载
    module->to(device_type);//转换到设备类型
}

void DetectorLibtorch::DoDetect(cv::Mat img,Box2D::BoxesPtr &boxes)
{
    boxes.reset(new Box2D::Boxes);
    cv::Mat img_size;
    cv::resize(img, img_size, cv::Size(640, 384));
    cv::cvtColor(img_size, img_size, cv::COLOR_BGR2RGB);
    torch::Tensor imgTensor = torch::from_blob(img_size.data, {img_size.rows, img_size.cols,3},torch::kByte);
    imgTensor = imgTensor.permute({2,0,1});
    imgTensor = imgTensor.toType(torch::kFloat);
    imgTensor = imgTensor.div(255);
    imgTensor = imgTensor.unsqueeze(0);

    auto predvalue=module->forward({imgTensor.to(device_type)});//模型预测

    torch::Tensor preds =predvalue.toTuple()->elements()[0].toTensor().to(at::kCPU);//取出张量并转换为CPU类型
    std::vector<torch::Tensor> dets = NMS(preds, 0.4, 0.5);

    if (!dets.empty()){
        for (int i=0; i < dets[0].sizes()[0]; ++ i){
            boxes->emplace_back(
                    std::max(0,static_cast<int>(dets[0][i][0].item().toFloat()*img.cols/640.0)),//left
                    std::max(0,static_cast<int>(dets[0][i][1].item().toFloat()*img.rows/384.0)),//top
                    std::min(img.cols,static_cast<int>(dets[0][i][2].item().toFloat()*img.rows/640.0)),//right
                    std::min(img.rows,static_cast<int>(dets[0][i][3].item().toFloat()*img.rows/384.0)),//bottom
                    dets[0][i][4].item().toFloat(),//score
                    dets[0][i][5].item().toInt()//classid
                    );
        }
    }

}


std::vector<torch::Tensor> DetectorLibtorch::NMS(torch::Tensor preds, float score_thresh, float iou_thresh)
{
    std::vector<torch::Tensor> output;
    for (size_t i=0; i < preds.sizes()[0]; ++i)
    {
        torch::Tensor pred = preds.select(0, i);
        // Filter by scores
        torch::Tensor scores = pred.select(1, 4) * std::get<0>( torch::max(
                pred.slice(1, 5, pred.sizes()[1]), 1));
        pred = torch::index_select(pred, 0, torch::nonzero(scores > score_thresh).
        select(1, 0));
        if (pred.sizes()[0] == 0) continue;
        // (center_x, center_y, w, h) to (left, top, right, bottom)
        pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
        pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
        pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
        pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);
        // Computing scores and classes
        std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(
                pred.slice(1, 5, pred.sizes()[1]), 1);
        pred.select(1, 4) = pred.select(1, 4) * std::get<0>(max_tuple);
        pred.select(1, 5) = std::get<1>(max_tuple);
        torch::Tensor  dets = pred.slice(1, 0, 6);

        torch::Tensor keep = torch::empty({dets.sizes()[0]});
        torch::Tensor areas = (dets.select(1, 3) - dets.select(1, 1)) *
                (dets.select(1, 2) - dets.select(1, 0));
        std::tuple<torch::Tensor, torch::Tensor> indexes_tuple = torch::sort(dets.select(1, 4),
                                                                             0, 1);
        torch::Tensor v = std::get<0>(indexes_tuple);
        torch::Tensor indexes = std::get<1>(indexes_tuple);
        int count = 0;
        while (indexes.sizes()[0] > 0)
        {
            keep[count] = (indexes[0].item().toInt());
            count += 1;

            // Computing overlaps
            torch::Tensor lefts = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor tops = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor rights = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor bottoms = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor widths = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor heights = torch::empty(indexes.sizes()[0] - 1);
            for (size_t i=0; i<indexes.sizes()[0] - 1; ++i)
            {
                lefts[i] = std::max(dets[indexes[0]][0].item().toFloat(), dets[indexes[i + 1]][0].item().toFloat());
                tops[i] = std::max(dets[indexes[0]][1].item().toFloat(), dets[indexes[i + 1]][1].item().toFloat());
                rights[i] = std::min(dets[indexes[0]][2].item().toFloat(), dets[indexes[i + 1]][2].item().toFloat());
                bottoms[i] = std::min(dets[indexes[0]][3].item().toFloat(), dets[indexes[i + 1]][3].item().toFloat());
                widths[i] = std::max(float(0), rights[i].item().toFloat() - lefts[i].item().toFloat());
                heights[i] = std::max(float(0), bottoms[i].item().toFloat() - tops[i].item().toFloat());
            }
            torch::Tensor overlaps = widths * heights;
            // FIlter by IOUs
            torch::Tensor ious = overlaps / (areas.select(0, indexes[0].item().toInt()) +
                    torch::index_select(areas, 0, indexes.slice(0, 1, indexes.sizes()[0])) - overlaps);
            indexes = torch::index_select(indexes, 0, torch::nonzero(ious <= iou_thresh).select(1, 0) + 1);
        }
        keep = keep.toType(torch::kInt64);
        output.push_back(torch::index_select(dets, 0, keep.slice(0, 0, count)));
    }
    return output;
}

