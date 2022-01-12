//
// Created by chen on 2021/8/27.
//

#ifndef DETECTOR_UTILS_H
#define DETECTOR_UTILS_H

#include "detector_desktop/Types.h"

#include <visualization_msgs/Marker.h>



void BuildLineStripMarker(Instance &inst,visualization_msgs::Marker &msg);

void BuildTextMarker(Instance &inst,visualization_msgs::Marker &msg);

#endif //DETECTOR_UTILS_H
