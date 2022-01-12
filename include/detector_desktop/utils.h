/*******************************************************
 *
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of detector_desktop.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DETECTOR_UTILS_H
#define DETECTOR_UTILS_H

#include "detector_desktop/Types.h"
#include <visualization_msgs/Marker.h>

void BuildLineStripMarker(Instance &inst,visualization_msgs::Marker &msg);

void BuildTextMarker(Instance &inst,visualization_msgs::Marker &msg);

#endif //DETECTOR_UTILS_H
