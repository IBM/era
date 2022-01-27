/*
 * Copyright 2022 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _CV_TOOLSET_H
#define _CV_TOOLSET_H

//#include <opencv2/opencv.hpp>
#include "globals.h"

/* Pre-defined labels used by the computer vision kernel */
typedef enum {
  myself = -1,
  no_label = 0,
  bicycle,  /* bus */
  car,
  pedestrian,
  truck,
  num_object_labels
} label_t;

status_t cv_toolset_init();
label_t run_object_classification(unsigned tr_val);
//label_t run_object_classification_2(const cv::Mat& image);

#endif
