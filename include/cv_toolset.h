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

#include "globals.h"

#ifdef USE_OLD_MODEL

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

#else

/*****************************************************************************/
/* NEW: PyTorch TinyYOLOv2 support (May 2022)                                */
typedef struct {
  int width;
  int height;
  int c;
} dim_t;

typedef struct {
  char class_label[256];
  long id;
  double confidence;
  /* Bounding box coordinates */
  double x_top_left;
  double y_top_left;
  double width;
  double height;
} detection_t;

int cv_toolset_init(char *python_module, char *model_weights);
detection_t *run_object_classification(unsigned char *data, dim_t dimensions,
					   char *filename, int *nboxes);
/*****************************************************************************/

#endif

#endif
