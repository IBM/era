/*
 * Copyright 2018 IBM
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "object_detection.hpp"
#include "costmap.hpp"

using namespace std;
using namespace cv;


string PATH_TO_GRAPH =  "../models/src/model/frozen_inference_graph.pb";
string PATH_TO_LABELS = "/home/nuc/local/ext/tensorflow/models/research/object_detection/data/mscoco_label_map.pbtxt";


int main(int charc, char** charv)
{
  costmap map(0.01);
  map.test("/home/nuc/r0_3.pcd",  0, 0, 0, 0, 0, 0);
  map.test("/home/nuc/r1_3.pcd",  3, 0, 2,  0, -1.57, 0); 

  //map.add_cloud("/home/nuc/r0_3.pcd", -1, 1, 0, 0, 0, 0);
  //map.add_cloud("/home/nuc/r1_3.pcd", 1, -2, 0, 0, 0, 1.57); 

  map.write_octomap("merge.bt");

  //map.update();

  //cout << "created a costmap" <<endl;

  //map.write_octomap("r1_3.bt");


  return 1;



  VideoCapture cap(charv[1]); 

  if(!cap.isOpened()) { 
      cout << "Cannot open the camera" << endl;
      return 1;
  }

  Mat frame;

  cap >> frame;
  if (frame.empty()) {
  	cout <<"empty frame" << endl;
    return 1;
  }

  object_detection detector(frame.cols, frame.rows, PATH_TO_GRAPH, PATH_TO_LABELS);

  while(true) {
    
    cap >> frame;
    if (frame.empty()) {
   		cout <<"empty frame" << endl;
      break;
    }

    detector.detect(frame);
 
    imshow( "Frame", frame );
 
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  
  
  cap.release();
  destroyAllWindows();
     
  return 0;

}
