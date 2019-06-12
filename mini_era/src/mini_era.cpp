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

using namespace std;
using namespace cv;


string PATH_TO_GRAPH =  "../models/src/model/frozen_inference_graph.pb";
string PATH_TO_LABELS = "/home/nuc/local/ext/tensorflow/models/research/object_detection/data/mscoco_label_map.pbtxt";


int main(int charc, char** charv)
{
  VideoCapture cap(charv[1]); 

  if(!cap.isOpened()) { 
      cout << "Cannot open the camera" << endl;
      return 1;
  }

  Mat frame;

  cap >> frame;
  if (frame.empty()) {
  	cout <<"empty frame" << endl;
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