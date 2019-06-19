
// Inspired by Object Detection CPP by https://github.com/lysukhin/tensorflow-object-detection-cpp.git

#include "object_detection.hpp"
#include "utils.hpp"
#include <regex>

using namespace std;
using namespace tensorflow;
using namespace cv;
using tensorflow::int32;


string ROOS_DIR= "../";
string MODEL_NAME = "ssd_mobilenet_v1_coco_2017_11_17";
string PATH_TO_GRAPH = MODEL_NAME + "/frozen_inference_graph.pb";
string PATH_TO_LABELS = "/home/nuc/local/ext/tensorflow/models/research/object_detection/data/mscoco_label_map.pbtxt";

string inputLayer = "image_tensor:0";
vector<string> outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};


object_detection::object_detection(int image_width, int image_height, string frozen_graph_path, string labels_path)
{

  tensorflow::Status loadGraphStatus = loadGraph(frozen_graph_path, &session);

  labelsMap = std::map<int,std::string>();
  tensorflow::Status readLabelsMapStatus = readLabelsMapFile(labels_path, labelsMap);
   if (!readLabelsMapStatus.ok()) {
        LOG(ERROR) << "readLabelsMapFile(): ERROR" << loadGraphStatus;
        
    } else
        LOG(INFO) << "readLabelsMapFile(): labels map loaded with " << labelsMap.size() << " label(s)" << endl;
  
  shape = tensorflow::TensorShape();
  shape.AddDim(1);
  shape.AddDim((tensorflow::int64)image_height);
  shape.AddDim((tensorflow::int64)image_width);
  shape.AddDim(3);

}

int object_detection::detect(cv::Mat& frame)
{
  std::vector<Tensor> outputs;

  Tensor tensor = Tensor(tensorflow::DT_FLOAT, shape);
  tensorflow::Status readTensorStatus = readTensorFromMat(frame, tensor);
  if (!readTensorStatus.ok()) {
    cout << "Mat->Tensor conversion failed: " << readTensorStatus;
    return -1;
  }

      // Run the graph on tensor
  outputs.clear();
  tensorflow::Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
  if (!runStatus.ok()) {
    cout << "Running model failed: " << runStatus;
    return -1;
  }

      // Extract results from the outputs vector
  tensorflow::TTypes<float>::Flat scores = outputs[1].flat<float>();
  tensorflow::TTypes<float>::Flat classes = outputs[2].flat<float>();
  tensorflow::TTypes<float>::Flat numDetections = outputs[3].flat<float>();
  tensorflow::TTypes<float, 3>::Tensor boxes = outputs[0].flat_outer_dims<float,3>();

  vector<size_t> goodIdxs = filterBoxes(scores, boxes, 0.5, 0.8);
  for (size_t i = 0; i < goodIdxs.size(); i++)
    cout << "score:" << scores(goodIdxs.at(i)) << ",class:" << labelsMap[classes(goodIdxs.at(i))]
  << " (" << classes(goodIdxs.at(i)) << "), box:" << "," << boxes(0, goodIdxs.at(i), 0) << ","
  << boxes(0, goodIdxs.at(i), 1) << "," << boxes(0, goodIdxs.at(i), 2) << ","
  << boxes(0, goodIdxs.at(i), 3) << endl;

      // Draw bboxes and captions
  //cvtColor(frame, frame, COLOR_BGR2RGB);
  drawBoundingBoxesOnImage(frame, scores, classes, boxes, labelsMap, goodIdxs);

}


