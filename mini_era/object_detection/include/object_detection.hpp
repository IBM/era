#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
#include "opencv2/opencv.hpp"


class object_detection
{
public:
	tensorflow::TensorShape shape;
  std::unique_ptr<tensorflow::Session> session;
  std::map<int, std::string> labelsMap;

	object_detection(int image_width, int image_height, std::string frozen_graph_path, std::string labels_path);

	int detect(cv::Mat& frame);


};