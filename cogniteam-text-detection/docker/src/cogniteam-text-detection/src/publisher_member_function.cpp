#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace cv;
using namespace cv::dnn;
using namespace std::chrono_literals;
using std::placeholders::_1;


class TextDetection : public rclcpp::Node{


public:
  TextDetection(): Node("textDetection_Node"){
    bbox_publisher_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>("bounding_box", 10);
    bbox_image_publisher_= this->create_publisher<sensor_msgs::msg::Image>("image_bbox", 10);
    cbbox_image_publisher_= this->create_publisher<sensor_msgs::msg::CompressedImage>("image_bbox/compressed", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&TextDetection::topic_callback, this, _1));
    this->declare_parameter<std::string>("model", "src/cogniteam-text-detection/resource/frozen_east_text_detection.pb");
    this->declare_parameter<int>("width", 320);
    this->declare_parameter<int>("height", 320);
    this->declare_parameter<float>("thr", 0.5);
    this->declare_parameter<float>("nms", 0.4);
    this->declare_parameter<std::string>("device", "cpu");

    this->get_parameter_or<std::string>("model",model, "src/cogniteam-text-detection/resource/frozen_east_text_detection.pb");
    this->get_parameter_or<int>("width",inpWidth, 320);
    this->get_parameter_or<int>("height",inpHeight, 320);
    this->get_parameter_or<float>("thr",nmsThreshold, 0.5);
    this->get_parameter_or<float>("nms",confThreshold, 0.4);
    this->get_parameter_or<std::string>("device",device, "cpu");

    CV_Assert(!model.empty());
    
    // Load network.
    net_ = readNet(model);
        if (device == "cpu")
    {
        std::cout << "Using CPU device" << std::endl;
        net_.setPreferableBackend(DNN_TARGET_CPU);
    }
    else if (device == "gpu")
    {
        std::cout << "Using GPU device" << std::endl;
        net_.setPreferableBackend(DNN_BACKEND_CUDA);
        net_.setPreferableTarget(DNN_TARGET_CUDA);
    }

  }


private:

  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
  {
    std::vector<Mat> output;
    std::vector<String> outputLayers(2);
    outputLayers[0] = "feature_fusion/Conv_7/Sigmoid";
    outputLayers[1] = "feature_fusion/concat_3";
    cv_bridge::CvImagePtr cv_ptr;
    Mat frame, blob;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
      frame = cv_ptr->image;
    }catch(cv::Exception& e){
      const char* err_msg = e.what();
      std::cout << "encoding: " << msg->encoding << std::endl;
      return;
    }
    try{ 
      blobFromImage(frame, blob, 1, Size(inpWidth, inpHeight), Scalar(123.68, 116.78, 103.94), true, false);
    }catch(cv::Exception& e){
      const char* err_msg = e.what();
      std::cout << "exception caught: " << msg->encoding << std::endl;
      return;
    }   
   
    net_.setInput(blob);
    net_.forward(output, outputLayers);

    Mat scores = output[0];
    Mat geometry = output[1];

    // Decode predicted bounding boxes.
    std::vector<RotatedRect> boxes;
    std::vector<float> confidences;
    decode(scores, geometry, confThreshold, boxes, confidences);

    // Apply non-maximum suppression procedure.
    std::vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Render detections.
    Point2f ratio((float)frame.cols / inpWidth, (float)frame.rows / inpHeight);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        RotatedRect& box = boxes[indices[i]];

        Point2f vertices[4];
        box.points(vertices);
        for (int j = 0; j < 4; ++j)
        {
            vertices[j].x *= ratio.x;
            vertices[j].y *= ratio.y;
        }
        for (int j = 0; j < 4; ++j)
            line(frame, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2, LINE_AA);
    }

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = getTickFrequency() / 1000;
    double t = net_.getPerfProfile(layersTimes) / freq;
    std::string label = format("Inference time: %.2f ms", t);
    putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
    sensor_msgs::msg::CompressedImage cimg_msg = *cv_ptr->toCompressedImageMsg(cv_bridge::Format::JPEG).get();
    sensor_msgs::msg::Image img_msg;
    cv_ptr->toImageMsg(img_msg);
    bbox_image_publisher_->publish(img_msg);
    cbbox_image_publisher_->publish(cimg_msg);
    // sensor_msgs::msg::Image::SharedPtr msg =
    // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
    //     .toImageMsg();
    // bbox_image_publisher_->publish(img_msg);
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  void decode(const Mat& scores, const Mat& geometry, float scoreThresh,
            std::vector<RotatedRect>& detections, std::vector<float>& confidences){
    detections.clear();
    CV_Assert(scores.dims == 4); CV_Assert(geometry.dims == 4); CV_Assert(scores.size[0] == 1);
    CV_Assert(geometry.size[0] == 1); CV_Assert(scores.size[1] == 1); CV_Assert(geometry.size[1] == 5);
    CV_Assert(scores.size[2] == geometry.size[2]); CV_Assert(scores.size[3] == geometry.size[3]);

    const int height = scores.size[2];
    const int width = scores.size[3];
    for (int y = 0; y < height; ++y)
    {
        const float* scoresData = scores.ptr<float>(0, 0, y);
        const float* x0_data = geometry.ptr<float>(0, 0, y);
        const float* x1_data = geometry.ptr<float>(0, 1, y);
        const float* x2_data = geometry.ptr<float>(0, 2, y);
        const float* x3_data = geometry.ptr<float>(0, 3, y);
        const float* anglesData = geometry.ptr<float>(0, 4, y);
        for (int x = 0; x < width; ++x)
        {
            float score = scoresData[x];
            if (score < scoreThresh)
                continue;

            // Decode a prediction.
            // Multiple by 4 because feature maps are 4 time less than input image.
            float offsetX = x * 4.0f, offsetY = y * 4.0f;
            float angle = anglesData[x];
            float cosA = std::cos(angle);
            float sinA = std::sin(angle);
            float h = x0_data[x] + x2_data[x];
            float w = x1_data[x] + x3_data[x];

            Point2f offset(offsetX + cosA * x1_data[x] + sinA * x2_data[x],
                           offsetY - sinA * x1_data[x] + cosA * x2_data[x]);
            Point2f p1 = Point2f(-sinA * h, -cosA * h) + offset;
            Point2f p3 = Point2f(-cosA * w, sinA * w) + offset;
            RotatedRect r(0.5f * (p1 + p3), Size2f(w, h), -angle * 180.0f / (float)CV_PI);
            detections.push_back(r);
            confidences.push_back(score);
        }
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr bbox_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cbbox_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bbox_image_publisher_;
  float confThreshold;
  float nmsThreshold;
  int inpWidth;
  int inpHeight;
  String model;
  String device;
  Net net_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextDetection>());
  rclcpp::shutdown();
  return 0;
}
