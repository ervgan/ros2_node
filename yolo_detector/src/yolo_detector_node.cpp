#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "yolo_interface/msg/detection.hpp"

class DetectorNode : public rclcpp::Node {
 public:
  DetectorNode() : Node("detector_node") {
    bounding_box_publisher_ =
        this->create_publisher<yolo_interface::msg::Detection>(
            "detection_topic", 10);
  }

  void detect_objects(/*const cv::Mat& image*/) {
    // Perform object detection using a yolo_detector class
    // yolo_detector_.detect(image);

    // Publish bounding box detections on a separate topic
    // auto bounding_box_msg = yolo_detector_node::msg::Detection();

    // for (auto detection : yolo_detector_.get_detections()) {
    // }
    auto detection = yolo_interface::msg::Detection();
    detection.x = 7;
    detection.y = 8;
    detection.width = 10;
    detection.height = 10;
    detection.confidence_score = 0.89;
    /*
    auto bbox = yolo_detector_node::msg::Detection;
    bbox.x = detection.x;
    bbox.y = detection.y;
    bbox.width = detection.width;
    bbox.height = detection.height;
    bbox.class_id = detection.class_id;
    bounding_box_msg.bounding_boxes.push_back(bbox);*/

    bounding_box_publisher_->publish(detection);
  }

 private:
  rclcpp::Publisher<yolo_interface::msg::Detection>::SharedPtr
      bounding_box_publisher_;
  // yolo_detector_node::YoloDetector yolo_detector_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance of the DetectorNode class
  auto detector_node = std::make_shared<DetectorNode>();

  // Load an image to detect objects in
  // cv::Mat image = cv::imread("path/to/image.jpg");

  // Detect objects in the image and publish the bounding box detections
  detector_node->detect_objects(/*image*/);
  rclcpp::spin(detector_node);
  rclcpp::shutdown();
  return 0;
}
