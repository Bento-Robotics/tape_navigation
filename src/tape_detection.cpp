#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/highgui.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

enum State
{
  lowerHSV,
  upperHSV,
  none
};

class TapeDetection : public rclcpp::Node
{

private:
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Subscriber> image_sub_;
  cv::Scalar min_hsv;
  cv::Scalar max_hsv;

  State state = none;

private:
  // https://stackoverflow.com/questions/8636689/opencv-trackbar-callback-in-c-class
  static void hue_max_slider_callback(int slider, void *object)
  {
    auto td = static_cast<TapeDetection *>(object);
    td->max_hsv[0] = slider;
  }
  static void saturation_max_slider_callback(int slider, void *object)
  {
    auto td = static_cast<TapeDetection *>(object);
    td->max_hsv[1] = slider;
  }
  static void value_max_slider_callback(int slider, void *object)
  {
    auto td = static_cast<TapeDetection *>(object);
    td->max_hsv[2] = slider;
  }
  static void hue_min_slider_callback(int slider, void *object)
  {
    auto td = static_cast<TapeDetection *>(object);
    td->min_hsv[0] = slider;
  }
  static void saturation_min_slider_callback(int slider, void *object)
  {
    auto td = static_cast<TapeDetection *>(object);
    td->min_hsv[1] = slider;
  }
  static void value_min_slider_callback(int slider, void *object)
  {
    auto td = static_cast<TapeDetection *>(object);
    td->min_hsv[2] = slider;
  }

  void update_hsv()
  {
    cv::setTrackbarPos("Hue_min", "view", min_hsv[0]);
    cv::setTrackbarPos("Hue_max", "view", max_hsv[0]);
    cv::setTrackbarPos("Saturation_min", "view", min_hsv[1]);
    cv::setTrackbarPos("Saturation_max", "view", max_hsv[1]);
    cv::setTrackbarPos("Value_min", "view", min_hsv[2]);
    cv::setTrackbarPos("Value_max", "view", max_hsv[2]);

    // this->set_parameter(rclcpp::Parameter("test", {min_hsv[0], min_hsv[1], min_hsv[2]}))
    // this->declare_parameter<std::vector<int>>("test", {min_hsv[0], min_hsv[1], min_hsv[2]});
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    update_hsv();
    try
    {
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat sobelx, sobely, gradient;

      // // Apply Sobel operator
      // cv::Sobel(img, sobelx, CV_64F, 1, 0, 3);
      // cv::Sobel(img, sobely, CV_64F, 0, 1, 3);

      // // Compute gradient magnitude
      // cv::magnitude(sobelx, sobely, gradient);

      // // Convert to 8-bit image
      // cv::Mat gradient_abs;
      // cv::convertScaleAbs(gradient, gradient_abs);

      cv::Mat out, hsvimg, blue_mask;
      cv::cvtColor(img, hsvimg, cv::COLOR_BGR2HSV);
      // https://pseudopencv.site/utilities/hsvcolormask/
      // cv::Scalar lower_blue(90, 35, 140);

      cv::inRange(hsvimg, min_hsv, max_hsv, blue_mask);
      cv::bitwise_and(img, img, out, blue_mask);

      cv::rectangle(out, cv::Point(0, 0), cv::Point(30, 30), ScalarHSV2RGB(max_hsv), cv::FILLED);
      cv::rectangle(out, cv::Point(30, 0), cv::Point(60, 30), ScalarHSV2RGB(min_hsv), cv::FILLED);

      // Display result
      cv::imshow("view", out);
      cv::waitKey(10);
    }
    catch (const cv_bridge::Exception &e)
    {
      auto logger = rclcpp::get_logger("subscriber_detection.cpp");
      RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

public:
  TapeDetection() : Node("tape_detection")
  {
    // TransportHints does not actually declare the parameter
    this->declare_parameter<std::string>("image_transport", "raw");
    this->declare_parameter<std::vector<int>>("min_hsv", {90, 50, 100});
    this->declare_parameter<std::vector<int>>("max_hsv", {153, 255, 255});
    this->declare_parameter<std::vector<int>>("test", {0, 0, 0});

    auto min_array = this->get_parameter("min_hsv").as_integer_array();
    min_hsv = cv::Scalar(min_array[0], min_array[1], min_array[2]);
    auto max_array = this->get_parameter("max_hsv").as_integer_array();
    max_hsv = cv::Scalar(max_array[0], max_array[1], max_array[2]);
  }

  void InitializeImageTransport()
  {
    // Create image transport
    it_ = std::make_shared<image_transport::ImageTransport>(
        this->shared_from_this()); // https://answers.ros.org/question/353828/getting-a-nodesharedptr-from-this/

    // image_transport::TransportHints hints(this.get());
    image_sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe("camera/image_raw", 1, std::bind(&TapeDetection::image_callback, this, std::placeholders::_1)));

    cv::namedWindow("view", cv::WINDOW_NORMAL);
    cv::startWindowThread();

    cv::createTrackbar("Hue_min", "view", 0, 180, &TapeDetection::hue_min_slider_callback, this);
    cv::createTrackbar("Saturation_min", "view", 0, 255, &TapeDetection::saturation_min_slider_callback, this);
    cv::createTrackbar("Value_min", "view", 0, 255, &TapeDetection::value_min_slider_callback, this);
    cv::createTrackbar("Hue_max", "view", 0, 180, &TapeDetection::hue_max_slider_callback, this);
    cv::createTrackbar("Saturation_max", "view", 0, 255, &TapeDetection::saturation_max_slider_callback, this);
    cv::createTrackbar("Value_max", "view", 0, 255, &TapeDetection::value_max_slider_callback, this);
  }

  ~TapeDetection()
  {
    image_sub_->shutdown();
    cv::destroyWindow("view");
  }

  static cv::Scalar ScalarHSV2RGB(cv::Scalar hsvscalar)
  {
    cv::Mat rgb;
    cv::Mat hsv(1, 1, CV_8UC3, hsvscalar);
    cv::cvtColor(hsv, rgb, CV_HSV2BGR);
    return cv::Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TapeDetection>();

  node->InitializeImageTransport();
  rclcpp::spin(node);

  return 0;
}
