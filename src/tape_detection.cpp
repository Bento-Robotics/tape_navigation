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
  cv::Scalar lower_blue;
  cv::Scalar upper_blue;

  State state = none;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
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

      cv::inRange(hsvimg, lower_blue, upper_blue, blue_mask);
      cv::bitwise_and(img, img, out, blue_mask);

      cv::rectangle(out, cv::Point(0, 0), cv::Point(30, 30), ScalarHSV2RGB(upper_blue), cv::FILLED);
      cv::rectangle(out, cv::Point(30, 0), cv::Point(60, 30), ScalarHSV2RGB(lower_blue), cv::FILLED);

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
    this->declare_parameter<std::vector<uint8_t>>("lower_blue", {90, 50, 100});

    this->lower_blue = cv::Scalar(90, 50, 100);
    this->upper_blue = cv::Scalar(153, 255, 255);
  }

  void InitializeImageTransport()
  {
    // Create image transport
    it_ = std::make_shared<image_transport::ImageTransport>(
        this->shared_from_this()); // https://answers.ros.org/question/353828/getting-a-nodesharedptr-from-this/

    // image_transport::TransportHints hints(this.get());
    image_sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe("camera/image_raw", 1, std::bind(&TapeDetection::image_callback, this, std::placeholders::_1)));

    cv::namedWindow("view");
    cv::startWindowThread();

    auto onHueChange = [](int, void *) {
      
    };
    cv::createTrackbar("Hue", "view", 0, 255, onHueChange);
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
