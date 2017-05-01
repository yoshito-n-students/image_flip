#ifndef IMAGE_FLIP_IMAGE_FLIP
#define IMAGE_FLIP_IMAGE_FLIP

#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>

namespace image_flip {

class ImageFlip : public nodelet::Nodelet {
public:
  ImageFlip() {}

  virtual ~ImageFlip() {}

private:
  virtual void onInit() {
    // get node handles
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load parameters
    horizontal_ = pnh.param("horizontal", true);
    vertical_ = pnh.param("vertical", false);
    if (!horizontal_ && !vertical_) {
      NODELET_WARN("Both ~horizontal and ~vertical are disabled. wiil just relay images.");
    }

    // start passing images
    image_transport::ImageTransport it(nh);
    publisher_ = it.advertise("image_flipped", 1);
    {
      // resolve the root topic name here so that the root topic remapping work as expected
      // even if the transport type is not "raw"
      const std::string topic(nh.resolveName("image_raw"));
      // use the right parameter namespace so that "~image_transport" parameter can make an effect
      const image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
      subscriber_ = it.subscribe(topic, 1, &ImageFlip::onImageReceived, this, hints);
    }
  }

  void onImageReceived(const sensor_msgs::ImageConstPtr &raw) {
    // ROS image message -> opencv image
    cv_bridge::CvImagePtr flipped(cv_bridge::toCvCopy(raw));
    if (!flipped) {
      NODELET_ERROR("Failed to extract a source image");
      return;
    }

    // flip the image
    if (horizontal_ && vertical_) {
      cv::flip(flipped->image, flipped->image, -1);
    } else if (horizontal_) {
      cv::flip(flipped->image, flipped->image, 1);
    } else if (vertical_) {
      cv::flip(flipped->image, flipped->image, 0);
    }

    // publish the flipped image
    publisher_.publish(flipped->toImageMsg());
  }

private:
  bool horizontal_;
  bool vertical_;
  image_transport::Subscriber subscriber_;
  image_transport::Publisher publisher_;
};
}

#endif