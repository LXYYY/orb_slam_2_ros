#include "RGBDNode.h"

#include <voxgraph_msgs/LoopClosure.h>

class LoopClosureSendFunctor {
public:
  LoopClosureSendFunctor(const ros::NodeHandle &node_handle)
      : node_handle_(node_handle) {
    loop_closure_pub_ = node_handle_.advertise<voxgraph_msgs::LoopClosure>(
        "loop_closure_output", 1);
  }
  bool operator()(const double & from_timestamp, const double & to_timestamp, const cv::Mat &R,
                  const cv::Mat &t) {
                    voxgraph_msgs::LoopClosure loop_closure_msg;
                    loop_closure_msg.from_timestamp=ros::Time(from_timestamp);
                    loop_closure_msg.to_timestamp=ros::Time(to_timestamp);
                  }

private:
  ros::NodeHandle node_handle_;
  ros::Publisher loop_closure_pub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "RGBD");
  ros::start();

  if (argc > 1) {
    ROS_WARN("Arguments supplied via command line are neglected.");
  }

  ros::NodeHandle node_handle;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  image_transport::ImageTransport image_transport(node_handle);

  LoopClosureSendFunctor loop_closure_send_functor(node_handle);
  ORB_SLAM2::System::fLoopClosureSendFunc loop_closure_send_func =
      loop_closure_send_functor;
  RGBDNode node(ORB_SLAM2::System::RGBD, node_handle, image_transport,
                loop_closure_send_func);

  node.Init();

  ros::spin();

  ros::shutdown();

  return 0;
}

RGBDNode::RGBDNode(
    const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle,
    image_transport::ImageTransport &image_transport,
    ORB_SLAM2::System::fLoopClosureSendFunc loop_closure_send_func)
    : Node(sensor, node_handle, image_transport, loop_closure_send_func) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);
  camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
}

RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

  Update ();
}
