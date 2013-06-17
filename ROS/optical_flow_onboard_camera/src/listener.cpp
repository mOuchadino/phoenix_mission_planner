#include <ros/ros.h>
#include "OpticalFlowNode.h"
#include <image_transport/image_transport.h>
#include <std_msgs/Float64MultiArray.h>

void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ROS_INFO("Received: [%d, %d]", (int )msg->data[0], (int )msg->data[1]);
  ros::spinOnce();
}

int main(int argc, char **argv)
{
  // Initialize ros node, providing the given arguments.
  ros::init(argc, argv, "opticalFlowCalculator");
  ros::NodeHandle nodeHandle;
  OpticalFlowNode ofn;

  // Subscribe to the image stream
  std::string imageTopic;
  nodeHandle.param("imageTopic", imageTopic, std::string("/usb_cam/image_raw"));

  image_transport::ImageTransport it(nodeHandle);
  image_transport::Subscriber sub = it.subscribe(imageTopic, 1, &OpticalFlowNode::processNextFrameCb, &ofn);

  // Sample listener
  std::string publishTopic;
  nodeHandle.param("publishTopic", publishTopic, std::string("opticalFlowCalculator/opticalFlow"));
  ros::Subscriber subSample = nodeHandle.subscribe(publishTopic, 1, chatterCallback);

  // Invoke spinning, so that the subscription-processing can begin
  ros::spin();
  return 0;
}
