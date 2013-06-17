#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>

#include "OpticalFlowNode.h"
#include "OptFlowResult.h"
#include "Frame.h"

OpticalFlowNode::OpticalFlowNode() :
    nodeHandle("~")
{
  nodeHandle.param("showCapture", showCapture, true);
  nodeHandle.param("windowName", windowName, std::string("Optical Flow"));
  previousFrame = NULL;
  resultPane = NULL;

  // Advertise the calculated optical flow
  std::string publishTopic;
  nodeHandle.param("publishTopic", publishTopic, std::string("opticalFlow"));
  publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>(publishTopic, 10);
}

OpticalFlowNode::~OpticalFlowNode()
{
  cv::destroyWindow(windowName);
}

void OpticalFlowNode::processNextFrameCb(const sensor_msgs::Image::ConstPtr& srcImg)
{
  IplImage img = convertImage(srcImg);
  processNextFrame(&img);
  ros::spinOnce();
}

void OpticalFlowNode::processNextFrame(IplImage* img)
{
  initialize(img);
  if (showCapture)
    cvConvertImage(img, resultPane, 0);

  // Calculate the optical flow (except during first call)
  OptFlowResult ofr = previousFrame->calculateOpticalFlow(img, resultPane);

  // Publish the result
  publisher.publish(ofr.toRosMessage());
  ROS_INFO("Published: [%d, %d]", (int )ofr.toRosMessage().data[0], (int )ofr.toRosMessage().data[1]);

  // Update the window with the current image and the calculated optical flow
  if (showCapture)
  {
    cvShowImage("Optical Flow", resultPane);
    cvWaitKey(1);
  }

  // We're done with calculating optical flow, therefore save this frame for the next round
  previousFrame->useLastComparison();
}

IplImage OpticalFlowNode::convertImage(const sensor_msgs::Image::ConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  IplImage tmp = cv_ptr.get()->image;
  return tmp;
}

void OpticalFlowNode::initialize(IplImage* img)
{
  // Abort if the initialization has already been done once before
  if (previousFrame != NULL)
    return;

  // Create the frame for the last analyzed frame
  CvSize frameSize;
  frameSize.height = img->height;
  frameSize.width = img->width;
  previousFrame = new Frame(Frame(&frameSize, img));

  // Use the new image as new pane to draw on
  if (showCapture)
  {
    resultPane = cvCreateImage(frameSize, IPL_DEPTH_8U, 3);
    cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Optical Flow", 50, 50);
  }
}


// Starts this OpticalFlow Nodelet
int opticalFlowNodelet(int argc, char **argv)
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

  // Invoke spinning, so that the subscription-processing can begin
  ros::spin();
  return 0;
}
