/*
 * OpticalFlowNode.h
 *
 *  Created on: 26.05.2013
 *      Author: ic3
 */

#ifndef OPTICALFLOWNODE_H_
#define OPTICALFLOWNODE_H_

#define SHOW_CAPTURE

#include "Frame.h"
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>

class OpticalFlowNode
{
public:

  ros::NodeHandle nodeHandle;
  Frame* previousFrame;

  OpticalFlowNode();
  ~OpticalFlowNode();

  // Captures the next frame and calculates the optical flow, based on the previous frame.
  void processNextFrameCb(const sensor_msgs::Image::ConstPtr& img);
  void processNextFrame(IplImage* img);

private:
  // This image will be shown to the user
  IplImage* resultPane;
  std::string windowName;
  bool showCapture;

  ros::Publisher publisher;

  // Convert from sensor_msgs::Image to IplImage
  IplImage convertImage(const sensor_msgs::Image::ConstPtr& img);

  // Initializes the members, e.g. capture-window, if not already done
  void initialize(IplImage* img);
};

#endif /* OPTICALFLOWNODE_H_ */
