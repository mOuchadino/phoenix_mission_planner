/*
 * OptFlowResult.h
 *
 *  Created on: 31.05.2013
 *      Author: ic3
 */

#ifndef OPTFLOWRESULT_H_
#define OPTFLOWRESULT_H_

#include <opencv/cv.h>
#include <std_msgs/Float64MultiArray.h>

class OptFlowResult
{
public:
  // Location of feature in the previous frame
  std::vector<CvPoint> prevPos;

  // Location of feature in given next frame
  std::vector<CvPoint> newPos;

  // Offset from previous location
  std::vector<CvPoint> offset;

  // Average offset from previous location
  struct avgOffset
  {
    double x;
    double y;
  } avgOffset;

  // Error of the search for a found feature
  std::vector<float> error;

  OptFlowResult(int featureCount, CvPoint2D32f* prevPos, CvPoint2D32f* newPos, char* detected, float* error);

  // Converts the result to a ROS message
  std_msgs::Float64MultiArray toRosMessage();

  // Draws the result (e.g. arrows from old to new position) on the given resultPane
  void draw(IplImage* resultPane);

private:

  void draw(CvPoint* currentLocation, CvPoint* offset, IplImage* resultPane);
};

#endif /* OPTFLOWRESULT_H_ */
