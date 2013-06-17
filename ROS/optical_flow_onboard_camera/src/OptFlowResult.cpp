/*
 * OptFlowResult.cpp
 *
 *  Created on: 31.05.2013
 *      Author: ic3
 */
#include "OptFlowResult.h"
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>

static const int lineThickness = 1;
static const double pi = 3.14159265358979323846;
static const CvScalar colorBlack = CV_RGB(0, 0, 0);
static const CvScalar lineColor = CV_RGB(0, 170, 255);

OptFlowResult::OptFlowResult(int featureCount, CvPoint2D32f* prevPosArray, CvPoint2D32f* newPosArray, char* detected,
                             float* error)
{
  avgOffset.x = 0;
  avgOffset.y = 0;
  for (int i = 0; i < featureCount; i++)
  {
    if (detected[i] == 0)
    {
      continue;
    }

    CvPoint prevPos, newPos, offset;
    prevPos.x = prevPosArray[i].x;
    prevPos.y = prevPosArray[i].y;
    newPos.x = newPosArray[i].x;
    newPos.y = newPosArray[i].y;
    offset.x = newPos.x - prevPos.x;
    offset.y = newPos.y - prevPos.y;

    avgOffset.x += offset.x;
    avgOffset.y += offset.y;

    this->newPos.push_back(newPos);
    this->prevPos.push_back(prevPos);
    this->offset.push_back(offset);
    this->error.push_back(error[i]);
  }

  avgOffset.x /= newPos.size();
  avgOffset.y /= newPos.size();
}

std_msgs::Float64MultiArray OptFlowResult::toRosMessage()
{
  std_msgs::Float64MultiArray msg;
  msg.data.clear();
  msg.data.push_back(avgOffset.x);
  msg.data.push_back(avgOffset.y);
  return msg;
}

void OptFlowResult::draw(IplImage* resultPane)
{
  // Draw each detected offset
  std::vector<CvPoint>::iterator itrOffset = offset.begin();
  for (std::vector<CvPoint>::iterator itr = newPos.begin(); itr != newPos.end(); ++itr)
  {
    draw(&*itr, &*itrOffset, resultPane);
    ++itrOffset;
  }

  // Draw the average result
  cvRectangle(resultPane, cvPoint(0, 0), cvPoint(200, 20), colorBlack, CV_FILLED);
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, .6, .6, 0, 1);
  std::stringstream txt;
  txt << "X: " << avgOffset.x << " Y: " << avgOffset.y;
//  cvPutText(resultPane, txt.str(), cvPoint(2, 18), &font, lineColor);
}

void OptFlowResult::draw(CvPoint* currentLocation, CvPoint* offset, IplImage* resultPane)
{
  CvPoint previousLocation;
  previousLocation.x = currentLocation->x + offset->x;
  previousLocation.y = currentLocation->y + offset->y;
  double angle = atan2((double)previousLocation.y - currentLocation->y,
                       (double)previousLocation.x - currentLocation->x);

  /* Now we draw the main line of the arrow
   * "CV_AA" means antialiased drawing.
   * "0" means no fractional bits in the center cooridinate or radius.
   */
  cvLine(resultPane, previousLocation, *currentLocation, lineColor, lineThickness, CV_AA, 0);
  /* Now draw the tips of the arrow.  I do some scaling so that the
   * tips look proportional to the main line of the arrow.
   */
  previousLocation.x = (int)(currentLocation->x + 6 * cos(angle + pi / 4));
  previousLocation.y = (int)(currentLocation->y + 6 * sin(angle + pi / 4));
  cvLine(resultPane, *currentLocation, previousLocation, lineColor, lineThickness, CV_AA, 0);

  previousLocation.x = (int)(currentLocation->x + 6 * cos(angle - pi / 4));
  previousLocation.y = (int)(currentLocation->y + 6 * sin(angle - pi / 4));
  cvLine(resultPane, *currentLocation, previousLocation, lineColor, lineThickness, CV_AA, 0);
}
