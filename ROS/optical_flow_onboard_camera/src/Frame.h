/*
 * Frame.h
 *
 *  Created on: 26.05.2013
 *      Author: ic3
 */

#ifndef FRAME_H_
#define FRAME_H_

#define maxFeatureCount 400

#include <opencv/cv.h>
#include "OptFlowResult.h"
#include <sensor_msgs/Image.h>

class Frame
{
public:
  // Variables required for calculating optical flow
  IplImage *eig_image, *temp_image;

  // The actual image
  IplImage* image;
  IplImage* lastCompared;

  /* This is some workspace for the algorithm.
   * (The algorithm actually carves the image into pyramids of different resolutions.)
   */
  IplImage* pyramid;
  IplImage* pyramid_other;

  // Features (noticeable points) in this frame
  CvPoint2D32f features[maxFeatureCount];

  // Number of features found in the image
  int featureCount;

  Frame(CvSize* frameSize, IplImage* img);

  // Calculates the optical flow. Draws the result on the pane (if not null)
  OptFlowResult calculateOpticalFlow(IplImage* newFrame, IplImage* resultPane);

  // Uses the last image as the new previous image
  void useLastComparison();

private:

  // Termination-Criteria: 20 iterations or epsilon is better than .3.
  CvTermCriteria terminationCriteria;

  // This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview").
  CvSize window;

  // Detects good features (noticeable pixels) of this frame's image.
  void detectFeatures();
};

#endif /* FRAME_H_ */
