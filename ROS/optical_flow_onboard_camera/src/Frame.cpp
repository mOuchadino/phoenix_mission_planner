#include "Frame.h"
#include "OptFlowResult.h"
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

static const int lineThickness = 1;
static const double pi = 3.14159265358979323846;
static const CvScalar lineColor = CV_RGB(0, 170, 255);

Frame::Frame(CvSize* frameSize, IplImage* img)
{
  featureCount = maxFeatureCount;

  image = cvCreateImage(*frameSize, IPL_DEPTH_8U, 1);
  lastCompared = cvCreateImage(*frameSize, IPL_DEPTH_8U, 1);
  pyramid = cvCreateImage(*frameSize, IPL_DEPTH_8U, 1);
  pyramid_other = cvCreateImage(*frameSize, IPL_DEPTH_8U, 1);

  eig_image = cvCreateImage(*frameSize, IPL_DEPTH_32F, 1);
  temp_image = cvCreateImage(*frameSize, IPL_DEPTH_32F, 1);

  terminationCriteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);
  window = cvSize(3, 3);

  // Use the given image as base image
  cvConvertImage(img, lastCompared, 0);
  useLastComparison();
}

void Frame::useLastComparison()
{
  // Convert whatever the source image format is into OpenCV's preferred format.
  IplImage* tmp = image;
  image = lastCompared;
  lastCompared = tmp;

  // Detect good features (noticeable pixels) in this image
  detectFeatures();
}

OptFlowResult Frame::calculateOpticalFlow(IplImage* nextImage, IplImage* resultPane = NULL)
{
  cvConvertImage(nextImage, lastCompared, 0);

  /* Pyramidal Lucas Kanade Optical Flow! */

  CvPoint2D32f newPosition[featureCount];
  char detected[featureCount];
  float error[featureCount];

  /* Actually run Pyramidal Lucas Kanade Optical Flow
   * pyramids are workspace for the algorithm
   * "features" are the features from the first frame
   * "detectedFeatures" is the (outputted) locations of those features in the second frame.
   * "featureCount" is the number of features in the features-array.
   * "window" is the size of the window to use to avoid the aperture problem.
   * "5" is the maximum number of pyramids to use.  0 would be just one level.
   * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
   * "optical_flow_feature_error" is as described above (error in the flow for this feature).
   * "termination_criteria" is as described above (how long the algorithm should look).
   * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
   */
  cvCalcOpticalFlowPyrLK(image, lastCompared, pyramid, pyramid_other, features, newPosition, featureCount, window, 5,
                         detected, error, terminationCriteria, 0);

  OptFlowResult ofr(featureCount, features, newPosition, detected, error);

  // Only draw on the result image, if one was given
  if (resultPane != NULL)
  {
    ofr.draw(resultPane);
//    std::vector<CvPoint>::iterator itrOffset = ofr.offset.begin();
//    for (std::vector<CvPoint>::iterator itr = ofr.newPos.begin(); itr != ofr.newPos.end(); ++itr)
//    {
//      drawOpticalFlowResults(&*itr, &*itrOffset, resultPane);
//      ++itrOffset;
//    }
  }

  ROS_INFO("Calculated avg. movement: X = %d, Y = %d using %d (%d) features", (int )ofr.avgOffset.x,
           (int )ofr.avgOffset.y, (int )ofr.newPos.size(), (int )(ofr.newPos.size() * 100 / featureCount));

  return ofr;
}

void Frame::drawOpticalFlowResults(CvPoint* currentLocation, CvPoint* offset, IplImage* resultPane)
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

void Frame::detectFeatures()
{
  /* The first ".01" specifies the minimum quality of the features (based on the eigenvalues).
   * The second ".01" specifies the minimum Euclidean distance between features.
   * "NULL" means use the entire input image.  You could point to a part of the image.
   */
  featureCount = maxFeatureCount;
  cvGoodFeaturesToTrack(image, eig_image, temp_image, features, &featureCount, .01, .01, NULL);
}

void Frame::updateOffset(double *offset, double featureCounter, int prevVal, int currVal)
{
  double factor = (featureCounter - 1) / featureCounter;
  *offset = (factor * *offset) + (1 - factor) * (prevVal - currVal);
}
