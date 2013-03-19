#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;

Point coordinate_system;
Point adjusted_coordinate_system;
Point target_point;

int cameraDistance=4943; //between lens & tracking plane in mm
int cameraConstant=670; //camera constant for facetime camera


Rect selection;
int vmin = 30, vmax = 256, smin = 60;

void onMouse( int event, int x, int y, int flags, void* param )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
        
        selection &= Rect(0, 0, image.cols, image.rows);
    }
    
    switch( event )
    {
        case CV_EVENT_LBUTTONDOWN:
            origin = Point(x,y);
            selection = Rect(x,y,0,0);
            selectObject = true;
            break;
        case CV_EVENT_LBUTTONUP:
            selectObject = false;
            if( selection.width > 0 && selection.height > 0 )
                trackObject = -1;
            break;
        case CV_EVENT_RBUTTONUP:
            if (coordinate_system.x==0) {
                coordinate_system.x=x;
                coordinate_system.y=y;
            }
            target_point.x=x;
            target_point.y=y;
            break;
        case CV_EVENT_LBUTTONDBLCLK:
            
            break;
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "phoenix_positioning");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("phoenix_positioning",1);
    geometry_msgs::Point position;
    
    VideoCapture cap;
    Rect trackWindow;
    RotatedRect trackBox;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        cap.open(argc == 2 ? argv[1][0] - '0' : 0);
    else if( argc == 2 )
        cap.open(argv[1]);
    
    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }
    
    cout << "Hot keys: \n"
    "\tESC - quit the program\n"
    "\tc - stop the tracking\n"
    "\tb - switch to/from backprojection view\n"
    "\th - show/hide object histogram\n"
    "To initialize tracking, select the object with mouse\n";
    
    namedWindow( "Histogram", 1 );
    namedWindow( "CamShift Demo", 1 );
    setMouseCallback( "CamShift Demo", onMouse, 0 );
    createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
    createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
    createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );
    
    Mat hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    
    for(;;)
    {
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;
        
        frame.copyTo(image);
        cvtColor(image, hsv, CV_BGR2HSV);
        
        if( trackObject )
        {
            int _vmin = vmin, _vmax = vmax;
            
            inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                    Scalar(180, 256, MAX(_vmin, _vmax)), mask);
            int ch[] = {0, 0};
            hue.create(hsv.size(), hsv.depth());
            mixChannels(&hsv, 1, &hue, 1, ch, 1);
            
            if( trackObject < 0 )
            {
                Mat roi(hue, selection), maskroi(mask, selection);
                calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                normalize(hist, hist, 0, 255, CV_MINMAX);
                
                trackWindow = selection;
                trackObject = 1;
                
                histimg = Scalar::all(0);
                int binW = histimg.cols / hsize;
                Mat buf(1, hsize, CV_8UC3);
                for( int i = 0; i < hsize; i++ )
                    buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                cvtColor(buf, buf, CV_HSV2BGR);
                
                for( int i = 0; i < hsize; i++ )
                {
                    int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                    rectangle( histimg, Point(i*binW,histimg.rows),
                              Point((i+1)*binW,histimg.rows - val),
                              Scalar(buf.at<Vec3b>(i)), -1, 8 );
                }
            }
            
            calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;
            RotatedRect trackBox = CamShift(backproj, trackWindow,
                                            TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
            trackBox.angle = 90-trackBox.angle;
            
            if( backprojMode )
                cvtColor( backproj, image, CV_GRAY2BGR );
            circle(image, trackBox.center, 2, Scalar(0,0,255));
            ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
            rectangle(image, trackBox.boundingRect(), Scalar(0,0,255));
            
            circle(image, coordinate_system, 5, Scalar(255,0,0)); //circle coo system
            adjusted_coordinate_system.x= trackBox.center.x - coordinate_system.x;
            adjusted_coordinate_system.y= coordinate_system.y - trackBox.center.y;
            pub.publish(position);

            
            position.x=adjusted_coordinate_system.x*cameraDistance/cameraConstant;
            position.y=adjusted_coordinate_system.y*cameraDistance/cameraConstant;
            position.z=0;
            
            cout <<"X: "<<position.x<<" Y: "<<position.y<<" Size: "<<trackBox.size<<endl;
            

            
        }
        
        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }
        
        imshow( "CamShift Demo", image );
        imshow( "Histogram", histimg );
        
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
        {
            case 'b':
                backprojMode = !backprojMode;
                break;
            case 'c':
                trackObject = 0;
                histimg = Scalar::all(0);
                break;
            case 'h':
                showHist = !showHist;
                if( !showHist )
                    destroyWindow( "Histogram" );
                else
                    namedWindow( "Histogram", 1 );
                break;
            default:
                ;
        }
    }
    
    return 0;
}
