/**
 * 
 * @author Wahib-Ul-Haq, wahib.tech@gmail.com, 2014
 * 
 *
 * contains code taken from:
 * 
 * "Lane detection using OpenCV" 
 * https://github.com/jdorweiler/lane-detection
 * Acknowledgments: jdorweiler
 * Contact: jason@transistor.io
 */

#include <jni.h>
#include "lane_detector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/gpumat.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "linefinder.h"
#include "lanemetrics.h"

#include <android/log.h>

#define PI 3.1415926

using namespace std;
using namespace cv;
using namespace gpu;

JNIEXPORT jintArray JNICALL Java_tum_andrive_lanedetection_LaneDetector_mainDelegate
        (JNIEnv *env, jobject obj, jlong in, jlong out, jint houghValue) {

    //tried using VideoCapture but didn't work and getting no access to camera. reason is ndk doesn't allow to access camera in this native way like on desktop.
    //somehow it is also not recommended

    //__android_log_print(ANDROID_LOG_ERROR, "LANEDETECTOR++", "%s", "start mainDelegate");

    Mat &imageOrig = *(Mat *) in;
    Mat &output = *(Mat *) out;

    Mat image(imageOrig);
    gpu::GpuMat gpuImageOrig = gpu::GpuMat(imageOrig);

    //////reducing the image size////
//    Mat image;
    gpu::GpuMat gpuImage;
    resize(gpuImageOrig, gpuImage, Size(), 0.25, 0.25,
           cv::INTER_LINEAR); //to reduce it to 1/4 of size

    int houghVote = houghValue;
    __android_log_print(ANDROID_LOG_ERROR, "LANEDETECTOR++", "hough value : %d", houghVote);

    //double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    //double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    if (gpuImage.empty()) {
        //break;
        __android_log_print(ANDROID_LOG_ERROR, "LANEDETECTOR++", "%s", "Cannot access the camera");

        exit(0);
    }

//    Mat gray;
    GpuMat gray;
    cvtColor(gpuImage, gray, CV_RGB2GRAY); //convert image to grayscale
    vector<string> codes;
//    Mat corners;
    GpuMat corners;
    findDataMatrix(gray, codes, corners);
    drawDataMatrixCodes(gpuImage, codes, corners);

    /*ROI returns a matrix that is pointing to the ROI of the original image, located at the place specified by the rectangle.
    so imageROI is really the Region of Interest (or subimage/submatrix) of the original image "image".
    If you modify imageROI it will consequently modify the original, larger matrix.
    Rect region_of_interest = Rect(x, y, w, h); current parameters try to take the lower half of the image on vertical frame*/

    Rect roi(0, gpuImage.cols / 3, gpuImage.cols - 1,
             gpuImage.rows - gpuImage.cols / 3);// set the ROI (region of interest) for the image
    //Rect roi(0,image.cols/2,image.cols-1,image.rows - image.cols/2);// set the ROI (region of interest) for the image

//    Mat imgROI = image(roi);
    GpuMat imgROI = gpuImage(roi);

    /* Canny Edge Detector algorithm : canny (source image, edge output image, first threshold for the hysteresis procedure,
       , second threshold for the hysteresis procedure, apersturesize i.e set to be 3 by default , L2gradient )
       threshold2 is recommended to be 3 times to threshold1 */

//    Mat contours;
    GpuMat contours;
    Canny(imgROI, contours, 80, 250, 3); //50, 150,3); //in original code => 50,250);

    /* Thresholding is to differentiate the pixels we are interested in from the rest */
//    Mat contoursInv;
    GpuMat contoursInv;
    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);

    // Hough tranform for line detection with feedback
    // Increase by 25 for the next frame if we found some lines.
    // This is so we don't miss other lines that may crop up in the next frame
    // but at the same time we don't want to start the feed back loop from scratch.

    vector<Vec2f> lines;
    if (houghVote < 1 or lines.size() > 2) { // we lost all lines. reset
        houghVote = houghValue; //previously it was set to 200 always
    }
    else { houghVote += 25; }

    //ensuring lines size is 5
    while (lines.size() < 5 && houghVote > 0) {
        HoughLines(contours, lines, 1, PI / 180, houghVote);
        houghVote -= 5;
    }

    //cout << houghVote << "\n";
//    Mat result(imgROI.size(), CV_8U, Scalar(255));
    GpuMat result(imgROI.size(), CV_8U, Scalar(255));
    imgROI.copyTo(result);

    // Draw the lines
    vector<Vec2f>::const_iterator it = lines.begin();
//    Mat hough(imgROI.size(), CV_8U, Scalar(0));
    GpuMat hough(imgROI.size(), CV_8U, Scalar(0));


//    while (it != lines.end()) {
//
//        //__android_log_print(ANDROID_LOG_INFO, "LANEDETECTOR++", "%s", "while of lines");
//
//        float rho = (*it)[0];   // first element is distance rho
//        float theta = (*it)[1]; // second element is angle theta
//
//        // filter to remove vertical and horizontal lines
//        //if(theta is between 5 degrees and 84 degrees) or if(theta is between 95 degrees and 180 degrees )
//        if ((theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66)) {
//            // point of intersection of the line with first row
//            Point pt1(rho / cos(theta), 0);//y = 0
//
//            // point of intersection of the line with last row
//            Point pt2((rho - result.rows * sin(theta)) / cos(theta), result.rows);//y = row count
//
//                    //This draws lines but without ends and in shape of X
//            line(hough, pt1, pt2, Scalar(255),
//                 5); //this is working and shows red lines of thickness 5
//        }
//        ++it;
//    }

    // Create LineFinder instance
    LineFinder ld;

    // Set probabilistic Hough parameters
    ld.setLineLengthAndGap(60, 10);
    ld.setMinVote(4);

    // Detect lines
    vector<Vec4i> li = ld.findLines(contours); //applying probablity hough transform

//    Mat houghP(imgROI.size(), CV_8U, Scalar(0));
    GpuMat houghP(imgROI.size(), CV_8U, Scalar(0));
    ld.setShift(0);
//    ld.drawDetectedLines(houghP);

    // bitwise AND of the two hough images.
    //1) Normal Hough -> without end points 2) Probabilistic Hough -> with end points

    bitwise_and(houghP, hough, houghP);
//    Mat houghPinv(imgROI.size(), CV_8U, Scalar(0));
//    Mat dst(imgROI.size(), CV_8U, Scalar(0));
    GpuMat houghPinv(imgROI.size(), CV_8U, Scalar(0));
    GpuMat dst(imgROI.size(), CV_8U, Scalar(0));
    threshold(houghP, houghPinv, 150, 255,
              THRESH_BINARY_INV); // threshold and invert to black lines

    Canny(houghPinv, contours, 100, 350);//100,150); //100,350);
    li = ld.findLines(contours);

    // Set probabilistic Hough parameters
    //These parameters set min and max values. If you set minvote as 100 then even if you set 60 from slider, it won't
    //show any lines

    ld.setLineLengthAndGap(5, 2); //5,2 original
    ld.setMinVote(1); //1 original

    ld.setShift(gpuImage.cols / 3);
    ld.drawDetectedLines(image);

    //to show the number of line segments found in a frame
    //putText(image, stream.str(), Point(10,image.rows-10), 4, 1, Scalar(0,255,255),0);

    lines.clear();

    //Hough Processing ends here

//    resize(image, image, Size(), 4, 4, cv::INTER_LINEAR);
    output = image;

    // Initialize output jintArray
    jintArray res = env->NewIntArray(4 * li.size());
    if(res == NULL) {
        return NULL;
    }

    jint fill[4 * li.size()];
    for(int i = 0; i < li.size(); i++) {
        fill[4 * i] = li[i].val[0];
        fill[4 * i + 1] = li[i].val[1];
        fill[4 * i + 2] = li[i].val[2];
        fill[4 * i + 3] = li[i].val[3];
    }

    env->SetIntArrayRegion(res, 0, 4 * li.size(), fill);

    return res;

    //__android_log_print(ANDROID_LOG_ERROR, "LANEDETECTOR++", "%s", "end mainDelegate");
}


