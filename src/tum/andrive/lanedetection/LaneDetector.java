
package tum.andrive.lanedetection;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.gpu.Gpu;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.Objdetect;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.PI;
import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.line;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.Canny;
import static org.opencv.imgproc.Imgproc.HoughLines;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;
import static org.opencv.imgproc.Imgproc.threshold;

/**
 * This is the main Activity which takes Camera input, captures frame, interact with OpenCV library, process frame and display overlays
 * on lanes detection. It uses Hough Value which is provided by the VerticalSliderActivity. This java activity acts as the Controller
 * here and core image processing work is done by the C++ files
 *
 * @author Wahib-Ul-Haq, wahib.tech@gmail.com, 2014
 *         <p>
 *         contains code taken from:
 *         <p>
 *         "Lane detection using OpenCV"
 *         https://github.com/jdorweiler/lane-detection
 *         Acknowledgments: jdorweiler
 *         Contact: jason@transistor.io
 */

public class LaneDetector extends Activity implements CvCameraViewListener2 {

    private CameraBridgeViewBase mOpenCvCameraView;
    private static final String TAG = "OCVSample::Activity";
    private Mat img;
    private int houghValue;
    private Mat mRgba;
    private int screen_w, screen_h;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    System.loadLibrary("LaneDetectionNative"); //library module name defined in Android.mk
                    mOpenCvCameraView.enableView();

                }
                break;

                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    public void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_8, this, mLoaderCallback);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_lane_test);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.AndriveLaneView);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        Intent intent = getIntent();
        houghValue = Integer.valueOf(intent.getStringExtra("houghvalue"));
        Toast.makeText(getApplicationContext(), String.valueOf(houghValue), Toast.LENGTH_LONG).show();
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null) {
            mOpenCvCameraView.disableView();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null) {
            mOpenCvCameraView.disableView();
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        Log.v("Screen Resolution", "Height: " + height + " Width: " + width);

        img = new Mat(height, width, CvType.CV_8UC4);
    }

    @Override
    public void onCameraViewStopped() {
        img.release();
    }

    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
//        mainDelegate(inputFrame.rgba().getNativeObjAddr(), img.getNativeObjAddr(), houghValue);
//        process(inputFrame.rgba().getNativeObjAddr(), img.getNativeObjAddr(), houghValue);

//        return inputFrame.rgba();
        process(inputFrame.rgba(), img, houghValue);



        return img;
    }

    public native int[] mainDelegate(long input, long output, int houghValue);

    private void process(Mat in, Mat out, int houghValue) {
        final Mat image = new Mat();

        Imgproc.resize(in, image, new Size(), 0.25, 0.25, Imgproc.INTER_LINEAR);

        Imgproc.cvtColor(image, image, COLOR_RGB2GRAY);

        final Mat imgROI = image.adjustROI(0, image.cols() / 3, image.cols() - 1, image.rows() - image.cols() / 3);
        final Mat contours = new Mat();
        Canny(imgROI, contours, 50, 250);

        final Mat contoursInv = new Mat();
        threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);

        final MatOfPoint2f lines = new MatOfPoint2f();
        HoughLines(contours, lines, 1, PI / 180, houghValue);

        final Mat result = new Mat(imgROI.size(), CV_8U, new Scalar(255));
        imgROI.copyTo(result);

        final Mat hough = new Mat(imgROI.size(), CV_8U, new Scalar(0));

        for(int x = 0; x < lines.cols(); x++) {
            float rho = (float)lines.get(0, x)[0];
            float theta = (float)lines.get(0, x)[1];

            if ((theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66)) {
                final Point pt1 = new Point(rho / Math.cos(theta), 0);
                final Point pt2 = new Point((rho - result.rows() * Math.sin(theta)) / Math.cos(theta), result.rows());

                line(hough, pt1, pt2, new Scalar(255), 5);
            }
        }

        final LineFinder ld = new LineFinder();
        ld.setLineLengthAndGap(60, 10);
        ld.setMinVote(4);

        ld.findLines(contours);

        final Mat houghP = new Mat(imgROI.size(), CV_8U, new Scalar(0));
        ld.setShift(0);

        bitwise_and(houghP, hough, houghP);

        final Mat houghPinv = new Mat(imgROI.size(), CV_8U, new Scalar(0));
        threshold(houghP, houghPinv, 150, 255, THRESH_BINARY_INV);

        Canny(houghPinv, contours, 100, 350);
        ld.findLines(contours);

        ld.setLineLengthAndGap(5, 2);
        ld.setMinVote(1);

        ld.setShift(image.cols() / 3);
        ld.drawDetectedLines(image);

        Imgproc.resize(image, out, new Size(), 4, 4, Imgproc.INTER_LINEAR);

//        Mat imageOrig = new Mat(in);
//        Mat output = new Mat(out);
//
//        Mat image = new Mat();
//
//        Imgproc.resize(imageOrig, image, new Size(), 0.25, 0.25, Imgproc.INTER_LINEAR);
//
//        int houghVote = houghValue;
//
//        Log.e("LANEDETECTOR++", "hough value : " + houghVote);
//
//        if (image.empty()) {
//            Log.e("LANEDETECTOR++", "Cannot access the camera");
//
//            return null;
//        }
//
//        Mat gray = new Mat();
//        Imgproc.cvtColor(image, gray, COLOR_RGB2GRAY);
//        List<String> codes = new ArrayList<String>();
//
//        Mat corners = new Mat();
//
////        findDataMatrix(gray, codes, corners);
////        drawDataMatrixCodes(image, codes, corners);
//
//    /*ROI returns a matrix that is pointing to the ROI of the original image, located at the place specified by the rectangle.
//    so imageROI is really the Region of Interest (or subimage/submatrix) of the original image "image".
//    If you modify imageROI it will consequently modify the original, larger matrix.
//    Rect region_of_interest = Rect(x, y, w, h); current parameters try to take the lower half of the image on vertical frame*/
//
//        Rect roi = new Rect(0, image.cols() / 3, image.cols() - 1, image.rows() - image.cols() / 3);// set the ROI (region of interest) for the image
//
//        Mat imgROI = image.adjustROI(0, image.cols() / 3, image.cols() - 1, image.rows() - image.cols() / 3);
//
//    /* Canny Edge Detector algorithm : canny (source image, edge output image, first threshold for the hysteresis procedure,
//       , second threshold for the hysteresis procedure, apersturesize i.e set to be 3 by default , L2gradient )
//       threshold2 is recommended to be 3 times to threshold1 */
//
//        Mat contours = new Mat();
//        Canny(imgROI, contours, 50, 250); //50, 150,3); //in original code => 50,250);
//
//    /* Thresholding is to differentiate the pixels we are interested in from the rest */
//        Mat contoursInv = new Mat();
//        threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);
//
//        // Hough tranform for line detection with feedback
//        // Increase by 25 for the next frame if we found some lines.
//        // This is so we don't miss other lines that may crop up in the next frame
//        // but at the same time we don't want to start the feed back loop from scratch.
//
//        Mat lines = new Mat();
//        if (houghVote < 1 || lines.cols() > 2) { // we lost all lines. reset
//            houghVote = houghValue; //previously it was set to 200 always
//        } else {
//            houghVote += 25;
//        }
//
//        //ensuring lines size is 5
//        while (lines.cols() < 5 && houghVote > 0) {
//            HoughLines(contours, lines, 1, PI / 180, houghVote);
//            houghVote -= 5;
//        }
//
//        Mat result = new Mat(imgROI.size(), CV_8U, new Scalar(255));
//        imgROI.copyTo(result);
//
//        // Draw the lines
//        Mat hough = new Mat(imgROI.size(), CV_8U, new Scalar(0));
//
//        for(int x = 0; x < lines.cols(); x++) {
//            float rho = (float)lines.get(0, x)[0];
//            float theta = (float)lines.get(0, x)[1];
//
//            if ((theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66)) {
//                // point of intersection of the line with first row
//                Point pt1 = new Point(rho / Math.cos(theta), 0);//y = 0
//
//                // point of intersection of the line with last row
//                Point pt2 = new Point((rho - result.rows() * Math.sin(theta)) / Math.cos(theta), result.rows());//y = row count
//
//                //This draws lines but without ends and in shape of X
//                line(hough, pt1, pt2, new Scalar(255), 5); //this is working and shows red lines of thickness 5
//            }
//        }
//
//        // Create LineFinder instance
//        LineFinder ld = new LineFinder();
//
//        // Set probabilistic Hough parameters
//        ld.setLineLengthAndGap(60, 10);
//        ld.setMinVote(4);
//
//        // Detect lines
//        Mat li = ld.findLines(contours); //applying probablity hough transform
//
//        Mat houghP = new Mat(imgROI.size(), CV_8U, new Scalar(0));
//        ld.setShift(0);
////    ld.drawDetectedLines(houghP);
//
//        // bitwise AND of the two hough images.
//        //1) Normal Hough -> without end points 2) Probabilistic Hough -> with end points
//
//        bitwise_and(houghP, hough, houghP);
//        Mat houghPinv = new Mat(imgROI.size(), CV_8U, new Scalar(0));
//        Mat dst = new Mat(imgROI.size(), CV_8U, new Scalar(0));
//        threshold(houghP, houghPinv, 150, 255, THRESH_BINARY_INV); // threshold and invert to black lines
//
//        Canny(houghPinv, contours, 100, 350);//100,150); //100,350);
//        li = ld.findLines(contours);
//
//        // Set probabilistic Hough parameters
//        //These parameters set min and max values. If you set minvote as 100 then even if you set 60 from slider, it won't
//        //show any lines
//
//        ld.setLineLengthAndGap(5, 2); //5,2 original
//        ld.setMinVote(1); //1 original
//
//        ld.setShift(image.cols() / 3);
//        ld.drawDetectedLines(image);
//
//        //to show the number of line segments found in a frame
//        //putText(image, stream.str(), Point(10,image.rows-10), 4, 1, Scalar(0,255,255),0);
//
//        lines = new Mat();
//
//        //Hough Processing ends here
//
//        Imgproc.resize(image, image, new Size(), 4, 4, INTER_LINEAR);
//        output = image;
//
//        return null;
    }
}
