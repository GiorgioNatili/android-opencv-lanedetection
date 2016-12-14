package tum.andrive.lanedetection;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt4;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import static java.lang.Math.PI;
import static org.opencv.core.Core.line;
import static org.opencv.imgproc.Imgproc.HoughLinesP;

/**
 * Line finder java port
 * @author Alexandre Lombard
 */
public class LineFinder {
    // vector containing the end points
    // of the detected lines
    private MatOfInt4 lines = new MatOfInt4();

    // accumulator resolution parameters
    private double deltaRho;
    private double deltaTheta;

    // minimum number of votes that a line must receive before being considered.
    //The minimum number of intersections to “detect” a line
    private int minVote;

    // min length for a line
    private double minLength;

    // max allowed gap along the line
    private double maxGap;

    // distance to shift the drawn lines down when using a ROI
    private int shift;

    public LineFinder() {
        this.deltaRho = 1;
        this.deltaTheta = PI / 180;
        this.minVote = 10;
        this.minLength = 0;
        this.maxGap = 0;
    }

    // Set the resolution of the accumulator
    public void setAccResolution(double dRho, double dTheta) {
        deltaRho = dRho;
        deltaTheta = dTheta;
    }

    // Set the minimum number of votes
    public void setMinVote(int minv) {
        minVote = minv;
    }

    // Set line length and gap
    public void setLineLengthAndGap(double length, double gap) {
        minLength = length;
        maxGap = gap;
    }

    // set image shift
    public void setShift(int imgShift) {
        shift = imgShift;
    }

    // Apply probabilistic Hough Transform
    public Mat findLines(Mat binary) {
        lines = new MatOfInt4();

        HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);

        return lines;
    }

    public void drawDetectedLines(Mat image) {
        drawDetectedLines(image, new Scalar(255));
    }

    // Draw the detected lines on an image
    public void drawDetectedLines(Mat image, Scalar color) {
        // Draw the lines
        for(int x = 0; x < lines.cols(); x++) {
            Point pt1 = new Point(lines.get(0, x)[0], lines.get(0, x)[1] + shift); //end coordinate
            Point pt2 = new Point(lines.get(0, x)[2], lines.get(0, x)[3] + shift); //start coordinate

            line(image, pt1, pt2, color, 10);
        }
    }

    // Eliminates lines that do not have an orientation equals to
    // the ones specified in the input matrix of orientations
    // At least the given percentage of pixels on the line must
    // be within plus or minus delta of the corresponding orientation
//    public Mat removeLinesOfInconsistentOrientations(
//            Mat orientations, double percentage, double delta) {
//
//        for(int x = 0; x < lines.cols(); x++) {
//
//            double x1 = lines.get(0, x)[0];
//            double y1 = lines.get(0, x)[1];
//            double x2 = lines.get(0, x)[2];
//            double y2 = lines.get(0, x)[3];
//
//            double ori1 = Math.atan2((double)(y1 - y2), (double)(x1 - x2)) + PI / 2;
//            if (ori1 > PI)
//                ori1 = ori1 - 2 * PI;
//
//            double ori2 = Math.atan2((double)(y2 - y1), (double)(x2 - x1)) + PI / 2;
//
//            if (ori2 > PI)
//                ori2 = ori2 - 2 * PI;
//
//            // for all points on the line
//            LineIterator lit = new LineIterator(orientations, new Point(x1, y1), new Point(x2, y2));
//            int i, count = 0;
//            for (i = 0, count = 0; i < lit.count; i++, ++lit) {
//
//                float ori = *(reinterpret_cast<float *>(*lit));
//
//                // is line orientation similar to gradient orientation ?
//                if (Math.min(Math.abs(ori - ori1), Math.abs(ori - ori2)) < delta)
//                    count++;
//            }
//
//            double consistency = count / (double)(i);
//
//            // set to zero lines of inconsistent orientation
//            if (consistency < percentage) {
//                lines.put(0, x, new double[4]);
//            }
//        }
//
//        return lines;
//    }
}
