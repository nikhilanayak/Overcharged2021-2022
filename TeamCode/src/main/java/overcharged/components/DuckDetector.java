package overcharged.components;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static overcharged.config.RobotConstants.TAG_TD;

/**
 * NOTE:
 * OpenCV is an image processing library. It contains a large collection of image processing functions.
 * To solve a computational challenge, most of the time you will end up using multiple functions of the library.
 * Because of this, passing images to functions is a common practice.
 * We should not forget that we are talking about image processing algorithms, which tend to be quite computationally heavy.
 * The last thing we want to do is further decrease the speed of your program by making unnecessary copies of potentially large images.
 */

//Red=left Blue=Right
public class DuckDetector extends DogeCVDetector {
    public static double C_ONE = 0.2;
    public static double C_TWO = 0.45;
    public static double C_THREE = 0.7;
    public static double C_DIFF = 0.05;

    public static double R_TOP = 0.7;
    public static double R_DIFF = 0.05;
    public static double R_BOTTOM = R_TOP+R_DIFF;

    public static long DETECTION_WAIT_TIME = 800;

    /*public static double START_ROW = 0.3; //0.455
    public static double END_ROW = 0.9; //0.63
    public static double BLUE_START_ROW = 0.38; //0.455
    public static double BLUE_END_ROW = 0.98; //0.63
    public static double COL_diff = 0.2; //0.15,0.18
    public double START_COL_RED = 0.1;
    public double END_COL_RED = START_COL_RED + COL_diff;
    public double START_COL_BLUE = 0.18;
    public double END_COL_BLUE = START_COL_BLUE + COL_diff;

    public double START_FAR_COL_RED = 0;
    public double END_FAR_COL_RED = START_FAR_COL_RED + COL_diff;
    public double START_FAR_COL_BLUE = 0.5;
    public double END_FAR_COL_BLUE = START_FAR_COL_BLUE + COL_diff;
    ///This is the minimum time needed to wait for the detection to work correctly

    //Put the values found during testing here
    public double BLUE_FIRST_CUTOFF = 125;
    public double BLUE_SECOND_CUTOFF = 35;

    public double RED_FIRST_CUTOFF = 28;
    public double RED_SECOND_CUTOFF = 113;*/

    // Results of the detector
    private Point screenPosition = new Point(); // Center screen position of the block
    private Rect foundRect = new Rect(); // Found rect
    private DuckPositions duckPositions = DuckPositions.A;

    private Mat rawImage1 = new Mat();
    private Mat rawImage2 = new Mat();
    private Mat rawImage3 = new Mat();
    private Mat outMat = new Mat();


    public Point getScreenPosition() {
        return screenPosition;
    }
    public Rect foundRectangle() {
        return foundRect;
    }
    public DuckPositions getDuckPositions() {
        return duckPositions;
    }


    public DuckDetector() {
        detectorName = "12599 Duck Detector";
    }

    public void reset() {
        found = false;
        foundRect = new Rect();
        screenPosition = new Point();
        duckPositions = DuckPositions.A;
    }

    @Override
    public Mat process(Mat input) {
        // These need to be computed at program start to prevent FTC dashboard from changing the values
        // of START_ROW and the like between when we crop RawImage and when we copy it and output it
        RobotLog.ii(TAG_TD, "colorGray=" + input.rows());
        int SC1 = (int)(input.cols() * C_ONE);
        int SC2 = (int)(input.cols() * C_TWO);
        int SC3 = (int)(input.cols() * C_THREE);

        int EC1 = (int)(input.cols() * (C_ONE+C_DIFF));
        int EC2 = (int)(input.cols() * (C_TWO+C_DIFF));
        int EC3 = (int)(input.cols() * (C_THREE+C_DIFF));

        int SR = (int)(input.rows() * R_TOP);
        int ER = (int)(input.rows() * R_BOTTOM);

        /*int startRow = (int) (input.rows() * (this.isRed ? START_ROW : BLUE_START_ROW));
        int endRow = (int) (input.rows() * (this.isRed ? END_ROW : BLUE_END_ROW));
        int startCol = (int) (input.cols() * (this.isRed ? START_COL_RED : START_COL_BLUE));
        int endCol = (int) (input.cols() * (this.isRed ? END_COL_RED : END_COL_BLUE));
        int margin = 30;
        //int halfCol = (startCol+endCol)/2;
        int halfRow = (startRow+endRow)/2 - 10;
        int lowRow = halfRow+margin;
        double greyRed = 0.7;
        double greyBlue = 0.7;
        int greyCol = (int) (input.cols() * (this.isRed ? greyRed : greyBlue));
        double greyColLength = 0.1;
        int greyEndCol = (int)(input.cols() * (greyColLength + (this.isRed ? greyRed : greyBlue)));*/

        Scalar A, B, C;
        double colorA, colorB, colorC, AB, AC, BC, tolerance=30, tolerance2=35;

        rawImage1 = input.submat(SR, ER, SC1, EC1);
        rawImage2 = input.submat(SR, ER, SC2, EC2);
        rawImage3 = input.submat(SR, ER, SC3, EC3);

        /*Point tl = new Point(SR, startRow);
        //Point br = new Point(halfCol-margin, endRow);
        Point br = new Point(endCol, halfRow);
        Rect rect1 = new Rect(tl,br);

        tl.y = startRow;
        tl.x = halfCol+margin;
        tl.y = lowRow;
        tl.x = startCol;
        br.y = endRow;
        br.x = endCol;
        Rect rect2 = new Rect(tl,br);*/

        A = Core.mean(rawImage1);
        B = Core.mean(rawImage2);
        C = Core.mean(rawImage3);
        /*colorA = (A.val)[2]+(A.val)[1]+(A.val)[2];
        colorB = (B.val)[2]+(B.val)[1]+(B.val)[2];
        colorC = (C.val)[2]+(C.val)[1]+(C.val)[2];*/
        colorA = (A.val)[0];
        colorB = (B.val)[0];
        colorC = (C.val)[0];
        AB = Math.abs(colorA-colorB);
        AC = Math.abs(colorA-colorC);
        BC = Math.abs(colorB-colorC);

        RobotLog.ii(TAG_TD, "Colors Log colorA: " + (A.val)[0] + " : " + (A.val)[1] + " : " + (A.val)[2] + " total= " + colorA);
        RobotLog.ii(TAG_TD, "Colors Log colorB: " + (B.val)[0] + " : " + (B.val)[1] + " : " + (B.val)[2] + " total= " +  colorA);
        RobotLog.ii(TAG_TD, "Colors Log colorC: " + (C.val)[0] + " : " + (C.val)[1] + " : " + (C.val)[2] + " total= " + colorC);

        if(BC<AB && BC<AC) {
            duckPositions = DuckPositions.A;
        } else if(AC<AB && AC<BC) {
            duckPositions = DuckPositions.B;
        } else {
            duckPositions = DuckPositions.C;
        }

        /*found = true;
        if (colorTop - colorBottom > tolerance*2) { //1 ring
            ringPosition = RingPosition.B;
            foundRect = rect2;
        } else if (colorGray - colorTop > tolerance2) { //4 rings
            ringPosition = RingPosition.C;
            foundRect = rect1;
        } else { //no ring
            ringPosition = RingPosition.A; //0 rings
            foundRect.height = 0;
            foundRect.width = 0;
            found = false;
        }
        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);

        input.copyTo(outMat);
        Point trueTL = new Point(startCol, startRow);
        Point trueBR1 = new Point(endCol, halfRow);
        Point trueBR = new Point(endCol, endRow);
        Point trueTL2 = new Point(startCol, lowRow);
        Rect crop = new Rect(trueTL, trueBR1);
        Rect crop2 = new Rect(trueTL2, trueBR);
        Point grey = new Point(greyCol, halfRow);
        Point grey2 = new Point(greyEndCol, endRow);
        Rect greyRect = new Rect(grey, grey2);

        Imgproc.rectangle(outMat, crop.tl(), crop.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, crop2.tl(), crop2.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, greyRect.tl(), greyRect.br(), new Scalar(0,0,255),2);
        if (found) {
            Imgproc.rectangle(outMat, foundRect, new Scalar(255, 0, 0), 4);
            screenPosition.x = foundRect.x + foundRect.width / 2;
            screenPosition.y = foundRect.y + foundRect.height / 2;
        } else {
            screenPosition.x = 0;
            screenPosition.y = 0;
        }*/

        input.copyTo(outMat);
        Point t1 = new Point(SC1, SR);
        Point b1 = new Point(EC1, ER);
        Rect rect1 = new Rect(t1, b1);

        Point t2 = new Point(SC2, SR);
        Point b2 = new Point(EC2, ER);
        Rect rect2 = new Rect(t2, b2);

        Point t3 = new Point(SC3, SR);
        Point b3 = new Point(EC3, ER);
        Rect rect3 = new Rect(t3, b3);

        Imgproc.rectangle(outMat, rect1.tl(), rect1.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, rect2.tl(), rect2.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, rect3.tl(), rect3.br(), new Scalar(0,0,255),2);
        return outMat;
    }

    public void setRange(double C1, double C2, double C3, double RS){
        C_ONE = C1;
        C_TWO = C2;
        C_THREE = C3;
        R_TOP = RS;
        R_BOTTOM = R_TOP+R_DIFF;
    }

    @Override
    public void useDefaults() {
    }
}
