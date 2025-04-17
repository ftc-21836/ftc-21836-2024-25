package org.firstinspires.ftc.teamcode.control.vision.pipeline;

import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.aqua;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.black;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.blue;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.green;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.lavender;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.red;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.white;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.yellow;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.*;
import static org.opencv.imgproc.Imgproc.contourArea;

import static java.lang.Double.NaN;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    public Sample targetColor1 = NEUTRAL, targetColor2 = targetColor1;
    public double
            minSize = 200,
            maxSize = 120000,

            fractionOfContours = 0.5,
            fractionOfByYs = 0.25,
            fractionOfDensities = 0.5,

            SCREEN_WIDTH = 1280,
            SCREEN_HEIGHT = 960,

            Y_IMPORTANCE_SCALAR = 2,

            PIXELS_PER_INCH = 20 *72 / 15.0 *1.5,

            DISTANCE_TO_SUB_INCH = 20,

            INTAKE_X_POSITON = 480;

//            C = -0,
//            Xc = 5.1,
//            Yc = 7.95,
//            Zc = 9.7,
//            a = toRadians(12),
//            b = toRadians(80),
//            tanB = tan(b);

    private final Point cornerBR = new Point(SCREEN_WIDTH, SCREEN_HEIGHT);
    private final Point cornerBL = new Point(INTAKE_X_POSITON, SCREEN_HEIGHT);

    public boolean viewContours = false;

    private final Mat hsvMat = new Mat();
    private final Mat hsvBinaryMat = new Mat(), hsvBinaryMat2 = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>(), contours2 = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    private final MatOfPoint2f contours2f = new MatOfPoint2f();

    public Scalar lineColor = new Scalar(255.0, 255.0, 255.0, 0.0),
                    minRed = new Scalar(150, 50, 0),
                    maxRed = new Scalar(255, 255.0, 255.0),
                    minBlue = new Scalar(55, 145, 0),
                    maxBlue = new Scalar(145, 255.0, 255.0),
                    minYellow = new Scalar(11, 83, 60),
                    maxYellow = new Scalar(52.4, 255.0, 255.0);

    public int lineThickness = 6;

    public Point target = null;
    public Point screenCenter = null;

    private final Telemetry telemetry;

    public SubmersiblePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private Scalar minBound(Sample color) {
        return color == RED ? minRed :
                color == BLUE ? minBlue :
                color == NEUTRAL ? minYellow :
                black;
    }

    private Scalar maxBound(Sample color) {
        return color == RED ? maxRed :
                color == BLUE ? maxBlue :
                color == NEUTRAL ? maxYellow :
                black;
    }

    @Override
    public Mat processFrame(Mat input) {
        Core.rotate(input, input, Core.ROTATE_180);
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, minBound(targetColor1), maxBound(targetColor1), hsvBinaryMat);
        if (!(targetColor2 == null || targetColor1 == targetColor2)) Core.inRange(hsvMat, minBound(targetColor2), maxBound(targetColor2), hsvBinaryMat2);

        hsvMat.release();

        contours.clear();

        contours2.clear();
        hierarchy.release();
        Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(hsvBinaryMat2, contours2, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        hsvBinaryMat2.release();

        contours.addAll(contours2);
        contours.removeIf(c -> {
            double area = contourArea(c);
            return area < minSize || area > maxSize;
        });

        // closest to bottom left
        {
            contours.sort((c1, c2) -> {

                contours2f.release();
                c2.convertTo(contours2f, CvType.CV_32F);
                Point rc2 = Imgproc.minAreaRect(contours2f).center;
                double d2 = hypot(cornerBL.x - rc2.x, Y_IMPORTANCE_SCALAR * (SCREEN_HEIGHT - rc2.y));


                contours2f.release();
                c1.convertTo(contours2f, CvType.CV_32F);
                Point rc1 = Imgproc.minAreaRect(contours2f).center;
                double d1 = hypot(cornerBL.x - rc1.x, Y_IMPORTANCE_SCALAR * (SCREEN_HEIGHT - rc1.y));

                return (int) (10000000 * (d1 - d2));
            });
            int numContoursByY = (int) round(contours.size() * fractionOfByYs);
            while (contours.size() > numContoursByY) contours.remove(contours.size() - 1);
        }

        /*// density
        {
            contours.sort((c1, c2) -> {
                contours2f.release();
                c2.convertTo(contours2f, CvType.CV_32F);
                RotatedRect r2 = Imgproc.minAreaRect(contours2f);

                contours2f.release();
                c1.convertTo(contours2f, CvType.CV_32F);
                RotatedRect r1 = Imgproc.minAreaRect(contours2f);

                return (int) (100000000 * (
                        contourArea(c2) / r2.size.area() -
                                contourArea(c1) / r1.size.area()
                ));
            });
            int numContoursDensity = (int) round(contours.size() * fractionOfDensities);
            while (contours.size() > 1) contours.remove(contours.size() - 1);
        }

        // contour area
        {
            contours.sort((c1, c2) -> (int) (100000000 * (contourArea(c2) - contourArea(c1))));
            int numContours = (int) round(contours.size() * fractionOfContours);
            while (contours.size() > numContours) contours.remove(contours.size() - 1);
        }*/


        int i = 0;
        for (MatOfPoint contour : contours) {
            contours2f.release();
            contour.convertTo(contours2f, CvType.CV_32F);
            double area = contourArea(contour);

            RotatedRect rect = Imgproc.minAreaRect(contours2f);

            Point[] rectPoints = new Point[4];
            rect.points(rectPoints);
            MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

            Imgproc.polylines(input, Collections.singletonList(matOfPoint), true, i == 0 ? lavender : white, lineThickness);

            double rectArea = rect.size.area();
            Point center = rect.center;
            Point vertex2 = new Point(center.x, SCREEN_HEIGHT);
//            Imgproc.line(input, center, cornerBR, white, 1);
            Imgproc.line(input, center, cornerBL, green, 1);
//            Imgproc.line(input, vertex2, cornerBL, blue, 1);

            Imgproc.putText(input, "" + ++i + " (" + center.x + ", " + center.y + ")", new Point(center.x, center.y - 20), 2, .5, aqua);
            Imgproc.putText(input, "" + (area / rectArea), center, 2, .5, lavender);
            Imgproc.putText(input, "" + ((int) area ), new Point(center.x, center.y + 20), 2, .5, green);
        }


        if (!contours.isEmpty()) {
            contours2f.release();
            contours.get(0).convertTo(contours2f, CvType.CV_32F);

            Point center = Imgproc.minAreaRect(contours2f).center;

            //sample center in inches
            target.x = (center.x) / PIXELS_PER_INCH;
            target.y = (center.y) / PIXELS_PER_INCH;

            //screen center in inches
            screenCenter.x = (0.5 * SCREEN_WIDTH) / PIXELS_PER_INCH;
            screenCenter.y = (0.5 * SCREEN_HEIGHT) / PIXELS_PER_INCH;

            //center to center distance
            double ctc = (screenCenter.y - target.y)/(screenCenter.x - target.x);

            //turn angle in radians -> intake to sample
            double turnAngle = (atan2(ctc,DISTANCE_TO_SUB_INCH));

            telemetry.addLine("Turn Angle " + toDegrees(turnAngle));

            /*double xv = (target.x - 0.5 * SCREEN_WIDTH) / PIXELS_PER_INCH;
            double yv = (target.y - 0.5 * SCREEN_HEIGHT) / PIXELS_PER_INCH;

            telemetry.addLine("camera view " + xv + " " + yv);

            double y = tan(b) * (Zc - (yv * sin(b) + 0.75)) - yv * cos(b) + C;

            telemetry.addLine("camera distance " + y);

            double rv = hypot(xv, y);
            double tv = -atan2(xv, y);
            double tv2 = tv + a;

            double xS = rv * cos(tv2) + Xc;
            double yS = rv * sin(tv2) + Yc;

            double theta = -atan2(xS, yS);

            double L = hypot(xS, yS) - 7.2;

            double angle = atan2(cornerBL.x - center.x, (SCREEN_HEIGHT-center.y));*/

            telemetry.addData("Target X in inches", target.x);
            telemetry.addData("Target Y in inches", target.y);
            //telemetry.addData("Turn angle (deg)", toDegrees(theta));
            //telemetry.addData("Extension (in)", L);
            telemetry.update();
            Imgproc.drawMarker(input, target, green, 2, 8);
        }

        if (viewContours) return hsvBinaryMat;
        hsvBinaryMat.release();
        return input;
    }
}