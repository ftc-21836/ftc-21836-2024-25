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
import static java.lang.Math.hypot;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    public Sample targetColor1 = NEUTRAL, targetColor2 = targetColor1;
    public double
            minSize = 1000,
            maxSize = 60000,

            fractionOfContours = 0.5,
            fractionOfByYs = 0.5,
            fractionOfDensities = 0.5,

            SCREEN_WIDTH = 800,
            SCREEN_HEIGHT = 600;

    private final Point cornerBR = new Point(SCREEN_WIDTH, SCREEN_HEIGHT);

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
                    minYellow = new Scalar(0, 36.8, 0),
                    maxYellow = new Scalar(60.9, 255.0, 255.0);

    public int lineThickness = 6;

    public Point target = null;

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

        // contour area
        {
            contours.sort((c1, c2) -> (int) (100000000 * (contourArea(c2) - contourArea(c1))));
            int numContours = (int) round(contours.size() * fractionOfContours);
            while (contours.size() > numContours) contours.remove(contours.size() - 1);
        }

        // density
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
            while (contours.size() > numContoursDensity) contours.remove(contours.size() - 1);
        }

        // closest to bottom right
        {
            contours.sort((c1, c2) -> {

                contours2f.release();
                c2.convertTo(contours2f, CvType.CV_32F);
                Point rc2 = Imgproc.minAreaRect(contours2f).center;
                double d2 = hypot(SCREEN_WIDTH - rc2.x, SCREEN_HEIGHT - rc2.y);


                contours2f.release();
                c1.convertTo(contours2f, CvType.CV_32F);
                Point rc1 = Imgproc.minAreaRect(contours2f).center;
                double d1 = hypot(SCREEN_WIDTH - rc1.x, SCREEN_HEIGHT - rc1.y);

                return (int) (1000000 * (d1 - d2));
            });
            int numContoursByY = (int) round(contours.size() * fractionOfByYs);
            while (contours.size() > 1) contours.remove(contours.size() - 1);
        }

        int i = 0;
        for (MatOfPoint contour : contours) {
            contours2f.release();
            contour.convertTo(contours2f, CvType.CV_32F);
            double area = contourArea(contour);

            RotatedRect rect = Imgproc.minAreaRect(contours2f);

            Point[] rectPoints = new Point[4];
            rect.points(rectPoints);
            MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

            Imgproc.polylines(input, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);

            double rectArea = rect.size.area();
            Point center = rect.center;
            Imgproc.line(input, center, cornerBR, white, 1);

            Imgproc.putText(input, "" + ++i + " (" + center.x + ", " + center.y + ")", new Point(center.x, center.y - 20), 2, .5, aqua);
            Imgproc.putText(input, "" + (area / rectArea), center, 2, .5, lavender);
            Imgproc.putText(input, "" + ((int) area ), new Point(center.x, center.y + 20), 2, .5, green);
        }


        contours2f.release();
        contours.get(0).convertTo(contours2f, CvType.CV_32F);

        Point center = Imgproc.minAreaRect(contours2f).center;
        target.x = center.x;
        target.y = center.y;

        telemetry.addData("Target X", target.x);
        telemetry.addData("Target Y", target.y);
        telemetry.update();
        Imgproc.drawMarker(input, target, green, 2, 8);

        if (viewContours) return hsvBinaryMat;
        hsvBinaryMat.release();
        return input;
    }
}