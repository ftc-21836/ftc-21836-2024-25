package org.firstinspires.ftc.teamcode.control.vision.pipeline;

import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.blue;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.green;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    public Scalar lowerHSV = new Scalar(151.0, 126.0, 69.0, 0.0);
    public Scalar upperHSV = new Scalar(255.0, 255.0, 255.0, 0.0);
    private final Mat hsvMat = new Mat();
    private final Mat hsvBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    private MatOfPoint2f contours2f = new MatOfPoint2f();
    private ArrayList<RotatedRect> contoursRotRects = new ArrayList<>();

    public Scalar lineColor = new Scalar(255.0, 255.0, 255.0, 0.0);
    public int lineThickness = 6;

    private MatOfPoint biggestContour = null;
    public Point target = null;

    private final Telemetry telemetry;

    public SubmersiblePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursRotRects.clear();
        biggestContour = null;
        for (MatOfPoint contour : contours) {
            contours2f.release();
            contour.convertTo(contours2f, CvType.CV_32F);

            RotatedRect rect = Imgproc.minAreaRect(contours2f);

            contoursRotRects.add(rect);

            Point[] rectPoints = new Point[4];
            rect.points(rectPoints);
            MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

            Imgproc.polylines(input, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);

            if(biggestContour == null || Imgproc.contourArea(contour) > Imgproc.contourArea(biggestContour)) {
                biggestContour = contour;
                target = rect.center.clone();
            }
        }

        telemetry.addData("Target X", target.x);
        telemetry.addData("Target Y", target.y);
        Imgproc.drawMarker(input, target, green, 2, 8);

        return input;
    }
}