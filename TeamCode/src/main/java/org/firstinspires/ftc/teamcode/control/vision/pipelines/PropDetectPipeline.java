package org.firstinspires.ftc.teamcode.control.vision.pipelines;

import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.RIGHT;
import static java.lang.Math.max;
import static java.lang.Math.round;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectPipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    public boolean isRed = true;

    public enum Randomization {
        LEFT,
        CENTER,
        RIGHT;

        public static final Randomization[] randomizations = values();
    }

    private Randomization location = RIGHT;

    private static final double
            X_LEFT_BOUND = 0,
            X_CENTER_BOUND = 240,
            Y_TOP = 0,
            Y_BOTTOM = 480,
            MIN_RED = 7,
            MIN_BLUE = 25;

    private final Scalar
            minBlue = new Scalar(
                    90,
                    95,
                    1
            ),
            maxBlue = new Scalar(
                    150,
                    255,
                    255
            ),
            minRed = new Scalar(
                    -30,
                    40,
                    0
            ),
            maxRed = new Scalar(
                    15,
                    255,
                    255
            );

    private final Rect
            LEFT_AREA = new Rect(
                    new Point(5, 0),
                    new Point(220, 300)
            ),
            CENTER_AREA = new Rect(
                    new Point(300, 0),
                    new Point(550, 150)
            );

    public PropDetectPipeline(Telemetry t) {
        telemetry = t;
    }

    public Randomization getLocation() {
        return location;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Core.inRange(input, (isRed ? minRed : minBlue), (isRed ? maxRed : maxBlue), input);

        Mat left = input.submat(LEFT_AREA);
        Mat middle = input.submat(CENTER_AREA);

        double leftValue = Core.sumElems(left).val[0] / LEFT_AREA.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / CENTER_AREA.area() / 255;

        int leftInt = (int) round(leftValue * 100);
        int middleInt = (int) round(middleValue * 100);

        left.release();
        middle.release();

        telemetry.addData("Left", leftInt + "%");
        telemetry.addData("Center", middleInt + "%");

        int max = max(leftInt, middleInt);

        location = max < (isRed ? MIN_RED : MIN_BLUE) ? RIGHT : max == leftInt ? LEFT : CENTER;
        telemetry.addData("Prop Location", location.toString());
        telemetry.update();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGB);

        Scalar colorDefault = new Scalar(255, 255, 255);
        Scalar colorFound = new Scalar(0, 255, 0);

        Imgproc.rectangle(input, LEFT_AREA, location == LEFT ? colorFound : colorDefault, 2);
        Imgproc.rectangle(input, CENTER_AREA, location == CENTER ? colorFound : colorDefault, 2);

        return input;
    }
}