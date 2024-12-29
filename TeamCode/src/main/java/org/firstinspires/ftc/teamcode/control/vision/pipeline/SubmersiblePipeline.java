package org.firstinspires.ftc.teamcode.control.vision.pipeline;

import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.blue;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.lavender;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.red;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline.yellow;
import static org.opencv.imgproc.Imgproc.FILLED;
import static org.opencv.imgproc.Imgproc.MARKER_CROSS;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    public boolean isRedAlliance = false, warp = false;

    private static final double
            TARGET_ANGLE = toRadians(30),
            SAMPLE_WIDTH = 85,
            SAMPLE_LENGTH = SAMPLE_WIDTH * 3.5 / 1.5;

    private final Scalar[] warpColors = {blue, red, lavender, yellow};

    private final Point
            TARGET_1 = new Point(450,800),
            TARGET_3 = new Point(TARGET_1.x + SAMPLE_LENGTH * cos(TARGET_ANGLE-PI/2),TARGET_1.y - SAMPLE_LENGTH * sin(TARGET_ANGLE-PI/2));

    private final Point[]
            TARGET = {
                    TARGET_1,
                    new Point(TARGET_1.x + SAMPLE_WIDTH * cos(TARGET_ANGLE),TARGET_1.y - SAMPLE_WIDTH * sin(TARGET_ANGLE)),
                    TARGET_3,
                    new Point(TARGET_3.x + SAMPLE_WIDTH * cos(TARGET_ANGLE),TARGET_3.y - SAMPLE_WIDTH * sin(TARGET_ANGLE))
            },
            SOURCE_1 = {
                    new Point(425, 535),
                    new Point(540, 500),
                    new Point(607, 715),
                    new Point(735, 665)
            };

    private final Scalar
            minRed = new Scalar(
                    0,
                    0.25,
                    0
            ),
            maxRed = new Scalar(
                    30,
                    0.75,
                    0.06
            ),
            minYellow = new Scalar(
                    80,
                    0.6,
                    0
            ),
            maxYellow = new Scalar(
                    96,
                    0.85,
                    0.3
            ),
            minBlue = new Scalar(
                    215,
                    0.6,
                    0
            ),
            maxBlue = new Scalar(
                    230,
                    0.9,
                    0.1
            );


    private final MatOfPoint2f
            source = new MatOfPoint2f(
                    SOURCE_1[0],
                    SOURCE_1[1],
                    SOURCE_1[2],
                    SOURCE_1[3]
            ),
            dest = new MatOfPoint2f(
                    TARGET[0],
                    TARGET[1],
                    TARGET[2],
                    TARGET[3]
            );

    private final Mat transformMatrix = Imgproc.getPerspectiveTransform(source, dest);

    public SubmersiblePipeline(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched

        for (int i = 0; i < warpColors.length; i++) {
            Imgproc.drawMarker(input, SOURCE_1[i], warpColors[i], MARKER_CROSS, 25, 10);
            Imgproc.circle(input, TARGET[i], 10, warpColors[i], 10, FILLED);
        }

        if (warp) {
            Imgproc.warpPerspective(input, input, transformMatrix, input.size());
        }

        return input;
    }
}