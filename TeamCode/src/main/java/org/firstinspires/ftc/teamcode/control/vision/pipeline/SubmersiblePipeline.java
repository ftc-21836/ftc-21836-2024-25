package org.firstinspires.ftc.teamcode.control.vision.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

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

    public SubmersiblePipeline(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched

        return input;
    }
}