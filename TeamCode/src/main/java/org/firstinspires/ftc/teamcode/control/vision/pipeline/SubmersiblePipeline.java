package org.firstinspires.ftc.teamcode.control.vision.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmersiblePipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    public Sample targetColor = Sample.RED;

    public SubmersiblePipeline(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Core.inRange(input, targetColor.min, targetColor.max, input);

        Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGB);
        return input;
    }
}