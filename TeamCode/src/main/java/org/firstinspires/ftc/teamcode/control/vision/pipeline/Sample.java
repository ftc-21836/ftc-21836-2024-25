package org.firstinspires.ftc.teamcode.control.vision.pipeline;

import org.opencv.core.Scalar;

public enum Sample {
    NEUTRAL (
            new Scalar(
                    10,
                    100,
                    50
            ),
            new Scalar(
                    30,
                    255,
                    255
            )
    ),
    BLUE (
             new Scalar(
                    90,
                    95,
                    50
            ),
            new Scalar(
                    150,
                    255,
                    255
            )
    ),
    RED (
            new Scalar(
                    150,
                    100,
                    50
            ),
            new Scalar(
                    180,
                    255,
                    255
            )
    );

    final Scalar min, max;

    Sample(Scalar min, Scalar max) {
        this.min = min;
        this.max = max;
    }
}
