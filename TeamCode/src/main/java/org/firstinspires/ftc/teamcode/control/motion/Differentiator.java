package org.firstinspires.ftc.teamcode.control.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class Differentiator {

    private double lastValue = Double.NaN, derivative = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public double getDerivative(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (dt != 0.0 && !Double.isNaN(lastValue)) {
            derivative = (newValue - lastValue) / dt;
        }

        lastValue = newValue;

        return derivative;
    }
}
