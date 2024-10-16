package org.firstinspires.ftc.teamcode.control.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class Integrator {

    private double integral = 0.0;
    private boolean stopIntegration = false;

    private final ElapsedTime timer = new ElapsedTime();

    public double getIntegral(double newValue) {

        double dt = timer.seconds();
        timer.reset();

        if (!stopIntegration) integral += newValue * dt;

        return integral;
    }

    public void stopIntegration(boolean stopIntegration) {
        this.stopIntegration = stopIntegration;
    }

    public void reset() {
        integral = 0.0;
    }
}
