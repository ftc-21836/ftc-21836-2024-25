package org.firstinspires.ftc.teamcode.control.motion;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;

@Config
public final class PIDDriver {

    public static PIDGains
            xyGains = new PIDGains(
            0.0175,
                    0,
                    0,
                    1
            ),
            rotGains = new PIDGains(
                    1.25,
                    0.75,
                    0,
                    1
            );

    public static double STRAFE_MULTIPLIER = 1;

    public static EditablePose admissibleError = new EditablePose(0.01, 0.01, 0.001);

    private final PIDController
            xController = new PIDController(),
            yController = new PIDController(),
            rotController = new PIDController();

    public void reset() {
        xController.reset();
        yController.reset();
        rotController.reset();
    }

    public DriverOutput driveTo(EditablePose current, EditablePose target) {

        double xError = target.x - current.x;
        double yError = target.y - current.y;
        double headingError = normalizeRadians(target.heading - current.heading);

        xController.setGains(xyGains);
        yController.setGains(xyGains);
        rotController.setGains(rotGains);

        xController.setTarget(new State(target.x));
        yController.setTarget(new State(target.y *1));
        rotController.setTarget(new State(headingError + current.heading));

        double x = xController.calculate(new State(current.x));
        double y = yController.calculate(new State(current.y *1));
        double rot = rotController.calculate(new State(current.heading));

        DriverOutput output = new DriverOutput();

        output.drivePower = new EditablePose(
                x,
                y * STRAFE_MULTIPLIER,
                -rot
        );

        output.withinError = abs(xError) <= admissibleError.x &&
                            abs(yError) <= admissibleError.y &&
                            abs(headingError) <= admissibleError.heading;

        return output;
    }

    public static class DriverOutput {

        public boolean withinError;
        public EditablePose drivePower;
    }
}
