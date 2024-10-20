package org.firstinspires.ftc.teamcode.control.motion;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

@Config
public final class PIDDriver {

    public static PIDGains
            xyGains = new PIDGains(
                    0,
                    0,
                    0,
                    1
            ),
            rotGains = new PIDGains(
                    0,
                    0,
                    0,
                    1
            );

    public static double STRAFE_MULTIPLIER = 1;

    public static EditablePose admissibleError = new EditablePose(0.01, 0.01, 0.001);

    private final PIDController
            xController = new PIDController(),
            yController = new PIDController(),
            rotController = new PIDController();

    public DriverOutput driveTo(EditablePose current, EditablePose target) {

        double currentX = current.x;
        double currentY = current.y;
        double currentHeading = current.heading;

        double targetX = target.x;
        double targetY = target.y;
        double targetHeading = target.heading;

        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = normalizeRadians(targetHeading - currentHeading);

        xController.setGains(xyGains);
        yController.setGains(xyGains);
        rotController.setGains(rotGains);

        xController.setTarget(new State(targetX));
        yController.setTarget(new State(targetY*1));
        rotController.setTarget(new State(headingError + currentHeading));

        double x = xController.calculate(new State(currentX));
        double y = yController.calculate(new State(currentY*1));
        double rot = rotController.calculate(new State(currentHeading));

        double theta = -currentHeading;
        double cos = cos(theta);
        double sin = sin(theta);
        double x2 = x * cos - y * sin;
        double y2 = y * cos + x * sin;

        DriverOutput output = new DriverOutput();

        output.drivePower = new EditablePose(
                x2,
                y2 * STRAFE_MULTIPLIER,
                rot
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
