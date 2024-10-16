package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;

public interface Drivetrain {

    void run(double xCommand, double yCommand, double turnCommand, boolean useSlowMode);
    void lockSlowMode();

    double getHeading();
    void setCurrentHeading(double angle);
    void update();
    boolean isBusy();
    void breakFollowing();

    void printNumericalTelemetry();

    void setPoseEstimate(EditablePose pose);

    EditablePose getPoseEstimate();

    void setDrivePower(EditablePose drive);
}
