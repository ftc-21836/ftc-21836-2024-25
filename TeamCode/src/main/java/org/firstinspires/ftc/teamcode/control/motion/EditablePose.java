package org.firstinspires.ftc.teamcode.control.motion;

import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_START_LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_START_RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.AutonVars;

public final class EditablePose {

    public double x, y, heading;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EditablePose(Pose2d pose) {
        this(pose.position.x, pose.position.y, pose.heading);
    }

    public EditablePose clone() {
        return new EditablePose(x, y, heading);
    }

    public EditablePose byAlliance() {
        double alliance = isRed ? 1 : -1;
        return new EditablePose(
                x,
                y * alliance,
                heading * alliance
        );
    }

    public EditablePose bySide() {
        return new EditablePose(
                x + (AutonVars.isBackdropSide ? 0 : X_START_LEFT - X_START_RIGHT),
                y,
                heading
        );
    }

    public EditablePose flipBySide() {
        return new EditablePose(
                AutonVars.isBackdropSide ? x : X_START_RIGHT + X_START_LEFT - x,
                y,
                AutonVars.isBackdropSide ? heading : Math.PI - heading
        );
    }

    public EditablePose byBoth() {
        return byAlliance().bySide();
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, heading);
    }
}
