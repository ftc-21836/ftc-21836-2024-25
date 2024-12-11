package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Arm {

    public static Position
            TRANSFER = new Position(30, 30),
            SAMPLE = new Position(120, 120),
            INTAKING = new Position(0, 0),
            SCORING_SPEC = new Position(90, 90);

    public static double
            TIME_RETRACTION = 0.25,
            TIME_POST_INTAKING = 0.5;

    private final ElapsedTime
            timeArmSpentRetracted = new ElapsedTime(),
            timeSinceIntakingSpecimen = new ElapsedTime();

    private final CachedSimpleServo rServo, lServo;

    Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();
    }

    boolean isExtended() {
        return timeArmSpentRetracted.seconds() <= TIME_RETRACTION;
    }
    boolean atSpecimenAngle() {
        return timeSinceIntakingSpecimen.seconds() <= TIME_POST_INTAKING;
    }

    void setPosition(Position position) {

        if (position != TRANSFER) {
            timeArmSpentRetracted.reset();
            if (position == INTAKING) timeSinceIntakingSpecimen.reset();
        }

        rServo.turnToAngle(position.right);
        lServo.turnToAngle(position.left);
    }

    public static class Position {

        public double right, left;

        Position(double right, double left) {
            this.right = right;
            this.left = left;
        }
    }

}
