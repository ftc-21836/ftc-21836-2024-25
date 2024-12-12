package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Arm {

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

    void setPosition(Arm.Position position) {

        if (position != Arm.Position.TRANSFER) {
            timeArmSpentRetracted.reset();
            if (position == Arm.Position.INTAKING) timeSinceIntakingSpecimen.reset();
        }

        rServo.turnToAngle(position.rightAngle());
        lServo.turnToAngle(position.leftAngle());
    }

    @Config
    public static class Position {

        public static Position
                INTAKING = new Position(50, 0),
                OBS_ZONE = new Position(50, 0),
                TRANSFER = new Position(80, 30),
                SPECIMEN = new Position(140, 90),
                SCORING_SPEC = new Position(140, 90),
                SAMPLE = new Position(170, 120);

        public double armAngle, wristAngle;

        private Position(double armAngle, double wristAngle) {
            this.armAngle = armAngle;
            this.wristAngle = wristAngle;
        }

        private double rightAngle() {
            return armAngle - wristAngle;
        }

        private double leftAngle() {
            return armAngle + wristAngle;
        }
    }

}
