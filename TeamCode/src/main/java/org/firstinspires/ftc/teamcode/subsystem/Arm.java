package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Arm {

    public static double
            TIME_RETRACTION = 0.75,
            TIME_POST_UNDERHAND = 0.75;

    public static Position
            INTAKING = new Position(0, 210),
            TRANSFER = new Position(150, -150),
            SPECIMEN = new Position(200, 50),
            SAMPLE = new Position(170, 120);

    private final ElapsedTime
            timeArmSpentRetracted = new ElapsedTime(),
            timeSinceUnderhand = new ElapsedTime();

    private final CachedSimpleServo rServo, lServo;

    Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();
    }

    boolean isExtended() {
        return timeArmSpentRetracted.seconds() <= TIME_RETRACTION;
    }
    boolean isUnderhand() {
        return timeSinceUnderhand.seconds() <= TIME_POST_UNDERHAND;
    }

    void setPosition(Arm.Position position) {

        if (position != TRANSFER) {
            timeArmSpentRetracted.reset();
            if (position == INTAKING) timeSinceUnderhand.reset();
        }

        rServo.turnToAngle(position.rightAngle());
        lServo.turnToAngle(position.leftAngle());
    }

    public static final class Position {

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
