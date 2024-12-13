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
            INTAKING =  new Position(-210, 210),
            TRANSFER =  new Position(300, 0),
            SPECIMEN =  new Position(150, 250),
            SAMPLE =    new Position(50, 290);

    private final ElapsedTime
            timeArmSpentRetracted = new ElapsedTime(),
            timeSinceUnderhand = new ElapsedTime();

    private final CachedSimpleServo rServo, lServo;

    public Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();
    }

    boolean isExtended() {
        return timeArmSpentRetracted.seconds() <= TIME_RETRACTION;
    }
    boolean isUnderhand() {
        return timeSinceUnderhand.seconds() <= TIME_POST_UNDERHAND;
    }

    public void setPosition(Arm.Position position) {

        if (position != TRANSFER) {
            timeArmSpentRetracted.reset();
            if (position == INTAKING) timeSinceUnderhand.reset();
        }

        rServo.turnToAngle(position.right);
        lServo.turnToAngle(position.left);
    }

    public static final class Position {

        public double right, left;

        private Position(double right, double left) {
            this.right = right;
            this.left = left;
        }
    }

}
