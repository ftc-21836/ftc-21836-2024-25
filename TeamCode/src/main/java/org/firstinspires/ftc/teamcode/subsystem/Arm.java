package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Arm {

    public static DiffyAngles
            TRANSFER = new DiffyAngles(30, 30),
            SAMPLE = new DiffyAngles(120, 120),
            INTAKING = new DiffyAngles(0, 0),
            SCORING_SPEC = new DiffyAngles(90, 90);

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

    void run(Deposit.State state) {

        if (state != RETRACTED) timeArmSpentRetracted.reset();

        DiffyAngles angles;

        switch (state) {
            case HAS_SAMPLE:
            case SAMPLE_FALLING:
                angles = SAMPLE;
                break;
            case INTAKING_SPECIMEN:
            case GRABBING_SPECIMEN:
            case RAISING_SPECIMEN:
                angles = INTAKING;
                timeSinceIntakingSpecimen.reset();
                break;
            case HAS_SPECIMEN:
            case SCORING_SPECIMEN:
            case RELEASING_SPECIMEN:
                angles = SCORING_SPEC;
                break;
            case RETRACTED:
            default:
                angles = TRANSFER;
                break;
        }

        rServo.turnToAngle(angles.right);
        lServo.turnToAngle(angles.left);


    }

    public static class DiffyAngles {

        public double right, left;

        DiffyAngles(double right, double left) {
            this.right = right;
            this.left = left;
        }
    }

}
