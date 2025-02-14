package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.level1Ascent;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Arm {

    public static double
            TIME_RETRACTED_TO_SAMPLE = 0.8,
            TIME_SAMPLE_TO_RETRACTED = 0.4,
            TIME_SAMPLE_TO_IN_BASKET = 0.15,
            TIME_RETRACTED_TO_INTAKING = 0.65,
            TIME_INTAKING_TO_SPEC = 1,
            TIME_SPEC_TO_RETRACTED = 0.4;

    public static Arm.Position
            TRANSFER =      new Arm.Position(25, 97, "TRANSFER"),
            INTAKING =      new Arm.Position(175, 20, "INTAKING"),
            SPECIMEN =      new Arm.Position(240, 220, "SPECIMEN"),
            SPEC_PRELOAD =  new Arm.Position(205, 42, "SPEC PRELOAD"),
            ASCENT =        new Arm.Position(255, 32, "LVL 1 ASCENT"),
            SAMPLE =        new Arm.Position(230, 180, "SAMPLE"),
            SCORING_SAMPLE =new Arm.Position(220, 270, "SCORING SAMPLE");

    private final ElapsedTime timer = new ElapsedTime();
    private boolean movingToTarget = false;
    private double getTimeTraveled() {
        return movingToTarget ? timer.seconds() : 0;
    }

    private Arm.Position target = TRANSFER, lastTarget = level1Ascent ? ASCENT : TRANSFER;
    private final CachedSimpleServo rServo, lServo;

    public Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();
    }

    private double timeToReachTarget() {
        return
                target == lastTarget ?      0 :
                target == SPEC_PRELOAD ?    0 :
                target == ASCENT ?          0 :
                target == INTAKING ?        TIME_RETRACTED_TO_INTAKING :
                target == SPECIMEN ?        TIME_INTAKING_TO_SPEC :
                target == SAMPLE ?          TIME_RETRACTED_TO_SAMPLE :
                target == SCORING_SAMPLE ?
                        lastTarget == SAMPLE ?          TIME_SAMPLE_TO_IN_BASKET :
                                                        TIME_RETRACTED_TO_SAMPLE + TIME_SAMPLE_TO_IN_BASKET :
                target == TRANSFER ?
                        lastTarget == SAMPLE ?          TIME_SAMPLE_TO_RETRACTED :
                        lastTarget == SCORING_SAMPLE ?  TIME_SAMPLE_TO_RETRACTED :
                        lastTarget == INTAKING ?        TIME_RETRACTED_TO_INTAKING :
                                                        TIME_SPEC_TO_RETRACTED :
                1;
    }

    boolean reachedTarget() {
        return getTimeTraveled() >= timeToReachTarget();
    }

    boolean movingNearIntake() {
        return (target == TRANSFER || (lastTarget == TRANSFER && getTimeTraveled() < TIME_RETRACTED_TO_INTAKING)) && target != Arm.SPEC_PRELOAD;
    }

    public boolean atPosition(Position position) {
        return this.target == position && reachedTarget();
    }

    public void setTarget(Arm.Position newTarget) {
        if (newTarget == target) return;
        lastTarget = target;
        target = newTarget;
        movingToTarget = false;
    }

    public void run(boolean canMove) {

        Position setpoint;

        if (canMove || movingToTarget) {

            if (!movingToTarget) {
                timer.reset();
                movingToTarget = true;
            }

            setpoint = target;

        } else {

            // Servos should be off/loose when teleop begins (after level 1 ascent)
            if (lastTarget == ASCENT) return;

            setpoint = lastTarget;
        }

        rServo.turnToAngle(setpoint.right);
        lServo.turnToAngle(setpoint.left);

    }

    public void printTelemetry() {
        mTelemetry.addLine("ARM:");
        mTelemetry.addLine();
        mTelemetry.addLine((reachedTarget() ? "Reached " : "Moving to ") + target.name + " (from " + lastTarget.name + ")");
    }

    public static final class Position {

        public double right, left;
        private final String name;

        private Position(double left, double right, String name) {
            this.right = right;
            this.left = left;
            this.name = name;
        }

        @NonNull
        public String toString() {
            return name + ", Right: " + right + ", Left: " + left;
        }
    }

}
