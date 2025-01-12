package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.MainAuton.mTelemetry;
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
            TIME_TRANSFER_TO_INTAKING = 0.65,
            TIME_INTAKING_TO_WRIST_FREE = 0.2,
            TIME_INTAKING_TO_SPEC = 0.65,
            TIME_SPEC_TO_RETRACTED = 0.35;

    public static Arm.Position
            INTAKING =  new Arm.Position(285, 0, "INTAKING"),
            TRANSFER =  new Arm.Position(85, 305, "TRANSFER"),
            POST_INTAKING =  new Arm.Position(355, 75, "POST INTAKING AVOID WALL"),
            SPECIMEN =  new Arm.Position(275, 240, "SPECIMEN"),
            ASCENT = new Arm.Position(285, 250, "ASCENT"),
            SAMPLE =    new Arm.Position(355, 355, "SAMPLE");

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
                target == INTAKING ?        TIME_TRANSFER_TO_INTAKING :
                target == SPECIMEN ?        TIME_INTAKING_TO_SPEC :
                target == SAMPLE ?          TIME_RETRACTED_TO_SAMPLE :
                target == ASCENT ?          0 :
                target == TRANSFER ?
                        lastTarget == INTAKING ?    TIME_TRANSFER_TO_INTAKING :
                        lastTarget == SAMPLE ?      TIME_SAMPLE_TO_RETRACTED :
                                                    TIME_SPEC_TO_RETRACTED :
                1;
    }

    boolean reachedTarget() {
        return getTimeTraveled() >= timeToReachTarget();
    }

    boolean isUnderhand() {
        return target == INTAKING || (lastTarget == INTAKING && getTimeTraveled() < TIME_TRANSFER_TO_INTAKING);
    }

    public boolean atPosition(Position position) {
        return this.target == position && reachedTarget();
    }

    public void setTarget(Arm.Position target) {
        if (this.target == target) return;
        lastTarget = this.target;
        this.target = target;
        movingToTarget = false;
    }

    public void run(boolean canMove) {

        Position setpoint;

        if (canMove || movingToTarget) {

            if (!movingToTarget) {
                timer.reset();
                movingToTarget = true;
            }

            setpoint = this.target;

        } else {

            // We don't want the arm to go back to level 1 ascent position when teleop begins
            // The arm should stay off until intake extends and arm "canMove" to retracted position
            // Returning early so the servos don't get a command = arm stays off/loose
            if (lastTarget == ASCENT) return;

            setpoint = this.lastTarget;
        }

        // Override wrist angle for 0.2ish seconds so specimen doesn't get stuck on wall
        Position adjustedSetpoint = setpoint == SPECIMEN && getTimeTraveled() <= TIME_INTAKING_TO_WRIST_FREE ?
                                Arm.POST_INTAKING :
                                setpoint;

        rServo.turnToAngle(adjustedSetpoint.right);
        lServo.turnToAngle(adjustedSetpoint.left);

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
