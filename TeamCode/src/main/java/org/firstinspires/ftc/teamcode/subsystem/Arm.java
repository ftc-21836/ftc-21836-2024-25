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
            TIME_STANDBY_TO_TRANSFER = 1;

    public static Arm.Position
            INTAKING =      new Arm.Position(0, 0, "INTAKING"),
            INTAKED =       new Arm.Position(0, 0, "INTAKED"),
            TRANSFER =      new Arm.Position(0, 0, "TRANSFER"),
            STANDBY =       new Arm.Position(0, 0, "STANDBY"),
            SPECIMEN =      new Arm.Position(0, 0, "SPECIMEN"),
            ASCENT =        new Arm.Position(0, 0, "LVL 1 ASCENT"),
            SAMPLE =        new Arm.Position(0, 0, "SAMPLE");

    private final ElapsedTime timer = new ElapsedTime();
    private boolean movingToTarget = false;
    private double getTimeTraveled() {
        return movingToTarget ? timer.seconds() : 0;
    }

    private Arm.Position target = TRANSFER, lastTarget = level1Ascent ? ASCENT : TRANSFER;
    private final CachedSimpleServo rServo, lServo, wrist;

    public Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();
        wrist = getAxon(hardwareMap, "wrist");
    }

    private double timeToReachTarget() {
        return 1;
    }

    boolean reachedTarget() {
        return getTimeTraveled() >= timeToReachTarget();
    }

    boolean movingNearIntake() {
        return target == TRANSFER || (lastTarget == TRANSFER && getTimeTraveled() < 0.75);
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

        rServo.turnToAngle(setpoint.arm);
        lServo.turnToAngle(setpoint.arm);
        wrist.turnToAngle(setpoint.wrist);
    }

    public void printTelemetry() {
        mTelemetry.addLine("ARM:");
        mTelemetry.addLine();
        mTelemetry.addLine((reachedTarget() ? "Reached " : "Moving to ") + target.name + " (from " + lastTarget.name + ")");
    }

    public static final class Position {

        public double arm, wrist;
        private final String name;

        private Position(double arm, double wrist, String name) {
            this.arm = arm;
            this.wrist = wrist;
            this.name = name;
        }

        @NonNull
        public String toString() {
            return name + ", Arm: " + arm + ", Wrist: " + wrist;
        }
    }

}
