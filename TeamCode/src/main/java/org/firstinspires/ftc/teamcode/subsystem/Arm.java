package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Arm {

    public static double
            TIME_RETRACTED_TO_SAMPLE = 0.85,
            TIME_RETRACTED_TO_INTAKING = 0.65,
            TIME_INTAKING_TO_WRIST_FREE = 0.2,
            TIME_INTAKING_TO_SPEC = 0.65,
            TIME_SPEC_TO_SCORED = 1,
            TIME_SCORING_SPEC_TO_RETRACTED = 0.35;

    public static Arm.Position
            INTAKING =  new Arm.Position(285, 0, "INTAKING"),
            TRANSFER =  new Arm.Position(85, 305, "TRANSFER"),
            POST_INTAKING =  new Arm.Position(355, 75, "POST INTAKING AVOID WALL"),
            SPECIMEN =  new Arm.Position(285, 215, "SPECIMEN"),
            SCORING_SPEC = new Arm.Position(285, 215, "SCORING SPEC"),
            SAMPLE =    new Arm.Position(355, 355, "SAMPLE");

    private final ElapsedTime timer = new ElapsedTime();

    private Arm.Position target = TRANSFER, lastTarget = TRANSFER;
    private final CachedSimpleServo rServo, lServo;

    public Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();

        setTarget(new Position(TRANSFER.left + 1, TRANSFER.right - 1, "POOPOO"));
        run();
    }

    private double timeToReachTarget() {
        return
                target == INTAKING ?        TIME_RETRACTED_TO_INTAKING :
                target == SPECIMEN ?        TIME_INTAKING_TO_SPEC :
                target == SCORING_SPEC ?    TIME_SPEC_TO_SCORED :
                target == SAMPLE ?          TIME_RETRACTED_TO_SAMPLE :
                target == TRANSFER ?
                        lastTarget == INTAKING ?    TIME_RETRACTED_TO_INTAKING :
                        lastTarget == SAMPLE ?      TIME_RETRACTED_TO_SAMPLE :
                                                    TIME_SCORING_SPEC_TO_RETRACTED :
                1;
    }

    boolean reachedTarget() {
        return timer.seconds() >= timeToReachTarget();
    }

    boolean isUnderhand() {
        return target == INTAKING || (lastTarget == INTAKING && timer.seconds() < TIME_RETRACTED_TO_INTAKING);
    }

    boolean atPosition(Position position) {
        return this.target == position && reachedTarget();
    }

    public void setTarget(Arm.Position target) {
        this.target = target;
        if (target != lastTarget) timer.reset();
        lastTarget = target;
    }

    public void run() {

        Position target = this.target == SPECIMEN && timer.seconds() <= TIME_INTAKING_TO_WRIST_FREE ?
                                Arm.POST_INTAKING :
                                this.target;

        rServo.turnToAngle(target.right);
        lServo.turnToAngle(target.left);
    }

    boolean collidingWithIntake() {
        return !reachedTarget() || target == SPECIMEN || target == SCORING_SPEC;
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM", target.name);
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
