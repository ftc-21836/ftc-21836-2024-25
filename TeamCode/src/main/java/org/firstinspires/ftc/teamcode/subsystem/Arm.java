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
            TIME_RETRACTED_TO_SAMPLE = 1,
            TIME_RETRACTED_TO_INTAKING = 1,
            TIME_INTAKING_TO_SPEC = 1,
            TIME_SPEC_TO_SCORED = 1,
            TIME_SCORING_SPEC_TO_RETRACTED = 1;

    public static Position
            INTAKING =  new Position(285, 0, "INTAKING"),
            TRANSFER =  new Position(85, 305, "TRANSFER"),
            POST_SPEC_INTAKE =  new Position(355, 90, "POST SPEC INTAKE"),
            SPECIMEN =  new Position(285, 215, "SPECIMEN"),
            SCORING_SPEC = new Position(285, 215, "SCORING SPEC"),
            SAMPLE =    new Position(355, 355, "SAMPLE");

    private final ElapsedTime timer = new ElapsedTime();

    private Position position = TRANSFER, lastPosition = TRANSFER;
    private final CachedSimpleServo rServo, lServo;

    public Arm(HardwareMap hardwareMap) {
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();

        setPosition(new Position(TRANSFER.left + 1, TRANSFER.right - 1, "POOPOO"));
    }

    private double timeToReachPosition() {

        if (position == TRANSFER) {

            if (lastPosition == INTAKING) return TIME_RETRACTED_TO_INTAKING;
            if (lastPosition == SAMPLE) return TIME_RETRACTED_TO_SAMPLE;
            return TIME_SCORING_SPEC_TO_RETRACTED;

        }

        if (position == INTAKING)                                   return TIME_RETRACTED_TO_INTAKING;
        if (position == POST_SPEC_INTAKE || position == SPECIMEN)   return TIME_INTAKING_TO_SPEC;
        if (position == SCORING_SPEC)                               return TIME_SPEC_TO_SCORED;
        if (position == SAMPLE)                                     return TIME_RETRACTED_TO_SAMPLE;
        return 1;
    }

    boolean reachedPosition() {
        return timer.seconds() >= timeToReachPosition();
    }

    boolean isUnderhand() {
        return position == INTAKING || (lastPosition == INTAKING && timer.seconds() < TIME_RETRACTED_TO_INTAKING);
    }

    boolean atPosition(Position position) {
        return this.position == position && reachedPosition();
    }

    public void setPosition(Arm.Position position) {

        this.position = position;

        if (position != lastPosition) timer.reset();

        lastPosition = position;

        rServo.turnToAngle(position.right);
        lServo.turnToAngle(position.left);
    }

    boolean collidingWithIntake() {
        return !reachedPosition() || position == POST_SPEC_INTAKE || position == SPECIMEN || position == SCORING_SPEC;
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM", position.name);
        mTelemetry.addLine();
        mTelemetry.addLine((reachedPosition() ? "Reached " : "Moving to ") + position.name + " (from " + lastPosition.name + ")");
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
