package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.GRABBING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.HAS_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SAMPLE_FALLING;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Deposit {

    public static double
            ANGLE_CLAW_OPEN = 70,
            ANGLE_CLAW_TRANSFER = 80,
            ANGLE_CLAW_CLOSED = 25,

            TIME_SAMPLE_RELEASE = 0.6,
            TIME_SAMPLE_RELEASE_TO_ARM_DOWN = 0.3,
            TIME_SPEC_GRAB = 0.25,
            TIME_SPEC_RELEASE = 0.1,

            TOLERANCE_ARM_SCORING_POS = 4,

            HEIGHT_ABOVE_INTAKE = 10,
            HEIGHT_OBSERVATION_ZONE = 0.01,
            HEIGHT_BASKET_LOW = 3.5,
            HEIGHT_BASKET_HIGH = 21.5,
            INCREMENT_REACH_ABOVE_BASKET = 1,
            HEIGHT_INTAKING_SPECIMEN = 0,
            HEIGHT_CHAMBER_HIGH = 0,
            HEIGHT_CHAMBER_LOW = HEIGHT_CHAMBER_HIGH,

            TIME_STANDBY_TO_IN_INTAKE = 1,
            TIME_IN_INTAKE_TO_STANDBY = 1,
            TIME_STANDBY_TO_BASKET = 1,
            TIME_STANDBY_TO_INTAKING_SPEC = 1,
            TIME_RAISE_SPEC = 1,
            TIME_RAISED_SPEC_TO_CHAMBER;

    public static ArmPosition
            INTAKING_SPEC = new ArmPosition(0, 0),
            RAISED_SPEC =   new ArmPosition(0, 0),
            CHAMBER =       new ArmPosition(0, 0),
            STANDBY =       new ArmPosition(0, 0),
            IN_INTAKE =     new ArmPosition(0, 0),
            BASKET =        new ArmPosition(0, 0),
            ASCENT =        new ArmPosition(0, 0);

    enum State {
        RETRACTED,
        HAS_SAMPLE,
        SAMPLE_FALLING,
        INTAKING_SPECIMEN,
        GRABBING_SPECIMEN,
        RAISING_SPECIMEN,
        HAS_SPECIMEN,
        RELEASING_SPECIMEN;
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
//    private final CachedSimpleServo claw;
    private final CachedSimpleServo rServo, lServo, wrist;

    private final ElapsedTime timer = new ElapsedTime();

    private Deposit.State state = RETRACTED;

    public static boolean level1Ascent = false;

    private double basketHeight = HEIGHT_BASKET_HIGH;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
//        claw = getAxon(hardwareMap, "claw").reversed();
        rServo = getAxon(hardwareMap, "arm right");
        lServo = getAxon(hardwareMap, "arm left").reversed();
        wrist = getAxon(hardwareMap, "wrist");
    }

    void run(boolean climbing) {

        switch (state) {

            case SAMPLE_FALLING:

                if (timer.seconds() >= TIME_SAMPLE_RELEASE) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case INTAKING_SPECIMEN:

//                if (intake.hasSample()) setPosition(FLOOR);
                break;

            case GRABBING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_GRAB) triggerClaw();
                else break;

            case RELEASING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_RELEASE) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

        }

        boolean reachedTarget = abs(lift.getPosition() - lift.getTarget()) <= TOLERANCE_ARM_SCORING_POS;

//        arm.setTarget(armPosition);

        boolean armCanMove = lift.getPosition() >= HEIGHT_ABOVE_INTAKE;
//        || intake.clearOfDeposit();
//        || !arm.movingNearIntake();

//        arm.run(armCanMove);

        boolean reachedTarget1 = true;
        lift.run(armCanMove || reachedTarget1, climbing);

//        claw.turnToAngle(
//                state == HAS_SAMPLE ?               ANGLE_CLAW_CLOSED :
//                state == SAMPLE_FALLING ?           ANGLE_CLAW_OPEN :
//                state == INTAKING_SPECIMEN ?        ANGLE_CLAW_OPEN :
//                state == GRABBING_SPECIMEN ?        ANGLE_CLAW_CLOSED :
//                state == RAISING_SPECIMEN ?         ANGLE_CLAW_CLOSED :
//                state == HAS_SPECIMEN ?             ANGLE_CLAW_CLOSED :
//                state == RELEASING_SPECIMEN ?       ANGLE_CLAW_OPEN :
//
//                                                    ANGLE_CLAW_TRANSFER
//        );

//        rServo.turnToAngle(state.armPosition.arm);
//        lServo.turnToAngle(state.armPosition.arm);
//        wrist.turnToAngle(state.armPosition.wrist);
    }

    public void preloadSpecimen() {
        state = HAS_SPECIMEN;
        closeClaw();
        lift.setTarget(HEIGHT_CHAMBER_HIGH);
    }

    public void closeClaw() {
//        claw.turnToAngle(ANGLE_CLAW_CLOSED);
    }

    // when does the intake need to move out of the way
    boolean requestingIntakeToMove() {
        return false;
//        return lift.getPosition() < HEIGHT_ABOVE_INTAKE && !arm.reachedTarget() && arm.movingNearIntake();
    }

    boolean readyToTransfer() {
        return false;
//        return state == RETRACTED && arm.atPosition(Arm.TRANSFER) && !lift.isExtended();
    }

    public void setPosition(Position position) {

        switch (state) {

            case INTAKING_SPECIMEN:
                if (position != FLOOR) break;
                state = RETRACTED;
            case RETRACTED:

                if (position == FLOOR) {
                    lift.setTarget(0);
                    break;
                }

            case HAS_SAMPLE:
            case SAMPLE_FALLING:

                lift.setTarget(
                        position == HIGH ?
                                lift.getTarget() >= HEIGHT_BASKET_HIGH ?
                                            lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                            HEIGHT_BASKET_HIGH :
                        position == LOW ?   HEIGHT_BASKET_LOW :
                                            HEIGHT_OBSERVATION_ZONE
                );

                break;

            case GRABBING_SPECIMEN:
                if (position != FLOOR) break;
            case HAS_SPECIMEN:

                if (position == FLOOR) {

                    while (state != INTAKING_SPECIMEN) {
                        triggerClaw();
                    }
                    break;
                }

            case RELEASING_SPECIMEN:

                if (position == FLOOR) {

                    while (state != HAS_SPECIMEN) {
                        triggerClaw();
                    }
                    break;
                }

                lift.setTarget(position == HIGH ? HEIGHT_CHAMBER_HIGH : HEIGHT_CHAMBER_LOW);

                break;
        }

    }

    public void triggerClaw() {
        switch (state) {

            case HAS_SAMPLE:

                basketHeight = lift.getTarget();
                state = SAMPLE_FALLING;
                timer.reset();

                break;

            case RELEASING_SPECIMEN:
            case SAMPLE_FALLING:
            case RETRACTED:

                state = INTAKING_SPECIMEN;
                lift.setTarget(HEIGHT_INTAKING_SPECIMEN);

                break;

            case INTAKING_SPECIMEN:

                state = GRABBING_SPECIMEN;
                timer.reset();

                break;

            case GRABBING_SPECIMEN:

                state = HAS_SPECIMEN;
                setPosition(HIGH);
                timer.reset();

                break;

            case HAS_SPECIMEN:

                state = RELEASING_SPECIMEN;
                timer.reset();

                break;
        }
    }

    public boolean hasSample() {
        return
                state == HAS_SAMPLE ||
                state == GRABBING_SPECIMEN ||
                state == RAISING_SPECIMEN ||
                state == HAS_SPECIMEN;
    }

    public boolean basketReady() {
        return state == HAS_SAMPLE;
    }

    public boolean hasSpecimen() {
        return state == HAS_SPECIMEN;
    }

    public boolean intaking() {
        return state == INTAKING_SPECIMEN;
    }

    public void transfer(Sample sample) {
        if (sample == null || state != RETRACTED) return;
        state = HAS_SAMPLE;
        closeClaw();
    }

    void printTelemetry() {
        String gameElement = state.ordinal() >= INTAKING_SPECIMEN.ordinal() ? "has specimen" : "has sample";
        mTelemetry.addData("DEPOSIT", state + ", " + (hasSample() ? gameElement : "empty"));
        divider();
        lift.printTelemetry();
    }

    public static final class ArmPosition {

        public double arm, wrist;

        private ArmPosition(double arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }
    }
}
