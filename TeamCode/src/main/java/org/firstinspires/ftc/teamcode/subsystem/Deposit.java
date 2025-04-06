package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_OBS_ZONE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.ENTERING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.EXITING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.GRABBING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_INTAKING_SPEC;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_OBS_ZONE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_PRE_OBS;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISED_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SCORING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.STANDBY_TO_SPEC_SCORING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Deposit {

    public static double
            ANGLE_CLAW_OPEN = 165, // 140
            ANGLE_CLAW_MOVING_TO_SPECIMEN = 165,
            ANGLE_CLAW_SAMPLE = 190,
            ANGLE_CLAW_SPECIMEN = 210,

            TIME_SAMPLE_RELEASE = 0.6,
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
            TIME_RAISED_SPEC_TO_CHAMBER = 1;

    public static ArmPosition
            ASCENT =        new ArmPosition(180, 100),
            BASKET =        new ArmPosition(313, 150),
            CHAMBER =       new ArmPosition(150, 80),
            INTAKING_SPEC = new ArmPosition(20, 55),
            IN_INTAKE =     new ArmPosition(100, 75),
            PRE_OBS_ZONE =  new ArmPosition(120, 56),
            RAISED_SPEC =   new ArmPosition(INTAKING_SPEC.arm, 20),
            STANDBY =       new ArmPosition(120, 56);

    enum State {
        STANDBY,

        ENTERING_BUCKET,
        COUNTER_ROLLING,
        TRANSFERRING,
        EXITING_BUCKET,

        MOVING_TO_BASKET,
        AT_BASKET,
        FALLING_BASKET,
        BASKET_TO_STANDBY,

        MOVING_TO_PRE_OBS,
        MOVING_TO_OBS_ZONE,
        AT_OBS_ZONE,
        FALLING_OBS,
        OBS_ZONE_TO_STANDBY,

        MOVING_TO_INTAKING_SPEC,
        INTAKING_SPECIMEN,
        GRABBING_SPECIMEN,
        RAISING_SPECIMEN,
        RAISED_TO_STANDBY,
        STANDBY_TO_SPEC_SCORING,
        SCORING_SPECIMEN,
        RELEASING_SPECIMEN,
        RELEASED_SPEC_TO_STANDBY;

        private static final State[] states = values();

        private State plus(int i) {
            int max = states.length;
            return states[((ordinal() + i) % max + max) % max];
        }
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
    private final CachedSimpleServo claw, wrist, armR, armL;

    private void setArm(double angle) {
        armR.turnToAngle(angle);
        armL.turnToAngle(angle);
    }

    private final ElapsedTime timer = new ElapsedTime();

    Deposit.State state = Deposit.State.STANDBY;

    private double sampleHeight = HEIGHT_BASKET_HIGH, specimenHeight = HEIGHT_CHAMBER_HIGH;

    Deposit(HardwareMap hardwareMap, MecanumDrive dt) {
        lift = new Lift(hardwareMap, dt);
        claw = getAxon(hardwareMap, "claw").reversed();
        armR = getAxon(hardwareMap, "arm right");
        armL = getAxon(hardwareMap, "arm left").reversed();
        wrist = getAxon(hardwareMap, "wrist").reversed();
    }

    void run(Intake intake) {



        lift.run();

//        claw.turnToAngle(
//        );

        armR.turnToAngle(STANDBY.arm);
        armL.turnToAngle(STANDBY.arm);
        wrist.turnToAngle(STANDBY.wrist);
    }

    public void preloadSpecimen() {
        state = SCORING_SPECIMEN;
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
        return state == State.STANDBY && !lift.isExtended();
    }

    public void setPosition(Position position) {

        switch (state) {

            case INTAKING_SPECIMEN:
                if (position != FLOOR) break;
                state = State.STANDBY;
            case STANDBY:

                if (position == FLOOR) {
                    lift.setTarget(0);
                    break;
                }

            case MOVING_TO_BASKET:
            case AT_BASKET:
            case FALLING_BASKET:

                if (position == FLOOR) {

                }

                lift.setTarget(
                        position == HIGH ?
                                lift.getTarget() >= HEIGHT_BASKET_HIGH ?
                                            lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                            HEIGHT_BASKET_HIGH :
                        position == LOW ?
                                lift.getTarget() >= HEIGHT_BASKET_LOW ?
                                        lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                        HEIGHT_BASKET_LOW :
                        0
                );

                break;

            case GRABBING_SPECIMEN:
                if (position != FLOOR) break;
            case SCORING_SPECIMEN:

                if (position == FLOOR) {

                    while (state != INTAKING_SPECIMEN) {
                        nextState();
                    }
                    break;
                }

            case RELEASING_SPECIMEN:

                if (position == FLOOR) {

                    while (state != SCORING_SPECIMEN) {
                        nextState();
                    }
                    break;
                }

                lift.setTarget(position == HIGH ? HEIGHT_CHAMBER_HIGH : HEIGHT_CHAMBER_LOW);

                break;
        }

    }

    public void nextState() {
        switch (state) {
            case STANDBY:

                lift.setTarget(HEIGHT_INTAKING_SPECIMEN);
                state = MOVING_TO_INTAKING_SPEC;
                break;

            case EXITING_BUCKET:

                if (sampleHeight == HEIGHT_OBSERVATION_ZONE) {
                    state = MOVING_TO_PRE_OBS;
                } else {
                    lift.setTarget(sampleHeight);
                    state = state.plus(1);
                }
                break;

            case GRABBING_SPECIMEN:
                lift.setTarget(specimenHeight);
                state = state.plus(1);
                break;

            case FALLING_BASKET:
            case FALLING_OBS:
            case RELEASING_SPECIMEN:
                lift.setTarget(0);
            case ENTERING_BUCKET:
            case COUNTER_ROLLING:
            case TRANSFERRING:
            case MOVING_TO_BASKET:
            case AT_BASKET:
            case MOVING_TO_PRE_OBS:
            case MOVING_TO_OBS_ZONE:
            case AT_OBS_ZONE:
            case MOVING_TO_INTAKING_SPEC:
            case INTAKING_SPECIMEN:
            case RAISING_SPECIMEN:
            case RAISED_TO_STANDBY:
            case STANDBY_TO_SPEC_SCORING:
            case SCORING_SPECIMEN:

                state = state.plus(1);
                break;

            case BASKET_TO_STANDBY:
            case OBS_ZONE_TO_STANDBY:
            case RELEASED_SPEC_TO_STANDBY:

                state = State.STANDBY;
                break;
        }
    }

    public boolean hasSample() {
        return
                state == TRANSFERRING ||
                state == EXITING_BUCKET ||
                state == MOVING_TO_BASKET ||
                state == AT_BASKET ||

                state == MOVING_TO_PRE_OBS ||
                state == MOVING_TO_OBS_ZONE ||
                state == AT_OBS_ZONE ||

                state == GRABBING_SPECIMEN ||
                state == RAISING_SPECIMEN ||
                state == RAISED_TO_STANDBY ||
                state == STANDBY_TO_SPEC_SCORING ||
                state == SCORING_SPECIMEN;
    }

    public boolean basketReady() {
        return state == AT_BASKET;
    }

    public boolean obsReady() {
        return state == AT_OBS_ZONE;
    }

    public boolean chamberReady() {
        return state == SCORING_SPECIMEN;
    }

    public boolean intaking() {
        return state == INTAKING_SPECIMEN;
    }

    public void transfer() {
        if (state != State.STANDBY) return;
        state = ENTERING_BUCKET;
        timer.reset();
    }

    void printTelemetry() {
        String gameElement = state.ordinal() >= MOVING_TO_INTAKING_SPEC.ordinal() ? "has specimen" : "has sample";
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
