package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_OBS_ZONE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.BASKET_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.COUNTER_ROLLING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.ENTERING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.EXITING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.FALLING_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.FALLING_OBS;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.GRABBING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_INTAKING_SPEC;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_OBS_ZONE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.MOVING_TO_PRE_OBS;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.OBS_ZONE_TO_PRE_OBS;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.OBS_ZONE_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISED_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASED_SPEC_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.STANDBY_TO_CHAMBER;
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
            ANGLE_CLAW_OPEN = 140,
            ANGLE_CLAW_MOVING_TO_SPECIMEN = 165,
            ANGLE_CLAW_SAMPLE = 190,
            ANGLE_CLAW_SPECIMEN = 210,

            TOLERANCE_ARM_SCORING_POS = 4,

            HEIGHT_ABOVE_INTAKE = 10,
            HEIGHT_OBSERVATION_ZONE = 0,
            HEIGHT_BASKET_LOW = 3.5,
            HEIGHT_BASKET_HIGH = 21.5,
            INCREMENT_REACH_ABOVE_BASKET = 1,
            HEIGHT_INTAKING_SPECIMEN = 0,

            HEIGHT_CHAMBER_HIGH = 0,
            HEIGHT_CHAMBER_LOW = HEIGHT_CHAMBER_HIGH,

            TIME_ENTERING_BUCKET = 1,
            TIME_COUNTER_ROLLING = 1,
            TIME_TRANSFERRING = 1,
            TIME_EXITING_BUCKET = 1,
            TIME_OBS_ZONE_TO_PRE_OBS = 1,
            TIME_TO_BASKET = 1,
            TIME_SAMPLE_RELEASE = 1,
            TIME_BASKET_TO_STANDBY = 1,
            TIME_TO_PRE_OBS = 1,
            TIME_TO_OBS_ZONE = 1,
            TIME_OBS_ZONE_TO_STANDBY = 1,
            TIME_TO_INTAKING_SPEC = 1,
            TIME_SPEC_GRAB = 1,
            TIME_RAISE_SPEC = 1,
            TIME_RAISED_SPEC_TO_STANDBY = 1,
            TIME_STANDBY_TO_CHAMBER = 1,
            TIME_SPEC_RELEASE = 1,
            TIME_RELEASED_SPEC_TO_STANDBY = 1;

    public static ArmPosition
            ASCENT =        new ArmPosition(180, 100),
            BASKET =        new ArmPosition(313, 150),
            CHAMBER =       new ArmPosition(150, 80),
            INTAKING_SPEC = new ArmPosition(18, 55),
            IN_INTAKE =     new ArmPosition(100, 75),
            PRE_OBS_ZONE =  new ArmPosition(120, 56),
            RAISED_SPEC =   new ArmPosition(INTAKING_SPEC.arm, 20),
            STANDBY =       new ArmPosition(120, 35);

    enum State {
        STANDBY         (Deposit.STANDBY),

        ENTERING_BUCKET (IN_INTAKE),
        COUNTER_ROLLING (IN_INTAKE),
        TRANSFERRING    (IN_INTAKE),
        EXITING_BUCKET  (Deposit.STANDBY),

        OBS_ZONE_TO_PRE_OBS (PRE_OBS_ZONE),

        MOVING_TO_BASKET    (BASKET),
        AT_BASKET           (BASKET),
        FALLING_BASKET      (BASKET),
        BASKET_TO_STANDBY   (Deposit.STANDBY),

        MOVING_TO_PRE_OBS   (PRE_OBS_ZONE),
        MOVING_TO_OBS_ZONE  (INTAKING_SPEC),
        AT_OBS_ZONE         (INTAKING_SPEC),
        FALLING_OBS         (INTAKING_SPEC),
        OBS_ZONE_TO_STANDBY (Deposit.STANDBY),

        MOVING_TO_INTAKING_SPEC (INTAKING_SPEC),
        INTAKING_SPECIMEN       (INTAKING_SPEC),
        GRABBING_SPECIMEN       (INTAKING_SPEC),
        RAISING_SPECIMEN        (RAISED_SPEC),
        RAISED_TO_STANDBY       (Deposit.STANDBY),
        STANDBY_TO_CHAMBER      (CHAMBER),
        AT_CHAMBER              (CHAMBER),
        RELEASING_SPECIMEN      (CHAMBER),
        RELEASED_SPEC_TO_STANDBY(Deposit.STANDBY);

        private final ArmPosition armPosition;

        State(ArmPosition armPosition) {
            this.armPosition = armPosition;
        }

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
    public final CachedSimpleServo claw;
    private final CachedSimpleServo wrist, armR, armL;

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

        switch (state) {
            case ENTERING_BUCKET:
                if (timer.seconds() >= TIME_ENTERING_BUCKET) nextState();
                break;
            case COUNTER_ROLLING:
                if (timer.seconds() >= TIME_COUNTER_ROLLING) nextState();
                break;
            case TRANSFERRING:
                if (timer.seconds() >= TIME_TRANSFERRING) nextState();
                break;
            case EXITING_BUCKET:
                if (timer.seconds() >= TIME_EXITING_BUCKET) nextState();
                break;
            case OBS_ZONE_TO_PRE_OBS:
                if (timer.seconds() >= TIME_OBS_ZONE_TO_PRE_OBS) nextState();
                break;
            case MOVING_TO_BASKET:
                if (timer.seconds() >= TIME_TO_BASKET) nextState();
                break;
            case FALLING_BASKET:
            case FALLING_OBS:
                if (timer.seconds() >= TIME_SAMPLE_RELEASE) nextState();
                break;
            case BASKET_TO_STANDBY:
                if (timer.seconds() >= TIME_BASKET_TO_STANDBY) nextState();
                break;
            case MOVING_TO_PRE_OBS:
                if (timer.seconds() >= TIME_TO_PRE_OBS) nextState();
                break;
            case MOVING_TO_OBS_ZONE:
                if (timer.seconds() >= TIME_TO_OBS_ZONE) nextState();
                break;
            case OBS_ZONE_TO_STANDBY:
                if (timer.seconds() >= TIME_OBS_ZONE_TO_STANDBY) nextState();
                break;
            case MOVING_TO_INTAKING_SPEC:
                if (timer.seconds() >= TIME_TO_INTAKING_SPEC) nextState();
                break;
            case GRABBING_SPECIMEN:
                if (timer.seconds() >= TIME_SPEC_GRAB) nextState();
                break;
            case RAISING_SPECIMEN:
                if (timer.seconds() >= TIME_RAISE_SPEC) nextState();
                break;
            case RAISED_TO_STANDBY:
                if (timer.seconds() >= TIME_RAISED_SPEC_TO_STANDBY) nextState();
                break;
            case STANDBY_TO_CHAMBER:
                if (timer.seconds() >= TIME_STANDBY_TO_CHAMBER) nextState();
                break;
            case RELEASING_SPECIMEN:
                if (timer.seconds() >= TIME_SPEC_RELEASE) nextState();
                break;
            case RELEASED_SPEC_TO_STANDBY:
                if (timer.seconds() >= TIME_RELEASED_SPEC_TO_STANDBY) nextState();
                break;
        }

        lift.run();

        armR.turnToAngle(state.armPosition.arm);
        armL.turnToAngle(state.armPosition.arm);
        wrist.turnToAngle(state.armPosition.wrist);

        claw.turnToAngle(
                state == State.STANDBY ?    ANGLE_CLAW_OPEN :

                state == ENTERING_BUCKET ?  ANGLE_CLAW_OPEN :
                state == COUNTER_ROLLING ?  ANGLE_CLAW_OPEN :
                state == TRANSFERRING ?     ANGLE_CLAW_SAMPLE :
                state == EXITING_BUCKET ?   ANGLE_CLAW_SAMPLE:

                state == MOVING_TO_BASKET ?     ANGLE_CLAW_SAMPLE :
                state == AT_BASKET ?            ANGLE_CLAW_SAMPLE :
                state == FALLING_BASKET ?       ANGLE_CLAW_OPEN :
                state == BASKET_TO_STANDBY ?    ANGLE_CLAW_OPEN :

                state == MOVING_TO_PRE_OBS ?    ANGLE_CLAW_SAMPLE :
                state == MOVING_TO_OBS_ZONE ?   ANGLE_CLAW_SAMPLE :
                state == AT_OBS_ZONE ?          ANGLE_CLAW_SAMPLE :
                state == FALLING_OBS ?          ANGLE_CLAW_OPEN :
                state == OBS_ZONE_TO_STANDBY ?  ANGLE_CLAW_MOVING_TO_SPECIMEN :

                state == MOVING_TO_INTAKING_SPEC ?  ANGLE_CLAW_MOVING_TO_SPECIMEN :
                state == INTAKING_SPECIMEN ?        ANGLE_CLAW_OPEN :
                state == GRABBING_SPECIMEN ?        ANGLE_CLAW_SPECIMEN :
                state == RAISING_SPECIMEN ?         ANGLE_CLAW_SPECIMEN :
                state == RAISED_TO_STANDBY ?        ANGLE_CLAW_SPECIMEN :
                state == STANDBY_TO_CHAMBER ?       ANGLE_CLAW_SPECIMEN :
                state == AT_CHAMBER ?               ANGLE_CLAW_SPECIMEN :
                state == RELEASING_SPECIMEN ?       ANGLE_CLAW_OPEN :
                state == RELEASED_SPEC_TO_STANDBY ? ANGLE_CLAW_OPEN :

                ANGLE_CLAW_MOVING_TO_SPECIMEN
        );
    }

    public void preloadSpecimen() {
        state = AT_CHAMBER;
        claw.turnToAngle(ANGLE_CLAW_SPECIMEN);
        lift.setTarget(specimenHeight);
    }

    // when does the intake need to move out of the way
    boolean requestingIntakeToMove() {
        return lift.getPosition() < HEIGHT_ABOVE_INTAKE && (
                        state == MOVING_TO_BASKET ||
                        state == MOVING_TO_PRE_OBS ||
                        state == MOVING_TO_OBS_ZONE ||
                        state == MOVING_TO_INTAKING_SPEC ||
                        state == RAISED_TO_STANDBY ||
                        state == STANDBY_TO_CHAMBER
        );
    }

    boolean readyToTransfer() {
        return state == State.STANDBY && !lift.isExtended();
    }

    public void setPosition(Position position) {

        switch (state) {

            case ENTERING_BUCKET:
            case COUNTER_ROLLING:
            case TRANSFERRING:
            case EXITING_BUCKET:
                break;

            case OBS_ZONE_TO_PRE_OBS:
            case MOVING_TO_BASKET:
            case AT_BASKET:
                if (position == FLOOR) {
                    state = MOVING_TO_PRE_OBS;
                    lift.setTarget(0);
                    break;
                }
            case STANDBY:
            case FALLING_OBS:
            case OBS_ZONE_TO_STANDBY:
            case RELEASED_SPEC_TO_STANDBY:
            case FALLING_BASKET:
            case BASKET_TO_STANDBY:
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

            case MOVING_TO_PRE_OBS:
            case MOVING_TO_OBS_ZONE:
            case AT_OBS_ZONE:

                if (position != FLOOR) {
                    lift.setTarget(sampleHeight);
                    state = OBS_ZONE_TO_PRE_OBS;
                }
                break;

            case MOVING_TO_INTAKING_SPEC:
            case INTAKING_SPECIMEN:
                if (position != FLOOR) break;
                state = State.STANDBY;
                lift.setTarget(0);
                break;

            case GRABBING_SPECIMEN:
                if (position != FLOOR) break;
            case RAISING_SPECIMEN:
            case RAISED_TO_STANDBY:
            case STANDBY_TO_CHAMBER:
            case AT_CHAMBER:

                if (position == FLOOR) {
                    while (state != MOVING_TO_INTAKING_SPEC) nextState();
                    break;
                }

            case RELEASING_SPECIMEN:

                if (position == FLOOR) {
                    while (state != AT_CHAMBER) nextState();
                    break;
                }

                lift.setTarget(
                        position == HIGH ?
                                lift.getTarget() >= HEIGHT_CHAMBER_HIGH ?
                                        lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                        HEIGHT_CHAMBER_HIGH :
                                lift.getTarget() >= HEIGHT_CHAMBER_LOW ?
                                        lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                        HEIGHT_CHAMBER_LOW
                );

                break;
        }

    }

    public void nextState() {
        timer.reset();
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
                    state = MOVING_TO_BASKET;
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
            case OBS_ZONE_TO_PRE_OBS:
            case MOVING_TO_BASKET:
            case AT_BASKET:
            case MOVING_TO_PRE_OBS:
            case MOVING_TO_OBS_ZONE:
            case AT_OBS_ZONE:
            case MOVING_TO_INTAKING_SPEC:
            case INTAKING_SPECIMEN:
            case RAISING_SPECIMEN:
            case RAISED_TO_STANDBY:
            case STANDBY_TO_CHAMBER:
            case AT_CHAMBER:

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
                state == STANDBY_TO_CHAMBER ||
                state == AT_CHAMBER;
    }

    public boolean basketReady() {
        return state == AT_BASKET;
    }

    public boolean obsReady() {
        return state == AT_OBS_ZONE;
    }

    public boolean chamberReady() {
        return state == AT_CHAMBER;
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
