package org.firstinspires.ftc.teamcode.subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
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
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SCORING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Deposit {

    public static double
            ANGLE_CLAW_OPEN = 70,
            ANGLE_CLAW_TRANSFER = 60,
            ANGLE_CLAW_TRANSFERRED = 30,
            ANGLE_CLAW_CLOSED = 25,

            TIME_SAMPLE_RELEASE = 0.5,
            TIME_SPEC_GRAB = 0.25,
            TIME_SPEC_RELEASE = 0.5,

            HEIGHT_ABOVE_INTAKE = 10,
            HEIGHT_ARM_SAFE = 7,
            HEIGHT_OBSERVATION_ZONE = 7,
            HEIGHT_BASKET_LOW = 0,
            HEIGHT_BASKET_HIGH = 18,
            HEIGHT_INTAKING_SPECIMEN = 7,
            HEIGHT_OFFSET_SPECIMEN_INTAKED = 2,
            HEIGHT_CHAMBER_HIGH = 9,
            HEIGHT_CHAMBER_LOW = HEIGHT_CHAMBER_HIGH,
            HEIGHT_OFFSET_SPECIMEN_SCORED = 8,
            HEIGHT_OFFSET_SPECIMEN_SCORING = 10;

    enum State {
        RETRACTED           (Arm.TRANSFER),
        HAS_SAMPLE          (Arm.SAMPLE),
        SAMPLE_FALLING      (Arm.SAMPLE),
        INTAKING_SPECIMEN   (Arm.INTAKING),
        GRABBING_SPECIMEN   (Arm.INTAKING),
        RAISING_SPECIMEN    (Arm.INTAKING),
        HAS_SPECIMEN        (Arm.SPECIMEN),
        SCORING_SPECIMEN    (Arm.SPECIMEN),
        RELEASING_SPECIMEN  (Arm.SPECIMEN);

        private final Arm.Position armPosition;

        State(Arm.Position armPosition) {
            this.armPosition = armPosition;
        }
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
    private final Arm arm;
    private final CachedSimpleServo claw;

    private final ElapsedTime timer = new ElapsedTime();

    private Sample sample, specimenColor = NEUTRAL;

    private Deposit.State state = RETRACTED;

    public void setAlliance(boolean redAlliance) {
        specimenColor = redAlliance ? RED : BLUE;
    }

    private double releaseSpecimenHeight = HEIGHT_CHAMBER_LOW + HEIGHT_OFFSET_SPECIMEN_SCORED;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = getGBServo(hardwareMap, "claw").reversed();
        claw.turnToAngle(ANGLE_CLAW_TRANSFER + 1);

        if (level1Ascent) {
            arm.postAscent();
            level1Ascent = false;
        }
    }

    void run(boolean intakeHasSample, boolean climbing, boolean intakeClear) {

        // home arm when climbing begins
        if (climbing) state = RETRACTED;
        else switch (state) {

            case SAMPLE_FALLING:

                if (timer.seconds() >= TIME_SAMPLE_RELEASE) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case INTAKING_SPECIMEN:

                if (intakeHasSample) setPosition(FLOOR);
                break;

            case GRABBING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_GRAB) triggerClaw();
                else break;

            case RAISING_SPECIMEN:

                if (lift.getPosition() >= HEIGHT_INTAKING_SPECIMEN + HEIGHT_OFFSET_SPECIMEN_INTAKED) triggerClaw();

                break;

            case SCORING_SPECIMEN:

                if (lift.getPosition() >= releaseSpecimenHeight) triggerClaw();
                else break;

            case RELEASING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_RELEASE) triggerClaw();

                break;

        }

        boolean aboveIntake = lift.getPosition() >= HEIGHT_ABOVE_INTAKE;
        boolean belowSafeHeight = lift.getPosition() < HEIGHT_ARM_SAFE;
        boolean liftLowering = lift.getTarget() < lift.getPosition();

        boolean obsZone = state.armPosition == Arm.SAMPLE && lift.getTarget() == HEIGHT_OBSERVATION_ZONE;
        Arm.Position armPosition = level1Ascent ? Arm.ASCENT : obsZone ? Arm.INTAKING : state.armPosition;

        boolean movingToUnderhand = armPosition == Arm.INTAKING;
        boolean armWouldHitDrivetrain = belowSafeHeight && movingToUnderhand;

        boolean armCanMove = !armWouldHitDrivetrain && (aboveIntake || intakeClear);

        arm.setTarget(armPosition);
        if (armCanMove) arm.run();

        boolean crushingArm = belowSafeHeight && liftLowering && arm.isUnderhand();
        boolean liftCanMove = !crushingArm && (aboveIntake || !arm.collidingWithIntake() || intakeClear);

        lift.run(liftCanMove, climbing);

        boolean intakePullingAwayPostTransfer = state == HAS_SAMPLE && arm.atPosition(Arm.TRANSFER) && !lift.isExtended() && !intakeClear;
        claw.turnToAngle(
                hasSample() ?
                        intakePullingAwayPostTransfer ?
                                ANGLE_CLAW_TRANSFERRED :
                                ANGLE_CLAW_CLOSED :
                state == RETRACTED ?
                        ANGLE_CLAW_TRANSFER :
                        ANGLE_CLAW_OPEN
        );
    }

    private static boolean level1Ascent = false;

    public void level1Ascent() {
        level1Ascent = true;
        lift.setTarget(0);
    }

    public void preloadSpecimen() {
        Deposit.State endState = hasSample() ? RETRACTED : HAS_SPECIMEN;
        while (state != endState) triggerClaw();
        claw.turnToAngle(hasSample() ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_TRANSFER);
    }

    // when does the intake need to move out of the way
    boolean requestingIntakeToMove() {
        return lift.getPosition() < HEIGHT_ABOVE_INTAKE && !arm.atPosition(Arm.TRANSFER) && !arm.reachedTarget();
    }

    boolean readyToTransfer() {
        return state == RETRACTED && arm.atPosition(Arm.TRANSFER) && !lift.isExtended();
    }

    public boolean reachedTarget(Arm.Position position, double liftTarget) {
        return arm.atPosition(position) && abs(liftTarget - lift.getPosition()) < Lift.HEIGHT_RETRACTED_THRESHOLD;
    }

    public void setPosition(Position position) {

        switch (state) {

            case RETRACTED:

                if (position == FLOOR) {
                    lift.setTarget(0);
                    break;
                }

            case HAS_SAMPLE:
            case SAMPLE_FALLING:

                lift.setTarget(
                        position == HIGH ?  HEIGHT_BASKET_HIGH :
                        position == LOW ?   HEIGHT_BASKET_LOW :
                                            HEIGHT_OBSERVATION_ZONE
                );

                break;

            case INTAKING_SPECIMEN:
                if (position != FLOOR) break;
            case GRABBING_SPECIMEN:
            case RAISING_SPECIMEN:
            case HAS_SPECIMEN:
            case SCORING_SPECIMEN:
            case RELEASING_SPECIMEN:

                if (position == FLOOR) {

                    while (state != RETRACTED) {
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

                sample = null;
                state = SAMPLE_FALLING;
                timer.reset();

                break;

            case SAMPLE_FALLING:
            case RETRACTED:

                state = INTAKING_SPECIMEN;
                lift.setTarget(HEIGHT_INTAKING_SPECIMEN);

                break;

            case INTAKING_SPECIMEN:

                sample = specimenColor;
                state = GRABBING_SPECIMEN;
                timer.reset();

                break;

            case GRABBING_SPECIMEN:

                state = RAISING_SPECIMEN;
                setPosition(LOW);
                timer.reset();

                break;

            case RAISING_SPECIMEN:

                state = HAS_SPECIMEN;
                timer.reset();

                break;

            case HAS_SPECIMEN:

                state = SCORING_SPECIMEN;

                double position = lift.getPosition();
                releaseSpecimenHeight = clip(position + HEIGHT_OFFSET_SPECIMEN_SCORED, 0, 32);
                lift.setTarget(position + HEIGHT_OFFSET_SPECIMEN_SCORING);

                break;

            case SCORING_SPECIMEN:

                sample = null;
                state = RELEASING_SPECIMEN;
                timer.reset();

                break;

            case RELEASING_SPECIMEN:

                state = RETRACTED;
                setPosition(FLOOR);

                break;
        }
    }

    public boolean hasSample() {
        return sample != null;
    }

    public boolean specimenIntaked() {
        return state == HAS_SPECIMEN;
    }

    Sample getSample() {
        return sample;
    }

    public void transfer(Sample sample) {
        if (sample == null || hasSample() || state != RETRACTED) return;
        this.sample = sample;
        state = HAS_SAMPLE;
        setPosition(LOW);
        claw.turnToAngle(ANGLE_CLAW_TRANSFERRED);
    }

    void printTelemetry() {
        String gameElement = sample + (state.ordinal() >= INTAKING_SPECIMEN.ordinal() ? " specimen" : " sample");
        mTelemetry.addData("DEPOSIT", state + ", " + (hasSample() ? gameElement : "empty"));
        divider();
        arm.printTelemetry();
        divider();
        lift.printTelemetry();
    }

}
