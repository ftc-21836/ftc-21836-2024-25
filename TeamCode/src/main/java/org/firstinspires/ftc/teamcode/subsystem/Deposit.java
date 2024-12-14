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
import static org.firstinspires.ftc.teamcode.subsystem.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Deposit {

    public static double
            ANGLE_CLAW_OPEN = 60,
            ANGLE_CLAW_CLOSED = 9,

            TIME_SPEC_RAISE = 0.5,
            TIME_SPEC_RELEASE = 0.5,
            TIME_SAMPLE = 0.5,
            TIME_GRAB = 0.25,

            HEIGHT_ABOVE_INTAKE = 10,
            HEIGHT_INTAKING_SPECIMEN = 7,
            HEIGHT_ARM_SAFE = 3,
            HEIGHT_OBSERVATION_ZONE = 0,
            HEIGHT_BASKET_LOW = 10,
            HEIGHT_BASKET_HIGH = 22,
            HEIGHT_CHAMBER_LOW = 9,
            HEIGHT_CHAMBER_HIGH = 17.5,
            HEIGHT_OFFSET_SPECIMEN_SCORED = 5,
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

    private Sample sample, specimenColor = RED;

    private Deposit.State state = RETRACTED;

    public void setAlliance(boolean redAlliance) {
        specimenColor = redAlliance ? RED : BLUE;
    }

    private double releaseSpecimenHeight = HEIGHT_CHAMBER_LOW + HEIGHT_OFFSET_SPECIMEN_SCORED;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = getGBServo(hardwareMap, "claw").reversed();
        claw.turnToAngle(ANGLE_CLAW_CLOSED);
    }

    void run(boolean intakeHasSample, boolean climbing, boolean intakeClear) {

        // release sample when climbing begins
        if (climbing) state = RETRACTED;
        else switch (state) {

            case SAMPLE_FALLING:

                if (timer.seconds() >= TIME_SAMPLE) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case INTAKING_SPECIMEN:

                if (intakeHasSample) setPosition(FLOOR);
                break;

            case GRABBING_SPECIMEN:

                if (timer.seconds() >= TIME_GRAB) triggerClaw();
                else break;

            case RAISING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_RAISE) triggerClaw();

                break;

            case SCORING_SPECIMEN:

                if (lift.getPosition() >= releaseSpecimenHeight) triggerClaw();
                else break;

            case RELEASING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_RELEASE) triggerClaw();

                break;

        }

        claw.turnToAngle(hasSample() ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_OPEN);

        boolean aboveIntake = lift.getPosition() >= HEIGHT_ABOVE_INTAKE;
        boolean belowSafeHeight = lift.getPosition() < HEIGHT_ARM_SAFE;
        boolean liftLowering = lift.getTarget() < lift.getPosition();

        boolean underhand = state.armPosition == Arm.INTAKING;
        boolean armHitting = belowSafeHeight && underhand;

        boolean armCanMove = !armHitting && (aboveIntake || intakeClear);

        arm.setPosition(armCanMove ? state.armPosition : Arm.TRANSFER);

        boolean crushingArm = belowSafeHeight && liftLowering && arm.isUnderhand();
        boolean liftCanMove = !crushingArm && (aboveIntake || !arm.isExtended() || intakeClear);

        lift.run(liftCanMove, climbing);
    }

    public void preload() {
        Deposit.State endState = hasSample() ? RETRACTED : HAS_SPECIMEN;
        while (state != endState) triggerClaw();
        arm.setPosition(state.armPosition);
        claw.turnToAngle(hasSample() ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_OPEN);
    }

    // when does the intake need to move out of the way
    boolean activeNearIntake() {
        return (lift.getPosition() < HEIGHT_ABOVE_INTAKE || lift.getTarget() < HEIGHT_ABOVE_INTAKE) && (arm.isExtended() || state.armPosition != Arm.TRANSFER);
    }

    boolean readyToTransfer() {
        return state == RETRACTED && !lift.isExtended() && !arm.isExtended();
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
                        position == HIGH ?  HEIGHT_BASKET_HIGH :
                        position == LOW ?   HEIGHT_BASKET_LOW :
                                            HEIGHT_OBSERVATION_ZONE
                );

                break;

            case SCORING_SPECIMEN:
                if (position == FLOOR) break;
                state = HAS_SPECIMEN;
            case GRABBING_SPECIMEN:
            case RAISING_SPECIMEN:
            case HAS_SPECIMEN:

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

    public void transfer(Sample sample) {
        if (sample == null || hasSample() || state != RETRACTED) return;
        this.sample = sample;
        state = HAS_SAMPLE;
        setPosition(FLOOR);
    }

    void printTelemetry() {
        String gameElement = sample + (state.ordinal() >= INTAKING_SPECIMEN.ordinal() ? " specimen" : " sample");
        mTelemetry.addData("DEPOSIT", state + ", " + (hasSample() ? gameElement : "empty"));
        divider();
        lift.printTelemetry();
    }

}
