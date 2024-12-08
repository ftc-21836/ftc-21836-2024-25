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
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SAMPLE_FALLING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SCORING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;

@Config
public final class Deposit {

    public static double
            ANGLE_ARM_RETRACTED = 3,
            ANGLE_ARM_SPECIMEN = 110, // wall pickup and chambers
            ANGLE_ARM_SAMPLE = 110, // dropping in observation zone and baskets

            ANGLE_CLAW_OPEN = 60,
            ANGLE_CLAW_CLOSED = 9,

            TIME_DROP = 0.5,
            TIME_ARM_RETRACTION = 0.25,
            TIME_GRAB = 0.25,

            HEIGHT_INTAKING_SPECIMEN = 1.9,
            HEIGHT_OBSERVATION_ZONE = 0.1,
            HEIGHT_BASKET_LOW = 22,
            HEIGHT_BASKET_HIGH = 32,
            HEIGHT_CHAMBER_LOW = 6,
            HEIGHT_CHAMBER_HIGH = 20,
            HEIGHT_OFFSET_SPECIMEN_SCORING = -10;

    enum State {
        RETRACTED,
        HAS_SAMPLE,
        SAMPLE_FALLING,
        INTAKING_SPECIMEN,
        GRABBING_SPECIMEN,
        HAS_SPECIMEN,
        SCORING_SPECIMEN,
        RELEASING_SPECIMEN,
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
    private final SimpleServoPivot arm, claw;

    private final ElapsedTime timer = new ElapsedTime(), timeArmSpentRetracted = new ElapsedTime();

    private Sample sample, specimenColor = RED;

    private Deposit.State state = RETRACTED;

    private double releaseSpecimenHeight = HEIGHT_CHAMBER_LOW + HEIGHT_OFFSET_SPECIMEN_SCORING;

    public void setAlliance(boolean redAlliance) {
        specimenColor = redAlliance ? RED : BLUE;
    }

    Deposit(HardwareMap hardwareMap) {

        lift = new Lift(hardwareMap);

        arm = new SimpleServoPivot(
                ANGLE_ARM_RETRACTED,
                ANGLE_ARM_SAMPLE,
                getAxon(hardwareMap, "arm left"),
                getAxon(hardwareMap, "arm right").reversed()
        );

        claw = new SimpleServoPivot(
                ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED,
                getGBServo(hardwareMap, "claw").reversed()
        );
    }

    void run(boolean intakeClear, boolean climbing) {

        boolean canMove = intakeClear || !hasSample() || handlingSpecimen();

        // release sample when climbing begins
        if (climbing) state = RETRACTED;
        else switch (state) {

            case SAMPLE_FALLING:

                if (timer.seconds() >= TIME_DROP) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case GRABBING_SPECIMEN:

                if (timer.seconds() >= TIME_GRAB) triggerClaw();

                break;

            case SCORING_SPECIMEN:

                if (lift.getPosition() <= releaseSpecimenHeight) triggerClaw();

                break;

            case RELEASING_SPECIMEN:

                if (timer.seconds() >= TIME_DROP) triggerClaw();

                break;

        }

        runArm(canMove);

        lift.run(canMove, climbing);
    }

    public void preload() {
        triggerClaw();
        triggerClaw();
        triggerClaw();
        runArm(true);
    }

    private void runArm(boolean canMove) {
        arm.updateAngles(
                ANGLE_ARM_RETRACTED,
                handlingSpecimen() ? ANGLE_ARM_SPECIMEN : ANGLE_ARM_SAMPLE
        );
        arm.setActivated(state != RETRACTED && canMove);
        arm.run();

        if (arm.isActivated()) timeArmSpentRetracted.reset();

        claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        claw.setActivated(hasSample());    // activate claw when we have a sample, otherwise deactivate
        claw.run();
    }

    private boolean handlingSpecimen() {
        return state.ordinal() >= INTAKING_SPECIMEN.ordinal();
    }

    boolean impedingIntake() {
        return !handlingSpecimen() && hasSample() && (
                state != RETRACTED ||
                lift.getTarget() != 0 ||
                lift.isExtended() ||
                timeArmSpentRetracted.seconds() <= TIME_ARM_RETRACTION
        );
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

                state = HAS_SPECIMEN;
                setPosition(LOW);

                break;

            case HAS_SPECIMEN:

                double position = lift.getPosition();
                releaseSpecimenHeight = clip(position + HEIGHT_OFFSET_SPECIMEN_SCORING, 0, position);
                lift.setTarget(0);

                state = SCORING_SPECIMEN;

                break;

            case SCORING_SPECIMEN:

                sample = null;
                state = RELEASING_SPECIMEN;
                lift.setTarget(0);
                timer.reset();

                break;

            case RELEASING_SPECIMEN:

                state = RETRACTED;

                break;
        }
    }

    boolean hasSample() {
        return sample != null;
    }

    public void transfer(Sample sample) {
        if (sample == null || hasSample() || state != RETRACTED) return;
        this.sample = sample;
        state = HAS_SAMPLE;
        setPosition(FLOOR);
    }

    void printTelemetry() {
        mTelemetry.addData("DEPOSIT", state + ", " + (hasSample() ? sample + (handlingSpecimen() ? " specimen" : " sample") : "empty"));
        divider();
        lift.printTelemetry();
    }

}
