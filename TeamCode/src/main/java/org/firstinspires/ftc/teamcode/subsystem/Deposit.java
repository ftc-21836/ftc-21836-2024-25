package org.firstinspires.ftc.teamcode.subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.HAS_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RETRACTED;
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
            ANGLE_ARM_RETRACTED = 10,
            ANGLE_ARM_SPECIMEN = 110, // wall pickup and chambers
            ANGLE_ARM_SAMPLE = 151, // dropping in observation zone and baskets

            ANGLE_CLAW_OPEN = 80,
            ANGLE_CLAW_TRANSFER = 45,
            ANGLE_CLAW_CLOSED = 29,

            TIME_DROP = 0.5,
            TIME_ARM_RETRACTION = 0.25,
            TIME_GRAB = 0.25,

            HEIGHT_INTAKING_SPECIMEN = 0.1,
            HEIGHT_OFFSET_POST_INTAKING = 4,
            HEIGHT_OBSERVATION_ZONE = 1,
            HEIGHT_BASKET_LOW = 20,
            HEIGHT_BASKET_HIGH = 32,
            HEIGHT_CHAMBER_LOW = 6,
            HEIGHT_CHAMBER_HIGH = 20,
            HEIGHT_OFFSET_SPECIMEN_SCORING = -10;

    enum State {
        RETRACTED,
        HAS_SAMPLE,
        INTAKING_SPECIMEN,
        HAS_SPECIMEN,
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
    private final SimpleServoPivot arm, claw;

    private final ElapsedTime
            timeSinceSampleReleased = new ElapsedTime(),
            timeSinceArmExtended = new ElapsedTime(),
            timeSinceSpecimenGrabbed = new ElapsedTime();

    private Sample sample, specimenColor;

    private Deposit.State state = RETRACTED;

    private double releaseSpecimenHeight = HEIGHT_CHAMBER_LOW + HEIGHT_OFFSET_SPECIMEN_SCORING;

    public void setAlliance(boolean redAlliance) {
        specimenColor = redAlliance ? RED : BLUE;
    }

    Deposit(HardwareMap hardwareMap) {
        this.lift = new Lift(hardwareMap);

        arm = new SimpleServoPivot(
                ANGLE_ARM_RETRACTED,
                ANGLE_ARM_SAMPLE,
                getAxon(hardwareMap, "arm left"),
                getAxon(hardwareMap, "arm right").reversed()
        );

        claw = new SimpleServoPivot(
                ANGLE_CLAW_TRANSFER,
                ANGLE_CLAW_CLOSED,
                getGBServo(hardwareMap, "claw").reversed()
        );
    }

    void run(boolean freeToMove, boolean climbing) {

        // release sample when climbing begins
        if (climbing && state != RETRACTED) {
            sample = null;
            state = RETRACTED;      // RETRACTED state means the following state machine is skipped
        }

        switch (state) {

            case INTAKING_SPECIMEN:

                if (!hasSample()) timeSinceSpecimenGrabbed.reset();
                else if (timeSinceSpecimenGrabbed.seconds() >= TIME_GRAB) {
                    state = HAS_SPECIMEN;
                    setPosition(LOW);
                }

                break;

            case HAS_SPECIMEN:

                if (hasSample() && lift.getTarget() == 0 && lift.getPosition() <= releaseSpecimenHeight) triggerClaw();

            case HAS_SAMPLE:

                if (!hasSample() && timeSinceSampleReleased.seconds() >= TIME_DROP) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

        }

        arm.updateAngles(
                ANGLE_ARM_RETRACTED,
                handlingSpecimen() ? ANGLE_ARM_SPECIMEN : ANGLE_ARM_SAMPLE
        );

        claw.updateAngles(
                state == RETRACTED || !freeToMove ? ANGLE_CLAW_TRANSFER : ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED
        );

        arm.setActivated(state != RETRACTED && freeToMove);
        claw.setActivated(hasSample());    // activate claw when we have a sample, otherwise deactivate

        arm.run();
        claw.run();

        if (arm.isActivated()) timeSinceArmExtended.reset();

        lift.run(freeToMove, climbing);
    }

    private boolean handlingSpecimen() {
        return state == INTAKING_SPECIMEN || state == HAS_SPECIMEN;
    }

    boolean isActive() {
        return state != RETRACTED || lift.getTarget() != 0 || lift.isExtended() || timeSinceArmExtended.seconds() <= TIME_ARM_RETRACTION;
    }

    boolean requestSlowMode() {
        return state != RETRACTED && state != HAS_SPECIMEN;
    }

    public void setPosition(Position position) {

        switch (state) {

            case INTAKING_SPECIMEN:

                if (position == FLOOR) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case HAS_SPECIMEN:

                lift.setTarget(position == HIGH ? HEIGHT_CHAMBER_HIGH : HEIGHT_CHAMBER_LOW);

                break;

            case HAS_SAMPLE:

                if (position == FLOOR) {
                    lift.setTarget(HEIGHT_OBSERVATION_ZONE);
                    break;
                }

            case RETRACTED:
            default:

                lift.setTarget(
                        position == HIGH ? HEIGHT_BASKET_HIGH :
                        position == LOW ? HEIGHT_BASKET_LOW :
                        0
                );

                break;
        }

    }

    public void triggerClaw() {
        switch (state) {
            case RETRACTED:

                state = INTAKING_SPECIMEN;
                lift.setTarget(HEIGHT_INTAKING_SPECIMEN);

                break;

            case INTAKING_SPECIMEN:

                if (!hasSample()) sample = specimenColor;

                break;

            case HAS_SPECIMEN:

                if (lift.getTarget() != 0) {
                    double position = lift.getPosition();
                    releaseSpecimenHeight = clip(position + HEIGHT_OFFSET_SPECIMEN_SCORING, 0, position);
                    lift.setTarget(0);
                    break;
                }

            case HAS_SAMPLE:

                sample = null;
                timeSinceSampleReleased.reset();

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
        mTelemetry.addLine("DEPOSIT: " + state);
        mTelemetry.addLine();
        mTelemetry.addLine(hasSample() ? sample + (handlingSpecimen() ? " specimen" : " sample") : "Empty");
        divider();
        lift.printTelemetry();
    }

}
