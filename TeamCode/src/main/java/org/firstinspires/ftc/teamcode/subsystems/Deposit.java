package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.HAS_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

@Config
public final class Deposit {

    public static double
            ANGLE_ARM_RETRACTED = 10,
            ANGLE_ARM_SPECIMEN = 100, // wall pickup and chambers
            ANGLE_ARM_SAMPLE = 130, // dropping in observation zone and baskets
            ANGLE_CLAW_OPEN = 13,
            ANGLE_CLAW_TRANSFER = 30,
            ANGLE_CLAW_CLOSED = 60,
            TIME_DROP = 1,
            TIME_ARM_RETRACTION = 1,
            COLOR_SENSOR_GAIN = 1,
            HEIGHT_INTAKING_SPECIMEN = 1,
            HEIGHT_OFFSET_POST_INTAKING = 1,
            HEIGHT_OBSERVATION_ZONE = 1,
            HEIGHT_BASKET_LOW = 1,
            HEIGHT_BASKET_HIGH = 1,
            HEIGHT_CHAMBER_LOW = 1,
            HEIGHT_CHAMBER_HIGH = 1,
            HEIGHT_OFFSET_SPECIMEN_SCORING = -1;

    /**
     * HSV value bound for specimen detection
     */
    public static HSV
            minRed = new HSV(
                    0,
                    0.5,
                    0
            ),
            maxRed = new HSV(
                    30,
                    0.75,
                    0.06
            ),
            minBlue = new HSV(
                    215,
                    0.6,
                    0
            ),
            maxBlue = new HSV(
                    230,
                    0.9,
                    0.1
            );

    public Sample hsvToSample(HSV hsv) {
        if (hsv.between(minRed, maxRed)) return RED;
        if (hsv.between(minBlue, maxBlue)) return BLUE;
        return null;
    }

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

    private final ColorSensor colorSensor;
    private HSV hsv = new HSV();

    private final ElapsedTime timeSinceSampleReleased = new ElapsedTime(), timeSinceArmExtended = new ElapsedTime();

    private Sample sample, badSample;

    private Deposit.State state = RETRACTED;

    private double releaseSpecimenHeight = HEIGHT_CHAMBER_LOW + HEIGHT_OFFSET_SPECIMEN_SCORING;

    public void setAlliance(boolean redAlliance) {
        badSample = redAlliance ? BLUE : RED;
    }

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);

        arm = new SimpleServoPivot(
                ANGLE_ARM_RETRACTED,
                ANGLE_ARM_SAMPLE,
                getAxonServo(hardwareMap, "arm left"),
                getReversedServo(getAxonServo(hardwareMap, "arm right"))
        );

        claw = new SimpleServoPivot(
                ANGLE_CLAW_TRANSFER,
                ANGLE_CLAW_CLOSED,
                getReversedServo(getGoBildaServo(hardwareMap, "claw"))
        );

        colorSensor = new ColorSensor(hardwareMap, "arm color", (float) COLOR_SENSOR_GAIN);
    }

    void run(boolean intakeClearOfDeposit, boolean climbing) {

        arm.updateAngles(
                ANGLE_ARM_RETRACTED,
                state == INTAKING_SPECIMEN || state == HAS_SPECIMEN ? ANGLE_ARM_SPECIMEN : ANGLE_ARM_SAMPLE
        );

        claw.updateAngles(
                state == RETRACTED ? ANGLE_CLAW_TRANSFER : ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED
        );

        arm.setActivated(intakeClearOfDeposit && state != RETRACTED);
        claw.setActivated(hasSample());    // activate claw when we have a sample, otherwise deactivate

        // release sample when climbing begins
        if (climbing && state != RETRACTED) {
            sample = null;
            state = RETRACTED;      // RETRACTED state means the following state machine is skipped
        }

        switch (state) {

            case HAS_SAMPLE:

                if (!claw.isActivated() && timeSinceSampleReleased.seconds() >= TIME_DROP) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case INTAKING_SPECIMEN:

                // check color sensor if needed
                colorSensor.update();
                hsv = colorSensor.getHSV();
                sample = hsvToSample(hsv);      // if color sensor found sample, hasSample() returns true

                // grab specimen if present
                if (hasSample()) triggerClaw();

                break;

            case HAS_SPECIMEN:

                if (lift.getTarget() == 0 && lift.getPosition() <= releaseSpecimenHeight) triggerClaw();

                break;

        }

        arm.run();
        claw.run();

        lift.run(intakeClearOfDeposit);

        if (arm.isActivated()) timeSinceArmExtended.reset();
    }

    boolean isActive() {
        return state != RETRACTED || lift.isExtended() || timeSinceArmExtended.seconds() <= TIME_ARM_RETRACTION;
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

                lift.setTarget(
                        position == HIGH ? HEIGHT_CHAMBER_HIGH :
                        position == LOW ? HEIGHT_CHAMBER_LOW :
                        HEIGHT_INTAKING_SPECIMEN + HEIGHT_OFFSET_POST_INTAKING
                );

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

            case HAS_SAMPLE:

                sample = null;
                timeSinceSampleReleased.reset();

                break;

            case INTAKING_SPECIMEN:

                if (!hasSample()) sample = badSample == RED ? BLUE : RED;

                state = HAS_SPECIMEN;
                setPosition(FLOOR);

                break;

            case HAS_SPECIMEN:

                if (lift.getTarget() != 0) {
                    releaseSpecimenHeight = lift.getPosition() + HEIGHT_OFFSET_SPECIMEN_SCORING;
                    lift.setTarget(0);
                } else {
                    sample = null;
                    state = RETRACTED;
                }

                break;
        }
    }

    boolean hasSample() {
        return sample != null;
    }

    public void transfer(Sample sample) {
        if (sample == null || hasSample()) return;
        this.sample = sample;
        state = HAS_SAMPLE;
        setPosition(FLOOR);
    }

    void printTelemetry() {
        mTelemetry.addLine("DEPOSIT: " + state);
        mTelemetry.addLine();
        mTelemetry.addLine(hasSample() ? sample + " sample" : "Empty");
        hsv.toTelemetry();
        divider();
        lift.printTelemetry();
    }

}
