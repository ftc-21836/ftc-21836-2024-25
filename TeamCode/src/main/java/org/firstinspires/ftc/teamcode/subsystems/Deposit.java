package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.ABOVE_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.HAS_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.CLIMBING_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.CLIMBING_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.OUTER_HOOKS_ENGAGING;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.SCORING_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.SCORING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.OutputPosition.HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.OutputPosition.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.OutputPosition.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.UserCommand.CLAW;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.UserCommand.CLIMB;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.UserCommand.RETRACT;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.NONE;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;
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
            ANGLE_HOOKS_RETRACTED = 0,
            ANGLE_HOOKS_EXTENDED = 90,
            ANGLE_BARS_RETRACTED = 0,
            ANGLE_BARS_EXTENDED = 90,
            TIME_DROP = 1,
            TIME_ARM_RETRACTION = 1,
            TIME_OUTER_HOOKS_EXTENSION = 0.5,
            TIME_OUTER_HOOKS_RETRACTION = 0.5,
            TIME_INNER_HOOKS_DISENGAGING = 0.5,
            DURATION_INNER_HOOKS_RETRACTED = 2,
            COLOR_SENSOR_GAIN = 1,
            SPEED_OUTER_HOOKS_EXTENDING = 0.5,
            SPEED_OUTER_HOOKS_RETRACTING = -.1,
            HEIGHT_INTAKING_SPECIMEN = 1,
            HEIGHT_OFFSET_POST_INTAKING = 1,
            HEIGHT_BASKET_LOW = 1,
            HEIGHT_BASKET_HIGH = 1,
            HEIGHT_CHAMBER_LOW = 1,
            HEIGHT_CHAMBER_HIGH = 1,
            HEIGHT_RUNG_LOW_RAISED = 1,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -1,
            HEIGHT_RUNG_HIGH_RAISED = 1,
            HEIGHT_TO_ACTIVATE_LIMITER_BAR = 1,
            HEIGHT_RUNG_HIGH_CLIMB_OFFSET = -1,
            HEIGHT_OFFSET_SPECIMEN_SCORING = -1;

    /**
     * HSV value bound for sample grabbing
     */
    public static HSV
            minRed = new HSV(
                    205,
                    0.55,
                    0.01
            ),
            maxRed = new HSV(
                    225,
                    1,
                    0.35
            ),
            minBlue = new HSV(
                    130,
                    0.5,
                    0.01
            ),
            maxBlue = new HSV(
                    160,
                    1,
                    0.2
            );

    public Sample hsvToSample(HSV hsv) {
        return
                hsv.between(minRed, maxRed) ? RED :
                hsv.between(minBlue, maxBlue) ? BLUE :
                NONE;
    }

    enum State {
        RETRACTED,

        HAS_SAMPLE,
        SCORING_SAMPLE,

        INTAKING_SPECIMEN,
        HAS_SPECIMEN,
        SCORING_SPECIMEN,

        ABOVE_LOW_RUNG,
        CLIMBING_LOW_RUNG,
        OUTER_HOOKS_ENGAGING,
        ABOVE_HIGH_RUNG,
        CLIMBING_HIGH_RUNG,
    }

    public enum OutputPosition {
        FLOOR,
        LOW,
        HIGH,
    }

    public enum UserCommand {
        CLAW,
        CLIMB,
        RETRACT,
    }

    public final Lift lift;
    private final SimpleServoPivot arm, claw, innerHooks, limiterBars;
    private final ColorSensor sampleSensor;

    private final CachedMotorEx outerHooks;

    private final ElapsedTime timeSinceSampleReleased = new ElapsedTime(), timeSinceArmExtended = new ElapsedTime(), timer = new ElapsedTime();

    Sample sample = NONE;

    private Deposit.State state = RETRACTED;

    private OutputPosition position = FLOOR;

    private UserCommand command = null;

    private double releaseSpecimenHeight = HEIGHT_CHAMBER_LOW + HEIGHT_OFFSET_SPECIMEN_SCORING;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);

        arm = new SimpleServoPivot(
                ANGLE_ARM_RETRACTED,
                ANGLE_ARM_SAMPLE,
                getAxonServo(hardwareMap, "arm left"),
                getReversedServo(getAxonServo(hardwareMap, "arm right"))
        );

        claw = new SimpleServoPivot(
                ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED,
                getReversedServo(getGoBildaServo(hardwareMap, "claw"))
        );

        innerHooks = new SimpleServoPivot(
                ANGLE_HOOKS_RETRACTED,
                ANGLE_HOOKS_EXTENDED,
                getGoBildaServo(hardwareMap, "right inner hook"),
                getReversedServo(getGoBildaServo(hardwareMap, "left inner hook"))
        );

        limiterBars = new SimpleServoPivot(
                ANGLE_BARS_RETRACTED,
                ANGLE_BARS_EXTENDED,
                getGoBildaServo(hardwareMap, "right bar"),
                getReversedServo(getGoBildaServo(hardwareMap, "left bar"))
        );

        outerHooks = new CachedMotorEx(hardwareMap, "outer hooks", Motor.GoBILDA.RPM_1150);
        outerHooks.setZeroPowerBehavior(FLOAT);

        sampleSensor = new ColorSensor(hardwareMap, "arm color", (float) COLOR_SENSOR_GAIN);
    }

    void run(boolean intakeClear) {

        if (command == RETRACT) setPosition(FLOOR);

        switch (state) {
            case RETRACTED:

                if (sample == NONE) {

                    if (command == CLAW) {

                        lift.setTargetPosition(HEIGHT_INTAKING_SPECIMEN);
                        state = INTAKING_SPECIMEN;
                        break;
                    }

                } else {
                    setPosition(FLOOR);
                    state = HAS_SAMPLE;
                    break;
                }

                if (command == CLIMB) climb();
                
                break;

            case HAS_SAMPLE:
                
                lift.setTargetPosition(
                    position == HIGH ? HEIGHT_BASKET_HIGH :
                    position == LOW ? HEIGHT_BASKET_LOW :
                    0
                );

                if (command == CLAW) {

                    claw.setActivated(false);
                    sample = NONE;
                    state = SCORING_SAMPLE;
                    timeSinceSampleReleased.reset();
                }

                if (command == CLIMB) climb();

                break;

            case SCORING_SAMPLE:

                if (timeSinceSampleReleased.seconds() >= TIME_DROP || command == RETRACT) {

                    lift.setTargetPosition(0);
                    state = RETRACTED;

                }

                if (command == CLIMB) climb();

                break;

            case INTAKING_SPECIMEN:

                if (command == RETRACT) {
                    lift.setTargetPosition(0);
                    state = RETRACTED;
                    break;
                }

                sample = command == CLAW ? NEUTRAL : hsvToSample(sampleSensor.getHSV());

                if (sample != NONE) {

                    claw.setActivated(true);
                    lift.setTargetPosition(HEIGHT_INTAKING_SPECIMEN + HEIGHT_OFFSET_POST_INTAKING);
                    setPosition(FLOOR);
                    state = HAS_SPECIMEN;

                }

                if (command == CLIMB) climb();

                break;

            case HAS_SPECIMEN:

                lift.setTargetPosition(
                    position == HIGH ? HEIGHT_CHAMBER_HIGH :
                    position == LOW ? HEIGHT_CHAMBER_LOW :
                    HEIGHT_INTAKING_SPECIMEN + HEIGHT_OFFSET_POST_INTAKING
                );

                if (command == CLAW) {

                    releaseSpecimenHeight = lift.currentState.x + HEIGHT_OFFSET_SPECIMEN_SCORING;

                    lift.setTargetPosition(0);
                    state = SCORING_SPECIMEN;

                }

                if (command == CLIMB) climb();

                break;

            case SCORING_SPECIMEN:

                if (lift.currentState.x <= releaseSpecimenHeight || command == CLAW) {
                    claw.setActivated(false);
                    state = RETRACTED;
                }

                if (command == CLIMB) climb();

                break;

            case ABOVE_LOW_RUNG:

                if (command == RETRACT) {
                    lift.setTargetPosition(0);
                    state = RETRACTED;
                    innerHooks.setActivated(false);
                    break;
                }

                if (command == CLIMB) {

                    lift.setTargetPosition(HEIGHT_RUNG_LOW_RAISED + HEIGHT_RUNG_LOW_CLIMB_OFFSET);
                    state = CLIMBING_LOW_RUNG;
                }

                break;

            case CLIMBING_LOW_RUNG:

                if (command == RETRACT) {
                    lift.setTargetPosition(HEIGHT_RUNG_LOW_RAISED);
                    state = ABOVE_LOW_RUNG;
                    break;
                }

                if (command == CLIMB) {

                    outerHooks.set(SPEED_OUTER_HOOKS_EXTENDING);
                    state = OUTER_HOOKS_ENGAGING;
                }

                break;

            case OUTER_HOOKS_ENGAGING:

                if (command == CLIMB) {

                    outerHooks.set(0);

                    lift.setTargetPosition(HEIGHT_RUNG_HIGH_RAISED);
                    state = ABOVE_HIGH_RUNG;

                    timer.reset();

                }

                break;

            case ABOVE_HIGH_RUNG:

                innerHooks.setActivated(
                        timer.seconds() < TIME_INNER_HOOKS_DISENGAGING ||
                        timer.seconds() > TIME_INNER_HOOKS_DISENGAGING + DURATION_INNER_HOOKS_RETRACTED
                );

                if (command == CLIMB) {

                    outerHooks.set(SPEED_OUTER_HOOKS_RETRACTING);
                    lift.setTargetPosition(HEIGHT_RUNG_HIGH_RAISED + HEIGHT_RUNG_HIGH_CLIMB_OFFSET);
                    state = CLIMBING_HIGH_RUNG;

                }

                break;

            case CLIMBING_HIGH_RUNG:

                if (command == RETRACT) {
                    lift.setTargetPosition(HEIGHT_RUNG_HIGH_RAISED);
                    state = ABOVE_HIGH_RUNG;
                    break;
                }

                if (command == CLIMB) {

                    if (outerHooks.get() != 0) outerHooks.set(0);
                    else limiterBars.setActivated(true);
                }

                break;
        }

        command = null;

        boolean climbing = state.ordinal() >= ABOVE_LOW_RUNG.ordinal();
        boolean extendArm = intakeClear && state != RETRACTED && !climbing;
        arm.setActivated(extendArm);

        boolean handlingSpecimen = INTAKING_SPECIMEN.ordinal() <= state.ordinal() && state.ordinal() <= SCORING_SPECIMEN.ordinal();

        arm.updateAngles(
                ANGLE_ARM_RETRACTED,
                handlingSpecimen ? ANGLE_ARM_SPECIMEN : ANGLE_ARM_SAMPLE
        );

        claw.updateAngles(
                state == RETRACTED ? ANGLE_CLAW_TRANSFER : ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED
        );

        innerHooks.updateAngles(ANGLE_HOOKS_RETRACTED, ANGLE_HOOKS_EXTENDED);

        arm.run();
        claw.run();
        innerHooks.run();
        limiterBars.run();

        lift.run(intakeClear);

        if (arm.isActivated()) timeSinceArmExtended.reset();
    }

    private void climb() {
        innerHooks.setActivated(true);
        lift.setTargetPosition(HEIGHT_RUNG_LOW_RAISED);
        state = ABOVE_LOW_RUNG;
    }

    boolean isActive() {
        return state != RETRACTED || lift.isExtended() || timeSinceArmExtended.seconds() <= TIME_ARM_RETRACTION;
    }

    boolean movingToScore() {
        return state != RETRACTED && state != HAS_SPECIMEN;
    }

    public void setCommand(UserCommand command) {
        this.command = command;
    }

    public void setPosition(OutputPosition position) {
        this.position = position;
    }

    public void transfer(Sample sample) {
        this.sample = sample;
        claw.setActivated(true);
    }

    void printTelemetry() {
        mTelemetry.addData("Current state", state);
        mTelemetry.addLine();
        mTelemetry.addData("Deposit", (sample == NONE ? "empty" : "contains a " + sample.name() + " sample"));
    }

}
