package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.AT_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.AT_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.CLIMBING_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.CLIMBING_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.OUTER_HOOKS_ENGAGING;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.ABOVE_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.SAMPLE_FALLING;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.State.SCORING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.Sample.NONE;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.maxVoltage;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
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
            TIME_RETRACTION_OUTER_HOOKS = 0.5,
            COLOR_SENSOR_GAIN = 1,
            SPEED_OUTER_HOOKS_EXTENDING = 0.5,
            SPEED_OUTER_HOOKS_RETRACTING = -.1;

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

    public Intake.Sample hsvToSample(HSV hsv) {
        return
                hsv.between(minRed, maxRed) ? RED :
                hsv.between(minBlue, maxBlue) ? BLUE :
                NONE;
    }

    public enum State {
        RETRACTED,
        AT_BASKET,
        SAMPLE_FALLING,
        INTAKING_SPECIMEN,
        HAS_SPECIMEN,
        AT_CHAMBER,
        SCORING_SPECIMEN,
        ABOVE_LOW_RUNG,
        CLIMBING_LOW_RUNG,
        OUTER_HOOKS_ENGAGING,
        ABOVE_HIGH_RUNG,
        CLIMBING_HIGH_RUNG,
    }

    public final Lift lift;
    private final SimpleServoPivot arm, claw, innerHooks, limiterBars;
    private final ColorSensor sampleSensor;

    private final MotorEx outerHooks;

    private final ElapsedTime timeSinceSampleReleased = new ElapsedTime(), timeSinceArmExtended = new ElapsedTime();

    Intake.Sample sample = NONE;

    private Deposit.State state = RETRACTED;

    private boolean highScorePosition, goToScoringPosition, handleSample, climb, retract;

    private double releaseSpecimenHeight = Lift.HEIGHT_CHAMBER_HIGH - Lift.HEIGHT_OFFSET_SPECIMEN_SCORING;

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

        outerHooks = new MotorEx(hardwareMap, "outer hooks", Motor.GoBILDA.RPM_1150);
        outerHooks.setZeroPowerBehavior(FLOAT);

        sampleSensor = new ColorSensor(hardwareMap, "arm color", (float) COLOR_SENSOR_GAIN);
    }

    void run(boolean intakeClear) {

        switch (state) {
            case RETRACTED:

                if (sample == NONE) {

                    if (handleSample) {

                        lift.setTargetPosition(Lift.HEIGHT_INTAKING_SPECIMEN);
                        state = INTAKING_SPECIMEN;
                        break;
                    }

                } else if (goToScoringPosition) {           // if go to scoring position, go to high or low basket

                    lift.setTargetPosition(highScorePosition ?
                            Lift.HEIGHT_BASKET_HIGH :
                            Lift.HEIGHT_BASKET_LOW
                    );

                    state = AT_BASKET;
                    break;

                }

                
                if (climb) {

                    innerHooks.setActivated(true);
                    lift.setTargetPosition(Lift.HEIGHT_RUNG_LOW_RAISED);
                    state = ABOVE_LOW_RUNG;
                }
                
                break;

            case AT_BASKET:

                if (retract) {
                    lift.setTargetPosition(Lift.HEIGHT_RETRACTED);
                    state = RETRACTED;
                    break;
                }

                if (goToScoringPosition) {

                    lift.setTargetPosition(highScorePosition ?
                            Lift.HEIGHT_BASKET_HIGH :
                            Lift.HEIGHT_BASKET_LOW
                    );
                }

                if (handleSample) {

                    claw.setActivated(false);
                    sample = NONE;
                    state = SAMPLE_FALLING;
                    timeSinceSampleReleased.reset();
                }

                break;

            case SAMPLE_FALLING:

                if (timeSinceSampleReleased.seconds() >= TIME_DROP || retract) {

                    lift.setTargetPosition(Lift.HEIGHT_RETRACTED);
                    state = RETRACTED;

                }

                break;

            case INTAKING_SPECIMEN:

                if (retract) {
                    lift.setTargetPosition(Lift.HEIGHT_RETRACTED);
                    state = RETRACTED;
                    break;
                }

                sample = handleSample ? NEUTRAL : hsvToSample(sampleSensor.getHSV());

                if (sample != NONE) {

                    claw.setActivated(true);
                    lift.setTargetPosition(Lift.HEIGHT_INTAKING_SPECIMEN + Lift.HEIGHT_OFFSET_POST_INTAKING);
                    state = HAS_SPECIMEN;

                }

                break;

            case HAS_SPECIMEN:

                if (goToScoringPosition) {

                    lift.setTargetPosition(highScorePosition ?
                            Lift.HEIGHT_CHAMBER_HIGH :
                            Lift.HEIGHT_CHAMBER_LOW
                    );

                    state = AT_CHAMBER;
                }

                break;

            case AT_CHAMBER:

                if (retract) {
                    lift.setTargetPosition(Lift.HEIGHT_INTAKING_SPECIMEN + Lift.HEIGHT_OFFSET_POST_INTAKING);
                    state = HAS_SPECIMEN;
                    break;
                }

                if (goToScoringPosition) {

                    lift.setTargetPosition(highScorePosition ?
                            Lift.HEIGHT_CHAMBER_HIGH :
                            Lift.HEIGHT_CHAMBER_LOW
                    );
                }

                if (handleSample) {

                    releaseSpecimenHeight = lift.currentState.x - Lift.HEIGHT_OFFSET_SPECIMEN_SCORING;

                    lift.setTargetPosition(Lift.HEIGHT_RETRACTED);
                    state = SCORING_SPECIMEN;

                }

                break;

            case SCORING_SPECIMEN:

                if (lift.currentState.x <= releaseSpecimenHeight || handleSample) {
                    claw.setActivated(false);
                    state = RETRACTED;
                }

                break;

            case ABOVE_LOW_RUNG:

                if (retract) {
                    lift.setTargetPosition(Lift.HEIGHT_RETRACTED);
                    state = RETRACTED;
                    innerHooks.setActivated(false);
                    break;
                }

                if (climb) {

                    lift.setTargetPosition(Lift.HEIGHT_RUNG_LOW_CLIMBING);
                    state = CLIMBING_LOW_RUNG;
                }

                break;

            case CLIMBING_LOW_RUNG:

                if (retract) {
                    lift.setTargetPosition(Lift.HEIGHT_RUNG_LOW_RAISED);
                    state = ABOVE_LOW_RUNG;
                    break;
                }

                if (climb) {

                    outerHooks.set(SPEED_OUTER_HOOKS_EXTENDING);
                    state = OUTER_HOOKS_ENGAGING;
                }

                break;

            case OUTER_HOOKS_ENGAGING:

                if (climb) {

                    outerHooks.set(0);

                    innerHooks.setActivated(false);
                    lift.setTargetPosition(Lift.HEIGHT_RUNG_HIGH_RAISED);
                    state = ABOVE_HIGH_RUNG;

                }

                break;

            case ABOVE_HIGH_RUNG:

                if (climb) {

                    outerHooks.set(SPEED_OUTER_HOOKS_RETRACTING);
                    innerHooks.setActivated(true);
                    lift.setTargetPosition(Lift.HEIGHT_RUNG_HIGH_CLIMBING);
                    state = CLIMBING_HIGH_RUNG;

                }

                break;

            case CLIMBING_HIGH_RUNG:

                if (retract) {
                    lift.setTargetPosition(Lift.HEIGHT_RUNG_HIGH_RAISED);
                    state = ABOVE_HIGH_RUNG;
                    break;
                }

                if (climb) {

                    if (outerHooks.get() != 0) outerHooks.set(0);
                    else limiterBars.setActivated(true);
                }

                break;
        }

        goToScoringPosition = handleSample = climb = retract = false;

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

    boolean isActive() {
        return state != RETRACTED || lift.isExtended() || timeSinceArmExtended.seconds() <= TIME_ARM_RETRACTION;
    }

    public boolean slowMode() {
        return state != RETRACTED && state != HAS_SPECIMEN;
    }

    public void goToScoringPosition(boolean highScorePosition) {
        goToScoringPosition = true;
        this.highScorePosition = highScorePosition;
    }

    public void handleSample() {
        handleSample = true;
    }

    public void retract() {
        retract = true;
    }
    
    public void climb() {
        climb = true;
    }

    public void transfer(Intake.Sample sample) {
        this.sample = sample;
        claw.setActivated(true);
    }

    void printTelemetry() {
        mTelemetry.addData("Current state", state);
        mTelemetry.addLine();
        mTelemetry.addData("Deposit", (sample == NONE ? "empty" : "contains a " + sample.name() + " sample"));
    }

    @Config
    public static final class Lift {

        public static PIDGains pidGains = new PIDGains(
                0.3,
                0.2,
                0,
                0.2
        );

        public static double
                kG = 0.15,
                INCHES_PER_TICK = 0.0088581424,
                HEIGHT_RETRACTED = 0,
                HEIGHT_RETRACTED_THRESHOLD = 0.5,
                HEIGHT_INTAKING_SPECIMEN = 1,
                HEIGHT_OFFSET_POST_INTAKING = 1,
                HEIGHT_BASKET_LOW = 1,
                HEIGHT_BASKET_HIGH = 1,
                HEIGHT_CHAMBER_LOW = 1,
                HEIGHT_CHAMBER_HIGH = 1,
                HEIGHT_RUNG_LOW_RAISED = 1,
                HEIGHT_RUNG_LOW_CLIMBING = 1,
                HEIGHT_RUNG_HIGH_RAISED = 1,
                HEIGHT_TO_ACTIVATE_LIMITER_BAR = 1,
                HEIGHT_RUNG_HIGH_CLIMBING = 1,
                HEIGHT_OFFSET_SPECIMEN_SCORING = 1;

        // Motors and variables to manage their readings:
        private final MotorEx[] motors;
        private final Motor.Encoder encoder;
        private org.firstinspires.ftc.teamcode.control.motion.State currentState, targetState;
        private final PIDController controller = new PIDController();

        private double manualLiftPower;
        private int encoderOffset;

        // Battery voltage sensor and variable to track its readings:
        private final VoltageSensor batteryVoltageSensor;

        private Lift(HardwareMap hardwareMap) {
            this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            this.motors = new MotorEx[]{
                    new MotorEx(hardwareMap, "lift right", RPM_312),
                    new MotorEx(hardwareMap, "lift left", RPM_312)
            };
            motors[1].setInverted(true);
            for (MotorEx motor : motors) motor.setZeroPowerBehavior(FLOAT);

            encoder = new MotorEx(hardwareMap, "right back", RPM_312).encoder;

            reset();
        }

        public void reset() {
            controller.reset();
            encoderOffset = encoder.getPosition();
            targetState = currentState = new org.firstinspires.ftc.teamcode.control.motion.State();
        }

        public boolean isExtended() {
            return currentState.x > HEIGHT_RETRACTED_THRESHOLD;
        }

        public void setTargetPosition(double inches) {
            this.targetState = new org.firstinspires.ftc.teamcode.control.motion.State(inches);
        }

        public void setLiftPower(double manualLiftPower) {
            this.manualLiftPower = manualLiftPower;
        }

        public void readSensors() {
            currentState = new org.firstinspires.ftc.teamcode.control.motion.State(INCHES_PER_TICK * (encoder.getPosition() - encoderOffset));
            controller.setGains(pidGains);
        }

        private void run(boolean intakeClear) {

            if (manualLiftPower != 0) targetState = currentState; // replace PID target with current state if using manual control

            controller.setTarget(intakeClear ? targetState : new org.firstinspires.ftc.teamcode.control.motion.State(0)); // set PID target to 0 (retract) if intake isn't yet out of the way

            boolean retracted = !isExtended();

            double voltageScalar = maxVoltage / batteryVoltageSensor.getVoltage();

            double gravityFeedforward = retracted ? 0 : kG * voltageScalar;

            double output = gravityFeedforward + (
                    manualLiftPower != 0 ?                              // if manual input is being used:
                            manualLiftPower * voltageScalar :               // control with manual power (and voltage compensate)
                            controller.calculate(currentState)  // control with PID output
            );

            for (MotorEx motor : motors) motor.set(output);
        }

        void printNumericalTelemetry() {
            mTelemetry.addData("Current position (in)", currentState.x);
            mTelemetry.addData("Target position (in)", targetState.x);
            mTelemetry.addLine();
            mTelemetry.addData("Lift error derivative (in/s)", controller.getFilteredErrorDerivative());
        }
    }
}
