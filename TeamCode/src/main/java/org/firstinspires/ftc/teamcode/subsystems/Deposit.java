package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Arm.TIME_DROP;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Arm.TIME_RETRACTION_FROM_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Arm.TIME_RETRACTION_FROM_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Lift.Position.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Lift.Position.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Lift.Position.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.Sample.BLUE;
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
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

@Config
public final class Deposit {

    public final Arm arm;
    public final Lift lift;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
    }

    private double specimenScoringHeight = Lift.HEIGHT_CHAMBER_HIGH;

    void run(boolean intakeClear) {

        if (!arm.doneScoring) {
            if (arm.specimenMode) {

                // score specimen
                if (lift.targetPosition != RETRACTED) {
                    specimenScoringHeight = lift.currentState.x;
                    lift.setTargetPosition(RETRACTED);
                } else if (lift.currentState.x <= specimenScoringHeight - Lift.HEIGHT_DIFFERENCE_SPECIMEN_SCORING) {
                    arm.claw.setActivated(false);
                    arm.doneScoring = true;
                    arm.specimenMode = false;
                }

            } else if (arm.scoreTimer.seconds() >= TIME_DROP) {
                arm.doneScoring = true;
                lift.setTargetPosition(RETRACTED);
            }
        } else {
            if (arm.specimenMode) {
                if (arm.sample == NONE && lift.targetPosition != INTAKING) lift.setTargetPosition(INTAKING);
                if (arm.sample != NONE && lift.targetPosition == INTAKING) lift.setTargetPosition(HAS_SPECIMEN);
            }
        }

        lift.run(intakeClear);

        boolean extendArm = intakeClear && (lift.isExtending() || (!arm.doneScoring && arm.specimenMode)) && !lift.targetPosition.isClimbing;

        arm.pivot.setActivated(extendArm);

        arm.run();
    }

    boolean isActive() {
        return lift.isExtending() || lift.isExtended() || arm.retractionTimer.seconds() <= (arm.specimenMode ? TIME_RETRACTION_FROM_CHAMBER : TIME_RETRACTION_FROM_BASKET);
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
                HEIGHT_RETRACTED_MAX = 0.5,
                HEIGHT_INTAKING = 1,
                HEIGHT_OFFSET_POST_INTAKING = 1,
                HEIGHT_BASKET_LOW = 1,
                HEIGHT_BASKET_HIGH = 1,
                HEIGHT_CHAMBER_LOW = 1,
                HEIGHT_CHAMBER_HIGH = 1,
                HEIGHT_RUNG_LOW_RAISED = 1,
                HEIGHT_RUNG_LOW_PULLING = 1,
                HEIGHT_RUNG_HIGH_RAISED = 1,
                HEIGHT_RUNG_HIGH_PULLING = 1,
                HEIGHT_TO_ACTIVATE_LIMITER_BAR = 1,
                HEIGHT_DIFFERENCE_SPECIMEN_SCORING = 1;

        public enum Position {
            RETRACTED,
            HAS_SPECIMEN,
            INTAKING,
            BASKET_LOW,
            BASKET_HIGH,
            CHAMBER_LOW,
            CHAMBER_HIGH,
            RUNG_LOW_RAISED,
            RUNG_LOW_PULLING,
            RUNG_HIGH_RAISED,
            RUNG_HIGH_ACTIVATE_LIMITER_BAR,
            RUNG_HIGH_PULLING;

            private final boolean isClimbing, autoSlow;

            Position() {
                isClimbing = ordinal() >= 7;
                autoSlow = ordinal() >= 2;
            }

            public double toInches() {
                switch (this) {
                    case HAS_SPECIMEN: return HEIGHT_INTAKING + HEIGHT_OFFSET_POST_INTAKING;
                    case INTAKING: return HEIGHT_INTAKING;
                    case BASKET_LOW: return HEIGHT_BASKET_LOW;
                    case BASKET_HIGH: return HEIGHT_BASKET_HIGH;
                    case CHAMBER_LOW: return HEIGHT_CHAMBER_LOW;
                    case CHAMBER_HIGH: return HEIGHT_CHAMBER_HIGH;
                    case RUNG_LOW_RAISED: return HEIGHT_RUNG_LOW_RAISED;
                    case RUNG_LOW_PULLING: return HEIGHT_RUNG_LOW_PULLING;
                    case RUNG_HIGH_RAISED: return HEIGHT_RUNG_HIGH_RAISED;
                    case RUNG_HIGH_ACTIVATE_LIMITER_BAR: return HEIGHT_TO_ACTIVATE_LIMITER_BAR;
                    case RUNG_HIGH_PULLING: return HEIGHT_RUNG_HIGH_PULLING;
                    default: return 0;
                }
            }
        }

        // Motors and variables to manage their readings:
        private final MotorEx[] motors;
        private final Motor.Encoder encoder;
        private State currentState, targetState;
        private final PIDController controller = new PIDController();

        private double manualLiftPower;
        private int encoderOffset;

        private Lift.Position targetPosition = RETRACTED;

        // Battery voltage sensor and variable to track its readings:
        private final VoltageSensor batteryVoltageSensor;

        private Lift(HardwareMap hardwareMap) {
            this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            this.motors = new MotorEx[]{
                    new MotorEx(hardwareMap, "lift right", RPM_1150),
                    new MotorEx(hardwareMap, "lift left", RPM_1150)
            };
            motors[1].setInverted(true);
            for (MotorEx motor : motors) motor.setZeroPowerBehavior(FLOAT);

            encoder = new MotorEx(hardwareMap, "right back", RPM_312).encoder;

            reset();
        }

        public void reset() {
            targetPosition = RETRACTED;
            controller.reset();
            encoderOffset = encoder.getPosition();
            targetState = currentState = new State();
        }

        public boolean heightWarrantsSlow() {
            return targetPosition.autoSlow;
        }

        boolean isExtending() {
            return targetPosition != RETRACTED;
        }

        public boolean isExtended() {
            return currentState.x > HEIGHT_RETRACTED_MAX;
        }

        public void setTargetPosition(Lift.Position targetPosition) {
            this.targetPosition = targetPosition;
            targetState = new State(targetPosition.toInches());
        }

        public void setLiftPower(double manualLiftPower) {
            this.manualLiftPower = manualLiftPower;
        }

        public void readSensors() {
            currentState = new State(INCHES_PER_TICK * (encoder.getPosition() - encoderOffset));
            controller.setGains(pidGains);
        }

        private void run(boolean intakeClear) {

            if (manualLiftPower != 0) targetState = currentState; // replace PID target with current state if using manual control

            controller.setTarget(intakeClear ? targetState : new State(0)); // set PID target to 0 (retract) if intake isn't yet out of the way

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

        void printTelemetry() {
            mTelemetry.addData("Named target position", targetPosition.name());
        }

        void printNumericalTelemetry() {
            mTelemetry.addData("Current position (in)", currentState.x);
            mTelemetry.addData("Target position (in)", targetState.x);
            mTelemetry.addLine();
            mTelemetry.addData("Lift error derivative (in/s)", controller.getFilteredErrorDerivative());
            mTelemetry.addLine();
            mTelemetry.addData("kD (computed)", pidGains.kD);
        }
    }

    @Config
    public static final class Arm {

        public static double
                ANGLE_PIVOT_RETRACTED = 10,
                ANGLE_PIVOT_SPECIMEN = 100, // wall pickup and chambers
                ANGLE_PIVOT_SAMPLE = 130, // dropping in observation zone and baskets
                ANGLE_CLAW_OPEN = 13,
                ANGLE_CLAW_TRANSFER = 30,
                ANGLE_CLAW_CLOSED = 60,
                TIME_DROP = 1,
                TIME_RETRACTION_FROM_BASKET = 1,
                TIME_RETRACTION_FROM_CHAMBER = 1,
                COLOR_SENSOR_GAIN = 1;

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

        private final SimpleServoPivot pivot, claw;

        private final ElapsedTime scoreTimer = new ElapsedTime(), retractionTimer = new ElapsedTime();
        private boolean doneScoring = true, specimenMode = false;
        private Intake.Sample sample = NONE;

        private final ColorSensor sampleSensor;

        private Arm(HardwareMap hardwareMap) {
            pivot = new SimpleServoPivot(
                    ANGLE_PIVOT_RETRACTED,
                    ANGLE_PIVOT_SAMPLE,
                    getAxonServo(hardwareMap, "arm left"),
                    getReversedServo(getAxonServo(hardwareMap, "arm right"))
            );

            claw = new SimpleServoPivot(
                    ANGLE_CLAW_OPEN,
                    ANGLE_CLAW_CLOSED,
                    getReversedServo(getGoBildaServo(hardwareMap, "claw"))
            );

            sampleSensor = new ColorSensor(hardwareMap, "arm color", (float) COLOR_SENSOR_GAIN);
        }

        void transfer(Intake.Sample sample) {
            this.sample = sample;
            claw.setActivated(true);
        }

        public void handleSample() {
            if (sample == NONE) {           // no sample --> toggle specimen mode
                specimenMode = !specimenMode;
//                pivot.setActivated(true);
            } else if (specimenMode) {      // has sample + in specimen mode --> score specimen
                doneScoring = false;
                scoreTimer.reset();
            } else {                        // has sample + not specimen mode --> score sample in basket
                sample = NONE;
                claw.setActivated(false);
                if (!pivot.isActivated()) return;
                doneScoring = false;
                scoreTimer.reset();
            }
        }

        private void run() {

            if (specimenMode) {
                sampleSensor.update();
                sample = hsvToSample(sampleSensor.getHSV());
                if (sample != NONE) claw.setActivated(true);
            }

            pivot.updateAngles(
                    ANGLE_PIVOT_RETRACTED,
                    specimenMode ? ANGLE_PIVOT_SPECIMEN : ANGLE_PIVOT_SAMPLE
            );

            claw.updateAngles(
                    pivot.isActivated() ? ANGLE_CLAW_OPEN : ANGLE_CLAW_TRANSFER,
                    ANGLE_CLAW_CLOSED
            );

            pivot.run();
            claw.run();

            if (pivot.isActivated()) retractionTimer.reset();
        }

    }
}
