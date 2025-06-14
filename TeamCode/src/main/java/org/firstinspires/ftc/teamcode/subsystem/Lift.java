package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ClimbState.INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Lift {

    enum ClimbState {
        INACTIVE,
        TILTING_SWITCHING,
        PULLING_FIRST_RUNG,
        RAISING_SECOND_RUNG,
        SWINGING_SWITCHING,
        PULLING_SECOND_RUNG,
        HOLDING_SECOND_RUNG;

        private static final ClimbState[] states = values();

        private ClimbState next() {
            int max = states.length;
            return states[((ordinal() + 1) % max + max) % max];
        }
    }

    public boolean isClimbing() {
        return climbState != INACTIVE;
    }

    ClimbState climbState = INACTIVE;
    private final ElapsedTime climbTimer = new ElapsedTime();

    interface Swingable {
        void swing();
    }

    private final Swingable arm;

    public void climb() {
        switch (climbState) {
            case INACTIVE:
                if (getTarget() != HEIGHT_ABOVE_FIRST_RUNG) {
                    setTarget(HEIGHT_ABOVE_FIRST_RUNG);
                    return;
                }
                gearSwitch.setActivated(true);
                tilt.setActivated(true);
                break;
            case TILTING_SWITCHING:
                runManual(-1);
                break;
            case PULLING_FIRST_RUNG:
                runManual(0);
                gearSwitch.setActivated(false);
                tilt.setActivated(false);
                setTarget(HEIGHT_ABOVE_SECOND_RUNG);
                break;
            case RAISING_SECOND_RUNG:
                arm.swing();
                gearSwitch.setActivated(true);
                break;
            case SWINGING_SWITCHING:
                runManual(-1);
                break;
            case PULLING_SECOND_RUNG:
                runManual(0);
                hold = true;
                break;
            case HOLDING_SECOND_RUNG:
                hold = false;
                gearSwitch.setActivated(false);
                break;
        }
        climbState = climbState.next();
        climbTimer.reset();
    }

    public boolean hold = false;

    public static PIDGains
            pidGains = new PIDGains(0.4, 0.4),
            dtPidGains = new PIDGains(0, 0);

    public static double
            kG = 0.5,
            SCALAR_DOWNWARD = 1,
            INCHES_PER_TICK = 0.031300435271111114,
            POSITION_TOLERANCE = 0.25,
            HEIGHT_RETRACTING = 0,
            SPEED_RETRACTION = -0,
            MAX_VOLTAGE = 13,

            kG_CLIMB = -0.85,

            HEIGHT_EXTENDED = 28.34645669291339,
            HEIGHT_START_kG = 1,

            HEIGHT_ABOVE_FIRST_RUNG = 13,
            HEIGHT_ABOVE_SECOND_RUNG = 25,

            ANGLE_TILTER_INACTIVE = 225,
            ANGLE_TILTER_TILTED = 130,

            ANGLE_SWITCH_INACTIVE = 0,
            ANGLE_SWITCH_ENGAGED = 80,
            ANGLE_RIGHT_SWITCH_OFFSET = 0,

            TIME_TILT_AND_SWITCH = 1,
//            TIME_PULL_FIRST_RUNG = 4,
            TIME_RAISE_SECOND_RUNG = 5,
            TIME_SWING_AND_SWITCH = 1,
            TIME_PULL_SECOND_RUNG = 4;

    public static Motor.ZeroPowerBehavior zeroPowerBehavior = FLOAT;

    private final CachedMotorEx[] motors;
    private final MecanumDrive dt;

    private final PIDController controller = new PIDController();
    private final VoltageSensor batteryVoltageSensor;
    private final TouchSensor liftSensor;

    public final SimpleServoPivot tilt, gearSwitch;
    private final CachedSimpleServo switchR;

    private double position, target, manualPower;

    Lift(HardwareMap hardwareMap, MecanumDrive dt, Swingable arm) {

        this.arm = arm;
        this.dt = dt;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "lift right", RPM_1620),
                new CachedMotorEx(hardwareMap, "lift left", RPM_1620),
        };
        motors[0].setInverted(true);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "left front", RPM_1620).encoder;
        motors[1].encoder = new CachedMotorEx(hardwareMap, "left back", RPM_1620).encoder;
        motors[1].encoder.setDirection(REVERSE);

        for (CachedMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            motor.encoder.setDistancePerPulse(INCHES_PER_TICK);
            motor.encoder.reset();
        }

        liftSensor = hardwareMap.get(TouchSensor.class, "lift sensor");

        tilt = new SimpleServoPivot(ANGLE_TILTER_INACTIVE, ANGLE_TILTER_TILTED,
                getAxon(hardwareMap, "tilt right"),
                getAxon(hardwareMap, "tilt left").reversed()
        );

        gearSwitch = new SimpleServoPivot(ANGLE_SWITCH_INACTIVE, ANGLE_SWITCH_ENGAGED,
                switchR = getGBServo(hardwareMap, "switch right"),
                getGBServo(hardwareMap, "switch left").reversed()
        );
    }

    boolean isExtended() {
        return !liftSensor.isPressed();
    }

    public void runManual(double power) {
        this.manualPower = power;
    }

    void run() {

        position = 0.5 * (motors[0].encoder.getDistance() + motors[1].encoder.getDistance());

        switch (climbState) {
            case TILTING_SWITCHING:
                if (climbTimer.seconds() >= TIME_TILT_AND_SWITCH) climb();
                else break;

            case RAISING_SECOND_RUNG:
                if (climbTimer.seconds() >= TIME_RAISE_SECOND_RUNG) climb();
                else break;

            case SWINGING_SWITCHING:
                if (climbTimer.seconds() >= TIME_SWING_AND_SWITCH) climb();
                else break;

            case PULLING_SECOND_RUNG:
                if (climbTimer.seconds() >= TIME_PULL_SECOND_RUNG) climb();
                else break;
        }

        double output;

        // When the magnet hits
        if (!gearSwitch.isActivated() && getTarget() == 0 && !isExtended() && manualPower <= 0) {
            controller.reset();
            for (CachedMotorEx motor : motors) motor.encoder.reset();
            setTarget(position = 0);
            output = 0;
        } else if (manualPower != 0) {
            setTarget(getPosition());
            output = manualPower;
        } else if (!gearSwitch.isActivated() && getTarget() == 0 && isExtended() && getPosition() > 0 && getPosition() <= HEIGHT_RETRACTING) {
            output = SPEED_RETRACTION;
        } else {
            controller.setGains(gearSwitch.isActivated() ? dtPidGains : pidGains);
            controller.setTarget(new State(getTarget()));
            output = controller.calculate(new State(getPosition()));
        }

        output = clip(output, -1, 1);

        tilt.updateAngles(ANGLE_TILTER_INACTIVE, ANGLE_TILTER_TILTED);
        gearSwitch.updateAngles(ANGLE_SWITCH_INACTIVE, ANGLE_SWITCH_ENGAGED);
        switchR.offset = ANGLE_RIGHT_SWITCH_OFFSET;

        tilt.run();
        gearSwitch.run();

        if (gearSwitch.isActivated()) {
            double power = hold ? kG_CLIMB : output;
            dt.leftFront.setPower(power);
            dt.leftBack.setPower(power);
            dt.rightBack.setPower(power);
            dt.rightFront .setPower(power);
            for (CachedMotorEx motor : motors) motor.set(0);
        } else {
            double kG = getPosition() <= HEIGHT_START_kG ? 0 : Lift.kG * MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
            for (CachedMotorEx motor : motors) motor.set(output * SCALAR_DOWNWARD + kG);
        }
    }

    void printTelemetry() {
        mTelemetry.addData("LIFT", isExtended() ? "EXTENDED" : "RETRACTED");
        mTelemetry.addLine();
        mTelemetry.addData("Position (in)", getPosition());
        mTelemetry.addData("Target (in)", getTarget());
        mTelemetry.addData("Error derivative (in/s)", controller.getFilteredErrorDerivative());
        mTelemetry.addLine();
        mTelemetry.addData("Right encoder (ticks)", motors[0].encoder.getPosition());
        mTelemetry.addData("Left encoder (ticks)", motors[1].encoder.getPosition());
        mTelemetry.addLine();
        mTelemetry.addData("Climb state", climbState);
        divider();
        mTelemetry.addData("GEAR SWITCH", gearSwitch.isActivated() ? "ENGAGED" : "INACTIVE");
        divider();
        mTelemetry.addData("TILT", tilt.isActivated() ? "TILTED" : "INACTIVE");
    }

    public double getPosition() {
        return position;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double inches) {
        target = clip(inches, 0, HEIGHT_EXTENDED);
    }

    public boolean atPosition(double liftTarget) {
        return abs(liftTarget - getPosition()) < Lift.POSITION_TOLERANCE;
    }

}
