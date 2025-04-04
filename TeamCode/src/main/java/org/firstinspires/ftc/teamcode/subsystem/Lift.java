package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ClimbState.INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ClimbState.PULLING_FIRST_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ClimbState.PULLING_SECOND_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ClimbState.RAISING_SECOND_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ClimbState.TILTING_SWITCHING;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import static java.lang.Double.max;
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
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Config
public final class Lift {

    enum ClimbState {
        INACTIVE,
        TILTING_SWITCHING,
        PULLING_FIRST_RUNG,
        RAISING_SECOND_RUNG,
        PULLING_SECOND_RUNG,
    }

    private ClimbState climbState = INACTIVE;
    private final ElapsedTime climbTimer = new ElapsedTime();

    public void climb() {
        switch (climbState) {
            case INACTIVE:
                if (getTarget() != HEIGHT_ABOVE_FIRST_RUNG) {
                    setTarget(HEIGHT_ABOVE_FIRST_RUNG);
                    break;
                }
                gearSwitch.setActivated(true);
                tilt.setActivated(true);
                climbState = TILTING_SWITCHING;
                break;
            case TILTING_SWITCHING:
                setTarget(0);
                climbState = PULLING_FIRST_RUNG;
                break;
            case PULLING_FIRST_RUNG:
                gearSwitch.setActivated(false);
                tilt.setActivated(false);
                setTarget(HEIGHT_ABOVE_SECOND_RUNG);
                climbState = RAISING_SECOND_RUNG;
                break;
            case RAISING_SECOND_RUNG:
                gearSwitch.setActivated(true);
                tilt.setActivated(true);
                setTarget(0);
                climbState = PULLING_SECOND_RUNG;
                break;
            case PULLING_SECOND_RUNG:
                gearSwitch.setActivated(false);
                tilt.setActivated(false);
                climbState = INACTIVE;
                break;
        }
        climbTimer.reset();
    }

    public static PIDGains
            pidGains = new PIDGains(0.05, 0.025),
            dtPidGains = new PIDGains(0.05, 0.025);

    public static double
            kG = 0.4,
            kG_CLIMB = -0.5,
            INCHES_PER_TICK = 0.0440162371,
            POSITION_TOLERANCE = 0.25,
            HEIGHT_RETRACTING = 0.25,
            HEIGHT_ABOVE_FIRST_RUNG = 10,
            HEIGHT_ABOVE_SECOND_RUNG = 15,
            SPEED_RETRACTION = -0,
            MAX_VOLTAGE = 13,
            ANGLE_TILTER_INACTIVE = 15,
            ANGLE_TILTER_TILTED = 120,
            ANGLE_SWITCH_INACTIVE = 15,
            ANGLE_SWITCH_ENGAGED = 30,
            TIME_TILT_AND_SWITCH = 10,
            TIME_RAISE_SECOND_RUNG = 10;

    public static Motor.ZeroPowerBehavior zeroPowerBehavior = BRAKE;

    // Motors and variables to manage their readings:
    private final CachedMotorEx[] motors;
    private final CachedMotorEx[] dtMotors;
    private final PIDController controller = new PIDController();
    private final VoltageSensor batteryVoltageSensor;
    private final TouchSensor liftSensor;

    public final SimpleServoPivot tilt, gearSwitch;

    private double position, target, manualPower;

    Lift(HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "lift right", RPM_1620),
                new CachedMotorEx(hardwareMap, "lift left", RPM_1620),
        };
        motors[0].setInverted(true);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "left front", RPM_1620).encoder;
        motors[1].encoder = new CachedMotorEx(hardwareMap, "left back", RPM_1620).encoder;

        liftSensor = hardwareMap.get(TouchSensor.class, "lift sensor");

        motors[1].encoder.setDirection(REVERSE);

        for (CachedMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            motor.encoder.setDistancePerPulse(INCHES_PER_TICK);
            motor.encoder.reset();
        }

        dtMotors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "left front", RPM_435),
                new CachedMotorEx(hardwareMap, "left back", RPM_435),
                new CachedMotorEx(hardwareMap, "right front", RPM_435),
                new CachedMotorEx(hardwareMap, "right back", RPM_435),
        };
        motors[0].setInverted(true);
        motors[1].setInverted(true);

        tilt = new SimpleServoPivot(ANGLE_TILTER_INACTIVE, ANGLE_TILTER_TILTED,
                getAxon(hardwareMap, "tilt right"),
                getAxon(hardwareMap, "tilt left").reversed()
        );

        gearSwitch = new SimpleServoPivot(ANGLE_SWITCH_INACTIVE, ANGLE_SWITCH_ENGAGED,
                getGBServo(hardwareMap, "switch right"),
                getGBServo(hardwareMap, "switch left").reversed()
        );
    }

    boolean isExtended() {
        return !liftSensor.isPressed();
    }

    public void runManual(double power) {
        this.manualPower = power;
    }

    void run(boolean canMove, boolean climbing) {

        double kG = !isExtended() || climbing ? 0 : Lift.kG * MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double setpoint = max(getTarget(), 0);

        position = 0.5 * (motors[0].encoder.getDistance() + motors[1].encoder.getDistance());

        double output;

        switch (climbState) {
            case TILTING_SWITCHING:
                if (climbTimer.seconds() >= TIME_TILT_AND_SWITCH) climb();
                else break;
            case PULLING_FIRST_RUNG:
                if (!isExtended()) climb();
                else break;
            case RAISING_SECOND_RUNG:
                if (climbTimer.seconds() >= TIME_RAISE_SECOND_RUNG) climb();
                else break;
        }

        // When the magnet hits
        if (!gearSwitch.isActivated() && setpoint == 0 && !isExtended() && manualPower <= 0) {
            controller.reset();
            for (CachedMotorEx motor : motors) motor.encoder.reset();
            setTarget(position = 0);
            output = 0;
        } else if (manualPower != 0) {
            setTarget(getPosition());
            output = manualPower;
        } else if (!gearSwitch.isActivated() && setpoint == 0 && isExtended() && getPosition() > 0 && getPosition() <= HEIGHT_RETRACTING) {
            output = SPEED_RETRACTION;
        } else {
            controller.setGains(gearSwitch.isActivated() ? dtPidGains : pidGains);
            controller.setTarget(new State(setpoint));
            output = controller.calculate(new State(getPosition()));
        }

        tilt.updateAngles(ANGLE_TILTER_INACTIVE, ANGLE_TILTER_TILTED);
        gearSwitch.updateAngles(ANGLE_SWITCH_INACTIVE, ANGLE_SWITCH_ENGAGED);
        tilt.run();
        gearSwitch.run();

        if (gearSwitch.isActivated()) {
            for (CachedMotorEx dtMotor : dtMotors) dtMotor.set(output + kG_CLIMB);
            for (CachedMotorEx motor : motors) {
                motor.set(0);
                motor.setZeroPowerBehavior(FLOAT);
            }
        } else for (CachedMotorEx motor : motors) {
            motor.set(output + kG);
            motor.setZeroPowerBehavior(zeroPowerBehavior);
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
    }

    public double getPosition() {
        return position;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double inches) {
        target = inches;
    }

    public boolean atPosition(double liftTarget) {
        return abs(liftTarget - getPosition()) < Lift.POSITION_TOLERANCE;
    }

}
