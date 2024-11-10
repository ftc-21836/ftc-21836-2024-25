package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;

@Config
public final class Lift {

    public static PIDGains pidGains = new PIDGains(
            0.5,
            0.4,
            0,
            1
    );

    public static double
            kG = 0.15,
            INCHES_PER_TICK = 0.0088581424,
            HEIGHT_RETRACTED_THRESHOLD = 0.5,
            MAX_VOLTAGE = 13;

    // Motors and variables to manage their readings:
    private final CachedMotorEx[] motors;
    private final PIDController controller = new PIDController();

    private final State ZERO_STATE = new State();

    private double position, target, manualLiftPower;

    // Battery voltage sensor and variable to track its readings:
    private final VoltageSensor batteryVoltageSensor;

    Lift(HardwareMap hardwareMap) {
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "lift right", RPM_312),
                new CachedMotorEx(hardwareMap, "lift left", RPM_312)
        };
        motors[1].setInverted(true);
        for (CachedMotorEx motor : motors) motor.setZeroPowerBehavior(FLOAT);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "right back", RPM_312).encoder;
        motors[1].encoder = new CachedMotorEx(hardwareMap, "left back", RPM_312).encoder;

//        motors[1].encoder.setDirection(REVERSE);

        reset();
    }

    public void reset() {
        controller.reset();
        for (CachedMotorEx motor : motors) motor.encoder.reset();
        setTarget(position = 0);
    }

    boolean isExtended() {
        return getPosition() > HEIGHT_RETRACTED_THRESHOLD;
    }

    public void setLiftPower(double manualLiftPower) {
        this.manualLiftPower = manualLiftPower;
    }

    void readSensors() {
        position = INCHES_PER_TICK * 0.5 * (motors[0].encoder.getPosition() + motors[1].encoder.getPosition());
        controller.setGains(pidGains);
    }

    void run(boolean freeToMove) {

        readSensors();

        if (manualLiftPower != 0) setTarget(getPosition()); // replace PID target with current state if using manual control

        controller.setTarget(freeToMove ? new State(getTarget()) : ZERO_STATE); // set PID target to 0 (retract) if intake isn't yet out of the way

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        double gravityFeedforward = isExtended() ? kG * voltageScalar : 0;

        double output = gravityFeedforward + (
                manualLiftPower != 0 ?                              // if manual input is being used:
                        manualLiftPower * voltageScalar :               // control with manual power (and voltage compensate)
                        controller.calculate(new State(getPosition()))  // control with PID output
        );

        for (CachedMotorEx motor : motors) motor.set(output);
    }

    void printTelemetry() {
        mTelemetry.addLine("LIFT: " + (isExtended() ? "EXTENDED" : "RETRACTED"));
        mTelemetry.addLine();
        mTelemetry.addData("Position (in)", getPosition());
        mTelemetry.addData("Target (in)", getTarget());
        mTelemetry.addData("Error derivative (in/s)", controller.getFilteredErrorDerivative());
    }

    double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    void setTarget(double inches) {
        target = inches;
    }

}
