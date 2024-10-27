package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;

@Config
public final class Lift {

    public static PIDGains pidGains = new PIDGains(
            0.3,
            0.2,
            0,
            0.2
    );

    public static double
            kG = 0.15,
            INCHES_PER_TICK = 0.0088581424,
            HEIGHT_RETRACTED_THRESHOLD = 0.5,
            MAX_VOLTAGE = 13;

    // Motors and variables to manage their readings:
    private final CachedMotorEx[] motors;
    private final Motor.Encoder encoder;
    double currentPosition, targetPosition;
    private final PIDController controller = new PIDController();

    private final State ZERO_STATE = new State();

    private double manualLiftPower;

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

        encoder = new CachedMotorEx(hardwareMap, "right back", RPM_312).encoder;

        reset();
    }

    public void reset() {
        controller.reset();
        encoder.reset();
        targetPosition = currentPosition = 0;
    }

    boolean isExtended() {
        return currentPosition > HEIGHT_RETRACTED_THRESHOLD;
    }

    public void setPosition(double inches) {
        this.targetPosition = inches;
    }

    public void setLiftPower(double manualLiftPower) {
        this.manualLiftPower = manualLiftPower;
    }

    void readSensors() {
        currentPosition = INCHES_PER_TICK * encoder.getPosition();
        controller.setGains(pidGains);
    }

    void run(boolean freeToMove) {

        if (manualLiftPower != 0) targetPosition = currentPosition; // replace PID target with current state if using manual control

        controller.setTarget(freeToMove ? new State(targetPosition) : ZERO_STATE); // set PID target to 0 (retract) if intake isn't yet out of the way

        boolean retracted = !isExtended();

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        double gravityFeedforward = retracted ? 0 : kG * voltageScalar;

        double output = gravityFeedforward + (
                manualLiftPower != 0 ?                              // if manual input is being used:
                        manualLiftPower * voltageScalar :               // control with manual power (and voltage compensate)
                        controller.calculate(new State(currentPosition))  // control with PID output
        );

        for (CachedMotorEx motor : motors) motor.set(output);
    }

    void printNumericalTelemetry() {
        mTelemetry.addData("Lift position (in)", currentPosition);
        mTelemetry.addData("Lift target (in)", targetPosition);
        mTelemetry.addLine();
        mTelemetry.addData("Lift error derivative (in/s)", controller.getFilteredErrorDerivative());
    }
}
