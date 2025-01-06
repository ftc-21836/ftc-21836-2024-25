package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.mTelemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Config
public final class Lift {

    public static PIDGains pidGains = new PIDGains(
            0.5,
            0.4
    );

    public static double
            kG = 0.125,
            INCHES_PER_TICK = 0.0088581424,
            POSITION_TOLERANCE = 0.25,
            MAX_VOLTAGE = 13;

    // Motors and variables to manage their readings:
    private final CachedMotorEx[] motors;
    private final PIDController controller = new PIDController();
    private final VoltageSensor batteryVoltageSensor;

    private double position, target, manualPower;

    Lift(HardwareMap hardwareMap) {
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "lift right", RPM_312),
                new CachedMotorEx(hardwareMap, "lift left", RPM_312),
                new CachedMotorEx(hardwareMap, "lift 3", RPM_312)
        };
        motors[1].setInverted(true);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "right back", RPM_312).encoder;
        motors[1].encoder = new CachedMotorEx(hardwareMap, "left back", RPM_312).encoder;
//        motors[2].encoder = new CachedMotorEx(hardwareMap, "left front", RPM_312).encoder;

//        motors[1].encoder.setDirection(REVERSE);

        for (CachedMotorEx motor : motors) {
            motor.setZeroPowerBehavior(FLOAT);
            motor.encoder.setDistancePerPulse(INCHES_PER_TICK);
        }

        reset();
    }

    public void reset() {
        controller.reset();
        for (CachedMotorEx motor : motors) motor.encoder.reset();
        setTarget(position = 0);
    }

    boolean isExtended() {
        return getPosition() > POSITION_TOLERANCE;
    }

    public void runManual(double power) {
        this.manualPower = power;
    }

    void run(boolean canMove, boolean climbing) {

        position = 0.5 * (motors[0].encoder.getDistance() + motors[1].encoder.getDistance());

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        double kG = !isExtended() || climbing ? 0 : Lift.kG * voltageScalar;
        double output;

        if (manualPower != 0) {

            setTarget(getPosition());
            output = manualPower;

        } else {

            controller.setGains(pidGains);
            controller.setTarget(new State(canMove ? getTarget() : getPosition()));
            output = controller.calculate(new State(getPosition()));

        }

        for (CachedMotorEx motor : motors) motor.set(output + kG);
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

    double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    public void setTarget(double inches) {
        target = inches;
    }

    public boolean atPosition(double liftTarget) {
        return abs(liftTarget - getPosition()) < Lift.POSITION_TOLERANCE;
    }

}
