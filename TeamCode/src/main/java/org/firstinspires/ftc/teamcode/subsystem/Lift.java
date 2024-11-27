package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;

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
    private final VoltageSensor batteryVoltageSensor;

    private double position, target, manualPower;

    Lift(HardwareMap hardwareMap) {
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "lift right", RPM_312),
                new CachedMotorEx(hardwareMap, "lift left", RPM_312),
//                new CachedMotorEx(hardwareMap, "lift 3", RPM_312)
        };
        motors[1].setInverted(true);
        for (CachedMotorEx motor : motors) motor.setZeroPowerBehavior(FLOAT);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "right back", RPM_312).encoder;
        motors[1].encoder = new CachedMotorEx(hardwareMap, "left back", RPM_312).encoder;
//        motors[2].encoder = new CachedMotorEx(hardwareMap, "left front", RPM_312).encoder;

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

    public void runManual(double power) {
        this.manualPower = power;
    }

    void run(boolean freeToMove, boolean climbing) {

        position = INCHES_PER_TICK * avgTicks();

        controller.setGains(pidGains);
        controller.setTarget(new State(getTarget()));

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        double output = !isExtended() || climbing ? 0 : kG * voltageScalar;

        if (manualPower != 0) {

            setTarget(getPosition());
            output += manualPower;

        } else if (freeToMove) {

            output += controller.calculate(new State(getPosition()));

        }

        for (CachedMotorEx motor : motors) motor.set(output);
    }

    private double avgTicks() {
        double sum = 0;
        for (CachedMotorEx motor : motors) sum += motor.encoder.getPosition();
        return sum / motors.length;
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
