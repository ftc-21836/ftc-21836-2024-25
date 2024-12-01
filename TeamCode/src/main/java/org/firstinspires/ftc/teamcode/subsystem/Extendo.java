package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static java.lang.Double.POSITIVE_INFINITY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Config
public final class Extendo {

    public static double
            SCALAR_MANUAL_SPEED = 1.0,
            LENGTH_DEPOSIT_CLEAR = 120,
            LENGTH_DEPOSIT_CLEAR_TOLERANCE = 20,
            LENGTH_EXTENDED = 410;

    public static PIDGains pidGains = new PIDGains(
            0.0225,
            0,
            0,
            POSITIVE_INFINITY
    );

    private final CachedMotorEx motor;
    private final PIDController controller = new PIDController();
    private final TouchSensor extendoSensor;

    private double position, target, manualPower;

    public Extendo(HardwareMap hardwareMap) {

        motor = new CachedMotorEx(hardwareMap, "extendo", RPM_117);
        motor.setInverted(true);

        motor.encoder = new CachedMotorEx(hardwareMap, "right front", RPM_312).encoder;
        motor.encoder.setDirection(REVERSE);

        extendoSensor = hardwareMap.get(TouchSensor.class, "extendo sensor");

        reset();
    }

    private void reset() {
        controller.reset();
        motor.encoder.reset();
        setTarget(position = 0);
    }

    public void runManual(double power) {
        this.manualPower = power * SCALAR_MANUAL_SPEED;
    }

    public void run() {

        if (getTarget() == 0 && !isExtended()) reset();

        position = motor.encoder.getPosition();

        if (manualPower != 0) {
            setTarget(getPosition());
            motor.set(manualPower);
            return;
        }

        controller.setGains(pidGains);
        controller.setTarget(new org.firstinspires.ftc.teamcode.control.motion.State(getTarget()));

        motor.set(controller.calculate(new org.firstinspires.ftc.teamcode.control.motion.State(getPosition())));
    }

    public void printTelemetry() {
        mTelemetry.addData("EXTENDO", isExtended() ? "EXTENDED" : "RETRACTED");
        mTelemetry.addLine();
        mTelemetry.addData("Position (ticks)", getPosition());
        mTelemetry.addData("Target (ticks)", getTarget());
        mTelemetry.addData("Error derivative (ticks/s)", controller.getFilteredErrorDerivative());
    }

    double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    void setTarget(double ticks) {
        target = ticks;
    }

    boolean isExtended() {
        return !extendoSensor.isPressed();
    }

    public void setExtended(boolean extended) {
        setTarget(extended ? LENGTH_EXTENDED : 0);
    }

    public void toggle() {
        setExtended(!isExtended());
    }

}
