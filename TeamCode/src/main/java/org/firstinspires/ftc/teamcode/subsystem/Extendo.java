package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static java.lang.Double.max;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Config
public final class Extendo {

    public static double
            TOUCHPAD_RANGE = 0.9,
            INCHES_PER_TICK = 0.0440162371,
            LENGTH_RETRACTING = 1,
            SPEED_RETRACTION = -0,
            LENGTH_DEPOSIT_CLEAR = 4,
            POSITION_TOLERANCE = 0.6,
            LENGTH_EXTENDED = 21.25984251968504;

    public static PIDGains pidGains = new PIDGains(0, 0);

    private final CachedMotorEx motor;
    private final PIDController controller = new PIDController();
    private final TouchSensor extendoSensor;

    private double position, target, manualPower;

    public Extendo(HardwareMap hardwareMap) {

        motor = new CachedMotorEx(hardwareMap, "extendo", RPM_1620);
        motor.setInverted(true);

        motor.encoder = new CachedMotorEx(hardwareMap, "right back", RPM_1620).encoder;
        motor.encoder.setDirection(REVERSE);
        motor.encoder.setDistancePerPulse(INCHES_PER_TICK);
        motor.encoder.reset();

        extendoSensor = hardwareMap.get(TouchSensor.class, "extendo sensor");
    }

    public void runManual(double power) {
        this.manualPower = power;
    }

    public void run() {

        // When the magnet hits
        if (getTarget() == 0 && !isExtended() && manualPower <= 0) {
            controller.reset();
            motor.encoder.reset();
            position = 0;
            setTarget(0);

        } else position = max(0, motor.encoder.getDistance());

        if (manualPower != 0) {
            setTarget(getPosition());
            motor.set(manualPower);
            return;
        }

        if (getTarget() == 0 && isExtended() && getPosition() > 0 && getPosition() <= LENGTH_RETRACTING) {
            motor.set(SPEED_RETRACTION);
            return;
        }

        controller.setGains(pidGains);
        controller.setTarget(new State(getTarget()));

        motor.set(controller.calculate(new State(getPosition())));
    }

    public void printTelemetry() {
        mTelemetry.addData("EXTENDO", isExtended() ? "EXTENDED" : "RETRACTED");
        mTelemetry.addLine();
        mTelemetry.addData("Position (mm)", getPosition());
        mTelemetry.addData("Target (mm)", getTarget());
        mTelemetry.addData("Error derivative (mm/s)", controller.getFilteredErrorDerivative());
        mTelemetry.addData("Encoder (rad)", motor.encoder.getDistance());
    }

    public double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    public void setTarget(double millimeters) {
        target = clip(millimeters, 0, LENGTH_EXTENDED);
    }

    public boolean atPosition(double target) {
        return abs(target - getPosition()) < POSITION_TOLERANCE;
    }

    public void setWithTouchpad(double x) {
        setTarget(
                x < -TOUCHPAD_RANGE ? 0 :
                x > TOUCHPAD_RANGE ? LENGTH_EXTENDED :
                LENGTH_EXTENDED * 0.5 * (x + 1)
        );
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
