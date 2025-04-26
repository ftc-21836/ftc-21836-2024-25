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
            INCHES_PER_TICK = 0.0440162371 * (16/24.0),
            LENGTH_RETRACTING = 2,
            SPEED_RETRACTION = -.85,
            LENGTH_DEPOSIT_CLEAR = 4,
            POSITION_TOLERANCE = 0.6,
            LENGTH_EXTENDED = 21.25984251968504;

    public static PIDGains pidGains = new PIDGains(0.3, 0.2);

    private final CachedMotorEx motor;
    private final PIDController controller = new PIDController();
    private final TouchSensor extendoSensor;

    public double powerCap = 1;

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

        motor.set(clip(controller.calculate(new State(getPosition())), -powerCap, powerCap));
    }

    public void printTelemetry() {
        mTelemetry.addData("EXTENDO", isExtended() ? "EXTENDED" : "RETRACTED");
        mTelemetry.addLine();
        mTelemetry.addData("Extendo position (in)", getPosition());
        mTelemetry.addData("Extendo target (in)", getTarget());
        mTelemetry.addData("Extendo error derivative (in/s)", controller.getFilteredErrorDerivative());
    }

    public double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    public void setTarget(double inches) {
        target = clip(inches, 0, LENGTH_EXTENDED);
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

    public boolean isExtended() {
        return !extendoSensor.isPressed();
    }

    public void setExtended(boolean extended) {
        setTarget(extended ? LENGTH_EXTENDED : 0);
    }

    public void toggle() {
        setExtended(!isExtended());
    }

}
