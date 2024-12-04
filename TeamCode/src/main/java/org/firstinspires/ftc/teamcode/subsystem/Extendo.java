package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static java.lang.Double.POSITIVE_INFINITY;
import static java.lang.Double.max;
import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

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

    public void run(boolean canRetract) {

        if (getTarget() == 0 && !isExtended()) reset();

        position = motor.encoder.getPosition();

        if (manualPower != 0) {
            setTarget(getPosition());
            motor.set(manualPower);
            return;
        }

        controller.setGains(pidGains);
        controller.setTarget(new State(
                canRetract ? getTarget() : max(getTarget(), LENGTH_DEPOSIT_CLEAR + LENGTH_DEPOSIT_CLEAR_TOLERANCE)
        ));

        motor.set(controller.calculate(new State(getPosition())));
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

    public void setTarget(double ticks) {
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



    private static final double
            A = 240,    A_2 = A*A,
            B = 360,    B_2 = B*B,
            H = 19.65,  H_2 = H*H,
            b = 1 / (2 * A),
            a = b * (B_2 - A_2 - H_2),
            bh2a = 2 * b * H_2 + a;

    private static double x(double theta) {
        double rightLeg = H + A * sin(theta);
        return sqrt(B_2 - rightLeg*rightLeg) - A * cos(theta);
    }

    private static double theta(double x) {
        double x_2 = x*x;
        return PI - asin(  (a - b*x_2) / sqrt(H_2 + x_2)  ) - atan(x / H);
    }

    private static double omega(double x, double v) {
        double x_2 = x*x;
        double x2h2 = x_2 + H_2;
        double num = x * (b*x_2 + bh2a);
        double binom = a - b * x_2;
        double denom = x2h2 * sqrt(x2h2 - binom*binom);
        return v * (num / denom - H / x2h2);
    }

    public static void main(String... args) {

        System.out.println(x(toRadians(124.5)) - 422.85163444);
        System.out.println(toDegrees(theta(237)) - 86.9634939233);
        System.out.println(toDegrees(omega(445, 1)) - 0.195843409013);

    }

}
