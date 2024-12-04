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
            RADIANS_DEPOSIT_CLEAR = 0.5290360909777928,
            RADIANS_DEPOSIT_CLEAR_TOLERANCE = 0.08817268182963214,
            RADIANS_EXTENDED = 1.8075399775074588;

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

        motor.encoder.setDistancePerPulse(2 * PI / motor.getCPR());

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

        position = motor.encoder.getDistance();

        if (manualPower != 0) {
            setTarget(getPosition());
            motor.set(manualPower);
            return;
        }

        controller.setGains(pidGains);
        controller.setTarget(new State(
                canRetract ? getTarget() : max(getTarget(), RADIANS_DEPOSIT_CLEAR + RADIANS_DEPOSIT_CLEAR_TOLERANCE)
        ));

        motor.set(controller.calculate(new State(getPosition())));
    }

    public void printTelemetry() {
        mTelemetry.addData("EXTENDO", isExtended() ? "EXTENDED" : "RETRACTED");
        mTelemetry.addLine();
        mTelemetry.addData("Position (rad)", getPosition());
        mTelemetry.addData("Target (rad)", getTarget());
        mTelemetry.addData("Error derivative (rad/s)", controller.getFilteredErrorDerivative());
    }

    double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    public void setTarget(double radians) {
        target = radians;
    }

    boolean isExtended() {
        return !extendoSensor.isPressed();
    }

    public void setExtended(boolean extended) {
        setTarget(extended ? RADIANS_EXTENDED : 0);
    }

    public void toggle() {
        setExtended(!isExtended());
    }



    private static final double
            RAD_RETRACTED = 0.894011730625,
            RAD_EXTENDED = 2.69343341362,
            MM_RETRACTED = 144.39994388,
            MM_EXTENDED = 554.399942992,
            A = 240,        A_2 = A*A,
            B = 360,        B_2 = B*B,
            H = 19.65017,   H_2 = H*H,
            b = 0.5 / A,
            a = b * (B_2 - A_2 - H_2),
            c = 2 * b * H_2 + a;

    private static double radToMM(double radians) {
        double rightLeg = H + A * sin(radians);
        return sqrt(B_2 - rightLeg*rightLeg) - A * cos(radians);
    }

    private static double mmToRadians(double millimeters) {
        double x_2 = millimeters*millimeters;
        return PI - asin(  (a - b*x_2) / sqrt(H_2 + x_2)  ) - atan(millimeters / H);
    }

    private static double mmPerSecToRadPerSec(double millimeters, double millimetersPerSecond) {
        double x_2 = millimeters*millimeters;
        double x2h2 = x_2 + H_2;
        double num = millimeters * (b*x_2 + c);
        double binom = a - b * x_2;
        double denom = x2h2 * sqrt(x2h2 - binom*binom);
        return millimetersPerSecond * (num / denom - H / x2h2);
    }

    public static void main(String[] args) {

        System.out.println(radToMM(RAD_RETRACTED) - MM_RETRACTED);
        System.out.println(mmToRadians(MM_RETRACTED) - RAD_RETRACTED);
        System.out.println(mmPerSecToRadPerSec(405, 1) - 0.00328031712158);

    }

}
