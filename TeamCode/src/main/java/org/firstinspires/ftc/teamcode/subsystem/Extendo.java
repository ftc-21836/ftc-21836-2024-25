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
            SPEED_RETRACTION = -0.5,
            LENGTH_RETRACTING = 15,
            LENGTH_DEPOSIT_CLEAR = 80,
            LENGTH_DEPOSIT_CLEAR_TOLERANCE = 5,
            LENGTH_EXTENDED = Math.MM_EXTENDED - Math.MM_RETRACTED,
            kS = 0;

    public static PIDGains pidGains = new PIDGains(
            0.015,
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
        position = 0;
        setTarget(0);
    }

    public void runManual(double power) {
        this.manualPower = power * SCALAR_MANUAL_SPEED;
    }

    public void run(boolean canRetract) {

        double setpoint = canRetract ? getTarget() : max(getTarget(), LENGTH_DEPOSIT_CLEAR + LENGTH_DEPOSIT_CLEAR_TOLERANCE);

        // When the magnet hits
        if (setpoint == 0 && !isExtended() && manualPower <= 0) reset();

        position = Math.millimeters(motor.encoder.getDistance() + Math.RAD_RETRACTED) - Math.MM_RETRACTED;

        if (manualPower != 0) {
            setTarget(getPosition());
            motor.set(manualPower);
            return;
        }

        if (setpoint == 0 && isExtended() && position > 0 && position <= LENGTH_RETRACTING) {
            motor.set(SPEED_RETRACTION);
            return;
        }

        controller.setGains(pidGains);
        controller.setTarget(new State(setpoint));

        double power = controller.calculate(new State(getPosition()));
        double slideBindingFF = power <= 0 ? 0 : position * kS;

        motor.set(power + slideBindingFF);
    }

    public void printTelemetry() {
        mTelemetry.addData("EXTENDO", isExtended() ? "EXTENDED" : "RETRACTED");
        mTelemetry.addLine();
        mTelemetry.addData("Position (mm)", getPosition());
        mTelemetry.addData("Target (mm)", getTarget());
        mTelemetry.addData("Error derivative (mm/s)", controller.getFilteredErrorDerivative());
        mTelemetry.addData("Encoder (rad)", motor.encoder.getDistance());
    }

    double getPosition() {
        return position;
    }

    double getTarget() {
        return target;
    }

    public void setTarget(double millimeters) {
        target = millimeters;
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

    /// <a href="https://www.desmos.com/calculator/xjwsz9pojl">Desmos diagrams + graphs</a>
    private static final class Math {

        private static final double
                A = 240,        A_2 = A*A,
                B = 360,        B_2 = B*B,
                H = 19.65017,   H_2 = H*H,
                B2_H2 = B_2 - H_2,
                AH2 = A * H * 2,
                b = 0.5 / A,                b_2 = b*b,
                a = b * (B_2 - A_2 - H_2),  a_2 = a*a,
                bh2a = 2 * b * H_2 + a,
                ab2 = a * b * 2,
                MM_RETRACTED = 144.39994388,
                MM_EXTENDED = 554.399942992,
                RAD_RETRACTED = radians(MM_RETRACTED),
                RAD_EXTENDED = radians(MM_EXTENDED),
                NORM_FACTOR = 1 / radiansPerMillimeter(MM_RETRACTED);

        private static double millimeters(double radians) {
            double sin = sin(radians);
            return sqrt(B2_H2 - AH2 * sin - A_2 * sin * sin) - A * cos(radians);
        }

        private static double radians(double millimeters) {
            double x_2 = millimeters*millimeters;
            return PI - asin(  (a - b*x_2) / sqrt(H_2 + x_2)  ) - atan(millimeters / H);
        }

        private static double radiansPerMillimeter(double millimeters) {
            double x_2 = millimeters*millimeters;
            double x2h2 = x_2 + H_2;
            double num = millimeters * (b*x_2 + bh2a);
            double denom = x2h2 * sqrt(x2h2 - a_2 + ab2 * x_2 - b_2 * x_2 * x_2);
            return num / denom - H / x2h2;
        }

        private static double scaledPower(double power, double millimeters) {
            return power * NORM_FACTOR * radiansPerMillimeter(millimeters) + (power <= 0 ? 0 : (millimeters - MM_RETRACTED) * kS);
        }

        public static void main(String[] args) {

            System.out.println(millimeters(RAD_RETRACTED) - 144.39994388);
            System.out.println(radians(MM_RETRACTED) - 0.89401173062);
            System.out.println(radiansPerMillimeter(MM_RETRACTED) - 0.0122469670588);

            System.out.println(scaledPower(1, MM_EXTENDED) - 0.445959977881);
            System.out.println(scaledPower(1, MM_RETRACTED) - 1);

        }

    }

}
