package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.mTelemetry;
import static java.lang.Double.max;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Config
public final class Extendo {

    public static double
            SPEED_RETRACTION = -0.75,
            TOUCHPAD_RANGE = 0.9,
            LENGTH_RETRACTING = 20,
            LENGTH_INTERFACING = 10,
            LENGTH_BUCKET_DOWN = 50,
            LENGTH_DEPOSIT_CLEAR = 100,
            POSITION_TOLERANCE = 15,
            LENGTH_EXTENDED = Math.MM_EXTENSION,
            kS = 0;

    public static PIDGains pidGains = new PIDGains(0.015, 0.01);

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
        motor.encoder.reset();

        extendoSensor = hardwareMap.get(TouchSensor.class, "extendo sensor");
    }

    public void runManual(double power) {
        this.manualPower = power;
    }

    public void run(boolean canRetract, boolean bucketDown) {

        double setpoint = max(getTarget(),
                !canRetract ?   LENGTH_DEPOSIT_CLEAR + POSITION_TOLERANCE :
                bucketDown ?    LENGTH_BUCKET_DOWN :
                                0
        );

        // When the magnet hits
        if (setpoint == 0 && !isExtended() && manualPower <= 0) {
            controller.reset();
            motor.encoder.reset();
            position = 0;
            setTarget(0);
        } else {
            position = Math.millimeters(max(0, motor.encoder.getDistance()) + Math.RAD_RETRACTED) - Math.MM_RETRACTED;
        }

        if (manualPower != 0) {
            setTarget(getPosition());
            motor.set(manualPower);
            return;
        }

        if (setpoint == 0 && isExtended() && getPosition() > 0 && getPosition() <= LENGTH_RETRACTING) {
            motor.set(SPEED_RETRACTION);
            return;
        }

        controller.setGains(pidGains);
        controller.setTarget(new State(setpoint));

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

    /// <a href="https://www.desmos.com/calculator/guflnpad5a">Desmos diagrams + graphs</a>
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
                MM_EXTENSION = 410,
                MM_EXTENDED = MM_RETRACTED + MM_EXTENSION,
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
