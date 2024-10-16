package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.BARE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.maxVoltage;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.AnalogEncoder;

@Config
public final class SwerveModule {

    public static double
            ANGLE_TOLERANCE = 0.2617993877991494,
            TEETH_MOTOR_PULLEY = 26,
            TEETH_TOP_PULLEY = 39,
            RATIO_TOP_PULLEY_TO_MOTOR = TEETH_TOP_PULLEY / TEETH_MOTOR_PULLEY,
            MAX_RPM_SERVO = 10,
            MAX_RPM_MOTOR = 5900,
            MAX_SERVO_TORQUE = 347,
            COAX_EFFECT_TORQUE = 20.45 * RATIO_TOP_PULLEY_TO_MOTOR,
            RATIO_RPM_SERVO_TO_MOTOR = MAX_RPM_SERVO / MAX_RPM_MOTOR,
            COAX_EFFECT_TORQUE_CORRECTION_GAIN = -COAX_EFFECT_TORQUE / MAX_SERVO_TORQUE,
            COAX_EFFECT_WHEEL_CORRECTION_GAIN = RATIO_TOP_PULLEY_TO_MOTOR * RATIO_RPM_SERVO_TO_MOTOR,
            kS_SERVO = 0,
            OFFSET_BR = 0,
            OFFSET_BL = 0,
            OFFSET_FR = 0,
            OFFSET_FL = 0;

    public static PIDGains thetaGains = new PIDGains(
            0,
            0,
            0,
            1
    );

    public enum SwerveModuleID {
        BR,
        BL,
        FR,
        FL;

        public final int o;
        public final String motorName, servoName, encoderName;

        SwerveModuleID() {
            this.o = ordinal();
            this.motorName = "motor " + name();
            this.servoName = "servo " + name();
            this.encoderName = "encoder " + name();
        }

        public double offset() {
            switch(this) {
                case BR: return OFFSET_BR;
                case BL: return OFFSET_BL;
                case FR: return OFFSET_FR;
                case FL: return OFFSET_FL;
            }
            return 0;
        }
    }

    private final SwerveModuleID id;
    private final SwerveModule.State current, target;

    private final MotorEx motor;
    private final CRServo servo;
    private final VoltageSensor batteryVoltageSensor;

    private final PIDController thetaController = new PIDController();

    private final AnalogEncoder thetaEncoder;

    public SwerveModule(HardwareMap hardwareMap, SwerveModuleID id) {

        this.id = id;

        this.motor = new MotorEx(hardwareMap, id.motorName, BARE);
        setZeroPowerBehavior(BRAKE);

        this.servo = new CRServo(hardwareMap, id.servoName);
        this.thetaEncoder = new AnalogEncoder(hardwareMap, id.encoderName, 2 * PI);

        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.target = this.current = new SwerveModule.State(0, 0);
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void readSensors() {
        current.theta = normalizeRadians(thetaEncoder.getPosition() - id.offset());
        thetaController.setGains(thetaGains);
    }

    public void setVelo(double velo) {
        target.velo = velo;
    }

    public void setTheta(double theta) {
        target.theta = normalizeRadians(theta);
    }

    public void run() {

        double scalar = maxVoltage / batteryVoltageSensor.getVoltage();

        target.optimize(current);

        double thetaError = normalizeRadians(target.theta - current.theta);
        double thetaTarget = thetaError + current.theta;

        thetaController.setTarget(new org.firstinspires.ftc.teamcode.control.motion.State(thetaTarget));

        double pidOutput = thetaController.calculate(new org.firstinspires.ftc.teamcode.control.motion.State(current.theta));
        
        double staticFF = kS_SERVO * signum(pidOutput) * scalar;

        double motorPower = abs(thetaError) > ANGLE_TOLERANCE ? 0 : target.velo;
        double servoPower = pidOutput + staticFF;

        double coaxWheelCorrection = servoPower * COAX_EFFECT_WHEEL_CORRECTION_GAIN;
        double coaxTorqueCorrection = motorPower * COAX_EFFECT_TORQUE_CORRECTION_GAIN;

        motor.set(motorPower + coaxWheelCorrection);
        servo.set(servoPower + coaxTorqueCorrection);
    }

    public static final class State {

        public double velo, theta;

        public State(double velo, double theta) {
            this.velo = velo;
            this.theta = normalizeRadians(theta);
        }

        private void optimize(SwerveModule.State real) {
            if (abs(normalizeRadians(real.theta - theta)) <= 0.5 * PI) return;
            velo *= -1;
            theta = normalizeRadians(theta + PI);
        }
    }
}
