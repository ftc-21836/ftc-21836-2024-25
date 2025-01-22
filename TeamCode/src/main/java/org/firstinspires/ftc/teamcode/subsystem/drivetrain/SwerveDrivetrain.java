package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.drivetrain.SwerveModule.SwerveModuleID.BL;
import static org.firstinspires.ftc.teamcode.subsystem.drivetrain.SwerveModule.SwerveModuleID.BR;
import static org.firstinspires.ftc.teamcode.subsystem.drivetrain.SwerveModule.SwerveModuleID.FL;
import static org.firstinspires.ftc.teamcode.subsystem.drivetrain.SwerveModule.SwerveModuleID.FR;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.filter.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.HeadingIMU;

@Config
public final class SwerveDrivetrain {

    public static double MAX_VOLTAGE = 13;

    private final SwerveModule[] modules;

    private final VoltageSensor batteryVoltageSensor;

    public SwerveDrivetrain(HardwareMap hardwareMap) {

        modules = new SwerveModule[]{
                new SwerveModule(hardwareMap, BR),
                new SwerveModule(hardwareMap, BL),
                new SwerveModule(hardwareMap, FR),
                new SwerveModule(hardwareMap, FL),
        };

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TODO: adjust the names of the following hardware devices to match your configuration

        // TODO: if desired, use setLocalizer() to change the localization method
//        localizer = new ThreeWheelTrackingLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());

        setCurrentHeading(0);
    }

    public void update() {
//        rawHeading = imu.getHeading();

        modules[BR.o].readSensors();
        modules[BL.o].readSensors();
        modules[FR.o].readSensors();
        modules[FL.o].readSensors();

//        DriveSignal signal = trajectorySequenceRunner.update(localizer.getPoseEstimate(), localizer.getPoseVelocity());
//        if (signal != null) setDriveSignal(signal);
    }

    private void setModules(SwerveModule.State... states) {

        double vBR = states[BR.o].velo;
        double vBL = states[BL.o].velo;
        double vFR = states[FR.o].velo;
        double vFL = states[FL.o].velo;

        // Get max of all motor power commands
        double max = max(1.0, max(
                max(abs(vBR), abs(vBL)),
                max(abs(vFR), abs(vFL))
        ));

        // Normalize motor powers to [-1, 1]
        vBR /= max;
        vBL /= max;
        vFR /= max;
        vFL /= max;

        modules[BR.o].setVelo(vBR);
        modules[BL.o].setVelo(vBL);
        modules[FR.o].setVelo(vFR);
        modules[FL.o].setVelo(vFL);

        boolean moving =
                        vBR != 0 ||
                        vBL != 0 ||
                        vFR != 0 ||
                        vFL != 0;

        if (moving) {
            modules[BR.o].setTheta(states[BR.o].theta);
            modules[BL.o].setTheta(states[BL.o].theta);
            modules[FR.o].setTheta(states[FR.o].theta);
            modules[FL.o].setTheta(states[FL.o].theta);
        }

        modules[BR.o].run();
        modules[BL.o].run();
        modules[FR.o].run();
        modules[FL.o].run();
    }

    private double headingOffset, rawHeading;
    public static double SLOW_FACTOR = 0.3, ACCEL_LIMIT = 10000;
    private boolean slowModeLocked = false;

    private final SlewRateLimiter
            xRateLimiter = new SlewRateLimiter(ACCEL_LIMIT),
            yRateLimiter = new SlewRateLimiter(ACCEL_LIMIT);

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in radians, 0 facing forward and increases counter-clockwise
     */
    public void setCurrentHeading(double angle) {
        headingOffset = normalizeRadians(getRawHeading() - angle);
    }

    public double getHeading() {
        return normalizeRadians(getRawHeading() - headingOffset);
    }

    private double getRawHeading() {
        return rawHeading;
    }

    /**
     * Field-centric driving using {@link HeadingIMU}
     *
     * @param xCommand strafing input
     * @param yCommand forward input
     * @param turnCommand turning input
     */
    public void run(double xCommand, double yCommand, double turnCommand, boolean useSlowMode) {

        xRateLimiter.setLimit(ACCEL_LIMIT);
        yRateLimiter.setLimit(ACCEL_LIMIT);

        xCommand = xRateLimiter.calculate(xCommand);
        yCommand = yRateLimiter.calculate(yCommand);

        // counter-rotate translation vector by current heading
        double
                theta = -getHeading(),
                cos = cos(theta),
                sin = sin(theta),
                x = xCommand,
                y = yCommand;

        xCommand = x * cos - y * sin;
        yCommand = y * cos + x * sin;

        if (useSlowMode) slowModeLocked = false;
        if (useSlowMode || slowModeLocked) {
            yCommand *= SLOW_FACTOR;
            xCommand *= SLOW_FACTOR;
            turnCommand *= SLOW_FACTOR;
        }

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        yCommand *= voltageScalar;
        xCommand *= voltageScalar;
        turnCommand *= voltageScalar;

        // run motors
//        setDrivePower(new Pose2d(
//                yCommand,
//                -xCommand,
//                -turnCommand
//        ));
    }

    public void lockSlowMode() {
        this.slowModeLocked = true;
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("Current heading (radians)", getHeading());
        mTelemetry.addData("Current heading (degrees)", toDegrees(getHeading()));
    }

    @Config
    public static final class SwerveKinematics {

        public static double
                WIDTH = 10,
                LENGTH = 10;

        public static SwerveModule.State[] robotToPodStates(EditablePose drive) {

            double

            iY = drive.x,
            iX = drive.y,
            t = drive.heading / hypot(WIDTH, LENGTH),

            // Calculate rotation component vectors
            tY = t * LENGTH,
            tX = t * WIDTH,

            // Combine component vectors into total x and y vectors (per pod)
            a = iY - tY,
            b = iY + tY,
            c = iX - tX,
            d = iX + tX,

            // Get velocity vector (hypotenuse) of total x and y vectors (per pod)
            vBR = hypot(a, d),
            vBL = hypot(a, c),
            vFR = hypot(b, d),
            vFL = hypot(b, c),

            // Calculate pod angles with total x and y vectors (per pod)
            aBR = atan2(a, d),
            aBL = atan2(a, c),
            aFR = atan2(b, d),
            aFL = atan2(b, c);

            return new SwerveModule.State[]{
                new SwerveModule.State(vBR, aBR),
                new SwerveModule.State(vBL, aBL),
                new SwerveModule.State(vFR, aFR),
                new SwerveModule.State(vFL, aFL),
            };
        }
    }
}
