package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.State.BUCKET_PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.State.BUCKET_RAISING;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.State.DROPPING_BAD_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.State.EXTENDO_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;
import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

@Config
public final class Intake {

    public static double
            SPEED_MULTIPLIER_EXTENDO = 0.1,
            DISTANCE_EXTENDO_ARMS_RETRACTED = 67.4,
            DISTANCE_EXTENDO_LINKAGE_ARM = 240,
            DISTANCE_EXTENDO_RETRACTED = 0,
            DISTANCE_EXTENDO_EXTENDED_MIN = 25.19855,
            DISTANCE_EXTENDO_EXTENDED_MAX = 410,
            ANGLE_EXTENDO_RETRACTED = extensionToAngle(DISTANCE_EXTENDO_RETRACTED),
            ANGLE_EXTENDO_EXTENDED_MAX = extensionToAngle(DISTANCE_EXTENDO_EXTENDED_MAX),
            ANGLE_BUCKET_RETRACTED = 9,
            ANGLE_BUCKET_INTAKING = 196.5,
            ANGLE_BUCKET_FLOOR_CLEARANCE = 60,
            ANGLE_BUCKET_VERTICAL = 90,
            ANGLE_LATCH_TRANSFERRING = 0,
            ANGLE_LATCH_INTAKING = 105,
            ANGLE_LATCH_LOCKED = 159,
            TIME_BUCKET_PIVOT = 1,
            TIME_DROP = 1,
            TIME_BUCKET_RAISE_TO_EXTEND = 1,
            TIME_BUCKET_RAISE_TO_DEPOSIT_LIFTING = 1,
            TIME_REVERSING = 0.175,
            SPEED_REVERSING = -0.6,
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake pixel detection
     */
    public static HSV
            minRed = new HSV(
                    20,
                    0.5,
                    0.025
            ),
            maxRed = new HSV(
                    30,
                    0.75,
                    0.06
            ),
            minYellow = new HSV(
                    83,
                    0.68,
                    0.2
            ),
            maxYellow = new HSV(
                    93,
                    0.76,
                    0.2
            ),
            minBlue = new HSV(
                    218,
                    0.75,
                    0.02
            ),
            maxBlue = new HSV(
                    226,
                    0.85,
                    0.04
            );

    /**
     * @return The {@link Sample} corresponding to the provided {@link HSV} as per the tuned value bounds
     */
    private static Sample hsvToSample(HSV hsv) {
        return
                hsv.between(minRed, maxRed) ? RED :
                hsv.between(minBlue, maxBlue) ? BLUE :
                hsv.between(minYellow, maxYellow) ? NEUTRAL :
                null;
    }

    private final CachedMotorEx motor;

    private final ColorSensor colorSensor;
    private HSV hsv = new HSV();
    private Sample sample, badSample;

    private final TouchSensor bucketSensor, extendoSensor;

    private final SimpleServoPivot bucket, latch, extendo;

    private Intake.State state = RETRACTED;

    private final ElapsedTime timer = new ElapsedTime(), timeSinceBucketRetracted = new ElapsedTime();

    private double extendedLength, extendedAngle;

    enum State {
        RETRACTED,
        BUCKET_RAISING,
        INTAKING,
        BUCKET_PIVOTING,
        DROPPING_BAD_SAMPLE,
        EXTENDO_RETRACTING,
    }

    public void setAlliance(boolean redAlliance) {
        badSample = redAlliance ? BLUE : RED;
    }

    Intake(HardwareMap hardwareMap) {

        resetExtendedLength();

        bucket = new SimpleServoPivot(
                ANGLE_BUCKET_RETRACTED,
                ANGLE_BUCKET_VERTICAL,
                getAxonServo(hardwareMap, "bucket right"),
                getReversedServo(getAxonServo(hardwareMap, "bucket left"))
        );

        latch = new SimpleServoPivot(
                ANGLE_LATCH_TRANSFERRING,
                ANGLE_LATCH_LOCKED,
                getGoBildaServo(hardwareMap, "latch")
        );

        extendo = new SimpleServoPivot(
                ANGLE_EXTENDO_RETRACTED,
                extendedAngle,
                getGoBildaServo(hardwareMap, "extendo right"),
                getReversedServo(getGoBildaServo(hardwareMap, "extendo left"))
        );

        motor = new CachedMotorEx(hardwareMap, "intake", RPM_1620);
        motor.setZeroPowerBehavior(FLOAT);
        motor.setInverted(true);

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket pivot sensor");
        extendoSensor = hardwareMap.get(TouchSensor.class, "extendo sensor");

        timer.reset();
    }

    void run(boolean depositHasSample, boolean depositIsActive) {

        switch (state) {

            case RETRACTED:

                bucket.setActivated(depositIsActive || (depositHasSample && hasSample()));
                break;

            case BUCKET_RAISING:

                if (timer.seconds() >= TIME_BUCKET_RAISE_TO_EXTEND) {
                    extendo.setActivated(true);
                    state = INTAKING;
                } else break;

            case INTAKING:

                colorSensor.update();
                hsv = colorSensor.getHSV();
                sample = hsvToSample(hsv);      // if color sensor found sample, hasSample() returns true

//                if (sample == badSample) {
//                    latch.setActivated(true);
//                    bucket.setActivated(false);
//                    state = BUCKET_PIVOTING;
//                    timer.reset();
//                } else {
//                    if (hasSample()) setExtended(false);
                    break;
//                }

            case BUCKET_PIVOTING:

                if (timer.seconds() >= TIME_BUCKET_PIVOT) {
                    releaseSample();
                    state = DROPPING_BAD_SAMPLE;
                    timer.reset();
                } else break;

            case DROPPING_BAD_SAMPLE:

                if (timer.seconds() >= TIME_DROP) {
                    state = INTAKING;
                    bucket.setActivated(true);
                }

                break;

            case EXTENDO_RETRACTING:

                if (extendoSensor.isPressed()) state = RETRACTED;
                else if (timer.seconds() <= TIME_REVERSING) motor.set(SPEED_REVERSING);
        }

        if (isRetracted()) timeSinceBucketRetracted.reset();

        double ANGLE_BUCKET_DOWN = state == INTAKING ?
                ANGLE_BUCKET_INTAKING - (motor.get() == 0 ? ANGLE_BUCKET_FLOOR_CLEARANCE : 0) :
                ANGLE_BUCKET_VERTICAL;

        bucket.updateAngles(ANGLE_BUCKET_RETRACTED, ANGLE_BUCKET_DOWN);

        double ANGLE_LATCH_UNLOCKED = state == INTAKING ? ANGLE_LATCH_INTAKING : ANGLE_LATCH_TRANSFERRING;

        latch.updateAngles(ANGLE_LATCH_UNLOCKED, ANGLE_LATCH_LOCKED);

        extendo.updateAngles(ANGLE_EXTENDO_RETRACTED, extendedAngle);

        bucket.run();
        latch.run();
        extendo.run();
    }

    private boolean hasSample() {
        return sample != null;
    }

    boolean clearOfDeposit() {
        return timeSinceBucketRetracted.seconds() >= TIME_BUCKET_RAISE_TO_DEPOSIT_LIFTING;
    }

    boolean awaitingTransfer() {
        return hasSample() && isRetracted();
    }

    private boolean isRetracted() {
        return bucketSensor.isPressed() && extendoSensor.isPressed();
    }

    Sample releaseSample() {
        Sample releasedSample = sample;
        sample = null;
        latch.setActivated(false);
        return releasedSample;
    }

    private void resetExtendedLength() {
        extendedLength = DISTANCE_EXTENDO_EXTENDED_MAX;
        extendedAngle = ANGLE_EXTENDO_EXTENDED_MAX;
    }

    public void offsetExtension(double offset) {
        if (offset == 0) return;

        extendedLength = clip(
                extendedLength + offset * SPEED_MULTIPLIER_EXTENDO,
                DISTANCE_EXTENDO_EXTENDED_MIN,
                DISTANCE_EXTENDO_EXTENDED_MAX
        );

        extendedAngle = extensionToAngle(extendedLength);
    }

    public void setMotorPower(double motorPower) {
        if (motorPower != 0) setExtended(true);
        motor.set(state == INTAKING ? motorPower : 0);
    }

    public void setExtended(boolean extend) {

        if (extend) {

            if (state == RETRACTED) {

                bucket.setActivated(true);
                state = BUCKET_RAISING;
                timer.reset();

            }

        } else if (state == INTAKING) {

            if (hasSample()) latch.setActivated(true);
            state = EXTENDO_RETRACTING;
            extendo.setActivated(false);
            resetExtendedLength();
            timer.reset();
        }

    }

    public void toggle() {
        setExtended(state == RETRACTED);
    }

    static double extensionToAngle(double millimeters) {
        return 180 / PI * asin(0.5 * (clip(millimeters, DISTANCE_EXTENDO_RETRACTED, DISTANCE_EXTENDO_EXTENDED_MAX) + DISTANCE_EXTENDO_ARMS_RETRACTED) / DISTANCE_EXTENDO_LINKAGE_ARM);
    }

    static double angleToExtension(double theta) {
        return 2 * DISTANCE_EXTENDO_LINKAGE_ARM * sin(PI / 180 * clip(theta, ANGLE_EXTENDO_RETRACTED, ANGLE_EXTENDO_EXTENDED_MAX)) - DISTANCE_EXTENDO_ARMS_RETRACTED;
    }

    void printTelemetry() {
        mTelemetry.addLine("INTAKE: " + state);
        mTelemetry.addLine(hasSample() ? "Contains " + sample.name() + " sample" : "Empty");
        hsv.toTelemetry("Bucket");
    }
}
