package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.MainAuton.divider;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_SEMI_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EJECTING_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@Config
public final class Intake {

    public static double

            ANGLE_BUCKET_RETRACTED = 7.5,
            ANGLE_BUCKET_PRE_TRANSFER = 25,
            ANGLE_BUCKET_OVER_BARRIER = 140,
            ANGLE_BUCKET_INTAKING_NEAR = 213,
            ANGLE_BUCKET_INTAKING_FAR = 213,

            TIME_EJECTING = 0.5,
            TIME_SAMPLE_SETTLING = 1.5,
            TIME_BUCKET_SEMI_RETRACT = 0.2,
            TIME_PRE_TRANSFER = 0.15,
            TIME_TRANSFER = 0.15,
            TIME_REVERSING = 0.25,

            SPEED_EJECTING = -0.25,
            SPEED_HOLDING = 0.25,
            SPEED_INTERFACING = 0.25,
            SPEED_PRE_TRANSFER = -0.1,
            SPEED_POST_TRANSFER = -0.2,
            SPEED_RETRACTED = -0.05,
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake bucket sample detection
     */
    public static HSV
            minRed = new HSV(
            0,
                    0.25,
                    0
            ),
            maxRed = new HSV(
                    30,
                    0.75,
                    0.06
            ),
            minYellow = new HSV(
                    80,
                    0.6,
                    0
            ),
            maxYellow = new HSV(
                    96,
                    0.85,
                    0.3
            ),
            minBlue = new HSV(
                    215,
                    0.6,
                    0
            ),
            maxBlue = new HSV(
                    230,
                    0.9,
                    0.1
            );

    /**
     * @return The {@link Sample} corresponding to the provided {@link HSV} as per the tuned value bounds
     */
    public static Sample hsvToSample(HSV hsv) {
        return
                hsv.between(minRed, maxRed) ? RED :
                hsv.between(minBlue, maxBlue) ? BLUE :
                hsv.between(minYellow, maxYellow) ? NEUTRAL :
                null;
    }

    private final CRServo roller;
    private double rollerSpeed;

    private final ColorSensor colorSensor;
    private HSV hsv = new HSV();
    private Sample sample, badSample;

    private final SimpleServoPivot bucket;
    private final TouchSensor bucketSensor;

    public final Extendo extendo;

    private Intake.State state = RETRACTED;

    private final ElapsedTime timer = new ElapsedTime();

    enum State {
        EJECTING_SAMPLE,
        INTAKING,
        BUCKET_SEMI_RETRACTING,
        EXTENDO_RETRACTING,
        BUCKET_RETRACTING,
        BUCKET_SETTLING,
        TRANSFERRING,
        RETRACTED,
    }

    public void setAlliance(boolean redAlliance) {
        badSample = redAlliance ? BLUE : RED;
    }

    Intake(HardwareMap hardwareMap) {

        extendo = new Extendo(hardwareMap);

        bucket = new SimpleServoPivot(
                ANGLE_BUCKET_RETRACTED,
                ANGLE_BUCKET_PRE_TRANSFER,
                getAxon(hardwareMap, "bucket right").reversed(),
                getAxon(hardwareMap, "bucket left")
        );

        roller = hardwareMap.get(CRServo.class, "intake");

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket pivot sensor");
    }

    void run(Deposit deposit) {

        switch (state) {

            case EJECTING_SAMPLE:

                if (timer.seconds() >= TIME_EJECTING) state = INTAKING;
                else break;

            case INTAKING:

                if (sampleLost(INTAKING)) {
                    if (rollerSpeed == 0) setExtended(false);
                    break;
                }

                if (sample == badSample) {

                    sample = null;
                    rollerSpeed = SPEED_EJECTING;
                    state = EJECTING_SAMPLE;
                    timer.reset();
                    break;

                }

                if (rollerSpeed == 0) setExtended(false);
                else break;

            case BUCKET_SEMI_RETRACTING:

                if (sampleLost(INTAKING)) break;

                if (timer.seconds() >= TIME_BUCKET_SEMI_RETRACT) state = EXTENDO_RETRACTING;
                else break;

            case EXTENDO_RETRACTING:

                if (sampleLost(INTAKING)) break;

                extendo.setExtended(false);

                if (extendo.getPosition() <= Extendo.LENGTH_INTERFACING) {
                    rollerSpeed = SPEED_INTERFACING;
                }

                if (!extendo.isExtended() && deposit.readyToTransfer()) {

                    bucket.setActivated(false);
                    state = BUCKET_RETRACTING;
                    timer.reset();

                } else break;

            case BUCKET_RETRACTING:

                if (sampleLost(RETRACTED)) break;

                extendo.setExtended(false);

                if (bucketSensor.isPressed()) {

                    rollerSpeed = SPEED_PRE_TRANSFER;
                    state = BUCKET_SETTLING;
                    timer.reset();

                } else break;

            case BUCKET_SETTLING:

                if (sampleLost(RETRACTED)) break;

                extendo.setExtended(false);

                if (timer.seconds() >= TIME_PRE_TRANSFER) transfer(deposit, sample);
                else break;

            case TRANSFERRING:

                extendo.setExtended(false);

                if (timer.seconds() >= TIME_TRANSFER) {

                    rollerSpeed = SPEED_POST_TRANSFER;
                    state = RETRACTED;
                    timer.reset();

                } else break;

            case RETRACTED:

                if (clearOfDeposit()) rollerSpeed = SPEED_RETRACTED;

                break;
        }

        double ANGLE_BUCKET_INTAKING = lerp(ANGLE_BUCKET_INTAKING_NEAR, ANGLE_BUCKET_INTAKING_FAR, extendo.getPosition() / Extendo.LENGTH_EXTENDED);

        double ANGLE_BUCKET_EXTENDED =
                state == EJECTING_SAMPLE ? ANGLE_BUCKET_INTAKING :
                state == INTAKING ? lerp(ANGLE_BUCKET_OVER_BARRIER, ANGLE_BUCKET_INTAKING, abs(rollerSpeed)) :
                ANGLE_BUCKET_PRE_TRANSFER;

        bucket.updateAngles(ANGLE_BUCKET_RETRACTED, ANGLE_BUCKET_EXTENDED);
        bucket.run();

        boolean bucketDown = bucket.isActivated() && ANGLE_BUCKET_EXTENDED > 0.5 * (ANGLE_BUCKET_OVER_BARRIER + ANGLE_BUCKET_INTAKING_NEAR);

        extendo.run(!deposit.requestingIntakeToMove() || state == TRANSFERRING, bucketDown);

        roller.setPower(deposit.hasSample() && state == INTAKING ? 0 : rollerSpeed);
    }

    public void transfer(Deposit deposit, Sample sample) {
        state = TRANSFERRING;
        deposit.transfer(sample);
        this.sample = null;
        rollerSpeed = 0;
        timer.reset();
    }

    private boolean sampleLost(State returnTo) {
        colorSensor.update();
        sample = hsvToSample(hsv = colorSensor.getHSV());

        if (hasSample()) return false;

        state = returnTo;
        timer.reset();
        return true;
    }

    /**
     * Interpolate between two values
     * @param start starting value, returned if t = 0
     * @param end ending value, returned if t = 1
     * @param t interpolation parameter, on the inclusive interval [0, 1]
     * @return the interpolated value t% between start and end, on the inclusive interval [start, end]
     */
    public static double lerp(double start, double end, double t) {
        return (1 - t) * start + t * end;
    }

    public boolean hasSample() {
        return sample != null;
    }

    Sample getSample() {
        return sample;
    }

    boolean clearOfDeposit() {
        return extendo.getPosition() >= Extendo.LENGTH_DEPOSIT_CLEAR;
    }

    public void runRoller(double power) {
        if (power != 0) setExtended(true);
        if (state == INTAKING) rollerSpeed = power;
    }

    public void setExtended(boolean extend) {
        switch (state) {

            case RETRACTED:

                if (!extend) break;

                bucket.setActivated(true);
                state = INTAKING;
                sample = null;

                break;

            case EJECTING_SAMPLE:
            case INTAKING:

                if (extend) break;

                if (hasSample()) {

                    state = BUCKET_SEMI_RETRACTING;
                    rollerSpeed = SPEED_HOLDING;
                    timer.reset();
                    break;

                }

                state = RETRACTED;
                bucket.setActivated(false);
                rollerSpeed = SPEED_RETRACTED;

                break;

        }
    }

    void printTelemetry() {
        mTelemetry.addData("INTAKE", state + ", " + (hasSample() ? sample + " sample" : "empty"));
        hsv.toTelemetry();
        divider();
        extendo.printTelemetry();
    }

}
