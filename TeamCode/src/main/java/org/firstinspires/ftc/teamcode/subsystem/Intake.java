package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EJECTING_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@Config
public final class Intake {

    public static double

            ANGLE_BUCKET_RETRACTED = 7.8,
            ANGLE_BUCKET_PRE_TRANSFER = 25,
            ANGLE_BUCKET_OVER_BARRIER = 165,
            ANGLE_BUCKET_INTAKING_NEAR = 207,
            ANGLE_BUCKET_INTAKING_FAR = 203,

            TIME_EJECTING = 0.5,
            TIME_SAMPLE_SETTLING = 1.5,
            TIME_PRE_TRANSFER = 0.25,
            TIME_TRANSFER = 0.25,
            TIME_POST_TRANSFER = 0.25,

            SPEED_EJECTING = -0.5,
            SPEED_POST_TRANSFER = -0.1,
            SPEED_HOLDING = 1,
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
//        roller.setDirection(REVERSE);

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket pivot sensor");
    }

    void run(boolean climbing, Deposit deposit) {


        if (state != EJECTING_SAMPLE && state != RETRACTED) {
            colorSensor.update();
            sample = hsvToSample(hsv = colorSensor.getHSV());
        }

        switch (state) {

            case EJECTING_SAMPLE:

                if (timer.seconds() >= TIME_EJECTING) state = INTAKING;
                else break;

            case INTAKING:

                if (sampleLost(INTAKING)) break;

                if (sample == badSample) {

                    sample = null;
                    rollerSpeed = SPEED_EJECTING;
                    state = EJECTING_SAMPLE;
                    timer.reset();
                    break;

                }

                if (timer.seconds() >= TIME_SAMPLE_SETTLING || rollerSpeed == 0) setExtended(false);
                else break;

            case EXTENDO_RETRACTING:

                if (sampleLost(INTAKING)) break;

                extendo.setExtended(false);

                if (!extendo.isExtended() && deposit.readyToTransfer()) {

                    bucket.setActivated(false);
                    state = BUCKET_RETRACTING;
                    timer.reset();

                } else break;

            case BUCKET_RETRACTING:

                if (sampleLost(RETRACTED)) break;

                extendo.setExtended(false);

                if (bucketSensor.isPressed()) {

                    state = BUCKET_SETTLING;
                    timer.reset();

                } else break;

            case BUCKET_SETTLING:

                if (sampleLost(RETRACTED)) break;

                extendo.setExtended(false);

                if (timer.seconds() >= TIME_PRE_TRANSFER) {

                    rollerSpeed = 0;
                    deposit.transfer(sample);
                    sample = null;
                    state = TRANSFERRING;
                    timer.reset();

                } else break;

            case TRANSFERRING:

                extendo.setExtended(false);

                if (timer.seconds() >= TIME_TRANSFER) {

                    rollerSpeed = SPEED_POST_TRANSFER;
                    state = RETRACTED;
                    timer.reset();

                } else break;

            case RETRACTED:

                if (timer.seconds() >= TIME_POST_TRANSFER) rollerSpeed = 0;

                break;
        }

        double ANGLE_BUCKET_INTAKING = lerp(ANGLE_BUCKET_INTAKING_NEAR, ANGLE_BUCKET_INTAKING_FAR, extendo.getPosition() / Extendo.LENGTH_EXTENDED);

        double ANGLE_BUCKET_EXTENDED =
                state == EJECTING_SAMPLE ? ANGLE_BUCKET_INTAKING :
                state == INTAKING ? lerp(ANGLE_BUCKET_OVER_BARRIER, ANGLE_BUCKET_INTAKING, abs(rollerSpeed)) :
                        ANGLE_BUCKET_PRE_TRANSFER;

        bucket.updateAngles(ANGLE_BUCKET_RETRACTED, ANGLE_BUCKET_EXTENDED);
        bucket.run();

        extendo.run(!deposit.activeNearIntake() || climbing || state == TRANSFERRING);

        roller.setPower(deposit.hasSample() ? 0 : rollerSpeed);
    }

    private boolean sampleLost(State returnTo) {
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

    boolean hasSample() {
        return sample != null;
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

//            case EXTENDO_RETRACTING:
//            case BUCKET_RETRACTING:
//            case BUCKET_SETTLING:
//                if (hasSample()) break;
            case RETRACTED:

                if (extend) {
                    bucket.setActivated(true);
//                    extendo.setExtended(true);
                    state = INTAKING;
                    sample = null;
                }

                break;

            case EJECTING_SAMPLE:
            case INTAKING:

                if (!extend) {
                    extendo.setExtended(false);
                    if (hasSample()) {

                        state = EXTENDO_RETRACTING;
                        rollerSpeed = SPEED_HOLDING;

                    } else {
                        state = RETRACTED;
                        bucket.setActivated(false);
                        rollerSpeed = 0;
                    }
                }

                break;

        }
    }

    public void toggle() {
        setExtended(state == RETRACTED);
    }

    void printTelemetry() {
        mTelemetry.addData("INTAKE", state + ", " + (hasSample() ? sample + " sample" : "empty"));
        hsv.toTelemetry();
        divider();
        extendo.printTelemetry();
    }

}
