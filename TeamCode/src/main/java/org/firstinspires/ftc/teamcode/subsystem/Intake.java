package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_SEMI_RETRACTING;
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

            ANGLE_BUCKET_RETRACTED = 5,
            ANGLE_BUCKET_PRE_TRANSFER = 25,
            ANGLE_BUCKET_OVER_BARRIER = 150,
            ANGLE_BUCKET_INTAKING_NEAR = 206,
            ANGLE_BUCKET_INTAKING_FAR = 203,

            TIME_EJECTING = 0.5,
            TIME_SAMPLE_SETTLING = 1.5,
            TIME_BUCKET_SEMI_RETRACT = 1,
            TIME_PRE_TRANSFER = 0.25,
            TIME_TRANSFER = 0.25,
            TIME_POST_TRANSFER = 0.25,

            SPEED_EJECTING = -0.5,
            SPEED_POST_TRANSFER = -0.1,
            SPEED_PRE_TRANSFER = -0.1,
            SPEED_HOLDING = 0.5,
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

    public Intake(HardwareMap hardwareMap) {

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

        switch (state) {

            case EJECTING_SAMPLE:
                if (timer.seconds() >= TIME_EJECTING)
                    state = INTAKING;
                else
                    break;

            case INTAKING:

                if (switchIfSampleLost(INTAKING))
                    break;

                // Eject bad samples
                if (sample == badSample) {

                    sample = null;
                    rollerSpeed = SPEED_EJECTING;
                    state = EJECTING_SAMPLE;
                    timer.reset();
                    break;

                }

                // If our input is zero let's retract the intake
                if (rollerSpeed == 0)
                    setExtended(false);
                else
                    break;

            case BUCKET_SEMI_RETRACTING:
                if (switchIfSampleLost(INTAKING))
                    break;

                // Wait for the bucket to flip upward halfway
                if (timer.seconds() >= TIME_BUCKET_SEMI_RETRACT)
                    state = EXTENDO_RETRACTING;
                else
                    break;

            case EXTENDO_RETRACTING:
                if (switchIfSampleLost(INTAKING))
                    break;

                // Keep telling our extendo to retract
                extendo.setExtended(false);

                // As soon as it's retracted & deposit ready, finish flipping the bucket
                if (!extendo.isExtended() && deposit.readyToTransfer()) {

                    bucket.setActivated(false);
                    state = BUCKET_RETRACTING;
                    timer.reset();

                } else break;

            case BUCKET_RETRACTING:
                if (switchIfSampleLost(RETRACTED))
                    break;

                // While the bucket is retracting make sure our extendo stays put
                extendo.setExtended(false);

                // Wait for the limit switch to trigger
                if (bucketSensor.isPressed()) {

                    rollerSpeed = SPEED_PRE_TRANSFER;
                    state = BUCKET_SETTLING;
                    timer.reset();

                } else break;

            case BUCKET_SETTLING:
                if (switchIfSampleLost(RETRACTED))
                    break;

                // Keep the extendo put
                extendo.setExtended(false);

                // Wait for the transfer to happen
                if (timer.seconds() >= TIME_PRE_TRANSFER)
                    transfer(deposit, sample);
                else
                    break;

            case TRANSFERRING:
                // Extendo should stay put while transferring
                extendo.setExtended(false);

                // Wait a bit for the transfer to be considered complete
                if (timer.seconds() >= TIME_TRANSFER) {

                    rollerSpeed = SPEED_POST_TRANSFER;
                    state = RETRACTED;
                    timer.reset();

                } else break;

            case RETRACTED:
                // Wait a bit after a transfer before stopping the roller
                if (timer.seconds() >= TIME_POST_TRANSFER)
                    rollerSpeed = 0;

                break;
        }

        // The intaking angle of the bucket corrected for the extendo's tilt
        double correctedIntakingAngle =
                lerp(ANGLE_BUCKET_INTAKING_NEAR, ANGLE_BUCKET_INTAKING_FAR,
                        extendo.getPosition() / Extendo.LENGTH_EXTENDED);

        double bucketExtendedAngle;

        if (state == EJECTING_SAMPLE)
            // If we're ejecting a sample, go to max extension
            bucketExtendedAngle = correctedIntakingAngle;
        else {
            if (state == INTAKING)
                bucketExtendedAngle = lerp(ANGLE_BUCKET_OVER_BARRIER, correctedIntakingAngle, abs(rollerSpeed));
            else
                bucketExtendedAngle = ANGLE_BUCKET_PRE_TRANSFER;
        }

        bucket.updateAngles(ANGLE_BUCKET_RETRACTED, bucketExtendedAngle);
        bucket.run();

        // Tell the extendo whether it's clear to retract
        extendo.run(!deposit.activeNearIntake() || climbing || state == TRANSFERRING);

        roller.setPower(deposit.hasSample() ? 0 : rollerSpeed);
    }

    public void transfer(Deposit deposit, Sample sample) {
        state = TRANSFERRING;
        deposit.transfer(sample);
        this.sample = null;
        rollerSpeed = 0;
        timer.reset();
    }

    /**
     * Check if we lost a sample, and if we did, switch our state
     * @param returnTo the fallback state to switch to
     * @return whether or not we lost a sample (and switched state)
     */
    private boolean switchIfSampleLost(State returnTo) {
        colorSensor.update();
        sample = hsvToSample(hsv = colorSensor.getHSV());

        if (hasSample())
            return false;

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

        if (state == RETRACTED){
            // If we're trying to retract, we're already good
            if (!extend)
                return;

            bucket.setActivated(true);
            state = INTAKING;
            sample = null;
        } else if (state == EJECTING_SAMPLE || state == INTAKING){
            // We're already extended
            if (extend)
                return;


            if (hasSample()) {

                state = BUCKET_SEMI_RETRACTING;
                rollerSpeed = SPEED_HOLDING;
                timer.reset();
                return;

            }

            extendo.setExtended(false);
            state = RETRACTED;
            bucket.setActivated(false);
            rollerSpeed = 0;

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
