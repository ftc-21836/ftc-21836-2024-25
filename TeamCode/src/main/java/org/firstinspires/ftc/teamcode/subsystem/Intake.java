package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
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
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
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
            TIME_BUCKET_SEMI_RETRACT = 0.2,
            TIME_PRE_TRANSFER = 0.15,
            TIME_TRANSFER = 0.15,

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
                    0.02
            ),
            maxRed = new HSV(
                    30,
                    0.75,
                    0.06
            ),
            minYellow = new HSV(
                    75,
                    0.6,
                    0.02
            ),
            maxYellow = new HSV(
                    96,
                    0.85,
                    0.3
            ),
            minBlue = new HSV(
                    215,
                    0.6,
                    0.02
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

    private final CachedSimpleServo[] bucket;
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

        bucket = new CachedSimpleServo[]{
                getAxon(hardwareMap, "bucket right").reversed(),
                getAxon(hardwareMap, "bucket left")
        };

        roller = hardwareMap.get(CRServo.class, "intake");

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket pivot sensor");
    }

    void run(Deposit deposit, boolean stopRoller) {

        boolean lookForSample = state == INTAKING || state == BUCKET_SEMI_RETRACTING || state == EXTENDO_RETRACTING;

        if (lookForSample) {
            colorSensor.update();
            sample = hsvToSample(hsv = colorSensor.getHSV());
        }

        if (lookForSample && !hasSample()) {

            if (state == INTAKING && rollerSpeed == 0) {
                state = RETRACTED;
                rollerSpeed = SPEED_RETRACTED;
            }
            else state = INTAKING;

            timer.reset();

        } else switch (state) {

            case EJECTING_SAMPLE:

                if (timer.seconds() >= TIME_EJECTING) state = INTAKING;
                else break;

            case INTAKING:

                if (getSample() == badSample) {
                    ejectSample();
                    break;
                }

                if (rollerSpeed == 0) {
                    state = BUCKET_SEMI_RETRACTING;
                    rollerSpeed = SPEED_HOLDING;
                    timer.reset();
                } else break;

            case BUCKET_SEMI_RETRACTING:

                if (timer.seconds() >= TIME_BUCKET_SEMI_RETRACT) {
                    state = EXTENDO_RETRACTING;
                    extendo.setExtended(false);
                } else break;

            case EXTENDO_RETRACTING:

                if (extendo.getPosition() <= Extendo.LENGTH_INTERFACING)
                    rollerSpeed = SPEED_INTERFACING;

                if (!extendo.isExtended() && deposit.readyToTransfer())
                    state = BUCKET_RETRACTING;
                else
                    break;

            case BUCKET_RETRACTING:

                extendo.setExtended(false);

                if (bucketSensor.isPressed()) {

                    rollerSpeed = SPEED_PRE_TRANSFER;
                    state = BUCKET_SETTLING;
                    timer.reset();

                } else break;

            case BUCKET_SETTLING:

                extendo.setExtended(false);

                if (timer.seconds() >= TIME_PRE_TRANSFER) transfer(deposit, getSample());
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

        for (CachedSimpleServo servo : bucket) servo.turnToAngle(
                state == EJECTING_SAMPLE ?          ANGLE_BUCKET_INTAKING :
                state == INTAKING ?                 lerp(ANGLE_BUCKET_OVER_BARRIER, ANGLE_BUCKET_INTAKING, abs(rollerSpeed)) :
                state == BUCKET_SEMI_RETRACTING ?   ANGLE_BUCKET_PRE_TRANSFER :
                state == EXTENDO_RETRACTING ?       ANGLE_BUCKET_PRE_TRANSFER :
                                                    ANGLE_BUCKET_RETRACTED
        );

        extendo.run(!deposit.requestingIntakeToMove() || state == TRANSFERRING);

        roller.setPower(
            stopRoller || (deposit.hasSample() && state == INTAKING) ? 0 :
            state == EJECTING_SAMPLE ? SPEED_EJECTING :
            rollerSpeed
        );
    }

    public void transfer(Deposit deposit, Sample sample) {
        state = TRANSFERRING;
        deposit.transfer(sample);
        this.sample = null;
        rollerSpeed = 0;
        timer.reset();
    }

    public void ejectSample() {
        sample = null;
        state = EJECTING_SAMPLE;
        timer.reset();
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
        return getSample() != null;
    }

    Sample getSample() {
        return sample;
    }

    boolean clearOfDeposit() {
        return extendo.getPosition() >= Extendo.LENGTH_DEPOSIT_CLEAR;
    }

    public void runRoller(double power) {
        if (power != 0) {
            state = INTAKING;
            sample = null;
        }
        if (state == INTAKING) rollerSpeed = power;
    }

    void printTelemetry() {
        mTelemetry.addData("INTAKE", state + ", " + (hasSample() ? getSample() + " sample" : "empty"));
        hsv.toTelemetry();
        divider();
        extendo.printTelemetry();
    }

}
