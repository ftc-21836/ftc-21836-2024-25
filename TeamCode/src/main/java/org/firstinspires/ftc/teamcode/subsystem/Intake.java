package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Extendo.LENGTH_DEPOSIT_CLEAR;
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

            ANGLE_BUCKET_RETRACTED = 11.55,
            ANGLE_BUCKET_VERTICAL = 90,
            ANGLE_BUCKET_FLOOR_CLEARANCE = 149.1,
            ANGLE_BUCKET_EJECTING = 204,
            ANGLE_BUCKET_INTAKING = 209.1,

            TIME_EJECTING = 0.5,
            TIME_DROP = 0.5,
            TIME_BUCKET_RAISE_TO_EXTEND = 0.15,
            TIME_BUCKET_RAISE_TO_DEPOSIT_LIFTING = 0.2,
            TIME_REVERSING = 0.175,
            TIME_PRE_TRANSFER = 0.25,
            TIME_TRANSFER = 0.25,
            TIME_POST_TRANSFER = 0.25,

            SPEED_EJECTING = -0.6,
            SPEED_POST_TRANSFER = -0.1,
            SPEED_HOLDING = 0.5,
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake bucket sample detection
     */
    public static HSV
            minRed = new HSV(
            0,
                    0.5,
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

    private final ColorSensor colorSensor;
    private HSV hsv = new HSV();
    private Sample sample, badSample;

    private final SimpleServoPivot bucket;
    private final TouchSensor bucketSensor;

    public final Extendo extendo;

    private Intake.State state = RETRACTED;

    private final ElapsedTime timer = new ElapsedTime();

    enum State {
        INTAKING,
        EJECTING_SAMPLE,
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
                ANGLE_BUCKET_VERTICAL,
                getAxon(hardwareMap, "bucket right").reversed(),
                getAxon(hardwareMap, "bucket left")
        );

        roller = hardwareMap.get(CRServo.class, "intake");
//        roller.setDirection(REVERSE);

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket pivot sensor");
    }

    interface Transferable{ void transfer(Sample sample); }

    void run(boolean depositHasSample, boolean depositActive, Transferable deposit) {

        switch (state) {

            case EJECTING_SAMPLE:

                roller.setPower(SPEED_EJECTING);

                if (timer.seconds() >= TIME_EJECTING) {
                    state = INTAKING;
                    roller.setPower(0);
                } else break;

            case INTAKING:

                colorSensor.update();
                sample = hsvToSample(hsv = colorSensor.getHSV());

                if (sample == badSample || (depositHasSample && this.hasSample())) {

                    sample = null;
                    state = EJECTING_SAMPLE;
                    timer.reset();
                    break;

                } else if (hasSample()) setExtended(false);
                else break;

            case EXTENDO_RETRACTING:

                roller.setPower(hasSample() ? SPEED_HOLDING : 0);

                if (!extendo.isExtended() && !depositActive) {

                    bucket.setActivated(false);
                    state = BUCKET_RETRACTING;
                    timer.reset();

                } else break;

            case BUCKET_RETRACTING:

                roller.setPower(hasSample() ? SPEED_HOLDING : 0);

                if (bucketSensor.isPressed()) {

                    roller.setPower(0);
                    state = BUCKET_SETTLING;
                    timer.reset();

                } else break;

            case BUCKET_SETTLING:

                roller.setPower(hasSample() ? SPEED_HOLDING : 0);

                if (!hasSample()) {

                    state = RETRACTED;
                    break;

                } else if (timer.seconds() >= TIME_PRE_TRANSFER) {

                    deposit.transfer(sample);
                    sample = null;
                    state = TRANSFERRING;
                    timer.reset();

                } else break;

            case TRANSFERRING:

                if (timer.seconds() >= TIME_TRANSFER) {

                    roller.setPower(SPEED_POST_TRANSFER);
                    state = RETRACTED;
                    timer.reset();

                } else break;

            case RETRACTED:

                if (timer.seconds() >= TIME_POST_TRANSFER) roller.setPower(0);

                extendo.setTarget(depositActive ? LENGTH_DEPOSIT_CLEAR : 0);

                break;
        }

        double ANGLE_BUCKET_EXTENDED =
                state == INTAKING ?
                        roller.getPower() == 0 ? ANGLE_BUCKET_FLOOR_CLEARANCE :
                        ANGLE_BUCKET_INTAKING :
                state == EJECTING_SAMPLE ? ANGLE_BUCKET_EJECTING :
                ANGLE_BUCKET_VERTICAL;

        bucket.updateAngles(ANGLE_BUCKET_RETRACTED, ANGLE_BUCKET_EXTENDED);
        bucket.run();

        extendo.run();
    }

    private boolean hasSample() {
        return sample != null;
    }

    boolean clearOfDeposit() {
        return extendo.getPosition() >= Extendo.LENGTH_DEPOSIT_CLEAR - Extendo.LENGTH_DEPOSIT_CLEAR_TOLERANCE;
    }

    public void runRoller(double power) {
        if (power != 0) setExtended(true);
        roller.setPower(state == INTAKING ? power : 0);
    }

    public void setExtended(boolean extend) {
        switch (state) {

            case EXTENDO_RETRACTING:
            case BUCKET_RETRACTING:
            case BUCKET_SETTLING:
            case RETRACTED:

                if (extend) {
                    bucket.setActivated(true);
                    extendo.setExtended(true);
                    state = INTAKING;
                }

                break;

            case INTAKING:
            case EJECTING_SAMPLE:

                if (!extend) {
                    extendo.setExtended(false);
                    state = EXTENDO_RETRACTING;
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
