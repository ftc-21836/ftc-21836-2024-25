package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.SPEED_INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_SEMI_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EJECTING_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.STANDBY;
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
            ANGLE_BUCKET_INTAKING_NEAR = 211,
            ANGLE_BUCKET_INTAKING_FAR = 211,

            TIME_EJECTING = 0.5,
            TIME_BUCKET_SEMI_RETRACT = 0.2,
            TIME_PRE_TRANSFER = 0.15,
            TIME_TRANSFER = 0.2,

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
                    0.005
            ),
            maxYellow = new HSV(
                    96,
                    0.85,
                    0.3
            ),
            minBlue = new HSV(
                    215,
                    0.6,
                    0.008
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

    public boolean retractBucketBeforeExtendo = true;

    private final CRServo roller;
    private double rollerSpeed;

    private final ColorSensor colorSensor;
    private HSV hsv = new HSV();
    private Sample sample, badSample = RED;

    private final CachedSimpleServo bucketR, bucketL;

    private void setBucket(double angle) {
        bucketR.turnToAngle(angle);
        bucketL.turnToAngle(angle);
    }
    
    private final TouchSensor bucketSensor;

    public final Extendo extendo;

    private Intake.State state = STANDBY;

    private final ElapsedTime timer = new ElapsedTime();

    enum State {
        EJECTING_SAMPLE,
        STANDBY,
        BUCKET_SEMI_RETRACTING,
        EXTENDO_RETRACTING,
        BUCKET_RETRACTING,
        BUCKET_SETTLING,
        TRANSFERRING,
    }

    public void setAlliance(boolean redAlliance) {
        badSample = redAlliance ? BLUE : RED;
    }

    Intake(HardwareMap hardwareMap) {

        extendo = new Extendo(hardwareMap);

        bucketR = getAxon(hardwareMap, "bucket right").reversed();
        bucketL = getAxon(hardwareMap, "bucket left");

        roller = hardwareMap.get(CRServo.class, "intake");

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket pivot sensor");
    }

    void run(Deposit deposit, boolean stopRoller) {

        double ANGLE_BUCKET_INTAKING = lerp(ANGLE_BUCKET_INTAKING_NEAR, ANGLE_BUCKET_INTAKING_FAR, extendo.getPosition() / Extendo.LENGTH_EXTENDED);

        switch (state) {

            case EJECTING_SAMPLE:

                setBucket(ANGLE_BUCKET_INTAKING);
                roller.setPower(stopRoller ? 0 : SPEED_EJECTING);

                if (timer.seconds() >= TIME_EJECTING) state = STANDBY;
                else break;

            case STANDBY:

                if (rollerSpeed != 0 && !stopRoller && !deposit.hasSample()) { // intaking, trigger held down

                    setBucket(lerp(ANGLE_BUCKET_OVER_BARRIER, ANGLE_BUCKET_INTAKING, abs(rollerSpeed)));
                    roller.setPower(rollerSpeed / SPEED_INTAKING);
                    
                    colorSensor.update();
                    sample = hsvToSample(hsv = colorSensor.getHSV());

                    if (getSample() == badSample) ejectSample();

                    break;
                    
                } else if (!hasSample()) { // retracted

                    setBucket(ANGLE_BUCKET_RETRACTED);
                    roller.setPower(
                        stopRoller ? 0 :
                        deposit.hasSample() && !clearOfDeposit() && deposit.requestingIntakeToMove() ? SPEED_POST_TRANSFER :
                        deposit.arm.movingNearIntake() ? SPEED_RETRACTED : 0
                    );

                    break;

                } else transfer(sample); // trigger released, sample acquired, initiate transfer

            case BUCKET_SEMI_RETRACTING:

                setBucket(ANGLE_BUCKET_PRE_TRANSFER);
                roller.setPower(stopRoller ? 0 : SPEED_HOLDING);

                if (timer.seconds() >= TIME_BUCKET_SEMI_RETRACT || bucketSensor.isPressed() || !retractBucketBeforeExtendo)
                    state = EXTENDO_RETRACTING;
                else break;

            case EXTENDO_RETRACTING:

                setBucket(ANGLE_BUCKET_PRE_TRANSFER);
                roller.setPower(
                    stopRoller ? 0 : 
                    extendo.getPosition() > Extendo.LENGTH_INTERFACING ? SPEED_HOLDING :
                    SPEED_INTERFACING
                );
                extendo.setExtended(false);

                if (!extendo.isExtended() && deposit.readyToTransfer())
                    state = BUCKET_RETRACTING;
                else break;

            case BUCKET_RETRACTING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.setPower(stopRoller ? 0 : SPEED_INTERFACING);
                extendo.setExtended(false);

                if (bucketSensor.isPressed()) {
                    state = BUCKET_SETTLING;
                    timer.reset();
                } else break;

            case BUCKET_SETTLING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.setPower(stopRoller ? 0 : SPEED_PRE_TRANSFER);
                extendo.setExtended(false);

                if (timer.seconds() >= TIME_PRE_TRANSFER) {
                    state = TRANSFERRING;
                    deposit.transfer(sample);
                    sample = null;
                    timer.reset();
                } else break;

            case TRANSFERRING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.setPower(0);
                extendo.setExtended(false);

                if (timer.seconds() >= TIME_TRANSFER) state = STANDBY;
                
                break;
        }

        extendo.run(!deposit.requestingIntakeToMove() || state == TRANSFERRING);
    }

    public void transfer(Sample sample) {
        this.sample = sample;
        state = BUCKET_SEMI_RETRACTING;
        timer.reset();
    }

    public void ejectSample() {
        this.sample = null;
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

    public Sample getSample() {
        return sample;
    }

    boolean clearOfDeposit() {
        return extendo.getPosition() >= Extendo.LENGTH_DEPOSIT_CLEAR;
    }

    public void runRoller(double power) {
        rollerSpeed = power;
    }

    void printTelemetry() {
        mTelemetry.addData("INTAKE", state + ", " + (hasSample() ? getSample() + " sample" : "empty"));
        hsv.toTelemetry();
        divider();
        extendo.printTelemetry();
    }

}
