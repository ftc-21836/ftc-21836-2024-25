package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.SPEED_INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.ARM_ENTERING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.ARM_EXITING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.SETTLING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EJECTING_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RE_EXTENDING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.STANDBY;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@Config
public final class Intake {

    public static double

            ANGLE_BUCKET_RETRACTED = 5,
            ANGLE_BUCKET_OVER_BARRIER = 140,
            ANGLE_BUCKET_INTAKING_NEAR = 206,
            ANGLE_BUCKET_INTAKING_FAR = 203.5,

            TIME_EJECTING = 0.5,
            TIME_MAX_EXTEND_BEFORE_RE_RETRACT = 0.65,
            TIME_MAX_RETRACT_BEFORE_REATTEMPT = 0.65,
            TIME_MAX_BUCKET_RETRACT = 0.75,
            TIME_BUCKET_SETTLING = 0,

            LENGTH_RE_RETRACT_TARGET = 150,

            SPEED_EJECTING = -0.25,
            SPEED_HOLDING = 0.25,
            SPEED_PRE_TRANSFER = -0.1,
            SPEED_POST_TRANSFER = -0.2,
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake bucket sample detection
     */
    public static HSV
            minRed = new HSV(
                    0,
                    0.25,
                    0.008
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

    private final CachedMotorEx roller;
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
        BUCKET_RETRACTING,
        EXTENDO_RE_EXTENDING,
        EXTENDO_RETRACTING,
        SETTLING,
        ARM_ENTERING_BUCKET,
        TRANSFERRING,
        ARM_EXITING_BUCKET,
    }

    public void setAlliance(boolean redAlliance) {
        badSample = redAlliance ? BLUE : RED;
    }

    Intake(HardwareMap hardwareMap) {

        extendo = new Extendo(hardwareMap);

        bucketR = getAxon(hardwareMap, "bucket right").reversed();
        bucketL = getAxon(hardwareMap, "bucket left");

        roller = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        roller.setInverted(true);

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket sensor");
    }

    void run(Deposit deposit) {

        double ANGLE_BUCKET_INTAKING = lerp(ANGLE_BUCKET_INTAKING_NEAR, ANGLE_BUCKET_INTAKING_FAR, extendo.getPosition() / Extendo.LENGTH_EXTENDED);

        switch (state) {

            case EJECTING_SAMPLE:

                setBucket(ANGLE_BUCKET_INTAKING);
                roller.set(SPEED_EJECTING);

                if (timer.seconds() >= TIME_EJECTING) state = STANDBY;
                else break;

            case STANDBY:

                if (rollerSpeed != 0) { // intaking, trigger held down

                    setBucket(lerp(ANGLE_BUCKET_OVER_BARRIER, ANGLE_BUCKET_INTAKING, abs(rollerSpeed)));
                    roller.set(rollerSpeed / SPEED_INTAKING);
                    
                    colorSensor.update();
                    sample = hsvToSample(hsv = colorSensor.getHSV());

                    if (getSample() == badSample || deposit.hasSample()) ejectSample();
                    
                    break;
                    
                } else if (!hasSample()) { // retracted

                    setBucket(ANGLE_BUCKET_RETRACTED);
                    roller.set(0);

                    break;

                } else transfer(sample);

            case BUCKET_RETRACTING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(SPEED_HOLDING);

                if (bucketSensor.isPressed() || timer.seconds() >= TIME_MAX_BUCKET_RETRACT || !retractBucketBeforeExtendo) {
                    state = EXTENDO_RETRACTING;
                    timer.reset();
                } else break;

            case EXTENDO_RE_EXTENDING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(SPEED_HOLDING);

                if (extendo.atPosition(LENGTH_RE_RETRACT_TARGET) || timer.seconds() > TIME_MAX_EXTEND_BEFORE_RE_RETRACT) {
                    state = EXTENDO_RETRACTING;
                    timer.reset();
                } else break;

            case EXTENDO_RETRACTING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(SPEED_HOLDING);
                extendo.setExtended(false);

                if (extendo.isExtended()) {
                    if (timer.seconds() >= TIME_MAX_RETRACT_BEFORE_REATTEMPT) {
                        state = EXTENDO_RE_EXTENDING;
                        extendo.setTarget(LENGTH_RE_RETRACT_TARGET);
                        timer.reset();
                    }
                    break;
                } else if (bucketSensor.isPressed() && deposit.readyToTransfer()) {
                    state = SETTLING;
                    timer.reset();
                } else break;

            case SETTLING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(SPEED_HOLDING);
                extendo.setExtended(false);

                if (timer.seconds() >= TIME_BUCKET_SETTLING) {
                    state = ARM_ENTERING_BUCKET;
                    deposit.transfer();
                    timer.reset();
                } else break;

            case ARM_ENTERING_BUCKET:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(SPEED_PRE_TRANSFER);
                extendo.setExtended(false);

                if (deposit.state == Deposit.State.TRANSFERRING) {
                    state = TRANSFERRING;
                    sample = null;
                    timer.reset();
                } else break;

            case TRANSFERRING:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(0);
                extendo.setExtended(false);

                if (deposit.state == Deposit.State.EXITING_BUCKET) {
                    state = ARM_EXITING_BUCKET;
                    timer.reset();
                } else break;

            case ARM_EXITING_BUCKET:

                setBucket(ANGLE_BUCKET_RETRACTED);
                roller.set(SPEED_POST_TRANSFER);
                extendo.setExtended(false);

                if (deposit.state != Deposit.State.EXITING_BUCKET) {
                    state = STANDBY;
                    timer.reset();
                } else break;
        }

        extendo.run();
    }

    public void transfer(Sample sample) {
        this.sample = sample;
        state = BUCKET_RETRACTING;
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
