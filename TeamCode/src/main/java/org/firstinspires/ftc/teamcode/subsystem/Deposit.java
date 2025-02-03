package org.firstinspires.ftc.teamcode.subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.GRABBING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.HAS_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.HAS_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASING_SPEC_PRELOAD;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SAMPLE_FALLING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SCORING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.SPEC_PRELOAD;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Deposit {

    public static double
            ANGLE_CLAW_OPEN = 70,
            ANGLE_CLAW_TRANSFER = 60,
            ANGLE_CLAW_SLIDING = 40,
            ANGLE_CLAW_CLOSED = 25,

            TIME_SAMPLE_RELEASE = 0.5,
            TIME_SPEC_GRAB = 0.25,
            TIME_SPEC_RELEASE = 0.5,

            HEIGHT_ABOVE_INTAKE = 10,
            HEIGHT_OBSERVATION_ZONE = 0.01,
            HEIGHT_BASKET_LOW = 0,
            HEIGHT_BASKET_HIGH = 18,
            HEIGHT_INTAKING_SPECIMEN = 0,
            HEIGHT_CHAMBER_HIGH = 0,
            HEIGHT_CHAMBER_LOW = HEIGHT_CHAMBER_HIGH,
            HEIGHT_SPECIMEN_PRELOAD = 9.5,
            HEIGHT_OFFSET_SPECIMEN_SCORED = 10,
            HEIGHT_OFFSET_SPECIMEN_SCORING = 11;

    enum State {
        RETRACTED           (Arm.TRANSFER),
        HAS_SAMPLE          (Arm.SAMPLE),
        SAMPLE_FALLING      (Arm.SAMPLE),
        INTAKING_SPECIMEN   (Arm.INTAKING),
        GRABBING_SPECIMEN   (Arm.INTAKING),
        HAS_SPECIMEN        (Arm.SPECIMEN),
        SCORING_SPECIMEN    (Arm.SCORING_SPEC),
        RELEASING_SPECIMEN  (Arm.SCORING_SPEC),
        SPEC_PRELOAD        (Arm.SPEC_PRELOAD),
        RELEASING_SPEC_PRELOAD (Arm.SPEC_PRELOAD);

        private final Arm.Position armPosition;

        State(Arm.Position armPosition) {
            this.armPosition = armPosition;
        }
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
    public final Arm arm;
    private final CachedSimpleServo claw;

    private final ElapsedTime timer = new ElapsedTime();

    private Sample sample, specimenColor = NEUTRAL;

    private Deposit.State state = RETRACTED;

    public void setAlliance(boolean redAlliance) {
        specimenColor = redAlliance ? RED : BLUE;
    }

    public static boolean level1Ascent = false;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = getGBServo(hardwareMap, "claw").reversed();
    }

    void run(Intake intake, boolean climbing) {

        // home arm when climbing begins
        if (climbing) {
            state = RETRACTED;
            sample = null;
        } else switch (state) {

            case SAMPLE_FALLING:

                if (timer.seconds() >= TIME_SAMPLE_RELEASE) {
                    state = RETRACTED;
                    setPosition(FLOOR);
                }

                break;

            case INTAKING_SPECIMEN:

                if (intake.hasSample()) setPosition(FLOOR);
                break;

            case GRABBING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_GRAB) triggerClaw();
                else break;

            case SCORING_SPECIMEN:

                if (arm.atPosition(Arm.SCORING_SPEC)) triggerClaw();
                else break;

            case RELEASING_SPEC_PRELOAD:
            case RELEASING_SPECIMEN:

                if (timer.seconds() >= TIME_SPEC_RELEASE) triggerClaw();

                break;

        }

        boolean aboveIntake = lift.getPosition() >= HEIGHT_ABOVE_INTAKE;
        boolean intakeClear = intake.clearOfDeposit();

        boolean obsZone = state.armPosition == Arm.SAMPLE && lift.getTarget() == HEIGHT_OBSERVATION_ZONE;
        boolean atBasket = state.armPosition == Arm.SAMPLE && (lift.atPosition(HEIGHT_BASKET_HIGH) || lift.atPosition(HEIGHT_BASKET_LOW));

        Arm.Position armPosition =
                level1Ascent ? Arm.ASCENT :
                obsZone ? Arm.INTAKING :
                atBasket ? Arm.SCORING_SAMPLE :
                state.armPosition;

        arm.setTarget(armPosition);

        boolean armCanMove = aboveIntake || intakeClear || !arm.movingNearIntake();

        arm.run(armCanMove);

        lift.run(armCanMove || arm.reachedTarget(), climbing);

        claw.turnToAngle(
                hasSample() ?
                        state == GRABBING_SPECIMEN || (state == HAS_SPECIMEN && !arm.atPosition(Arm.SPECIMEN)) ?
                                ANGLE_CLAW_SLIDING :
                                ANGLE_CLAW_CLOSED :
                state == RETRACTED ?    ANGLE_CLAW_TRANSFER :
                                        ANGLE_CLAW_OPEN
        );
    }

    public void preloadSpecimen() {
        sample = specimenColor;
        state = SPEC_PRELOAD;
        closeClaw();
        lift.setTarget(HEIGHT_SPECIMEN_PRELOAD);
    }

    public void closeClaw() {
        claw.turnToAngle(ANGLE_CLAW_CLOSED);
    }

    // when does the intake need to move out of the way
    boolean requestingIntakeToMove() {
        return lift.getPosition() < HEIGHT_ABOVE_INTAKE && !arm.reachedTarget() && arm.movingNearIntake();
    }

    boolean readyToTransfer() {
        return state == RETRACTED && arm.atPosition(Arm.TRANSFER) && !lift.isExtended();
    }

    public void setPosition(Position position) {

        switch (state) {

            case INTAKING_SPECIMEN:
                if (position != FLOOR) break;
                state = RETRACTED;
            case RETRACTED:

                if (position == FLOOR) {
                    lift.setTarget(0);
                    break;
                }

            case HAS_SAMPLE:
            case SAMPLE_FALLING:

                lift.setTarget(
                        position == HIGH ?  HEIGHT_BASKET_HIGH :
                        position == LOW ?   HEIGHT_BASKET_LOW :
                                            HEIGHT_OBSERVATION_ZONE
                );

                break;

            case GRABBING_SPECIMEN:
                if (position != FLOOR) break;
            case HAS_SPECIMEN:

                if (position == FLOOR) {

                    while (state != INTAKING_SPECIMEN) {
                        triggerClaw();
                    }
                    break;
                }

            case SCORING_SPECIMEN:
            case RELEASING_SPECIMEN:

                if (position == FLOOR) {

                    while (state != HAS_SPECIMEN) {
                        triggerClaw();
                    }
                    break;
                }

                lift.setTarget(position == HIGH ? HEIGHT_CHAMBER_HIGH : HEIGHT_CHAMBER_LOW);

                break;
        }

    }

    public void triggerClaw() {
        switch (state) {

            case HAS_SAMPLE:

                sample = null;
                state = SAMPLE_FALLING;
                timer.reset();

                break;

            case SAMPLE_FALLING:
            case RETRACTED:

                state = INTAKING_SPECIMEN;
                lift.setTarget(HEIGHT_INTAKING_SPECIMEN);

                break;

            case INTAKING_SPECIMEN:

                sample = specimenColor;
                state = GRABBING_SPECIMEN;
                timer.reset();

                break;

            case GRABBING_SPECIMEN:

                state = HAS_SPECIMEN;
                setPosition(HIGH);
                timer.reset();

                break;

            case HAS_SPECIMEN:

                state = SCORING_SPECIMEN;

                break;

            case SCORING_SPECIMEN:

                sample = null;
                state = RELEASING_SPECIMEN;
                timer.reset();

                break;

            case SPEC_PRELOAD:

                sample = null;
                state = RELEASING_SPEC_PRELOAD;
                timer.reset();

                break;

            case RELEASING_SPEC_PRELOAD:
            case RELEASING_SPECIMEN:

                state = RETRACTED;
                setPosition(FLOOR);

                break;
        }
    }

    public boolean hasSample() {
        return sample != null;
    }

    public boolean basketReady() {
        return state == HAS_SAMPLE;
    }

    public boolean hasSpecimen() {
        return state == HAS_SPECIMEN;
    }

    public boolean intaking() {
        return state == INTAKING_SPECIMEN;
    }

    Sample getSample() {
        return sample;
    }

    public void transfer(Sample sample) {
        if (sample == null || hasSample() || state != RETRACTED) return;
        this.sample = sample;
        state = HAS_SAMPLE;
        setPosition(HIGH);
        closeClaw();
    }

    void printTelemetry() {
        String gameElement = sample + (state.ordinal() >= INTAKING_SPECIMEN.ordinal() ? " specimen" : " sample");
        mTelemetry.addData("DEPOSIT", state + ", " + (hasSample() ? gameElement : "empty"));
        divider();
        arm.printTelemetry();
        divider();
        lift.printTelemetry();
    }

}
