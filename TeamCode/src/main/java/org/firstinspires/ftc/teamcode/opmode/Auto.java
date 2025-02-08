package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_CYCLES;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_WAIT;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PRELOAD_AND_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_2;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_CHAMBER_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_OFFSET_PRELOAD_SCORED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_OFFSET_PRELOAD_SCORING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_SPECIMEN_PRELOAD;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;
import static java.lang.Math.ceil;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.Arrays;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class Auto extends LinearOpMode {


    enum State {
        PRELOAD_AND_1,
        SCORING_1,
        INTAKING_2,
        SCORING_2,
        INTAKING_3,
        SCORING,
        DRIVING_TO_SUB,
        INTAKING,
        PARKING
    }

    public static MultipleTelemetry mTelemetry;

    public static void divider() {
        mTelemetry.addLine();
        mTelemetry.addLine("--------------------------------------------------------------------------");
        mTelemetry.addLine();
    }

    public static double
            LENGTH_ROBOT = 17.30327,
            WIDTH_ROBOT = 16.42126,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,
            EXTEND_SAMPLE_1 = 410,
            EXTEND_SAMPLE_2 = 395,
            EXTEND_SAMPLE_3 = 350,
            EXTEND_OVER_SUB_BAR = 80,
            EXTEND_SUB_MIN = 254,
            EXTEND_SUB_MAX = 410,
            EXTEND_RETURN_OFFSET = 50,
            TIME_EXTEND_CYCLE = 1,
            SPEED_SWEEPING_SUB = 5,
            SPEED_SWEEPING_SUB_TURNING = 0.5,
            SPEED_INCHING = 5,
            SPEED_INCHING_TURNING = 0.75,
            SPEED_INTAKING = 0.85,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_POST_INTAKING_SPIKE = 0.3,
            WAIT_POST_INTAKING_SUB = 0.3,
            WAIT_SCORE_BASKET = 0.25,
            WAIT_SCORE_CHAMBER = 0.1,
            WAIT_SCORE_SPEC_PRELOAD = 0.75,
            WAIT_DROP_TO_EXTEND = 0.75,
            WAIT_INTAKE_RETRACT_POST_SUB = 0.5,
            WAIT_EXTEND_MAX_SPIKE = 0.75,
            WAIT_SWEEPER_EXTEND = 0.3,
            WAIT_SWEEPER_RETRACT = 0,
            WAIT_EXTEND_POST_SWEEPER = 0.3,
            LIFT_HEIGHT_TOLERANCE = 3.75,
            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,
            Y_INCHING_FORWARD_WHEN_INTAKING = 5,
            TIME_CYCLE = 5,
            TIME_SCORE = 2;

    /// <a href="https://www.desmos.com/calculator/sishohvpwc">Visualize spike samples</a>
    public static EditablePose
            admissibleError = new EditablePose(1, 1, toRadians(2)),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            basket = new EditablePose(-57, -56, PI / 4),
            basketFromSub = new EditablePose(-58, -58, 0.765),

            sample1 = new EditablePose(-49.24,-28.3, PI / 2),
            sample2 = new EditablePose(-57.85,-27.77, PI / 2),
            sample3 = new EditablePose(-69.2,-26, PI / 2),

            intaking1 = new EditablePose(-54.6,-50, toRadians(84.36)),
            intaking2 = new EditablePose(-56.9,-49.9, toRadians(105)),
            intaking3 = new EditablePose(-53.5,-43.5, 2 * PI / 3),

            intakingSub = new EditablePose(-22.5, -11, 0),
            sweptSub = new EditablePose(-22.5, 8, 0),
            sub2 = new EditablePose(-22.5, 0, 0),
    
            parkLeft = new EditablePose(-22.5, -11, 0),
            aroundBeamPushing = new EditablePose(35, -30, PI / 2),
    
            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, - PI / 2),
            chamberLeft = new EditablePose(-chamberRight.x, -33, PI / 2),
    
            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(63, pushing1.y, - PI / 2),
    
            pushed1 = new EditablePose(pushing1.x, -46, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, pushed1.y, - PI / 2),
    
            intakingSpec = new EditablePose(36, -60, PI / 2),
            intakingFirstSpec = new EditablePose(55, intakingSpec.y, -intakingSpec.heading);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    enum AutonConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PRELOAD,
        EDITING_WAIT,
        EDITING_CYCLES;

        public static final AutonConfig[] selections = values();

        public AutonConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(AutonConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Deposit.level1Ascent = false;

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, pose);
        robot.deposit.closeClaw();

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        AutonConfig selection = EDITING_ALLIANCE;

        boolean specimenSide = false;
        double partnerWait = 0;
        int cycles = 3;
        boolean specimenPreload = false;

        ElapsedTime timer = new ElapsedTime();

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && timer.seconds() < 5 && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();

            boolean up = gamepadEx1.wasJustPressed(DPAD_UP);
            boolean down = gamepadEx1.wasJustPressed(DPAD_DOWN);
            boolean y = gamepadEx1.wasJustPressed(Y);
            boolean x = gamepadEx1.wasJustPressed(X);
            boolean a = gamepadEx1.wasJustPressed(A);

            if (up || down || y || a || x) timer.reset();

            if (up) {
                do selection = selection.plus(-1);
                while (
                        selection == EDITING_PRELOAD && specimenSide ||
                        selection == EDITING_WAIT && !specimenSide && !specimenPreload ||
                        selection == EDITING_CYCLES && !specimenSide
                );
            } else if (down) {
                do selection = selection.plus(1);
                while (
                        selection == EDITING_PRELOAD && specimenSide ||
                        selection == EDITING_WAIT && !specimenSide && !specimenPreload ||
                        selection == EDITING_CYCLES && !specimenSide
                );
            }

            switch (selection) {
                case EDITING_ALLIANCE:
                    if (x) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    if (x) specimenSide = !specimenSide;
                    break;
                case EDITING_PRELOAD:
                    if (!specimenSide && x) specimenPreload = !specimenPreload;
                    break;
                case EDITING_WAIT:
                    if ((specimenPreload || specimenSide) && y) partnerWait++;
                    if ((specimenPreload || specimenSide) && a && partnerWait > 0) partnerWait--;
                    break;
                case EDITING_CYCLES:
                    if (specimenSide && y) cycles++;
                    if (specimenSide && a && cycles > 0) cycles--;
                    break;
            }

            gamepad1.setLedColor(
                    isRedAlliance ? 1 : 0,
                    0,
                    !isRedAlliance ? 1 : 0,
                    Gamepad.LED_DURATION_CONTINUOUS
            );

            mTelemetry.addLine("CONFIRMING IN " + (int) ceil(5 - timer.seconds()) + " SECONDS!");
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine((specimenSide ? "RIGHT (OBSERVATION-SIDE)" : "LEFT (BASKET-SIDE)") + selection.markIf(EDITING_SIDE));
            if (!specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine((specimenPreload ? "SPECIMEN" : "SAMPLE") + " preload" + selection.markIf(EDITING_PRELOAD));
            }
            if (specimenPreload || specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine("Wait " + partnerWait + " sec" + (partnerWait == 1 ? "" : "s") + " before specimen preload" + selection.markIf(EDITING_WAIT));
            }
            if (specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine(cycles + " specimen" + (cycles == 1 ? "" : "s") + " from observation zone" + selection.markIf(EDITING_CYCLES));
            }

            mTelemetry.update();
        }

        specimenPreload = specimenSide || specimenPreload;

        mTelemetry.addLine("GENERATING TRAJECTORY...");
        mTelemetry.update();

        mTelemetry.addLine("TRAJECTORY GENERATED:");
        mTelemetry.addLine();
        mTelemetry.addLine("> " + (isRedAlliance ? "Red" : "Blue") + " alliance");
        robot.intake.setAlliance(isRedAlliance);

        Action trajectory;

        if (specimenSide) {

            while (!robot.deposit.hasSpecimen()) robot.deposit.triggerClaw();

            pose = new Pose2d(chamberRight.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, - PI / 2);

            TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(pose);

            mTelemetry.addLine("> Right side (observation zone)");

            /// Score preloaded specimen
            builder = builder
                    .waitSeconds(partnerWait)
                    .strafeTo(chamberRight.toVector2d())
                    .stopAndAdd(scoreSpecimen(robot))
            ;

            mTelemetry.addLine("> Score preloaded specimen");

            if (cycles > 0) {

                /// Push samples
                builder = builder
                        .afterTime(0, robot.deposit::triggerClaw)
                        .setTangent(- PI / 2);
                
                EditablePose[] pushingPoses = {aroundBeamPushing, pushing1, pushed1, pushing2, pushed2, pushing3, pushed3, intakingFirstSpec};
                for (EditablePose pose : pushingPoses) {
                    builder = builder.splineToConstantHeading(pose.toVector2d(), pose.heading);
                }
                mTelemetry.addLine("> Push samples");

                double[] chamberXs = {
                        X_OFFSET_CHAMBER_1,
                        X_OFFSET_CHAMBER_2,
                        X_OFFSET_CHAMBER_3,
                        X_OFFSET_CHAMBER_4,
                };

                /// Cycle specimens
                for (int i = 0; i < min(chamberXs.length, cycles); i++) {
                    if (i > 0) builder = builder
                            .afterTime(0, robot.deposit::triggerClaw)
                            .setTangent(- PI / 2)
                            .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                    ;
                    builder = builder
                            .waitSeconds(WAIT_APPROACH_WALL)
                            .afterTime(0, robot.deposit::triggerClaw)
                            .stopAndAdd(telemetryPacket -> !robot.deposit.hasSpecimen())
                            .setTangent(PI / 2)
                            .splineToConstantHeading(new Vector2d(chamberRight.x + chamberXs[i] * DISTANCE_BETWEEN_SPECIMENS, chamberRight.y), chamberRight.heading)
                            .stopAndAdd(scoreSpecimen(robot))
                    ;

                    mTelemetry.addLine("> Specimen cycle " + (i + 1));
                }


            }

            /// Park in observation zone
            builder = builder.strafeTo(intakingSpec.toVector2d());

            mTelemetry.addLine("> Park in observation zone");

            trajectory = builder.build();

        } else {

            robot.intake.retractBucketBeforeExtendo = false;
            robot.deposit.liftBeforePointArm = true;

            if (specimenPreload)
                robot.deposit.preloadSpecimen();
            else
                robot.deposit.transfer(NEUTRAL);

            pose = specimenPreload ?
                    new Pose2d(chamberLeft.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, PI / 2) :
                    new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);

            mTelemetry.addLine("> Left side (near basket)");
            mTelemetry.addLine("> Score preloaded " + (specimenPreload ? "specimen" : "sample"));

            MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(SPEED_INCHING),
                    new AngularVelConstraint(SPEED_INCHING_TURNING)
            ));

            MinVelConstraint sweepConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(SPEED_SWEEPING_SUB),
                    new AngularVelConstraint(SPEED_SWEEPING_SUB_TURNING)
            ));

            intaking1.heading = atan2(sample1.y - intaking1.y, sample1.x - intaking1.x);
            intaking2.heading = atan2(sample2.y - intaking2.y, sample2.x - intaking2.x);
            intaking3.heading = atan2(sample3.y - intaking3.y, sample3.x - intaking3.x);

            Action preloadAnd1 =
                    (specimenPreload ?
                            robot.drivetrain.actionBuilder(pose)
                                    .waitSeconds(partnerWait)
                                    .strafeTo(chamberLeft.toVector2d())
                                    .waitSeconds(WAIT_APPROACH_CHAMBER)
                                    .stopAndAdd(telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SPEC_PRELOAD) && robot.deposit.lift.atPosition(HEIGHT_SPECIMEN_PRELOAD))) // wait until deposit in position
                                    .stopAndAdd(() -> robot.deposit.lift.setTarget(HEIGHT_SPECIMEN_PRELOAD + HEIGHT_OFFSET_PRELOAD_SCORING))
                                    .stopAndAdd(telemetryPacket -> robot.deposit.lift.getPosition() < HEIGHT_SPECIMEN_PRELOAD + HEIGHT_OFFSET_PRELOAD_SCORED)
                                    .stopAndAdd(robot.deposit::triggerClaw)
                                    .waitSeconds(WAIT_SCORE_SPEC_PRELOAD)
                                    .strafeToSplineHeading(intaking1.toVector2d(), intaking1.heading)
                                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                                    .waitSeconds(WAIT_DROP_TO_EXTEND) :
                            robot.drivetrain.actionBuilder(pose)
                                    .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                                    .stopAndAdd(scoreSample(robot))
                                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                                    .strafeToSplineHeading(intaking1.toVector2d(), intaking1.heading)
                    )
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
                        timer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)))
                    .setTangent(intaking2.heading)
                    .lineToY(intaking1.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action intake2 = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                    .strafeToSplineHeading(intaking2.toVector2d(), intaking2.heading)
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                        timer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                    .setTangent(intaking2.heading)
                    .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action intake3 = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                    .strafeToSplineHeading(intaking3.toVector2d(), intaking3.heading)
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                        timer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)))
                    .setTangent(PI / 2)
                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action i3ToSub = robot.drivetrain.actionBuilder(
                            new Pose2d(intaking3.x, intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, intaking3.heading)
                    )
                    .afterTime(0.5, sweep(robot))
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR);
                        robot.intake.runRoller(0);
                        robot.intake.ejectSample();
                        robot.deposit.liftBeforePointArm = false;
                    })
                    .setTangent(PI / 4)
                    .splineToSplineHeading(intakingSub.toPose2d(), intakingSub.heading)
                    .stopAndAdd(sweep(robot))
                    .stopAndAdd(() -> robot.intake.runRoller(SPEED_INTAKING))
                    .waitSeconds(WAIT_DROP_TO_EXTEND)
                    .stopAndAdd(() -> robot.intake.extendo.setExtended(true))
                    .waitSeconds(WAIT_EXTEND_POST_SWEEPER)
                    .build();

            Action toSub = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR);
                        robot.deposit.liftBeforePointArm = false;
                    })
                    .afterTime(0, sweep(robot))
                    .setTangent(basket.heading)
                    .splineTo(intakingSub.toVector2d(), intakingSub.heading)
                    .stopAndAdd(sweep(robot))
                    .stopAndAdd(() -> robot.intake.runRoller(SPEED_INTAKING))
                    .waitSeconds(WAIT_DROP_TO_EXTEND)
                    .stopAndAdd(() -> robot.intake.extendo.setExtended(true))
                    .waitSeconds(WAIT_EXTEND_POST_SWEEPER)
                    .build();

            Action park = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> {
                        Deposit.level1Ascent = true;
                        robot.deposit.lift.setTarget(0);
                    })
                    .splineTo(parkLeft.toVector2d(), parkLeft.heading)
                    .build();

            trajectory = new Action() {

                State state = PRELOAD_AND_1;

                ElapsedTime matchTimer = null;

                Action activeTraj = preloadAnd1;

                EditablePose sub = intakingSub;
                double subExtend = EXTEND_SUB_MAX;

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = 30 - matchTimer.seconds();

                    boolean trajDone = !activeTraj.run(p);

                    switch (state) {
                        case PRELOAD_AND_1:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        /// Score
                                        .afterTime(0, () -> robot.intake.runRoller(1))
                                        .waitSeconds(WAIT_POST_INTAKING_SPIKE)
                                        .afterTime(0, () -> robot.intake.runRoller(0))
                                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                                        .stopAndAdd(scoreSample(robot))
                                        .build();
                                state = SCORING_1;
                            } else if (trajDone) { // skip to 2 if didn't get 1
                                activeTraj = intake2;
                                state = INTAKING_2;
                                robot.intake.extendo.setExtended(false);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING_1:
                            if (trajDone) {
                                activeTraj = intake2;
                                state = INTAKING_2;
                            }
                            break;
                        case INTAKING_2:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .afterTime(0, () -> robot.intake.runRoller(1))
                                        .waitSeconds(WAIT_POST_INTAKING_SPIKE)
                                        .afterTime(0, () -> robot.intake.runRoller(0))
                                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                                        .stopAndAdd(scoreSample(robot))
                                        .build();
                                state = SCORING_2;
                            } else if (trajDone) { // skip to 3 if didn't get 2
                                activeTraj = intake3;
                                state = INTAKING_3;
                                robot.intake.extendo.setExtended(false);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING_2:
                            if (trajDone) {
                                activeTraj = intake3;
                                state = INTAKING_3;
                            }
                            break;
                        case INTAKING_3:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .afterTime(0, () -> robot.intake.runRoller(1))
                                        .waitSeconds(WAIT_POST_INTAKING_SPIKE)
                                        .afterTime(0, () -> robot.intake.runRoller(0))
                                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                                        .stopAndAdd(scoreSample(robot))
                                        .build();
                                state = SCORING;
                            } else if (trajDone) { // skip to sub if didn't get 3
                                activeTraj = i3ToSub;
                                state = DRIVING_TO_SUB;
                            }

                            break;

                        case SCORING:
                            if (trajDone) {
                                if (remaining < TIME_CYCLE) {
                                    activeTraj = park;
                                    state = PARKING;
                                } else {
                                    activeTraj = sub == intakingSub ? toSub :
                                            robot.drivetrain.actionBuilder(basket.toPose2d())
                                                    .afterTime(0, () -> {
                                                        robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR);
                                                        robot.deposit.liftBeforePointArm = false;
                                                    })
                                                    .afterTime(0, sweep(robot))
                                                    .setTangent(basket.heading)
                                                    .splineTo(sub2.toVector2d(), sub2.heading)
                                                    .stopAndAdd(sweep(robot))
                                                    .stopAndAdd(() -> robot.intake.runRoller(SPEED_INTAKING))
                                                    .waitSeconds(WAIT_DROP_TO_EXTEND)
                                                    .stopAndAdd(() -> robot.intake.extendo.setExtended(true))
                                                    .waitSeconds(WAIT_EXTEND_POST_SWEEPER)
                                                    .build();
                                    state = DRIVING_TO_SUB;
                                }
                            } else if (remaining < WAIT_SCORE_BASKET && robot.deposit.hasSample()) {
                                robot.deposit.triggerClaw();
                                return false;
                            }
                            break;

                        case DRIVING_TO_SUB:
                            if (trajDone) {
                                activeTraj = robot.drivetrain.actionBuilder(sub.toPose2d())
                                        .setTangent(sub.y > 0 ? - PI / 2 : PI / 2)
                                        .lineToY(sub.y > 0 ? intakingSub.y : sweptSub.y, sweepConstraint)
                                        .build();
                                state = INTAKING;
                                timer.reset();
                            }
                            break;
                        case INTAKING:

                            if (remaining < TIME_SCORE) {
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .afterTime(0, () -> {
                                            robot.intake.ejectSample();
                                            robot.intake.runRoller(0);
                                            Deposit.level1Ascent = true;
                                            robot.deposit.lift.setTarget(0);
                                        })
                                        .afterTime(1, () -> robot.intake.extendo.setExtended(false))
                                        .setTangent(PI / 2)
                                        .lineToY(robot.drivetrain.pose.position.y + 0.25)
                                        .build();
                                state = PARKING;
                                break;
                            }

                            // Sample intaked
                            if (robot.intake.hasSample()) {

                                sub = new EditablePose(robot.drivetrain.pose);
                                subExtend = robot.intake.extendo.getPosition() - EXTEND_RETURN_OFFSET;

                                activeTraj = robot.drivetrain.actionBuilder(sub.toPose2d())
                                        .afterTime(0, () -> robot.intake.runRoller(1))
                                        .waitSeconds(WAIT_POST_INTAKING_SUB)
                                        .afterTime(0, () -> robot.intake.runRoller(0))
                                        .setTangent(PI + sub.heading)
                                        .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                                        .afterDisp(12, () -> robot.sweeper.setActivated(true))
                                        .splineTo(basketFromSub.toVector2d(), PI + basketFromSub.heading)
                                        .afterTime(0, () -> robot.sweeper.setActivated(false))
                                        .stopAndAdd(scoreSample(robot))
                                        .build();

                                state = SCORING;
                            } else {

                                /// <a href="https://www.desmos.com/calculator/2jddu08h7f">Graph</a>
                                robot.intake.extendo.setTarget(
                                        EXTEND_SUB_MIN + (EXTEND_SUB_MAX - EXTEND_SUB_MIN) * (1 + cos(2 * PI * timer.seconds() / TIME_EXTEND_CYCLE)) / 2
                                );

                                // sweep the other way
                                if (trajDone) activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .setTangent(robot.drivetrain.pose.position.y > 0 ? - PI / 2 : PI / 2)
                                        .lineToY(robot.drivetrain.pose.position.y > 0 ? intakingSub.y : sweptSub.y, sweepConstraint)
                                        .build();

                            }

                            break;

                        case PARKING:
                            return !trajDone;
                    }

                    return true;
                }
            };
        }

        // Parallel action to bulk read, update trajectory, and update robot (robot.run())
        ParallelAction auton = new ParallelAction(
                telemetryPacket -> {
                    robot.bulkReader.bulkRead();
                    return opModeIsActive();
                },
                trajectory,
                telemetryPacket -> {
                    pose = robot.drivetrain.pose;
                    robot.run();
                    return opModeIsActive();
                }
        );

        mTelemetry.update();

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.pinpoint.setPositionRR(pose);

        Actions.runBlocking(auton);
    }

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> {
                    if (!robot.hasSample()) robot.deposit.transfer(NEUTRAL);
                }),
                new SleepAction(WAIT_APPROACH_BASKET),
                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SCORING_SAMPLE) && abs(robot.deposit.lift.getPosition() - HEIGHT_BASKET_HIGH) <= LIFT_HEIGHT_TOLERANCE),
                new InstantAction(robot.deposit::triggerClaw),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action scoreSpecimen(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> {
                    if (!robot.hasSample()) while (!robot.deposit.hasSpecimen()) robot.deposit.triggerClaw();
                }),
                new SleepAction(WAIT_APPROACH_CHAMBER),
                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SPECIMEN) && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
                new InstantAction(robot.deposit::triggerClaw),
                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

    private static Action sweep(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> robot.sweeper.setActivated(true)),
                new SleepAction(WAIT_SWEEPER_EXTEND),
                new InstantAction(() -> robot.sweeper.setActivated(false)),
                new SleepAction(WAIT_SWEEPER_RETRACT)
        );
    }

}
