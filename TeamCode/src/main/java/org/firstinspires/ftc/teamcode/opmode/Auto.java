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
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_CHAMBER_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_OFFSET_PRELOAD_SCORED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_OFFSET_PRELOAD_SCORING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_SPECIMEN_PRELOAD;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class Auto extends LinearOpMode {

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
            EXTEND_SAMPLE_1 = 300,
            EXTEND_SAMPLE_2 = 300,
            EXTEND_SAMPLE_3 = 410,
            EXTEND_SUB_MIN = 60,
            EXTEND_SUB_MAX = 410,
            TIME_EXTEND_CYCLE = 2,
            SPEED_SWEEPING_SUB = 2.5,
            SPEED_SWEEPING_SUB_TURNING = 0.5,
            SPEED_INCHING = 5,
            SPEED_INCHING_TURNING = 0.75,
            SPEED_INTAKING = 0.85,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_POST_INTAKING_SPIKE = 0.2,
            WAIT_POST_INTAKING_SUB = 0.5,
            WAIT_SCORE_BASKET = 0.1,
            WAIT_SCORE_CHAMBER = 0.75,
            WAIT_DROP_TO_EXTEND = 0.75,
            WAIT_INTAKE_RETRACT = 0.75,
            WAIT_EXTEND = 0.75,
            WAIT_SWEEPER_EXTEND = 0.2,
            WAIT_SWEEPER_RETRACT = 0.1,
            WAIT_EXTEND_POST_SWEEP = 0.75,
            LIFT_HEIGHT_TOLERANCE = 3.75,
            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,
            Y_INCHING_FORWARD_WHEN_INTAKING = 5,
            TIME_CYCLE = 5,
            TIME_SCORE = 2;

    public static EditablePose
            sample1 = new EditablePose(-48, -27.75, PI / 2),
            sample1SpecPreload = new EditablePose(-50, -27.75, sample1.heading),
            sample2 = new EditablePose(-58, -27.75, sample1.heading),
            sample3 = new EditablePose(-68.75, -26.5, sample1.heading),
            basket = new EditablePose(-56.25, -56.25, PI / 4),
            intaking1 = new EditablePose(-50, -46, toRadians(84.36)),
            intaking1SpecPreload = new EditablePose(-51, -46, toRadians(84.36)),
            intaking2 = new EditablePose(-54, -45, toRadians(105)),
            intaking3 = new EditablePose(-54, -43, 2 * PI / 3),
            intakingSub = new EditablePose(-22, -11, 0),
            sweptSub = new EditablePose(-22, 0, 0),
            parkLeft = new EditablePose(-22, -11, 0),
            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33,  - PI / 2),
            chamberLeft = new EditablePose(-chamberRight.x, -33, PI / 2),
            aroundBeamPushing = new EditablePose(35, -30, PI / 2),
            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(63, pushing1.y, - PI / 2),
            pushed1 = new EditablePose(pushing1.x, -46, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, pushed1.y, - PI / 2),
            intakingSpec = new EditablePose(36, -60, PI / 2),
            intakingFirstSpec = new EditablePose(55, intakingSpec.y, -intakingSpec.heading),
            fromSub = new EditablePose(-36, -12, atan2(-12 + 48, -36 + 48));

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

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
                do selection = selection.plus(-1);
                while (
                        selection == EDITING_PRELOAD && specimenSide ||
                        selection == EDITING_WAIT && !specimenSide && !specimenPreload ||
                        selection == EDITING_CYCLES && !specimenSide
                );
            } else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                do selection = selection.plus(1);
                while (
                        selection == EDITING_PRELOAD && specimenSide ||
                        selection == EDITING_WAIT && !specimenSide && !specimenPreload ||
                        selection == EDITING_CYCLES && !specimenSide
                );
            }

            switch (selection) {
                case EDITING_ALLIANCE:
                    if (gamepadEx1.wasJustPressed(X)) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    if (gamepadEx1.wasJustPressed(X)) specimenSide = !specimenSide;
                    break;
                case EDITING_PRELOAD:
                    if (!specimenSide && gamepadEx1.wasJustPressed(X)) specimenPreload = !specimenPreload;
                    break;
                case EDITING_WAIT:
                    if ((specimenPreload || specimenSide) && gamepadEx1.wasJustPressed(Y)) partnerWait++;
                    if ((specimenPreload || specimenSide) && gamepadEx1.wasJustPressed(A) && partnerWait > 0) partnerWait--;
                    break;
                case EDITING_CYCLES:
                    if (specimenSide && gamepadEx1.wasJustPressed(Y)) cycles++;
                    if (specimenSide && gamepadEx1.wasJustPressed(A) && cycles > 0) cycles--;
                    break;
            }

            gamepad1.setLedColor(
                    isRedAlliance ? 1 : 0,
                    0,
                    !isRedAlliance ? 1 : 0,
                    Gamepad.LED_DURATION_CONTINUOUS
            );

            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");
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

            intaking1SpecPreload.heading = atan2(sample1SpecPreload.y - intaking1SpecPreload.y, sample1SpecPreload.x - intaking1SpecPreload.x);
            intaking1.heading = atan2(sample1.y - intaking1.y, sample1.x - intaking1.x);
            intaking2.heading = atan2(sample2.y - intaking2.y, sample2.x - intaking2.x);
            intaking3.heading = atan2(sample3.y - intaking3.y, sample3.x - intaking3.x);

            EditablePose i1 = specimenPreload ? intaking1SpecPreload : intaking1;

            ElapsedTime extendoTimer = new ElapsedTime();

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
                                    .waitSeconds(WAIT_SCORE_CHAMBER)
                                    .strafeToSplineHeading(intaking1SpecPreload.toVector2d(), intaking1SpecPreload.heading)
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
                        extendoTimer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(extendoTimer.seconds() >= WAIT_EXTEND || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)))
                    .lineToY(i1.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score1 = robot.drivetrain.actionBuilder(i1.toPose2d())
                    /// Score
                    .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action i1To2 = robot.drivetrain.actionBuilder(
                            new Pose2d(i1.x, i1.y + Y_INCHING_FORWARD_WHEN_INTAKING, i1.heading)
                    )
                    .afterTime(0, () -> {
                        robot.intake.extendo.setExtended(false);
                        robot.intake.ejectSample();
                    })
                    .setTangent(i1.heading + PI)
                    .splineToLinearHeading(intaking2.toPose2d(), intaking2.heading)
                    .afterTime(0, () -> {
                        robot.intake.runRoller(SPEED_INTAKING);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                        extendoTimer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(extendoTimer.seconds() >= WAIT_EXTEND || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                    .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action intake2 = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                    .strafeToSplineHeading(intaking2.toVector2d(), intaking2.heading)
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                        extendoTimer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(extendoTimer.seconds() >= WAIT_EXTEND || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                    .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score2 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                    .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action i2To3 = robot.drivetrain.actionBuilder(
                            new Pose2d(intaking2.x, intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, intaking2.heading)
                    )
                    .afterTime(0, () -> {
                        robot.intake.extendo.setExtended(false);
                        robot.intake.ejectSample();
                    })
                    .setTangent(intaking2.heading + PI)
                    .splineToLinearHeading(intaking3.toPose2d(), intaking3.heading)
                    .afterTime(0, () -> {
                        robot.intake.runRoller(SPEED_INTAKING);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                        extendoTimer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(extendoTimer.seconds() >= WAIT_EXTEND || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)))
                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action intake3 = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                    .strafeToSplineHeading(intaking3.toVector2d(), intaking3.heading)
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                        extendoTimer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(extendoTimer.seconds() >= WAIT_EXTEND || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)))
                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score3 = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                    .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action i3ToSub = robot.drivetrain.actionBuilder(
                            new Pose2d(intaking3.x, intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, intaking3.heading)
                    )
                    .afterTime(0, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SUB_MIN);
                        robot.intake.runRoller(0);
                        robot.intake.ejectSample();
                    })
                    .setTangent(PI / 4)
                    .splineToSplineHeading(intakingSub.toPose2d(), intakingSub.heading)
                    .stopAndAdd(() -> robot.sweeper.setActivated(true))
                    .waitSeconds(WAIT_SWEEPER_EXTEND)
                    .stopAndAdd(() -> robot.sweeper.setActivated(false))
                    .waitSeconds(WAIT_SWEEPER_RETRACT)
                    .stopAndAdd(() -> robot.intake.runRoller(SPEED_INTAKING))
                    .waitSeconds(WAIT_DROP_TO_EXTEND)
                    .stopAndAdd(() -> robot.intake.extendo.setExtended(true))
                    .waitSeconds(WAIT_EXTEND_POST_SWEEP)
                    .build();

            Action subPark = robot.drivetrain.actionBuilder(sweptSub.toPose2d())
                    .afterTime(0, () -> {
                        robot.intake.runRoller(0);
                        Deposit.level1Ascent = true;
                        robot.deposit.lift.setTarget(0);
                    })
                    .afterTime(1, () -> robot.intake.extendo.setExtended(false))
                    .strafeToSplineHeading(parkLeft.toVector2d(), parkLeft.heading)
                    .build();

            Action park = robot.drivetrain.actionBuilder(basket.toPose2d())
                    .afterTime(0, () -> {
                        Deposit.level1Ascent = true;
                        robot.deposit.lift.setTarget(0);
                    })
                    .splineTo(parkLeft.toVector2d(), parkLeft.heading)
                    .build();

            ArrayList<Action>
                    toSubs = new ArrayList<>(),
                    sweepLefts = new ArrayList<>(),
                    sweepRights = new ArrayList<>(),
                    scores = new ArrayList<>();

            for (int i = 0; i < 6; i++) {
                toSubs.add(robot.drivetrain.actionBuilder(basket.toPose2d())
                        .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_SUB_MIN))
                        .setTangent(basket.heading)
                        .splineTo(intakingSub.toVector2d(), intakingSub.heading)
                        .stopAndAdd(i > 0 ? new InstantAction(() -> {}) : new SequentialAction(
                                new InstantAction(() -> robot.sweeper.setActivated(true)),
                                new SleepAction(WAIT_SWEEPER_EXTEND),
                                new InstantAction(() -> robot.sweeper.setActivated(false)),
                                new SleepAction(WAIT_SWEEPER_RETRACT)
                        ))
                        .stopAndAdd(() -> robot.intake.runRoller(SPEED_INTAKING))
                        .waitSeconds(WAIT_DROP_TO_EXTEND)
                        .stopAndAdd(() -> robot.intake.extendo.setExtended(true))
                        .waitSeconds(WAIT_EXTEND_POST_SWEEP)
                        .build()
                );
                sweepLefts.add(robot.drivetrain.actionBuilder(intakingSub.toPose2d())
                        .strafeToSplineHeading(sweptSub.toVector2d(), sweptSub.heading, sweepConstraint)
                        .build()
                );
                sweepRights.add(robot.drivetrain.actionBuilder(sweptSub.toPose2d())
                        .strafeToSplineHeading(intakingSub.toVector2d(), intakingSub.heading, sweepConstraint)
                        .build()
                );
                scores.add(robot.drivetrain.actionBuilder(fromSub.toPose2d())
                        .setTangent(PI + fromSub.heading)
                        .waitSeconds(WAIT_INTAKE_RETRACT)
                        .splineTo(basket.toVector2d(), PI + basket.heading)
                        .stopAndAdd(scoreSample(robot))
                        .build()
                );
            }

            trajectory = new BasketAuto(
                    robot,
                    preloadAnd1,
                    score1,
                    intake2,
                    score2,
                    intake3,
                    score3,
                    park,
                    i1To2,
                    i2To3,
                    i3ToSub,
                    subPark,
                    toSubs,
                    sweepLefts,
                    sweepRights,
                    scores
            );
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

}
