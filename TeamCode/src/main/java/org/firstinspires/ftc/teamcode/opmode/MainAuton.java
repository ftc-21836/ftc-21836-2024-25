package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.DISTANCE_BETWEEN_SPECIMENS;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.EXTEND_SAMPLE_1;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.EXTEND_SAMPLE_2;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.EXTEND_SAMPLE_3;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.LENGTH_ROBOT;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.SIZE_HALF_FIELD;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.SIZE_TILE;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.SPEED_INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_APPROACH_BASKET;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_APPROACH_CHAMBER;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_DROP_TO_EXTEND;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_POST_INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_SCORE_BASKET;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_SCORE_CHAMBER;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WIDTH_ROBOT;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.aroundBeamParkLeft;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.aroundBeamPushing;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.basket;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.chamber0;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.chamberLeft;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.intaking1;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.intaking2;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.intaking3;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.intakingFirstSpec;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.intakingSpec;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.parkLeft;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.parkRight;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.pushed1;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.pushed2;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.pushed3;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.pushing1;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.pushing2;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.pushing3;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.sample1;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.sample2;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.sample3;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_CYCLES;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_WAIT;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.PRELOAD_SAMPLE;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.PRELOAD_SPECIMEN;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.loopMod;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.pose;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_CHAMBER_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    public static boolean TELEMETRY = false;

    enum AutonConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        PRELOAD_SAMPLE,
        PRELOAD_SPECIMEN,
        EDITING_CYCLES,
        EDITING_WAIT;

        public static final AutonConfig[] selections = values();

        public AutonConfig plus(int i) {
            return selections[(int) loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(AutonConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, pose);

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        AutonConfig selection = EDITING_ALLIANCE;

        boolean specimenSide = false;
        double partnerWait = 0;
        int cycles = 3;

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP))   selection = selection.plus(-1);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) selection = selection.plus(1);

            if (gamepadEx1.wasJustPressed(X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    specimenSide = !specimenSide;
                    break;
                case PRELOAD_SAMPLE:
                    robot.deposit.transfer(NEUTRAL);
                    break;
                case PRELOAD_SPECIMEN:
                    robot.deposit.preloadSpecimen();
                    break;
            }

            if (selection == EDITING_CYCLES) {
                if (gamepadEx1.wasJustPressed(Y)) cycles++;
                if (gamepadEx1.wasJustPressed(A) && cycles > 0) cycles--;
            } else if (selection == EDITING_WAIT) {
                if (gamepadEx1.wasJustPressed(Y)) partnerWait++;
                if (gamepadEx1.wasJustPressed(A) && partnerWait > 0) partnerWait--;
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
            mTelemetry.addLine((specimenSide ? "RIGHT (SPECIMEN-SIDE)" : "LEFT (SAMPLE-SIDE)") + selection.markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine("Preload sample" + selection.markIf(PRELOAD_SAMPLE));
            mTelemetry.addLine();
            mTelemetry.addLine("Preload specimen" + selection.markIf(PRELOAD_SPECIMEN));
            mTelemetry.addLine();
            mTelemetry.addLine(cycles + " cycles" + selection.markIf(EDITING_CYCLES));
            mTelemetry.addLine();
            mTelemetry.addData("Wait before scoring specimen preload (sec)", partnerWait + selection.markIf(EDITING_WAIT));

            mTelemetry.update();
        }

        boolean specimenPreload = !specimenSide && robot.deposit.specimenIntaked();

        mTelemetry.addLine("GENERATING TRAJECTORY...");
        mTelemetry.addLine();
        mTelemetry.addLine("> " + (isRedAlliance ? "Red" : "Blue") + " alliance");

        Pose2d startPose = new Pose2d(
                specimenSide ? chamber0.x : specimenPreload ? chamberLeft.x : 0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE,
                0.5 * (specimenSide || specimenPreload ? LENGTH_ROBOT : WIDTH_ROBOT) - SIZE_HALF_FIELD,
                specimenSide || specimenPreload ? PI / 2 : 0
        );

        robot.drivetrain.pinpoint.setPositionRR(startPose);

        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(startPose);

        if (specimenSide) {

            mTelemetry.addLine("> Right side");

            /// Score preloaded specimen
            builder = builder
                    .waitSeconds(partnerWait)
                    .strafeTo(chamber0.toVector2d())
                    .stopAndAdd(scoreSpecimen(robot))
            ;

            mTelemetry.addLine("> Score preloaded specimen");

            if (cycles > 0) {

                /// Push samples
                builder = builder
                        .setTangent(-PI / 2)
                        .splineToConstantHeading(aroundBeamPushing.toVector2d(), aroundBeamPushing.heading)
                        .splineToConstantHeading(pushing1.toVector2d(), pushing1.heading)
                        .splineToConstantHeading(pushed1.toVector2d(), pushed1.heading)
                        .splineToConstantHeading(pushing2.toVector2d(), pushing2.heading)
                        .splineToConstantHeading(pushed2.toVector2d(), pushed2.heading)
                        .splineToConstantHeading(pushing3.toVector2d(), pushing3.heading)
                        .splineToConstantHeading(pushed3.toVector2d(), pushed3.heading)
                        .splineToConstantHeading(intakingFirstSpec.toVector2d(), intakingFirstSpec.heading)
                ;

                mTelemetry.addLine("> Push samples");

                /// Cycle specimens
                for (int i = 0; i < cycles; i++) {
                    builder = builder
                            .stopAndAdd(new SequentialAction(
                                    new InstantAction(robot.deposit::triggerClaw),
                                    telemetryPacket -> !robot.deposit.specimenIntaked(),
                                    new InstantAction(() -> robot.deposit.setPosition(HIGH))
                            ))
                            .setTangent(PI / 2)
                            .splineToConstantHeading(new Vector2d(chamber0.x - (i + 1) * DISTANCE_BETWEEN_SPECIMENS, chamber0.y), chamber0.heading)
                            .stopAndAdd(scoreSpecimen(robot))
                            .afterTime(0, robot.deposit::triggerClaw)
                            .setTangent(- PI / 2)
                    ;
                    if (i < cycles - 1) builder = builder
                            .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                    ;

                    mTelemetry.addLine("> Specimen cycle " + (i + 1));
                }


            }

            /// Park in observation zone
            builder = builder.strafeTo(parkRight.toVector2d());

            mTelemetry.addLine("> Park in observation zone");

        } else {

            mTelemetry.addLine("> Left side");

            if (specimenPreload) {
                /// Score preloaded specimen
                builder = builder
                        .waitSeconds(partnerWait)
                        .strafeTo(chamberLeft.toVector2d())
                        .stopAndAdd(scoreSpecimen(robot))
                ;

                mTelemetry.addLine("> Score preloaded specimen");
            } else {
                /// Score preloaded sample
                builder = builder
                        .afterTime(0, raiseLift(robot))
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample(robot))
                ;

                mTelemetry.addLine("> Score preloaded sample");
            }

            double[] extendoMMs = {EXTEND_SAMPLE_1, EXTEND_SAMPLE_2, EXTEND_SAMPLE_3};
            EditablePose[] intakingPositions = {intaking1, intaking2, intaking3};
            EditablePose[] samplePositions = {sample1, sample2, sample3};

            /// Cycle samples off the floor
            for (int i = 0; i < min(3, cycles); i++) {

                double millimeters = extendoMMs[i];
                EditablePose intakingPos = intakingPositions[i];
                EditablePose samplePos = samplePositions[i];

                // Calculate angle to point intake at floor sample
                intakingPos.heading = atan2(samplePos.y - intakingPos.y, samplePos.x - intakingPos.x);

                builder = builder

                        /// Intake
                        .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                        .afterTime(0, new SequentialAction(
                                telemetryPacket -> !robot.intake.hasSample() && robot.intake.extendo.getPosition() < millimeters - 5,
                                new InstantAction(() -> {
                                    if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                                })
                        ))
                        .stopAndAdd(new SequentialAction(
                                new InstantAction(() -> robot.intake.runRoller(SPEED_INTAKING)),
                                new SleepAction(WAIT_DROP_TO_EXTEND),
                                new InstantAction(() -> robot.intake.extendo.setTarget(millimeters)),
                                telemetryPacket -> !robot.intake.hasSample(), // wait until intake gets a sample
                                new SleepAction(WAIT_POST_INTAKING),
                                new InstantAction(() -> robot.intake.runRoller(0))
                        ))

                        /// Score
                        .afterTime(0, raiseLift(robot))
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample(robot))
                ;

                mTelemetry.addLine("> Sample cycle " + (i + 1));
            }

            /// Raise arm for level 1 ascent
            builder = builder.afterTime(0, robot.deposit::level1Ascent);

            mTelemetry.addLine("> Raise lift for level 1 ascent");

            /// Drive to level 1 ascent location
            if (specimenPreload && cycles == 0)
                builder = builder
                        .setTangent(- 5 * PI / 6)
                        .splineToSplineHeading(aroundBeamParkLeft.toPose2d(), PI / 2)
                        .splineToConstantHeading(parkLeft.toVector2d(), parkLeft.heading)
                ;
            else builder = builder.splineTo(parkLeft.toVector2d(), parkLeft.heading);

            mTelemetry.addLine("> Drive to level 1 ascent location");
        }

        // Parallel action to bulk read, update trajectory, and update robot (robot.run())
        ParallelAction auton = new ParallelAction(
                telemetryPacket -> {
                    robot.bulkReader.bulkRead();
                    return opModeIsActive();
                },
                builder.build(),
                // only print telemetry if enabled from dashboard
                TELEMETRY ?
                        telemetryPacket -> {
                            pose = robot.drivetrain.pose;
                            robot.run();
                            robot.printTelemetry(); // telemetry
                            mTelemetry.update();    // telemetry
                            return opModeIsActive();
                        } :
                        telemetryPacket -> {
                            pose = robot.drivetrain.pose;
                            robot.run();
                            return opModeIsActive();
                        }
        );

        mTelemetry.addLine();
        mTelemetry.addLine("TRAJECTORY GENERATED");
        mTelemetry.addLine("(Press play to run autonomous)");
        mTelemetry.update();

        waitForStart();

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        Actions.runBlocking(auton);
    }

    private static Action scoreSpecimen(Robot robot) {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
                telemetryPacket -> !robot.deposit.reachedTarget(Arm.SPECIMEN, HEIGHT_CHAMBER_HIGH), // wait until deposit in position
                new InstantAction(robot.deposit::triggerClaw),
                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

    private static Action raiseLift(Robot robot) {
        return new SequentialAction(
                telemetryPacket -> !robot.deposit.hasSample(),
                new SleepAction(Intake.TIME_TRANSFER),
                new InstantAction(() -> robot.deposit.setPosition(HIGH))
        );
    }

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_BASKET),
                telemetryPacket -> !robot.deposit.reachedTarget(Arm.SAMPLE, HEIGHT_BASKET_HIGH),
                new InstantAction(robot.deposit::triggerClaw),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

}
