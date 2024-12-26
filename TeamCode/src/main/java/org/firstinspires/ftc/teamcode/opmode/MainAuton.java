package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.DISTANCE_BETWEEN_SPECIMENS;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.EXTEND_SAMPLE_1;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.EXTEND_SAMPLE_2;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.EXTEND_SAMPLE_3;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.LENGTH_ROBOT;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.LIFT_PARK_LEFT;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.SIZE_HALF_FIELD;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.SIZE_TILE;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_APPROACH_BASKET;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_APPROACH_CHAMBER;
import static org.firstinspires.ftc.teamcode.opmode.AutonVars.WAIT_EXTEND_SPEC_PRELOAD;
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
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.NEUTRAL;
import static java.lang.Math.atan2;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    private Action asyncIntakeSequence(Robot robot, double extension) {
        return new SequentialAction(
                new InstantAction(() -> {
                     robot.intake.extendo.setTarget(extension);
                     robot.intake.runRoller(0.8);
                }),
                telemetryPacket -> !robot.intake.hasSample(), // wait until intake gets a sample
                new SleepAction(WAIT_POST_INTAKING),
                new InstantAction(() -> robot.intake.runRoller(0))
        );
    }

    private Pose2d chamber(int id) {
        return new Pose2d(chamber0.x - id * DISTANCE_BETWEEN_SPECIMENS, chamber0.y, chamber0.heading);
    }

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

        boolean right = false;
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
                    right = !right;
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

            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");
            mTelemetry.addLine();
            mTelemetry.addLine();
            printConfig(selection, right, cycles, partnerWait);

            mTelemetry.update();
        }

        boolean specimenPreload = !right && robot.deposit.specimenIntaked();

        mTelemetry.addLine("CONFIRMED:");
        mTelemetry.addLine();
        mTelemetry.addLine();
        printConfig(selection, right, cycles, partnerWait);
        mTelemetry.update();

        Pose2d startPose = new Pose2d(
                right ? chamber0.x : specimenPreload ? chamberLeft.x : (SIZE_TILE * -1.5),
                0.5 * (right || specimenPreload ? LENGTH_ROBOT : WIDTH_ROBOT) - SIZE_HALF_FIELD,
                toRadians(right || specimenPreload ? 90 : 0)
        );

        robot.drivetrain.localizer.setPosition(startPose);

        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(startPose);

        Action scoreSpec = new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
                telemetryPacket -> !robot.deposit.reachedTarget(), // wait until deposit in position
                new InstantAction(robot.deposit::triggerClaw),
                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );

        if (right) {
            Action intakeSpec = new SequentialAction(
                    new InstantAction(robot.deposit::triggerClaw),
                    telemetryPacket -> !robot.deposit.specimenIntaked(),
                    new InstantAction(() -> robot.deposit.setPosition(HIGH))
            );

            /// Score preloaded specimen
            builder = builder
                    .strafeTo(chamber0.toVector2d())
                    .stopAndAdd(scoreSpec)
            ;

            if (cycles >= 0) {

                /// Push samples
                builder = builder
                        .setTangent(toRadians(-90))
                        .splineToConstantHeading(aroundBeamPushing.toVector2d(), aroundBeamPushing.heading)
                        .splineToConstantHeading(pushing1.toVector2d(), pushing1.heading)
                        .splineToConstantHeading(pushed1.toVector2d(), pushed1.heading)
                        .splineToConstantHeading(pushing2.toVector2d(), pushing2.heading)
                        .splineToConstantHeading(pushed2.toVector2d(), pushed2.heading)
                        .splineToConstantHeading(pushing3.toVector2d(), pushing3.heading)
                        .splineToConstantHeading(pushed3.toVector2d(), pushed3.heading)
                        .splineToConstantHeading(intakingFirstSpec.toVector2d(), intakingFirstSpec.heading)
                ;

                /// Cycle specimens
                for (int i = 0; i < cycles; i++) {
                    builder = builder
                            .stopAndAdd(intakeSpec)
                            .setTangent(toRadians(90))
                            .splineToConstantHeading(chamber(i + 1).position, toRadians(90))
                            .stopAndAdd(scoreSpec)
                            .afterTime(0, robot.deposit::triggerClaw)
                            .setTangent(toRadians(-90))
                    ;
                    if (i < cycles - 1) builder = builder
                            .splineToConstantHeading(intakingSpec.toVector2d(), toRadians(-90))
                    ;
                }


            }

            /// Park in observation zone
            builder = builder.strafeTo(parkRight.toVector2d());

        } else {
            Action scoreSample = new SequentialAction(
                    new SleepAction(WAIT_APPROACH_BASKET),
                    telemetryPacket -> !robot.deposit.reachedTarget(),
                    new InstantAction(robot.deposit::triggerClaw),
                    new SleepAction(WAIT_SCORE_BASKET)
            );

            Action raiseLift = new SequentialAction(
                    telemetryPacket -> !robot.deposit.hasSample(),
                    new InstantAction(() -> robot.deposit.setPosition(HIGH))
            );

            if (specimenPreload) {
                /// Score preloaded specimen
                builder = builder
                        .waitSeconds(partnerWait)
                        .strafeTo(chamberLeft.toVector2d())
                        .stopAndAdd(scoreSpec)
                ;
            } else {
                /// Score preloaded sample
                builder = builder
                        .afterTime(0, raiseLift)
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample)
                ;
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
                        .afterTime(i == 0 && specimenPreload ? WAIT_EXTEND_SPEC_PRELOAD : 0, asyncIntakeSequence(robot, millimeters))
                        .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                        .afterTime(0, () -> {
                             if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                        })
                        // wait for intake to get sample:
                        .stopAndAdd(telemetryPacket -> robot.getSample() == null)

                        /// Score
                        .afterTime(0, raiseLift)
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample)
                ;
            }

            /// Level 1 ascent (left park)
            builder = builder
                    .afterTime(0, () -> {
                         robot.deposit.triggerClaw();
                         robot.deposit.triggerClaw();
                         robot.deposit.lift.setTarget(LIFT_PARK_LEFT);
                    });

            if (specimenPreload && cycles == 0)
                builder = builder
                        .setTangent(toRadians(-150))
                        .splineToSplineHeading(aroundBeamParkLeft.toPose2d(), toRadians(90))
                        .splineToConstantHeading(parkLeft.toVector2d(), parkLeft.heading)
                        ;
            else builder = builder.splineTo(parkLeft.toVector2d(), parkLeft.heading);
        }

        Action trajectory = builder.build();

        waitForStart();

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        Actions.runBlocking(new ParallelAction(
                telemetryPacket -> {
                    pose = robot.drivetrain.pose;
                    robot.bulkReader.bulkRead();
                    robot.run();
                    robot.printTelemetry();
                    mTelemetry.update();
                    return opModeIsActive();
                },
                trajectory
        ));
    }

    private void printConfig(AutonConfig selection, boolean isRight, int cycles, double partnerWait) {
        mTelemetry.addLine((isRedAlliance ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
        mTelemetry.addLine();
        mTelemetry.addLine((isRight ? "RIGHT " : "LEFT ") + "side" + selection.markIf(EDITING_SIDE));
        mTelemetry.addLine();
        mTelemetry.addLine("Preload sample" + selection.markIf(PRELOAD_SAMPLE));
        mTelemetry.addLine();
        mTelemetry.addLine("Preload specimen" + selection.markIf(PRELOAD_SPECIMEN));
        mTelemetry.addLine();
        mTelemetry.addLine(cycles + "cycles" + selection.markIf(EDITING_CYCLES));
        mTelemetry.addLine();
        mTelemetry.addLine("Pause after spike: " + partnerWait + selection.markIf(EDITING_WAIT));
    }
}
