package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_CYCLES;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_WAIT;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.AutonConfig.EDITING_PRELOAD;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_CHAMBER_HIGH;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

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
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

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
            SPEED_INTAKING = 0.875,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_POST_INTAKING = 0.5,
            WAIT_SCORE_BASKET = 0.25,
            WAIT_SCORE_CHAMBER = 0.75,
            WAIT_DROP_TO_EXTEND = 0.75,
            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3;

    public static EditablePose
            sample1 = new EditablePose(-48, -27.75, PI / 2),
            sample1SpecPreload = new EditablePose(-50, -27.75, sample1.heading),
            sample2 = new EditablePose(-58, -27.75, sample1.heading),
            sample3 = new EditablePose(-68.75, -26.5, sample1.heading),
            basket = new EditablePose(-57.5, -57.5, PI / 4),
            intaking1 = new EditablePose(-50, -46, toRadians(84.36)),
            intaking1SpecPreload = new EditablePose(-51, -46, toRadians(84.36)),
            intaking2 = new EditablePose(-54, -45, toRadians(105)),
            intaking3 = new EditablePose(-54, -43, 2 * PI / 3),
            aroundBeamParkLeft = new EditablePose(-40, -25, 0),
            parkLeft = new EditablePose(-22, -11, 0),
            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, PI / 2),
            chamberLeft = new EditablePose(-chamberRight.x, chamberRight.y, chamberRight.heading),
            aroundBeamPushing = new EditablePose(35, -30, PI / 2),
            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, -13, toRadians(-70)),
            pushing3 = new EditablePose(62, -13, - PI / 2),
            pushed1 = new EditablePose(46, -50, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, -50, toRadians(110)),
            pushed3 = new EditablePose(62, -50, - PI / 2),
            intakingSpec = new EditablePose(36, -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5, PI / 2),
            intakingFirstSpec = new EditablePose(55, intakingSpec.y, -intakingSpec.heading),
            parkRight = new EditablePose(36, -60, PI / 2);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    enum AutonConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PRELOAD,
        EDITING_CYCLES,
        EDITING_WAIT;

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
                selection = selection.plus(-1);
                if (specimenSide && selection == EDITING_PRELOAD) selection = selection.plus(-1);
            }
            else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                selection = selection.plus(1);
                if (specimenSide && selection == EDITING_PRELOAD) selection = selection.plus(1);
            }

            switch (selection) {
                case EDITING_ALLIANCE:
                    if (gamepadEx1.wasJustPressed(X)) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    if (gamepadEx1.wasJustPressed(X)) specimenSide = !specimenSide;
                    break;
                case EDITING_PRELOAD:
                    if (gamepadEx1.wasJustPressed(X) && !specimenSide) specimenPreload = !specimenPreload;
                    break;
                case EDITING_CYCLES:
                    if (gamepadEx1.wasJustPressed(Y)) cycles++;
                    if (gamepadEx1.wasJustPressed(A) && cycles > 0) cycles--;
                    break;
                case EDITING_WAIT:
                    if (gamepadEx1.wasJustPressed(Y)) partnerWait++;
                    if (gamepadEx1.wasJustPressed(A) && partnerWait > 0) partnerWait--;
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
            mTelemetry.addLine();
            if (!specimenSide) {
                mTelemetry.addLine("Preloading a " + (specimenPreload ? "specimen" : "sample") + selection.markIf(EDITING_PRELOAD));
                mTelemetry.addLine();
            }
            mTelemetry.addLine(cycles + " cycles" + selection.markIf(EDITING_CYCLES));
            mTelemetry.addLine();
            mTelemetry.addData("Wait before scoring specimen preload (sec)", partnerWait + selection.markIf(EDITING_WAIT));

            mTelemetry.update();
        }

        specimenPreload = specimenSide || specimenPreload;

        if (specimenPreload) robot.deposit.preloadSpecimen();
        else robot.deposit.transfer(NEUTRAL);

        mTelemetry.addLine("GENERATING TRAJECTORY...");
        mTelemetry.update();

        mTelemetry.addLine("TRAJECTORY GENERATED:");
        mTelemetry.addLine();
        mTelemetry.addLine("> " + (isRedAlliance ? "Red" : "Blue") + " alliance");
        robot.intake.setAlliance(isRedAlliance);
        robot.deposit.setAlliance(isRedAlliance);

        pose = new Pose2d(
                specimenSide ? chamberRight.x : specimenPreload ? chamberLeft.x : 0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE,
                0.5 * (specimenPreload ? LENGTH_ROBOT : WIDTH_ROBOT) - SIZE_HALF_FIELD,
                specimenPreload ? PI / 2 : 0
        );

        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(pose);

        if (specimenSide) {

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
            builder = builder.strafeTo(parkRight.toVector2d());

            mTelemetry.addLine("> Park in observation zone");

        } else {

            mTelemetry.addLine("> Left side (near basket)");

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
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample(robot));

                mTelemetry.addLine("> Score preloaded sample");
            }

            double[] extendoMMs = {EXTEND_SAMPLE_1, EXTEND_SAMPLE_2, EXTEND_SAMPLE_3};
            EditablePose[] intakingPositions = {intaking1, intaking2, intaking3};
            EditablePose[] samplePositions = {sample1, sample2, sample3};

            if (specimenPreload) {
                intakingPositions[0] = intaking1SpecPreload;
                samplePositions[0] = sample1SpecPreload;
            }

            /// Cycle samples off the floor
            for (int i = 0; i < min(intakingPositions.length, cycles); i++) {

                boolean firstAfterSpec = i == 0 && specimenPreload;

                double millimeters = extendoMMs[i];
                EditablePose intakingPos = intakingPositions[i];
                EditablePose samplePos = samplePositions[i];

                // Calculate angle to point intake at floor sample
                intakingPos.heading = atan2(samplePos.y - intakingPos.y, samplePos.x - intakingPos.x);

                /// Drop bucket now (if not specimen preload)
                if (!firstAfterSpec) builder = builder
                        .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING));

                /// Drive to intaking position
                builder = builder
                        .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                        .afterTime(0, new SequentialAction(
                                telemetryPacket -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(millimeters)),
                                new InstantAction(() -> {
                                    if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                                })
                        ));

                /// Drop bucket after reaching pos (if specimen preload)
                if (firstAfterSpec) builder = builder
                        .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                        .waitSeconds(WAIT_DROP_TO_EXTEND);

                /// Intaking sample
                builder = builder
                        .afterTime(0, () -> robot.intake.extendo.setTarget(millimeters))
                        .stopAndAdd(telemetryPacket -> !robot.intake.hasSample()) // wait until intake gets a sample
                        .waitSeconds(WAIT_POST_INTAKING)
                        .afterTime(0, () -> robot.intake.runRoller(0))
                /// Score
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample(robot));

                mTelemetry.addLine("> Sample cycle " + (i + 1));
            }

            /// Raise arm for level 1 ascent
            builder = builder.afterTime(0, () -> {
                Deposit.level1Ascent = true;
                robot.deposit.lift.setTarget(0);
            });

            mTelemetry.addLine("> Raise arm for level 1 ascent");

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
                telemetryPacket -> {
                    pose = robot.drivetrain.pose;
                    robot.run();
                    return opModeIsActive();
                }
        );

        mTelemetry.update();

        while (opModeInInit()) {
            if (specimenPreload) robot.deposit.arm.run(true);
        }

        //------------------------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.pinpoint.setPositionRR(pose);

        Actions.runBlocking(auton);
    }

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_BASKET),
                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SAMPLE) && robot.deposit.lift.atPosition(HEIGHT_BASKET_HIGH)),
                new InstantAction(robot.deposit::triggerClaw),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action scoreSpecimen(Robot robot) {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SPECIMEN) && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
                new InstantAction(robot.deposit::triggerClaw),
                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

}
