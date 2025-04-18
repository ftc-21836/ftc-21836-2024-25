package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.CONFIRMING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_CYCLES;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_PARTNER_SAMPLE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_PARTNER_SAMPLE_X;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_PUSHING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_SUB_1_Y;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_SUB_2_Y;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_WAIT;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_2;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
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
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.Arrays;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class Auto extends LinearOpMode {


    enum State {
        SCORING_PRELOAD,
        INTAKING_PARTNER_SAMPLE,
        SCORING_PARTNER_SAMPLE,
        INTAKING_1,
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
            DEAD_TIME = 0.5,
            LENGTH_ROBOT = 14.2,
            WIDTH_ROBOT = 14.2,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,
            DISTANCE_FROM_BASKET_SWEEP = 30,
            EXTEND_SAMPLE_1 = 21,
            EXTEND_SAMPLE_2 = 20,
            EXTEND_SAMPLE_3 = 20,

            TIME_EXTEND = 1,
            TIME_RETRACT = 1,
            SPEED_EXTEND = 1,
            SPEED_RETRACT = -0.6,

            EXTEND_OVER_SUB_BAR = 2,
            EXTEND_SUB = 14.173228346456694,

            WAIT_RE_SWEEP = 1,
            ANGLE_PRE_SLAM = 0.1,
            WAIT_PRE_SLAM_BUCKET = 0.25,
            ANGLE_SLAMMING = 1.1,
            WAIT_SLAMMING_BUCKET = 0.35,
            SPEED_SLAMMING = -0.75,
            SPEED_INTAKING = 1,


            SPEED_SPIKE_TURNING = 2,
            SPEED_SWEEPING_SUB = 6.5,
            SPEED_SWEEPING_SUB_TURNING = 0.5,
            SPEED_INCHING = 5,
            SPEED_INCHING_TURNING = 0.75,

            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_POST_INTAKING = 0,
            WAIT_SCORE_BASKET = 0.25,
            WAIT_SCORE_CHAMBER = 0.1,
            WAIT_DROP_TO_EXTEND = 0.2,
            WAIT_INTAKE_RETRACT_POST_SUB = 0,
            WAIT_EXTEND_MAX_SPIKE = 3,

            LIFT_HEIGHT_TOLERANCE = 3.75,

            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,

            Y_INCHING_FORWARD_WHEN_INTAKING = 10,
            Y_OFFSET_SUB_APPROACHES = 4,

            TIME_CYCLE = 3,
            TIME_SCORE = 0.5;

    /// <a href="https:///www.desmos.com/calculator/l8pl2gf1mb">Adjust spikes 1 and 2</a>
    /// <a href="https://www.desmos.com/calculator/sishohvpwc">Visualize spike samples</a>
    public static EditablePose
            admissibleError = new EditablePose(1, 1, toRadians(2)),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            intakingPartnerSample = new EditablePose(-29,7 - SIZE_HALF_FIELD, 0),

            intaking1 = new EditablePose(-61, -54, PI/3),
            intaking2 = new EditablePose(-62, -52, 1.4632986527692424),
            intaking3 = new EditablePose(-61, -50, 2 * PI / 3),

            sample1 = new EditablePose(-51, -26.8, PI / 2),
            sample2 = new EditablePose(-58, -27.4, PI / 2),
            sample3 = new EditablePose(-68.5, -27.8, PI / 2),

            basket2 = new EditablePose(-63, -54, 1),
            basket3 = new EditablePose(-54.5, -54.5, PI / 4),

            sub1 = new EditablePose(-22.5, -11, 0),
            sub2 = new EditablePose(sub1.x, -4, 0),

            basketFromSub = new EditablePose(-59, -55, 0.765),

            aroundBeamPushing = new EditablePose(35, -30, PI / 2),

            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, - PI / 2),
            chamberLeft = new EditablePose(-chamberRight.x, -33, PI / 2),

            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(63, pushing1.y, - PI / 2),

            intakingSpec = new EditablePose(36, -60, PI / 2),

            pushed1 = new EditablePose(pushing1.x, -46, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, intakingSpec.y, - PI / 2);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    enum AutonConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PRELOAD,
        EDITING_PARTNER_SAMPLE,
        EDITING_PARTNER_SAMPLE_X,
        EDITING_WAIT,
        EDITING_SUB_1_Y,
        EDITING_SUB_2_Y,
        EDITING_PUSHING,
        EDITING_CYCLES,
        CONFIRMING;

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

        intaking1.heading = intaking1.angleTo(sample1);
        intaking2.heading = intaking2.angleTo(sample2);
        intaking3.heading = intaking3.angleTo(sample3);

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, pose);
//        robot.deposit.claw.turnToAngle(ANGLE_CLAW_SAMPLE);

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        AutonConfig selection = EDITING_ALLIANCE;

        boolean specimenSide = false;
        int partnerWait = 0;
        int cycles = 1;
        boolean push = false;
        boolean specimenPreload = false;
        boolean usePartnerSample = false;

        ElapsedTime timer = new ElapsedTime();

        EditablePose sub1Edited = sub1.clone(), sub2Edited = sub2.clone();

        config:
        while (opModeInInit() && timer.seconds() < 5) {
            gamepadEx1.readButtons();

            boolean up = gamepadEx1.wasJustPressed(DPAD_UP);
            boolean down = gamepadEx1.wasJustPressed(DPAD_DOWN);
            boolean y = gamepadEx1.wasJustPressed(Y);
            boolean x = gamepadEx1.wasJustPressed(X);
            boolean a = gamepadEx1.wasJustPressed(A);
            boolean b = gamepadEx1.wasJustPressed(B);

            if (up || down || y || a || x || b) timer.reset();

            if (up) {
                do selection = selection.plus(-1);
                while (
                        selection == EDITING_PRELOAD && specimenSide ||
                        selection == EDITING_PARTNER_SAMPLE && specimenSide ||
                        selection == EDITING_PARTNER_SAMPLE_X && (specimenSide || !usePartnerSample) ||
                        selection == EDITING_WAIT && !specimenSide && !specimenPreload ||
                        selection == EDITING_SUB_1_Y && specimenSide ||
                        selection == EDITING_SUB_2_Y && specimenSide ||
                        selection == EDITING_PUSHING && !specimenSide ||
                        selection == EDITING_CYCLES && !specimenSide
                );
            } else if (down) {
                do selection = selection.plus(1);
                while (
                        selection == EDITING_PRELOAD && specimenSide ||
                        selection == EDITING_PARTNER_SAMPLE && specimenSide ||
                        selection == EDITING_PARTNER_SAMPLE_X && (specimenSide || !usePartnerSample) ||
                        selection == EDITING_WAIT && !specimenSide && !specimenPreload ||
                        selection == EDITING_SUB_1_Y && specimenSide ||
                        selection == EDITING_SUB_2_Y && specimenSide ||
                        selection == EDITING_PUSHING && !specimenSide ||
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
                case EDITING_PARTNER_SAMPLE:
                    if (!specimenSide && x) usePartnerSample = !usePartnerSample;
                    break;
                case EDITING_PARTNER_SAMPLE_X:
                    if (!specimenSide && usePartnerSample && b) intakingPartnerSample.x++;
                    if (!specimenSide && usePartnerSample && x) intakingPartnerSample.x--;
                    break;
                case EDITING_WAIT:
                    if ((specimenPreload || specimenSide) && y) partnerWait++;
                    if ((specimenPreload || specimenSide) && a && partnerWait > 0) partnerWait--;
                    break;
                case EDITING_SUB_1_Y:
                    if (!specimenSide && y && sub1Edited.y < -sub1.y) sub1Edited.y++;
                    if (!specimenSide && a && sub1Edited.y > sub1.y) sub1Edited.y--;
                    break;
                case EDITING_SUB_2_Y:
                    if (!specimenSide && y && sub2Edited.y < -sub1.y) sub2Edited.y++;
                    if (!specimenSide && a && sub2Edited.y > sub1.y) sub2Edited.y--;
                    break;
                case EDITING_PUSHING:
                    if (specimenSide && x) push = !push;
                    break;
                case EDITING_CYCLES:
                    if (specimenSide && y) cycles++;
                    if (specimenSide && a && cycles > 0) cycles--;
                    break;
                case CONFIRMING:
                    if (x) break config;
            }

            gamepad1.setLedColor(
                    isRedAlliance ? 1 : 0,
                    0,
                    !isRedAlliance ? 1 : 0,
                    Gamepad.LED_DURATION_CONTINUOUS
            );

            mTelemetry.addLine((isRedAlliance ? "Red" : "Blue") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine((specimenSide ? "Right (observation-side)" : "Left (basket-side)") + selection.markIf(EDITING_SIDE));
            if (!specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine((specimenPreload ? "Specimen" : "Sample") + " preload" + selection.markIf(EDITING_PRELOAD));
                mTelemetry.addLine();
                mTelemetry.addLine((usePartnerSample ? "Teamwork makes the dream work" : "Lonely sad") + selection.markIf(EDITING_PARTNER_SAMPLE));
                if (usePartnerSample) {
                    mTelemetry.addLine();
                    mTelemetry.addLine("Partner sample X = " + intakingPartnerSample.x + selection.markIf(EDITING_PARTNER_SAMPLE_X));
                }
            }
            if (specimenPreload || specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine("Wait " + partnerWait + " sec" + (partnerWait == 1 ? "" : "s") + " before specimen preload" + selection.markIf(EDITING_WAIT));
            }
            if (!specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine("First sub intaking Y = " + sub1Edited.y + " (default " + sub1.y + ")" + selection.markIf(EDITING_SUB_1_Y));
                mTelemetry.addLine();
                mTelemetry.addLine("Second sub intaking Y = " + sub2Edited.y + " (default " + sub2.y + ")" + selection.markIf(EDITING_SUB_2_Y));
            }
            if (specimenSide) {
                mTelemetry.addLine();
                mTelemetry.addLine((push ? "Push samples after preload" : "Go directly to observation zone") + selection.markIf(EDITING_PUSHING));
                mTelemetry.addLine();
                mTelemetry.addLine(cycles + " specimen" + (cycles == 1 ? "" : "s") + " from observation zone" + selection.markIf(EDITING_CYCLES));
            }
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Confirm configuration (confirming in " + (int) ceil(5 - timer.seconds()) + " seconds)" + selection.markIf(CONFIRMING));

            mTelemetry.update();
        }

        specimenPreload = specimenSide || specimenPreload;

        robot.intake.setAlliance(isRedAlliance);

        Action trajectory;

        if (specimenSide) {

            robot.deposit.preloadSpecimen();

            pose = new Pose2d(chamberRight.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, - PI / 2);

            TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(pose);

            /// Score preloaded specimen
            builder = builder
                    .waitSeconds(partnerWait)
                    .strafeTo(chamberRight.toVector2d())
                    .stopAndAdd(scoreSpecimen(robot))
            ;

            if (cycles > 0) {

                /// Push samples
                if (push) {
                    builder = builder
                            .afterTime(0, robot.deposit::nextState)
                            .setTangent(- PI / 2);

                    EditablePose[] pushingPoses = {aroundBeamPushing, pushing1, pushed1, pushing2, pushed2, pushing3, pushed3};
                    for (EditablePose pose : pushingPoses) {
                        builder = builder.splineToConstantHeading(pose.toVector2d(), pose.heading);
                    }
                }

                double[] chamberXs = {
                        X_OFFSET_CHAMBER_1,
                        X_OFFSET_CHAMBER_2,
                        X_OFFSET_CHAMBER_3,
                        X_OFFSET_CHAMBER_4,
                };

                /// Cycle specimens
                for (int i = 0; i < min(chamberXs.length, cycles); i++) {
                    if (!push || i > 0) builder = builder
                            .afterTime(0, robot.deposit::nextState)
                            .setTangent(- PI / 2)
                            .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                    ;
                    builder = builder
                            .waitSeconds(WAIT_APPROACH_WALL)
                            .afterTime(0, robot.deposit::nextState)
                            .stopAndAdd(telemetryPacket -> !robot.deposit.intaked())
                            .setTangent(PI / 2)
                            .splineToConstantHeading(new Vector2d(chamberRight.x + chamberXs[i] * DISTANCE_BETWEEN_SPECIMENS, chamberRight.y), PI / 2)
                            .stopAndAdd(scoreSpecimen(robot))
                    ;
                }

            }

            /// Park in observation zone
            builder = builder.strafeTo(intakingSpec.toVector2d());

            trajectory = builder.build();

        } else {

            robot.deposit.preloadSample();

            robot.intake.retractBucketBeforeExtendo = false;
//            robot.deposit.pauseBeforeAutoRetractingLift = false;

//            if (specimenPreload) {
//                robot.deposit.preloadSpecimen();
//                robot.deposit.lift.setTarget(HEIGHT_CHAMBER_HIGH);
//            } else {
////                robot.deposit.preloadSample();
//                robot.deposit.lift.setTarget(HEIGHT_BASKET_HIGH);
//            }

            pose = specimenPreload ?
                    new Pose2d(chamberLeft.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, PI / 2) :
                    new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);

            AngularVelConstraint spikeConstraint = new AngularVelConstraint(SPEED_SPIKE_TURNING);

            MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(SPEED_INCHING),
                    new AngularVelConstraint(SPEED_INCHING_TURNING)
            ));

            MinVelConstraint sweepConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(SPEED_SWEEPING_SUB),
                    new AngularVelConstraint(SPEED_SWEEPING_SUB_TURNING)
            ));

            // wait until deposit in position
            Action scorePreload = robot.drivetrain.actionBuilder(pose)
                    .strafeToLinearHeading(intaking1.toVector2d(), intaking1.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            {
//            Action intakePartnerSample = robot.drivetrain.actionBuilder(basket3.toPose2d())
//                    .afterTime(0, () -> {
//                        robot.intake.runRoller(SPEED_INTAKING);
//                    })
//                    .strafeToSplineHeading(intakingPartnerSample.toVector2d(), intakingPartnerSample.heading, spikeConstraint)
//                    .afterTime(0, () -> {
//                        robot.intake.runRoller(SPEED_INTAKING);
//                        robot.intake.extendo.setExtended(true);
//                        timer.reset();
//                    })
//                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(410)))
//                    .setTangent(intakingPartnerSample.heading)
//                    .lineToX(intakingPartnerSample.x + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//                    .build();
//
//            Action scorePartnerSample = robot.drivetrain.actionBuilder(intakingPartnerSample.toPose2d())
////                    .afterTime(0, preExtend(robot, PRE_EXTEND_1))
//                    .stopAndAdd(finishIntaking(robot))
//                    .strafeToSplineHeading(intaking1.toVector2d(), intaking1.heading)
//                    .stopAndAdd(scoreSample(robot))
//                    .build();
//
//            Action intakingPartnerTo1 = robot.drivetrain.actionBuilder(intakingPartnerSample.toPose2d())
//                    .afterTime(0, () -> {
//                        robot.intake.runRoller(SPEED_INTAKING);
//                        robot.intake.extendo.setTarget(PRE_EXTEND_1);
//                    })
//                    .strafeToSplineHeading(intaking1.toVector2d(), intaking1.heading, spikeConstraint)
//                    .afterTime(0, () -> {
//                        robot.intake.runRoller(SPEED_INTAKING);
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
//                        timer.reset();
//                    })
//                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)))
//                    .setTangent(intaking1.heading)
//                    .lineToY(intaking1.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//                    .build();
            }

            Action intake1 = robot.drivetrain.actionBuilder(intaking1.toPose2d())
                    .afterTime(0, () -> {
                        robot.intake.setRollerAndAngle(SPEED_INTAKING);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
                        timer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)))
                    .setTangent(intaking1.heading)
                    .lineToY(intaking1.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score1 = robot.drivetrain.actionBuilder(intaking1.toPose2d())
                    .stopAndAdd(finishIntaking(robot))
                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action intake2 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                    .afterTime(0, () -> {
                        robot.intake.setRollerAndAngle(SPEED_INTAKING);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                        timer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                    .setTangent(intaking2.heading)
                    .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score2 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                    .stopAndAdd(finishIntaking(robot))
                    .strafeToLinearHeading(basket2.toVector2d(), basket2.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action intake3 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                    .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading, spikeConstraint)
                    .afterTime(0, () -> {
                        robot.intake.setRollerAndAngle(SPEED_INTAKING);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                        timer.reset();
                    })
                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)))
                    .setTangent(PI / 2)
                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score3 = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                    .stopAndAdd(finishIntaking(robot))
                    .strafeToLinearHeading(basket3.toVector2d(), basket3.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action i3ToSub = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                    .setTangent(basket3.heading)
                    .splineToSplineHeading(sub1Edited.toPose2d(), sub1Edited.heading)
                    .stopAndAdd(approachSub(robot))
                    .build();

            Action toSub1 = robot.drivetrain.actionBuilder(basket3.toPose2d())
                    .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR))
                    .setTangent(basket3.heading)
                    .splineTo(sub1Edited.toVector2d(), sub1Edited.heading)
                    .stopAndAdd(approachSub(robot))
                    .build();

            Action intakingSub1 = robot.drivetrain.actionBuilder(sub1Edited.toPose2d())
                    .setTangent(PI / 2)
                    .lineToY(-sub1.y, sweepConstraint)
                    .build();

            Action park = robot.drivetrain.actionBuilder(basketFromSub.toPose2d())
                    .afterTime(0, () -> {
                        robot.deposit.lift.setTarget(0);
                    })
                    .splineTo(sub1.toVector2d(), sub1.heading)
                    .build();

            boolean usePartnerSample_Final = usePartnerSample;

            trajectory = new Action() {

                State state = SCORING_PRELOAD;

                ElapsedTime matchTimer = null;

                Action activeTraj = scorePreload;

                int subCycle = 1;

                boolean extending = false;

                void stopDt() {
                    robot.drivetrain.leftFront.setPower(0);
                    robot.drivetrain.leftBack.setPower(0);
                    robot.drivetrain.rightBack.setPower(0);
                    robot.drivetrain.rightFront.setPower(0);
                }

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();

                    boolean trajDone = !activeTraj.run(p);

                    switch (state) {
                        case SCORING_PRELOAD:

                            if (trajDone) {
                                if (usePartnerSample_Final) {
//                                    activeTraj = intakePartnerSample;
//                                    state = INTAKING_PARTNER_SAMPLE;
                                } else {
                                    activeTraj = intake1;
                                    state = INTAKING_1;
                                }
                            }
                            break;

//                        case INTAKING_PARTNER_SAMPLE:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample()) {
//                                activeTraj = scorePartnerSample;
//                                state = SCORING_PARTNER_SAMPLE;
//                                stopDt();
//                            } else if (trajDone) { // skip to 2 if didn't get 1
//                                activeTraj = intakingPartnerTo1;
//                                state = INTAKING_1;
//                                robot.intake.extendo.setExtended(false);
//                                robot.intake.ejectSample();
//                            }
//
//                            break;
//
//                        case SCORING_PARTNER_SAMPLE:
//                            if (trajDone) {
//                                activeTraj = intake1;
//                                state = INTAKING_1;
//                            }
//                            break;

                        case INTAKING_1:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                activeTraj = score1;
                                state = SCORING_1;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 2 if didn't get 1
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .afterTime(0, () -> robot.intake.extendo.setExtended(false))
                                        .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                                        .afterTime(0, () -> {
                                            robot.intake.setRollerAndAngle(SPEED_INTAKING);
                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                                            timer.reset();
                                        })
                                        .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                                        .setTangent(intaking2.heading)
                                        .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                                        .build();
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
                                activeTraj = score2;
                                state = SCORING_2;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 3 if didn't get 2
                                activeTraj = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                                        .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading)
                                        .afterTime(0, () -> {
                                            robot.intake.setRollerAndAngle(SPEED_INTAKING);
                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                                            timer.reset();
                                        })
                                        .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                                        .setTangent(intaking3.heading)
                                        .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                                        .build();
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
                                activeTraj = score3;
                                state = SCORING;
                                stopDt();
                            }
                            else if (trajDone) { // skip to sub if didn't get 3
                                activeTraj = i3ToSub;
                                state = DRIVING_TO_SUB;
                                robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR);
                                robot.intake.setRollerAndAngle(0);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING:
                            if (trajDone) {
                                if (remaining < TIME_CYCLE
//                                        || subCycle > 2
                                ) {
                                    activeTraj = park;
                                    state = PARKING;
                                    stopDt();
                                } else {
                                    Pose2d subPos = new Pose2d(sub1.x, sub1.y + Y_OFFSET_SUB_APPROACHES * (subCycle - 1), 0);
                                    activeTraj = subCycle == 1 ? toSub1 : robot.drivetrain.actionBuilder(basketFromSub.toPose2d())
                                            .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR))
                                            .setTangent(basketFromSub.heading)
                                            .splineTo(subPos.position, subPos.heading)
                                            .stopAndAdd(approachSub(robot))
                                            .build();
                                    state = DRIVING_TO_SUB;
                                }
                            } else if (remaining < WAIT_SCORE_BASKET && robot.deposit.hasSample()) {
                                robot.deposit.nextState();
                            }
                            break;

                        case DRIVING_TO_SUB:
                            if (trajDone) {
                                activeTraj = intakingSub1;
                                state = INTAKING;
                                timer.reset();
                                extending = true;
                            }
                            break;
                        case INTAKING:

                            if (remaining < TIME_SCORE) {
                                robot.intake.extendo.runManual(0);
                                robot.intake.ejectSample();
                                robot.intake.setRollerAndAngle(0);
                                robot.deposit.lvl1Ascent = true;
                                timer.reset();
                                stopDt();
                                state = PARKING;
                                break;
                            }

                            // Sample intaked
                            if (robot.intake.hasSample()) {

                                robot.intake.extendo.runManual(0);

                                Pose2d current = robot.drivetrain.pose;
                                double y = current.position.y;

                                robot.intake.retractBucketBeforeExtendo = true;

                                activeTraj = robot.drivetrain.actionBuilder(new Pose2d(sub1.x, y, 0))
                                        .stopAndAdd(finishIntaking(robot))
                                        .stopAndAdd(new SequentialAction(
                                                new InstantAction(() -> robot.intake.setAngle(ANGLE_PRE_SLAM)),
                                                new SleepAction(WAIT_PRE_SLAM_BUCKET),
                                                new InstantAction(() -> robot.intake.setRoller(SPEED_SLAMMING)),
                                                new InstantAction(() -> robot.intake.setAngle(ANGLE_SLAMMING)),
                                                new SleepAction(WAIT_SLAMMING_BUCKET),
                                                new InstantAction(() -> robot.intake.setRollerAndAngle(0))
                                        ))
                                        .setTangent(PI + current.heading.toDouble())
                                        .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                                        .splineTo(basketFromSub.toVector2d(), PI + basketFromSub.heading)
                                        .stopAndAdd(scoreSample(robot))
                                        .build();

                                subCycle++;
                                state = SCORING;
                                stopDt();
                            } else {

//                                robot.intake.extendo.runManual(extending ? SPEED_EXTEND : SPEED_RETRACT);
                                if (robot.intake.extendo.atPosition(extending ? EXTEND_SUB : EXTEND_OVER_SUB_BAR) || timer.seconds() >= (extending ? TIME_EXTEND : TIME_RETRACT)) {
                                    timer.reset();
                                    extending = !extending;
                                    robot.intake.extendo.setTarget(extending ? EXTEND_OVER_SUB_BAR : EXTEND_SUB);
                                    if (!extending) robot.intake.ejectSample();
                                }

                                // sweep the other way
                                if (trajDone) {

                                    double y = robot.drivetrain.pose.position.y;

                                    activeTraj = robot.drivetrain.actionBuilder(new Pose2d(sub1.x, y, 0))
                                        .afterTime(0, () -> {
                                            robot.intake.ejectSample();
                                            robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR);
                                            timer.reset();
                                        })
                                        .stopAndAdd(t -> !(robot.intake.extendo.atPosition(EXTEND_OVER_SUB_BAR) || timer.seconds() >= TIME_RETRACT))
                                        .waitSeconds(WAIT_RE_SWEEP)
                                        .stopAndAdd(approachSub(robot))
                                        .setTangent(y > 0 ? - PI / 2 : PI / 2)
                                        .lineToY(y > 0 ? sub1.y : -sub1.y, sweepConstraint)
                                        .build();
                                }

                            }

                            break;

                        case PARKING:
                            robot.deposit.lvl1Ascent = true;
                            if (timer.seconds() >= 1) robot.intake.extendo.setExtended(false);
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


        mTelemetry.addLine("AUTONOMOUS READY");
        mTelemetry.update();

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.pinpoint.setPositionRR(pose);

        Actions.runBlocking(auton);

        Thread.sleep((long) (DEAD_TIME * 1000));
    }

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> {
                    if (!robot.hasSample()) robot.intake.transfer(NEUTRAL);
                }),
                new SleepAction(WAIT_APPROACH_BASKET),
                telemetryPacket -> !(robot.deposit.basketReady() && abs(robot.deposit.lift.getPosition() - HEIGHT_BASKET_HIGH) <= LIFT_HEIGHT_TOLERANCE),
                new InstantAction(robot.deposit::nextState),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action scoreSpecimen(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> {
                    if (!robot.hasSample()) while (!robot.deposit.chamberReady()) robot.deposit.nextState();
                }),
                new SleepAction(WAIT_APPROACH_CHAMBER),
//                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SPECIMEN) && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
                new InstantAction(robot.deposit::nextState),
                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

    private static Action finishIntaking(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.setRollerAndAngle(1)),
                new SleepAction(WAIT_POST_INTAKING),
                new InstantAction(() -> robot.intake.setRollerAndAngle(0))
        );
    }

    private static Action approachSub(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.setRollerAndAngle(SPEED_INTAKING)),
                new SleepAction(WAIT_DROP_TO_EXTEND),
                new InstantAction(() -> robot.intake.extendo.setTarget(EXTEND_SUB))
//                new SleepAction(TIME_EXTEND)
        );
    }

}
