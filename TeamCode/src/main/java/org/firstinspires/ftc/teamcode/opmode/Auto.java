package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.CONFIRMING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_CYCLES;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.TAKING_PICTURE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SUB_INTAKING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
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
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.FirstTerminateAction;
import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.vision.AutoSampleAligner;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.control.vision.LimelightEx;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotorEx;

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
        TAKING_PICTURE,
        SUB_INTAKING,
        PARKING
    }

    public static MultipleTelemetry mTelemetry;

    public static void divider() {
        mTelemetry.addLine();
        mTelemetry.addLine("--------------------------------------------------------------------------");
        mTelemetry.addLine();
    }

    public static double
            DEAD_TIME = 0,
            LENGTH_ROBOT = 14.386976771653545,
            WIDTH_ROBOT = 14.28740157480315,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,

            LL_ANGLE_BUCKET_INCREMENT = 50,
            LL_DISTANCE_START_LOWERING = 13,
            LL_EXTEND_OFFSET = -9.5,
            LL_MAX_PICTURE_TIME = 3,
            LL_MIN_PICTURE_TIME = 0,
            LL_NO_DETECTION_Y_MOVE = 3,
            LL_SPEED_MAX_EXTENDO = 1,
            LL_SWEEP_ANGLE_RANGE = 10,
            LL_SWEEP_SPEED = 0.5,
            LL_WAIT_INTAKE = 1,

            WAIT_MAX_INTAKE = 1,

            ANGLE_PITCH_SPIKES = 0.5,
            ANGLE_PITCH_FROM_SUB = 0.5,

            EXTEND_SAMPLE_1 = 21,
            EXTEND_SAMPLE_2 = 20,
            EXTEND_SAMPLE_3 = 24,

            PRE_EXTEND_SAMPLE_1 = 12.5,
            PRE_EXTEND_SAMPLE_2 = 12,
            PRE_EXTEND_SAMPLE_3 = 12,

            VEL_INCHING = 5,
            VEL_ANG_INCHING = 0.75,
            VEL_ANG_INTAKING_3 = 2,

            WAIT_BEFORE_RE_SEARCH = 0.25,
            WAIT_BEFORE_SCORING_PRELOAD = 0.8,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_SCORE_BASKET = 0.2,
            WAIT_SCORE_CHAMBER = 0.1,
            WAIT_INTAKE_RETRACT_POST_SUB = 0,
            WAIT_EXTEND_MAX_SPIKE = 2,

            LIFT_HEIGHT_TOLERANCE = 3.75,

            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,

            Y_INCHING_FORWARD_WHEN_INTAKING = 10,

            TIME_CYCLE = 3;

    /// <a href="https:///www.desmos.com/calculator/l8pl2gf1mb">Adjust spikes 1 and 2</a>
    /// <a href="https://www.desmos.com/calculator/sishohvpwc">Visualize spike samples</a>
    public static EditablePose
            admissibleError = new EditablePose(1, 1, 0.05),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            basketError = new EditablePose(1, 1, toRadians(4)),
            basketVelError = new EditablePose(12, 12, toRadians(30)),

            intaking1 = new EditablePose(-61, -54, PI/3),
            intaking2 = new EditablePose(-62, -51.5, 1.4632986527692424),
            intaking3 = new EditablePose(-59, -50, 2 * PI / 3),

            snapshotPos = new EditablePose(-25, -10, toRadians(20)),

            scoring = new EditablePose(-56, -56, PI / 4),
            scoringFromSub = new EditablePose(-57.25, -57.25, PI/4),

            sample1 = new EditablePose(-48, -26.8, PI / 2),
            sample2 = new EditablePose(-60, -27.4, PI / 2),
            sample3 = new EditablePose(-69, -27.8, PI / 2),

            sub = new EditablePose(-22.5, -11, 0),

            aroundBeamPushing = new EditablePose(35, -30, PI / 2),

            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, - PI / 2),

            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(63, pushing1.y, - PI / 2),

            intakingSpec = new EditablePose(36, -60, PI / 2),

            pushed1 = new EditablePose(pushing1.x, -46, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, intakingSpec.y, - PI / 2),

            targetOffset = new EditablePose(0 ,0, 0);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    enum AutonConfig {
        CONFIRMING,
        EDITING_ALLIANCE,
        EDITING_SIDE,
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
        int cycles = 4;

        ElapsedTime timer = new ElapsedTime();

        config:
        while (opModeInInit() && timer.seconds() < 5) {
            gamepadEx1.readButtons();

            boolean up = gamepadEx1.wasJustPressed(DPAD_UP);
            boolean down = gamepadEx1.wasJustPressed(DPAD_DOWN);
            boolean y = gamepadEx1.wasJustPressed(Y);
            boolean x = gamepadEx1.wasJustPressed(X);
            boolean a = gamepadEx1.wasJustPressed(A);

            if (up || down || y || a || x) timer.reset();

            if (up)
                do selection = selection.plus(-1);
                while (selection == EDITING_CYCLES && !specimenSide);
            else if (down)
                do selection = selection.plus(1);
                while (selection == EDITING_CYCLES && !specimenSide);

            switch (selection) {
                case CONFIRMING:
                    if (x) break config;
                case EDITING_ALLIANCE:
                    if (x) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    if (x) specimenSide = !specimenSide;
                    break;
                case EDITING_CYCLES:
                    if (specimenSide && y) cycles++;
                    if (specimenSide && a && cycles > 0) cycles--;
                    break;
            }

            printConfig(false, timer.seconds(), selection, specimenSide, cycles);
        }

        Limelight3A limelight3a = hardwareMap.get(Limelight3A.class, "limelight");
        AutoSampleAligner sampleAligner = new AutoSampleAligner(new LimelightEx(limelight3a, hardwareMap));
        sampleAligner.activateLimelight(
                specimenSide ?
                /*specimen*/ isRedAlliance ? AutoSampleAligner.Pipeline.RED : AutoSampleAligner.Pipeline.BLUE :
                /*sample*/   isRedAlliance ? AutoSampleAligner.Pipeline.YELLOW_RED : AutoSampleAligner.Pipeline.YELLOW_BLUE
        );
        limelight3a.stop();
        limelight3a.start();

        robot.intake.setAlliance(isRedAlliance);

        Action trajectory;

        if (specimenSide) {

            robot.deposit.preloadSpecimen();

            pose = new Pose2d(chamberRight.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, - PI / 2);

            TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(pose);

            /// Score preloaded specimen
            builder = builder
                    .strafeTo(chamberRight.toVector2d())
                    .stopAndAdd(scoreSpecimen(robot))
            ;

            if (cycles > 0) {

                /// Push samples
                builder = builder
                        .afterTime(0, robot.deposit::nextState)
                        .setTangent(-PI / 2);

                EditablePose[] pushingPoses = {aroundBeamPushing, pushing1, pushed1, pushing2, pushed2, pushing3, pushed3};
                for (EditablePose pose : pushingPoses) {
                    builder = builder.splineToConstantHeading(pose.toVector2d(), pose.heading);
                }

                double[] chamberXs = {
                        X_OFFSET_CHAMBER_1,
                        X_OFFSET_CHAMBER_2,
                        X_OFFSET_CHAMBER_3,
                        X_OFFSET_CHAMBER_4,
                };

                /// Cycle specimens
                for (int i = 0; i < min(chamberXs.length, cycles); i++) {
                    if (i > 0) builder = builder
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
            robot.deposit.requireDistBeforeLoweringLift = false;

            pose = new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);

            TurnConstraints llSweepConstraint = new TurnConstraints(LL_SWEEP_SPEED, -MecanumDrive.PARAMS.maxAngAccel, MecanumDrive.PARAMS.maxAngAccel);

            MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(VEL_INCHING),
                    new AngularVelConstraint(VEL_ANG_INCHING)
            ));

            robot.deposit.setWristPitchingAngle(ANGLE_PITCH_SPIKES);

            // wait until deposit in position
            Action scorePreload = robot.drivetrain.actionBuilder(pose)
                    .afterTime(0, preExtend(robot, PRE_EXTEND_SAMPLE_1))
                    .afterTime(WAIT_BEFORE_SCORING_PRELOAD, scoreSample(robot))
                    .strafeToLinearHeading(intaking1.toVector2d(), intaking1.heading)
                    .build();

            Action intake1 = new SequentialAction(
                    new InstantAction(() -> {
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
                    }),
                    new FirstTerminateAction(
                            t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)),
                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                    ),
                    new SleepAction(WAIT_MAX_INTAKE)
            );

            Action score1 = robot.drivetrain.actionBuilder(intaking1.toPose2d())
                    .afterTime(0, preExtend(robot, PRE_EXTEND_SAMPLE_2))
                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action intake2 = new SequentialAction(
                    new InstantAction(() -> {
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                    }),
                    new FirstTerminateAction(
                            t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)),
                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                    ),
                    new SleepAction(WAIT_MAX_INTAKE)
            );

            Action score2 = new ParallelAction(
                    preExtend(robot, PRE_EXTEND_SAMPLE_3),
                    scoreSample(robot)
            );

            Action intake3 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                    .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading, new AngularVelConstraint(VEL_ANG_INTAKING_3))
                    .stopAndAdd(() -> {
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                    })
                    .stopAndAdd(new FirstTerminateAction(
                            telemetryPacket -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)),
                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                    ))
                    .setTangent(PI / 2)
                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score3 = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                    .stopAndAdd(new InstantAction(() -> robot.intake.setRollerAndAngle(0)))
                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action park = robot.drivetrain.actionBuilder(scoring.toPose2d())
                    .afterTime(0, () -> robot.deposit.lift.setTarget(0))
                    .splineTo(sub.toVector2d(), sub.heading)
                    .build();

            CachedDcMotorEx[] dtMotors = {
                    robot.drivetrain.leftFront,
                    robot.drivetrain.leftBack,
                    robot.drivetrain.rightBack,
                    robot.drivetrain.rightFront,
            };

            trajectory = new Action() {

                State state = SCORING_PRELOAD;

                Action snapshotAction = null;

                ElapsedTime matchTimer = null;

                Action activeTraj = scorePreload;

                int subCycle = 1;

                void stopDt() {
                    for (CachedDcMotorEx motor : dtMotors) motor.setPower(0);
                }

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();

                    boolean trajDone = !activeTraj.run(p);

                    switch (state) {
                        case SCORING_PRELOAD:

                            if (trajDone || atPose(robot, intaking1) && !robot.hasSample()) {
                                stopDt();
                                activeTraj = intake1;
                                state = INTAKING_1;
                            }
                            break;

                        case INTAKING_1:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                robot.intake.setRollerAndAngle(0);
                                activeTraj = score1;
                                state = SCORING_1;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 2 if didn't get 1
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .afterTime(0, () -> robot.intake.extendo.setExtended(false))
                                        .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                                        .stopAndAdd(() -> {
                                            robot.intake.setRollerAndAngle(1);
                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                telemetryPacket -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)),
                                                new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                                        ))
                                        .waitSeconds(WAIT_MAX_INTAKE)
//                                        .setTangent(scoring1Intaking2Scoring2.heading)
//                                        .lineToY(scoring1Intaking2Scoring2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
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
                                robot.intake.setRollerAndAngle(0);
                                activeTraj = score2;
                                state = SCORING_2;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 3 if didn't get 2
                                activeTraj = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                                        .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading)
                                        .stopAndAdd(() -> {
                                            robot.intake.setRollerAndAngle(1);
                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                telemetryPacket -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)),
                                                new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                                        ))
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
                                robot.intake.setRollerAndAngle(0);
                                activeTraj = score3;
                                state = SCORING;
                                stopDt();
                            }
                            else if (trajDone) { // skip to sub if didn't get 3
                                activeTraj = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                                        .setTangent(scoring.heading)
                                        .splineToSplineHeading(snapshotPos.toPose2d(), snapshotPos.heading)
                                        .build();
                                state = DRIVING_TO_SUB;
                                robot.intake.setRollerAndAngle(0);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING:
                            if (trajDone) {
                                if (remaining < TIME_CYCLE) {
                                    activeTraj = park;
                                    state = PARKING;
                                    stopDt();
                                } else {
                                    Pose2d current = robot.drivetrain.pose;
                                    activeTraj = robot.drivetrain.actionBuilder(current)
                                            .setTangent(current.heading.toDouble())
                                            .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
                                            .build();
                                    state = DRIVING_TO_SUB;
                                }
                            }
                            break;

                        case DRIVING_TO_SUB:
                            robot.headlight.setActivated(true);
                            if (trajDone) {
                                state = TAKING_PICTURE;
                                timer.reset();
                                snapshotAction = sampleAligner.detectTarget(LL_MAX_PICTURE_TIME);
                            }
                            break;
                        case TAKING_PICTURE:
                            snapshotAction.run(p);

                            if (timer.seconds() < LL_MIN_PICTURE_TIME) break;

                            EditablePose offset = new EditablePose(sampleAligner.getTargetOffset());

                            if (!(offset.x == 0 && offset.y == 0 && offset.heading == 0)) {

                                targetOffset = offset;
                                double extendoInches = hypot(targetOffset.x, targetOffset.y) + LL_EXTEND_OFFSET;

                                robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;

                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .turn(-targetOffset.heading * 1.25)
                                        .stopAndAdd(() -> {
                                            robot.intake.extendo.setTarget(extendoInches);
                                            robot.intake.setAngle(0.01);
                                            robot.intake.setRoller(0);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> robot.intake.extendo.getPosition() < extendoInches - LL_DISTANCE_START_LOWERING,
                                                new SleepAction(1)
                                        ))
                                        .stopAndAdd(() -> {
                                            robot.intake.setRoller(1);
                                        })
                                        .stopAndAdd(timer::reset)
                                        .afterTime(0, t -> !robot.intake.setAngle(timer.seconds() * LL_ANGLE_BUCKET_INCREMENT))
                                        // .turn(toRadians(LL_SWEEP_ANGLE_RANGE), llSweepConstraint)
                                        // .turn(-2 * toRadians(LL_SWEEP_ANGLE_RANGE), llSweepConstraint)
                                        // .turn(toRadians(LL_SWEEP_ANGLE_RANGE), llSweepConstraint)
                                        .waitSeconds(LL_WAIT_INTAKE)
                                        .build();

                                state = SUB_INTAKING;

                            } else if (timer.seconds() > LL_MAX_PICTURE_TIME) searchAgainForSample(robot);

                            break;

                        case SUB_INTAKING:


                            if (robot.hasSample()) {
                                robot.headlight.setActivated(false);
                                Pose2d current = robot.drivetrain.pose;

                                robot.deposit.requireDistBeforeLoweringLift = true;
                                robot.intake.retractBucketBeforeExtendo = true;
                                robot.intake.extendo.powerCap = 1;
                                robot.deposit.setWristPitchingAngle(0);

                                activeTraj = robot.drivetrain.actionBuilder(current)
                                        .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
                                        .setTangent(PI + current.heading.toDouble())
                                        .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                                        .splineTo(scoringFromSub.toVector2d(), PI + scoringFromSub.heading)
                                        .afterTime(0, () -> robot.deposit.setWristPitchingAngle(ANGLE_PITCH_FROM_SUB))
                                        .stopAndAdd(scoreSample(robot))
                                        .afterTime(0, () -> robot.deposit.setWristPitchingAngle(0))
                                        .build();

                                subCycle++;
                                state = SCORING;
                                stopDt();

                            } else if (trajDone) searchAgainForSample(robot);

                            break;

                        case PARKING:
                            robot.deposit.lvl1Ascent = true;
                            if (timer.seconds() >= 1) robot.intake.extendo.setExtended(false);
                            return !trajDone;
                    }

                    return true;
                }

                private void searchAgainForSample(Robot robot) {
                    robot.intake.setRollerAndAngle(0);
                    robot.intake.extendo.setExtended(false);
                    robot.intake.ejectSample();

                    Pose2d current = robot.drivetrain.pose;
                    activeTraj = new SleepAction(WAIT_BEFORE_RE_SEARCH);
//                            robot.drivetrain.actionBuilder(current)
//                            .setTangent(PI / 2)
//                            .strafeToLinearHeading(snapshotPos.toVector2d(), snapshotPos.heading)
//                            .build();

                    state = DRIVING_TO_SUB;
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

        printConfig(true, 0, selection, specimenSide, cycles);
        mTelemetry.update();

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.pinpoint.setPositionRR(pose);

        Actions.runBlocking(auton);

        if (robot.deposit.hasSample()) Tele.holdingSample = true;

        Thread.sleep((long) (DEAD_TIME * 1000));
    }

    private static void printConfig(boolean confirmed, double t, AutonConfig selection, boolean specimenSide, int cycles) {
        mTelemetry.addLine(confirmed ?
                "AUTONOMOUS READY" :
                "Confirm configuration (confirming in " + (int) ceil(5 - t) + " seconds)" + selection.markIf(CONFIRMING)
        );
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
        mTelemetry.addLine();
        if (specimenSide) {
            mTelemetry.addLine("RIGHT (observation-side)" + selection.markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine((cycles + 1) + "+0 (" + cycles + " from observation zone)" + selection.markIf(EDITING_CYCLES));

        } else mTelemetry.addLine("LEFT (basket-side)" + selection.markIf(EDITING_SIDE));

        mTelemetry.update();
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

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> {
                    if (!robot.hasSample()) robot.intake.transfer(NEUTRAL);
                }),
                new SleepAction(WAIT_APPROACH_BASKET),
                t -> !(robot.deposit.basketReady() && abs(robot.deposit.lift.getPosition() - HEIGHT_BASKET_HIGH) <= LIFT_HEIGHT_TOLERANCE),
                new InstantAction(robot.deposit::nextState),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action preExtend(Robot robot, double length) {
        return new SequentialAction(
                t -> robot.intake.hasSample(),
                t -> robot.deposit.state.ordinal() < Deposit.State.ARM_MOVING_TO_BASKET.ordinal(),
                new InstantAction(() -> {
                    robot.intake.extendo.setTarget(length);
                    robot.intake.setRollerAndAngle(1);
                })
        );
    }

    private static boolean atPose(Robot robot, EditablePose target) {
        EditablePose current = new EditablePose(robot.drivetrain.pose);
        PoseVelocity2d velocityRR = robot.drivetrain.pinpoint.getVelocityRR();
        return  abs(target.x - current.x) <= basketError.x &&
                abs(target.y - current.y) <= basketError.y &&
                abs(target.heading - current.heading) <= basketError.heading &&
                abs(velocityRR.linearVel.x) <= basketVelError.x &&
                abs(velocityRR.linearVel.y) <= basketVelError.y &&
                abs(velocityRR.angVel) <= basketVelError.heading;

    }

}
