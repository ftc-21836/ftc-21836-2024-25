package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {

    public static double
            LENGTH_ROBOT = 14.2,
            WIDTH_ROBOT = 14.2,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,
            EXTEND_SAMPLE_1 = 410,
            EXTEND_SAMPLE_2 = 395,
            EXTEND_SAMPLE_3 = 350,
            EXTEND_OVER_SUB_BAR_1 = 80,
            EXTEND_OVER_SUB_BAR_2 = 100,
            TIME_EXTEND = 0.6,
            TIME_RETRACT = 0.4,
            SPEED_EXTEND = 1,
            SPEED_RETRACT = 0.6,
            SPEED_SWEEPING_SUB = 5,
            SPEED_SWEEPING_SUB_TURNING = 0.5,
            SPEED_INCHING = 5,
            SPEED_INCHING_TURNING = 0.75,
            SPEED_SLAM_BUCKET = 0.4,
            WAIT_SLAM_BUCKET = 0.2,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_POST_INTAKING = 0.3,
            WAIT_SCORE_BASKET = 0.25,
            WAIT_SCORE_CHAMBER = 0.1,
            WAIT_SCORE_SPEC_PRELOAD = 0.75,
            WAIT_DROP_TO_EXTEND = 0.2,
            WAIT_INTAKE_RETRACT_POST_SUB = 0,
            WAIT_EXTEND_MAX_SPIKE = 0.75,
            WAIT_SWEEPER_EXTEND = 0.3,
            WAIT_SWEEPER_RETRACT = 0,
            LIFT_HEIGHT_TOLERANCE = 3.75,
            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,
            Y_INCHING_FORWARD_WHEN_INTAKING = 5,
            TIME_CYCLE = 5,
            TIME_SCORE = 0.5;

    /// <a href="https://www.desmos.com/calculator/l8pl2gf1mb">Adjust spikes 1 and 2</a>
    /// <a href="https://www.desmos.com/calculator/sishohvpwc">Visualize spike samples</a>
    public static EditablePose
            admissibleError = new EditablePose(1, 1, toRadians(2)),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

    intaking1 = new EditablePose(-54.6,-50, toRadians(84.36)),
            intaking2 = new EditablePose(-56.85,-50.9, toRadians(105)),
            intaking3 = new EditablePose(-53.5,-43.5, 2 * PI / 3),

    sample1 = new EditablePose(-49.24,-28.3, PI / 2),
            sample2 = new EditablePose(-57.85,-27.77, PI / 2),
            sample3 = new EditablePose(-69.2,-26, PI / 2),

    basket = new EditablePose(-57, -56, PI / 4),

    sub1 = new EditablePose(-22.5, -11, 0),
            sub2 = new EditablePose(sub1.x, -6, 0),

    basketFromSub = new EditablePose(-58, -58, 0.765),

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

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        boolean specimenSide = false;
        double partnerWait = 0;
        int cycles = 1;
        boolean specimenPreload = false;

        specimenPreload = specimenSide || specimenPreload;

        pose = new Pose2d(
                specimenSide ? chamberRight.x : specimenPreload ? chamberLeft.x : 0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE,
                0.5 * (specimenPreload ? LENGTH_ROBOT : WIDTH_ROBOT) - SIZE_HALF_FIELD,
                specimenPreload ? PI / 2 : 0
        );

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.3, 20, 15.649408396574804)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(pose)
                .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.3, 20, 15.649408396574804)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(pose)
                .build();

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0));
        TrajectoryActionBuilder builder2 = myBot2.getDrive().actionBuilder(new Pose2d(chamberRight.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, - PI / 2));


//            mTelemetry.addLine("> Right side (observation zone)");

        /// Score preloaded specimen

        /// Score preloaded specimen
        builder2 = builder2
                .waitSeconds(partnerWait)
                .strafeTo(chamberRight.toVector2d())
                .stopAndAdd(scoreSpecimen())
        ;

        if (cycles > 0) {

            /// Push samples
            if (false) {
                builder2 = builder2
//                        .afterTime(0, robot.deposit::triggerClaw)
                        .setTangent(- PI / 2);

                EditablePose[] pushingPoses = {aroundBeamPushing, pushing1, pushed1, pushing2, pushed2, pushing3, pushed3, intakingFirstSpec};
                for (EditablePose pose : pushingPoses) {
                    builder2 = builder2.splineToConstantHeading(pose.toVector2d(), pose.heading);
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
                if (!false || i > 0) builder2 = builder2
//                        .afterTime(0, robot.deposit::triggerClaw)
                        .setTangent(- PI / 2)
                        .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                        ;
                builder2 = builder2
                        .waitSeconds(WAIT_APPROACH_WALL)
//                        .afterTime(0, robot.deposit::triggerClaw)
//                        .stopAndAdd(telemetryPacket -> !robot.deposit.hasSpecimen())
                        .setTangent(PI / 2)
                        .splineToConstantHeading(new Vector2d(chamberRight.x + chamberXs[i] * DISTANCE_BETWEEN_SPECIMENS, chamberRight.y), PI / 2)
                        .stopAndAdd(scoreSpecimen())
                ;
            }

        }

        /// Park in observation zone
        builder2 = builder2.strafeTo(intakingSpec.toVector2d());

//            mTelemetry.addLine("> Park in observation zone");


        pose = specimenPreload ?
                new Pose2d(chamberLeft.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, PI / 2) :
                new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);

        MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SPEED_INCHING),
                new AngularVelConstraint(SPEED_INCHING_TURNING)
        ));

        MinVelConstraint sweepConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SPEED_SWEEPING_SUB),
                new AngularVelConstraint(SPEED_SWEEPING_SUB_TURNING)
        ));

        builder = builder
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample())
//                    .afterTime(0, () -> robot.intake.runRoller(1))
                .strafeToSplineHeading(intaking1.toVector2d(), intaking1.heading)
                .afterTime(0, () -> {
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
//                        timer.reset();
                })
//                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)))
                .setTangent(intaking1.heading)
                .lineToY(intaking1.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                .stopAndAdd(intake())
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample())
//                    .afterTime(0, () -> robot.intake.runRoller(1))
                .strafeToSplineHeading(intaking2.toVector2d(), intaking2.heading)
                .afterTime(0, () -> {
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
//                        timer.reset();
                })
//                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
                .setTangent(intaking2.heading)
                .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                .stopAndAdd(intake())
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample())
//                    .afterTime(0, () -> robot.intake.runRoller(1))
                .strafeToSplineHeading(intaking3.toVector2d(), intaking3.heading)
                .afterTime(0, () -> {
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
//                        timer.reset();
                })
//                    .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)))
                .setTangent(PI / 2)
                .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                .stopAndAdd(intake())
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample())
                .afterTime(0, () -> {
//                        robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR_1);
//                        robot.deposit.liftBeforePointArm = false;
//                        robot.intake.retractBucketBeforeExtendo = true;
//                        robot.deposit.pauseBeforeAutoRetractingLift = true;
                })
                .setTangent(basket.heading)
                .splineTo(sub1.toVector2d(), sub1.heading)
                .stopAndAdd(sweep())
//                    .stopAndAdd(() -> robot.intake.runRoller(1))
                .waitSeconds(WAIT_DROP_TO_EXTEND)
                .setTangent(PI / 2)
                .lineToY(-sub1.y, sweepConstraint)
                .stopAndAdd(intake())
                // .stopAndAdd(new SequentialAction(
                //         new InstantAction(() -> robot.intake.runRoller(SPEED_SLAM_BUCKET)),
                //         new SleepAction(WAIT_SLAM_BUCKET),
                //         new InstantAction(() -> robot.intake.runRoller(1)),
                //         new SleepAction(WAIT_SLAM_BUCKET),
                //         new InstantAction(() -> robot.intake.runRoller(0))
                // ))
                .setTangent(PI)
                .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
//                    .afterDisp(12, () -> robot.sweeper.setActivated(true))
                .splineTo(basketFromSub.toVector2d(), PI + basketFromSub.heading)
//                    .afterTime(0, () -> robot.sweeper.setActivated(false))
                .stopAndAdd(scoreSample())
//                    .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR_2))
                .setTangent(basketFromSub.heading)
                .splineTo(sub2.toVector2d(), sub2.heading)
                .stopAndAdd(sweep())
//                    .stopAndAdd(() -> robot.intake.runRoller(1))
                .waitSeconds(WAIT_DROP_TO_EXTEND)
                .setTangent(PI / 2)
                .lineToY(-sub1.y, sweepConstraint)
                .stopAndAdd(intake())
                // .stopAndAdd(new SequentialAction(
                //         new InstantAction(() -> robot.intake.runRoller(SPEED_SLAM_BUCKET)),
                //         new SleepAction(WAIT_SLAM_BUCKET),
                //         new InstantAction(() -> robot.intake.runRoller(1)),
                //         new SleepAction(WAIT_SLAM_BUCKET),
                //         new InstantAction(() -> robot.intake.runRoller(0))
                // ))
                .setTangent(PI)
                .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
//                    .afterDisp(12, () -> robot.sweeper.setActivated(true))
                .splineTo(basketFromSub.toVector2d(), PI + basketFromSub.heading)
//                    .afterTime(0, () -> robot.sweeper.setActivated(false))
                .stopAndAdd(scoreSample());

        myBot.runAction(builder.build());
        myBot2.runAction(builder2.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }

    private static Action sweep() {
        return new SequentialAction(
//                new InstantAction(() -> robot.sweeper.setActivated(true)),
                new SleepAction(WAIT_SWEEPER_EXTEND),
//                new InstantAction(() -> robot.sweeper.setActivated(false)),
                new SleepAction(WAIT_SWEEPER_RETRACT)
        );
    }

    private static Action intake() {
        return new SequentialAction(
//                new InstantAction(() -> robot.intake.runRoller(1)),
                new SleepAction(WAIT_POST_INTAKING)
//                new InstantAction(() -> robot.intake.runRoller(0))
        );
    }

    private static Action scoreSample() {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_BASKET),
//                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SAMPLE) && robot.deposit.lift.atPosition(HEIGHT_BASKET_HIGH)),
//                new InstantAction(robot.deposit::triggerClaw),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action scoreSpecimen() {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
//                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SPECIMEN) && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
//                new InstantAction(robot.deposit::triggerClaw),
//                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }
}