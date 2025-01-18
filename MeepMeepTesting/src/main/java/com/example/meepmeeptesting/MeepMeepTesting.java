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
            LENGTH_ROBOT = 17.30327,
            WIDTH_ROBOT = 16.42126,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,
            EXTEND_SAMPLE_1 = 300,
            EXTEND_SAMPLE_2 = 300,
            EXTEND_SAMPLE_3 = 410,
            EXTEND_SUB_START = 200,
            EXTEND_SUB = 350,
            SPEED_SWEEPING_SUB = 20,
            SPEED_SWEEPING_SUB_TURNING = PI / 2,
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
            intakingSub = new EditablePose(-22, -11, 0),
            sweptSub = new EditablePose(-28, 0, PI / 4),
            aroundBeamParkLeft = new EditablePose(-40, -25, 0),
            parkLeft = new EditablePose(-22, -11, 0),
            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, PI / 2),
            chamberLeft = new EditablePose(-chamberRight.x, chamberRight.y, chamberRight.heading),
            aroundBeamPushing = new EditablePose(35, -30, PI / 2),
            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(63, pushing1.y, - PI / 2),
            pushed1 = new EditablePose(pushing1.x, -46, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, pushed1.y, - PI / 2),
            intakingSpec = new EditablePose(36, -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5, PI / 2),
            intakingFirstSpec = new EditablePose(55, intakingSpec.y, -intakingSpec.heading);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        boolean specimenSide = false;
        double partnerWait = 0;
        int cycles = 3;
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

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(pose);

        if (specimenSide) {

//            mTelemetry.addLine("> Right side (observation zone)");

            /// Score preloaded specimen
            builder = builder
                    .waitSeconds(partnerWait)
                    .strafeTo(chamberRight.toVector2d())
                    .stopAndAdd(scoreSpecimen())
            ;

//            mTelemetry.addLine("> Score preloaded specimen");

            if (cycles > 0) {

                /// Push samples
                builder = builder
//                        .afterTime(0, robot.deposit::triggerClaw)
                        .setTangent(- PI / 2);

                EditablePose[] pushingPoses = {aroundBeamPushing, pushing1, pushed1, pushing2, pushed2, pushing3, pushed3, intakingFirstSpec};
                for (EditablePose pose : pushingPoses) {
                    builder = builder.splineToConstantHeading(pose.toVector2d(), pose.heading);
                }
//                mTelemetry.addLine("> Push samples");

                double[] chamberXs = {
                        X_OFFSET_CHAMBER_1,
                        X_OFFSET_CHAMBER_2,
                        X_OFFSET_CHAMBER_3,
                        X_OFFSET_CHAMBER_4,
                };

                /// Cycle specimens
                for (int i = 0; i < min(chamberXs.length, cycles); i++) {
                    if (i > 0) builder = builder
//                            .afterTime(0, robot.deposit::triggerClaw)
                            .setTangent(- PI / 2)
                            .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                            ;
                    builder = builder
                            .waitSeconds(WAIT_APPROACH_WALL)
//                            .afterTime(0, robot.deposit::triggerClaw)
//                            .stopAndAdd(telemetryPacket -> !robot.deposit.hasSpecimen())
                            .setTangent(PI / 2)
                            .splineToConstantHeading(new Vector2d(chamberRight.x + chamberXs[i] * DISTANCE_BETWEEN_SPECIMENS, chamberRight.y), chamberRight.heading)
                            .stopAndAdd(scoreSpecimen())
                    ;

//                    mTelemetry.addLine("> Specimen cycle " + (i + 1));
                }


            }

            /// Park in observation zone
            builder = builder.strafeTo(intakingSpec.toVector2d());

//            mTelemetry.addLine("> Park in observation zone");

        } else {

//            mTelemetry.addLine("> Left side (near basket)");

            if (specimenPreload) {
                /// Score preloaded specimen
                builder = builder
                        .waitSeconds(partnerWait)
                        .strafeTo(chamberLeft.toVector2d())
                        .stopAndAdd(scoreSpecimen())
                ;

//                mTelemetry.addLine("> Score preloaded specimen");
            } else {
                /// Score preloaded sample
                builder = builder
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample());

//                mTelemetry.addLine("> Score preloaded sample");
            }

            double[] extendoMMs = {EXTEND_SAMPLE_1, EXTEND_SAMPLE_2, EXTEND_SAMPLE_3};
            EditablePose[] intakingPositions = {intaking1, intaking2, intaking3};
            EditablePose[] samplePositions = {sample1, sample2, sample3};

            if (specimenPreload) {
                intakingPositions[0] = intaking1SpecPreload;
                samplePositions[0] = sample1SpecPreload;
            }

            /// Cycle samples off the floor
            for (int i = 0; i < min(3, cycles); i++) {

                boolean firstAfterSpec = i == 0 && specimenPreload;

                double millimeters = extendoMMs[i];
                EditablePose intakingPos = intakingPositions[i];
                EditablePose samplePos = samplePositions[i];

                // Calculate angle to point intake at floor sample
                intakingPos.heading = atan2(samplePos.y - intakingPos.y, samplePos.x - intakingPos.x);

                /// Drop bucket now (if not specimen preload)
                if (!firstAfterSpec) builder = builder
//                        .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                ;

                /// Drive to intaking position
                builder = builder
                        .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                        .afterTime(0, new SequentialAction(
//                                telemetryPacket -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(millimeters)),
                                new InstantAction(() -> {
//                                    if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                                })
                        ));

                /// Drop bucket after reaching pos (if specimen preload)
                if (firstAfterSpec) builder = builder
//                        .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                        .waitSeconds(WAIT_DROP_TO_EXTEND);

                /// Intaking sample
                builder = builder
//                        .afterTime(0, () -> robot.intake.extendo.setTarget(millimeters))
//                        .stopAndAdd(telemetryPacket -> !robot.intake.hasSample()) // wait until intake gets a sample
                        .waitSeconds(WAIT_POST_INTAKING)
//                        .afterTime(0, () -> robot.intake.runRoller(0))
                        /// Score
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample());

//                mTelemetry.addLine("> Sample cycle " + (i + 1));
            }

            for (int i = 0; i < max(0, cycles - 3); i++) {

                builder = builder
                        .splineTo(intakingSub.toVector2d(), intakingSub.heading)
//                        .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_SUB_START))
//                        .stopAndAdd(telemetryPacket -> !robot.intake.extendo.atPosition(EXTEND_SUB))
//                        .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                        .waitSeconds(WAIT_DROP_TO_EXTEND)
//                        .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_SUB))
                        .strafeToSplineHeading(sweptSub.toVector2d(), sweptSub.heading, new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(SPEED_SWEEPING_SUB),
                                new AngularVelConstraint(SPEED_SWEEPING_SUB_TURNING)
                        )))
                        .afterTime(0, () -> {
//                            robot.intake.runRoller(0);
//                            if (!robot.intake.hasSample()) robot.intake.extendo.setExtended(false);
                        })
                        .setTangent(PI + sweptSub.heading)
                        .splineTo(basket.toVector2d(), PI + basket.heading)
                        .waitSeconds(WAIT_APPROACH_BASKET)
//                        .stopAndAdd(telemetryPacket -> robot.getSample() != null && !(robot.deposit.arm.atPosition(Arm.SAMPLE) && robot.deposit.lift.atPosition(HEIGHT_BASKET_HIGH)))
                        .afterTime(0, () -> {
//                            if (robot.getSample() != null) robot.deposit.triggerClaw();
                        })
                        .waitSeconds(WAIT_SCORE_BASKET)
                ;

//                mTelemetry.addLine("> Submersible sample " + (i + 1));

            }

            /// Raise arm for level 1 ascent
            builder = builder.afterTime(0, () -> {
//                Deposit.level1Ascent = true;
//                robot.deposit.lift.setTarget(0);
            });

//            mTelemetry.addLine("> Raise arm for level 1 ascent");

            /// Drive to level 1 ascent location
            if (specimenPreload && cycles == 0)
                builder = builder
                        .setTangent(- 5 * PI / 6)
                        .splineToSplineHeading(aroundBeamParkLeft.toPose2d(), PI / 2)
                        .splineToConstantHeading(parkLeft.toVector2d(), parkLeft.heading)
                        ;
            else builder = builder.splineTo(parkLeft.toVector2d(), parkLeft.heading);

//            mTelemetry.addLine("> Drive to level 1 ascent location");
        }

        myBot.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
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