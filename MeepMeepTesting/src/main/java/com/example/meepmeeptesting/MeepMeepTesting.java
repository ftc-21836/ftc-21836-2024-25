package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.DISTANCE_BETWEEN_SPECIMENS;
import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_1;
import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_2;
import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_3;
import static com.example.meepmeeptesting.AutonVars.LENGTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.SIZE_HALF_FIELD;
import static com.example.meepmeeptesting.AutonVars.SIZE_TILE;
import static com.example.meepmeeptesting.AutonVars.WAIT_APPROACH_BASKET;
import static com.example.meepmeeptesting.AutonVars.WAIT_APPROACH_CHAMBER;
import static com.example.meepmeeptesting.AutonVars.WAIT_EXTEND_SPEC_PRELOAD;
import static com.example.meepmeeptesting.AutonVars.WAIT_POST_INTAKING;
import static com.example.meepmeeptesting.AutonVars.WAIT_SCORE_BASKET;
import static com.example.meepmeeptesting.AutonVars.WAIT_SCORE_CHAMBER;
import static com.example.meepmeeptesting.AutonVars.WIDTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.aroundBeamParkLeft;
import static com.example.meepmeeptesting.AutonVars.aroundBeamPushing;
import static com.example.meepmeeptesting.AutonVars.basket;
import static com.example.meepmeeptesting.AutonVars.chamber0;
import static com.example.meepmeeptesting.AutonVars.chamberLeft;
import static com.example.meepmeeptesting.AutonVars.intaking1;
import static com.example.meepmeeptesting.AutonVars.intaking2;
import static com.example.meepmeeptesting.AutonVars.intaking3;
import static com.example.meepmeeptesting.AutonVars.intakingFirstSpec;
import static com.example.meepmeeptesting.AutonVars.intakingSpec;
import static com.example.meepmeeptesting.AutonVars.parkLeft;
import static com.example.meepmeeptesting.AutonVars.parkRight;
import static com.example.meepmeeptesting.AutonVars.pushed1;
import static com.example.meepmeeptesting.AutonVars.pushed2;
import static com.example.meepmeeptesting.AutonVars.pushed3;
import static com.example.meepmeeptesting.AutonVars.pushing1;
import static com.example.meepmeeptesting.AutonVars.pushing2;
import static com.example.meepmeeptesting.AutonVars.pushing3;
import static com.example.meepmeeptesting.AutonVars.sample1;
import static com.example.meepmeeptesting.AutonVars.sample2;
import static com.example.meepmeeptesting.AutonVars.sample3;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        boolean specimenSide = false, specimenPreload = false;
        double partnerWait = 0;
        int cycles = 3;

        Pose2d startPose = new Pose2d(
                specimenSide ? chamber0.x : specimenPreload ? chamberLeft.x : 0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE,
                0.5 * (specimenSide || specimenPreload ? LENGTH_ROBOT : WIDTH_ROBOT) - SIZE_HALF_FIELD,
                specimenSide || specimenPreload ? PI / 2 : 0
        );

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.3, 20.378758517977975, 14.47)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(startPose)
                .build();

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(startPose);

        Action scoreSpecimen = new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
//                telemetryPacket -> !robot.deposit.reachedTarget(), // wait until deposit in position
//                new InstantAction(robot.deposit::triggerClaw),
//                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );

        if (specimenSide) {

            /// Score preloaded specimen
            builder = builder
                    .strafeTo(chamber0.toVector2d())
                    .stopAndAdd(scoreSpecimen)
            ;

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

                Action intakeSpec = new SequentialAction(
//                    new InstantAction(robot.deposit::triggerClaw),
//                    telemetryPacket -> !robot.deposit.specimenIntaked(),
//                    new InstantAction(() -> robot.deposit.setPosition(HIGH)),
                        new SleepAction(1) // TODO remove in opmode
                );

                /// Cycle specimens
                for (int i = 0; i < cycles; i++) {
                    builder = builder
                            .stopAndAdd(intakeSpec)
                            .setTangent(PI / 2)
                            .splineToConstantHeading(new Vector2d(chamber0.x - (i + 1) * DISTANCE_BETWEEN_SPECIMENS, chamber0.y), chamber0.heading)
                            .stopAndAdd(scoreSpecimen)
//                            .afterTime(0, robot.deposit::triggerClaw)
                            .setTangent(- PI / 2)
                    ;
                    if (i < cycles - 1) builder = builder
                            .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                            ;

//                    mTelemetry.addLine(genLog += "\n> Specimen cycle " + (i + 1));
//                    mTelemetry.update();
                }


            }

            /// Park in observation zone
            builder = builder.strafeTo(parkRight.toVector2d());

        } else {
            Action scoreSample = new SequentialAction(
                    new SleepAction(WAIT_APPROACH_BASKET),
//                    telemetryPacket -> !robot.deposit.reachedTarget(),
//                    new InstantAction(robot.deposit::triggerClaw),
                    new SleepAction(WAIT_SCORE_BASKET)
            );

            Action raiseLift = new SequentialAction(
//                    telemetryPacket -> !robot.deposit.hasSample(),
//                    new InstantAction(() -> robot.deposit.setPosition(HIGH))
            );

            if (specimenPreload) {
                /// Score preloaded specimen
                builder = builder
                        .waitSeconds(partnerWait)
                        .strafeTo(chamberLeft.toVector2d())
                        .stopAndAdd(scoreSpecimen)
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
                        .afterTime(i == 0 && specimenPreload ? WAIT_EXTEND_SPEC_PRELOAD : 0, new SequentialAction(
                                new InstantAction(() -> {
//                                    robot.intake.extendo.setTarget(millimeters);
//                                    robot.intake.runRoller(0.8);
                                }),
//                                telemetryPacket -> !robot.intake.hasSample(), // wait until intake gets a sample
                                new SleepAction(WAIT_POST_INTAKING)
//                                new InstantAction(() -> robot.intake.runRoller(0))
                        ))
                        .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                        .afterTime(0, () -> {
//                             if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                        })
                        // wait for intake to get sample:
//                        .stopAndAdd(telemetryPacket -> robot.getSample() == null)
                        .waitSeconds(1)

                        /// Score
                        .afterTime(0, raiseLift)
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample)
                ;
            }

            /// Level 1 ascent (left park)
            builder = builder
                    .afterTime(0, () -> {
                        // robot.deposit.triggerClaw();
                        // robot.deposit.triggerClaw();
                        // robot.deposit.lift.setTarget(LIFT_PARK_LEFT);
                    });

            if (specimenPreload && cycles == 0)
                builder = builder
                        .setTangent(- 5 * PI / 6)
                        .splineToSplineHeading(aroundBeamParkLeft.toPose2d(), PI / 2)
                        .splineToConstantHeading(parkLeft.toVector2d(), parkLeft.heading)
                ;
            else builder = builder.splineTo(parkLeft.toVector2d(), parkLeft.heading);
        }

        myBot.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}