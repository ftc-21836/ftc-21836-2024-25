package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.DISTANCE_BETWEEN_SPECIMENS;
import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_1;
import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_2;
import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_3;
import static com.example.meepmeeptesting.AutonVars.LEFT_SPEC_ID;
import static com.example.meepmeeptesting.AutonVars.LENGTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.ParkingLocation.TOUCHING_RUNG;
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
import static com.example.meepmeeptesting.AutonVars.intaking1;
import static com.example.meepmeeptesting.AutonVars.intaking2;
import static com.example.meepmeeptesting.AutonVars.intaking3;
import static com.example.meepmeeptesting.AutonVars.intakingSpec;
import static com.example.meepmeeptesting.AutonVars.parkLeft;
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
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static Action asyncIntakeSequence(double extension) {
        return new SequentialAction(
            new InstantAction(() -> {
                // robot.intake.extendo.setTarget(extension);
                // robot.intake.runRoller(0.8);
            }),
            telemetryPacket -> {
                // return !robot.intake.hasSample();
                return false;
            },
            new SleepAction(WAIT_POST_INTAKING),
            new InstantAction(() -> {
                // robot.intake.runRoller(0);
            })
        );
    }

    private static Pose2d chamber(int id) {
        return new Pose2d(chamber0.x - id * DISTANCE_BETWEEN_SPECIMENS, chamber0.y, chamber0.heading);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isRight = false, specimenPreload = false;
        double partnerWait = 0;
        int cycles = 3;
        AutonVars.ParkingLocation parking = TOUCHING_RUNG;

        Pose2d startPose = new Pose2d(
                SIZE_TILE * (isRight ? 0.5 : specimenPreload ? -0.5 : -1.5),
                0.5 * (isRight || specimenPreload ? LENGTH_ROBOT : WIDTH_ROBOT) - SIZE_HALF_FIELD,
                isRight || specimenPreload ? 0.5 * PI : 0
        );

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, 5.3, 20.378758517977975, 14.47)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(startPose)
                .build();

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(startPose);

        Action scoreSpec = new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
                telemetryPacket -> {
                    // return !robot.deposit.reachedTarget();
                    return false;
                },
                new InstantAction(() -> {
                    // robot.deposit.triggerClaw();
                }),
                telemetryPacket -> {
                    // return robot.deposit.hasSample();
                    return false;
                },
                new SleepAction(WAIT_SCORE_CHAMBER)
        );

        if (isRight) {
            Action intakeSpec = new SequentialAction(
                    new InstantAction(() -> {
                        // robot.deposit.triggerClaw();
                    }),
                    telemetryPacket -> {
                        // return !robot.deposit.specimenIntaked();
                        return false;
                    },
                    new SleepAction(1) // TODO remove in opmode
            );

            /// Score preloaded specimen
            builder = builder
                    .strafeTo(chamber(0).position)
                    .stopAndAdd(scoreSpec)
            ;

            if (cycles > 0) {
                /// Push samples
                builder = builder
                        .setTangent(toRadians(-90))
                        .splineToConstantHeading(aroundBeamPushing.toVector2d(), toRadians(90))
                        .splineToConstantHeading(pushing1.toVector2d(), toRadians(-90))
                        .splineToConstantHeading(pushed1.toVector2d(), toRadians(-90))
                        .waitSeconds(0.1)
                        .setTangent(toRadians(90))
                        .splineToConstantHeading(pushing2.toVector2d(), pushing2.heading)
                        .splineToConstantHeading(pushed2.toVector2d(), toRadians(-90))
                ;
                if (cycles == 4 && specimenPreload) builder = builder
                        .splineToConstantHeading(pushing3.toVector2d(), toRadians(-90))
                        .splineToConstantHeading(pushed3.toVector2d(), toRadians(-90))
                ;


            } else builder = builder.strafeTo(intakingSpec.toVector2d());

        } else {
            Action scoreSample = new SequentialAction(
                    new SleepAction(WAIT_APPROACH_BASKET),
                    telemetryPacket -> {
                        // return !robot.deposit.reachedTarget();
                        return false;
                    },
                    new InstantAction(() -> {
                        // robot.deposit.triggerClaw();
                    }),
                    new SleepAction(WAIT_SCORE_BASKET)
            );

            Action raiseLift = new SequentialAction(
                    telemetryPacket -> {
                        // return !robot.deposit.hasSample();
                        return false;
                    },
                    new InstantAction(() -> {
                        // robot.deposit.setPosition(HIGH);
                    })
            );

            if (specimenPreload) {
                /// Score preloaded specimen
                builder = builder
                        .waitSeconds(partnerWait)
                        .strafeTo(chamber(LEFT_SPEC_ID).position)
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
                        .afterTime(i == 0 && specimenPreload ? WAIT_EXTEND_SPEC_PRELOAD : 0, asyncIntakeSequence(millimeters))
                        .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                        .afterTime(0, () -> {
                            // if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                        })
                        // wait for intake to get sample:
                        .stopAndAdd(telemetryPacket -> {
                            // return robot.getSample() == null;
                            return false;
                        })
                        .waitSeconds(2) // TODO remove in opmode

                        /// Score
                        .afterTime(0, raiseLift)
                        .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                        .stopAndAdd(scoreSample)
                ;
            }

            /// Level 1 ascent (left park)
            if (parking == TOUCHING_RUNG) {

                builder = builder
                        .afterTime(0, () -> {
                            // robot.deposit.triggerClaw();
                            // robot.deposit.triggerClaw();
                            // robot.deposit.lift.setTarget(LIFT_PARK_LEFT);
                        });

                if (specimenPreload && cycles == 0)
                    builder = builder
                            .setTangent(toRadians(-150))
                            .splineToSplineHeading(aroundBeamParkLeft.toPose2d(), toRadians(90))
                            .splineToConstantHeading(parkLeft.toVector2d(), parkLeft.heading)
                    ;
                else builder = builder.splineTo(parkLeft.toVector2d(), parkLeft.heading);
            }
        }

        myBot.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}