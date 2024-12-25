package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.LENGTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.WAIT_APPROACH_BASKET;
import static com.example.meepmeeptesting.AutonVars.WAIT_POST_INTAKING;
import static com.example.meepmeeptesting.AutonVars.WAIT_SCORE_BASKET;
import static com.example.meepmeeptesting.AutonVars.WIDTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.basket;
import static com.example.meepmeeptesting.AutonVars.extendoMMs;
import static com.example.meepmeeptesting.AutonVars.intakingPositions;
import static com.example.meepmeeptesting.AutonVars.parkLeft;
import static com.example.meepmeeptesting.AutonVars.sampleCycles;
import static com.example.meepmeeptesting.AutonVars.samplePositions;
import static com.example.meepmeeptesting.AutonVars.startLeft;
import static com.example.meepmeeptesting.AutonVars.startRight;
import static java.lang.Math.atan2;

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

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isRight = false;

        Pose2d startPose = (isRight ? startRight : startLeft).toPose2d();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.3, 20.378758517977975, 14.47)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(startPose)
                .build();

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(startPose);

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

        /// Score preload
        builder = builder
                .afterTime(0, raiseLift)
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample)
        ;

        /// Cycle samples off the floor
        for (int i = 0; i < sampleCycles; i++) {

            double millimeters = extendoMMs[i];
            EditablePose intakingPos = intakingPositions[i];
            EditablePose samplePos = samplePositions[i];

            // Calculate angle to point intake at floor sample
            intakingPos.heading = atan2(samplePos.y - intakingPos.y, samplePos.x - intakingPos.x);

            builder = builder

                    /// Intake
                    .afterTime(0, asyncIntakeSequence(millimeters))
                    .strafeToSplineHeading(intakingPos.toVector2d(), intakingPos.heading)
                    .afterTime(0, () -> {
                        // if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                    })
                    // wait for intake to get sample:
                    .stopAndAdd(telemetryPacket -> {
                        // return robot.getSample() == null;
                        return false;
                    })
                    .waitSeconds(5) // simulate intaking

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
                })
                .splineTo(parkLeft.toVector2d(), parkLeft.heading)
        ;

        myBot.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}