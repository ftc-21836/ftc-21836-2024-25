package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.EXTEND_SAMPLE_1;
import static com.example.meepmeeptesting.AutonVars.LENGTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.WAIT_APPROACH_BASKET;
import static com.example.meepmeeptesting.AutonVars.WAIT_POST_INTAKING;
import static com.example.meepmeeptesting.AutonVars.WAIT_SCORE_BASKET;
import static com.example.meepmeeptesting.AutonVars.WIDTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.X_START_LEFT;
import static com.example.meepmeeptesting.AutonVars.X_START_RIGHT;
import static com.example.meepmeeptesting.AutonVars.Y_START;
import static com.example.meepmeeptesting.AutonVars.basket;
import static com.example.meepmeeptesting.AutonVars.sample1;
import static com.example.meepmeeptesting.AutonVars.sample1Floor;
import static java.lang.Math.PI;
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

        Pose2d startPose = new Pose2d(isRight ? X_START_RIGHT : X_START_LEFT, Y_START, isRight ? 0.5 * PI : 0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.3, 20.378758517977975, 14.47)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(startPose)
                .build();

        sample1.heading = atan2(sample1Floor.y - sample1.y, sample1Floor.x - sample1.x);

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

        Action waitForIntake = telemetryPacket -> {
            // return robot.getSample() == null;
            return false;
        };

        Action raiseLift = new SequentialAction(
                telemetryPacket -> {
                    // return !robot.deposit.hasSample();
                    return false;
                },
                new InstantAction(() -> {
                    // robot.deposit.setPosition(HIGH);
                })
        );

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(startPose)

                /// Score preloaded
                .afterTime(0, raiseLift)
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample)

                /// Intake first
                .afterTime(0, asyncIntakeSequence(EXTEND_SAMPLE_1))
                .strafeToSplineHeading(sample1.toVector2d(), sample1.heading)
                .afterTime(0, () -> {
                    // if (!robot.intake.hasSample()) robot.intake.runRoller(1);
                })
                .stopAndAdd(waitForIntake) // wait for sample
                .waitSeconds(5) // simulate intaking

                /// Score first
                .afterTime(0, raiseLift)
                .strafeToSplineHeading(basket.toVector2d(), basket.heading)
                .stopAndAdd(scoreSample)

//                // Score first
//                .setReversed(true)
//                .splineTo(new Vector2d(-52, -53), toRadians(225))
//                .waitSeconds(0.1)

//                // Grab second
//                .setReversed(false)
//                .turnTo(toRadians(105))
////                .splineTo(new Vector2d(-54, -45), Math.toRadians(90))
//                .waitSeconds(10)
//
//                // Score second
//                .setReversed(true)
//                .splineTo(new Vector2d(-52, -53), toRadians(225))
////                .waitSeconds(10)
                ;



        myBot.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}