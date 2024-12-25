package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.X_START_LEFT;
import static com.example.meepmeeptesting.AutonVars.X_START_RIGHT;
import static com.example.meepmeeptesting.AutonVars.Y_START;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        boolean isRight = false;

        Pose2d startPose = new Pose2d(isRight ? X_START_RIGHT : X_START_LEFT, Y_START, Math.toRadians(0));


        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(startPose)
                .setTangent(Math.toRadians(150))

                // Score preloaded
                .splineToLinearHeading(new Pose2d(-52, -53, Math.toRadians(45)), Math.toRadians(135))
                .waitSeconds(0.1)

                // Grab first
                .splineTo(new Vector2d(-48, -45), Math.toRadians(90))
                .waitSeconds(0.2)

                // Score first
                .setReversed(true)
                .splineTo(new Vector2d(-52, -53), Math.toRadians(225))
                .waitSeconds(0.1)

                // Grab second
                .setReversed(false)
                .turnTo(Math.toRadians(105))
//                .splineTo(new Vector2d(-54, -45), Math.toRadians(90))
                .waitSeconds(10)

                // Score second
                .setReversed(true)
                .splineTo(new Vector2d(-52, -53), Math.toRadians(225))
                .waitSeconds(10)

                // Something else
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(0, -55), Math.toRadians(90))
                .waitSeconds(0.2);



        myBot.runAction(builder.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}