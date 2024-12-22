package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.X_START_LEFT;
import static com.example.meepmeeptesting.AutonVars.X_START_RIGHT;
import static com.example.meepmeeptesting.AutonVars.Y_START;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        boolean isRight = false;

        Pose2d startPose = new Pose2d(isRight ? X_START_RIGHT : X_START_LEFT, Y_START, 0.5 * PI);
        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .lineToY(-30)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}