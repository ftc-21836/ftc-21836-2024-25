package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Scanner;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        AutonVars.ParkingLocation parking = AutonVars.ParkingLocation.CORNER;

        boolean isRight = true, cycle = false;

        double partnerWait = 0;

        Scanner scanny = new Scanner(System.in);

        // Configure autonomous
        System.out.print("Red Alliance? (y/n): ");
        OpModeVars.isRedAlliance = scanny.nextLine().equalsIgnoreCase("y");

        System.out.print("Right? (y/n): ");
        isRight = scanny.nextLine().equalsIgnoreCase("y");

        System.out.print("parking type?\n" +
                "   (0) CORNER\n" +
                "   (1) OUTER\n" +
                "   (2) TOUCHING_RUNG\n" +
                " (0-2): ");
        parking = AutonVars.ParkingLocation.locations[scanny.nextInt()];

        System.out.print("Cycle? (y/n): ");
        cycle = scanny.nextLine().equalsIgnoreCase("y");

        System.out.print("Wait time? ");
        partnerWait = scanny.nextDouble();

        scanny.close();


        Pose2d startPose = AutonVars.startPose.toPose2d();
        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .lineToY(30)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}