package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;

public final class OpModeVars {

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);

    static boolean isRedAlliance = true;

    public static double loopMod(double x, double max) {
        return (x % max + max) % max;
    }
}
