package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.PrimitiveIterator;


public final class AutonVars {

    public static double
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.3984665354,
            WIDTH_ROBOT = 16.4220472441,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            Y_START = -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5;

    public enum ParkingLocation {
        CORNER,
        OUTER,
        TOUCHING_RUNG;

        public static final ParkingLocation[] locations = values();

        public ParkingLocation plus(int i) {
            int x = ordinal() + 1, max = locations.length;
            return locations[(x % max + max) % max];
        }
    }

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, Y_START, 0.5 * PI)
            ;
}
