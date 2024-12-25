package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

public final class AutonVars {

    enum ParkingLocation {
        CORNER,
        OUTER,
        TOUCHING_RUNG;

        public static final ParkingLocation[] locations = values();

        public ParkingLocation plus(int i) {
            int x = ordinal() + 1, max = locations.length;
            return locations[(x % max + max) % max];
        }
    }

    public static double
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.30327,
            WIDTH_ROBOT = 16.42126,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            WAIT_APPROACH_BASKET = 0,
            WAIT_SCORE_BASKET = 1.5,
            WAIT_POST_INTAKING = 0.5,
            WAIT_MAX_INTAKING = 3,
            EXTEND_SAMPLE_1 = 300,
            EXTEND_SAMPLE_2 = 300,
            EXTEND_SAMPLE_3 = 300;

    public static EditablePose
            startRight = new EditablePose(SIZE_TILE * 0.5, -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5, 0.5 * PI),
            startLeft = new EditablePose(SIZE_TILE * -1.5, -SIZE_HALF_FIELD + WIDTH_ROBOT * 0.5, 0),
            sample1Floor = new EditablePose(-48, -27.75, toRadians(90)),
            basket = new EditablePose(-56, -56, toRadians(45)),
            sample1 = new EditablePose(-50, -48, toRadians(84.36)),
            sample2 = new EditablePose(-54, -45, toRadians(105)),
            sample3 = new EditablePose(-48, -49, toRadians(120)),
            parkLeft = new EditablePose(-30, -10, toRadians(0));
}
