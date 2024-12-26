package com.example.meepmeeptesting;

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

    public static int LEFT_SPEC_ID = 10;

    public static double
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.30327,
            WIDTH_ROBOT = 16.42126,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_SCORE_BASKET = 1.5,
            WAIT_SCORE_CHAMBER = 0.5,
            WAIT_POST_INTAKING = 0.5,
            WAIT_EXTEND_SPEC_PRELOAD = 2,
            LIFT_PARK_LEFT = 3,
            EXTEND_SAMPLE_1 = 300,
            EXTEND_SAMPLE_2 = 300,
            EXTEND_SAMPLE_3 = 300,
            DISTANCE_BETWEEN_SPECIMENS = 2;

    public static EditablePose
            sample1 = new EditablePose(-48, -27.75, toRadians(90)),
            sample2 = new EditablePose(-58.5, sample1.y, sample1.heading),
            sample3 = new EditablePose(-68.75, sample1.y, sample1.heading),
            basket = new EditablePose(-56, -56, toRadians(45)),
            intaking1 = new EditablePose(-50, -48, toRadians(84.36)),
            intaking2 = new EditablePose(-54, -45, toRadians(105)),
            intaking3 = new EditablePose(-48, -49, toRadians(120)),
            aroundBeamParkLeft = new EditablePose(-40, -25, toRadians(0)),
            parkLeft = new EditablePose(-25, -11, toRadians(0)),
            chamber0 = new EditablePose(10, -33, toRadians(90)),
            aroundBeamPushing = new EditablePose(35, -28, toRadians(90)),
            pushing1 = new EditablePose(48, -13, toRadians(90)),
            pushing2 = new EditablePose(58.5, -13, toRadians(-45)),
            pushing3 = new EditablePose(62.28937, -13, toRadians(90)),
            pushed1 = new EditablePose(pushing1.x, -55, toRadians(90)),
            pushed2 = new EditablePose(pushing2.x, -55, toRadians(90)),
            pushed3 = new EditablePose(pushing3.x, -55, toRadians(90)),
            intakingSpec = new EditablePose(36, -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5, toRadians(90));
}
