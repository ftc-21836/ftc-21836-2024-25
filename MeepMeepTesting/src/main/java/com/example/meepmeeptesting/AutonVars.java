package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

public final class AutonVars {

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
            chamber0 = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, toRadians(90)),
            chamberLeft = new EditablePose(-chamber0.x, chamber0.y, chamber0.heading),
            aroundBeamPushing = new EditablePose(35, -30, toRadians(90)),
            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(55, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(62, pushing1.y, toRadians(-90)),
            pushed1 = new EditablePose(pushing1.x, -50, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, pushed1.y, toRadians(-90)),
            intakingSpec = new EditablePose(36, -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5, toRadians(90)),
            intakingFirstSpec = new EditablePose(55, intakingSpec.y, -intakingSpec.heading),
            parkRight = new EditablePose(36, -60, toRadians(90));
}
