package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.loopMod;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class AutonVars {

    public static double
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.30327,
            WIDTH_ROBOT = 16.42126,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            Y_START = -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5;

    enum ParkingLocation {
        CORNER,
        OUTER,
        TOUCHING_RUNG;

        public static final ParkingLocation[] locations = values();

        public ParkingLocation plus(int i) {
            return locations[(int) loopMod(ordinal() + i, locations.length)];
        }
    }
}
