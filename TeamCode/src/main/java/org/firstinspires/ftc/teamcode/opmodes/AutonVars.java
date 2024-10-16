package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ParkingLocation.CORNER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.loopMod;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;

@Config
public final class AutonVars {

    enum ParkingLocation {
        CORNER,
        OUTER,
        TOUCHING_RUNG;

        public static final ParkingLocation[] locations = values();

        public ParkingLocation plus(int i) {
            return locations[loopMod(ordinal() + i, locations.length)];
        }
    }


    public static boolean
            isRed = true,
            isBackdropSide = true,
            cycle = false;

    static ParkingLocation parking = CORNER;

    public static double
            PARTNER_WAIT = 1,
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.3984665354,
            WIDTH_ROBOT = 16.4220472441,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            Y_START = -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, Y_START, FORWARD);
}
