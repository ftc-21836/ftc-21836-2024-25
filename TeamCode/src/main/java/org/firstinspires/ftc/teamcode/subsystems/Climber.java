package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Climber {

    public static double
            ANGLE_HOOKS_RETRACTED;

    void printTelemetry() {
        mTelemetry.addLine("CLIMBER: ");
    }
}
