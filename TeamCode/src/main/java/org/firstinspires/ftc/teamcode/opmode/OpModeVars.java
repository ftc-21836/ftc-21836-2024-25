package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public final class OpModeVars {

    public static MultipleTelemetry mTelemetry;

    public static void divider() {
        mTelemetry.addLine();
        mTelemetry.addLine("--------------------------------------------------------------------------");
        mTelemetry.addLine();
    }

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);

    static boolean isRedAlliance = false;

    public static double loopMod(double x, double max) {
        return (x % max + max) % max;
    }
}
