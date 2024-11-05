package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public final class SharedVars {

    public static final double
            LEFT = PI,
            FORWARD = 0.5 * PI,
            RIGHT = 0,
            BACKWARD = -FORWARD;

    public static MultipleTelemetry mTelemetry;

    public static void divider() {
        mTelemetry.addLine();
        mTelemetry.addLine("------------------------------------------------------------------------------------");
        mTelemetry.addLine();
    }

    public static Pose2d pose = new Pose2d(0,0,FORWARD);

    static boolean isRedAlliance = true;

    public static int loopMod(int a, int b) {
        return (int) loopMod(a,(double) b);
    }

    public static double loopMod(double a, double b) {
        return (a % b + b) % b;
    }
}
