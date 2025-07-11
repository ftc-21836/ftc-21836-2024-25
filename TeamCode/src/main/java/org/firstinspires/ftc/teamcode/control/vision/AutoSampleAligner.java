package org.firstinspires.ftc.teamcode.control.vision;

import static java.lang.Math.atan2;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public class AutoSampleAligner {
    
    private final LimelightEx limelightEx;

    private boolean sampleDetected;

    public enum Pipeline {
        YELLOW_BLUE,
        YELLOW_RED,
        BLUE,
        RED,
    }

    public static double
            lensXFromRobotCenter = 4.4,
            lensYFromRobotCenter = 4.5,
            lensAngleFromFlatOnFloor = 60,
            lensHeightFromFloor = 16,
            LL_TURN_MULTIPLIER = 1.25;

    private double
            xDistance = 0,
            yDistance = 0,
            xDistanceFromCenter = 0, // x 13, y 33, d 34
            yDistanceFromCenter = 0;

    public double
            measuredXDegreesDiff = 0,
            measuredYDegreesDiff = 0;

    private Pose2d targetOffset = new Pose2d(0, 0, 0);

    public AutoSampleAligner(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;
    }

    public void activateLimelight(Pipeline pipeline) {
        limelightEx.limelight.pipelineSwitch(pipeline.ordinal());

        limelightEx.limelight.setPollRateHz(10);
    }

    public boolean targetSample() {
        limelightEx.update();
        List<LLResultTypes.DetectorResult> targets = limelightEx.getDetectorResult();

        // checks and returns detection
        if (targets == null || targets.isEmpty() || targets.get(0) == null) return false;
        
        measuredXDegreesDiff = targets.get(0).getTargetXDegrees() + 0.0;
        measuredYDegreesDiff = targets.get(0).getTargetYDegrees() + 0.0;
        return true;

    }

    public Action detectTarget(double secondsToExpire) {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    targetOffset = new Pose2d(0, 0, 0);
                    sampleDetected = false;
                    isFirstTime = false;
                    expirationTimer.reset();
                }

                if (!sampleDetected) {
                    sampleDetected = targetSample();
                    if (sampleDetected) limelightEx.limelight.captureSnapshot("detected");
                }

                if (sampleDetected) {

                    yDistance = lensHeightFromFloor * tan(toRadians(lensAngleFromFlatOnFloor + measuredYDegreesDiff));
                    xDistance = tan(toRadians(measuredXDegreesDiff)) * yDistance;

                    yDistanceFromCenter = yDistance + lensYFromRobotCenter;
                    xDistanceFromCenter = xDistance + lensXFromRobotCenter;

                    targetOffset = new Pose2d(xDistanceFromCenter, yDistanceFromCenter, atan2(xDistanceFromCenter, yDistanceFromCenter));
                }

                return !(expirationTimer.seconds() > secondsToExpire || sampleDetected);
            }
        };
    }

    public Pose2d getTargetOffset() {
        return targetOffset;
    }
}
