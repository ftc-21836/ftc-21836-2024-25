package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.atan2;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;

import java.util.List;

@Config
public class AutoAlignToSample {
    private final LimelightEx limelightEx;

    private boolean isSampleDetected;

    public enum Pipeline {
        YELLOW_BLUE,
        YELLOW_RED,
        BLUE,
        RED;

        public final int number;

        Pipeline() {
            this.number = ordinal();
        }
    }

    public Sample targetColor;

    public static double
            xOffset = 5,
            yOffset = 5,
            limelightTilt = 58.5,
            limelightHeight = 16.25;

    private double
            xDistance = 0,
            yDistance = 0,
            xDistanceFromCenter = 0,
            yDistanceFromCenter = 0,
            desiredSampleX = 0,
            desiredSampleY = 0;

    private Pose2d targetedPoseOffset = new Pose2d(0, 0, 0);

    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;
    }

    public void activateLimelight(Pipeline pipeline) {
        limelightEx.limelight.pipelineSwitch(pipeline.number);

        limelightEx.limelight.setPollRateHz(10);
    }

    public boolean targetSample() {
        limelightEx.update();
        List<LLResultTypes.DetectorResult> targets = limelightEx.getDetectorResult();

        // checks and returns detection
        if (targets != null && !targets.isEmpty() && targets.get(0) != null) {
            desiredSampleX = targets.get(0).getTargetXDegrees() + 0.0;
            desiredSampleY = targets.get(0).getTargetYDegrees() + 0.0;
            return true;
        }

        return false;
    }

    public Action detectTarget(double secondsToExpire) {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    targetedPoseOffset = new Pose2d(0, 0, 0);
                    isSampleDetected = false;
                    isFirstTime = false;
                    expirationTimer.reset();
                }

                if (!isSampleDetected) {
                    isSampleDetected = targetSample();
                    if (isSampleDetected) limelightEx.limelight.captureSnapshot("detected");
                }

                if (isSampleDetected) {

                    yDistance = tan(Math.toRadians(limelightTilt + desiredSampleY)) * limelightHeight;
                    xDistance = tan(Math.toRadians(desiredSampleX)) * yDistance;

                    yDistanceFromCenter = yDistance + yOffset;
                    xDistanceFromCenter = xDistance + xOffset;

                    targetedPoseOffset = new Pose2d(xDistanceFromCenter, yDistanceFromCenter, atan2(xDistanceFromCenter, yDistanceFromCenter));
                }

                return expirationTimer.seconds() <= secondsToExpire && !isSampleDetected;
            }
        };
    }

    public Pose2d getTargetedOffset() {
        return targetedPoseOffset;
    }
}
