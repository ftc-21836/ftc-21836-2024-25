package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public final class LimelightEx {
    private LLResult result;
    public final Limelight3A limelight;

    public LimelightEx(Limelight3A limelight, HardwareMap hardwareMap) {
        this.limelight = limelight;
    }

    public LLResult update() {
        return result = limelight.getLatestResult();
    }

    public LLResult getResult() {
        return result;
    }

    public List<LLResultTypes.ColorResult> getColorResult() {
        return result.getColorResults();
    }

    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        if (result != null) return result.getDetectorResults();
        return null;
    }

    public Pose2d getPoseEstimate() {
        Pose2d pose = null;
        if (result != null && result.isValid()) {
            Pose3D pose3D = result.getBotpose_MT2();
            if (pose3D != null) {
                pose = new Pose2d(pose3D.getPosition().x, pose3D.getPosition().y, 0);
            }
        }
        return pose;
    }
}