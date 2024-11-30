package org.firstinspires.ftc.teamcode.control.vision.detector;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class AprilTagDetector {

    public static double
            CAMERA_FX = 578.272,
            CAMERA_FY = 578.272,
            CAMERA_CX = 402.145,
            CAMERA_CY = 221.506;

    private AprilTagDetection detectedTag = null;

    private final OpenCvCamera camera;

    private final AprilTagDetectionPipeline pipeline;

    private int[] targetIDs;

    private boolean tagVisible = false;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     * @param cameraRotation  physical orientation of camera
     * @param targetIDs integer IDs of April Tags to look for
     */
    public AprilTagDetector(HardwareMap hardwareMap, OpenCvCameraRotation cameraRotation, String cameraName, double tagSize, int... targetIDs) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, cameraName),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        pipeline = new AprilTagDetectionPipeline(
                tagSize,
                CAMERA_FX,
                CAMERA_FY,
                CAMERA_CX,
                CAMERA_CY
        );
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, cameraRotation);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        setTargetIDs(targetIDs);
    }

    public void setTargetIDs(int[] targetIDs) {
        this.targetIDs = targetIDs;
    }

    /**
     * Gets detections from pipeline<p>
     * Use {@link #isTagVisible()} ()} and {@link #getDetectedTag()}
     */
    public void run() {
        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
        if (detections.isEmpty()) {
            tagVisible = false;
            return;
        }
        for (AprilTagDetection detection : detections) {
            for (int tagId : targetIDs) {
                if (detection.id == tagId) {
                    detectedTag = detection;
                    tagVisible = true;
                    return;
                }
            }
        }
        tagVisible = false;
    }

    public boolean isTagVisible() {
        return tagVisible;
    }

    public AprilTagDetection getDetectedTag() {
        return detectedTag;
    }

    /**
     * Prints tag visibility to telemetry <p>
     * telemetry.update() should be called after this method
     */
    public void printTagIsVisible() {
        mTelemetry.addData("A tag of interest is", (isTagVisible() ? "" : "not ") + "visible");
    }

    /**
     * Prints last {@link #detectedTag} to telemetry <p>
     * telemetry.update() should be called after this method
     */
    public void printDetectedTag() {
        AprilTagDetection tag = getDetectedTag();
        mTelemetry.addData("A tag has", tag == null ? "never been detected" : "been detected: " + tag.id);
    }

    /**
     * Closes the camera
     */
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
