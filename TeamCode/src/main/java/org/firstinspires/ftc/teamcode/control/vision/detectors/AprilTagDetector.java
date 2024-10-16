package org.firstinspires.ftc.teamcode.control.vision.detectors;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline;
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

    private int[] tagIdsToLookFor;

    private boolean tagVisible = false;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     * @param cameraRotation  physical orientation of camera
     * @param tagIdsToLookFor integer IDs of April Tags to look for
     */
    public AprilTagDetector(HardwareMap hardwareMap, OpenCvCameraRotation cameraRotation, String cameraName, double tagSize, int... tagIdsToLookFor) {
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
        setTagIdsToLookFor(tagIdsToLookFor);
    }

    public void setTagIdsToLookFor(int[] tagIdsToLookFor) {
        this.tagIdsToLookFor = tagIdsToLookFor;
    }

    /**
     * Gets detections from pipeline<p>
     * Use {@link #isTagVisible()} ()} and {@link #getDetectedTag()}
     */
    public void run() {
        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
        if (detections.size() == 0) {
            tagVisible = false;
            return;
        }
        for (AprilTagDetection detection : detections) {
            for (int tagId : tagIdsToLookFor) {
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
        mTelemetry.addLine("A tag of interest is " + (isTagVisible() ? "" : "not ") + "visible");
    }

    /**
     * Prints last {@link #detectedTag} to telemetry <p>
     * telemetry.update() should be called after this method
     */
    public void printDetectedTag() {
        AprilTagDetection detectedTag = getDetectedTag();
        mTelemetry.addLine("A tag has" + (
                detectedTag == null ?
                        "never been detected" :
                        "been detected: " + detectedTag.id
        ));
    }

    /**
     * Closes the camera
     */
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
