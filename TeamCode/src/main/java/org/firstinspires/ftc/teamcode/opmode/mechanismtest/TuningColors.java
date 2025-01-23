package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@TeleOp(group = "Single mechanism test")
public final class TuningColors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorSensor bucketColor = new ColorSensor(hardwareMap, "bucket color", (float) Intake.COLOR_SENSOR_GAIN);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            bucketColor.update();

            HSV bucketHSV = bucketColor.getHSV();
            Sample bucketSample = Intake.hsvToSample(bucketHSV);

            mTelemetry.addLine("BUCKET: ");
            mTelemetry.addLine();
            mTelemetry.addLine(bucketSample != null ? bucketSample + " sample" : "Empty");
            bucketHSV.toTelemetry();
            mTelemetry.update();
        }
    }
}
