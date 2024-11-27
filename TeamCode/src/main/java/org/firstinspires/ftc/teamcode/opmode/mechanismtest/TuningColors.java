package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@TeleOp(group = "Single mechanism test")
public final class TuningColors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorSensor armColor = new ColorSensor(hardwareMap, "arm color", (float) Deposit.COLOR_SENSOR_GAIN);
        ColorSensor bucketColor = new ColorSensor(hardwareMap, "bucket color", (float) Intake.COLOR_SENSOR_GAIN);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            armColor.update();

            HSV armHSV = armColor.getHSV();
            Sample armSample = Deposit.hsvToSample(armHSV);

            mTelemetry.addLine("ARM: ");
            mTelemetry.addLine();
            mTelemetry.addLine(armSample != null ? armSample + " sample" : "Empty");
            armHSV.toTelemetry();

            divider();

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
