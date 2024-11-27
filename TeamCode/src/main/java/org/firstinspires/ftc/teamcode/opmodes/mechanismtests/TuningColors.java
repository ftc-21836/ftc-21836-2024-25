package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sample;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

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

//            armColor.update();
//
//            HSV armHSV = armColor.getHSV();
//            Sample armSample = Deposit.hsvToSample(armHSV);
//
//            mTelemetry.addLine("ARM: ");
//            mTelemetry.addLine();
//            mTelemetry.addLine(armSample != null ? armSample + " sample" : "Empty");
//            armHSV.toTelemetry();
//
//            divider();
//
//            bucketColor.update();
//
//            HSV bucketHSV = bucketColor.getHSV();
//            Sample bucketSample = Intake.hsvToSample(bucketHSV);
//
//            mTelemetry.addLine("BUCKET: ");
//            mTelemetry.addLine();
//            mTelemetry.addLine(bucketSample != null ? bucketSample + " sample" : "Empty");
//            bucketHSV.toTelemetry();
//            mTelemetry.update();
        }
    }
}
