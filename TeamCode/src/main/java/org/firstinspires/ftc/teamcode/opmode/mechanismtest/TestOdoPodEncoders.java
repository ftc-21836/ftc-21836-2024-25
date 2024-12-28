package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

@TeleOp(group = "Single mechanism test")
public final class TestOdoPodEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            localizer.update();

            mTelemetry.addData("Forward (X)", localizer.rawEncoderX());
            mTelemetry.addData("Perpendicular (Y)", localizer.rawEncoderY());
            mTelemetry.update();
        }
    }

}
