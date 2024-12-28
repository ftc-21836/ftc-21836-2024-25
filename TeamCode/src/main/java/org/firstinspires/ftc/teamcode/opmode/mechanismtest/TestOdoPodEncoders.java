package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TestOdoPodEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);

        waitForStart();

        localizer.setPosition(new Pose2d(0, 0, 0.5 * PI));

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            localizer.update();

            mTelemetry.addData("Forward (X)", localizer.rawEncoderX());
            mTelemetry.addData("Perpendicular (Y)", localizer.rawEncoderY());
            mTelemetry.update();
        }
    }

}
