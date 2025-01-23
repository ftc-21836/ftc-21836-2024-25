package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;


@TeleOp(group = "Single mechanism test")
public final class TestExpansionHubIMU extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
//        HeadingIMU imu = new HeadingIMU(hardwareMap, "imu", new RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();
//            imu.update();
//            mTelemetry.addData("Heading", imu.getHeading());
            mTelemetry.update();
        }
    }
}
