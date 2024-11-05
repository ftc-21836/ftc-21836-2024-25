package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer.X_POD_DIRECTION;
import static org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer.Y_POD_DIRECTION;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TestOdoPodEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);
        localizer.setPosition(new Pose2d(0, 0, 0.5 * PI));

        int xReversed = X_POD_DIRECTION == REVERSED ? -1 : 1;
        int yReversed = Y_POD_DIRECTION == REVERSED ? -1 : 1;

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();

            int x = localizer.rawEncoderX() * xReversed;
            int y = localizer.rawEncoderY() * yReversed;

            mTelemetry.addData("Forward (X)", x);
            mTelemetry.addData("Perpendicular (Y)", y);
            mTelemetry.update();
        }
    }

}
