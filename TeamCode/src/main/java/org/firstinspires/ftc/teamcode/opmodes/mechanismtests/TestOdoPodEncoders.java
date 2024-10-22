package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TestOdoPodEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(
                hardwareMap
        );

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();


            mTelemetry.addData("Left", localizer.par0.getPositionAndVelocity().position);
            mTelemetry.addData("Right", localizer.par1.getPositionAndVelocity().position);
            mTelemetry.addData("Lateral", localizer.perp.getPositionAndVelocity().position);
            mTelemetry.update();
        }
    }
}
