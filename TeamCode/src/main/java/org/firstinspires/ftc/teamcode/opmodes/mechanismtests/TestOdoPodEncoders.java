package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.roadrunner.util.Encoder.Direction.REVERSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TestOdoPodEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right front"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left front"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left back"));
        leftEncoder.setDirection(REVERSE);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();


            mTelemetry.addData("Right", rightEncoder.getCurrentPosition());
            mTelemetry.addData("Left", leftEncoder.getCurrentPosition());
            mTelemetry.addData("Lateral", frontEncoder.getCurrentPosition());
            mTelemetry.update();
        }
    }
}
