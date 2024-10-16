package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TestOdoPodEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Motor.Encoder leftEncoder = new MotorEx(hardwareMap, "right front", RPM_312).encoder;
        Motor.Encoder rightEncoder = new MotorEx(hardwareMap, "left front", RPM_312).encoder;
        Motor.Encoder frontEncoder = new MotorEx(hardwareMap, "left back", RPM_312).encoder;
        leftEncoder.setDirection(REVERSE);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();


            mTelemetry.addData("Right", rightEncoder.getPosition());
            mTelemetry.addData("Left", leftEncoder.getPosition());
            mTelemetry.addData("Lateral", frontEncoder.getPosition());
            mTelemetry.update();
        }
    }
}
