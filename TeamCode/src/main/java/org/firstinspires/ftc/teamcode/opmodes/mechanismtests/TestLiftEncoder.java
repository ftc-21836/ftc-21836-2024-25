package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Lift.INCHES_PER_TICK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.CachedMotorEx;

@TeleOp(group = "Single mechanism test")
public final class TestLiftEncoder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BulkReader bulkReader = new BulkReader(hardwareMap);

        // Motors and variables to manage their readings:
        Motor.Encoder encoder = new CachedMotorEx(hardwareMap, "right back", RPM_312).encoder;
        double offset = encoder.getPosition();
        
        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();

            double x = INCHES_PER_TICK * (encoder.getPosition() - offset);

            mTelemetry.addData("Current position (in)", x);
            mTelemetry.update();
        }
    }

}
