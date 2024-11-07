package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_312;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Lift.INCHES_PER_TICK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;

@TeleOp(group = "Single mechanism test")
public final class TestLiftEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BulkReader bulkReader = new BulkReader(hardwareMap);

        Motor.Encoder encoder0 = new CachedMotorEx(hardwareMap, "right back", RPM_312).encoder;
        Motor.Encoder encoder1 = new CachedMotorEx(hardwareMap, "left back", RPM_312).encoder;
        
        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();

            double position = INCHES_PER_TICK * 0.5 * (encoder0.getPosition() + encoder1.getPosition());

            mTelemetry.addData("Position (in)", position);
            mTelemetry.update();
        }
    }

}
