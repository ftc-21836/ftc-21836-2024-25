package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.COLOR_SENSOR_GAIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

@TeleOp(group = "Single mechanism test")
public final class TuningIntakeColors extends LinearOpMode {

    ColorSensor bottomSensor, topSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bottomSensor = new ColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            bottomSensor.update();
            topSensor.update();

            HSV top = topSensor.getHSV(), bottom = bottomSensor.getHSV();

            mTelemetry.addData("Top color", Intake.fromHSV(top).name());
            mTelemetry.addData("Bottom color", Intake.fromHSV(bottom).name());
            mTelemetry.addLine();
            top.toTelemetry("Top HSV");
            mTelemetry.addLine();
            bottom.toTelemetry("Bottom HSV");
            mTelemetry.update();
        }
    }
}
