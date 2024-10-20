package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;


@TeleOp(group = "Single mechanism test")
public final class TestTank extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        CachedMotorEx[] motors = {
                new CachedMotorEx(hardwareMap, "m0", 560, 300),
                new CachedMotorEx(hardwareMap, "m1", 560, 300),
                new CachedMotorEx(hardwareMap, "m2", 560, 300),
                new CachedMotorEx(hardwareMap, "m3", 560, 300),
        };

        motors[1].setInverted(true);
        motors[2].setInverted(true);
        motors[3].setInverted(true);

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();

            gamepadEx1.readButtons();

            double scalar = 12 / batteryVoltageSensor.getVoltage();

            double y = gamepadEx1.getLeftY();
            double turn = gamepadEx1.getRightX();

            double left = y + turn;
            double right = y - turn;

            double max = max(1.0, max(left, right));

            left /= max;
            right /= max;

            left *= scalar;
            right *= scalar;

            motors[0].set(left);
            motors[1].set(left);
            motors[2].set(right);
            motors[3].set(right);
        }
    }
}
