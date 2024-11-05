package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.AMBER;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator;


@TeleOp(group = "Single mechanism test")
public final class TestIndicators extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        LEDIndicator[] indicators = {
                new LEDIndicator(hardwareMap, "led left green", "led left red"),
                new LEDIndicator(hardwareMap, "led right green", "led right red")
        };

        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(B)) {
                for (LEDIndicator indicator : indicators) indicator.setState(RED);
            }
            if (gamepadEx1.wasJustPressed(A)) {
                for (LEDIndicator indicator : indicators) indicator.setState(GREEN);
            }
            if (gamepadEx1.wasJustPressed(X)) {
                for (LEDIndicator indicator : indicators) indicator.setState(OFF);
            }
            if (gamepadEx1.wasJustPressed(Y)) {
                for (LEDIndicator indicator : indicators) indicator.setState(AMBER);
            }
        }
    }
}
