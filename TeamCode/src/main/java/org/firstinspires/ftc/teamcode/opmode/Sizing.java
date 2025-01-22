package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.pose;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@TeleOp
public final class Sizing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, pose);
        Deposit.level1Ascent = false;

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart(); // ----------------------------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.bulkReader.bulkRead();
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(X)) {
                if (robot.deposit.hasSample()) {
                    robot.deposit.triggerClaw();
                    robot.intake.extendo.setExtended(false);
                } else {
                    robot.deposit.transfer(NEUTRAL);
                    robot.intake.extendo.setExtended(true);
                }
            }

            robot.run();

        }
    }
}
