package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.subsystem.Extendo.LENGTH_EXTENDED;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@TeleOp(group = "A - inspection")
public final class Sizing extends LinearOpMode {

    public static double ANGLE_BUCKET = 0.5, LENGTH_EXTEND = LENGTH_EXTENDED;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, pose);
        robot.deposit.steepArm = true;
        robot.intake.setRollerAndAngle(0.01);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart(); // ----------------------------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.bulkReader.bulkRead();
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(X)) {
                if (robot.deposit.hasSample()) {
                    robot.deposit.nextState();
                    robot.intake.extendo.setExtended(false);
                    robot.intake.setAngle(0);
                } else {
                    robot.deposit.goToBasket();
                    robot.intake.extendo.setTarget(LENGTH_EXTEND);
                    robot.intake.setAngle(ANGLE_BUCKET);
                }
            }

            robot.run();

        }
    }
}
