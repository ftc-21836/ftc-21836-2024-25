package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.ANGLE_ARM_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.ANGLE_ARM_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedSimpleServo.getAxon;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TuningArmKd extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SimpleServoPivot arm = new SimpleServoPivot(
                ANGLE_ARM_RETRACTED,
                ANGLE_ARM_SAMPLE,
                getAxon(hardwareMap, "arm left"),
                getAxon(hardwareMap, "arm right").reversed()
        );

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            arm.updateAngles(ANGLE_ARM_RETRACTED, ANGLE_ARM_SAMPLE);

            if (gamepadEx1.wasJustPressed(X)) arm.toggle();

            arm.run();
        }
    }
}
