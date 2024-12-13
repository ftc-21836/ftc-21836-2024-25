package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.loopMod;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ANGLE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ANGLE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

import java.util.logging.Handler;

@TeleOp(group = "Single mechanism test")
public final class TuningArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);
        CachedSimpleServo claw = getGBServo(hardwareMap, "claw").reversed();

        Arm.Position[] positions = {
                Arm.INTAKING,
                Arm.TRANSFER,
                Arm.SPECIMEN,
                Arm.SAMPLE,
        };

        int index = 0;
        boolean closed = false;

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) index = (int) loopMod(index - 1, positions.length);
            if (gamepadEx1.wasJustPressed(DPAD_UP)) index = (int) loopMod(index + 1, positions.length);

            arm.setPosition(positions[index]);

            if (gamepadEx1.wasJustPressed(B)) closed = !closed;
            claw.turnToAngle(closed ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_OPEN);
        }
    }
}
