package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.loopMod;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Arm.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystem.Arm.SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Arm.SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Arm.TRANSFER;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ANGLE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ANGLE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

        Arm.Position position = TRANSFER;
        boolean closed = false;

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) position = INTAKING;
            if (gamepadEx1.wasJustPressed(DPAD_UP)) position = SAMPLE;
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) position = SPECIMEN;
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) position = TRANSFER;

            arm.setPosition(position);

            if (gamepadEx1.wasJustPressed(B)) closed = !closed;
            claw.turnToAngle(closed ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_OPEN);

            mTelemetry.addData("Position", position.toString());
            mTelemetry.update();
        }
    }
}
