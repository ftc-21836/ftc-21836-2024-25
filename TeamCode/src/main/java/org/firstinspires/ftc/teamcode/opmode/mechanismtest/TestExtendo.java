package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Extendo;

@TeleOp(group = "Single mechanism test")
public final class TestExtendo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Extendo extendo = new Extendo(hardwareMap);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) extendo.setExtended(true);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) extendo.setExtended(false);
            if (gamepadEx1.wasJustPressed(X)) extendo.toggle();

            extendo.run(true);

            extendo.printTelemetry();
            mTelemetry.update();

        }

    }

}
