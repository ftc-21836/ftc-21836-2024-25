package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@TeleOp(group = "Single mechanism test")
public final class TuningArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);
        CachedSimpleServo claw = getGBServo(hardwareMap, "claw").reversed();

        boolean closed = false;

        ElapsedTime timer = new ElapsedTime();
        double seconds = 0;

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                arm.setTarget(INTAKING);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
                arm.setTarget(SAMPLE);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) {
                arm.setTarget(SPECIMEN);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) {
                arm.setTarget(TRANSFER);
                timer.reset();
            }

            if (gamepadEx1.wasJustPressed(X)) seconds = timer.seconds();

            arm.run();

            if (gamepadEx1.wasJustPressed(B)) closed = !closed;
            claw.turnToAngle(closed ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_OPEN);

            arm.printTelemetry();
            mTelemetry.addData("Time to reach target (sec)", seconds);
            divider();
            mTelemetry.addData("CLAW", closed ? "CLOSED" : "OPEN");
            mTelemetry.update();
        }
    }
}
