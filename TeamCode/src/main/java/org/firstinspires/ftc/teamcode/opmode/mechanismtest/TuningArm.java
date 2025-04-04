package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ANGLE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ANGLE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.ASCENT;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.INTAKING_SPEC;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.IN_INTAKE;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.RAISED_SPEC;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@TeleOp(group = "Single mechanism test")
public final class TuningArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CachedSimpleServo claw = getAxon(hardwareMap, "claw").reversed();

        CachedSimpleServo rServo = getAxon(hardwareMap, "arm right");
        CachedSimpleServo lServo = getAxon(hardwareMap, "arm left").reversed();

        CachedSimpleServo wrist = getAxon(hardwareMap, "wrist");

        boolean closed = false;

        ElapsedTime timer = new ElapsedTime();
        double seconds = 0;

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Deposit.ArmPosition target = STANDBY;

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                target = (INTAKING_SPEC);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
                target = (BASKET);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) {
                target = (CHAMBER);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) {
                target = (IN_INTAKE);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(B)) {
                target = (ASCENT);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(X)) {
                target = (STANDBY);
                timer.reset();
            }
            if (gamepadEx1.wasJustPressed(Y)) {
                target = (RAISED_SPEC);
                timer.reset();
            }

            if (gamepadEx1.wasJustPressed(LEFT_BUMPER)) seconds = timer.seconds();

            rServo.turnToAngle(target.arm);
            lServo.turnToAngle(target.arm);
            wrist.turnToAngle(target.wrist);

            if (gamepadEx1.wasJustPressed(RIGHT_BUMPER)) closed = !closed;
            claw.turnToAngle(closed ? ANGLE_CLAW_CLOSED: ANGLE_CLAW_OPEN);

            mTelemetry.addData("Time to reach target (sec)", seconds);
            divider();
            mTelemetry.addData("CLAW", closed ? "CLOSED" : "OPEN");
            mTelemetry.addLine();
            mTelemetry.addLine("down intaking");
            mTelemetry.addLine("up basket");
            mTelemetry.addLine("left chamber");
            mTelemetry.addLine("right transfer/in intake");
            mTelemetry.addLine("B ascent");
            mTelemetry.addLine("X standby");
            mTelemetry.addLine("Y raised spec");
            mTelemetry.addLine("left bumper print time");
            mTelemetry.addLine("right bumper claw");
            mTelemetry.update();
        }
    }
}
