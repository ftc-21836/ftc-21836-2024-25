package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.ANGLE_BUCKET_INTAKING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.ANGLE_BUCKET_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TuningBucketKd extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SimpleServoPivot bucket = new SimpleServoPivot(
                ANGLE_BUCKET_RETRACTED,
                ANGLE_BUCKET_INTAKING,
                getAxon(hardwareMap, "bucket right").reversed(),
                getAxon(hardwareMap, "bucket left")
        );

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            bucket.updateAngles(
                ANGLE_BUCKET_RETRACTED,
                ANGLE_BUCKET_INTAKING
            );

            if (gamepadEx1.wasJustPressed(X)) bucket.toggle();

            bucket.run();
        }
    }
}
