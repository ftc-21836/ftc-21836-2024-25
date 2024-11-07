package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ANGLE_BUCKET_INTAKING;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ANGLE_BUCKET_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TuningIntakeKd extends LinearOpMode {

    SimpleServoPivot pivot;

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);

        pivot = new SimpleServoPivot(
                ANGLE_BUCKET_RETRACTED,
                ANGLE_BUCKET_INTAKING,
                getAxonServo(hardwareMap, "bucket right"),
                getAxonServo(hardwareMap, "bucket left").reversed()
        );

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            gamepadEx1.readButtons();

            pivot.updateAngles(
                ANGLE_BUCKET_RETRACTED,
                ANGLE_BUCKET_INTAKING
            );

            if (gamepadEx1.wasJustPressed(X)) pivot.toggle();

            pivot.run();
        }
    }
}
