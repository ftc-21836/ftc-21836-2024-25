package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.ANGLE_ARM_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.ANGLE_ARM_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TuningArmKd extends LinearOpMode {

    SimpleServoPivot pivot;

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);

        pivot = new SimpleServoPivot(
                ANGLE_ARM_RETRACTED,
                ANGLE_ARM_SAMPLE,
                getAxonServo(hardwareMap, "deposit left"),
                getReversedServo(getAxonServo(hardwareMap, "deposit right"))
        );

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            gamepadEx1.readButtons();

            pivot.updateAngles(ANGLE_ARM_RETRACTED, ANGLE_ARM_SAMPLE);

            if (gamepadEx1.wasJustPressed(X)) pivot.toggle();

            pivot.run();
        }
    }
}
