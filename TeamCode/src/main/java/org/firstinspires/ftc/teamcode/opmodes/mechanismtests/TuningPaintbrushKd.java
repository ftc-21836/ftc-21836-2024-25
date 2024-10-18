package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.subsystems.Deposit.Arm.ANGLE_PIVOT_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TuningPaintbrushKd extends LinearOpMode {

    SimpleServoPivot pivot;

    @Override
    public void runOpMode() throws InterruptedException {

        BulkReader bulkReader = new BulkReader(hardwareMap);

        pivot = new SimpleServoPivot(
                ANGLE_PIVOT_RETRACTED,
                ANGLE_PIVOT_RETRACTED + 120,
                getAxonServo(hardwareMap, "deposit left"),
                getReversedServo(getAxonServo(hardwareMap, "deposit right"))
        );

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            gamepadEx1.readButtons();

            pivot.updateAngles(ANGLE_PIVOT_RETRACTED, ANGLE_PIVOT_RETRACTED + 120);

            if (keyPressed(1, X)) pivot.toggle();

            pivot.run();
        }
    }
}
