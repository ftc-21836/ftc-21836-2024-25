package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ANGLE_EXTENDO_EXTENDED_MAX;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ANGLE_EXTENDO_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedSimpleServo.getGBServo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TestExtendo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SimpleServoPivot extendo = new SimpleServoPivot(
                ANGLE_EXTENDO_RETRACTED,
                ANGLE_EXTENDO_EXTENDED_MAX,
                getGBServo(hardwareMap, "extendo right").reversed(),
                getGBServo(hardwareMap, "extendo left").reversed()
        );

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(X)) extendo.toggle();

            extendo.updateAngles(ANGLE_EXTENDO_RETRACTED, ANGLE_EXTENDO_EXTENDED_MAX);
            extendo.run();

        }

    }

}
