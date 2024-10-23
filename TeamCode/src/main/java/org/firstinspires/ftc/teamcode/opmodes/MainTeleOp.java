package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.loopMod;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.mTelemetry;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_SLOW_LOCK,
        EDITING_FIELD_CENTRIC;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            return selections[loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, autonEndPose);

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        TeleOpConfig selection = EDITING_ALLIANCE;

        boolean slowModeLocked = false, useFieldCentric = true;
        // Get gamepad 1 button input and locks slow mode:
        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (keyPressed(1, DPAD_UP))   selection = selection.plus(-1);
            if (keyPressed(1, DPAD_DOWN)) selection = selection.plus(1);

            if (keyPressed(1, X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRed = !isRed;
                    break;
                case EDITING_SLOW_LOCK:
                    slowModeLocked = !slowModeLocked;
                    break;
                case EDITING_FIELD_CENTRIC:
                default:
                    useFieldCentric = !useFieldCentric;
                    break;
            }

            mTelemetry.addLine((isRed ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Slow mode " + (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(EDITING_SLOW_LOCK));
            mTelemetry.addLine();
            mTelemetry.addLine((useFieldCentric ? "Field" : "ROBOT") + " centric driving" + selection.markIf(EDITING_FIELD_CENTRIC));

            mTelemetry.update();
        }

//        robot.intake.updateAlliance(isRed);

//        if (autonEndPose == null) autonEndPose = OpModeVars.startPose.byBoth().toPose2d();
//        robot.drivetrain.setPoseEstimate(autonEndPose);
//        robot.drivetrain.setCurrentHeading(autonEndPose.getHeading() - (isRed ? FORWARD : BACKWARD));

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            double rightX = gamepadEx1.getRightX();
            double leftX = gamepadEx1.getLeftX();
            double leftY = gamepadEx1.getLeftY();
            boolean overrideMode = gamepadEx1.isDown(LEFT_BUMPER);

            if (overrideMode) {

//                robot.intake.offsetExtension(
//                        gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
//                );
//
//                robot.deposit.lift.setLiftPower(gamepadEx1.getLeftY());
//                if (keyPressed(1, LEFT_STICK_BUTTON))   robot.deposit.lift.reset();

                // SET HEADING:
                double y = gamepadEx1.getRightY();
                if (hypot(rightX, y) >= 0.8) robot.drivetrain.setCurrentHeading(-atan2(y, rightX) - FORWARD);
                rightX = 0;
                leftX = 0;
                leftY = 0;

    //            if (keyPressed(1, DPAD_UP))         robot.drivetrain.setTargetHeading(0);
    //            else if (keyPressed(1, DPAD_LEFT))  robot.drivetrain.setTargetHeading(PI * 0.5);
    //            else if (keyPressed(1, DPAD_DOWN))  robot.drivetrain.setTargetHeading(PI);
    //            else if (keyPressed(1, DPAD_RIGHT)) robot.drivetrain.setTargetHeading(PI * 1.5);

            } else {

//                robot.intake.setMotorPower(
//                        gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
//                );
//
//                if (keyPressed(1, DPAD_UP))         robot.deposit.goToScoringPosition(true);
//                else if (keyPressed(1, DPAD_LEFT))  robot.deposit.goToScoringPosition(false);
//                else if (keyPressed(1, DPAD_DOWN))  robot.deposit.retract();
//                else if (keyPressed(1, DPAD_RIGHT)) robot.deposit.transfer(Sample.NEUTRAL);
//
//                if (keyPressed(1, X))               robot.intake.toggle();
//                if (keyPressed(1, Y))               robot.deposit.climb();
//                if (keyPressed(1, B))               robot.deposit.handleSample();

            }

            robot.run();

            // Field-centric driving with control stick inputs:

            robot.drivetrain.run(
                    leftX,
                    leftY,
                    rightX,
                    slowModeLocked ||
                                robot.requestingSlowMode() ||
                                gamepadEx1.isDown(RIGHT_BUMPER) ||
                                gamepadEx1.getTrigger(RIGHT_TRIGGER) > 0,
                    useFieldCentric
            );
            robot.printTelemetry();
            mTelemetry.update();
        }
    }
}
