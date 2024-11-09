package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.loopMod;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.pose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    enum TeleOpConfig {
        EDITING_ALLIANCE,
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

        ElapsedTime loopTimer = new ElapsedTime();

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, pose);
//        robot.drivetrain.localizer.trackHeadingOnly(true);

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        TeleOpConfig selection = EDITING_ALLIANCE;

        boolean slowModeLocked = false, useFieldCentric = true;
        // Get gamepad 1 button input and locks slow mode:
        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP))   selection = selection.plus(-1);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) selection = selection.plus(1);

            if (gamepadEx1.wasJustPressed(X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SLOW_LOCK:
                    slowModeLocked = !slowModeLocked;
                    break;
                case EDITING_FIELD_CENTRIC:
                    useFieldCentric = !useFieldCentric;
                    break;
            }

            mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Slow mode " + (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(EDITING_SLOW_LOCK));
            mTelemetry.addLine();
            mTelemetry.addLine((useFieldCentric ? "Field centric" : "ROBOT CENTRIC") + " driving" + selection.markIf(EDITING_FIELD_CENTRIC));

            mTelemetry.update();
        }

//        robot.drivetrain.localizer.setPosition(pose);

//        robot.intake.setAlliance(isRedAlliance);
//        robot.deposit.setAlliance(isRedAlliance);

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

            if (gamepadEx1.isDown(LEFT_BUMPER)) {

//                robot.intake.offsetExtension(
//                        gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
//                );
//
//                robot.deposit.lift.setLiftPower(gamepadEx1.getLeftY());
//                if (gamepadEx1.wasJustPressed(LEFT_STICK_BUTTON)) robot.deposit.lift.reset();
//
//                if (gamepadEx1.wasJustPressed(RIGHT_STICK_BUTTON)) robot.drivetrain.localizer.reset();
//
//                // SET HEADING:
//                double rightY = gamepadEx1.getRightY();
//                if (rightX*rightX + rightY*rightY >= 0.64) {
//                    double radians = -atan2(rightY, rightX);
//                    robot.drivetrain.localizer.setHeading(radians);
//                }

                rightX = 0;
                leftX = 0;
                leftY = 0;

                // if (keyPressed(gamepadEx1, DPAD_UP))         autoTurner.setTargetHeading(0);
                // else if (keyPressed(gamepadEx1, DPAD_LEFT))  autoTurner.setTargetHeading(PI * 0.5);
                // else if (keyPressed(gamepadEx1, DPAD_DOWN))  autoTurner.setTargetHeading(PI);
                // else if (keyPressed(gamepadEx1, DPAD_RIGHT)) autoTurner.setTargetHeading(PI * 1.5);

            } else {

//                robot.intake.setMotorPower(
//                        gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
//                );
//
//                if (gamepadEx1.wasJustPressed(X))               robot.intake.toggle();
//                if (gamepadEx1.wasJustPressed(A))               robot.intake.dumpSample();
//                if (gamepadEx1.wasJustPressed(Y))               robot.climber.climb();
//
//                if (robot.climber.isActive()) {
//
//                    if (gamepadEx1.wasJustPressed(DPAD_DOWN)) robot.climber.cancelClimb();
//
//                } else {
//
//                    if (gamepadEx1.wasJustPressed(DPAD_UP)) robot.deposit.setPosition(HIGH);
//                    else if (gamepadEx1.wasJustPressed(DPAD_LEFT)) robot.deposit.setPosition(LOW);
//                    else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) robot.deposit.setPosition(FLOOR);
//                    else if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.deposit.transfer(NEUTRAL);
//
//                    if (gamepadEx1.wasJustPressed(B))               robot.deposit.triggerClaw();
//                }

            }

            robot.run();

            // Field-centric driving with control stick inputs:

            robot.drivetrain.run(
                    leftX,
                    leftY,
                    rightX,
                    true,
                    false
            );

            mTelemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            divider();
            robot.printTelemetry();
            mTelemetry.update();
        }
    }
}
