package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.loopMod;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.pose;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.NEUTRAL;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        EDITING_SLOW_LOCK,
        EDITING_FIELD_CENTRIC;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            return selections[(int) loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime loopTimer = new ElapsedTime();

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);
        robot.drivetrain.localizer.trackHeadingOnly(true);

        robot.deposit.preload();

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        TeleOpConfig selection = EDITING_ALLIANCE;

        boolean slowModeLocked = false, useFieldCentric = true;
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
            mTelemetry.addData("Slow mode", (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(EDITING_SLOW_LOCK));
            mTelemetry.addLine();
            mTelemetry.addLine((useFieldCentric ? "Field centric" : "ROBOT CENTRIC") + " driving" + selection.markIf(EDITING_FIELD_CENTRIC));

            mTelemetry.update();
        }

        robot.drivetrain.localizer.setPosition(pose);

        robot.intake.setAlliance(isRedAlliance);
        robot.deposit.setAlliance(isRedAlliance);

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

            double rTrigger = gamepadEx1.getTrigger(RIGHT_TRIGGER);
            double lTrigger = gamepadEx1.getTrigger(LEFT_TRIGGER);

            if (gamepadEx1.isDown(LEFT_BUMPER)) {

                robot.intake.extendo.runManual(rTrigger - lTrigger);
                robot.deposit.lift.runManual(leftY);
                robot.intake.runRoller(0);
                
                if (gamepadEx1.wasJustPressed(LEFT_STICK_BUTTON))   robot.deposit.lift.reset();
                if (gamepadEx1.wasJustPressed(RIGHT_STICK_BUTTON))  robot.drivetrain.localizer.reset();

                // SET HEADING:
                double y = gamepadEx1.getRightY(), x = rightX;
                if (x*x + y*y >= 0.64) robot.drivetrain.localizer.setHeading(-atan2(y, x));

                rightX = leftX = leftY = 0;

            } else {

                robot.intake.extendo.setTarget(lTrigger * Extendo.LENGTH_EXTENDED);
                robot.intake.runRoller(rTrigger);
                robot.deposit.lift.runManual(0);

                if (gamepadEx1.wasJustPressed(X))                   robot.intake.toggle();
                if (gamepadEx1.wasJustPressed(Y))                   robot.climber.climb();

                if (!robot.climber.isActive()) {

                    if (gamepadEx1.wasJustPressed(DPAD_UP))         robot.deposit.setPosition(HIGH);
                    else if (gamepadEx1.wasJustPressed(DPAD_LEFT))  robot.deposit.setPosition(LOW);
                    else if (gamepadEx1.wasJustPressed(DPAD_DOWN))  robot.deposit.setPosition(FLOOR);
                    else if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.deposit.transfer(NEUTRAL);

                    if (gamepadEx1.wasJustPressed(B))               robot.deposit.triggerClaw();

                } else if (gamepadEx1.wasJustPressed(DPAD_DOWN))    robot.climber.cancelClimb();

            }

            robot.run();

            robot.drivetrain.run(
                    leftX,
                    leftY,
                    rightX,
                    slowModeLocked || robot.requestingSlowMode() || gamepadEx1.isDown(RIGHT_BUMPER) || rTrigger > 0,
                    useFieldCentric
            );

            mTelemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            divider();
            robot.printTelemetry();
            mTelemetry.update();
        }
    }
}
