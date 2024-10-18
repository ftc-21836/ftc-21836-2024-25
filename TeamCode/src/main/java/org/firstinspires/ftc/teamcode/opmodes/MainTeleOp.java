package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRight;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.loopMod;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_AUTO_SLOW;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    static boolean doAutoSlow = true;

    @Override
    public void runOpMode() throws InterruptedException {

        teleOpInit(this);

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            teleOpControls();

            robot.run();

            mTelemetry.addData("Auto slow is", doAutoSlow ? "enabled" : "disabled");
            mTelemetry.addLine();
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_AUTO_SLOW,
        EDITING_SLOW_LOCK;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            return selections[loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    static void teleOpInit(LinearOpMode opMode) {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        robot = new Robot(opMode.hardwareMap);
        robot.run();

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(opMode.gamepad1);
        gamepadEx2 = new GamepadEx(opMode.gamepad2);

        TeleOpConfig selection = EDITING_ALLIANCE;

        boolean lockSlowMode = false;
        // Get gamepad 1 button input and locks slow mode:
        while (opMode.opModeInInit()) {
            gamepadEx1.readButtons();

            if (keyPressed(1, DPAD_UP))   selection = selection.plus(-1);
            if (keyPressed(1, DPAD_DOWN)) selection = selection.plus(1);

            if (keyPressed(1, X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRed = !isRed;
                    break;
                case EDITING_SIDE:
                    isRight = !isRight;
                    break;
                case EDITING_AUTO_SLOW:
                    doAutoSlow = !doAutoSlow;
                    break;
                case EDITING_SLOW_LOCK:
                default:
                    lockSlowMode = !lockSlowMode;
                    break;
            }

            mTelemetry.addLine((isRed ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine((isRight ? "RIGHT " : "LEFT ") + "side" + selection.markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine("Auto slow is " + (doAutoSlow ? "enabled" : "disabled") + selection.markIf(EDITING_AUTO_SLOW));
            mTelemetry.addLine();
            mTelemetry.addLine("Slow mode is " + (lockSlowMode ? "LOCKED" : "OFF") + selection.markIf(EDITING_SLOW_LOCK));

            mTelemetry.update();
        }

        if (lockSlowMode) robot.drivetrain.lockSlowMode();

        if (autonEndPose == null) autonEndPose = AutonVars.startPose.byBoth().toPose2d();
//        robot.drivetrain.setPoseEstimate(autonEndPose);
//        robot.drivetrain.setCurrentHeading(autonEndPose.getHeading() - (isRed ? FORWARD : BACKWARD));
    }

    static void teleOpControls() {

        if (keyPressed(2, LEFT_BUMPER))   doAutoSlow = !doAutoSlow;

        robot.intake.setMotorPower(
                gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
        );

        double x = gamepadEx1.getRightX();
        boolean overrideMode = gamepadEx1.isDown(LEFT_BUMPER);

        if (overrideMode) {

            robot.deposit.lift.setLiftPower(gamepadEx1.getLeftY());
            if (keyPressed(1, LEFT_STICK_BUTTON))   robot.deposit.lift.reset();

            // SET HEADING:
            double y = gamepadEx1.getRightY();
            if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(-atan2(y, x) - FORWARD);
            x = 0;

//            if (keyPressed(1, DPAD_UP))         robot.drivetrain.setTargetHeading(0);
//            else if (keyPressed(1, DPAD_LEFT))  robot.drivetrain.setTargetHeading(PI * 0.5);
//            else if (keyPressed(1, DPAD_DOWN))  robot.drivetrain.setTargetHeading(PI);
//            else if (keyPressed(1, DPAD_RIGHT)) robot.drivetrain.setTargetHeading(PI * 1.5);

        } else {

            if (keyPressed(1, DPAD_DOWN))       robot.deposit.goToScoringPosition(false);
            else if (keyPressed(1, DPAD_UP))    robot.deposit.goToScoringPosition(true);
            else if (keyPressed(1, DPAD_LEFT))  robot.deposit.handleSample();
            else if (keyPressed(1, DPAD_RIGHT)) robot.deposit.transfer(Intake.Sample.NEUTRAL);

            if (keyPressed(1, A))               robot.deposit.retract();
            if (keyPressed(1, B))               robot.deposit.incrementClimb();

            if (keyPressed(1, X))               robot.intake.toggle();

        }

        // Field-centric driving with control stick inputs:
        boolean driveSlow =
                gamepadEx1.isDown(RIGHT_BUMPER) ||
                (doAutoSlow && (
                        robot.deposit.slowMode() ||
                        gamepadEx1.getTrigger(RIGHT_TRIGGER) > 0
                ));

        robot.run();
        robot.drivetrain.run(
                overrideMode ? 0 : gamepadEx1.getLeftX(),
                overrideMode ? 0 : gamepadEx1.getLeftY(),
                x,
                driveSlow
        );
    }

    static boolean isTranslating() {
        return gamepadEx1.getLeftX() != 0 || gamepadEx1.getLeftY() != 0 || gamepadEx1.getRightX() != 0;
    }
}
